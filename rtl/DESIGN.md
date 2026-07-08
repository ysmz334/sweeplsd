# SweepLSD RTL — phase-2 design (Digilent Atlys / Spartan-6 LX45)

Hand-written Verilog port of the phase-1 HLS core (`../hls/`), targeting the
2014-era hardware the thesis had in mind: a **live HDMI in → detect → overlay
→ HDMI out** demo on the Digilent Atlys (XC6SLX45), with **no frame buffer and
no external memory** — the detector state is the thesis's ~70 KiB of BRAM.

The architecture is the one already validated end-to-end in phase 1 (bit-exact
C-sim over the full Waseda corpus + C/RTL co-sim): II=1 line-buffered
front-end → sparse event FIFO → elastic labelling back-end → segment records.
This phase re-expresses it in portable Verilog because Spartan-6 is not a
Vitis HLS target (ISE 14.7 only).

## Verification strategy (same standard as phase 1)

Bit-exactness against `sweeplsd::detect()` stays the acceptance criterion:

- `tools/dump_vectors.cpp` (host, links the sweeplsd library) renders the
  phase-1 test images and dumps each stage boundary as hex vector files:
  gray in, gaussian, gradient(power,dir), edge, feature, event stream,
  accepted segment records.
- `tb/*.v` (Icarus Verilog, OSS CAD Suite — no ISE needed for the inner loop)
  drive each module and the full core from those vectors and fail on the first
  mismatching sample. ISE is only needed for the board build (task #10).

## Clocking (Atlys)

Single processing clock = the **recovered HDMI RX pixel clock** (74.25 MHz for
both demo modes), so the whole path RX → core → overlay → TX is synchronous
and frame-buffer-free. Serdes clocks (5×, DDR) come from PLL_ADV + BUFPLL as
in the standard Spartan-6 TMDS designs.

| mode | pixel clk | TMDS bit rate | note |
|---|---|---|---|
| 720p60 (default) | 74.25 MHz | 742.5 Mbps | most compatible source mode |
| 1080p30 (option) | 74.25 MHz | 742.5 Mbps | thesis evaluation format (1920×1080) |

1080p60 (1.485 Gbps) exceeds Spartan-6 serdes capability — out of scope.
Core margin: the same datapath met 100 MHz on Artix-7; 74.25 MHz on Spartan-6
(-2/-3) is the comparable envelope, with the judge unit sequential anyway.

**v2a (implemented)**: on Spartan-6 the lockstep front-end's combinational
chain (gauss BRAM read → vertical → horizontal → gradient → NMS → edge BRAM
write) measures ~25 ns — fine at VGA's 40 ns, not at 720p's 13.3 ns. Rather
than pipelining the stages (which would shift every stage's column-lag
constants and the row-boundary wrap-around), the core runs at HALF the pixel
rate; a detector pass over 720p takes ~25 ms => segment updates every 2
display frames (~30 Hz), display always 60 Hz. Two half-rate mechanisms
exist:

- The portable core takes a global clock enable `en` (sweep_core.v) gating
  every sequential element; en = 1 is full rate and any 1-in-N duty is an
  exact time dilation. Bit-exactness is regression-tested with CE_DIV=2 runs
  of tb_sweep_core / tb_core_chain / tb_backend / tb_judge. Stage `o_valid`s
  and the walker's px_addr prediction are deliberately position-only (not
  en-gated) so the every-cycle enable never enters the long datapaths.
- The Atlys board build, however, drives the core from a REAL half-rate
  clock (PLL CLKOUT2 = 37.5 MHz, phase-aligned 2:1 with the 75 MHz pixel
  clock; en tied 1). The clock-enable + FROM:TO×2 multicycle-constraint
  variant was tried first and works in simulation, but ISE's ngdbuild
  silently dropped part of the INST-wildcard TNM group (gauss/gradient
  line-buffer BRAMs, ~300 backend FFs), leaving the front-end chain
  constrained at the pixel period — a real clock cannot be mis-grouped, and
  the auto-derived 26.6 ns period covers every core path. The pix<->core
  handshakes (sof stretch + edge detect in, record strobe out) are
  synchronous 2:1 paths, fully covered by static timing.

Internal-scene v2a uses 75.0 MHz from the on-board 100 MHz oscillator
(1650×750 total => 60.6 Hz, +1 % over CEA — displays accept it; exact
74.25 MHz arrives with the HDMI RX clock in v2b).

## Module hierarchy

```
boards/atlys/atlys_top.v      — pins, PLLs, resets
  hdmi_rx.v                   — ISERDES2 + TMDS decode → pix clk, rgb, de/hs/vs  (from Atlys reference designs)
  rgb2gray.v                  — BT.601 integer luma: (77R + 150G + 29B) >> 8
  core/sweep_core.v           — PORTABLE (no vendor primitives):
    gauss_v.v  gauss_h.v      — 5-tap separable, {16,64,96,64,16}, >>10
    gradient.v                — 2x2, power + H/V dir
    edge_nms.v                — threshold + NMS (3x3 window on power)
    feature.v                 — 5x5 endpoint-candidate (endpoint_core.v = pure LUT logic)
    event_pack.v              — dense feature -> sparse events (+EOR/EOF)
    event_fifo.v              — 2048-deep elastic buffer (backpressures front-end)
    backend.v                 — labelling FSM (below)
  seg_db.v                    — per-frame segment store (ping-pong), feeds overlay
  overlay.v                   — half-res 1-bit mask, ping-pong BRAM + Bresenham drawer
  hdmi_tx.v                   — TMDS encode + OSERDES2                            (from Atlys reference designs)
```

`core/` is simulation-portable Verilog-2001 (inferred BRAM only); everything
vendor-specific (serdes, PLL, TMDS) lives under `boards/atlys/`.

Front-end modules translate 1:1 from `../hls/src/frontend.cpp` — same loop
structure ((h+2)×(w+2) walk with zero-padded borders), same line-buffer
counts, same bit widths (u16 vertical sums, u14 gaussian, u15 power). Each is
a free-running II=1 pipeline with a `valid` handshake and a `stall` input from
the FIFO.

### (d) Adaptive hysteresis (v2c)

`core/hyst_hist.v` (instantiated in `stage_edge`) is the RTL twin of
`kernels::AdaptiveLowTh`: a decayed 64-bin power histogram whose low threshold
is ~2× the 80th-percentile power (integer only: counts ×256, decay `v-=v>>8`,
percentile via `cum*5 >= total*4`). The NMS uses this LOW threshold; a per-pixel
`strong` bit (`power >= power_th`, the HIGH threshold) rides the edge → feature
→ event path (feature carries it on a 2-buffer copy of the window-centre path),
and the back-end keeps a per-label `strong_cnt`, rejecting a segment in the
judge if `strong_cnt < hyst_strong_min` (all criteria are ANDed, so the gate is
applied at record emission).

Two hardware-shaped choices, mirrored back into SW/HLS so all three stay
bit-exact: **(1) two-row lag** — row *m*'s threshold is `lowTh(H_{m-2})`; the
scan runs over a snapshot taken at each row start, giving it a full row (needs
**width ≥ 64**; narrow test images fall back to the fixed low threshold) instead
of the ~10-cycle inter-row gap. **(2) per-frame clear** — the histogram is
global state that cannot self-clear through the flush rows (unlike the line
buffers), so `frame_start` clears it; each frame cold-starts exactly like a
fresh `detect()`. Skipping this only diverges from the golden on the SECOND
frame of a real photo — the FullHD 2-frame regression is what catches it.

### (c) Sub-pixel NMS — SW/HLS reference only (out of RTL scope)

Improvement (c) fits a parabola through the three NMS-axis power samples of each
surviving edge pixel and shifts that pixel's moment contribution by the vertex
offset (±0.5 px, in 1/16-px units). It lives in `sweeplsd::detect()` and the HLS
C model but is deliberately NOT ported to this board RTL, for two reasons:

- **Invisible on the demo.** The overlay draws endpoints at half (720p,
  `(c+1)>>1`) or quarter (1080p, `(c+2)>>2`) resolution with an integer Bresenham
  walker. Measured on the FullHD Waseda photo IMGP0942 (strict + adaptive
  hysteresis + bbox + half-shift), enabling (c) moves endpoints by a mean of
  **0.064 px** (max 0.43 px) and changes the accepted count by **+1** (3098 →
  3099) — a shift that always quantises to the same mask cell, so nothing on the
  screen changes.
- **It breaks the judge's 128-bit envelope.** The exact integer test
  `kRejN·T² ≤ kRejD·R²` is calibrated so both sides fit 128 bits (~u127 worst
  case). Sub-pixel accumulation scales every position by 16 (`fx = 16x + dx`),
  growing the second moments +8 bits and the judge products T²/R² to ~u143 —
  past 128 bits for ANY position scaling (even ×2 overflows). Supporting it would
  force the shared sequential multiplier to ~160-bit arithmetic (more passes; the
  g++ csim would need a software 256-bit integer since `__int128` is the host
  maximum), plus a wider event word (delta+dir) and wider moment/record fields —
  a large datapath change for a sub-visible gain.

The RTL "improved" configuration is therefore FINAL at **(a) strict NMS + (j)
half-pixel shift + (f) bbox endpoints + (d) adaptive hysteresis** — all four
three-way (SW/HLS/RTL) bit-exact. (c) remains a SW/HLS-level refinement.

## Back-end FSM (from `../hls/src/backend.cpp`)

Per event: INGEST → (at EOR) PROCESS_ROW → per interior x: GATHER (≤4 finds,
each a BRAM pointer chase; measured depth ≤ 1 on the corpus) → RESOLVE
(create / adopt / MERGE) → ACCUMULATE → CONTACT (endpoint touch → open or
close a segment → JUDGE) → SCAVENGE at end of row. All state is the phase-1
memory map: 1024-label SoA table, tag-validated label/feature rows, ping-pong
interior lists, touched lists + free-list ring (release at `last_row ≤ y−2`).

**Judge = one shared sequential MAC.** The exact integer test
`361·T² ≤ 441·R²` needs 9 wide products (up to 128×128). Rather than the 79
DSPs the HLS version spends (Artix-7 87 %), phase 2 schedules every product
onto **one 36×36 pipelined multiplier (4 DSP48A1)** decomposed into 18×18
partial products — ~50-80 cycles per segment, ~10³ segments/frame ≪ the 2.5 M
cycle frame budget. Total DSP48A1 estimate: judge 4 + accumulate (x², x·y,
11×11) ~3 ≈ **7 of 58** — comfortable on the LX45.

Per-pixel moment accumulate stays combinational-narrow (11-bit squares); all
adders ≤ 41 bits.

## Overlay (demo path, outside the verified core)

Segments finalised during frame N are drawn into a **half-resolution 1-bit
mask** (720p: 640×360 = 28.8 KiB; ×2 ping-pong = 57.6 KiB BRAM) by a Bresenham
FSM using the raw integer endpoints from the record (the sub-pixel projection
is cosmetic at demo scale); frame N+1 is displayed with mask N mixed in
(green). Endpoints map to mask cells as `(c+1)>>1` (`(c+2)>>2` at the FullHD
/4 scale) — the cell nearest the TRUE edge position `c+0.5`, since the 2×2
gradient samples at pixel corners (improvement (j), v2c; clamped at the
right/bottom edge). Total BRAM: core ~70 KiB + overlay ~58 KiB + video I/O ≈ 140 KiB of
261 KiB — still no external memory. (Fallback if timing/BRAM pinch: draw
endpoints only, or use the DDR2 for the mask — the *detector* remains
memory-free either way.)

## Board bring-up plan (task #10)

1. Start from a known-good Atlys HDMI pass-through (Digilent demo /
   Hamsterworks Atlys HDMI projects) under ISE 14.7; verify RX→TX untouched.
2. Insert rgb2gray + sweep_core tap (records to ChipScope/UART first).
3. Add seg_db + overlay; then tune.

Programming via Digilent Adept 2 (`djtgcfg prog`).

## Repository layout

```
rtl/
  DESIGN.md          — this file
  core/              — portable detector RTL (the deliverable)
  tb/                — Icarus Verilog testbenches (golden-vector parity)
  tools/             — dump_vectors.cpp (golden vector generator)
  boards/atlys/      — top level, HDMI PHY, UCF constraints, ISE build script
```
