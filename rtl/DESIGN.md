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

> **Building the board bitstream** (ISE 14.7) and the third-party licensing of
> the HDMI glue — in particular the Xilinx **XAPP495** DVI reference design,
> which is git-ignored and must be fetched separately — are documented in
> [`boards/atlys/README.md`](boards/atlys/README.md).

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

### (h) Max-perp-spread + (i) border margin (v2d)

Both are **once-per-segment judge-level rejections** — no per-pixel or labelling
change — so both fold into the existing back-end at essentially zero cost and
stay three-way (SW/HLS/RTL) bit-exact.

- **(h) curve rejection.** A curved arc bows off its chord, inflating the smaller
  eigenvalue of the *normalised* covariance (the perpendicular variance in px²);
  the aspect-ratio test alone misses short low-curvature arcs. SW rejects when
  `ev_min = ½(T−R)/N² > max_perp_spread²` (default 1). This is done sqrt-free and
  **inside the existing 128-bit judge**: with `A := T − 2·mps²·N²`, reject iff
  `A > 0 && A² > R²`, reusing the aspect test's `T` and `R²` (`A²`, `R²` ≈ u118,
  well under 128). In `judge_unit.v` it is two extra products (`N²`, then `A²`)
  time-multiplexed onto the same shared multiplier — a handful more cycles on the
  already-off-critical-path judge. Unlike (c), it does **not** grow the datapath.
- **(i) border margin.** The 2×2 gradient biases the very edge of the frame, so a
  ring of spurious segments traces the border. Defined as a **bounding-box
  rejection**: drop a segment whose bbox reaches within `border` (=3) px of the
  frame — `min_x < b || max_x ≥ w−b || min_y < b || max_y ≥ h−b`, a pure integer
  compare on the record's own extremes, applied at record emission in `backend.v`
  (companion to the (d) strong-count gate). *(The thesis-prose per-pixel "skip
  labelling border pixels" form was tried first but is fundamentally incompatible
  with the RTL's `Interior ⇒ labelled` invariant and its `w_sav`/row-tag carries —
  an unlabelled-but-featured pixel makes a neighbour chase a stale label. The bbox
  form removes the identical frame artifacts — same accepted counts on the whole
  corpus, e.g. IMGP1033 2027, IMGP0942 2936 — and was adopted uniformly in
  SW/HLS/RTL so the three stay bit-exact.)*

The RTL "improved" configuration is therefore **(a) strict NMS + (j) half-pixel
shift + (f) bbox endpoints + (d) adaptive hysteresis + (h) max-perp-spread +
(i) border margin** — all six three-way (SW/HLS/RTL) bit-exact.

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

The RTL "improved" configuration is therefore **(a) strict NMS + (j) half-pixel
shift + (f) bbox endpoints + (d) adaptive hysteresis + (h) max-perp-spread +
(i) border margin** (see the v2d subsection above) — all six three-way
(SW/HLS/RTL) bit-exact. (c) sub-pixel NMS remains a SW/HLS-level refinement.

## Back-end FSM (from `../hls/src/backend.cpp`)

Per event: INGEST → (at EOR) PROCESS_ROW → per interior x: GATHER (≤4 finds,
each a BRAM pointer chase; measured depth ≤ 1 on the corpus) → RESOLVE
(create / adopt / MERGE) → ACCUMULATE → CONTACT (endpoint touch → open or
close a segment → JUDGE) → SCAVENGE at end of row. All state is the phase-1
memory map: 1024-label SoA table, tag-validated label/feature rows, ping-pong
interior lists, touched lists + free-list ring (release at `last_row ≤ y−2`).

**GATHER dispatch is parallel-skip (`backend.v`, throughput opt).** The four
already-labelled neighbours are visited in golden order NE, N, NW, W, but the
FSM does **not** step a counter through all four slots one cycle each — an
absent neighbour would waste a cycle. Instead a combinational present-mask
`gpres = {n_aR,n_aC,n_aL,n_cL}==K_INT` and a priority pick (`grem = gpres &
~pdone`, `pdone` = neighbours already find-launched) jump straight to the next
present neighbour; RESOLVE fires when `grem==0`. The find loop
(`S_GWAIT`/`S_GATH1`), the `label0`/`label1` accumulation and the single-hop
path-compression are **unchanged** — only the dead cycles between finds are
removed, so records stay bit-exact (verified incl. the FullHD gate). Per-pixel
cost drops by `4−k` cycles for `k` present neighbours (largest on the sparse
thin-line pixels that dominate): measured **18.9 → 16.0 cy/interior on
IMGP1033 (×1.18)**, cutting the back-end's share of the 1080p30 frame-clock
budget from ~93 % to ~79 %. This lowers the FIFO backlog on dense rows, so it
helps overflow; the actual measured overflow figures are in **Overflow reality
check** below. (The per-lever FIFO-drop *co-sim* loss numbers this section once
quoted — ~52 % → ~34 % corpus — came from a lump-drain host model later shown to
be ~2× over-pessimistic; see below.) The gather bottleneck was found by a
state-occupancy histogram (42 % of back-end cycles were the serial gi=0..5 walk).

**GATHER W-continuation fast-path (`backend.v`, throughput opt).** Re-running
the state-occupancy histogram *after* the parallel-skip showed GATHER was still
the largest single group (588 k / 1.83 M cy ≈ 32 %): the remaining cost is a
2-cycle floor per pixel (the `gi=0` right-neighbour setup + the `grem==0`
RESOLVE) plus **3 cycles per present neighbour** (dispatch + `S_GWAIT`/`S_GATH1`,
the conn-BRAM `addr@T→data@T+2` find latency), at ~0.94 present neighbours/pixel.
The W (left) neighbour's label is the carry register `w_sav`, and `w_sav` is
provably a **root**: `center` is only ever loaded from a find result / fresh
label / merge survivor (all roots) and `w_sav <= center`; single-hop
path-compression never rewrites `conn[root]`, and no merge runs between the
previous pixel's ACCUMULATE and this GATHER. So `find(w_sav)` always returns
immediately — the FSM folds `w_sav` into `label0`/`label1` directly (exactly as
`S_GATH1`'s root branch, no compression write) and skips the 2-cycle read. Gated
on the run-continuation `prev_x == px−1` so `w_sav` is guaranteed to belong to
`px−1` even in the label-exhaustion skip path (where `prev_x` is left stale).
Measured **−82 k cy** (40,995 W-finds folded × 2 cy): GATHER 4.81 → 4.14
cy/interior; bit-exact (FullHD gate imp 2027 / base 2106, CE_DIV 1 & 2,
tb_sweep_core full chain).

**INGEST is one state, not two (`backend.v`, throughput opt).** The `event_fifo`
is first-word-fall-through (`front = mem[rp]`, combinational), so the front event
is valid the same cycle it is `!empty`. The old `S_POP` (latch) → `S_EV` (apply)
split was pure redundancy; the two are fused into a single `S_POP` that applies
the FWFT front directly. For this to advance one event per cycle, `ev_pop` is
made a **combinational** output (`state==S_POP && !ev_empty`) so the FIFO
advances the same edge the event is applied — a registered pop would re-present
the same front next cycle and double-apply it; the consumer gates the pop with
`en` and the FSM only steps on `en`, so exactly one event is consumed per
en-cycle in `S_POP`. Every event (interior + endpoint + row markers) now costs 1
cycle instead of 2: INGEST **340 k → 170 k cy** (−9.3 % of the back-end).
Bit-exact across the same gate.

**Combined (INGEST fuse + W fast-path): IMGP1033_imp back-end 1.83 M → 1.58 M cy
(−13.8 %), 14.96 → 12.90 cy/interior, 1080p30 frame-budget share ~74 % → ~64 %.**
GATHER is still #1 (506 k, 32 %); FETCH is now #2 (371 k, 23.5 %, the short-path
`addr@T→data@T+2` floor).

**GATHER `gi=0` setup folded into dispatch (`backend.v`, throughput opt).** The
per-pixel GATHER floor was 2 cycles: a `gi=0` state that only *captured* the
right (px+1) column neighbours, then the first dispatch/resolve one cycle later.
The right column's feature banks (`fq_*`) and row-label (`rowq_*`) are still
combinationally valid on entry to `S_GATH0` (the read port is only repurposed for
the next-pixel prefetch *inside* that cycle), so the dispatch/resolve chain now
reads them combinationally and runs in the **same** cycle as the capture: `gi=0`
both latches `n_aR`/`rq_lab_p1` (for the later iterations) *and* dispatches the
highest-priority neighbour. The gather inputs are gi-aware — `naR_eff` /
`rqlab_p1_eff` read the live right-column at `gi=0` and the latched
`n_aR`/`rq_lab_p1` at `gi≥1`; all other neighbours (N/NW/W) are already latched by
`S_RC`. The `naR_eff` NE capture is **inlined** (not `fcap(...)`): a function
call reading the `fq_*` banks *internally* from a continuous assignment has an
incomplete iverilog sensitivity list (it would hold a stale power-up X even after
the banks settle — synthesis/HW are unaffected, but the sim gate is not); the
inlined tag/kind reads make the wire depend on `fq_*` directly, and a tag miss
masks the kind to 0 so an uninitialised-bank X cannot leak. Measured
**−122,215 cy** = exactly one cycle per interior pixel (122,215): GATHER 506 k →
384 k, back-end **1.58 M → 1.456 M cy** (−7.6 %), 12.90 → **11.92 cy/interior**,
1080p30 frame-budget share ~64 % → ~59 %. Bit-exact (FullHD gate imp 2027 / base
2106, CE_DIV 1 & 2, tb_sweep_core full chain). This lever adds combinational
`fcap→conn-address` logic, so it is the one with timing risk — verified in
`synth_be`: 80.32 MHz, 0 failing endpoints, critical path unchanged (still
`u_judge/Mmult_mps_thr`, the (h) DSP), so the 74.25 MHz constraint still closes.

**FETCH `S_RB` bubble removed on adjacent pixels (`backend.v`, throughput opt).**
The 3-cycle short-path fetch is `S_RNEXT` (capture the left column + issue the
centre address) → **`S_RB`** (issue px+1, a pure `addr@T→data@T+2` wait) → `S_RC`
(capture the centre column px). On a run-continuation (`prev_x == px−1`) the centre
column was already read as the PREVIOUS pixel's right column and still sits in
`n_aR`/`n_bR`/`rq_*_p1` (those are overwritten only at this pixel's `S_GATH0`), so
it is **carried** rather than re-read: `S_RNEXT` issues the right column (px+1)
directly and a single wait state (**`S_RC2`**) covers its read latency — 2 cycles
instead of 3. Because `S_RC` is skipped, `pxn`/`pstrongn` (the next interior x) are
captured at `S_GATH0` from the still-valid `xl*_q` (`via_fast` flag). The carry is
bit-identical to a fresh read at column px (same bank/tag/address, and `row_lab[px]`
is untouched between the previous `S_GATH0` and here — the previous pixel writes
`row_lab[px−1]`); the `prev_x==px−1` gate is the same one the W fast-path uses and
is provably false after any label-exhaustion skip (px jumps by ≥2), so the carried
registers always belong to px−1. Measured: FETCH **−114 k cy on IMGP1077** (54.7 %
of pixels adjacent), back-end 2.25 M → **2.137 M cy (10.76 → 10.22 cy/interior,
−5.0 %)**; IMGP1033 (33.5 % adjacent) 1.456 M → 1.416 M (−2.8 %). Bit-exact (FullHD
gate imp 2027 / 3030 / 2617 / 3384 records, small vectors + tb_sweep_core, CE_DIV
1 & 2). Overflow effect (RTL burst TB, see below): corpus loss **4.83 % → 3.99 %**
(−17 % relative), worst frames −3…−9 pt (IMGP1049 38.9→29.7 %, IMGP1077 24.6→18.8 %,
one frame +0.8 pt from benign burst-phase reshuffling). The added `via_fast` mux on
the `f_ra` address is off the FE critical path (re-confirm `synth_be` before a
reflash).

**GATHER terminal-resolve fusion (`backend.v`, throughput opt).** Every pixel
used to end its gather with a dedicated `grem==0` `S_GATH0` pass whose only job
was to look at `label0`/`label1` and pick create/adopt/merge/accumulate — but by
then the outcome is already decided: the W fast-path fold is provably the LAST
gather action (`gselbit==0001` inside `grem!=0` implies W is the only remaining
neighbour), and at the last `S_GATH1` the in-flight find is already excluded
from `grem` (pdone is updated at dispatch). Both terminal sites now compute the
folded `label0'`/`label1'` combinationally and branch straight to
`S_ACC`/`S_ARD`/`S_MRDA` (the create path stays at the `gi==0` resolve — a
terminal fold/find implies a non-zero root, so create is unreachable there).
The old exit pass costs 1 cycle per pixel except the no-neighbour pattern
(whose `gi=0` already was the resolve), so the saving is `pixels − PAT0000`:
measured GATHER **−192,306 cy on IMGP1077** (571,976 → 379,670) and **−112,373
on IMGP1033** (384,035 → 271,662) — both matching the prediction exactly; net
back-end −8.8 % / −7.8 % after ~2–3 k cy of judge-hold elasticity flows back
into CONT/MEXEC. IMGP1077 2.137 M → **1.948 M cy (10.22 → 9.32 cy/interior)**,
frame-budget share ~79 %. Overflow (RTL burst TB, depth 2048): IMGP1077 drops
15,360 → **9,452 (−38 %)**, rows-with-drop 123 → 77. Bit-exact everywhere
(small vectors + tb_sweep_core full chain + FullHD gates imp 2027 / base 2106 /
1077 3030 / 1049 3384 / 1062 1713, CE_DIV 1 & 2). Timing: `synth_be` 80.47 MHz,
0 failing endpoints, critical path unchanged (`u_judge/Mmult_mps_thr`) — the
resolve mux rides the gather cycle without entering the critical path.

**FETCH folded into S_ACC/S_CONT (`backend.v`, throughput opt — the big one).**
In the steady state the fetch state machine (S_RNEXT + S_RC2 on adjacent
pixels, S_RNEXT + S_RB + S_RC on run starts) did no computation at all: it
issued the three feature/row-buffer column addresses and burned the
`addr@T→data@T+2` BRAM latency as explicit wait states. Those issues now ride
the edges of states the pixel already pays for: `gi=0` keeps issuing the
successor's LEFT column (`xl_q − 1`, now universal — `pxn`/`pstrongn` are
always captured at `gi=0` from the still-valid `xl*_q`); `S_ACC` issues the
successor's remaining column (right = pxn+1 for an ADJACENT successor, whose
left+centre come from carries; centre = pxn for a NON-adjacent one) and stages
the list read for the pixel after it (`hn2`); the first `S_CONT` cycle
(`cfirst` flag) shifts the neighbour window — adjacent: left ⇐ this pixel's
centre (row-y kind is K_INT by construction, NW ⇐ the pre-overwrite `nw_sav`
carry), non-adjacent: left read fresh from fq/rowq, which deliver column pxn−1
during that cycle for ANY resolve length (f_ra holds pxn−1 from `gi=0` through
the `S_ACC` edge, so even the 1-cycle fold-direct path is covered); and the
`S_CONT` exit branches straight to the successor's `S_GATH0` (adjacent) or
through the new 1-cycle **`S_RX`** wait (non-adjacent — its centre column,
addressed at `S_ACC`, is valid there; its right column, addressed at the exit,
lands at `S_GATH0`). Two hazards and their fixes: (i) `touches_end` reads the
window regs, which the shift overwrites, and judge-hold cycles re-evaluate it —
so `S_ACC` registers it (**`te_r`**) on the old window; (ii) all issue points
are hold-immune because f_ra/row_ra stay put through any judge-hold and the
one-shot captures fire on the `cfirst` cycle only. The end-of-row branch (the
scavenger setup that lived in S_RNEXT) moves to the `S_CONT` exit; S_RNEXT /
S_RB / S_RC / S_RC2 and the 7-cycle row-start path are kept verbatim as the
row-entry and label-exhaustion fallback (the fallback re-entry re-establishes
every steady-state invariant, so the flows compose). Fetch cost per pixel:
adjacent 2 → **0** cycles, non-adjacent 3 → **1** (the S_RX wait is the true
port floor: three columns through one read port at 2-cycle latency needs one
un-hidden slot). Measured, matching the port-schedule prediction exactly:
IMGP1077 FETCH 517,194 → **100,127 cy**, back-end 1.948 M → **1.543 M cy (9.32
→ 7.38 cy/interior)**, frame-budget share 79 % → **62 %**; IMGP1033 FETCH
329,949 → 86,592, back-end 1.305 M → 1.067 M (8.73 cy/interior); IMGP1049
2.087 M → 1.745 M; IMGP1062 1.374 M → 1.091 M. Session cumulative (terminal
resolve + fetch fold): IMGP1077 2.137 M → 1.543 M = **−27.8 %**. **Overflow
(RTL burst TB, depth 2048): IMGP1077 — the worst frame of the corpus, 15,360
dropped events at the session start — now drops NOTHING (peak occupancy
530/2048, all 3030 records emitted).** Bit-exact everywhere (small vectors +
tb_sweep_core full chain + FullHD gates imp 2027 / base 2106 / 1077 3030 /
1049 3384 / 1062 1713, CE_DIV 1 & 2). Timing: `synth_be` 81.34 MHz, 0 failing
endpoints, critical path unchanged (judge DSP).

**Concurrent event ingest (`backend.v`, throughput opt — decouples supply from
drain).** The FSM used to ingest serially: all of row y+1's events popped
back-to-back in S_POP (1 cycle each) *between* row labellings, so the FIFO
backed up during a dense row's labelling AND the labeller paid 1 cy/event
afterwards. Ingestion now runs as a parallel engine in the same clocked
process: whenever an event is available it is filed into the feature banks /
interior lists at 1 event/cycle, concurrently with labelling an earlier row.
Structure: feature banks go 3 → **4** (row mod 4 — the labeller reads rows
py−1/py/py+1 while the engine writes py+2) and the interior-x lists go
parity → **3 banks** (row mod 3: one being read, one complete, one being
written); the S_POP state degenerates to a row-handshake (label row proc_y
once `ingest_y ≥ proc_y+2`, or EOF). Flow control: the engine pops only while
`ingest_y ≤ proc_y+2`, which keeps every written bank disjoint from every
read bank, and each array keeps exactly ONE textual writer (XST
single-write-port rule). Effects on IMGP1077: ingest cycles 277,257 →
**1,562** (startup sync only); back-end 1.543 M → **1.267 M cy (6.06
cy/interior)**, frame-budget share 62 % → **51 %**; IMGP1033 1.067 M →
0.899 M. Session cumulative (terminal resolve + fetch fold + concurrent
ingest): IMGP1077 **2.137 M → 1.267 M = −40.7 %**. The burst behaviour
changes qualitatively: events drain into the banks at line rate while the
labeller works, so the FIFO only backs up when the engine is flow-blocked
(labeller > 3 rows behind) — IMGP1077's FIFO peak occupancy drops to **1**
(!), IMGP1007 (the densest frame) to 169 with zero drops. The single
remaining dropper is IMGP1032 (a long ~18-row adopt-heavy dense band): its
true backlog measured 5,975 events (drop-proof FIFO run), i.e. a 8192-deep
event FIFO would make the whole corpus zero-drop. Bit-exact everywhere
(small vectors + tb_sweep_core full chain + FullHD gates imp 2027 / base
2106 / 1077 3030 / 1049 3384 / 1062 1713, CE_DIV 1 & 2). Timing: `synth_be`
81.67 MHz, 0 failing endpoints, critical path unchanged (judge DSP); backend
BRAM after the extra banks: 36 RAMB16 + 16 RAMB8 (of 116/232).

**Judge = one shared sequential MAC.** The exact integer test
`361·T² ≤ 441·R²` needs 9 wide products. Rather than the 79
DSPs the HLS version spends (Artix-7 87 %), phase 2 schedules every product
onto **one 36×36 pipelined multiplier (4 DSP48A1)** decomposed into 18×18
partial products — ~10³ segments/frame ≪ the 2.5 M cycle frame budget. Total
DSP48A1 estimate: judge 4 + accumulate (x², x·y, 11×11) ~3 ≈ **7 of 58** —
comfortable on the LX45.

**Judge datapath sized to the empirical worst case, not the field cap.** The
per-label moment fields can hold n up to 2^18, which would make the products
128-bit; but `hls/tools/moment_probe` measured the LARGEST moments the judge
actually sees over the Waseda corpus — n<2^12, Sx/Sy<2^22, Sxx/Syy/Sxy<2^33
(edges are ≤2 px wide, so a component is a thin curve: n ≈ 2·W). Hence
ma/mc/mb<2^45, T<2^46, T²<2^92, and 361·T² / 441·R² < 2^102. `judge_unit.v` is
narrowed to that (operands 60→48 bit, accumulator/compare 128→104 bit) — **exact,
no rounding**: the removed high bits are provably zero for any input within those
bounds (a pathological blob beyond them is rejected on aspect anyway; the fields
could add a saturate for full safety). With the narrower operands most base
products have a zero high half (n and Sx/Sy fit the low 30 bits), so the
multiplier **skips the hi·lo / lo·hi / hi·hi passes when a half is zero** — a base
product costs 1–2 passes instead of 4. Bit-exact (tb_judge 303 cases, small
vectors, FullHD gate IMGP1033 base 2106 / imp 2027, CE_DIV 1&2); it trims the
judge's cycle count and hence the back-end's CONT wait-for-judge stall
(IMGP1033 FullHD back-end 1.95M → 1.83M cycles, on top of the parallel-skip
gather; frame-budget share ~79 % → ~74 %). SW/HLS are unchanged — being exact,
the narrowed RTL still matches the 128-bit golden.

Per-pixel moment accumulate stays combinational-narrow (11-bit squares); all
adders ≤ 41 bits.

**Label-table Σx²/Σy²/Σxy stored in 34 bits (BRAM saving).** The same
`moment_probe` measurement bounds the per-label second/cross moments at ≤2^32
over the corpus, so `t_xss/t_yss/t_xys` are declared u34 (4× headroom) instead
of u41. Measured by XST (xc6slx45-3, `synth_be`): the narrowing drops the
back-end from **42 → 40 block-RAM sites (RAMB8BWER 15 → 12; RAMB16 unchanged)**
— **−2 of 116 18-Kb sites, ~36 KiB** freed toward a second labelling engine (2-stripe).
Only the storage changes: the datapath regs (`q_/c_/a_/s_/f_/j_`, `rec_*`) stay
u41, so reads zero-extend 34→41 and the accumulate write truncates 41→34
losslessly (bounded ≤2^32 < 2^34). Bit-exact (FullHD gate IMGP1033 imp 2027 /
base 2106, CE_DIV 1&2, small vectors). Σx/Σy (u30) and n (u18) are left as-is:
narrowing them to u22/u12 crosses no 18-bit BRAM boundary, so it would trim only
register area, not block count.

**Overflow reality check (RTL ground-truth burst sim).** The per-lever FIFO-drop
loss figures elsewhere in this doc came from a *host* co-sim
(`hls/tools/fifo_dropsim.cpp`) that modelled the back-end drain as a **lump** — a
whole row's PROCESS cost charged when the EOR marker is popped, stalling the pop
for thousands of cycles. The real FSM **interleaves**: it processes each interior
event right after popping it, so it drains continuously and never stalls a whole
row at once. That difference makes the lump model manufacture multi-row pile-ups
the hardware never has. A cycle-accurate RTL burst testbench measured the truth:

- `rtl/tb/tb_backend_burst.v` = the real `backend.v` + real `event_fifo.v` in
  `drop_mode=1`, fed the golden event stream at **1080p30 pixel timing** (active
  1 px/clk at the event's column, EOR, then `hblank` idle clocks). Validated: with
  an oversized (drop-proof) FIFO it reproduces the **exact golden record count**,
  so the producer/back-end/plumbing are faithful; its peak *backlog* on the
  densest frame is ~25.7 k events (12.5× the 2048-deep FIFO) — overflow is a
  **burst** limit, not an average-throughput one (the back-end fits the frame
  budget on average).
- Over all **150 Waseda frames**: **corpus segment loss 4.83 %** (10,731 /
  222,263) on the `gi=0`-fold back-end, **3.99 %** (8,879) after the `S_RB`-bubble
  removal above; **26–27/150 frames drop anything, ~123 are lossless**, worst-case
  **44–47 %** (IMGP1010) — not the ~79–85 % the lump model predicted for the
  densest frames.
- 3-way corpus: lump co-sim **9.2 %** (≈2× high), interleaved-drain co-sim
  **1.3–2.4 %** (low — a *constant* per-interior cost cannot match the per-image
  cy/interior spread, e.g. two similar-density frames lose 9.8 % vs 47 %), **RTL
  ground-truth 4.83 %**. `fifo_dropsim` was corrected to the interleaved model
  (`--lump` kept for comparison) but is only a fast estimate; the **RTL burst TB
  is the canonical overflow tool**.

So single-engine overflow on the LX45 is a **minor, dense-frame-localised** effect
(~15 frames), not the corpus-wide problem the co-sim implied — the earlier "needs
stripe parallelism for zero-drop" framing was driven by the over-pessimistic
model. The `S_RB`-bubble removal (above) took the corpus from 4.83 % to 3.99 %.

**After the terminal-resolve fusion + the S_ACC/S_CONT fetch fold (above), the
RTL burst sim over all 150 frames reads: corpus segment loss 0.215 % (477 /
222,263), 148/150 frames completely lossless.** Only IMGP1032 (15.5 %, 5,592
dropped events) and IMGP1007 (the densest frame of the corpus — 2.1 %) still
touch the 2040 afull threshold; the third-highest FIFO peak in the corpus is
972 (< half the depth). The per-row burst-band analysis that motivated the two
levers (drain cy/row vs the 2200-cycle row budget, backlog integral validated
against a drop-proof-FIFO run to 1.6 %) predicted the densest frame's backlog
would shrink from ~19 k events to under the FIFO depth — confirmed: IMGP1077,
the worst live-demo frame, now drops nothing (peak 530/2048).

**With the concurrent-ingest engine (above) the corpus reads 0.166 % at the
2048 FIFO — a single lossy frame (IMGP1032, 368 segments; every other frame
now peaks at ≤ 346 FIFO entries, 6× under the depth) — and ZERO with the
8192-deep FIFO now instantiated in `sweep_core`**: IMGP1032's true backlog is
5,975 events (drop-proof-FIFO run), directly verified zero-drop at 8192 with
all 2,617 records emitted, and every no-drop trajectory is depth-invariant, so
the whole 150-frame corpus is provably lossless. Journey of the corpus overflow
loss: live config 52 % → gather/judge/L3 4.83 % → RB fold 3.99 % → terminal
resolve + fetch fold 0.215 % → concurrent ingest 0.166 % → **8192 FIFO 0 %.**

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
