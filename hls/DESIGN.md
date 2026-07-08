# SweepLSD HLS — architecture design

Hardware (FPGA) implementation of SweepLSD via high-level synthesis. This realises the
form the 2014 thesis left as future work: the algorithm was designed for a line-buffer
streaming pipeline from the start (integer-only per-pixel core, O(width) memory, 7-row
labelling lag), so the HLS port is a *translation*, not a redesign.

**Golden model**: `detectOnePass()` in `../src/sweeplsd_onepass.cpp`. Every hardware
stage must match the software intermediates bit-exactly (the same standard the software
one-pass == multi-pass parity test already enforces).

## Targets

| Phase | Tool | Device | Deliverable |
|---|---|---|---|
| 1 (this) | Vitis HLS | Artix-7 (`xc7a35t` class) | C-sim parity + synthesis/co-sim reports, II=1 front-end |
| 2 | ISE 14.7, hand Verilog | Spartan-6 LX45 (Digilent Atlys) | HDMI in → overlay → HDMI out live demo |

Phase 1's architecture decisions (fixed-point formats, label recycling, event-driven
labelling) are made Spartan-6-compatible on purpose so phase 2 is a re-expression in
Verilog, not another redesign. Note Vitis HLS cannot target Spartan-6 (7-series+ only);
that is why phase 2 is hand RTL.

- Throughput goal: **1 pixel/cycle (II=1)** in the pixel-rate front-end.
  FullHD\@30fps = 62.7 Mpx/s ⇒ met at 100 MHz with headroom (720p60 = 74.25 MHz
  pixel clock for the Atlys demo).
- Image size: width **compile-time max 1920** (line-buffer depth), runtime `width ≤ 1920`,
  `height` free-running (row counter).
- Algorithm scope: **baseline first** (`Params{}` thesis behaviour), improved()'s
  integer-friendly additions staged in later (see "Improved-mode notes").

## Pipeline structure

Two clock-decoupled halves connected by a FIFO — this is the load-bearing decision:

```
pixel-rate front-end (II=1, rigid)          event-rate back-end (elastic)
┌─────────┐ ┌─────────┐ ┌───────┐ ┌───────┐   FIFO   ┌──────────┐ ┌───────┐
│gaussian │→│gradient │→│ edge  │→│feature│ ═══════> │ labeling │→│ judge │→ segment
│5x5 sep. │ │2x2 |dx|+│ │th+NMS │ │5x5 ring│  events  │union-find│ │int PCA│  records
└─────────┘ └─────────┘ └───────┘ └───────┘          └──────────┘ └───────┘
```

The front-end is a classic line-buffer + window dataflow: every stage advances one
pixel per cycle, unconditionally. The back-end is **event-driven**: the front-end emits
a record only for feature pixels (`Interior`/`Endpoint`) plus one end-of-row marker per
row. Edge density is 1–5 % on real images, so the labelling engine may spend several
cycles per event (union-find chase, merge read-modify-write) and still track the pixel
clock with a small FIFO. This removes the only part of the algorithm that resists II=1
from the II=1 domain entirely.

Worst-case input (pathological dense edge map) throttles via FIFO backpressure and
stalls the front-end; the FIFO is sized for ~1 row of events (2^11). Real images never
approach this (measure in co-sim; report the high-water mark).

### Stage lag budget (same as software)

Input row `y` arriving ⇒ gaussian finalises row `y−2`, gradient `y−3`, edge `y−4`,
feature `y−6`, labelling `y−7`. After the last row, 7 flush rows of zeros sweep the
pipeline out (software does the identical flush).

## Front-end stages (all integer, all II=1)

Bit widths are exact bounds carried over from the software comments:

| signal | width | bound | note |
|---|---|---|---|
| src pixel | u8 | 255 | |
| gauss vertical sum | u16 | 65 280 | `16·(r0+r4)+64·(r1+r3)+96·r2` |
| gauss out | u14 | 16 320 | horizontal weights then single `>>10` |
| gradient power | u15 | 32 640 | `(|dx|+|dy|+1)/2`, dx/dy from 2×2 on gauss |
| dir | u1 | | 1 = vertical (\|dx\| dominant) |
| edge | u1 | | `power ≥ th` ∧ NMS survivor |
| feature | u2 | | 0 none / 1 interior / 2 endpoint |

1. **Gaussian** (§3.2.1.1): separable `{16,64,96,64,16}`, vertical-then-horizontal,
   single `>>10` in the horizontal pass — the exact software formulation, which is
   bit-identical to direct 5×5 `>>10`. Needs 4 line buffers of u8 (7.5 KiB).
   Border rows/cols emit 0 exactly as `gaussianHorizontalRow` does (rows `<2`, `≥h−2`,
   cols `<2`, `≥w−2`).
2. **Gradient** (§3.2.1.2–4): 2×2 window on gaussian rows `r, r+1` ⇒ needs 1 line
   buffer of u14. Bottom edge: row `r+1` outside the image reads as zeros; right edge:
   column `x+1` reads 0 (the software's explicit last-column case).
3. **Edge = threshold + NMS** (§3.2.1.5–7): window of 3 power rows × 3 columns plus the
   centre-row dir bit ⇒ 2 power line buffers + 1 dir line buffer. Competitor pair by
   dir class (H: up/down, V: left/right), survivor iff `c ≥ th ∧ c ≥ Pm ∧ c ≥ Pp`
   (baseline: non-strict ties, `s = 0`). Pure comparators.
4. **Feature / endpoint candidates** (§3.2.2): 5×5 window of edge bits ⇒ 4 one-bit
   line buffers (960 B). `endpointCore` is already written as branch-free boolean
   algebra over 24 window bits — it synthesises to a few LUT levels verbatim. Out-of-
   image window taps read 0 (matches the software's `border`/zero-row handling).

Front-end line-buffer total ≈ **21 KiB** — BRAM, dual-port, one read + one write per
cycle each.

### Event record (front-end → FIFO)

```
{ kind: u2 (interior / endpoint / end-of-row / end-of-frame),
  x: u11 }                                   // y is implicit: back-end row counter
```

Baseline needs nothing else. (Improved mode appends `power: u15, dir: u1, delta: i5`
for hysteresis + sub-pixel moments — reserved, see below.)

Endpoint-candidate pixels must be in the stream even though they are not labelled:
the labelling neighbourhood test reads the *feature values* of all 8 neighbours (for
`touches_end`) and the causal 4 for label gathering. The back-end therefore
reconstructs 3 feature rows (`above`/`cur`/`below`) from the event stream into 2-bit
row buffers — cheaper than shipping the dense rows, and it keeps the FIFO sparse.
Consequence: the back-end labels row `y` only after row `y+1`'s events have arrived
(the software has the same structure: `processRow(r)` runs at input lag 7 with
feature rows `r−1, r, r+1`).

## Back-end: labelling + judgment (§3.2.3–4)

State (the thesis's §3.4 memory budget, realised — as implemented in
`src/backend.cpp`; buffers are **row-tag validated** instead of cleared, since
an O(width) per-row clear would drag the engine back to pixel rate):

| store | size | contents |
|---|---|---|
| label row ×1 | 2048 × (10+11) b ≈ 5.3 KiB | one tag-validated row serves as prev (tag `y−1`) and cur (tag `y`) |
| feature rows ×3 | 3 × 2048 × (2+11) b ≈ 9.8 KiB | rebuilt from events, tag-validated |
| interior-x lists ×2 | 2 × 2048 × 11 b ≈ 5.5 KiB | ping-pong: row `y` processed while `y+1` ingests |
| label table (hot) | 1024 × ~215 b ≈ 27 KiB | see below |
| label table (cold) | 1024 × 23 b ≈ 3 KiB | `has_start, start_x, start_y` |
| free list ring | 1024 × 10 b ≈ 1.3 KiB | recycled label ids |
| touched lists ×3 | 3 × 1024 × 10 b ≈ 3.8 KiB | scavenger input (labels first-touched per row) |
| segment output | streamed out | no on-chip segment DB needed (AXIS out) |

Measured over the full Waseda corpus (all 150 FullHD photos) + dense-noise
stress images (tb stats): high-water **380 live labels** of 1023, find-chain
depth **1** (the original's single-level connect was empirically sufficient —
our full union-find matches the golden model exactly anyway), zero free-list
underflows. All 160 parity runs bit-exact end to end.

Hot entry (baseline, integer exact — widths sized for the worst case `N ≤ 2^18`,
`x < 2^11`, `y < 2^11`):

```
connect:u10, last_row:u11, latest_x:u11, pix_num:u18,
Sx:u29, Sy:u29, Sxx:u40, Syy:u40, Sxy:u40
```

(Software accumulates these in `double`; integers ≤ 2^53 are exact there, so the
integer hardware matches bit-for-bit. `w_sum == pix_num` when unweighted — dropped.)

Per interior-pixel event, the engine:

1. Reads the 4 causal neighbour labels (NE, N, NW from `prev_row`; W from `cur_row`),
   gathers ≤ 2 distinct roots via **find** (pointer chase through `connect`;
   variable latency, amortised ~1 — chains are created by merges and collapsed by
   path compression on the way).
2. `create` (pop free list) / adopt / **merge** (moment add, survivor = greater
   `(last_row, latest_x)` — same rule as software so parity holds; loser's `connect`
   points at survivor; a merge of two started halves closes a segment → judge).
3. Accumulates moments into the root, writes `cur_row[x]`.
4. `touches_end` (any of 8 neighbour features == Endpoint): first contact records
   `start`, second closes the segment → judge.

Estimated 2–6 cycles/event; with ≤ 5 % event density that is ≤ 0.3 cycles/pixel
average — the FIFO absorbs bursts.

### Label recycling

Software grows the table without bound; hardware cannot. The thesis budget is 960
concurrent labels; we use **1024 + free-list ring** (the scheme of the original
`ref/main.cpp`, which recycles by comparing a per-label row-parity bit and
`latest_x` against the scan wavefront). Our rule is that scheme made one row more
conservative, as full union-find path compression (the golden semantics) requires:
**free every label whose `last_row ≤ y−2`**. Safety argument: a label is reachable
only (a) raw from a row cell — cells written in row `t` hold ids whose
`last_row ≥ t` (the write co-occurs with an accumulate) and only rows `y−1, y` are
live — or (b) through a `connect` chain, and every traversable chain node
accumulated (as a merge survivor) no earlier than one row before any cell that can
still reach it, so its `last_row ≥ y−1` too. Implementation: per-row *touched
lists* (a label id is appended on its first accumulate of each row), consumed two
rows later — ids whose `last_row` still equals that row are dead and return to the
ring. Recycling changes label *ids* only; survivor choice and output order are
id-independent (`(last_row, latest_x)` ordering + raster emission), so golden
parity is preserved. Overflow (all 1023 in flight) is counted in a status register
and drops new components — never fires on the evaluation corpus (assert in tb).

### Line judgment — exact integer form (no sqrt, no division)

With unnormalised scatter `ma = N·Sxx − Sx²`, `mb = N·Sxy − Sx·Sy`,
`mc = N·Syy − Sy²` (all integer), `T = ma + mc`, `R² = (ma−mc)² + 4mb²`:

```
reject  ⇔  λmin/λmax > r  ⇔  T(1−r) > R(1+r)  ⇔  T²(1−r)² > R²(1+r)²
```

`aspect_th = 0.05 = 1/20` exactly, so the gate is the pure-integer comparison

```
361 · T²  >  441 · R²        (reject when true; both sides exact)
```

Together with `pix_num ≥ pixel_num_th` this is the whole baseline judgment — the
software's `sqrt` disappears in the exact reformulation. Widths are large (T² up to
~2^110 worst-case) but this unit runs once per *segment* (~10³/frame), so it is a
multi-cycle sequential multiplier datapath, not a pipeline stage. (Boundary caveat:
software compares via `double`; exact-tie inputs could theoretically differ — the
parity test must confirm zero mismatches on the corpus, else mirror the same
comparison in the golden harness.)

### Segment output

Phase-1 output record per accepted segment (AXI4-Stream, one wide word or a small
burst):

```
{ sx:u11, sy:u11, ex:u11, ey:u11,            // raw endpoint contacts
  N:u18, Sx:u29, Sy:u29, Sxx:u40, Syy:u40, Sxy:u40,    // exact moments
  min_x:u11, min_x_y:u11, max_x:u11, max_x_y:u11,      // (f) bbox extreme
  min_y:u11, min_y_x:u11, max_y:u11, max_y_x:u11 }     //     points
```

The bbox extreme points (v2c improvement f) are always tracked and emitted;
the host finalisation decides whether to use them (`endpoint_from_bbox`:
endpoints = projection extremes among these four points + the two contacts).
In the label table they cost 7 fields (max_y == last_row since rows arrive in
order; max_y_x rides the first-touch-this-row branch of the accumulate).

The only floating-point step in the whole algorithm — projecting the endpoints onto
the PCA axis (`fitEndpoints`) — moves to the **host** (or a phase-2 fixed-point/CORDIC
block). The testbench applies the *existing* software finalisation to these records
and requires bit-identical `LineSegment`s vs `detect()`. For the Atlys overlay demo
the raw integer endpoints are already sufficient.

## Resource estimate (baseline, 1920 wide)

BRAM ≈ 21 KiB (front-end) + ~37 KiB (back-end) ≈ **58 KiB** — inside the thesis's
~70 KiB HW projection, 22 % of Spartan-6 LX45's 261 KiB, 26 % of xc7a35t. DSP usage:
`x²`/`x·y` products in the accumulate path (few), judgment multipliers (time-shared).
No frame buffer anywhere — that is the point.

## Improved-mode notes (staged later)

Integer-clean and stream-compatible, in adoption order:
- (a) strict NMS tie-break — DONE (v2c step 1). NOTE: in RTL write it as
  parallel comparators muxed by strict (`strict ? > : >=`), NOT `Pm + s` —
  the serial +1 adder broke the front-end critical chain (74.25 MHz).
- (j) lattice half-shift — DONE (v2c step 1): host-side +0.5 on finished
  segments; on the board, the overlay draw rounding ((c+1)>>1, clamped).
- (c) sub-pixel NMS — parabola vertex `8(Pp−Pm)/den` needs one small divider (or
  16-entry reciprocal LUT + multiply) in the front-end; moments then accumulate in
  1/16-px units ⇒ +4 fractional bits on Sx/Sy, +8 on Sxx/Syy/Sxy.
- (d) hysteresis — per-label `strong_cnt` (u10) + power in the event record. The
  *adaptive* low threshold's decayed float histogram must be re-expressed in fixed
  point (bins ×256, decay = `v − (v>>8)`) — **not** bit-identical to the float
  software; either mirror the fixed-point form in software (preferred, then re-tune
  nothing: behaviour change is sub-LSB) or pin `hysteresis_adaptive=false` in HW mode.
- (f) bbox endpoints — DONE (v2c step 2): 7 extra fields in the hot entry
  (+77 b), strict-compare updates parallel to the moment adders; see the
  segment-output section above.
- (h) perp-spread — second integer inequality on the same moments:
  `N²·λmin_norm ≤ (N·th)²`-style cross-multiplied form; same judgment unit.
- (i) border margin — comparators on x/y. Free.
- (5) link_collinear stays host-side (float geometry, unbounded active list).

## Repository layout

```
hls/
  DESIGN.md            — this file
  src/                 — synthesizable C++ (top + stages); no dependency on ../src
  tb/                  — C testbench: golden parity vs sweeplsd::detect/intermediates
  compat/              — ap_uint/hls::stream shims so plain g++/CMake builds & runs csim
  scripts/run_hls.tcl  — Vitis HLS: csim → csynth (xc7a35t) → cosim → export reports
  CMakeLists.txt       — host-compiled csim target (tool-free inner loop)
```

Development loop: the tool-free g++ build of `tb/` is the everyday check (seconds);
Vitis HLS csim/csynth/cosim validates pragmas, II, and RTL on demand.

## Phase-1 results (Vitis HLS 2026.1, xc7a35t-cpg236-1, 100 MHz — 2026-07-07)

Reports archived in `reports/` (csynth + cosim).

- **C simulation**: the full parity testbench (10 synthetic + optional corpus
  images) passes with 0 errors under the Vitis compiler — same bit-exact
  standard as the host build (all 160 runs incl. the 150 Waseda FullHD photos).
- **Synthesis**: timing met (slack +0.02 ns @ 10 ns, 1.0 ns uncertainty);
  **BRAM 79/100 (79 %), DSP 79/90 (87 %), LUT 19,789/20,800 (95 %), FF 35 %**.
  Every front-end pixel loop achieves **II=1**; `endpointCore` folds into the
  feature pipeline at II=1 / 3-cycle latency.
- **C/RTL co-simulation**: PASS (xsim, Verilog). Measured frame latency:
  sparse frame 3,306 cycles for 64×48 ≈ (h+2)(w+2) — the pipeline runs at
  exactly pixel rate end to end; full-amplitude-noise frame (worst-case event
  density) ~6 cycles/px, correctly absorbed by FIFO backpressure.
- Two synthesis-driven code rules learned (both output-invariant, enforced in
  the sources): no capturing-lambda accessors in the back-end (pointer-to-
  pointer, HLS 214-134) and no literal `% 3` on row counters (lowers to a
  36-cycle sequential remainder unit); the judgment unit shares ONE multiplier
  via `ALLOCATION` (parallel it costs 177 DSPs = 196 %).
- Toolchain note (2026.1 unified): run with
  `vitis-run --mode hls --tcl scripts/run_hls.tcl`; a **Vivado BASIC** (free
  tier, replaces "ML Standard" from 2026.1) license file must exist before the
  tool launches — `XILINXD_LICENSE_FILE` → `~/.Xilinx/Xilinx.lic`.

Remaining for phase 2 (Atlys / Spartan-6 LX45, hand Verilog): the shared
judgment multiplier as designed here already fits the DSP48A1 budget thinking
(one 128-bit product time-multiplexed); HDMI in/out at 720p60 or 1080p30; ISE
14.7 toolchain.
