# Changelog

## v3.0.3 (2026-07-17)

- **Kernel performance: Full-HD one-pass ~13.5 → ~12.0 ms (−11 %), output
  bit-identical.** Three independent rewrites of the shared per-pixel kernels in
  `src/kernels.hpp`; no algorithm, numerics, parameter, or API change. Measured
  on the 150-image Full-HD corpus (median of 5 runs per image, i7-8700K, MinGW
  g++ 8.1, AVX2) with the old and new builds interleaved so both see the same
  thermal conditions. The multi-pass detector drops ~17.4 → ~16.0 ms, and
  `Params::original2014()` one-pass ~10.9 → ~9.8 ms. Segment counts are
  unchanged, and the HLS core stays bit-exact against the software golden model
  (the kernels are shared with it).
- **Edge NMS: select the competitor pair with a uint16 blend, not int
  multiplies.** `edgeRow`'s interior picked its two NMS competitors with
  `isV*pc[..] + isH*pa[..]`, which promoted every value to `int`, halving the
  loop to 8 AVX2 lanes and adding four `vpmulld` plus word→dword widening per
  group. Selecting the pair with a `0x0000`/`0xffff` direction mask keeps the
  whole loop in `uint16` (16 lanes, multiply-free): ~2.9× on the isolated loop,
  **edge stage −37 %**. This is safe because every power is < 2^15 (the gaussian
  `>>10` bounds g ≤ 16320, so `power = (|dx|+|dy|+1)/2 ≤ 32640`), so no `uint16`
  add or compare can overflow — verified over 3M randomized pixels across mixed
  ranges, thresholds, and both directions.
- **Endpoint: compute the straight-line test from window parities, not a
  thermometer byte.** `endpointCore` assembled a 15-term thermometer XOR (`xm`,
  ~44 ops) only to test `popcount(xm) > 6`. Each bit of `xm` is exactly the
  parity of an 8-pixel ring window, so that popcount is just the number of
  odd-parity windows: the eight parities are now computed incrementally
  (`par_b = par_{b-1} ^ f_b ^ f_{b+8}`) and summed, dropping both the byte
  assembly and the SWAR popcount (~54 → ~28 ops). **Endpoint stage −17 %.**
  Verified exhaustively over all 2^16 ring configurations. The kernel stays
  branch-free, which is load-bearing: it is what lets `featureRowInterior`
  auto-vectorize.
- **Sub-pixel NMS: index neighbours directly, peel the border columns.**
  `nmsSubpixelRow` ran every surviving edge pixel's three power samples through
  a bounds-checked accessor. Only the two border columns can have an NMS
  neighbour outside the row, so those are now peeled into the checked path and
  the interior scan indexes `pc`/`pa`/`pb` directly. **Sub-pixel stage −9 %**
  (TSC, 3×30-run A/B), holding across edge densities from 6 % to 20 %.
  Behaviour is unchanged for any `edge_border_margin`, including 0.

## v3.0.2 (2026-07-15)

- **Build portability fixes (no behaviour change).** v3.0.1's cold-throw helper
  used `__attribute__((noinline, cold))`, which MSVC cannot parse, so the core
  library failed to build under Visual C++. The `noinline` attribute is now
  emitted through a portable `SWEEPLSD_COLD_NOINLINE` macro (`__declspec(noinline)`
  on MSVC, `__attribute__((noinline, cold))` elsewhere); the `cold` hint is a
  GCC/Clang-only bonus while the load-bearing `noinline` works everywhere.
- **`sweeplsd_hotspots` restricted to x86.** The one-pass hotspot profiler uses
  `rdtsc` / `<x86intrin.h>` and never built on ARM64 (e.g. Apple-silicon
  `macos-latest`). Its CMake target is now gated on `SWEEPLSD_ARCH_X86` and the
  source carries a matching `#error` guard, so non-x86 builds simply skip this
  diagnostic tool instead of failing. The shipped library, CLI, and tests are
  unaffected. CI is green on linux-gcc, linux-clang, windows-msvc, and
  macos-clang (ARM64).

## v3.0.1 (2026-07-15)

- **Labelling performance fix (output bit-identical).** The pool-overflow
  `throw` added in v3.0.0 lived inline in `LabelTable::create()`, which is
  inlined into the per-pixel labelling hot loop. The exception landing pads it
  injected poisoned that loop's codegen, so the labeller ran ~47 % slower than
  necessary even though the throw is never taken on valid input. The throw is
  now in a cold, non-inlined `[[noreturn]]` helper (`poolOverflowError`), which
  restores the hot path. On the 150-image Full-HD corpus this cuts the
  streaming detector from ~17.7 ms to ~13.8 ms per frame (one-pass, i7-8700K,
  −22 %); output is bit-identical to v3.0.0 on all 150 images, and the
  grow-and-report and hard-error behaviours are unchanged. The pool still
  starts at width/4 and hard-errors above width/2.

## v3.0.0 (2026-07-15)

- **BREAKING: removed `Params::sparse_label_scan`.** The labeling row scan's
  8&nbsp;px zero-word skip is gone; the scan is now a single deterministic
  left-to-right sweep with the two border columns hoisted out of the hot loop.
  The skip was mean-neutral but added content-dependent timing jitter (best
  case sped up, worst case not helped). Code that set `sparse_label_scan` must
  drop the assignment; the output is unchanged. (`sparse_feature_scan` is kept —
  the endpoint stage runs its 5×5 kernel on every pixel, so skipping blank runs
  there saves real work.)
- **Bounded label pool — O(width) label memory.** The per-frame label table no
  longer grows unbounded (it reached ~25k slots / ~2.5&nbsp;MB on a dense
  Full-HD frame and never recycled). Both detectors now keep a **fixed pool of
  label records addressed through a ring free-list** and recycle each label's
  slot the moment it dies (end-of-row retire sweep, death rule `last_row < y`),
  matching the thesis's bounded design. The pool starts at the practical
  width/4 and grows toward the ⌈width/2⌉ theoretical bound only if an input
  needs it. Working set drops to ~15&nbsp;KB (cache-resident): labeling is
  ~2.4% faster on Full-HD/dense frames, the tail (p95) ~4% lower, and per-frame
  time is far more stable under system load. **Output is bit-identical** to the
  previous release (verified across the 150-image 720p corpus, default and
  gradient-weighted). Full internals and the recycling safety proof:
  `docs/labeling-internals.md`.
- **New API: `lastPoolGrowthEvents()`.** Returns the number of label-pool
  growths during the most recent `detect()`/`detectOnePass()` on the calling
  thread (0 = normal). Growth past width/4 is surfaced via this counter plus a
  stderr warning; needing more than width/2 is impossible for a correct input,
  so it throws `std::runtime_error` rather than masking a bug.
- **Streaming labeller speedup.** Templated unweighted-fast moment accumulation
  (bit-identical when `weight_by_gradient` is off).
- **One-pass hotspot profiling tools.** `sweeplsd_hotspots` (per-stage cycle
  breakdown), a self-contained sampling line profiler (`tools/line_profiler.cpp`,
  DWARF/`addr2line`-based, no PDB/VS needed), and `docs/profiling.md`.

## v2.0.0 (2026-07-14)

- **BREAKING: `Params{}` is now the shipped configuration** (all measured
  refinements enabled), matching how the paper presents SweepLSD — one
  detector, one configuration. Code that relied on the old default's
  2014-thesis behaviour should switch to the new
  **`Params::original2014()`**; `Params::improved()` remains as an alias of
  the default, so code written against earlier releases keeps compiling and
  keeps its meaning. The CLI gained `--2014` for the original behaviour, and
  all benches/testbenches/golden-vector generators were moved to
  `original2014()` where they measured the 2014 configuration (their outputs
  are unchanged).

## v1.2.0 (2026-07-13)

- **Stronger collinear linking** (`Params::link_collinear`, still off by
  default). The linker used to match on direction + endpoint gap alone, so a
  gap above ~4 px fused the two parallel flanks of a thin bar into one
  diagonal; three changes make larger jumps safe and useful:
  - **Lateral consistency** (`link_lateral_tol`, default 1 px): segments may
    link only if each one's endpoints lie on the *line* of the other, not
    merely parallel to it.
  - `link_max_gap` default **4 → 9 px** (the largest discontinuity jump of
    ELSED, safe under the lateral test) — junction cuts and noise breaks are
    jumped.
  - **Two-stage length threshold** (`link_admit_pix`, default 5): fragments
    at least that many pixels enter the linker, and a chain that never
    cleared `pixel_num_th` on its own must evidence that length as *span*
    when it leaves; segments the judge accepted at the full threshold are
    never dropped.

  Synthetic-GT F-max (strict one-to-one) for improved+link: σ0 0.966→0.973,
  σ10 →0.953, σ20 0.907→**0.935** (previous linker: 0.969/0.949/0.920);
  geometry errors unchanged. Isotropy probes: circles stay **0**, curve
  chords are not assembled (zone 27→32, CoV 1.58→1.37). Downstream
  Manhattan-frame medians improve accordingly — fixed estimator: York
  1.05°→**0.99°**, NYU 12.7°→**10.7°**; fair best-estimator-per-detector
  protocol (NYU): **5.23°** cross-validated, 5.25° at the shipped estimator
  (improved: 6.08°/5.98°).

- **Benchmark harness additions**: genuine-ELSED ingestion (`--elsed-dir`)
  for the synthetic-GT, isotropy, and vanishing-point studies; the
  estimator-menu fair-protocol study is now a committed tool
  (`sweeplsd_vp_bestcfg` + `tools/vp_bestcfg_cv.py`, with an `implink`
  method) whose aggregation reproduces the published numbers exactly.

## v1.1.0 (2026-07-13)

The FPGA release: the thesis proposed OPLSD as a hardware-oriented method but
left the FPGA form as future work — this release closes that gap.

- **HLS implementation** (`hls/`): the full detector as synthesizable HLS C++
  (Vitis HLS; Artix-7 xc7a35t reports, front-end II=1 at 100 MHz), plus a
  tool-free g++ compatibility shim and golden-parity testbenches against
  `detect()`.
- **Verilog RTL implementation** (`rtl/`): portable hand-written core
  (streaming front-end → event FIFO → labelling back-end → integer judge),
  held **bit-exact** against the HLS C model and the C++ reference — the
  SW == HLS == RTL parity gate covers the full 150-photo Full-HD corpus.
- **Live board demo** (`rtl/boards/atlys/`): Digilent Atlys (Spartan-6 LX45,
  2009 silicon) — HDMI in → detect → green segment overlay → HDMI out at
  1080p30 and 720p60, no frame buffer, no external memory, single
  recovered-clock domain. The `improved()` refinements that fit the
  streaming/integer model are all in the hardware.
- **Back-end throughput levers** (each measured and bit-exact): gather
  parallel-skip, judge datapath narrowing + zero-pass skip, fetch folding,
  concurrent event ingest. Dense-frame event-FIFO overflow on the LX45 went
  from ~52 % corpus segment loss (first live build) to **~0.2 %** at the
  shipped 2048-deep FIFO (one dense frame still sheds; zero-drop proven in
  simulation at depth 8192 — see `rtl/DESIGN.md`, "Overflow reality check").
- **Live-robustness set**: event-FIFO marker reserve + hysteretic shedding,
  overlay record-FIFO deepening, judge watchdog, per-pass UART telemetry and
  diagnostic LEDs.
- **XST workaround (important for board builders)**: ISE XST's FSM
  re-encoding **mis-synthesizes the back-end FSM** — a silent functional
  divergence at Timing Score 0, invisible to RTL simulation, diagnosed by
  gate-level simulation of the netlist. All ISE builds now pass
  `-fsm_extract NO`; the full story is in `rtl/DESIGN.md`.
- **Fidelity fix (changes default output)**: the outer 3 px of the frame are
  excluded from the edge test (`Params::edge_border_margin = 3`), removing a
  spurious full-frame ring of false edges that the 2×2 gradient manufactures
  at the image border (the original 2014 implementation suppressed a
  comparable band; the reproduction had dropped that). Baseline segment
  counts drop ~4 % on the Full-HD corpus and the baseline's heavy-noise
  F-max recovers (σ10: 0.47 → 0.94); the improved config is unchanged.
  Evaluation pages regenerated.
- **Evaluation tooling**: 3-level SW / HLS-C / RTL comparison renderer, RTL
  ground-truth burst-overflow simulator, FIFO-drop visualizers.

## v1.0.1 (2026-07-06)

- Fix non-x86 builds and the bench directory listing; add repository links.

## v1.0.0 (2026-07-06)

First public release.

- **Core library** (`sweeplsd::sweeplsd`, MIT, zero dependencies): the
  one-pass line segment detector proposed as *OPLSD* in a 2014 master's
  thesis (Yoshiyasu Shimizu, Waseda University), reimplemented from scratch
  in C++17. Two drivers over shared kernels, tested identical: `detect()`
  (multi-pass, readable) and `detectOnePass()` (streaming single sweep,
  O(width) memory, fastest). Integer-only per-pixel core, no SIMD
  intrinsics — GCC/Clang auto-vectorize the kernels.
- **Thesis-faithful baseline + measured improvements**: `Params{}`
  reproduces the thesis; `Params::improved()` enables sub-pixel NMS,
  adaptive streaming hysteresis, curve rejection, border margin,
  half-pixel lattice correction, and more — each individually benchmarked.
- **I/O and adapters**: `sweeplsd::io` (stb-based PNG/JPG),
  `sweeplsd::opencv` (header-only `cv::Mat` adapter).
- **Tools**: CLI, per-stage profiler, stage-dump.
- **Examples**: calibrated Manhattan-frame estimation with the
  measured-best estimator configuration (`sweeplsd_manhattan`),
  uncalibrated vanishing points, OpenCV integration.
- **Benchmarks** (`-DSWEEPLSD_BUILD_BENCH=ON`): synthetic-GT quality,
  timing, isotropy, and downstream vanishing-point evaluation against the
  genuine author implementations of LSD (AGPL, fetched at configure time —
  never vendored) and ED_Lib EDLines (MIT, fetched; needs OpenCV).
- **Docs** (GitHub Pages): algorithm walkthrough with real stage
  intermediates, speed/quality/vanishing-point evaluations, all numbers
  regenerated from this repository's harness.

Headline numbers (Full-HD, i7-8700K, single thread, AVX2): ~17 ms/frame
one-pass — ~2.5× faster than ED_Lib EDLines, ~14× faster than LSD; best
clean/low-noise F-max and best per-segment direction accuracy of the three;
known limitation (soft low-contrast edges) measured and documented.
