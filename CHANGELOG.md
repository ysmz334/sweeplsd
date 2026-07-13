# Changelog

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
