# Changelog

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
