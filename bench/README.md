# Benchmarks

The benchmark harness compares SweepLSD against the **genuine author
implementations** of the baselines:

- **LSD** (R. Grompone von Gioi et al.) — **AGPL v3**, which is why it is not
  vendored in this MIT repository. It is downloaded at configure time.
- **EDLines / ED_Lib** (C. Akinlar & C. Topal) — MIT; fetched the same way for
  symmetry.

Enable with:

```sh
cmake -S . -B build -DSWEEPLSD_BUILD_BENCH=ON
```

Note: building with `SWEEPLSD_BUILD_BENCH=ON` compiles and links AGPL-licensed
code into the benchmark executables (only). The `sweeplsd` library itself and
all default build targets remain MIT-clean.

(Harness and fetch scripts land before v1.0 — tracked in the repo plan.)
