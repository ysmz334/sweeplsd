# Profiling SweepLSD — where does the one-pass time go?

Three ways to see how the streaming `detectOnePass()` spends its time. All
measure the **shipped AVX2 build** (MinGW `g++ -O3 -mavx2`); the MSVC build does
not auto-vectorize the byte kernels and has a different, slower profile, so do
not profile it if you want representative numbers.

| view | tool | granularity | setup |
|------|------|-------------|-------|
| which **stage** dominates | `sweeplsd_hotspots` (A) | per pipeline stage | none |
| which **source line** | `line_profiler` (B) | source file:line (sampled) | none |
| the same, in a **GUI** | VS Performance Profiler + cv2pdb (C) | source line (sampled) | one download |

Use A first (exact, instant). For line level, **B is the recommended path** — it
reads the DWARF directly and has no VS/PDB moving parts. C is the same data in
the Visual Studio GUI, if you prefer it.

---

## A. `sweeplsd_hotspots` — per-stage breakdown

A CMake target that mirrors `src/sweeplsd_onepass.cpp` and wraps each stage of
the single downward sweep with a cycle counter, then reports each stage's share
of the measured one-pass time. The real kernels and the real `Labeler` are used
unchanged, and the segment count is checked against `detectOnePass()` so the
instrumented copy is provably equivalent.

```powershell
cmake --build build --target sweeplsd_hotspots
.\build\sweeplsd_hotspots.exe <img> [<img> ...] [--runs N]
```

Example (Full-HD, i7-class):

```
Image: IMGP0942.png (1920x1080) | 2938 segments | detectOnePass 16.8 ms
  stage               Mcycles    share    ms(est)
  1. gaussian 5x5       204.6     8.5%     1.43
  2. gradient 2x2       126.9     5.3%     0.89
  3. edge thr+NMS       248.4    10.3%     1.73
     sub-pixel NMS      280.1    11.6%     1.95
  4. endpoint cand      368.4    15.3%     2.57
  5. label+judge       1163.1    48.2%     8.11
```

The split is **content-dependent**: on sparse images the endpoint-candidate
stage dominates (~34 %); on dense/cluttered ones the labeller does (~48 %). Run
a few representative images.

---

## B. `line_profiler` — source-line level, self-contained (recommended)

A tiny sampling profiler (`tools/line_profiler.cpp`) that launches the `-g`
binary, samples its instruction pointer at ~1 kHz, and resolves each sample to a
source `file:line` and function with `addr2line`, which reads the DWARF debug
info directly. **No cv2pdb, no Visual Studio, no PDB** — so none of the source
paths / checksum quirks that trip VS up.

### Build and run

```bat
tools\build_profile.bat
```

This compiles the library + `tools/profile_driver.cpp` with `-O3 -mavx2 -g`
into `build_profile\profile_driver.exe` (DWARF kept), builds
`build_profile\line_profiler.exe`, and also makes the VS copy (see C). The
driver loads one image once and runs the detector in a tight loop, so nearly all
time is inside the pipeline. Then, **with MinGW on PATH** (the profiler shells
out to `addr2line`):

```powershell
$env:Path = "C:\Program Files\mingw-w64\x86_64-8.1.0-posix-sjlj-rt_v6-rev0\mingw64\bin;" + $env:Path
.\build_profile\line_profiler.exe .\build_profile\profile_driver.exe `
    E:\dataset\WasedaDataset\IMGP0942.png --iters 400
```

Output: a **per-file rollup**, the **hottest source lines** (`file:line` +
sample %), and the **hottest functions** (demangled). Example (dense image):

```
== per-file rollup (self time) ==
   56.1%   kernels.hpp          (the 5 auto-vectorized stage kernels live here)
   35.5%   labeling.cpp         (the streaming labeller)
    2.9%   stl_vector.h            (the label pool's row buffers)
    1.0%   sweeplsd_onepass.cpp    (the ring-buffer driver)
    0.7%   stb_image.h             (one-off: decoding the input)
    1.0%   atan2.c / sin / cos     (per-segment judgment transcendentals)

== hottest SOURCE LINES ==
    4.9%  kernels.hpp:234          (sub-pixel NMS parabola)
    3.6%  labeling.cpp:574         (the per-pixel labelling body)
    3.5%  kernels.hpp:79           (gaussian, horizontal pass)
    3.5%  kernels.hpp:373          (endpoint-candidate emit)
    2.4%  labeling.cpp:137         (union-find find() loop)
    ...
```

The rollup is flat by design — no single line dominates. Two shapes are worth
recognising if you are used to older profiles of this code: the label table no
longer reallocates (the bounded pool made `stl_uninitialized.h` / `new` vanish
from the top; they were ~12 % combined), and the labelling **row scan** no
longer shows up (v3.0.4's word skip took it from 47 % of `labeling.cpp` to
13 %, leaving the per-pixel body as the labeller's real cost).

Notes:
- `--iters` sets sample count (~1 kHz × runtime): 400 iters ≈ a few thousand
  samples, enough to rank lines. Bump it for a smoother profile.
- Inlining does **not** hide the inner helpers: GCC's line table maps each
  inlined instruction to *its own* source line, so `find`, `accumulate`,
  `merge`, `subpixelOffset` show up on **their** `labeling.cpp` lines even
  though they're inlined into `processRow`.
- The `[library / no-symbol]` bucket in the *function* list is optimized code
  `addr2line -f` can't name; the *line* rollup still resolves it (that is the
  view to read). It cross-checks against tool A (labelling ≈ 48 %).

## C. Visual Studio Performance Profiler + cv2pdb (GUI)

Same sampled data in the VS GUI. VS wants a PDB;
[`cv2pdb`](https://github.com/rainers/cv2pdb) converts the MinGW DWARF→PDB.
`build_profile.bat` (above) already produced `build_profile\profile_driver_vs.exe`
+ `.pdb` for this.

- **Get cv2pdb** (one-time): `build_profile.bat` expects
  `tools/cv2pdb/cv2pdb64.exe`, a third-party download (GPL), not vendored. Grab
  the latest zip from <https://github.com/rainers/cv2pdb/releases> and extract
  `cv2pdb64.exe` into `tools/cv2pdb/`.
- **Profile**: VS 2022 → **Debug → Performance Profiler** (Alt + F2) → target
  **Executable** `build_profile\profile_driver_vs.exe`, arguments e.g.
  `E:\dataset\WasedaDataset\IMGP0942.png --iters 400` → tick **CPU Usage** →
  **Start**. In the report, **Show External Code** OFF, sort by **Self CPU**,
  **double-click** a hot function to open its source with a per-line heat column.

### If .cpp sources show "source not found" (but headers open)

A known cv2pdb quirk with VS: cv2pdb writes **no source checksums** (they show as
`(None)` in the PDB). VS applies its "does the source match?" check to a
compiland's **primary source** (the `.cpp`) but is lenient on included headers —
so `.hpp` files open while `.cpp` files are rejected. Fix:

- **Tools → Options → Debugging → General → untick “Require source files to
  exactly match the original version.”** Then re-open the source.
- The `.cpp` paths are otherwise correct (verified: `build_profile.bat`
  normalizes them, and the PDB records e.g. `E:\dev\claude\sweeplsd\src\labeling.cpp`
  with ~435 line records). If it still refuses, use **B** — it has none of this
  fragility.

> Other DWARF-native profilers that skip cv2pdb entirely: **Very Sleepy** (small
> GUI sampler) or **Intel VTune** (best line level; point it at the `-g`
> `profile_driver.exe` directly).
