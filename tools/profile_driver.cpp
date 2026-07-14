// profile_driver — a minimal, long-running driver for external sampling
// profilers (Visual Studio Performance Profiler "CPU Usage", Very Sleepy, …).
//
// It loads one image once and then runs detectOnePass() in a tight loop for a
// fixed number of iterations, so that virtually all wall-clock time is spent
// inside the detector (no per-iteration I/O). Point a sampling profiler at the
// resulting binary and the samples land on the real pipeline code —
// labelling, the endpoint kernel, the moment accumulation, etc.
//
// Because MinGW g++ emits DWARF (not PDB), build with -g and convert the debug
// info with cv2pdb so Visual Studio can attribute samples to source lines; see
// tools/build_profile.bat and docs/profiling.md.
//
//   profile_driver <image> [--iters N] [--onepass|--multipass]

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("Usage: %s <image> [--iters N] [--onepass|--multipass]\n", argv[0]);
        return 1;
    }
    std::string path = argv[1];
    int iters = 400;
    bool onepass = true;
    for (int i = 2; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--iters" && i + 1 < argc) iters = std::atoi(argv[++i]);
        else if (a == "--multipass") onepass = false;
        else if (a == "--onepass") onepass = true;
    }

    sweeplsd::GrayImage src = sweeplsd::loadGray(path);
    if (src.width == 0) {
        std::printf("Error: could not load '%s'\n", path.c_str());
        return 1;
    }
    const sweeplsd::Params params{};

    std::printf("Profiling %s (%dx%d), %s, %d iterations. Attach the profiler now...\n",
                path.c_str(), src.width, src.height, onepass ? "one-pass" : "multi-pass", iters);

    std::size_t sink = 0;
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < iters; ++i) {
        auto segs = onepass ? sweeplsd::detectOnePass(src, params) : sweeplsd::detect(src, params);
        sink += segs.size();  // keep the call from being optimized away
    }
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::printf("Done: %d iters, %.1f ms total, %.3f ms/iter (%zu segment-sum)\n", iters, ms,
                ms / iters, sink);
    return 0;
}
