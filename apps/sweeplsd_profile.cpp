#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"

// Per-stage profiler for SweepLSD: shows where the time goes in the pipeline, plus
// the multi-pass vs one-pass totals. Build flags / SIMD target affect the
// numbers, so it prints under whatever ISA the binary was compiled for.

namespace {
template <class F>
double medianMs(int runs, F&& fn) {
    std::vector<double> t;
    for (int i = 0; i < runs; ++i) {
        auto a = std::chrono::steady_clock::now();
        fn();
        auto b = std::chrono::steady_clock::now();
        t.push_back(std::chrono::duration<double, std::milli>(b - a).count());
    }
    std::sort(t.begin(), t.end());
    return t[t.size() / 2];
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("Usage: %s <image> [--runs N]\n", argv[0]);
        return 1;
    }
    std::string input = argv[1];
    int runs = 10;
    for (int i = 2; i < argc; ++i)
        if (std::string(argv[i]) == "--runs" && i + 1 < argc) runs = std::atoi(argv[++i]);

    sweeplsd::GrayImage src = sweeplsd::loadGray(input);
    if (src.width == 0) {
        std::printf("Error: could not load '%s'\n", input.c_str());
        return 1;
    }

    auto stages = sweeplsd::profileStages(src, {}, runs);
    double stage_sum = 0;
    for (const auto& s : stages) stage_sum += s.ms;

    std::vector<sweeplsd::LineSegment> segs;
    double total_multi = medianMs(runs, [&] { segs = sweeplsd::detect(src); });
    double total_one = medianMs(runs, [&] { sweeplsd::detectOnePass(src); });

    std::printf("Image: %s (%dx%d), median of %d runs\n\n", input.c_str(), src.width, src.height, runs);
    std::printf("SweepLSD multi-pass, per stage:\n");
    std::printf("  %-20s %10s %8s\n", "stage", "ms", "%");
    for (const auto& s : stages)
        std::printf("  %-20s %10.2f %7.1f%%\n", s.name.c_str(), s.ms, 100.0 * s.ms / stage_sum);
    std::printf("  %-20s %10.2f %7.1f%%\n", "(sum of stages)", stage_sum, 100.0);

    std::printf("\nTotals:\n");
    std::printf("  %-20s %10.2f ms  (%zu segments)\n", "detect (multi-pass)", total_multi, segs.size());
    std::printf("  %-20s %10.2f ms\n", "detectOnePass", total_one);
    return 0;
}
