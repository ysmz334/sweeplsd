// Standalone runner for the *real* EDLines (Cihan Topal's ED_Lib, MIT, OpenCV).
//
// Built separately with MSVC + OpenCV (see tools/build_edlines_real.bat) because
// the rest of the project builds with MinGW and the local OpenCV is an MSVC
// build. It runs EDLines on an image, times it (median over N runs), and writes
// a plain-text results file that sweeplsd_compare ingests via --edlines-dir:
//
//   <count> <median_ms>
//   x0 y0 x1 y1
//   ...
//
// This keeps the comparison rendering/tabulation in one place while using the
// authors' genuine EDLines for the EDLines column.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "EDLines.h"

int main(int argc, char** argv) {
    std::string input, out;
    int runs = 5;
    int minlen = -1;  // -1 = ED_Lib's auto (NFA-based) minimum line length
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--runs" && i + 1 < argc) runs = std::atoi(argv[++i]);
        else if (a == "--out" && i + 1 < argc) out = argv[++i];
        else if (a == "--minlen" && i + 1 < argc) minlen = std::atoi(argv[++i]);
        else if (!a.empty() && a[0] != '-') input = a;
    }
    if (input.empty() || out.empty()) {
        std::printf("Usage: %s <image> --out <segs.txt> [--runs N] [--minlen N]\n", argv[0]);
        return 1;
    }

    cv::Mat img = cv::imread(input, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::printf("Error: could not load '%s'\n", input.c_str());
        return 1;
    }

    std::vector<LS> lines;
    std::vector<double> times;
    for (int i = 0; i < runs; ++i) {
        auto t0 = std::chrono::steady_clock::now();
        EDLines detector(img, 1.0, minlen);  // default line_error, sweepable min length
        lines = detector.getLines();
        auto t1 = std::chrono::steady_clock::now();
        times.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
    std::sort(times.begin(), times.end());
    double median = times[times.size() / 2];

    std::ofstream o(out);
    if (!o) {
        std::printf("Error: cannot write '%s'\n", out.c_str());
        return 1;
    }
    // Full precision: ED_Lib's endpoints come from its least-squares line fit
    // and carry sub-pixel information the geometric evaluation needs.
    o.precision(6);
    o << std::fixed;
    o << lines.size() << " " << median << "\n";
    for (const LS& l : lines)
        o << l.start.x << " " << l.start.y << " " << l.end.x << " " << l.end.y << "\n";

    std::printf("%s: %zu lines, %.2f ms (median of %d)\n", input.c_str(), lines.size(), median, runs);
    return 0;
}
