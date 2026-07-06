// Dataset-wide processing-time comparison across detectors, on <indir>/*.png.
// For each image, times each method as the MEDIAN of N runs (warm), then
// aggregates those per-image medians across the dataset (mean / median / min /
// max) and reports mean segment counts too. Investigation tool only.
//
// Methods: SweepLSD baseline (multi/one), SweepLSD improved (multi/one), LSD, EDLines-style.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <filesystem>

#include "edlines.hpp"
#include "edreal_io.hpp"
#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"

#include "lsd.h"  // third_party/lsd (AGPL) — investigation only, not shipped

using sweeplsd::GrayImage;
using sweeplsd::LineSegment;

static double median(std::vector<double> v) {
    std::sort(v.begin(), v.end());
    return v.empty() ? 0.0 : v[v.size() / 2];
}

template <class F>
static double timeMedian(int runs, F&& fn) {
    std::vector<double> t;
    for (int i = 0; i < runs; ++i) {
        auto a = std::chrono::steady_clock::now();
        fn();
        auto b = std::chrono::steady_clock::now();
        t.push_back(std::chrono::duration<double, std::milli>(b - a).count());
    }
    return median(t);
}

static std::vector<LineSegment> runLsd(const GrayImage& s) {
    std::vector<double> buf(std::size_t(s.width) * s.height);
    for (int i = 0; i < s.width * s.height; ++i) buf[i] = double(s.data[i]);
    int n = 0;
    double* out = lsd(&n, buf.data(), s.width, s.height);
    std::vector<LineSegment> segs(n);
    for (int j = 0; j < n; ++j)
        segs[j] = {(float)out[7 * j], (float)out[7 * j + 1],
                   (float)out[7 * j + 2], (float)out[7 * j + 3]};
    std::free(out);
    return segs;
}

struct Method {
    std::string name;
    std::vector<double> ms;      // per-image median time
    std::vector<double> segs;    // per-image segment count
};

static void report(const Method& m) {
    std::vector<double> t = m.ms;
    std::sort(t.begin(), t.end());
    double sum = 0; for (double x : t) sum += x;
    double ssum = 0; for (double x : m.segs) ssum += x;
    int n = (int)t.size();
    std::printf("  %-24s  mean=%7.1f  median=%7.1f  min=%6.1f  max=%7.1f   segs~%6.0f\n",
                m.name.c_str(), sum / n, t[n / 2], t.front(), t.back(), ssum / n);
}

static bool isPng(const std::string& s) {
    return s.size() > 4 && (s.substr(s.size() - 4) == ".png" || s.substr(s.size() - 4) == ".PNG");
}

int main(int argc, char** argv) {
    if (argc < 2) { std::printf("usage: %s <indir> [--runs N] [--limit K]\n", argv[0]); return 2; }
    std::string indir = argv[1];
    int runs = 3, limit = 1 << 30;
    std::string edreal_dir;  // genuine ED_Lib results (<name>.txt with its own median ms)
    for (int i = 2; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--runs" && i + 1 < argc) runs = std::atoi(argv[++i]);
        else if (a == "--limit" && i + 1 < argc) limit = std::atoi(argv[++i]);
        else if (a == "--edreal-dir" && i + 1 < argc) edreal_dir = argv[++i];
    }

    std::vector<std::string> names;
    std::error_code ec;
    for (const auto& e : std::filesystem::directory_iterator(indir, ec)) {
        std::string fn = e.path().filename().string();
        if (isPng(fn.c_str())) names.push_back(fn);
    }
    std::sort(names.begin(), names.end());
    if ((int)names.size() > limit) names.resize(limit);
    if (names.empty()) { std::printf("no .png in %s\n", indir.c_str()); return 1; }

    Method mm{"SweepLSD (multi)"}, m1{"SweepLSD (one-pass)"},
           im{"SweepLSD-improved (multi)"}, i1{"SweepLSD-improved (one)"},
           lsd_m{"LSD"}, ed{"EDLines-style"}, edr{"EDLines (ED_Lib)"};
    const sweeplsd::Params imp = sweeplsd::Params::improved();

    std::printf("timing %zu images, median of %d runs each ...\n", names.size(), runs);
    int done = 0;
    for (const auto& n : names) {
        GrayImage s = sweeplsd::loadGray(indir + "/" + n);
        if (s.width == 0) { std::printf("  LOAD FAIL %s\n", n.c_str()); continue; }
        std::vector<LineSegment> seg;
        mm.ms.push_back(timeMedian(runs, [&]{ seg = sweeplsd::detect(s, sweeplsd::Params{}); }));       mm.segs.push_back(seg.size());
        m1.ms.push_back(timeMedian(runs, [&]{ seg = sweeplsd::detectOnePass(s, sweeplsd::Params{}); }));  m1.segs.push_back(seg.size());
        im.ms.push_back(timeMedian(runs, [&]{ seg = sweeplsd::detect(s, imp); }));                     im.segs.push_back(seg.size());
        i1.ms.push_back(timeMedian(runs, [&]{ seg = sweeplsd::detectOnePass(s, imp); }));              i1.segs.push_back(seg.size());
        lsd_m.ms.push_back(timeMedian(runs, [&]{ seg = runLsd(s); }));                              lsd_m.segs.push_back(seg.size());
        ed.ms.push_back(timeMedian(runs, [&]{ seg = edlines::detect(s); }));                        ed.segs.push_back(seg.size());
        if (!edreal_dir.empty()) {  // ED_Lib timed by its own runner (MSVC+OpenCV, AVX2)
            std::vector<LineSegment> er;
            double ms = 0;
            std::string base = n.substr(0, n.size() - 4);
            if (readEdRealFile(edreal_dir + "/" + base + ".txt", er, &ms)) {
                edr.ms.push_back(ms);
                edr.segs.push_back(er.size());
            }
        }
        if (++done % 25 == 0) { std::printf("  ... %d/%zu\n", done, names.size()); std::fflush(stdout); }
    }

    std::printf("\n=== per-image processing time over %zu images (ms) ===\n", names.size());
    report(mm); report(m1); report(im); report(i1); report(lsd_m); report(ed);
    if (!edr.ms.empty()) report(edr);
    return 0;
}
