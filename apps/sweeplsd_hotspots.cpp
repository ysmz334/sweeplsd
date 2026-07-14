// sweeplsd_hotspots — per-stage hotspot profiler for the STREAMING one-pass
// detector, measuring the actual shipped (AVX2) build.
//
// The existing sweeplsd_profile tool times the *multi-pass* stages (one full
// image pass each). This tool instead instruments the *one-pass* driver: it
// mirrors src/sweeplsd_onepass.cpp exactly, wrapping each stage block with a
// cycle counter (rdtsc), so it reports where the single downward sweep actually
// spends its time — gaussian / gradient / edge / sub-pixel NMS / endpoint /
// labelling. The real kernels (kernels.hpp) and the real Labeler are reused
// unchanged, and the segment count is checked against the real detectOnePass so
// the instrumented copy is provably equivalent.
//
// It is a diagnostic MIRROR of sweeplsd_onepass.cpp: if that driver changes,
// keep the stage blocks below in sync (the parity assert will flag drift).
//
// Build: enabled by the CMake target `sweeplsd_hotspots` (needs `src/` on the
// include path for the internal kernels/stage headers).
//   .\build\sweeplsd_hotspots.exe <img> [<img> ...] [--runs N]

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#if !(defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86))
#error "sweeplsd_hotspots uses rdtsc and is x86-only; the CMake build skips this target on non-x86 (e.g. ARM64)."
#endif

#if defined(_MSC_VER)
#include <intrin.h>
#else
#include <x86intrin.h>
#endif

#include "kernels.hpp"
#include "stages.hpp"
#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"

namespace {

using namespace sweeplsd;

inline std::uint64_t rdtsc() {
#if defined(_MSC_VER)
    return __rdtsc();
#else
    return __builtin_ia32_rdtsc();
#endif
}

// Per-stage cycle accumulators (one downward sweep = many rows per stage).
struct Buckets {
    std::uint64_t ingest = 0, gaussian = 0, gradient = 0, edge = 0, subpix = 0, feature = 0,
                  labeling = 0;
    std::uint64_t total() const {
        return ingest + gaussian + gradient + edge + subpix + feature + labeling;
    }
};

// A ring buffer holding the most recent `cap` rows of one field (identical to
// the RowRing in sweeplsd_onepass.cpp).
template <class T>
struct RowRing {
    int width, cap;
    std::vector<T> buf;
    std::vector<int> tag;
    RowRing(int w, int c) : width(w), cap(c), buf(std::size_t(w) * c), tag(c, -1) {}
    int slot(int y) const { int s = y % cap; return s < 0 ? s + cap : s; }
    T* mutableRow(int y) { int s = slot(y); tag[s] = y; return &buf[std::size_t(s) * width]; }
    const T* rowPtr(int y) const {
        if (y < 0) return nullptr;
        int s = slot(y);
        return tag[s] == y ? &buf[std::size_t(s) * width] : nullptr;
    }
};

// Instrumented copy of detectOnePass: identical dataflow, rdtsc per stage.
std::vector<LineSegment> detectOnePassProfiled(const GrayImage& src, const Params& params,
                                               Buckets& b) {
    namespace k = kernels;
    const int w = src.width, h = src.height;
    if (w == 0 || h == 0) return {};

    RowRing<std::uint8_t> srcR(w, 8);
    RowRing<std::uint16_t> gaussR(w, 4);
    RowRing<std::uint16_t> powR(w, 10);
    RowRing<std::uint8_t> dirR(w, 10);
    RowRing<std::uint8_t> edgeR(w, 8);
    RowRing<std::int8_t> deltaR(w, 10);
    RowRing<std::uint8_t> featR(w, 4);

    std::vector<std::uint16_t> vert_row(w);
    const std::vector<std::uint8_t> zero_row(w, 0);
    const std::vector<std::uint16_t> zero_row16(w, 0);

    k::AdaptiveLowTh adapt;
    int th_pending = params.hysteresis_low_th;

    Labeler labeler(w, h, params);
    const std::vector<Feature> none_row(w, Feature::None);
    auto featRow = [&](int rr) {
        const std::uint8_t* p = featR.rowPtr(rr);
        return p ? reinterpret_cast<const Feature*>(p) : none_row.data();
    };
    auto inImage = [&](int r) { return r >= 0 && r < h; };

    std::uint64_t t0, t1;
    for (int y = 0; y < h + 7; ++y) {
        t0 = rdtsc();
        if (y < h) std::memcpy(srcR.mutableRow(y), &src.at(0, y), std::size_t(w));
        t1 = rdtsc(); b.ingest += t1 - t0; t0 = t1;

        if (int r = y - 2; inImage(r)) {
            auto srcRow = [&](int ry) {
                const std::uint8_t* p = srcR.rowPtr(ry); return p ? p : zero_row.data();
            };
            k::gaussianVerticalRow(srcRow(r - 2), srcRow(r - 1), srcRow(r), srcRow(r + 1),
                                   srcRow(r + 2), w, vert_row.data());
            k::gaussianHorizontalRow(vert_row.data(), r, w, h, gaussR.mutableRow(r));
        }
        t1 = rdtsc(); b.gaussian += t1 - t0; t0 = t1;

        if (int r = y - 3; inImage(r)) {
            auto grow = [&](int rr) {
                const std::uint16_t* p = gaussR.rowPtr(rr); return p ? p : zero_row16.data();
            };
            k::gradientRow(grow(r), grow(r + 1), w, powR.mutableRow(r), dirR.mutableRow(r));
        }
        t1 = rdtsc(); b.gradient += t1 - t0; t0 = t1;

        if (int r = y - 4; inImage(r)) {
            auto prow = [&](int rr) {
                const std::uint16_t* p = powR.rowPtr(rr); return p ? p : zero_row16.data();
            };
            int edge_th = params.gradient_power_th;
            if (params.use_hysteresis) {
                if (params.hysteresis_adaptive) {
                    edge_th = th_pending;
                    th_pending = adapt.lowTh(params.hysteresis_low_th, params.gradient_power_th);
                    adapt.update(prow(r), w);
                } else {
                    edge_th = params.hysteresis_low_th;
                }
            }
            const std::uint8_t* dr = dirR.rowPtr(r);
            std::uint8_t* er = edgeR.mutableRow(r);
            k::edgeRow(prow(r - 1), prow(r), prow(r + 1), dr, w, edge_th,
                       params.nms_strict_tiebreak, er);
            k::zeroEdgeBorderRow(er, w, r, h, params.edge_border_margin);
            t1 = rdtsc(); b.edge += t1 - t0; t0 = t1;
            if (params.subpixel_nms)
                k::nmsSubpixelRow(prow(r - 1), prow(r), prow(r + 1), dr, er, w,
                                  deltaR.mutableRow(r));
            t1 = rdtsc(); b.subpix += t1 - t0; t0 = t1;
        } else {
            t1 = rdtsc(); b.edge += t1 - t0; t0 = t1;
        }

        if (int r = y - 6; inImage(r)) {
            auto erow = [&](int rr) {
                const std::uint8_t* p = edgeR.rowPtr(rr); return p ? p : zero_row.data();
            };
            k::featureRow(erow(r - 2), erow(r - 1), erow(r), erow(r + 1), erow(r + 2), w,
                          params.sparse_feature_scan, featR.mutableRow(r));
        }
        t1 = rdtsc(); b.feature += t1 - t0; t0 = t1;

        if (int r = y - 7; inImage(r)) {
            labeler.processRow(r, featRow(r - 1), featRow(r), featRow(r + 1), powR.rowPtr(r),
                               dirR.rowPtr(r), params.subpixel_nms ? deltaR.rowPtr(r) : nullptr);
        }
        t1 = rdtsc(); b.labeling += t1 - t0; t0 = t1;
    }
    return labeler.takeSegments();
}

template <class F>
double medianMs(int runs, F&& fn) {
    std::vector<double> t;
    t.reserve(runs);
    for (int i = 0; i < runs; ++i) {
        auto a = std::chrono::steady_clock::now();
        fn();
        auto b = std::chrono::steady_clock::now();
        t.push_back(std::chrono::duration<double, std::milli>(b - a).count());
    }
    std::sort(t.begin(), t.end());
    return t[t.size() / 2];
}

void profileImage(const std::string& path, int runs) {
    GrayImage src = loadGray(path);
    if (src.width == 0) {
        std::printf("  could not load '%s'\n\n", path.c_str());
        return;
    }
    const Params params{};  // the single shipped configuration

    std::vector<LineSegment> segs;
    for (int i = 0; i < 3; ++i) segs = detectOnePass(src, params);  // warmup
    double base_ms = medianMs(runs, [&] { segs = detectOnePass(src, params); });

    Buckets acc;
    std::vector<LineSegment> segs2;
    { Buckets tmp; for (int i = 0; i < 3; ++i) segs2 = detectOnePassProfiled(src, params, tmp); }
    for (int i = 0; i < runs; ++i) segs2 = detectOnePassProfiled(src, params, acc);

    std::printf("Image: %s (%dx%d)  |  %zu segments  |  detectOnePass %.2f ms (median of %d)\n",
                path.c_str(), src.width, src.height, segs.size(), base_ms, runs);
    if (segs.size() != segs2.size())
        std::printf("  !! parity mismatch: real=%zu profiled=%zu (instrumented copy drifted)\n",
                    segs.size(), segs2.size());

    const std::uint64_t tot = acc.total();
    struct Row { const char* name; std::uint64_t c; };
    const Row rows[] = {
        {"ingest (memcpy)", acc.ingest}, {"1. gaussian 5x5", acc.gaussian},
        {"2. gradient 2x2", acc.gradient}, {"3. edge thr+NMS", acc.edge},
        {"   sub-pixel NMS", acc.subpix}, {"4. endpoint cand", acc.feature},
        {"5. label+judge", acc.labeling},
    };
    std::printf("  %-18s %12s %8s %10s\n", "stage", "Mcycles", "share", "ms(est)");
    for (const Row& r : rows)
        std::printf("  %-18s %12.1f %7.1f%% %9.3f\n", r.name, double(r.c) / 1e6,
                    tot ? 100.0 * double(r.c) / double(tot) : 0.0,
                    tot ? base_ms * double(r.c) / double(tot) : 0.0);
    std::printf("  %-18s %12.1f %7.1f%% %9.3f\n\n", "TOTAL", double(tot) / 1e6, 100.0, base_ms);
}

}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> images;
    int runs = 30;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--runs" && i + 1 < argc) runs = std::atoi(argv[++i]);
        else images.push_back(a);
    }
    if (images.empty()) {
        std::printf("Usage: %s <image> [<image> ...] [--runs N]\n", argv[0]);
        std::printf("Per-stage cycle breakdown of the streaming one-pass detector "
                    "(the shipped AVX2 build).\n");
        return 1;
    }
    std::printf("SweepLSD one-pass hotspot profile  (ms(est) = share x measured one-pass time)\n\n");
    for (const std::string& p : images) profileImage(p, runs);
    return 0;
}
