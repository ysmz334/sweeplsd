// Parity testbench: the HLS front-end vs the software golden model.
//
// Every stage of hls/src is compared bit-exactly against the corresponding
// software stage (gaussianBlur / computeGradient / extractEdges /
// extractEndpointCandidates), on synthetic images that exercise borders,
// dense noise, and full-width rows. Finally the event stream is fed to the
// *software* Labeler and the resulting segments must equal detect() exactly —
// which validates the event abstraction end to end before the hardware
// labelling engine exists.
//
// Runs both as a plain host build (CMake target sweeplsd_hls_tb, seconds) and
// as the Vitis HLS C simulation testbench.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "backend.hpp"
#include "finalize.hpp"
#include "frontend.hpp"
#include "stages.hpp"  // software golden internals (include dir: ../../src)
#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"

using namespace sweeplsd;
namespace H = sweeplsd_hls;

static int g_failures = 0;

// ---- deterministic synthetic images ----------------------------------------

static std::uint32_t lcg(std::uint32_t& s) {
    s = s * 1664525u + 1013904223u;
    return s >> 8;
}

// Dark background + a few bright bars at assorted angles + mild noise.
static GrayImage makeLineImage(int w, int h, std::uint32_t seed, int nseg = 8) {
    GrayImage img(w, h, 30);
    std::uint32_t s = seed;
    struct Seg {
        float x0, y0, x1, y1, halfw;
    };
    std::vector<Seg> segs;
    for (int i = 0; i < nseg; ++i) {
        Seg sg;
        sg.x0 = float(lcg(s) % std::uint32_t(w));
        sg.y0 = float(lcg(s) % std::uint32_t(h));
        sg.x1 = float(lcg(s) % std::uint32_t(w));
        sg.y1 = float(lcg(s) % std::uint32_t(h));
        sg.halfw = 0.8f + float(lcg(s) % 100) / 100.0f;  // 0.8 .. 1.8 px
        segs.push_back(sg);
    }
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            bool on = false;
            for (const Seg& sg : segs) {
                float dx = sg.x1 - sg.x0, dy = sg.y1 - sg.y0;
                float len2 = dx * dx + dy * dy;
                float t = len2 > 0 ? ((x - sg.x0) * dx + (y - sg.y0) * dy) / len2 : 0.0f;
                if (t < 0) t = 0;
                if (t > 1) t = 1;
                float ex = sg.x0 + t * dx - x, ey = sg.y0 + t * dy - y;
                if (ex * ex + ey * ey <= sg.halfw * sg.halfw) {
                    on = true;
                    break;
                }
            }
            int v = on ? 220 : 30;
            v += int(lcg(s) % 11) - 5;  // +-5 noise
            if (v < 0) v = 0;
            if (v > 255) v = 255;
            img.at(x, y) = std::uint8_t(v);
        }
    return img;
}

// Full-amplitude noise: stresses NMS ties, dense feature rows, event volume.
static GrayImage makeNoiseImage(int w, int h, std::uint32_t seed) {
    GrayImage img(w, h);
    std::uint32_t s = seed;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) img.at(x, y) = std::uint8_t(lcg(s) & 0xff);
    return img;
}

// ---- stream helpers ---------------------------------------------------------

template <class T>
static void pushGrid(const Grid<T>& g, hls::stream<T>& s) {
    for (int y = 0; y < g.height; ++y)
        for (int x = 0; x < g.width; ++x) s.write(g.at(x, y));
}

template <class T>
static Grid<T> pullGrid(hls::stream<T>& s, int w, int h) {
    Grid<T> g(w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) g.at(x, y) = s.read();
    return g;
}

template <class T, class U>
static bool compareGrids(const char* what, const char* img, const Grid<T>& got,
                         const Grid<U>& want) {
    static_assert(sizeof(T) == sizeof(U), "comparable cell types");
    for (int y = 0; y < want.height; ++y)
        for (int x = 0; x < want.width; ++x)
            if (std::memcmp(&got.at(x, y), &want.at(x, y), sizeof(T)) != 0) {
                std::printf("FAIL [%s] %s: first mismatch at (%d, %d)\n", img, what, x, y);
                ++g_failures;
                return false;
            }
    return true;
}

// Rebuild the dense feature map from the sparse event stream.
static Grid<Feature> featureFromEvents(hls::stream<H::Event>& ev, int w, int h) {
    Grid<Feature> feat(w, h, Feature::None);
    int y = 0;
    bool eof = false;
    while (!ev.empty()) {
        H::Event e = ev.read();
        if (e.kind == H::kEventEndOfFrame) {
            eof = true;
            break;
        }
        if (e.kind == H::kEventEndOfRow) {
            ++y;
            continue;
        }
        if (y >= h || e.x >= std::uint16_t(w)) {
            std::printf("FAIL events: coordinate out of range (y=%d x=%u)\n", y, e.x);
            ++g_failures;
            return feat;
        }
        feat.at(int(e.x), y) = Feature(e.kind);
    }
    if (!eof || y != h || !ev.empty()) {
        std::printf("FAIL events: framing broken (eof=%d rows=%d/%d leftover=%zu)\n",
                    int(eof), y, h, ev.size());
        ++g_failures;
    }
    return feat;
}

// ---- per-image test ----------------------------------------------------------

static void testImage(const char* name, const GrayImage& src, const Params& params) {
    const int w = src.width, h = src.height;
    std::printf("-- %s (%dx%d)\n", name, w, h);

    // Software golden chain.
    Grid<std::uint16_t> gauss_sw = gaussianBlur(src);
    GradientField grad_sw = computeGradient(gauss_sw, params);
    EdgeField edge_sw = extractEdges(grad_sw, params);
    Grid<Feature> feat_sw = extractEndpointCandidates(edge_sw.edge, params);
    std::vector<LineSegment> segs_sw = detect(src, params);

    // HLS chain, stage by stage (materialised between stages so every stage
    // output can be compared, while still feeding each stage from the previous
    // HLS stage's output — errors cannot cancel out).
    hls::stream<std::uint8_t> src_s;
    pushGrid(src, src_s);
    hls::stream<std::uint16_t> gauss_s;
    H::hlsGaussian(src_s, gauss_s, w, h);
    Grid<std::uint16_t> gauss_hw = pullGrid(gauss_s, w, h);
    compareGrids("gaussian", name, gauss_hw, gauss_sw);

    hls::stream<std::uint16_t> gauss_s2;
    pushGrid(gauss_hw, gauss_s2);
    hls::stream<H::PowerDir> grad_s;
    H::hlsGradient(gauss_s2, grad_s, w, h);
    Grid<std::uint16_t> pow_hw(w, h);
    Grid<std::uint8_t> dir_hw(w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            H::PowerDir pd = grad_s.read();
            pow_hw.at(x, y) = pd.power;
            dir_hw.at(x, y) = pd.dir;
        }
    compareGrids("gradient.power", name, pow_hw, grad_sw.power);
    compareGrids("gradient.dir", name, dir_hw, grad_sw.dir);

    hls::stream<H::PowerDir> grad_s2;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            grad_s2.write(H::PowerDir{pow_hw.at(x, y), dir_hw.at(x, y)});
    hls::stream<std::uint8_t> edge_s;
    H::hlsEdge(grad_s2, edge_s, w, h, params.gradient_power_th, params.nms_strict_tiebreak,
               params.use_hysteresis, params.hysteresis_adaptive, params.hysteresis_low_th);
    Grid<std::uint8_t> edge_hw = pullGrid(edge_s, w, h);   // {strong<<1 | edge}
    Grid<std::uint8_t> edge_bit(w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) edge_bit.at(x, y) = edge_hw.at(x, y) & 1;
    compareGrids("edge", name, edge_bit, edge_sw.edge);
    // (d) strong bit = power >= the HIGH threshold; verify it at the edge tap.
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            std::uint8_t want =
                grad_sw.power.at(x, y) >= std::uint16_t(params.gradient_power_th) ? 1 : 0;
            if (((edge_hw.at(x, y) >> 1) & 1) != want) {
                std::printf("FAIL [%s] strong: first mismatch at (%d, %d)\n", name, x, y);
                ++g_failures;
                y = h;
                break;
            }
        }

    hls::stream<std::uint8_t> edge_s2;
    pushGrid(edge_hw, edge_s2);   // full {strong<<1 | edge} feeds the feature stage
    hls::stream<std::uint8_t> feat_s;
    H::hlsFeature(edge_s2, feat_s, w, h);
    Grid<std::uint8_t> feat_hw_raw = pullGrid(feat_s, w, h);   // {strong<<2 | code}
    Grid<std::uint8_t> feat_hw(w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) feat_hw.at(x, y) = feat_hw_raw.at(x, y) & 3;
    compareGrids("feature", name, feat_hw,
                 *reinterpret_cast<const Grid<std::uint8_t>*>(&feat_sw));

    // Top-level dataflow: source pixels in, events out; the reconstructed
    // feature map must equal the golden one, and the software Labeler run on
    // it must reproduce detect() bit-exactly (the event stream carries
    // everything the baseline back-end needs).
    hls::stream<std::uint8_t> src_s2;
    pushGrid(src, src_s2);
    hls::stream<H::Event> ev_s;
    const H::HystCfg hyst{params.use_hysteresis, params.hysteresis_adaptive,
                          params.hysteresis_low_th, params.hysteresis_strong_min};
    H::sweeplsdFrontend(src_s2, ev_s, w, h, params.gradient_power_th,
                        params.nms_strict_tiebreak, hyst);
    Grid<Feature> feat_ev = featureFromEvents(ev_s, w, h);
    compareGrids("events->feature", name,
                 *reinterpret_cast<const Grid<std::uint8_t>*>(&feat_ev),
                 *reinterpret_cast<const Grid<std::uint8_t>*>(&feat_sw));

    // The SW Labeler recomputes the strong count from power (== the event's
    // strong bit), so pass the golden power/dir/delta as detect() does.
    std::vector<LineSegment> segs_ev =
        labelAndJudge(feat_ev, params, &grad_sw.power, &grad_sw.dir,
                      params.subpixel_nms ? &edge_sw.delta : nullptr);
    bool seg_ok = segs_ev.size() == segs_sw.size();
    if (seg_ok)
        for (std::size_t i = 0; i < segs_sw.size(); ++i)
            if (std::memcmp(&segs_ev[i], &segs_sw[i], sizeof(LineSegment)) != 0) {
                seg_ok = false;
                break;
            }
    if (!seg_ok) {
        std::printf("FAIL [%s] segments: %zu via events vs %zu via detect()\n", name,
                    segs_ev.size(), segs_sw.size());
        ++g_failures;
    }

    // Full hardware core (front-end + event-driven labelling back-end),
    // driven through the synthesis top (co-simulation hooks the DUT there):
    // records finalised on the host must reproduce detect() bit-exactly.
    hls::stream<std::uint8_t> src_s3;
    pushGrid(src, src_s3);
    hls::stream<H::SegmentRecord> rec_s;
    sweeplsd_core_top(src_s3, rec_s, w, h, params.gradient_power_th,
                      params.nms_strict_tiebreak, params.pixel_num_th,
                      params.use_hysteresis, params.hysteresis_adaptive,
                      params.hysteresis_low_th, params.hysteresis_strong_min);
    std::vector<LineSegment> segs_hw =
        H::finalizeStream(rec_s, params.max_segments, params.endpoint_from_bbox,
                          params.lattice_half_shift);
    bool hw_ok = segs_hw.size() == segs_sw.size();
    if (hw_ok)
        for (std::size_t i = 0; i < segs_sw.size(); ++i)
            if (std::memcmp(&segs_hw[i], &segs_sw[i], sizeof(LineSegment)) != 0) {
                std::printf("FAIL [%s] core: segment %zu differs "
                            "(hw %.4f,%.4f-%.4f,%.4f vs sw %.4f,%.4f-%.4f,%.4f)\n",
                            name, i, segs_hw[i].x0, segs_hw[i].y0, segs_hw[i].x1,
                            segs_hw[i].y1, segs_sw[i].x0, segs_sw[i].y0, segs_sw[i].x1,
                            segs_sw[i].y1);
                hw_ok = false;
                break;
            }
    if (!hw_ok) {
        if (segs_hw.size() != segs_sw.size()) {
            std::printf("FAIL [%s] core: %zu segments vs %zu from detect()\n", name,
                        segs_hw.size(), segs_sw.size());
        }
        ++g_failures;
    }
    if (seg_ok && hw_ok)
        std::printf("   ok: all stages + %zu segments bit-exact (core: %zu)\n",
                    segs_sw.size(), segs_hw.size());
}

int main(int argc, char** argv) {
    Params params;  // baseline (thesis behaviour) — the phase-1 target
    // v2c improved-mode switches — each turns the feature on in BOTH the
    // software golden and the HLS chain (+ host finalisation), validating
    // that path end-to-end over the same corpus.
    if (std::getenv("SWEEPLSD_TB_STRICT") != nullptr)
        params.nms_strict_tiebreak = true;     // (a) strict NMS tie-break
    if (std::getenv("SWEEPLSD_TB_BBOX") != nullptr)
        params.endpoint_from_bbox = true;      // (f) bbox endpoint choice
    if (std::getenv("SWEEPLSD_TB_HALFPX") != nullptr)
        params.lattice_half_shift = true;      // (j) half-pixel shift
    if (std::getenv("SWEEPLSD_TB_HYST") != nullptr)
        params.use_hysteresis = true;          // (d) streaming hysteresis (adaptive)

    // RTL co-simulation runs this whole testbench against the generated RTL;
    // full-size frames would simulate for hours, so SWEEPLSD_TB_SMALL=1
    // restricts the run to the small frames (still covers borders, labelling,
    // merging, and segment emission).
    const bool small_only = std::getenv("SWEEPLSD_TB_SMALL") != nullptr;
    if (small_only) {
        testImage("lines-64x48", makeLineImage(64, 48, 1), params);
        testImage("noise-64x64", makeNoiseImage(64, 64, 6), params);
        testImage("tiny-16x16", makeLineImage(16, 16, 10, 2), params);
        if (g_failures == 0) {
            std::printf("\nALL PASS (small set)\n");
            return 0;
        }
        std::printf("\n%d FAILURE(S)\n", g_failures);
        return 1;
    }

    // Any image files on the command line are tested too (e.g. the FullHD
    // evaluation photos); width must be <= sweeplsd_hls::kMaxWidth.
    for (int i = 1; i < argc; ++i) {
        GrayImage img = loadGray(argv[i]);
        if (img.width == 0) {
            std::printf("FAIL: cannot load %s\n", argv[i]);
            ++g_failures;
            continue;
        }
        if (img.width > H::kMaxWidth) {
            std::printf("SKIP %s: width %d > kMaxWidth %d\n", argv[i], img.width,
                        H::kMaxWidth);
            continue;
        }
        testImage(argv[i], img, params);
    }

    testImage("lines-64x48", makeLineImage(64, 48, 1), params);
    testImage("lines-320x240", makeLineImage(320, 240, 2), params);
    testImage("lines-640x480", makeLineImage(640, 480, 3, 14), params);
    testImage("lines-1920x120", makeLineImage(1920, 120, 4, 12), params);
    testImage("noise-320x240", makeNoiseImage(320, 240, 5), params);
    testImage("noise-64x64", makeNoiseImage(64, 64, 6), params);
    testImage("tiny-4x7", makeNoiseImage(4, 7, 7), params);
    testImage("tiny-7x4", makeNoiseImage(7, 4, 8), params);
    testImage("tiny-5x5", makeNoiseImage(5, 5, 9), params);
    testImage("tiny-16x16", makeLineImage(16, 16, 10, 2), params);

    H::BackendStats st = H::backendStats();
    std::printf("\nbackend high-water: %d live labels (cap %d), %d events/row, "
                "find chain %d, freelist underflows %d\n",
                st.max_live_labels, H::kMaxLabels - 1, st.max_row_events,
                st.max_find_chain, st.freelist_underflows);
    if (st.freelist_underflows != 0) {
        std::printf("FAIL: label free-list underflow\n");
        ++g_failures;
    }

    if (g_failures == 0) {
        std::printf("\nALL PASS — HLS core is bit-exact vs the software golden model\n");
        return 0;
    }
    std::printf("\n%d FAILURE(S)\n", g_failures);
    return 1;
}
