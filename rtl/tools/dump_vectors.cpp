// Golden-vector generator for the RTL testbenches (rtl/tb).
//
// Renders the same deterministic test images as the phase-1 HLS testbench,
// runs the *verified* golden chain (the sweeplsd software stages for the
// intermediate taps, the phase-1 HLS C model for events and segment records
// — itself bit-exact against detect() on the whole corpus), and writes one
// hex file per stage boundary for $readmemh:
//
//   <name>_meta.txt    "width height power_th pixel_num_th strict"
//   <name>_gray.hex    w*h bytes           (raster order, 2 hex digits)
//   <name>_gauss.hex   w*h u16
//   <name>_power.hex   w*h u16
//   <name>_dir.hex     w*h u1
//   <name>_edge.hex    w*h u1
//   <name>_feat.hex    w*h u2
//   <name>_events.hex  one event per line: kind(2) x(4)   [kind:x packed]
//   <name>_records.hex one record per line, fields hex-packed (see tb)
//
// Usage: dump_vectors <outdir> [--improved] [image.png ...]
//
// --improved dumps the v2c improved-mode vectors instead: strict NMS
// tie-break on (Params::nms_strict_tiebreak), names suffixed "_imp" so both
// variants can coexist in one directory (run_tb.sh runs every *_meta.txt and
// passes the strict flag to the testbench). The (j) half-pixel shift is a
// once-per-segment output correction (overlay drawing on the board) and does
// not change events/records, so it needs no vectors.

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "backend.hpp"
#include "frontend.hpp"
#include "stages.hpp"
#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"

using namespace sweeplsd;
namespace H = sweeplsd_hls;

// -- same deterministic generators as hls/tb/tb_frontend.cpp ---------------
static std::uint32_t lcg(std::uint32_t& s) {
    s = s * 1664525u + 1013904223u;
    return s >> 8;
}

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
        sg.halfw = 0.8f + float(lcg(s) % 100) / 100.0f;
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
            v += int(lcg(s) % 11) - 5;
            if (v < 0) v = 0;
            if (v > 255) v = 255;
            img.at(x, y) = std::uint8_t(v);
        }
    return img;
}

static GrayImage makeNoiseImage(int w, int h, std::uint32_t seed) {
    GrayImage img(w, h);
    std::uint32_t s = seed;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) img.at(x, y) = std::uint8_t(lcg(s) & 0xff);
    return img;
}

// ---------------------------------------------------------------------------

template <class T>
static void writeGridHex(const std::string& path, const Grid<T>& g, int digits) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) {
        std::perror(path.c_str());
        std::exit(1);
    }
    for (int y = 0; y < g.height; ++y)
        for (int x = 0; x < g.width; ++x)
            std::fprintf(f, "%0*llx\n", digits,
                         (unsigned long long)(std::uint64_t)g.at(x, y));
    std::fclose(f);
}

static void dumpImage(const std::string& dir, const std::string& name,
                      const GrayImage& src, const Params& params_in) {
    const int w = src.width, h = src.height;
    const std::string base = dir + "/" + name;

    // The RTL adaptive histogram runs a 64-bin percentile scan over a full row
    // and therefore needs width >= 64 (all board inputs are 640/1280/1920). For
    // narrow unit-test images fall back to the FIXED low threshold so the golden
    // stays reproducible in RTL; SW/HLS honour the same hysteresis_adaptive flag.
    Params params = params_in;
    if (params.use_hysteresis && params.hysteresis_adaptive && w < 64)
        params.hysteresis_adaptive = false;

    // (h) 2*max_perp_spread^2 as an integer (0 = off; the default preset mps=1
    // gives 2). Guard that it is integer-representable — the only value the RTL
    // supports; other mps would need a rational threshold.
    const int mps_2sq = params.max_perp_spread > 0.0
                            ? int(2.0 * params.max_perp_spread * params.max_perp_spread + 0.5)
                            : 0;

    // meta
    {
        std::FILE* f = std::fopen((base + "_meta.txt").c_str(), "w");
        // width height power_th pix_th strict hyst_on hyst_adaptive hyst_low
        //   hyst_strong_min border_margin mps_2sq
        std::fprintf(f, "%d %d %d %d %d %d %d %d %d %d %d\n", w, h, params.gradient_power_th,
                     params.pixel_num_th, params.nms_strict_tiebreak ? 1 : 0,
                     params.use_hysteresis ? 1 : 0, params.hysteresis_adaptive ? 1 : 0,
                     params.hysteresis_low_th, params.hysteresis_strong_min,
                     params.border_margin, mps_2sq);
        std::fclose(f);
    }

    writeGridHex(base + "_gray.hex", src, 2);

    // golden intermediates (software stages — the reference the HLS core was
    // proven against)
    Grid<std::uint16_t> gauss = gaussianBlur(src);
    GradientField grad = computeGradient(gauss, params);
    EdgeField edge = extractEdges(grad, params);
    Grid<Feature> feat = extractEndpointCandidates(edge.edge, params);
    writeGridHex(base + "_gauss.hex", gauss, 4);
    writeGridHex(base + "_power.hex", grad.power, 4);
    writeGridHex(base + "_dir.hex", *reinterpret_cast<const Grid<std::uint8_t>*>(&grad.dir), 1);
    writeGridHex(base + "_edge.hex", edge.edge, 1);
    writeGridHex(base + "_feat.hex", *reinterpret_cast<const Grid<std::uint8_t>*>(&feat), 1);

    // events + records via the phase-1 C model (bit-exact vs detect())
    hls::stream<std::uint8_t> src_s;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) src_s.write(src.at(x, y));
    hls::stream<H::Event> ev_s;
    const H::HystCfg hyst{params.use_hysteresis, params.hysteresis_adaptive,
                          params.hysteresis_low_th, params.hysteresis_strong_min};
    H::sweeplsdFrontend(src_s, ev_s, w, h, params.gradient_power_th,
                        params.nms_strict_tiebreak, hyst);
    {
        // Packed golden event word: bit14 = strong ((d) hysteresis, interior
        // only), bits[13:12] = kind, bits[11:0] = x.
        std::FILE* f = std::fopen((base + "_events.hex").c_str(), "w");
        while (!ev_s.empty()) {
            H::Event e = ev_s.read();
            std::fprintf(f, "%04x\n", ((e.strong & 1) << 14) | ((e.kind & 3) << 12) |
                                          (e.x & 0xfff));
            if (e.kind == H::kEventEndOfFrame) break;
        }
        std::fclose(f);
    }

    hls::stream<std::uint8_t> src_s2;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) src_s2.write(src.at(x, y));
    hls::stream<H::SegmentRecord> rec_s;
    H::sweeplsdCore(src_s2, rec_s, w, h, params.gradient_power_th,
                    params.nms_strict_tiebreak, params.pixel_num_th, hyst,
                    params.border_margin, mps_2sq);
    {
        // One record per line: sx sy ex ey n xs ys xss yss xys, then the (f)
        // bbox extreme points minx minxy maxx maxxy miny minyx maxy maxyx
        // (fixed-width hex fields, space-separated — the tb reads with
        // $fscanf %h). The bbox fields are always emitted; baseline testbench
        // columns 1-10 are unchanged.
        std::FILE* f = std::fopen((base + "_records.hex").c_str(), "w");
        int count = 0;
        while (!rec_s.empty()) {
            H::SegmentRecord r = rec_s.read();
            if (r.n == 0) break;
            std::fprintf(f,
                         "%03x %03x %03x %03x %05x %08llx %08llx %011llx %011llx %011llx"
                         " %03x %03x %03x %03x %03x %03x %03x %03x\n",
                         r.sx, r.sy, r.ex, r.ey, r.n,
                         (unsigned long long)r.x_sum, (unsigned long long)r.y_sum,
                         (unsigned long long)r.x_sq_sum, (unsigned long long)r.y_sq_sum,
                         (unsigned long long)r.xy_sum,
                         r.min_x, r.min_x_y, r.max_x, r.max_x_y,
                         r.min_y, r.min_y_x, r.max_y, r.max_y_x);
            ++count;
        }
        std::fclose(f);
        std::printf("%s: %dx%d, %d records\n", name.c_str(), w, h, count);
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr,
                     "usage: dump_vectors <outdir> [--improved] [image.png ...]\n");
        return 1;
    }
    const std::string dir = argv[1];
    Params params;  // baseline — the phase-2 target, same as phase 1
    std::string suffix;
    int argi = 2;
    if (argi < argc && std::string(argv[argi]) == "--improved") {
        // v2c improved mode: the switches implemented in RTL so far
        //   (a) strict NMS, (d) streaming hysteresis with the adaptive low
        //   threshold. (f) bbox endpoints and (j) half-shift are always in the
        //   record / drawn on the board, so they need no separate golden.
        params.nms_strict_tiebreak = true;
        params.use_hysteresis = true;       // adaptive (default) low threshold
        params.hysteresis_low_th = 120;
        params.hysteresis_strong_min = 3;
        params.max_perp_spread = 1.0;       // (h) curve rejection (2*mps^2 = 2)
        params.border_margin = 3;           // (i) drop frame-edge artifacts
        suffix = "_imp";
        ++argi;
    }

    dumpImage(dir, "lines64" + suffix, makeLineImage(64, 48, 1), params);
    dumpImage(dir, "noise64" + suffix, makeNoiseImage(64, 64, 6), params);
    dumpImage(dir, "tiny16" + suffix, makeLineImage(16, 16, 10, 2), params);

    for (int i = argi; i < argc; ++i) {
        GrayImage img = loadGray(argv[i]);
        if (img.width == 0) {
            std::fprintf(stderr, "cannot load %s\n", argv[i]);
            return 1;
        }
        std::string name = argv[i];
        std::size_t slash = name.find_last_of("/\\");
        if (slash != std::string::npos) name = name.substr(slash + 1);
        std::size_t dot = name.find_last_of('.');
        if (dot != std::string::npos) name = name.substr(0, dot);
        dumpImage(dir, name + suffix, img, params);
    }
    return 0;
}
