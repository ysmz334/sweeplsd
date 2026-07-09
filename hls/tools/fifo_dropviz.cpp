// Visualiser for the FIFO-drop study (companion to fifo_dropsim.cpp): for each
// image it renders two PNGs over the dimmed grayscale source.
//
//  <base>_buffer.png : every produced event pixel, coloured by the elastic
//     FIFO's occupancy at the moment it was produced — a blue ramp from pale
//     (empty, plenty of headroom) to deep blue (near the 2040 drop line) — and
//     every DROPPED event pixel in red. A 12-px left gutter shows, per image
//     row, the peak occupancy of that row (same ramp; red if the row dropped),
//     so the buffer's fill trajectory down the frame is readable at a glance.
//
//  <base>_segs.png : the finalised segments, GREEN if they survive the drop
//     and RED if they are lost (present in the full-stream detection but gone
//     once the dropped events are removed). Same overlay style as
//     saveSegmentVisualization.
//
// Everything is driven by the bit-exact HLS front-end + back-end and the same
// row-granular FIFO timing model as fifo_dropsim.cpp (1080p30 live geometry,
// improved-mode config = live_core.v). Usage:
//
//   fifo_dropviz --out DIR <img...>  [--hblank N --ing N --proc N --scav N
//                                     --depth N --half]

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include "backend.hpp"
#include "finalize.hpp"
#include "frontend.hpp"
#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"

using namespace sweeplsd;
namespace H = sweeplsd_hls;

namespace {

int g_hblank = 280, g_ing = 2, g_proc = 15, g_scav = 0, g_depth = 2048;
bool g_half = false;
std::string g_out = ".";

struct Ev { std::uint8_t kind; std::uint16_t x; std::uint8_t strong; int y; long pcyc; };

struct RGB { std::vector<unsigned char> px; int w = 0, h = 0;
    void init(const GrayImage& g, double dim) {
        w = g.width; h = g.height; px.assign(std::size_t(w) * h * 3, 0);
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                unsigned char v = (unsigned char)(g.at(x, y) * dim);
                std::size_t o = (std::size_t(y) * w + x) * 3;
                px[o] = px[o + 1] = px[o + 2] = v;
            }
    }
    void set(int x, int y, unsigned char r, unsigned char g, unsigned char b) {
        if (x < 0 || y < 0 || x >= w || y >= h) return;
        std::size_t o = (std::size_t(y) * w + x) * 3;
        px[o] = r; px[o + 1] = g; px[o + 2] = b;
    }
};

// occupancy fraction -> pale-blue (0) .. deep-blue (1)
void rampBlue(double t, unsigned char& r, unsigned char& g, unsigned char& b) {
    if (t < 0) t = 0; if (t > 1) t = 1;
    r = g = (unsigned char)(205.0 * (1.0 - t));
    b = 255;
}

void thickLine(RGB& img, double x0, double y0, double x1, double y1,
               unsigned char r, unsigned char g, unsigned char b, int rad) {
    // integer Bresenham with a small square brush
    int X0 = (int)std::lround(x0), Y0 = (int)std::lround(y0);
    int X1 = (int)std::lround(x1), Y1 = (int)std::lround(y1);
    int dx = std::abs(X1 - X0), sx = X0 < X1 ? 1 : -1;
    int dy = -std::abs(Y1 - Y0), sy = Y0 < Y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        for (int ay = -rad; ay <= rad; ++ay)
            for (int ax = -rad; ax <= rad; ++ax) img.set(X0 + ax, Y0 + ay, r, g, b);
        if (X0 == X1 && Y0 == Y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; X0 += sx; }
        if (e2 <= dx) { err += dx; Y0 += sy; }
    }
}

void savePng(const std::string& path, const RGB& img) {
    saveRgbPng(path, img.w, img.h, img.px);
}
void saveScaled(const std::string& path, const RGB& img, int f) {
    // downscale by integer factor f, but keep any saturated red/blue marker in
    // the f x f block (max over the coloured channel) so thin dropped/full
    // pixels survive downsampling instead of averaging away.
    int w2 = img.w / f, h2 = img.h / f;
    std::vector<unsigned char> out(std::size_t(w2) * h2 * 3);
    for (int y = 0; y < h2; ++y)
        for (int x = 0; x < w2; ++x) {
            int br = 0, bg = 0, bb = 0, redness = -1, blueness = -1;
            for (int dy = 0; dy < f; ++dy)
                for (int dx = 0; dx < f; ++dx) {
                    std::size_t o = (std::size_t(y * f + dy) * img.w + (x * f + dx)) * 3;
                    int r = img.px[o], g = img.px[o + 1], b = img.px[o + 2];
                    int rr = r - (g + b) / 2, bl = b - (r + g) / 2;
                    if (rr > redness) { redness = rr; }
                    if (bl > blueness) { blueness = bl; }
                    br += r; bg += g; bb += b;
                }
            int n = f * f;
            unsigned char ar = (unsigned char)(br / n), ag = (unsigned char)(bg / n),
                          ab = (unsigned char)(bb / n);
            std::size_t oo = (std::size_t(y) * w2 + x) * 3;
            if (redness > 60) { out[oo] = 235; out[oo + 1] = 30; out[oo + 2] = 30; }
            else if (blueness > 60) { out[oo] = ar; out[oo + 1] = ag; out[oo + 2] = 255; }
            else { out[oo] = ar; out[oo + 1] = ag; out[oo + 2] = ab; }
        }
    saveRgbPng(path, w2, h2, out);
}

std::vector<LineSegment> backendSegs(const std::vector<Ev>& evs, const std::vector<char>& drop,
                                     int w, int h, const Params& p) {
    hls::stream<H::Event> es;
    for (std::size_t i = 0; i < evs.size(); ++i) {
        if (!drop.empty() && drop[i]) continue;
        es.write(H::Event{evs[i].kind, evs[i].x, evs[i].strong});
    }
    hls::stream<H::SegmentRecord> rec;
    const H::HystCfg hyst{p.use_hysteresis, p.hysteresis_adaptive, p.hysteresis_low_th,
                          p.hysteresis_strong_min};
    const int mps_2sq = int(2.0 * p.max_perp_spread * p.max_perp_spread + 0.5);
    H::sweeplsdBackend(es, rec, w, h, p.pixel_num_th, hyst, p.border_margin, mps_2sq);
    return H::finalizeStream(rec, p.max_segments, p.endpoint_from_bbox, p.lattice_half_shift);
}

struct Mid { double cx, cy, ang, len; };
Mid midOf(const LineSegment& s) {
    double dx = s.x1 - s.x0, dy = s.y1 - s.y0;
    return {0.5 * (s.x0 + s.x1), 0.5 * (s.y0 + s.y1), std::atan2(dy, dx),
            std::sqrt(dx * dx + dy * dy)};
}

void process(const std::string& path, const Params& p) {
    GrayImage img = loadGray(path);
    if (img.width == 0) { std::fprintf(stderr, "SKIP (load): %s\n", path.c_str()); return; }
    if (img.width > H::kMaxWidth) { std::fprintf(stderr, "SKIP (width): %s\n", path.c_str()); return; }
    std::string base = path;
    std::size_t s = base.find_last_of("/\\");
    if (s != std::string::npos) base = base.substr(s + 1);
    std::size_t dot = base.find_last_of('.');
    if (dot != std::string::npos) base = base.substr(0, dot);

    const int W = img.width, Hh = img.height;
    const long Tline = W + g_hblank;

    // front-end -> events
    hls::stream<std::uint8_t> src_s;
    for (int y = 0; y < Hh; ++y)
        for (int x = 0; x < W; ++x) src_s.write(img.at(x, y));
    hls::stream<H::Event> ev;
    const H::HystCfg hyst{p.use_hysteresis, p.hysteresis_adaptive, p.hysteresis_low_th,
                          p.hysteresis_strong_min};
    H::sweeplsdFrontend(src_s, ev, W, Hh, p.gradient_power_th, p.nms_strict_tiebreak, hyst);

    std::vector<Ev> evs;
    std::vector<long> interior_row(Hh, 0);
    int y = 0;
    while (!ev.empty()) {
        H::Event e = ev.read();
        Ev x{e.kind, e.x, e.strong, y, 0};
        if (e.kind == H::kEventEndOfRow) { x.pcyc = long(y) * Tline + W; ++y; }
        else if (e.kind == H::kEventEndOfFrame) { x.pcyc = long(y) * Tline + W + 1; }
        else {
            x.pcyc = long(y) * Tline + e.x;
            if (e.kind == H::kEventInterior && y < Hh) interior_row[y]++;
        }
        evs.push_back(x);
    }

    // FIFO co-sim, recording per-event occupancy + drop
    const long afull = g_depth - 8;
    auto cost = [&](const Ev& e) -> long {
        if (e.kind == H::kEventEndOfRow)
            return g_ing + ((e.y >= 1 && e.y - 1 < Hh) ? long(g_proc) * interior_row[e.y - 1] + g_scav : 0);
        return g_ing;
    };
    std::deque<int> fifo;
    long be_free = 0;
    std::vector<char> drop(evs.size(), 0);
    std::vector<long> occ(evs.size(), 0);
    std::vector<long> row_peak(Hh, 0);
    std::vector<int> row_drop(Hh, 0);
    for (std::size_t i = 0; i < evs.size(); ++i) {
        const long pc = evs[i].pcyc;
        while (!fifo.empty()) {
            int j = fifo.front();
            long pop_cyc = std::max(be_free, evs[j].pcyc);
            if (pop_cyc > pc) break;
            be_free = pop_cyc + cost(evs[j]);
            fifo.pop_front();
        }
        long o = (long)fifo.size();
        occ[i] = o;
        const bool is_data = evs[i].kind == H::kEventInterior || evs[i].kind == H::kEventEndpoint;
        int ry = evs[i].y;
        if (ry >= 0 && ry < Hh) row_peak[ry] = std::max(row_peak[ry], o);
        if (is_data && o >= afull) {
            drop[i] = 1;
            if (ry >= 0 && ry < Hh) row_drop[ry]++;
        } else fifo.push_back((int)i);
    }

    // ---- buffer map ----
    RGB buf; buf.init(img, 0.32);
    const int GUT = 14;  // left gutter width
    for (int ry = 0; ry < Hh; ++ry) {
        unsigned char r, g, b;
        if (row_drop[ry] > 0) { r = 235; g = 30; b = 30; }
        else rampBlue(double(row_peak[ry]) / double(afull), r, g, b);
        for (int gx = 0; gx < GUT; ++gx) buf.set(gx, ry, r, g, b);
    }
    for (std::size_t i = 0; i < evs.size(); ++i) {
        if (evs[i].kind != H::kEventInterior && evs[i].kind != H::kEventEndpoint) continue;
        int x = evs[i].x, yy = evs[i].y;
        if (x < GUT) x = GUT;  // don't overwrite the gutter
        if (drop[i]) buf.set(x, yy, 235, 30, 30);
        else {
            unsigned char r, g, b; rampBlue(double(occ[i]) / double(afull), r, g, b);
            buf.set(x, yy, r, g, b);
        }
    }
    std::string bpath = g_out + "/" + base + "_buffer.png";
    savePng(bpath, buf);
    if (g_half) saveScaled(g_out + "/" + base + "_buffer_q.png", buf, 4);

    // ---- segment map ----
    std::vector<LineSegment> full = backendSegs(evs, {}, W, Hh, p);
    std::vector<LineSegment> kept = backendSegs(evs, drop, W, Hh, p);
    std::vector<Mid> km; km.reserve(kept.size());
    for (const auto& k : kept) km.push_back(midOf(k));
    std::vector<char> used(kept.size(), 0);
    long survived = 0;
    RGB seg; seg.init(img, 0.32);
    // draw lost (red) first, then survived (green) on top
    std::vector<char> is_surv(full.size(), 0);
    for (std::size_t fi = 0; fi < full.size(); ++fi) {
        Mid fm = midOf(full[fi]);
        int best = -1; double bestd = 7.0;  // midpoint tol px
        for (std::size_t ki = 0; ki < kept.size(); ++ki) {
            if (used[ki]) continue;
            double da = std::fabs(fm.ang - km[ki].ang);
            if (da > M_PI) da = 2 * M_PI - da;
            if (da > 0.14) continue;  // ~8 deg
            double d = std::hypot(fm.cx - km[ki].cx, fm.cy - km[ki].cy);
            if (d < bestd) { bestd = d; best = (int)ki; }
        }
        if (best >= 0) { used[best] = 1; is_surv[fi] = 1; ++survived; }
    }
    for (std::size_t fi = 0; fi < full.size(); ++fi)
        if (!is_surv[fi])
            thickLine(seg, full[fi].x0, full[fi].y0, full[fi].x1, full[fi].y1, 235, 40, 40, 1);
    for (std::size_t fi = 0; fi < full.size(); ++fi)
        if (is_surv[fi])
            thickLine(seg, full[fi].x0, full[fi].y0, full[fi].x1, full[fi].y1, 40, 220, 40, 1);
    std::string spath = g_out + "/" + base + "_segs.png";
    savePng(spath, seg);
    if (g_half) saveScaled(g_out + "/" + base + "_segs_q.png", seg, 4);

    long dropped = 0; for (char d : drop) dropped += d;
    std::printf("%-14s  full=%zu kept=%zu lost=%zu (%.1f%%)  dropped=%ld/%zu  -> %s , %s\n",
                base.c_str(), full.size(), kept.size(), full.size() - survived,
                full.empty() ? 0.0 : 100.0 * double(full.size() - survived) / double(full.size()),
                dropped, evs.size(), bpath.c_str(), spath.c_str());
}

}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> imgs;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto nx = [&]() { return std::atoi(argv[++i]); };
        if (a == "--out" && i + 1 < argc) { g_out = argv[++i]; continue; }
        if (a == "--hblank") { g_hblank = nx(); continue; }
        if (a == "--ing") { g_ing = nx(); continue; }
        if (a == "--proc") { g_proc = nx(); continue; }
        if (a == "--scav") { g_scav = nx(); continue; }
        if (a == "--depth") { g_depth = nx(); continue; }
        if (a == "--half") { g_half = true; continue; }
        imgs.push_back(a);
    }
    if (imgs.empty()) {
        std::fprintf(stderr, "usage: fifo_dropviz --out DIR <img...> [--half --hblank N "
                             "--ing N --proc N --scav N --depth N]\n");
        return 2;
    }
    const Params p = Params::improved();
    for (const std::string& path : imgs) process(path, p);
    return 0;
}
