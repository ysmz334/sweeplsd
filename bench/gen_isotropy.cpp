// Isotropy probe: run each detector on geometric charts whose edges are, by
// construction, uniformly distributed over all orientations (Siemens star,
// zone plate, concentric circles, angular line fan). A perfectly isotropic
// detector recovers a flat length-weighted orientation histogram; SweepLSD's
// H/V-only gradient quantisation shows deficits near +-45deg (an inherent trait
// of its integer 2x2 operator — a 4-direction NMS was evaluated to fill these
// but dropped, as it fragmented diagonal edges; see the accuracy/speed report).
//
// Outputs, per chart: the chart PNG, a per-method line overlay, a per-method
// polar "rose" of the orientation histogram, and a CoV isotropy metric.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"
#include "edlines.hpp"
#include "lsd.h"
#include "edreal_io.hpp"
#include "mlsd_io.hpp"

using sweeplsd::GrayImage;
using sweeplsd::LineSegment;
static const double PI = 3.14159265358979323846;

// ----------------------------------------------------------------- charts
GrayImage makeSiemensStar(int N, int sectors, double rIn, double rOut) {
    GrayImage g(N, N, 255);
    double cx = N / 2.0, cy = N / 2.0;
    int ss = 4;  // supersample for anti-aliasing
    for (int y = 0; y < N; ++y)
        for (int x = 0; x < N; ++x) {
            double acc = 0;
            for (int sy = 0; sy < ss; ++sy)
                for (int sx = 0; sx < ss; ++sx) {
                    double px = x + (sx + 0.5) / ss - cx, py = y + (sy + 0.5) / ss - cy;
                    double r = std::sqrt(px * px + py * py);
                    if (r < rIn || r > rOut) { acc += 255; continue; }
                    double a = std::atan2(py, px);  // [-pi,pi]
                    double s = (a + PI) / (2 * PI) * sectors;
                    acc += (int(std::floor(s)) & 1) ? 30 : 255;
                }
            g.at(x, y) = (unsigned char)std::lround(acc / (ss * ss));
        }
    return g;
}

GrayImage makeZonePlate(int N, double cycles, double rOut) {
    GrayImage g(N, N, 128);
    double cx = N / 2.0, cy = N / 2.0;
    for (int y = 0; y < N; ++y)
        for (int x = 0; x < N; ++x) {
            double px = x - cx, py = y - cy, r = std::sqrt(px * px + py * py);
            if (r > rOut) { g.at(x, y) = 128; continue; }
            double rn = r / rOut;
            double v = 128 + 127 * std::cos(PI * cycles * rn * rn);
            g.at(x, y) = (unsigned char)std::lround(std::min(255.0, std::max(0.0, v)));
        }
    return g;
}

GrayImage makeConcentric(int N, double spacing, double rOut) {
    GrayImage g(N, N, 255);
    double cx = N / 2.0, cy = N / 2.0;
    int ss = 4;
    for (int y = 0; y < N; ++y)
        for (int x = 0; x < N; ++x) {
            double acc = 0;
            for (int sy = 0; sy < ss; ++sy)
                for (int sx = 0; sx < ss; ++sx) {
                    double px = x + (sx + 0.5) / ss - cx, py = y + (sy + 0.5) / ss - cy;
                    double r = std::sqrt(px * px + py * py);
                    if (r > rOut) { acc += 255; continue; }
                    double ph = std::fmod(r, spacing) / spacing;  // [0,1)
                    acc += (ph < 0.5) ? 30 : 255;                 // equal-width dark/light rings
                }
            g.at(x, y) = (unsigned char)std::lround(acc / (ss * ss));
        }
    return g;
}

GrayImage makeLineFan(int N, int nlines, double rOut) {
    GrayImage g(N, N, 255);
    double cx = N / 2.0, cy = N / 2.0;
    auto draw = [&](double a) {  // a full diameter line through centre at angle a
        double dx = std::cos(a), dy = std::sin(a);
        for (double t = -rOut; t <= rOut; t += 0.25) {
            double fx = cx + t * dx, fy = cy + t * dy;
            int xi = (int)std::lround(fx), yi = (int)std::lround(fy);
            for (int oy = -1; oy <= 1; ++oy)
                for (int ox = -1; ox <= 1; ++ox) {  // ~3px thick
                    int x = xi + ox, y = yi + oy;
                    if (x >= 0 && y >= 0 && x < N && y < N) g.at(x, y) = 30;
                }
        }
    };
    for (int i = 0; i < nlines; ++i) draw(i * PI / nlines);  // angles 0..pi, evenly
    return g;
}

// ------------------------------------------------------------- histogram
static const int K = 36;  // 5-degree bins over [0,180)

std::vector<double> orientationHist(const std::vector<LineSegment>& segs) {
    std::vector<double> h(K, 0.0);
    for (const LineSegment& s : segs) {
        double dx = s.x1 - s.x0, dy = s.y1 - s.y0;
        double len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-6) continue;
        double a = std::atan2(dy, dx);
        if (a < 0) a += PI;
        if (a >= PI) a -= PI;
        int b = (int)(a / (PI / K));
        if (b < 0) b = 0; if (b >= K) b = K - 1;
        h[b] += len;
    }
    return h;
}

// coefficient of variation of the length-weighted histogram (0 = perfectly
// isotropic). Lower is better.
double covOf(const std::vector<double>& h) {
    double sum = 0; for (double v : h) sum += v;
    if (sum <= 0) return 0;
    double mean = sum / K, var = 0;
    for (double v : h) var += (v - mean) * (v - mean);
    var /= K;
    return std::sqrt(var) / mean;
}

// ------------------------------------------------------------- rose plot
void drawRose(const std::string& out_png, const std::vector<double>& h,
              const unsigned char col[3]) {
    int S = 360; double cx = S / 2.0, cy = S / 2.0, Rmax = 150;
    std::vector<unsigned char> rgb(std::size_t(S) * S * 3, 255);
    auto plot = [&](int x, int y, unsigned char r, unsigned char g, unsigned char b, double a) {
        if (x < 0 || y < 0 || x >= S || y >= S) return;
        std::size_t i = (std::size_t(y) * S + x) * 3;
        rgb[i] = (unsigned char)(rgb[i] * (1 - a) + r * a);
        rgb[i + 1] = (unsigned char)(rgb[i + 1] * (1 - a) + g * a);
        rgb[i + 2] = (unsigned char)(rgb[i + 2] * (1 - a) + b * a);
    };
    // grid circles at 25/50/75/100%
    for (double frac : {0.25, 0.5, 0.75, 1.0})
        for (double t = 0; t < 2 * PI; t += 0.004) {
            int x = (int)std::lround(cx + Rmax * frac * std::cos(t));
            int y = (int)std::lround(cy + Rmax * frac * std::sin(t));
            plot(x, y, 200, 205, 212, 1.0);
        }
    // axis spokes every 45deg
    for (int k = 0; k < 8; ++k) {
        double a = k * PI / 4;
        for (double t = 0; t <= Rmax; t += 0.5)
            plot((int)std::lround(cx + t * std::cos(a)), (int)std::lround(cy + t * std::sin(a)),
                 222, 226, 232, 1.0);
    }
    double vmax = 0; for (double v : h) vmax = std::max(vmax, v);
    if (vmax <= 0) vmax = 1;
    // filled wedges, mirrored over 360 (orientation is mod 180)
    double bw = PI / K;  // bin width in the 180-space
    for (int kk = 0; kk < 2 * K; ++kk) {
        double v = h[kk % K] / vmax;
        double r = Rmax * v;
        double a0 = kk * bw;  // note: 2K bins of width bw cover 2*pi
        for (double sub = 0; sub <= bw; sub += bw / 24) {
            double a = a0 + sub;
            for (double t = 0; t <= r; t += 0.5)
                plot((int)std::lround(cx + t * std::cos(a)), (int)std::lround(cy - t * std::sin(a)),
                     col[0], col[1], col[2], 0.55);
        }
    }
    sweeplsd::saveRgbPng(out_png, S, S, rgb);
}

// ------------------------------------------------------------- detectors
std::vector<LineSegment> runLsd(const GrayImage& s) {
    std::vector<double> buf(std::size_t(s.width) * s.height);
    for (int i = 0; i < s.width * s.height; ++i) buf[i] = double(s.data[i]);
    int n = 0;
    double* out = LineSegmentDetection(&n, buf.data(), s.width, s.height,
                                       0.8, 0.6, 2.0, 22.5, 0.0, 0.7, 1024,
                                       NULL, NULL, NULL);
    std::vector<LineSegment> segs;
    segs.reserve(n);
    for (int j = 0; j < n; ++j)
        segs.push_back({(float)out[7 * j], (float)out[7 * j + 1],
                        (float)out[7 * j + 2], (float)out[7 * j + 3]});
    std::free(out);
    return segs;
}

int main(int argc, char** argv) {
    int N = 1024;
    std::string od = ".", mlsd_dir, edreal_dir, elsed_dir;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--mlsd-dir" && i + 1 < argc) mlsd_dir = argv[++i];
        else if (a == "--edreal-dir" && i + 1 < argc) edreal_dir = argv[++i];
        // --elsed-dir DIR : genuine ELSED results per probe, "iso_<key>.txt"
        // (same file format as the EDLines runner).
        else if (a == "--elsed-dir" && i + 1 < argc) elsed_dir = argv[++i];
        else if (!a.empty() && a[0] != '-') od = a;
    }
    double R = N * 0.47;

    struct Chart { std::string key, label; GrayImage img; };
    std::vector<Chart> charts = {
        {"star", "Siemens Star (72 sectors)", makeSiemensStar(N, 72, 36, R)},
        {"zone", "Zone Plate (cos r^2, 26 cyc)", makeZonePlate(N, 26, R)},
        {"circles", "Concentric Circles (16px)", makeConcentric(N, 16, R)},
        {"fan", "Angular Line Fan (48 lines)", makeLineFan(N, 48, R)}};

    sweeplsd::Params imp = sweeplsd::Params::improved();
    sweeplsd::Params implink = sweeplsd::Params::improved();
    implink.link_collinear = true;  // linker defaults (lateral tol, gap, two-stage)

    struct Method { std::string key, label; unsigned char col[3]; };
    std::vector<Method> methods = {
        {"sweeplsd", "SweepLSD baseline", {23, 105, 170}},
        {"sweeplsdimp", "SweepLSD improved", {123, 45, 139}},
        {"sweeplsdimplink", "SweepLSD imp+link", {192, 73, 122}},
        {"lsd", "LSD", {232, 130, 12}},
        {"ed", "EDLines-style", {106, 191, 138}}};
    if (!edreal_dir.empty())
        methods.push_back({"edreal", "EDLines (ED_Lib)", {46, 158, 79}});
    if (!elsed_dir.empty())
        methods.push_back({"elsed", "ELSED", {184, 134, 11}});
    if (!mlsd_dir.empty())
        methods.push_back({"mlsd", "M-LSD", {214, 40, 120}});

    std::printf("isotropy probe: %dx%d, orientation hist K=%d bins (5deg), length-weighted\n", N, N, K);
    std::printf("CoV = coeff. of variation of the histogram (0 = perfectly isotropic; lower better)\n\n");

    for (Chart& c : charts) {
        sweeplsd::saveGrayPng(od + "/iso_" + c.key + "_src.png", c.img);
        std::printf("== %s ==\n", c.label.c_str());
        std::printf("  %-16s %7s %10s %8s\n", "method", "nseg", "totLen", "CoV");
        for (Method& m : methods) {
            std::vector<LineSegment> segs;
            if (m.key == "sweeplsd") segs = sweeplsd::detect(c.img, sweeplsd::Params::original2014());
            else if (m.key == "sweeplsdimp") segs = sweeplsd::detect(c.img, imp);
            else if (m.key == "sweeplsdimplink") segs = sweeplsd::detect(c.img, implink);
            else if (m.key == "lsd") segs = runLsd(c.img);
            else if (m.key == "edreal") readEdRealFile(edreal_dir + "/iso_" + c.key + ".txt", segs);
            else if (m.key == "elsed") readEdRealFile(elsed_dir + "/iso_" + c.key + ".txt", segs);
            else if (m.key == "mlsd") readMlsdFile(mlsd_dir + "/iso_" + c.key + "_mlsd.txt", segs);
            else segs = edlines::detect(c.img);

            sweeplsd::saveSegmentVisualization(od + "/iso_" + c.key + "_" + m.key + ".png", c.img, segs);
            std::vector<double> h = orientationHist(segs);
            drawRose(od + "/iso_" + c.key + "_" + m.key + "_rose.png", h, m.col);
            double tot = 0; for (double v : h) tot += v;
            std::printf("  %-16s %7zu %10.0f %8.3f\n", m.label.c_str(), segs.size(), tot, covOf(h));
        }
        std::printf("\n");
    }
    return 0;
}
