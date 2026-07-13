// Quantitative evaluation of SweepLSD vs LSD vs EDLines on SYNTHETIC images with
// exact ground truth.
//
// The comparison tool (app/compare.cpp) only reports counts and timing. This
// tool fills the missing quality axis with a fair, controlled protocol:
//
//   * Synthetic ground truth — we draw known line segments (anti-aliased) onto
//     a canvas, so we know the exact GT. Orientations are sampled uniformly in
//     [0,180) to avoid biasing the axis-aligned methods (SweepLSD quantises the
//     gradient direction to H/V, so diagonal lines must be represented). The
//     geometry is fixed per image index and only Gaussian noise sigma varies,
//     isolating the effect of noise.
//
//   * Fair operating points — a single threshold comparison is unfair (one
//     method tuned aggressive, another conservative), so we sweep each method's
//     principal sensitivity knob and trace its precision/recall FRONTIER:
//         SweepLSD  : pixel_num_th  (min pixels per segment)
//         LSD    : eps           (the -log10(NFA) detection threshold)
//         EDLines: min_length    (shortest accepted chain)
//     and summarise with F-max and AP (area under the PR frontier).
//
//   * Matching — a detection matches a GT segment when their directions agree
//     (<= angle_th), the detection lies laterally within tau of the GT line,
//     and their projections overlap; matched greedily one-to-one by overlap.
//     TP = matched detections, FP = the rest, FN = unmatched GT.
//
//   * Geometric accuracy — for matched pairs we report the lateral
//     localization error and the angular error (robust to fragmentation).
//
// Speed/ISA fairness is handled by compare.cpp (all methods built at AVX2); the
// focus here is detection quality, so timing is reported only for context.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "edlines.hpp"
#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"
#include "edreal_io.hpp"
#include "mlsd_io.hpp"

#include "lsd.h"  // third_party/lsd (AGPL)

namespace {

constexpr double kPi = 3.14159265358979323846;
using sweeplsd::LineSegment;

// ---------------------------------------------------------------------------
// Segment geometry
// ---------------------------------------------------------------------------

double segLength(const LineSegment& s) {
    double dx = s.x1 - s.x0, dy = s.y1 - s.y0;
    return std::sqrt(dx * dx + dy * dy);
}

// Undirected orientation in [0, pi).
double segOrient(const LineSegment& s) {
    double a = std::atan2(s.y1 - s.y0, s.x1 - s.x0);
    if (a < 0) a += kPi;
    if (a >= kPi) a -= kPi;
    return a;
}

// Smallest angle between two undirected orientations, in [0, pi/2].
double orientDiff(double a, double b) {
    double d = std::fabs(a - b);
    if (d > kPi / 2) d = kPi - d;
    return d;
}

// Distance from point p to the infinite line through segment g, and the
// projection parameter t of p along g's unit direction (origin at g.p0).
void pointToLine(const LineSegment& g, double px, double py, double& perp, double& t) {
    double ux = g.x1 - g.x0, uy = g.y1 - g.y0;
    double len = std::sqrt(ux * ux + uy * uy);
    if (len < 1e-9) { perp = std::hypot(px - g.x0, py - g.y0); t = 0; return; }
    ux /= len; uy /= len;
    double vx = px - g.x0, vy = py - g.y0;
    t = vx * ux + vy * uy;
    perp = std::fabs(vx * uy - vy * ux);  // |cross|
}

// ---------------------------------------------------------------------------
// Matching and metrics
// ---------------------------------------------------------------------------

struct Stats {
    long tp = 0, fp = 0, fn = 0;
    double sum_lat = 0, sum_ang = 0;  // accumulated over matched pairs
    long n_matched = 0;
    void add(const Stats& o) {
        tp += o.tp; fp += o.fp; fn += o.fn;
        sum_lat += o.sum_lat; sum_ang += o.sum_ang; n_matched += o.n_matched;
    }
    double precision() const { return (tp + fp) ? double(tp) / (tp + fp) : 0.0; }
    double recall() const { return (tp + fn) ? double(tp) / (tp + fn) : 0.0; }
    double f1() const {
        double p = precision(), r = recall();
        return (p + r) > 0 ? 2 * p * r / (p + r) : 0.0;
    }
    double latErr() const { return n_matched ? sum_lat / n_matched : 0.0; }
    double angErrDeg() const { return n_matched ? sum_ang / n_matched * 180.0 / kPi : 0.0; }
};

// Greedy one-to-one matching of detections to GT.
Stats matchSegments(const std::vector<LineSegment>& gt,
                    const std::vector<LineSegment>& det, double tau, double ang_th) {
    struct Cand { double overlap; int gi, di; double lat, ang; };
    std::vector<Cand> cands;
    for (int di = 0; di < (int)det.size(); ++di) {
        const LineSegment& d = det[di];
        double od = segOrient(d), ld = segLength(d);
        for (int gi = 0; gi < (int)gt.size(); ++gi) {
            const LineSegment& g = gt[gi];
            double ang = orientDiff(od, segOrient(g));
            if (ang > ang_th) continue;
            double perp0, t0, perp1, t1;
            pointToLine(g, d.x0, d.y0, perp0, t0);
            pointToLine(g, d.x1, d.y1, perp1, t1);
            if (perp0 > tau || perp1 > tau) continue;
            double lg = segLength(g);
            double lo = std::min(t0, t1), hi = std::max(t0, t1);
            double overlap = std::min(hi, lg) - std::max(lo, 0.0);
            if (overlap <= 0 || overlap < 0.5 * ld) continue;  // detection mostly on GT
            cands.push_back({overlap, gi, di, 0.5 * (perp0 + perp1), ang});
        }
    }
    std::sort(cands.begin(), cands.end(),
              [](const Cand& a, const Cand& b) { return a.overlap > b.overlap; });
    std::vector<char> g_used(gt.size(), 0), d_used(det.size(), 0);
    Stats s;
    for (const Cand& c : cands) {
        if (g_used[c.gi] || d_used[c.di]) continue;
        g_used[c.gi] = d_used[c.di] = 1;
        ++s.tp; ++s.n_matched;
        s.sum_lat += c.lat; s.sum_ang += c.ang;
    }
    s.fp = (long)det.size() - s.tp;
    s.fn = (long)gt.size() - s.tp;
    return s;
}

// ---------------------------------------------------------------------------
// Synthetic image generation (exact ground truth)
// ---------------------------------------------------------------------------

constexpr double kBg = 210.0, kFg = 40.0;  // dark bars on a light background
constexpr double kBarWidth = 3.0;          // bar width; each bar = TWO edges (its flanks)

double distToSegment(double px, double py, const LineSegment& s) {
    double vx = s.x1 - s.x0, vy = s.y1 - s.y0;
    double wx = px - s.x0, wy = py - s.y0;
    double c1 = vx * wx + vy * wy;
    double c2 = vx * vx + vy * vy;
    double t = c2 > 1e-9 ? c1 / c2 : 0.0;
    t = std::max(0.0, std::min(1.0, t));
    double dx = px - (s.x0 + t * vx), dy = py - (s.y0 + t * vy);
    return std::sqrt(dx * dx + dy * dy);
}

// A clean float canvas (light background, dark bars). Each bar is a filled
// rectangle of width kBarWidth, so it presents TWO parallel intensity edges
// (its flanks) — which is what an edge-based detector actually finds. The
// ground truth is therefore the two flank lines of every bar. Geometry depends
// only on `seed`. Returns the bar centerlines via `bars` and the GT flanks via
// `gt`.
std::vector<float> genClean(int w, int h, int n_seg, unsigned seed,
                            std::vector<LineSegment>& gt,
                            std::vector<LineSegment>& bars) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> uAng(0.0, kPi);
    std::uniform_real_distribution<double> uLen(40.0, 200.0);
    std::uniform_real_distribution<double> uX(20.0, w - 20.0);
    std::uniform_real_distribution<double> uY(20.0, h - 20.0);

    bars.clear();
    gt.clear();
    const double off = 0.5 * kBarWidth;
    for (int k = 0; k < n_seg; ++k) {
        for (int tries = 0; tries < 20; ++tries) {
            double a = uAng(rng), len = uLen(rng);
            double cx = uX(rng), cy = uY(rng);
            double hx = 0.5 * len * std::cos(a), hy = 0.5 * len * std::sin(a);
            LineSegment s{float(cx - hx), float(cy - hy), float(cx + hx), float(cy + hy)};
            if (s.x0 < 5 || s.y0 < 5 || s.x1 < 5 || s.y1 < 5 ||
                s.x0 > w - 5 || s.y0 > h - 5 || s.x1 > w - 5 || s.y1 > h - 5)
                continue;
            bars.push_back(s);
            // Two flank edges, offset along the bar normal.
            double nx = -std::sin(a), ny = std::cos(a);
            gt.push_back({float(s.x0 + off * nx), float(s.y0 + off * ny),
                          float(s.x1 + off * nx), float(s.y1 + off * ny)});
            gt.push_back({float(s.x0 - off * nx), float(s.y0 - off * ny),
                          float(s.x1 - off * nx), float(s.y1 - off * ny)});
            break;
        }
    }

    std::vector<float> buf(std::size_t(w) * h, float(kBg));
    for (const LineSegment& s : bars) {
        int xmin = std::max(0, int(std::floor(std::min(s.x0, s.x1))) - 3);
        int xmax = std::min(w - 1, int(std::ceil(std::max(s.x0, s.x1))) + 3);
        int ymin = std::max(0, int(std::floor(std::min(s.y0, s.y1))) - 3);
        int ymax = std::min(h - 1, int(std::ceil(std::max(s.y0, s.y1))) + 3);
        for (int y = ymin; y <= ymax; ++y)
            for (int x = xmin; x <= xmax; ++x) {
                // Sample pixel (x,y) AT (x,y): the canvas, the GT lines and the
                // detectors all share the pixel-centre coordinate frame (canonical
                // LSD also reports in this frame after its own +0.5 correction).
                // Sampling at (x+0.5, y+0.5) would shift the rendered edges half a
                // pixel away from the GT lines and charge every detector a
                // spurious 0.5 px of lateral error.
                double d = distToSegment(double(x), double(y), s);
                double cov = std::max(0.0, std::min(1.0, off + 0.5 - d));  // filled bar, AA edges
                if (cov <= 0) continue;
                float& px = buf[std::size_t(y) * w + x];
                float v = float(kBg + (kFg - kBg) * cov);
                if (v < px) px = v;  // darkest wins (dark bar on light bg)
            }
    }
    return buf;
}

// Add Gaussian noise and quantise to 8-bit.
sweeplsd::GrayImage addNoise(const std::vector<float>& clean, int w, int h,
                          double sigma, unsigned seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, sigma);
    sweeplsd::GrayImage img(w, h);
    for (std::size_t i = 0; i < clean.size(); ++i) {
        double v = clean[i] + (sigma > 0 ? noise(rng) : 0.0);
        img.data[i] = std::uint8_t(std::max(0.0, std::min(255.0, v)) + 0.5);
    }
    return img;
}

// ---------------------------------------------------------------------------
// Method runners (each with its swept sensitivity knob)
// ---------------------------------------------------------------------------

std::vector<LineSegment> runSweeplsd(const sweeplsd::GrayImage& src, int pixel_num_th) {
    sweeplsd::Params p = sweeplsd::Params::original2014();
    p.pixel_num_th = pixel_num_th;
    return sweeplsd::detect(src, p);
}

std::vector<LineSegment> runSweeplsdImproved(const sweeplsd::GrayImage& src, int pixel_num_th) {
    sweeplsd::Params p = sweeplsd::Params::improved();
    p.pixel_num_th = pixel_num_th;
    return sweeplsd::detect(src, p);
}

std::vector<LineSegment> runSweeplsdImprovedLink(const sweeplsd::GrayImage& src, int pixel_num_th) {
    sweeplsd::Params p = sweeplsd::Params::improved();
    p.pixel_num_th = pixel_num_th;
    p.link_collinear = true;  // linker defaults: lateral tol, gap, two-stage admit
    return sweeplsd::detect(src, p);
}

std::vector<LineSegment> runLsdEps(const sweeplsd::GrayImage& src, double eps) {
    std::vector<double> buf(std::size_t(src.width) * src.height);
    for (int i = 0; i < src.width * src.height; ++i) buf[i] = double(src.data[i]);
    int n = 0;
    // Standard LSD defaults, varying only log_eps (the NFA detection threshold).
    double* out = LineSegmentDetection(&n, buf.data(), src.width, src.height,
                                       0.8, 0.6, 2.0, 22.5, eps, 0.7, 1024,
                                       NULL, NULL, NULL);
    std::vector<LineSegment> segs;
    segs.reserve(n);
    for (int j = 0; j < n; ++j)
        segs.push_back({(float)out[7 * j], (float)out[7 * j + 1],
                        (float)out[7 * j + 2], (float)out[7 * j + 3]});
    std::free(out);
    return segs;
}

std::vector<LineSegment> runEd(const sweeplsd::GrayImage& src, int min_length) {
    edlines::Params p;
    p.min_length = min_length;
    return edlines::detect(src, p);
}

// ---------------------------------------------------------------------------
// PR frontier
// ---------------------------------------------------------------------------

struct PrPoint { double knob, recall, precision, f1, lat, ang; };
struct Curve {
    std::string method, color;
    std::vector<PrPoint> pts;  // one per knob value
    double fmax = 0, ap = 0;
    PrPoint best{};  // operating point with max F
};

double areaUnderPr(std::vector<PrPoint> pts) {
    std::sort(pts.begin(), pts.end(),
              [](const PrPoint& a, const PrPoint& b) { return a.recall < b.recall; });
    double ap = 0, prevR = 0;
    for (const PrPoint& p : pts) {
        ap += (p.recall - prevR) * p.precision;
        prevR = p.recall;
    }
    return ap;
}

void finalizeCurve(Curve& c) {
    c.fmax = 0;
    for (const PrPoint& p : c.pts)
        if (p.f1 > c.fmax) { c.fmax = p.f1; c.best = p; }
    c.ap = areaUnderPr(c.pts);
}

// ---------------------------------------------------------------------------
// HTML report
// ---------------------------------------------------------------------------

// Map (recall, precision) in [0,1]^2 to SVG plot coordinates.
struct Plot { double x0, y0, w, h; };
std::string svgXY(const Plot& pl, double r, double p) {
    char b[64];
    std::snprintf(b, sizeof(b), "%.1f,%.1f", pl.x0 + r * pl.w, pl.y0 + (1 - p) * pl.h);
    return b;
}

std::string prCurveSvg(const std::vector<Curve>& curves, const std::string& title) {
    Plot pl{55, 20, 360, 300};
    std::string s;
    s += "<svg viewBox=\"0 0 460 380\" width=\"460\" height=\"380\">";
    s += "<rect x=\"55\" y=\"20\" width=\"360\" height=\"300\" fill=\"#fff\" stroke=\"#ccc\"/>";
    // gridlines + ticks
    for (int i = 0; i <= 5; ++i) {
        double f = i / 5.0;
        char g[512];
        std::snprintf(g, sizeof(g),
            "<line x1=\"%.0f\" y1=\"20\" x2=\"%.0f\" y2=\"320\" stroke=\"#eee\"/>"
            "<line x1=\"55\" y1=\"%.0f\" x2=\"415\" y2=\"%.0f\" stroke=\"#eee\"/>"
            "<text x=\"%.0f\" y=\"335\" font-size=\"11\" text-anchor=\"middle\" fill=\"#666\">%.1f</text>"
            "<text x=\"48\" y=\"%.0f\" font-size=\"11\" text-anchor=\"end\" fill=\"#666\">%.1f</text>",
            pl.x0 + f * pl.w, pl.x0 + f * pl.w, pl.y0 + f * pl.h, pl.y0 + f * pl.h,
            pl.x0 + f * pl.w, f, pl.y0 + (1 - f) * pl.h + 4, f);
        s += g;
    }
    s += "<text x=\"235\" y=\"358\" font-size=\"13\" text-anchor=\"middle\">Recall</text>";
    s += "<text x=\"16\" y=\"170\" font-size=\"13\" text-anchor=\"middle\" transform=\"rotate(-90 16 170)\">Precision</text>";
    s += "<text x=\"235\" y=\"14\" font-size=\"13\" text-anchor=\"middle\" font-weight=\"600\">" + title + "</text>";
    for (const Curve& c : curves) {
        std::vector<PrPoint> pts = c.pts;
        std::sort(pts.begin(), pts.end(),
                  [](const PrPoint& a, const PrPoint& b) { return a.recall < b.recall; });
        std::string poly;
        for (const PrPoint& p : pts) poly += svgXY(pl, p.recall, p.precision) + " ";
        s += "<polyline points=\"" + poly + "\" fill=\"none\" stroke=\"" + c.color + "\" stroke-width=\"2\"/>";
        for (const PrPoint& p : pts) {
            std::string xy = svgXY(pl, p.recall, p.precision);
            s += "<circle cx=\"" + xy.substr(0, xy.find(',')) + "\" cy=\"" +
                 xy.substr(xy.find(',') + 1) + "\" r=\"2.5\" fill=\"" + c.color + "\"/>";
        }
    }
    // legend
    double ly = 30;
    for (const Curve& c : curves) {
        char lg[256];
        std::snprintf(lg, sizeof(lg),
            "<line x1=\"320\" y1=\"%.0f\" x2=\"345\" y2=\"%.0f\" stroke=\"%s\" stroke-width=\"3\"/>"
            "<text x=\"350\" y=\"%.0f\" font-size=\"12\" fill=\"#333\">%s</text>",
            ly, ly, c.color.c_str(), ly + 4, c.method.c_str());
        s += lg;
        ly += 18;
    }
    s += "</svg>";
    return s;
}

}  // namespace

int main(int argc, char** argv) {
    int w = 1280, h = 720, n_seg = 18, images = 4;
    std::string html_path, assets_dir, dump_dir, mlsd_dir, edreal_dir, elsed_dir;
    std::vector<double> sigmas = {0, 5, 10, 20};
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--html" && i + 1 < argc) html_path = argv[++i];
        else if (a == "--assets" && i + 1 < argc) assets_dir = argv[++i];
        else if (a == "--images" && i + 1 < argc) images = std::atoi(argv[++i]);
        else if (a == "--size" && i + 2 < argc) { w = std::atoi(argv[++i]); h = std::atoi(argv[++i]); }
        else if (a == "--nseg" && i + 1 < argc) n_seg = std::atoi(argv[++i]);
        // --dump-images DIR : write each (sigma,image) input PNG so M-LSD can be
        //   run on the exact same synthetic inputs offline (Python).
        else if (a == "--dump-images" && i + 1 < argc) dump_dir = argv[++i];
        // --mlsd-dir DIR : ingest precomputed M-LSD segments per (sigma,image,knob)
        //   as files "eval_s<sig>_im<im>_k<ki>.txt", scored by the SAME matcher.
        else if (a == "--mlsd-dir" && i + 1 < argc) mlsd_dir = argv[++i];
        // --edreal-dir DIR : ingest genuine ED_Lib EDLines segments per
        //   (sigma,image,knob) as "eval_s<sig>_im<im>_k<ki>.txt" (see
        //   tools/edlines_real.exe --minlen), scored by the SAME matcher.
        else if (a == "--edreal-dir" && i + 1 < argc) edreal_dir = argv[++i];
        // --elsed-dir DIR : ingest genuine ELSED segments per (sigma,image,knob)
        //   as "eval_s<sig>_im<im>_k<ki>.txt" (elsed_runner --minlen sweep over
        //   the same values as the EDLines minlen knob), same matcher.
        else if (a == "--elsed-dir" && i + 1 < argc) elsed_dir = argv[++i];
    }
    const bool have_mlsd = !mlsd_dir.empty();
    const bool have_edreal = !edreal_dir.empty();
    const bool have_elsed = !elsed_dir.empty();
    // M-LSD score-threshold sweep (its principal sensitivity knob; lower = more
    // detections). Must match the values mlsd_runner.py is invoked with.
    const std::vector<double> mlsd_knobs = {0.30, 0.20, 0.10, 0.05, 0.02};
    auto sigTag = [](double s) { char b[16]; std::snprintf(b, sizeof(b), "%d", (int)s); return std::string(b); };

    const double tau = 2.0;                  // lateral matching tolerance (px); < bar width so
                                             // a flank detection matches its own edge, not the other
    const double ang_th = 10.0 * kPi / 180;  // angular matching tolerance
    const std::vector<int> sweeplsd_knobs = {64, 48, 32, 24, 16, 12, 8, 5};
    const std::vector<double> lsd_knobs = {4, 3, 2, 1, 0, -1, -2};
    const std::vector<int> ed_knobs = {40, 30, 20, 15, 10, 7, 5};

    std::printf("Synthetic evaluation: %dx%d, %d segments/image, %d images/condition\n",
                w, h, n_seg, images);
    std::printf("Matching: lateral<=%.1fpx, angle<=%.1fdeg\n\n", tau, ang_th * 180 / kPi);

    // Pre-generate clean canvases + GT (geometry fixed across noise conditions).
    std::vector<std::vector<float>> cleans(images);
    std::vector<std::vector<LineSegment>> gts(images);
    std::vector<std::vector<LineSegment>> bars(images);
    for (int im = 0; im < images; ++im)
        cleans[im] = genClean(w, h, n_seg, 1000u + im, gts[im], bars[im]);

    struct Condition { double sigma; std::vector<Curve> curves; };
    std::vector<Condition> conditions;

    for (double sigma : sigmas) {
        // Accumulate stats per (method, knob) across all images (micro-average).
        std::vector<Stats> acc_sweeplsd(sweeplsd_knobs.size());
        std::vector<Stats> acc_sweeplsd_imp(sweeplsd_knobs.size());
        std::vector<Stats> acc_sweeplsd_implink(sweeplsd_knobs.size());
        std::vector<Stats> acc_lsd(lsd_knobs.size());
        std::vector<Stats> acc_ed(ed_knobs.size());
        std::vector<Stats> acc_mlsd(mlsd_knobs.size());
        std::vector<Stats> acc_edreal(ed_knobs.size());
        std::vector<Stats> acc_elsed(ed_knobs.size());
        // ELSED aborts (upstream assert) below minLineLen ~7-10, so some knob
        // files may be absent; a knob joins the curve only if every image of
        // the condition produced a result.
        std::vector<int> elsed_got(ed_knobs.size(), 0);

        for (int im = 0; im < images; ++im) {
            sweeplsd::GrayImage img = addNoise(cleans[im], w, h, sigma, 7000u + im);
            const std::vector<LineSegment>& gt = gts[im];
            if (!dump_dir.empty())
                sweeplsd::saveGrayPng(dump_dir + "/eval_s" + sigTag(sigma) + "_im" +
                                   std::to_string(im) + ".png", img);
            for (std::size_t k = 0; k < sweeplsd_knobs.size(); ++k)
                acc_sweeplsd[k].add(matchSegments(gt, runSweeplsd(img, sweeplsd_knobs[k]), tau, ang_th));
            for (std::size_t k = 0; k < sweeplsd_knobs.size(); ++k)
                acc_sweeplsd_imp[k].add(matchSegments(gt, runSweeplsdImproved(img, sweeplsd_knobs[k]), tau, ang_th));
            for (std::size_t k = 0; k < sweeplsd_knobs.size(); ++k)
                acc_sweeplsd_implink[k].add(matchSegments(gt, runSweeplsdImprovedLink(img, sweeplsd_knobs[k]), tau, ang_th));
            for (std::size_t k = 0; k < lsd_knobs.size(); ++k)
                acc_lsd[k].add(matchSegments(gt, runLsdEps(img, lsd_knobs[k]), tau, ang_th));
            for (std::size_t k = 0; k < ed_knobs.size(); ++k)
                acc_ed[k].add(matchSegments(gt, runEd(img, ed_knobs[k]), tau, ang_th));
            if (have_mlsd)
                for (std::size_t k = 0; k < mlsd_knobs.size(); ++k) {
                    std::vector<LineSegment> ml;
                    readMlsdFile(mlsd_dir + "/eval_s" + sigTag(sigma) + "_im" +
                                 std::to_string(im) + "_k" + std::to_string(k) + ".txt", ml);
                    acc_mlsd[k].add(matchSegments(gt, ml, tau, ang_th));
                }
            if (have_edreal)
                for (std::size_t k = 0; k < ed_knobs.size(); ++k) {
                    std::vector<LineSegment> er;
                    readEdRealFile(edreal_dir + "/eval_s" + sigTag(sigma) + "_im" +
                                   std::to_string(im) + "_k" + std::to_string(k) + ".txt", er);
                    acc_edreal[k].add(matchSegments(gt, er, tau, ang_th));
                }
            if (have_elsed)
                for (std::size_t k = 0; k < ed_knobs.size(); ++k) {
                    std::vector<LineSegment> el;
                    if (readEdRealFile(elsed_dir + "/eval_s" + sigTag(sigma) + "_im" +
                                       std::to_string(im) + "_k" + std::to_string(k) + ".txt", el)) {
                        acc_elsed[k].add(matchSegments(gt, el, tau, ang_th));
                        ++elsed_got[k];
                    }
                }
        }

        auto buildCurve = [&](const std::string& name, const std::string& color,
                              const std::vector<Stats>& acc, auto& knobs) {
            Curve c; c.method = name; c.color = color;
            for (std::size_t k = 0; k < acc.size(); ++k)
                c.pts.push_back({double(knobs[k]), acc[k].recall(), acc[k].precision(),
                                 acc[k].f1(), acc[k].latErr(), acc[k].angErrDeg()});
            finalizeCurve(c);
            return c;
        };
        Condition cond;
        cond.sigma = sigma;
        cond.curves.push_back(buildCurve("SweepLSD", "#1769aa", acc_sweeplsd, sweeplsd_knobs));
        cond.curves.push_back(buildCurve("SweepLSD-improved", "#7b2d8b", acc_sweeplsd_imp, sweeplsd_knobs));
        cond.curves.push_back(buildCurve("SweepLSD-imp+link", "#c0497a", acc_sweeplsd_implink, sweeplsd_knobs));
        cond.curves.push_back(buildCurve("LSD", "#e8820c", acc_lsd, lsd_knobs));
        if (have_edreal)
            cond.curves.push_back(buildCurve("EDLines (ED_Lib)", "#2e9e4f", acc_edreal, ed_knobs));
        if (have_elsed) {
            std::vector<Stats> acc_f;
            std::vector<int> knobs_f;
            for (std::size_t k = 0; k < ed_knobs.size(); ++k)
                if (elsed_got[k] == images) { acc_f.push_back(acc_elsed[k]); knobs_f.push_back(ed_knobs[k]); }
            if (!acc_f.empty())
                cond.curves.push_back(buildCurve("ELSED", "#b8860b", acc_f, knobs_f));
        }
        cond.curves.push_back(buildCurve("EDLines-style", "#6abf8a", acc_ed, ed_knobs));
        if (have_mlsd)
            cond.curves.push_back(buildCurve("M-LSD", "#d62878", acc_mlsd, mlsd_knobs));

        std::printf("noise sigma = %.0f\n", sigma);
        std::printf("  %-14s %6s %6s %7s %7s %9s %8s\n",
                    "method", "F-max", "AP", "P@best", "R@best", "latErr", "angErr");
        for (const Curve& c : cond.curves)
            std::printf("  %-14s %6.3f %6.3f %7.3f %7.3f %7.2fpx %6.2fdeg\n",
                        c.method.c_str(), c.fmax, c.ap, c.best.precision, c.best.recall,
                        c.best.lat, c.best.ang);
        std::printf("\n");
        conditions.push_back(std::move(cond));
    }

    // --- HTML report (sample gallery at sigma=10, PR curves per condition) ---
    if (!html_path.empty() && !assets_dir.empty()) {
        // Representative sample image (image 0 at sigma=10) + overlays.
        double rep_sigma = 10;
        sweeplsd::GrayImage rep = addNoise(cleans[0], w, h, rep_sigma, 7000u);
        sweeplsd::saveSegmentVisualization(assets_dir + "/eval_src.png", rep, {});
        sweeplsd::saveSegmentVisualization(assets_dir + "/eval_gt.png", rep, gts[0]);
        sweeplsd::saveSegmentVisualization(assets_dir + "/eval_sweeplsd.png", rep, runSweeplsd(rep, 16));
        sweeplsd::saveSegmentVisualization(assets_dir + "/eval_sweeplsd_improved.png", rep, runSweeplsdImproved(rep, 16));
        sweeplsd::saveSegmentVisualization(assets_dir + "/eval_sweeplsd_implink.png", rep, runSweeplsdImprovedLink(rep, 16));
        sweeplsd::saveSegmentVisualization(assets_dir + "/eval_lsd.png", rep, runLsdEps(rep, 0));
        if (have_edreal) {
            std::vector<LineSegment> er;  // minlen=10 (knob index 4) on s=10, image 0
            readEdRealFile(edreal_dir + "/eval_s10_im0_k4.txt", er);
            sweeplsd::saveSegmentVisualization(assets_dir + "/eval_ed.png", rep, er);
        } else {
            sweeplsd::saveSegmentVisualization(assets_dir + "/eval_ed.png", rep, runEd(rep, 10));
        }
        if (have_mlsd) {
            std::vector<LineSegment> ml;  // score=0.10 (knob index 2) on s=10, image 0
            readMlsdFile(mlsd_dir + "/eval_s10_im0_k2.txt", ml);
            sweeplsd::saveSegmentVisualization(assets_dir + "/eval_mlsd.png", rep, ml);
        }

        std::ofstream o(html_path);
        if (o) {
            // Asset path relative to the HTML file's directory (e.g. html in
            // docs/, assets in docs/assets/eval/  ->  "assets/eval").
            std::string html_dir;
            std::string::size_type hs = html_path.find_last_of("/\\");
            if (hs != std::string::npos) html_dir = html_path.substr(0, hs);
            std::string ar = assets_dir;
            if (!html_dir.empty() && ar.size() > html_dir.size() &&
                ar.compare(0, html_dir.size(), html_dir) == 0 &&
                (ar[html_dir.size()] == '/' || ar[html_dir.size()] == '\\'))
                ar = ar.substr(html_dir.size() + 1);
            for (char& ch : ar) if (ch == '\\') ch = '/';

            o << "<!doctype html><html lang=\"ja\"><head><meta charset=\"utf-8\">"
                 "<title>SweepLSD 定量評価（合成GT）</title><style>"
                 "body{font-family:'Segoe UI',Meiryo,sans-serif;margin:0;background:#f5f6f8;color:#1d2027;line-height:1.7}"
                 ".wrap{max-width:1000px;margin:0 auto;padding:32px 24px 64px}"
                 "h1{font-size:25px;margin:0 0 4px}h2{margin-top:38px;border-bottom:2px solid #d8dbe0;padding-bottom:6px}"
                 "table{border-collapse:collapse;width:100%;background:#fff;box-shadow:0 1px 3px rgba(0,0,0,.08);border-radius:8px;overflow:hidden;margin-top:10px}"
                 "th,td{padding:8px 11px;text-align:right;border-bottom:1px solid #eef0f3}th:first-child,td:first-child{text-align:left}"
                 "thead th{background:#2b3245;color:#fff}.sweeplsd{color:#1769aa;font-weight:600}"
                 ".gallery{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px;margin-top:14px}"
                 ".card{background:#fff;border-radius:8px;padding:9px;box-shadow:0 1px 3px rgba(0,0,0,.08)}"
                 ".card img{width:100%;border-radius:4px;background:#222}.card h3{margin:6px 2px;font-size:14px}"
                 ".curves{display:flex;flex-wrap:wrap;gap:8px;margin-top:12px}"
                 ".note{background:#fff8e1;border-left:4px solid #f5c518;padding:12px 16px;border-radius:4px;font-size:14px}"
                 "code{background:#eceff4;padding:1px 5px;border-radius:4px}</style></head><body><div class=\"wrap\">";

            o << "<h1>SweepLSD の定量評価 — 合成グラウンドトゥルースによる公平な比較</h1>"
                 "<p>既存手法（LSD・EDLines-style）との公平な比較のため、<b>既知の線分を描画した合成画像</b>で "
                 "Precision/Recall・幾何精度を測る。線分の向きは [0,180&deg;) 一様（SweepLSD は勾配方向を H/V に量子化"
                 "するため、斜め線を含めないと不公平）。幾何は固定し、ガウシアンノイズ &sigma; のみ変えてノイズ感度を分離。</p>";

            o << "<h2>公平化のための条件設定</h2><ul>"
                 "<li><b>同一入力</b>：全手法に同一の8bitグレー画像を渡す。</li>"
                 "<li><b>操作点の掃引</b>：単一しきい値の比較は不公平なので、各手法の主感度ノブを振って "
                 "<b>PR フロンティア</b>を描く（SweepLSD=<code>pixel_num_th</code>, LSD=<code>eps</code>(NFA), "
                 "EDLines=<code>min_length</code>）。要約は <b>F-max</b> と <b>AP</b>（PR曲線下面積）。</li>"
                 "<li><b>マッチング</b>：角度差&le;10&deg; かつ横方向距離&le;3px かつ射影が重なる検出を、"
                 "重なり長で1対1貪欲対応。TP=対応した検出, FP=残り, FN=未対応のGT。</li>"
                 "<li><b>幾何精度</b>：対応した対について横方向誤差・角度誤差（分断に頑健）。</li>"
                 "<li><b>速度/ISA</b> は <code>sweeplsd_compare</code> 側で AVX2 整合済み。本ツールは品質に集中。</li>"
                 "</ul>";

            o << "<h2>サンプル（&sigma;=10, 画像0）</h2><div class=\"gallery\">";
            const char* keys[][2] = {{"src", "入力（合成＋ノイズ）"}, {"gt", "Ground Truth"},
                                     {"sweeplsd", "SweepLSD"}, {"lsd", "LSD"}, {"ed", "EDLines-style"}};
            for (auto& kv : keys)
                o << "<div class=\"card\"><img src=\"" << ar << "/eval_" << kv[0]
                  << ".png\"><h3>" << kv[1] << "</h3></div>";
            o << "</div>";

            o << "<h2>PR フロンティア</h2><div class=\"curves\">";
            for (const Condition& c : conditions) {
                char t[64];
                std::snprintf(t, sizeof(t), "sigma = %.0f", c.sigma);
                o << prCurveSvg(c.curves, t);
            }
            o << "</div>";

            o << "<h2>メトリクス（ノイズ条件別, 画像横断のマイクロ平均）</h2>"
                 "<table><thead><tr><th>&sigma;</th><th>手法</th><th>F-max</th><th>AP</th>"
                 "<th>P@best</th><th>R@best</th><th>横方向誤差</th><th>角度誤差</th></tr></thead><tbody>";
            for (const Condition& c : conditions) {
                for (std::size_t i = 0; i < c.curves.size(); ++i) {
                    const Curve& cu = c.curves[i];
                    char row[512];
                    std::snprintf(row, sizeof(row),
                        "<tr>%s<td%s>%s</td><td%s>%.3f</td><td%s>%.3f</td><td>%.3f</td><td>%.3f</td>"
                        "<td>%.2f px</td><td>%.2f&deg;</td></tr>",
                        i == 0 ? ("<td rowspan=\"" + std::to_string(c.curves.size()) + "\">" +
                                  std::to_string((int)c.sigma) + "</td>").c_str() : "",
                        cu.method == "SweepLSD" ? " class=\"sweeplsd\"" : "", cu.method.c_str(),
                        cu.method == "SweepLSD" ? " class=\"sweeplsd\"" : "", cu.fmax,
                        cu.method == "SweepLSD" ? " class=\"sweeplsd\"" : "", cu.ap,
                        cu.best.precision, cu.best.recall, cu.best.lat, cu.best.ang);
                    o << row;
                }
            }
            o << "</tbody></table>";

            o << "<h2>読み方</h2><div class=\"note\">"
                 "<b>F-max</b> は各手法が達成できる最良の精度・再現バランス、<b>AP</b> は操作点全域での総合力。"
                 "<b>横方向誤差・角度誤差</b>は当たった線分の幾何精度（小さいほど正確）。"
                 "LSD/EDLines は a-contrario 検証で高精度寄り、SweepLSD の既定判定はより単純。"
                 "SweepLSD でも <code>--nfa</code>（improvement 4）を有効化すると同様の高精度側に寄せられる。"
                 "合成GTは制御性が高い反面、実写の質感エッジは表現しないため、実画像評価（York Urban 等）は別途必要。"
                 "</div>";
            o << "</div></body></html>";
            std::printf("HTML report: %s\n", html_path.c_str());
        }
    }
    return 0;
}
