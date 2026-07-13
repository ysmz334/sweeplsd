// Test & benchmark harness for the improved SweepLSD.
//  1. parity:   detect() (multi-pass) vs detectOnePass() must agree EXACTLY for
//               every parameter combination (the one-pass concept's contract).
//  2. accuracy: synthetic bars at angles 0..165 deg + gaussian noise, with
//               known ground-truth edge lines; per-improvement ablation.
//  3. speed:    1920x1080 scene, per-stage profile and end-to-end medians.
#include <algorithm>
#include <chrono>
#include <cmath>

#ifndef M_PI  // not guaranteed by the standard; MSVC needs _USE_MATH_DEFINES
#define M_PI 3.14159265358979323846
#endif
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <tuple>
#include <vector>

#include "sweeplsd/sweeplsd.hpp"

using namespace sweeplsd;

// ---------------------------------------------------------------- synthetic
struct GtLine {  // ground-truth edge line (one flank of a bar)
    double x0, y0, x1, y1;
    double len() const { return std::hypot(x1 - x0, y1 - y0); }
};
struct Scene {
    GrayImage img;
    std::vector<GtLine> gt;
};

// Bars laid out one per grid cell (no overlaps/junctions), angles cycling
// through 0,15,...,165 deg. Each bar of width W contributes its two flank
// edges as ground truth. Soft 1px ramp at the boundary so sub-pixel structure
// is meaningful; gaussian pixel noise on top.
Scene makeScene(int w, int h, int cell, double contrast, double noise_sigma,
                unsigned seed) {
    Scene sc;
    sc.img = GrayImage(w, h, 0);
    const double bg = 95.0;
    std::vector<double> acc(std::size_t(w) * h, bg);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> U(0.0, 1.0);

    const double W = 6.0;           // bar width
    const int margin = 24;
    int angle_idx = 0;
    for (int cy = 0; cy + cell <= h; cy += cell)
        for (int cx = 0; cx + cell <= w; cx += cell) {
            double theta = (15.0 * (angle_idx++ % 12) + (U(rng) - 0.5) * 6.0) * M_PI / 180.0;
            double L = cell - 2.0 * margin - 20.0 + U(rng) * 16.0;
            double mx = cx + cell / 2.0 + (U(rng) - 0.5) * 10.0;
            double my = cy + cell / 2.0 + (U(rng) - 0.5) * 10.0;
            double dx = std::cos(theta), dy = std::sin(theta);
            double nx = -dy, ny = dx;
            // raster the bar with a 1px soft edge
            int x0 = std::max(0, int(mx - L)), x1 = std::min(w - 1, int(mx + L));
            int y0 = std::max(0, int(my - L)), y1 = std::min(h - 1, int(my + L));
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x) {
                    double u = (x - mx) * dx + (y - my) * dy;
                    double v = (x - mx) * nx + (y - my) * ny;
                    double cu = std::min(1.0, std::max(0.0, 0.5 + (L / 2 - std::fabs(u))));
                    double cv = std::min(1.0, std::max(0.0, 0.5 + (W / 2 - std::fabs(v))));
                    double cov = cu * cv;
                    if (cov > 0) {
                        double& a = acc[std::size_t(y) * w + x];
                        a = std::max(a, bg + contrast * cov);
                    }
                }
            // the two flank edges, inset by 2px at the caps (cap wrap-around)
            double in = 2.0;
            for (int s = -1; s <= 1; s += 2) {
                GtLine g;
                g.x0 = mx - dx * (L / 2 - in) + nx * s * W / 2;
                g.y0 = my - dy * (L / 2 - in) + ny * s * W / 2;
                g.x1 = mx + dx * (L / 2 - in) + nx * s * W / 2;
                g.y1 = my + dy * (L / 2 - in) + ny * s * W / 2;
                sc.gt.push_back(g);
            }
        }
    std::normal_distribution<double> N(0.0, noise_sigma);
    for (int i = 0; i < w * h; ++i) {
        double v = acc[i] + N(rng);
        sc.img.data[i] = std::uint8_t(std::min(255.0, std::max(0.0, v)));
    }
    return sc;
}

// ---------------------------------------------------------------- metrics
struct Metrics {
    double recall = 0;      // GT length covered by matched detections
    double precision = 0;   // matched detections / all detections
    double angle_rmse = 0;  // deg, matched detections, length-weighted
    double perp_rmse = 0;   // px, endpoint distance to the GT line
    double n_det = 0;
};

Metrics evaluate(const std::vector<Scene>& scenes,
                 const std::vector<std::vector<LineSegment>>& dets) {
    double gt_total = 0, gt_covered = 0;
    double det_total = 0, det_matched = 0;
    double ang_sq = 0, ang_wt = 0, perp_sq = 0;
    long perp_n = 0;

    for (std::size_t si = 0; si < scenes.size(); ++si) {
        const auto& gt = scenes[si].gt;
        const auto& det = dets[si];
        det_total += det.size();
        // per-GT coverage intervals
        std::vector<std::vector<std::pair<double, double>>> cover(gt.size());
        for (const LineSegment& s : det) {
            double sl = std::hypot(s.x1 - s.x0, s.y1 - s.y0);
            if (sl < 1e-6) continue;
            double sa = std::atan2(s.y1 - s.y0, s.x1 - s.x0);
            bool matched = false;
            for (std::size_t gi = 0; gi < gt.size(); ++gi) {
                const GtLine& g = gt[gi];
                double gl = g.len();
                double ga = std::atan2(g.y1 - g.y0, g.x1 - g.x0);
                double da = std::fabs(sa - ga);
                while (da > M_PI / 2) da = std::fabs(da - M_PI);
                if (da > 6.0 * M_PI / 180.0) continue;
                double gdx = (g.x1 - g.x0) / gl, gdy = (g.y1 - g.y0) / gl;
                auto perp = [&](double px, double py) {
                    return std::fabs((px - g.x0) * (-gdy) + (py - g.y0) * gdx);
                };
                double p0 = perp(s.x0, s.y0), p1 = perp(s.x1, s.y1);
                if (std::max(p0, p1) > 2.5) continue;
                auto t = [&](double px, double py) {
                    return (px - g.x0) * gdx + (py - g.y0) * gdy;
                };
                double t0 = t(s.x0, s.y0), t1 = t(s.x1, s.y1);
                if (t0 > t1) std::swap(t0, t1);
                double ov = std::min(t1, gl) - std::max(t0, 0.0);
                if (ov < 0.5 * sl || ov <= 0) continue;
                // matched
                matched = true;
                cover[gi].push_back({std::max(t0, 0.0), std::min(t1, gl)});
                ang_sq += da * da * sl;
                ang_wt += sl;
                perp_sq += p0 * p0 + p1 * p1;
                perp_n += 2;
                break;
            }
            det_matched += matched;
        }
        for (std::size_t gi = 0; gi < gt.size(); ++gi) {
            double gl = gt[gi].len();
            gt_total += gl;
            auto& iv = cover[gi];
            std::sort(iv.begin(), iv.end());
            double covered = 0, cur_a = -1, cur_b = -1;
            for (auto& p : iv) {
                if (p.first > cur_b) {
                    if (cur_b > cur_a) covered += cur_b - cur_a;
                    cur_a = p.first; cur_b = p.second;
                } else cur_b = std::max(cur_b, p.second);
            }
            if (cur_b > cur_a) covered += cur_b - cur_a;
            gt_covered += covered;
        }
    }
    Metrics m;
    m.recall = gt_total > 0 ? gt_covered / gt_total : 0;
    m.precision = det_total > 0 ? det_matched / det_total : 0;
    m.angle_rmse = ang_wt > 0 ? std::sqrt(ang_sq / ang_wt) * 180.0 / M_PI : 0;
    m.perp_rmse = perp_n > 0 ? std::sqrt(perp_sq / perp_n) : 0;
    m.n_det = det_total / scenes.size();
    return m;
}

// ---------------------------------------------------------------- parity
bool sameSegments(std::vector<LineSegment> a, std::vector<LineSegment> b) {
    if (a.size() != b.size()) return false;
    auto key = [](const LineSegment& s) {
        return std::make_tuple(s.x0, s.y0, s.x1, s.y1);
    };
    std::sort(a.begin(), a.end(), [&](auto& p, auto& q) { return key(p) < key(q); });
    std::sort(b.begin(), b.end(), [&](auto& p, auto& q) { return key(p) < key(q); });
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::memcmp(&a[i], &b[i], sizeof(LineSegment)) != 0) return false;
    return true;
}

// ---------------------------------------------------------------- ppm out
void savePpm(const std::string& path, const GrayImage& img,
             const std::vector<LineSegment>& segs) {
    int w = img.width, h = img.height;
    std::vector<unsigned char> rgb(std::size_t(w) * h * 3);
    for (int i = 0; i < w * h; ++i)
        rgb[i * 3] = rgb[i * 3 + 1] = rgb[i * 3 + 2] = img.data[i] / 2;
    std::srand(0);
    auto draw = [&](int x0, int y0, int x1, int y1, unsigned char r, unsigned char g,
                    unsigned char b) {
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (true) {
            if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h) {
                std::size_t i = (std::size_t(y0) * w + x0) * 3;
                rgb[i] = r; rgb[i + 1] = g; rgb[i + 2] = b;
            }
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    };
    for (const auto& s : segs) {
        static const unsigned char c[6][3] = {{255, 80, 80},  {80, 255, 120}, {90, 160, 255},
                                              {255, 200, 60}, {255, 110, 255}, {90, 240, 240}};
        int hue = std::rand() % 6;
        draw(int(std::lround(s.x0)), int(std::lround(s.y0)), int(std::lround(s.x1)),
             int(std::lround(s.y1)), c[hue][0], c[hue][1], c[hue][2]);
    }
    FILE* f = std::fopen(path.c_str(), "wb");
    std::fprintf(f, "P6\n%d %d\n255\n", w, h);
    std::fwrite(rgb.data(), 1, rgb.size(), f);
    std::fclose(f);
}

// ---------------------------------------------------------------- main
template <class F>
double medianMs(int runs, F&& fn) {
    std::vector<double> t;
    for (int i = 0; i < runs; ++i) {
        auto t0 = std::chrono::steady_clock::now();
        fn();
        auto t1 = std::chrono::steady_clock::now();
        t.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
    std::sort(t.begin(), t.end());
    return t[t.size() / 2];
}

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);  // unbuffered: keep output on crash
    const bool quick = argc > 1 && std::string(argv[1]) == "quick";
    const int quickRuns = quick ? 5 : 9;

    // ---------- 1. parity: multi-pass == one-pass, every config -------------
    {
        std::vector<std::pair<std::string, Params>> cfgs;
        Params base = Params::original2014();
        cfgs.push_back({"baseline", base});
        Params p = base; p.nms_strict_tiebreak = true; cfgs.push_back({"strict", p});
        p = base; p.subpixel_nms = true; cfgs.push_back({"subpix", p});
        p = base; p.use_hysteresis = true; cfgs.push_back({"hyst", p});
        p = base; p.endpoint_from_bbox = true; cfgs.push_back({"bbox", p});
        p = base; p.lattice_half_shift = true; cfgs.push_back({"halfpx", p});
        p = base; p.use_nfa = true; p.nfa_window_rows = 128; cfgs.push_back({"nfa_local", p});
        p = base; p.weight_by_gradient = true; cfgs.push_back({"weight", p});
        p = base; p.link_collinear = true; cfgs.push_back({"link", p});
        cfgs.push_back({"improved", Params::improved()});
        p = Params::improved(); p.sparse_feature_scan = p.sparse_label_scan = false;
        cfgs.push_back({"improved_nosparse", p});

        int fails = 0;
        for (unsigned seed = 1; seed <= 3; ++seed) {
            Scene sc = makeScene(640, 480, 160, 95.0, 8.0, seed);
            for (auto& [name, cfg] : cfgs) {
                auto a = detect(sc.img, cfg);
                auto b = detectOnePass(sc.img, cfg);
                if (!sameSegments(a, b)) {
                    std::printf("PARITY FAIL: %-18s seed=%u  multi=%zu one=%zu\n",
                                name.c_str(), seed, a.size(), b.size());
                    ++fails;
                }
            }
        }
        // sparse scan must not change the output either
        for (unsigned seed = 1; seed <= 3; ++seed) {
            Scene sc = makeScene(640, 480, 160, 95.0, 8.0, seed);
            Params c1 = Params::improved();
            Params c2 = c1; c2.sparse_feature_scan = c2.sparse_label_scan = false;
            if (!sameSegments(detect(sc.img, c1), detect(sc.img, c2))) {
                std::printf("SPARSE-EXACTNESS FAIL seed=%u\n", seed);
                ++fails;
            }
        }
        std::printf("[parity] %s\n\n", fails == 0 ? "all configs: multi-pass == one-pass, "
                                                    "sparse == dense  (OK)"
                                                  : "FAILURES PRESENT");
        if (fails) return 1;
    }

    // ---------- 2. accuracy ablation ----------------------------------------
    auto runAblation = [&](const char* title, double contrast, double sigma, int hyst_low,
                           int n_scenes) {
        std::vector<Scene> scenes;
        for (int s = 0; s < n_scenes; ++s)
            scenes.push_back(makeScene(640, 480, 160, contrast, sigma, 100 + s));

        std::vector<std::pair<std::string, Params>> cfgs;
        Params base = Params::original2014();
        cfgs.push_back({"baseline (orig)", base});
        Params p = base; p.nms_strict_tiebreak = true; cfgs.push_back({"+a strictNMS", p});
        p.subpixel_nms = true; cfgs.push_back({"+c subpixel", p});
        p.use_hysteresis = true; p.hysteresis_low_th = hyst_low;
        cfgs.push_back({"+d hysteresis", p});
        p.endpoint_from_bbox = true; cfgs.push_back({"+f bboxEnds", p});
        p.lattice_half_shift = true; cfgs.push_back({"+j halfpx", p});
        Params wq = p; wq.weight_by_gradient = true;
        cfgs.push_back({"(+3 gradWeight)", wq});  // side-test, not in the chain
        Params q = p; q.use_nfa = true; q.nfa_window_rows = 128;
        cfgs.push_back({"+g localNFA", q});
        Params r = q; r.link_collinear = true;
        cfgs.push_back({"+link (imp.5)", r});

        std::printf("== %s (contrast=%.0f, noise sigma=%.0f, %d scenes, %zu GT lines) ==\n",
                    title, contrast, sigma, n_scenes, scenes.size() * scenes[0].gt.size() /
                                                          scenes.size() * scenes.size());
        std::printf("%-18s %7s %9s %10s %9s %7s\n", "config", "recall", "precision",
                    "angRMSE", "perpRMSE", "#det");
        for (auto& [name, cfg] : cfgs) {
            std::vector<std::vector<LineSegment>> dets;
            for (auto& sc : scenes) dets.push_back(detect(sc.img, cfg));
            Metrics m = evaluate(scenes, dets);
            std::printf("%-18s %6.1f%% %8.1f%% %9.3f° %7.3fpx %7.1f\n", name.c_str(),
                        m.recall * 100, m.precision * 100, m.angle_rmse, m.perp_rmse, m.n_det);
        }
        std::printf("\n");
        return scenes;
    };

    int n_sc = quick ? 4 : 12;
    auto scenes_hi = runAblation("high contrast", 95.0, 8.0, 120, n_sc);
    runAblation("low contrast (hysteresis target)", 13.0, 3.0, 120, n_sc);

    // visualization of one scene, baseline vs improved
    {
        Params imp = Params::improved();
        savePpm("out_baseline.ppm", scenes_hi[0].img,
                detect(scenes_hi[0].img, Params::original2014()));
        savePpm("out_improved.ppm", scenes_hi[0].img,
                detect(scenes_hi[0].img, imp));
    }

    // ---------- 2.5 robustness: degenerate sizes -----------------------------
    {
        int fails = 0;
        for (auto [tw, th2] : {std::pair<int,int>{0,0},{1,1},{3,2},{2,7},{5,5},{7,7},{8,1}}) {
            GrayImage im(tw, th2, 128);
            for (auto& cfg : {Params::original2014(), Params{}}) {
                auto a = detect(im, cfg);
                auto b = detectOnePass(im, cfg);
                if (!sameSegments(a, b)) { std::printf("TINY FAIL %dx%d\n", tw, th2); ++fails; }
            }
        }
        std::printf("[robustness] degenerate sizes: %s\n\n", fails ? "FAIL" : "OK");
        if (fails) return 1;
    }

    // ---------- 3. speed -----------------------------------------------------
    auto speedScene = [&](const char* title, double sigma) {
        Scene big = makeScene(1920, 1080, 160, 95.0, sigma, 7);
        std::printf("== speed, %dx%d, noise sigma=%.0f (%s) ==\n", big.img.width,
                    big.img.height, sigma, title);
        const int runs = quickRuns;

        struct Cfg { const char* name; Params p; };
        Params b0 = Params::original2014();  b0.sparse_feature_scan = b0.sparse_label_scan = false;
        Params b1 = Params::original2014();  // baseline + sparse (default on)
        Params i0 = Params::improved();  i0.sparse_feature_scan = i0.sparse_label_scan = false;
        Params i1 = Params::improved();
        Cfg cfgs[] = {{"baseline  dense ", b0}, {"baseline  sparse", b1},
                      {"improved  dense ", i0}, {"improved  sparse", i1}};
        for (auto& c : cfgs) {
            std::vector<LineSegment> r1, r2;
            double tm = medianMs(runs, [&] { r1 = detect(big.img, c.p); });
            double t1 = medianMs(runs, [&] { r2 = detectOnePass(big.img, c.p); });
            std::printf("%s  multi-pass %7.2f ms   one-pass %7.2f ms   (%zu segs)\n",
                        c.name, tm, t1, r1.size());
        }
        std::printf("per-stage profile (multi-pass, median of %d):\n", runs);
        for (auto& c : cfgs) {
            auto prof = profileStages(big.img, c.p, runs);
            double tot = 0;
            std::printf("  %s :", c.name);
            for (auto& st : prof) { std::printf("  %s %.2f", st.name.c_str() + 3, st.ms); tot += st.ms; }
            std::printf("   | total %.2f ms\n", tot);
        }
        std::printf("\n");
    };
    speedScene("noisy", 8.0);
    speedScene("clean", 1.5);
    if (false) {
        Scene big = makeScene(1920, 1080, 160, 95.0, 8.0, 7);
        const int runs = quickRuns;

        struct Cfg { const char* name; Params p; };
        Params b0 = Params::original2014();  b0.sparse_feature_scan = b0.sparse_label_scan = false;
        Params b1 = Params::original2014();  // baseline + sparse (default on)
        Params i0 = Params::improved();  i0.sparse_feature_scan = i0.sparse_label_scan = false;
        Params i1 = Params::improved();
        Cfg cfgs[] = {{"baseline  dense ", b0}, {"baseline  sparse", b1},
                      {"improved  dense ", i0}, {"improved  sparse", i1}};
        for (auto& c : cfgs) {
            std::vector<LineSegment> r1, r2;
            double tm = medianMs(runs, [&] { r1 = detect(big.img, c.p); });
            double t1 = medianMs(runs, [&] { r2 = detectOnePass(big.img, c.p); });
            std::printf("%s  multi-pass %7.2f ms   one-pass %7.2f ms   (%zu segs)\n",
                        c.name, tm, t1, r1.size());
        }
        std::printf("\nper-stage profile (multi-pass, median of %d):\n", runs);
        for (auto& c : cfgs) {
            auto prof = profileStages(big.img, c.p, runs);
            double tot = 0;
            std::printf("  %s :", c.name);
            for (auto& st : prof) { std::printf("  %s %.2f", st.name.c_str() + 3, st.ms); tot += st.ms; }
            std::printf("   | total %.2f ms\n", tot);
        }
    }
    return 0;
}
