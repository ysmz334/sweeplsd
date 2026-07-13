// Estimator-menu study for the fair "best estimator per detector" protocol
// (paper Section 6.2 / docs vp_evaluation.html Section 4).
//
// For each image of a YUD/NYU manifest and each detector, this runs a MENU of
// Manhattan-frame estimator variants over the same calibrated lines and writes
// one CSV row per (image, method, variant) with the mean frame error:
//
//     img,method,variant,err
//
// The split-half cross-validation aggregation (select the best variant on one
// half by median, evaluate on the other, average the folds) is done downstream
// (tools/vp_bestcfg_cv.py). The estimator mathematics are copied verbatim from
// yud_eval.cpp; a variant only changes (a) the line weight (length vs unit),
// (b) the inlier band tau, (c) the seed budget of the multi-start search
// ("S" = strong search, 20 first-axis seeds instead of 10), and (d) a vertical
// prior seed ("V", the calibrated vertical appended to the seed list).
//
// This tool is a reconstruction of the ad-hoc harness used for the 2026-07
// study (vp_bestcfg.cpp, then unversioned); it is validated by reproducing
// that study's per-image CSV rows exactly for the LSD and ED_Lib columns.
//
// Usage:
//   sweeplsd_vp_bestcfg <manifest.txt> --out rows.csv
//       [--methods imp,lsd,edreal,elsed] [--edreal-dir D] [--elsed-dir D]
//       [--seed-cap-s N] [--vprior-mode M]     (reconstruction trial knobs)

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"
#include "edreal_io.hpp"

#include "lsd.h"  // third_party (AGPL, benchmark only)

namespace {

constexpr double kPi = 3.14159265358979323846;
using sweeplsd::LineSegment;

struct Vec3 { double x = 0, y = 0, z = 0; };
double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }
Vec3 normalize(const Vec3& a) {
    double n = norm(a);
    return n > 1e-12 ? Vec3{a.x / n, a.y / n, a.z / n} : Vec3{0, 0, 0};
}

void jacobiEigen(double A[3][3], double w[3], double V[3][3]) {
    double a[3][3];
    std::memcpy(a, A, sizeof(a));
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) V[i][j] = (i == j) ? 1.0 : 0.0;
    for (int sweep = 0; sweep < 50; ++sweep) {
        double off = std::fabs(a[0][1]) + std::fabs(a[0][2]) + std::fabs(a[1][2]);
        if (off < 1e-14) break;
        for (int p = 0; p < 2; ++p)
            for (int q = p + 1; q < 3; ++q) {
                if (std::fabs(a[p][q]) < 1e-18) continue;
                double theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                double t = (theta >= 0 ? 1.0 : -1.0) /
                           (std::fabs(theta) + std::sqrt(theta * theta + 1));
                double c = 1.0 / std::sqrt(t * t + 1), s = t * c;
                for (int k = 0; k < 3; ++k) {
                    double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    double vkp = V[k][p], vkq = V[k][q];
                    V[k][p] = c * vkp - s * vkq;
                    V[k][q] = s * vkp + c * vkq;
                }
            }
    }
    for (int i = 0; i < 3; ++i) w[i] = a[i][i];
}

Vec3 smallestEigenvector(double S[3][3]) {
    double w[3], V[3][3];
    jacobiEigen(S, w, V);
    int m = 0;
    if (w[1] < w[m]) m = 1;
    if (w[2] < w[m]) m = 2;
    return normalize({V[0][m], V[1][m], V[2][m]});
}

struct CalLine { Vec3 n; double w; };

std::vector<CalLine> calibrate(const std::vector<LineSegment>& segs,
                               double f, double cx, double cy, double min_len) {
    std::vector<CalLine> out;
    out.reserve(segs.size());
    for (const LineSegment& s : segs) {
        double dx = s.x1 - s.x0, dy = s.y1 - s.y0;
        double len = std::sqrt(dx * dx + dy * dy);
        if (len < min_len) continue;
        Vec3 r0 = normalize({(s.x0 - cx) / f, (cy - s.y0) / f, 1.0});
        Vec3 r1 = normalize({(s.x1 - cx) / f, (cy - s.y1) / f, 1.0});
        Vec3 n = cross(r0, r1);
        if (norm(n) < 1e-9) continue;
        out.push_back({normalize(n), len});
    }
    return out;
}

Vec3 refitAxis(const std::vector<CalLine>& lines, const Vec3& d, double tau) {
    double S[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    int cnt = 0;
    for (const CalLine& l : lines) {
        if (std::fabs(dot(l.n, d)) > tau) continue;
        double w = l.w;
        const Vec3& n = l.n;
        S[0][0] += w * n.x * n.x; S[0][1] += w * n.x * n.y; S[0][2] += w * n.x * n.z;
        S[1][1] += w * n.y * n.y; S[1][2] += w * n.y * n.z; S[2][2] += w * n.z * n.z;
        ++cnt;
    }
    if (cnt < 2) return d;
    S[1][0] = S[0][1]; S[2][0] = S[0][2]; S[2][1] = S[1][2];
    return smallestEigenvector(S);
}

double frameScore(const std::vector<CalLine>& lines, const std::array<Vec3, 3>& D, double tau) {
    double s = 0;
    for (const CalLine& l : lines) {
        double best = 1e9;
        for (const Vec3& d : D) best = std::min(best, std::fabs(dot(l.n, d)));
        if (best < tau) s += l.w;
    }
    return s;
}

std::array<Vec3, 3> orthonormalize(Vec3 a, Vec3 b) {
    a = normalize(a);
    b = normalize({b.x - dot(b, a) * a.x, b.y - dot(b, a) * a.y, b.z - dot(b, a) * a.z});
    Vec3 c = normalize(cross(a, b));
    return {a, b, c};
}

// yud_eval.cpp's estimator with the menu knobs exposed: seed_cap (10 default,
// 20 for "S") and vprior (append the calibrated vertical as one extra seed).
bool estimateManhattan(const std::vector<CalLine>& lines, double tau,
                       int seed_cap, int vprior_mode, std::array<Vec3, 3>& out) {
    const int N = (int)lines.size();
    if (N < 8) return false;
    std::mt19937 rng(12345);
    std::uniform_int_distribution<int> pick(0, N - 1);

    std::vector<Vec3> cand;
    int pairs = std::min(4000, N * (N - 1) / 2);
    cand.reserve(pairs);
    for (int t = 0; t < pairs; ++t) {
        int i = pick(rng), j = pick(rng);
        if (i == j) continue;
        Vec3 d = cross(lines[i].n, lines[j].n);
        if (norm(d) < 1e-6) continue;
        cand.push_back(normalize(d));
    }
    if (cand.empty()) return false;

    auto axisScore = [&](const Vec3& d) {
        double s = 0;
        for (const CalLine& l : lines)
            if (std::fabs(dot(l.n, d)) < tau) s += l.w;
        return s;
    };

    std::sort(cand.begin(), cand.end(),
              [&](const Vec3& a, const Vec3& b) { return axisScore(a) > axisScore(b); });
    std::vector<Vec3> seeds;
    for (const Vec3& d : cand) {
        bool dup = false;
        for (const Vec3& s : seeds) if (std::fabs(dot(d, s)) > 0.99) { dup = true; break; }
        if (!dup) seeds.push_back(d);
        if ((int)seeds.size() >= seed_cap) break;
    }
    if (vprior_mode == 1) seeds.push_back(Vec3{0, 1, 0});          // append vertical
    else if (vprior_mode == 2) seeds.insert(seeds.begin(), Vec3{0, 1, 0});  // prepend

    auto refine = [&](std::array<Vec3, 3> tri) {
        for (int iter = 0; iter < 8; ++iter) {
            std::array<Vec3, 3> R{refitAxis(lines, tri[0], tau),
                                  refitAxis(lines, tri[1], tau),
                                  refitAxis(lines, tri[2], tau)};
            double sc[3];
            for (int a = 0; a < 3; ++a) sc[a] = axisScore(R[a]);
            int order[3] = {0, 1, 2};
            std::sort(order, order + 3, [&](int a, int b) { return sc[a] > sc[b]; });
            tri = orthonormalize(R[order[0]], R[order[1]]);
        }
        return tri;
    };

    double bestScore = -1;
    bool found = false;
    for (const Vec3& d1 : seeds) {
        Vec3 d2{0, 0, 0};
        double best2 = -1;
        for (const Vec3& d : cand) {
            if (std::fabs(dot(d, d1)) > 0.26) continue;
            double s = axisScore(d);
            if (s > best2) { best2 = s; d2 = d; }
        }
        if (best2 < 0) {
            Vec3 t = std::fabs(d1.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
            d2 = normalize(cross(d1, t));
        }
        std::array<Vec3, 3> tri = refine(orthonormalize(d1, d2));
        double sc = frameScore(lines, tri, tau);
        if (sc > bestScore) { bestScore = sc; out = tri; found = true; }
    }
    return found;
}

struct FrameError { double mean_deg, max_deg; };
FrameError frameError(const std::array<Vec3, 3>& est, const std::array<Vec3, 3>& gt) {
    static const int perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2},
                                    {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
    double best_sum = 1e18, best_each[3] = {0, 0, 0};
    for (auto& p : perms) {
        double each[3], sum = 0;
        for (int i = 0; i < 3; ++i) {
            double c = std::min(1.0, std::fabs(dot(est[i], gt[p[i]])));
            each[i] = std::acos(c) * 180.0 / kPi;
            sum += each[i];
        }
        if (sum < best_sum) {
            best_sum = sum;
            for (int i = 0; i < 3; ++i) best_each[i] = each[i];
        }
    }
    double mx = std::max(best_each[0], std::max(best_each[1], best_each[2]));
    return {best_sum / 3.0, mx};
}

std::vector<LineSegment> runLsd(const sweeplsd::GrayImage& src) {
    std::vector<double> buf(std::size_t(src.width) * src.height);
    for (int i = 0; i < src.width * src.height; ++i) buf[i] = double(src.data[i]);
    int n = 0;
    double* out = lsd(&n, buf.data(), src.width, src.height);
    std::vector<LineSegment> segs;
    segs.reserve(n);
    for (int j = 0; j < n; ++j)
        segs.push_back({(float)out[7 * j], (float)out[7 * j + 1],
                        (float)out[7 * j + 2], (float)out[7 * j + 3]});
    std::free(out);
    return segs;
}

struct Variant {
    const char* name;
    bool unit;       // unit weights instead of length
    double tau_deg;
    bool strong;     // "S": larger seed budget
    bool vprior;     // "V": vertical prior seed
};

}  // namespace

int main(int argc, char** argv) {
    std::string manifest_path, out_path, edreal_dir, elsed_dir,
        methods = "lsd";
    int seed_cap_s = 20, vprior_mode = 1;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--out" && i + 1 < argc) out_path = argv[++i];
        else if (a == "--methods" && i + 1 < argc) methods = argv[++i];
        else if (a == "--edreal-dir" && i + 1 < argc) edreal_dir = argv[++i];
        else if (a == "--elsed-dir" && i + 1 < argc) elsed_dir = argv[++i];
        else if (a == "--seed-cap-s" && i + 1 < argc) seed_cap_s = std::atoi(argv[++i]);
        else if (a == "--vprior-mode" && i + 1 < argc) vprior_mode = std::atoi(argv[++i]);
        else if (!a.empty() && a[0] != '-') manifest_path = a;
    }
    if (manifest_path.empty() || out_path.empty()) {
        std::printf("Usage: %s <manifest.txt> --out rows.csv [--methods imp,lsd,edreal,elsed]\n"
                    "       [--edreal-dir D] [--elsed-dir D] [--seed-cap-s N] [--vprior-mode M]\n",
                    argv[0]);
        return 1;
    }
    auto want = [&](const char* m) { return methods.find(m) != std::string::npos; };

    std::ifstream in(manifest_path);
    if (!in) { std::printf("Error: cannot open %s\n", manifest_path.c_str()); return 1; }
    double f, cx, cy;
    {
        std::string l0;
        std::getline(in, l0);
        std::istringstream is(l0);
        is >> f >> cx >> cy;
    }
    struct Row { std::string name, path; std::array<Vec3, 3> gt; };
    std::vector<Row> rows;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::istringstream is(line);
        Row r;
        double m[9];
        is >> r.name >> r.path;
        for (double& v : m) is >> v;
        r.gt = {Vec3{m[0], m[3], m[6]}, Vec3{m[1], m[4], m[7]}, Vec3{m[2], m[5], m[8]}};
        rows.push_back(r);
    }

    static const Variant kMenu[] = {
        {"len_t1", false, 1, false, false},  {"unit_t1", true, 1, false, false},
        {"len_t2", false, 2, false, false},  {"unit_t2", true, 2, false, false},
        {"len_t3", false, 3, false, false},  {"unit_t3", true, 3, false, false},
        {"len_t2S", false, 2, true, false},  {"unit_t2S", true, 2, true, false},
        {"len_t3S", false, 3, true, false},  {"unit_t3S", true, 3, true, false},
        {"len_t2SV", false, 2, true, true},  {"unit_t2SV", true, 2, true, true},
    };
    const double min_len = 12.0;

    std::ofstream o(out_path);
    if (!o) { std::printf("Error: cannot write %s\n", out_path.c_str()); return 1; }
    o << "img,method,variant,err\n";
    char buf[256];

    int done = 0;
    for (const Row& r : rows) {
        struct M { const char* key; std::vector<LineSegment> segs; };
        std::vector<M> ms;
        sweeplsd::GrayImage gray;
        if (want("imp") || want("lsd")) {
            gray = sweeplsd::loadGray(r.path);
            if (gray.width == 0) { std::printf("  skip (load fail): %s\n", r.path.c_str()); continue; }
        }
        if (want("imp")) ms.push_back({"imp", sweeplsd::detect(gray, sweeplsd::Params::improved())});
        if (want("lsd")) ms.push_back({"lsd", runLsd(gray)});
        if (want("edreal") && !edreal_dir.empty()) {
            std::vector<LineSegment> v;
            readEdRealFile(edreal_dir + "/" + r.name + ".txt", v);
            ms.push_back({"edreal", v});
        }
        if (want("elsed") && !elsed_dir.empty()) {
            std::vector<LineSegment> v;
            readEdRealFile(elsed_dir + "/" + r.name + ".txt", v);
            ms.push_back({"elsed", v});
        }

        for (const M& m : ms) {
            std::vector<CalLine> base = calibrate(m.segs, f, cx, cy, min_len);
            for (const Variant& v : kMenu) {
                std::vector<CalLine> cl = base;
                if (v.unit) for (CalLine& c : cl) c.w = 1.0;
                double tau = std::sin(v.tau_deg * kPi / 180.0);
                std::array<Vec3, 3> frame;
                if (!estimateManhattan(cl, tau, v.strong ? seed_cap_s : 10,
                                       v.vprior ? vprior_mode : 0, frame))
                    continue;
                FrameError e = frameError(frame, r.gt);
                std::snprintf(buf, sizeof(buf), "%s,%s,%s,%.6f\n",
                              r.name.c_str(), m.key, v.name, e.mean_deg);
                o << buf;
            }
        }
        if (++done % 50 == 0) std::printf("  ...%d/%zu\n", done, rows.size());
    }
    std::printf("wrote %s (%d images)\n", out_path.c_str(), done);
    return 0;
}
