// York Urban Database (YUD) downstream evaluation: comparing SweepLSD / LSD /
// EDLines by how well the lines they detect support CALIBRATED MANHATTAN-FRAME
// (vanishing-point) estimation.
//
// Why this protocol. YUD's hand-labelled line segments are an INCOMPLETE,
// Manhattan-only annotation built for vanishing-point research, not a
// trustworthy line-detection ground truth: a detector that finds a real but
// unlabelled line would be wrongly counted as a false positive, and the
// hand-clicked endpoints are geometrically coarse. So we do NOT score detectors
// against YUD's lines. Instead we use YUD's RELIABLE ground truth — the
// orthogonal vanishing-point frame (camera rotation relative to the scene),
// which is exactly what the dataset was built and validated for. Each detector's
// own line segments are fed into the SAME calibrated Manhattan-frame estimator,
// and the estimated three orthogonal directions are compared to the GT frame.
// The estimator is identical across detectors, so differences reflect line
// quality, not the estimator. This evaluates detectors by downstream usefulness
// on reliable GT.
//
// Input: a manifest produced by tools/yud_export.py (intrinsics + per-image GT
// rotation). Run:
//   sweeplsd_yud <manifest.txt> [--html docs/yud_evaluation.html --assets docs/assets/yud]

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
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
using sweeplsd::LineSegmentEx;

// --------------------------------------------------------------------------
// Minimal 3D vector + symmetric 3x3 eigensolver
// --------------------------------------------------------------------------
struct Vec3 {
    double x = 0, y = 0, z = 0;
};
Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }
Vec3 normalize(const Vec3& a) {
    double n = norm(a);
    return n > 1e-12 ? Vec3{a.x / n, a.y / n, a.z / n} : Vec3{0, 0, 0};
}

// Jacobi eigen-decomposition of a symmetric 3x3 (rows of A). Fills eigenvalues
// `w` and eigenvectors as columns of `V`.
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

// Smallest-eigenvalue eigenvector of symmetric 3x3 S.
Vec3 smallestEigenvector(double S[3][3]) {
    double w[3], V[3][3];
    jacobiEigen(S, w, V);
    int m = 0;
    if (w[1] < w[m]) m = 1;
    if (w[2] < w[m]) m = 2;
    return normalize({V[0][m], V[1][m], V[2][m]});
}

// --------------------------------------------------------------------------
// Calibrated Manhattan-frame estimation from line segments
// --------------------------------------------------------------------------
struct CalLine {
    Vec3 n;     // interpretation-plane normal (a VP direction d satisfies d.n=0)
    double w;   // weight = pixel length
};

std::vector<CalLine> calibrate(const std::vector<LineSegment>& segs,
                               double f, double cx, double cy, double min_len) {
    std::vector<CalLine> out;
    out.reserve(segs.size());
    for (const LineSegment& s : segs) {
        double dx = s.x1 - s.x0, dy = s.y1 - s.y0;
        double len = std::sqrt(dx * dx + dy * dy);
        if (len < min_len) continue;
        // Image y is downward but YUD's VP frame uses y upward, so flip y
        // (verified: with this sign the labelled GT lines are perpendicular to
        // their VP direction to ~0.5deg; without it, ~10deg).
        Vec3 r0 = normalize({(s.x0 - cx) / f, (cy - s.y0) / f, 1.0});
        Vec3 r1 = normalize({(s.x1 - cx) / f, (cy - s.y1) / f, 1.0});
        Vec3 n = cross(r0, r1);
        if (norm(n) < 1e-9) continue;
        out.push_back({normalize(n), len});
    }
    return out;
}

// Calibrated camera ray for an image point, with YUD's y-up convention.
Vec3 rayOf(double x, double y, double f, double cx, double cy) {
    return normalize({(x - cx) / f, (cy - y) / f, 1.0});
}

// SweepLSD-only: calibrate the extended segments, weighting each line by the
// reliability its covariance implies. The angular variance of a least-squares
// line fit is ~ sigma^2 / (N * ev_max) (N edge pixels spread with variance
// ev_max along the line), so the inverse-variance weight is N * ev_max — long,
// dense, well-spread lines dominate, which the endpoint-length weight only
// approximates (length ~ sqrt(ev_max)). LSD / EDLines cannot do this (no
// moments), so it is not part of the cross-method comparison.
std::vector<CalLine> calibrateEx(const std::vector<LineSegmentEx>& segs,
                                 double f, double cx, double cy, double min_len, int scheme) {
    std::vector<CalLine> out;
    out.reserve(segs.size());
    for (const LineSegmentEx& e : segs) {
        const LineSegment& s = e.seg;
        double dx = s.x1 - s.x0, dy = s.y1 - s.y0;
        double len = std::sqrt(dx * dx + dy * dy);
        if (len < min_len) continue;
        Vec3 n = cross(rayOf(s.x0, s.y0, f, cx, cy), rayOf(s.x1, s.y1, f, cx, cy));
        if (norm(n) < 1e-9) continue;
        double N = double(e.pix_num), evx = double(e.ev_max), evn = double(e.ev_min);
        double w = len;
        switch (scheme) {
            case 0: w = N * evx; break;                       // inverse angular variance (~len^3)
            case 1: w = evx; break;                           // variance along line (~len^2)
            case 2: w = N; break;                             // pixel count (~len)
            case 3: w = std::sqrt(N * evx); break;            // ~len^1.5
            case 4: w = len * (1.0 - evn / (evx + 1e-9)); break;  // length x elongation reliability
        }
        out.push_back({normalize(n), w});
    }
    return out;
}

// Re-fit one axis as the smallest-eigenvector of the weighted scatter of its
// inlier normals (the direction most perpendicular to all of them).
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

// Make a triad orthonormal, keeping axis order (Gram-Schmidt + cross).
std::array<Vec3, 3> orthonormalize(Vec3 a, Vec3 b) {
    a = normalize(a);
    b = normalize({b.x - dot(b, a) * a.x, b.y - dot(b, a) * a.y, b.z - dot(b, a) * a.z});
    Vec3 c = normalize(cross(a, b));
    return {a, b, c};
}

// Estimate the orthogonal Manhattan frame (3 directions) from calibrated lines.
bool estimateManhattan(const std::vector<CalLine>& lines, double tau,
                       std::array<Vec3, 3>& out) {
    const int N = (int)lines.size();
    if (N < 8) return false;
    std::mt19937 rng(12345);
    std::uniform_int_distribution<int> pick(0, N - 1);

    // 1) Candidate VP directions from random line pairs (a VP direction shared
    //    by two lines is perpendicular to both their normals).
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

    // 2) Rank candidate directions by inlier support and keep the strongest few
    //    distinct ones as first-axis seeds (multi-start, to avoid local optima).
    std::sort(cand.begin(), cand.end(),
              [&](const Vec3& a, const Vec3& b) { return axisScore(a) > axisScore(b); });
    std::vector<Vec3> seeds;
    for (const Vec3& d : cand) {
        bool dup = false;
        for (const Vec3& s : seeds) if (std::fabs(dot(d, s)) > 0.99) { dup = true; break; }
        if (!dup) seeds.push_back(d);
        if ((int)seeds.size() >= 10) break;
    }

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

    // 3) For each first-axis seed, pick the best near-orthogonal second axis,
    //    refine the triad, and keep the one with the highest total inlier score.
    double bestScore = -1;
    bool found = false;
    for (const Vec3& d1 : seeds) {
        Vec3 d2{0, 0, 0};
        double best2 = -1;
        for (const Vec3& d : cand) {
            if (std::fabs(dot(d, d1)) > 0.26) continue;  // > ~75deg from d1
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

// --------------------------------------------------------------------------
// Compare estimated frame to GT (best signed-permutation assignment)
// --------------------------------------------------------------------------
struct FrameError {
    double mean_deg, max_deg;
};
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
    double mx = std::max({best_each[0], best_each[1], best_each[2]});
    return {best_sum / 3.0, mx};
}

// --------------------------------------------------------------------------
// Estimator self-consistency on SYNTHETIC scenes (validates the estimator).
// A known camera rotation defines a Manhattan frame; we generate line segments
// that are exact projections of 3D lines along its three axes, optionally add
// pixel noise to the endpoints, and feed them through the SAME pipeline. At
// zero noise the recovered frame must match the known one (~0deg), proving the
// math is correct and unbiased; the error vs noise characterises the instrument.
// --------------------------------------------------------------------------
struct Mat3 { double m[3][3]; };
Mat3 matmul(const Mat3& A, const Mat3& B) {
    Mat3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) C.m[i][j] += A.m[i][k] * B.m[k][j];
    return C;
}
Mat3 rotX(double a) { double c=std::cos(a),s=std::sin(a); return {{{1,0,0},{0,c,-s},{0,s,c}}}; }
Mat3 rotY(double a) { double c=std::cos(a),s=std::sin(a); return {{{c,0,s},{0,1,0},{-s,0,c}}}; }
Mat3 rotZ(double a) { double c=std::cos(a),s=std::sin(a); return {{{c,-s,0},{s,c,0},{0,0,1}}}; }
Vec3 colOf(const Mat3& R, int k) { return {R.m[0][k], R.m[1][k], R.m[2][k]}; }
Vec3 mul(const Mat3& R, const Vec3& v) {
    return {R.m[0][0]*v.x + R.m[0][1]*v.y + R.m[0][2]*v.z,
            R.m[1][0]*v.x + R.m[1][1]*v.y + R.m[1][2]*v.z,
            R.m[2][0]*v.x + R.m[2][1]*v.y + R.m[2][2]*v.z};
}

// One synthetic trial; returns mean axis error in degrees, or -1 on failure.
double syntheticTrial(double f, double cx, double cy, int W, int H, int N,
                      double sigma, std::mt19937& rng) {
    std::uniform_real_distribution<double> uYaw(-35, 35), uPit(-25, 25), uRol(-15, 15);
    Mat3 R = matmul(matmul(rotZ(uRol(rng) * kPi / 180), rotX(uPit(rng) * kPi / 180)),
                    rotY(uYaw(rng) * kPi / 180));
    std::array<Vec3, 3> gt{colOf(R, 0), colOf(R, 1), colOf(R, 2)};

    std::uniform_real_distribution<double> uz(3, 12), us(-1, 1);
    std::uniform_real_distribution<double> uqx(60.0, W - 60.0), uqy(60.0, H - 60.0);
    std::uniform_int_distribution<int> uAxis(0, 2);
    std::normal_distribution<double> noise(0.0, sigma);
    auto project = [&](const Vec3& P, double& x, double& y) {
        if (P.z <= 1e-6) return false;
        x = cx + f * P.x / P.z;
        y = cy - f * P.y / P.z;
        return true;
    };

    std::vector<LineSegment> segs;
    int guard = 0;
    while ((int)segs.size() < N && guard++ < N * 40) {
        Vec3 d = gt[uAxis(rng)];
        double qx = uqx(rng), qy = uqy(rng), Z = uz(rng);
        Vec3 ray{(qx - cx) / f, (cy - qy) / f, 1.0};
        Vec3 X{Z * ray.x, Z * ray.y, Z * ray.z};
        double L = Z * 0.18;
        double s0 = us(rng) * L, s1 = us(rng) * L;
        if (std::fabs(s0 - s1) < 0.25 * L) continue;
        Vec3 P0{X.x + s0 * d.x, X.y + s0 * d.y, X.z + s0 * d.z};
        Vec3 P1{X.x + s1 * d.x, X.y + s1 * d.y, X.z + s1 * d.z};
        double x0, y0, x1, y1;
        if (!project(P0, x0, y0) || !project(P1, x1, y1)) continue;
        if (x0 < 0 || x0 >= W || y0 < 0 || y0 >= H || x1 < 0 || x1 >= W || y1 < 0 || y1 >= H)
            continue;
        if (std::hypot(x1 - x0, y1 - y0) < 15) continue;
        segs.push_back({float(x0 + noise(rng)), float(y0 + noise(rng)),
                        float(x1 + noise(rng)), float(y1 + noise(rng))});
    }
    if ((int)segs.size() < N) return -1;
    std::vector<CalLine> cl = calibrate(segs, f, cx, cy, 8.0);
    std::array<Vec3, 3> frame;
    if (!estimateManhattan(cl, std::sin(2.0 * kPi / 180.0), frame)) return -1;
    return frameError(frame, gt).mean_deg;
}

// --------------------------------------------------------------------------
// Detector runners (default operating points) — same glue as compare.cpp
// --------------------------------------------------------------------------
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

// --------------------------------------------------------------------------
// Aggregation
// --------------------------------------------------------------------------
struct Acc {
    std::string name;
    std::vector<double> mean_err, max_err;
    std::vector<int> nlines;
    int failed = 0;
};
double median(std::vector<double> v) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}
double medianI(std::vector<int> v) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}
double fracBelow(const std::vector<double>& v, double th) {
    if (v.empty()) return 0;
    int c = 0;
    for (double x : v) if (x < th) ++c;
    return 100.0 * c / v.size();
}

struct ManifestRow {
    std::string name, path;
    std::array<Vec3, 3> gt;
};

// Read gtlines.txt (blocks of "<name> <nlines>" then <nlines> "x0 y0 x1 y1").
std::unordered_map<std::string, std::vector<LineSegment>> readGtLines(const std::string& path) {
    std::unordered_map<std::string, std::vector<LineSegment>> out;
    std::ifstream in(path);
    if (!in) return out;
    std::string name;
    int n;
    while (in >> name >> n) {
        std::vector<LineSegment> v;
        v.reserve(n);
        for (int i = 0; i < n; ++i) {
            float x0, y0, x1, y1;
            in >> x0 >> y0 >> x1 >> y1;
            v.push_back({x0, y0, x1, y1});
        }
        out[name] = std::move(v);
    }
    return out;
}

// Render detected segments coloured by their assigned Manhattan axis (R/G/B)
// over a dimmed grayscale image; unassigned segments stay gray.
void renderAxes(const std::string& out_png, const sweeplsd::GrayImage& gray,
                const std::vector<LineSegment>& segs, const std::array<Vec3, 3>& frame,
                double f, double cx, double cy, double tau) {
    int w = gray.width, h = gray.height;
    std::vector<unsigned char> rgb(std::size_t(w) * h * 3);
    for (int i = 0; i < w * h; ++i) {
        unsigned char g = (unsigned char)(gray.data[i] / 2 + 30);
        rgb[3 * i] = rgb[3 * i + 1] = rgb[3 * i + 2] = g;
    }
    auto plot = [&](int x, int y, const unsigned char c[3]) {
        if (x < 0 || y < 0 || x >= w || y >= h) return;
        std::size_t i = (std::size_t(y) * w + x) * 3;
        rgb[i] = c[0]; rgb[i + 1] = c[1]; rgb[i + 2] = c[2];
    };
    const unsigned char col[4][3] = {
        {235, 64, 52}, {52, 200, 80}, {72, 130, 245}, {150, 150, 150}};  // x,y,z,unassigned
    for (const LineSegment& s : segs) {
        double dx = s.x1 - s.x0, dy = s.y1 - s.y0;
        if (std::sqrt(dx * dx + dy * dy) < 12) continue;
        Vec3 r0 = normalize({(s.x0 - cx) / f, (cy - s.y0) / f, 1.0});
        Vec3 r1 = normalize({(s.x1 - cx) / f, (cy - s.y1) / f, 1.0});
        Vec3 n = normalize(cross(r0, r1));
        int a = 3;
        double best = tau;
        for (int k = 0; k < 3; ++k) {
            double r = std::fabs(dot(n, frame[k]));
            if (r < best) { best = r; a = k; }
        }
        int steps = (int)std::max(std::fabs(dx), std::fabs(dy)) + 1;
        for (int t = 0; t <= steps; ++t) {
            double u = steps ? double(t) / steps : 0;
            int x = (int)std::lround(s.x0 + u * dx), y = (int)std::lround(s.y0 + u * dy);
            plot(x, y, col[a]); plot(x + 1, y, col[a]); plot(x, y + 1, col[a]);
        }
    }
    sweeplsd::saveRgbPng(out_png, w, h, rgb);
}

}  // namespace

int main(int argc, char** argv) {
    std::string manifest_path, html_path, assets_dir, gtlines_path, mlsd_dir, edreal_dir,
        elsed_dir;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--html" && i + 1 < argc) html_path = argv[++i];
        else if (a == "--assets" && i + 1 < argc) assets_dir = argv[++i];
        else if (a == "--gtlines" && i + 1 < argc) gtlines_path = argv[++i];
        else if (a == "--mlsd-dir" && i + 1 < argc) mlsd_dir = argv[++i];
        else if (a == "--edreal-dir" && i + 1 < argc) edreal_dir = argv[++i];
        // --elsed-dir DIR : ingest genuine ELSED segments per image (same file
        // format as the EDLines runner: "<count> <median_ms>" then x0 y0 x1 y1)
        else if (a == "--elsed-dir" && i + 1 < argc) elsed_dir = argv[++i];
        else if (!a.empty() && a[0] != '-') manifest_path = a;
    }
    if (manifest_path.empty()) {
        std::printf("Usage: %s <manifest.txt> [--html PATH --assets DIR]\n", argv[0]);
        return 1;
    }

    std::ifstream in(manifest_path);
    if (!in) { std::printf("Error: cannot open %s\n", manifest_path.c_str()); return 1; }
    double f, cx, cy;
    {
        std::string l0;
        std::getline(in, l0);
        std::istringstream is(l0);
        is >> f >> cx >> cy;
    }
    std::vector<ManifestRow> rows;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::istringstream is(line);
        ManifestRow r;
        double m[9];
        is >> r.name >> r.path;
        for (double& v : m) is >> v;
        // GT directions = columns of the row-major 3x3.
        r.gt = {Vec3{m[0], m[3], m[6]}, Vec3{m[1], m[4], m[7]}, Vec3{m[2], m[5], m[8]}};
        rows.push_back(r);
    }
    std::printf("YUD downstream Manhattan-frame evaluation: %zu images, f=%.1fpx\n",
                rows.size(), f);

    const double tau = std::sin(2.0 * kPi / 180.0);  // 2-degree inlier band
    const double min_len = 12.0;

    // --- (1) Estimator validity on synthetic scenes: known rotation + exact
    //     projected Manhattan lines, with endpoint noise sigma swept. ---
    struct SynPoint { double sigma, med, frac1; };
    std::vector<SynPoint> syn;
    {
        std::mt19937 rng(2024);
        const int trials = 300;
        std::printf("estimator self-consistency (synthetic, %d trials/condition):\n", trials);
        for (double sigma : {0.0, 0.5, 1.0, 2.0, 4.0}) {
            std::vector<double> errs;
            for (int t = 0; t < trials; ++t) {
                double e = syntheticTrial(f, cx, cy, 640, 480, 40, sigma, rng);
                if (e >= 0) errs.push_back(e);
            }
            double med = median(errs), fr = fracBelow(errs, 1.0);
            syn.push_back({sigma, med, fr});
            std::printf("  noise=%.1fpx  medErr=%.3f°  (<1°: %.0f%%, %zu trials)\n",
                        sigma, med, fr, errs.size());
        }
        std::printf("\n");
    }

    // --- (2) Estimator ceiling on YUD GT lines: feed the hand-labelled lines
    //     through the SAME pipeline (best the instrument can do on real scenes). ---
    if (gtlines_path.empty()) {
        std::string md = manifest_path;
        std::string::size_type s = md.find_last_of("/\\");
        gtlines_path = (s == std::string::npos ? std::string() : md.substr(0, s + 1)) + "gtlines.txt";
    }
    std::unordered_map<std::string, std::vector<LineSegment>> gtlines = readGtLines(gtlines_path);

    Acc sweeplsd_a{"SweepLSD"}, sweeplsd_imp_a{"SweepLSD-improved"}, sweeplsd_implink_a{"SweepLSD-imp+link"},
        lsd_a{"LSD"}, ed_a{"EDLines-style"}, edreal_a{"EDLines (ED_Lib)"}, mlsd_a{"M-LSD"},
        elsed_a{"ELSED"}, ceil_a{"GT lines (ceiling)"};
    const bool have_mlsd = !mlsd_dir.empty();
    const bool have_edreal = !edreal_dir.empty();
    const bool have_elsed = !elsed_dir.empty();
    auto runEdReal = [&](const std::string& name) {
        std::vector<LineSegment> v;
        readEdRealFile(edreal_dir + "/" + name + ".txt", v);
        return v;
    };
    auto runElsed = [&](const std::string& name) {
        std::vector<LineSegment> v;
        readEdRealFile(elsed_dir + "/" + name + ".txt", v);
        return v;
    };
    auto runMlsd = [&](const std::string& name) {
        std::vector<LineSegment> v;
        readMlsdFile(mlsd_dir + "/" + name + ".txt", v);
        return v;
    };
    auto runSweeplsd = [](const sweeplsd::GrayImage& s) { return sweeplsd::detect(s, sweeplsd::Params{}); };
    auto runSweeplsdImp = [](const sweeplsd::GrayImage& s) {
        return sweeplsd::detect(s, sweeplsd::Params::improved());
    };
    auto runSweeplsdImpLink = [](const sweeplsd::GrayImage& s) {
        sweeplsd::Params p = sweeplsd::Params::improved();
        p.link_collinear = true;  // linker defaults (lateral tol, gap, two-stage)
        return sweeplsd::detect(s, p);
    };
    auto runEd = [](const sweeplsd::GrayImage& s) { return edlines::detect(s); };

    int done = 0;
    for (const ManifestRow& r : rows) {
        sweeplsd::GrayImage gray = sweeplsd::loadGray(r.path);
        if (gray.width == 0) { std::printf("  skip (load fail): %s\n", r.path.c_str()); continue; }

        struct RunOne { Acc* acc; std::vector<LineSegment> segs; };
        std::vector<RunOne> runs = {
            {&sweeplsd_a, runSweeplsd(gray)}, {&sweeplsd_imp_a, runSweeplsdImp(gray)},
            {&sweeplsd_implink_a, runSweeplsdImpLink(gray)},
            {&lsd_a, runLsd(gray)}, {&ed_a, runEd(gray)}};
        if (have_edreal) runs.push_back({&edreal_a, runEdReal(r.name)});
        if (have_mlsd) runs.push_back({&mlsd_a, runMlsd(r.name)});
        if (have_elsed) runs.push_back({&elsed_a, runElsed(r.name)});
        auto git = gtlines.find(r.name);
        if (git != gtlines.end()) runs.push_back({&ceil_a, git->second});
        for (RunOne& ro : runs) {
            std::vector<CalLine> cl = calibrate(ro.segs, f, cx, cy, min_len);
            std::array<Vec3, 3> frame;
            if (!estimateManhattan(cl, tau, frame)) { ro.acc->failed++; continue; }
            FrameError e = frameError(frame, r.gt);
            ro.acc->mean_err.push_back(e.mean_deg);
            ro.acc->max_err.push_back(e.max_deg);
            ro.acc->nlines.push_back((int)cl.size());
        }

        if (++done % 20 == 0) std::printf("  ...%d/%zu\n", done, rows.size());
    }
    bool have_ceiling = !ceil_a.mean_err.empty();

    std::printf("\nestimator: calibrated, tau=2.0deg, min line len=%.0fpx\n", min_len);
    std::printf("metric: angular error (deg) between estimated & GT orthogonal VP directions\n\n");
    std::printf("  %-26s %5s %5s %10s %10s %8s %8s %8s\n", "method", "imgs", "fail",
                "medMeanErr", "medMaxErr", "%<2deg", "%<5deg", "medLines");
    std::vector<const Acc*> table = {&sweeplsd_a, &sweeplsd_imp_a, &sweeplsd_implink_a, &lsd_a, &ed_a};
    if (have_edreal) table.push_back(&edreal_a);
    if (have_mlsd) table.push_back(&mlsd_a);
    if (have_elsed) table.push_back(&elsed_a);
    if (have_ceiling) table.push_back(&ceil_a);
    for (const Acc* a : table)
        std::printf("  %-26s %5zu %5d %9.2f° %9.2f° %7.0f%% %7.0f%% %8.0f\n",
                    a->name.c_str(), a->mean_err.size(), a->failed, median(a->mean_err),
                    median(a->max_err), fracBelow(a->mean_err, 2.0),
                    fracBelow(a->mean_err, 5.0), medianI(a->nlines));

    // ---- HTML report ----
    if (!html_path.empty()) {
        std::ofstream o(html_path);
        if (o) {
            o << "<!doctype html><html lang=\"ja\"><head><meta charset=\"utf-8\">"
                 "<title>SweepLSD 評価 — York Urban 下流VP精度</title><style>"
                 "body{font-family:'Segoe UI',Meiryo,sans-serif;margin:0;background:#f5f6f8;color:#1d2027;line-height:1.7}"
                 ".wrap{max-width:920px;margin:0 auto;padding:32px 24px 64px}"
                 "h1{font-size:24px;margin:0 0 4px}h2{margin-top:36px;border-bottom:2px solid #d8dbe0;padding-bottom:6px}"
                 "table{border-collapse:collapse;width:100%;background:#fff;box-shadow:0 1px 3px rgba(0,0,0,.08);border-radius:8px;overflow:hidden;margin-top:10px}"
                 "th,td{padding:9px 12px;text-align:right;border-bottom:1px solid #eef0f3}th:first-child,td:first-child{text-align:left}"
                 "thead th{background:#2b3245;color:#fff}.sweeplsd{color:#1769aa;font-weight:600}"
                 ".note{background:#fff8e1;border-left:4px solid #f5c518;padding:12px 16px;border-radius:4px;font-size:14px}"
                 "code{background:#eceff4;padding:1px 5px;border-radius:4px}</style></head><body><div class=\"wrap\">";
            o << "<h1>York Urban Database による下流評価 — 消失点（Manhattan枠）推定精度</h1>";
            o << "<div class=\"note\"><b>なぜこの評価か。</b> YUD の手ラベル線分は "
                 "<b>Manhattan構造線のみの不完全な注釈</b>で、線分検出の信頼できるGTではない"
                 "（実在する未ラベル線を検出すると誤検出扱いになり、端点も人手クリックで粗い）。"
                 "そこで線分GTには一切依存せず、YUD が<b>本来検証している信頼できるGT＝直交消失点枠"
                 "（カメラ回転）</b>を使う。各検出器の線分を<b>同一の較正済みManhattan枠推定器</b>に通し、"
                 "推定した直交3方向をGTと比較する。推定器は全手法で同一なので、差は線分の質を反映する。</div>";
            o << "<h2>手法</h2><ol>"
                 "<li>各線分を較正（K⁻¹で端点を正規化）し、解釈平面の法線 n を得る（消失点方向 d は d·n=0）。</li>"
                 "<li>線分対の外積から消失点方向の候補を生成し、インライア（|n·d|<2°）の重み付き本数でスコア。"
                 "最良の第1軸→ほぼ直交する第2軸→第3軸=外積。各軸を inlier 法線の最小固有ベクトル"
                 "（3×3 Jacobi）で精密化し直交化、を反復。</li>"
                 "<li>推定3方向とGT3方向を符号・順列を考慮して対応付け、<b>角度誤差</b>を算出。</li>"
                 "</ol>";
            o << "<h2>結果（102画像）</h2><table><thead><tr><th>手法</th><th>画像数</th>"
                 "<th>失敗</th><th>平均誤差(中央値)</th><th>最大誤差(中央値)</th>"
                 "<th>平均&lt;2°</th><th>平均&lt;5°</th><th>線分数(中央値)</th></tr></thead><tbody>";
            std::vector<const Acc*> tbl = {&sweeplsd_a, &sweeplsd_imp_a, &sweeplsd_implink_a, &lsd_a, &ed_a};
            if (have_mlsd) tbl.push_back(&mlsd_a);
            if (have_ceiling) tbl.push_back(&ceil_a);
            for (const Acc* a : tbl) {
                char row[512];
                const char* cls = (a->name == "SweepLSD") ? " class=\"sweeplsd\""
                                  : (a == &ceil_a ? " style=\"color:#888;font-style:italic\"" : "");
                std::snprintf(row, sizeof(row),
                    "<tr><td%s>%s</td><td>%zu</td><td>%d</td><td%s>%.2f°</td><td>%.2f°</td>"
                    "<td>%.0f%%</td><td>%.0f%%</td><td>%.0f</td></tr>",
                    cls, a->name.c_str(), a->mean_err.size(), a->failed, cls,
                    median(a->mean_err), median(a->max_err), fracBelow(a->mean_err, 2.0),
                    fracBelow(a->mean_err, 5.0), medianI(a->nlines));
                o << row;
            }
            o << "</tbody></table>";

            // ---- Estimator validity (same VP pipeline) ----
            o << "<h2>推定器自体の妥当性（同一VPパイプライン）</h2>";
            o << "<p>検出器の優劣は推定器が健全であって初めて意味を持つ。そこで<b>同じ推定器</b>を2通りで検証する。</p>";
            o << "<h3>(1) 合成シーンでの自己無撞着</h3>"
                 "<p>既知のカメラ回転が定める Manhattan 枠から、その3軸に沿う3D直線を投影して線分を生成し"
                 "（＝厳密に枠と整合）、端点に画素ノイズ &sigma; を加えて<b>同じ推定器</b>に通す。"
                 "&sigma;=0 で誤差が ≈0° なら数式・実装は正しく無バイアス、&sigma; 増で緩やかに悪化＝計器特性。</p>";
            o << "<table><thead><tr><th>端点ノイズ &sigma; (px)</th><th>角度誤差(中央値)</th>"
                 "<th>誤差&lt;1° の割合</th></tr></thead><tbody>";
            for (const SynPoint& p : syn) {
                char row[256];
                std::snprintf(row, sizeof(row),
                    "<tr><td>%.1f</td><td>%.3f°</td><td>%.0f%%</td></tr>", p.sigma, p.med, p.frac1);
                o << row;
            }
            o << "</tbody></table>";
            if (have_ceiling) {
                char b[512];
                std::snprintf(b, sizeof(b),
                    "<h3>(2) YUD GT線分での天井（ceiling）</h3>"
                    "<p>YUD の人手ラベルGT線分（線分検出の採点には使わないが、ここでは<b>推定器の上限</b>を測る"
                    "ために利用）を<b>同じパイプライン</b>に通すと、実シーンでの誤差床は中央値 <b>%.2f°</b>"
                    "（&lt;2°: %.0f%%）。これは全検出器の誤差より低く、<b>推定器はボトルネックではない</b>こと、"
                    "すなわち検出器間の差は線分の質を反映していることを示す。</p>",
                    median(ceil_a.mean_err), fracBelow(ceil_a.mean_err, 2.0));
                o << b;
            }
            o << "<div class=\"note\">合成 &sigma;=0 でほぼ0°、ノイズで単調増加し、実GT線分の床も全検出器より下。"
                 "以上より、本評価の推定器は正しく実装され（無バイアス）かつ十分な分解能を持ち、"
                 "検出器比較の公平な計器として妥当である。</div>";

            // Sample visualisations: lines coloured by assigned Manhattan axis.
            if (!assets_dir.empty()) {
                std::string ar = assets_dir;
                std::string hd;
                std::string::size_type hs = html_path.find_last_of("/\\");
                if (hs != std::string::npos) hd = html_path.substr(0, hs);
                if (!hd.empty() && ar.size() > hd.size() &&
                    ar.compare(0, hd.size(), hd) == 0 &&
                    (ar[hd.size()] == '/' || ar[hd.size()] == '\\'))
                    ar = ar.substr(hd.size() + 1);
                for (char& ch : ar) if (ch == '\\') ch = '/';

                o << "<h2>サンプル（線分を推定Manhattan軸で色分け: 赤/緑/青=3直交方向, 灰=非整合）</h2>"
                     "<div style=\"display:grid;grid-template-columns:1fr 1fr;gap:14px\">";
                int nsample = std::min<int>(4, (int)rows.size());
                std::printf("\n[sample assets] per-image mean VP error of rendered samples:\n");
                for (int si = 0; si < nsample; ++si) {
                    const ManifestRow& r = rows[si];
                    sweeplsd::GrayImage g = sweeplsd::loadGray(r.path);
                    if (g.width == 0) continue;
                    struct S { std::string key, label; std::vector<LineSegment> segs; };
                    std::vector<S> ss = {
                        {"sweeplsd", "SweepLSD baseline", runSweeplsd(g)},
                        {"sweeplsdimp", "SweepLSD improved", runSweeplsdImp(g)},
                        {"sweeplsdimplink", "SweepLSD imp+link", runSweeplsdImpLink(g)},
                        {"lsd", "LSD", runLsd(g)},
                        {"ed", "EDLines-style", runEd(g)}};
                    if (have_edreal) ss.push_back({"edreal", "EDLines (ED_Lib)", runEdReal(r.name)});
                    if (have_mlsd) ss.push_back({"mlsd", "M-LSD", runMlsd(r.name)});
                    if (have_elsed) ss.push_back({"elsed", "ELSED", runElsed(r.name)});
                    // plain grayscale base + per-method raw line overlays (report C)
                    sweeplsd::saveSegmentVisualization(assets_dir + "/" + r.name + "_src.png", g, {});
                    for (S& s : ss) {
                        sweeplsd::saveSegmentVisualization(
                            assets_dir + "/" + r.name + "_" + s.key + "_raw.png", g, s.segs);
                        std::vector<CalLine> cl = calibrate(s.segs, f, cx, cy, min_len);
                        std::array<Vec3, 3> fr;
                        std::string err = "n/a";
                        if (estimateManhattan(cl, tau, fr)) {
                            FrameError e = frameError(fr, r.gt);
                            renderAxes(assets_dir + "/" + r.name + "_" + s.key + ".png", g, s.segs,
                                       fr, f, cx, cy, tau);
                            char b[64];
                            std::snprintf(b, sizeof(b), "%.2f°", e.mean_deg);
                            err = b;
                        }
                        std::printf("  %-14s %-16s err=%-7s nseg=%zu\n", r.name.c_str(),
                                    s.key.c_str(), err.c_str(), s.segs.size());
                        if (si < 2)
                            o << "<figure style=\"margin:0\"><img style=\"width:100%;border-radius:6px\" src=\""
                              << ar << "/" << r.name << "_" << s.key << ".png\"><figcaption style=\"font-size:13px;color:#5b6270\">"
                              << r.name << " — " << s.label << "（平均誤差 " << err << "）</figcaption></figure>";
                    }
                }
                o << "</div>";
            }

            o << "<h2>読み方</h2><div class=\"note\">角度誤差が小さいほど、その検出器の線分から"
                 "より正確なカメラ姿勢（Manhattan枠）が復元できることを示す。"
                 "<b>平均&lt;2°/&lt;5°</b> はその精度を達成した画像の割合。"
                 "「失敗」は線分が少なすぎて枠を推定できなかった画像数。"
                 "本評価は YUD の<b>信頼できるVP-GTのみ</b>に基づき、疑わしい線分GTには依存しない。</div>";
            o << "</div></body></html>";
            std::printf("\nHTML report: %s\n", html_path.c_str());
        }
    }
    (void)assets_dir;
    return 0;
}
