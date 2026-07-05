// Example application: calibrated Manhattan-frame (3 orthogonal vanishing
// directions = camera rotation w.r.t. the scene) from SweepLSD segments.
//
// Uses ONLY the public sweeplsd library. This is the RECOMMENDED
// vanishing-point configuration for SweepLSD, selected by measurement on the
// York Urban (outdoor) and NYU (indoor) datasets:
//
//   * detector: Params::improved(), default thresholds
//   * each line VOTES ONCE (unit weight) — length-weighted voting lets long
//     clutter contours (furniture, shadows) hijack the search in indoor
//     scenes, and SweepLSD's support is many short-but-accurate lines
//   * strong candidate search (16k line pairs, 25 seeds)
//   * a vertical-prior seed: photos are usually roughly upright, so the best
//     near-vertical candidate always enters the seed list
//
// With this estimator SweepLSD measured best-in-class on indoor NYU (median
// 5.13 deg vs LSD 7.66 / EDLines 7.64) and on-par outdoors — see
// docs/vp_evaluation.html for the full protocol.
//
// Geometry: normalize endpoints with K^-1 to camera rays; a segment's
// interpretation-plane normal is n = r0 x r1; a vanishing direction d
// satisfies d . n = 0 for every line pointing at it. Candidates come from
// pairs (d = n_i x n_j), axes are refined as the smallest eigenvector of the
// inlier normals' scatter, and the triad is kept orthonormal.
//
// Usage: sweeplsd_manhattan <image> --focal F [--cx X --cy Y] [out.png]
//   (cx, cy default to the image centre)

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

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

// Smallest-eigenvalue eigenvector of a symmetric 3x3 (Jacobi rotations).
Vec3 smallestEigenvector(double A[3][3]) {
    double M[3][3], V[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    std::copy(&A[0][0], &A[0][0] + 9, &M[0][0]);
    for (int sweep = 0; sweep < 50; ++sweep) {
        int p = 0, q = 1;
        double mx = std::fabs(M[0][1]);
        if (std::fabs(M[0][2]) > mx) { mx = std::fabs(M[0][2]); p = 0; q = 2; }
        if (std::fabs(M[1][2]) > mx) { mx = std::fabs(M[1][2]); p = 1; q = 2; }
        if (mx < 1e-15) break;
        double th = 0.5 * std::atan2(2 * M[p][q], M[q][q] - M[p][p]);
        double c = std::cos(th), s = std::sin(th);
        double Mpp = M[p][p], Mqq = M[q][q], Mpq = M[p][q];
        M[p][p] = c * c * Mpp - 2 * s * c * Mpq + s * s * Mqq;
        M[q][q] = s * s * Mpp + 2 * s * c * Mpq + c * c * Mqq;
        M[p][q] = M[q][p] = 0;
        for (int k = 0; k < 3; ++k) {
            if (k != p && k != q) {
                double Mkp = M[k][p], Mkq = M[k][q];
                M[k][p] = M[p][k] = c * Mkp - s * Mkq;
                M[k][q] = M[q][k] = s * Mkp + c * Mkq;
            }
            double Vkp = V[k][p], Vkq = V[k][q];
            V[k][p] = c * Vkp - s * Vkq;
            V[k][q] = s * Vkp + c * Vkq;
        }
    }
    int m = 0;
    for (int i = 1; i < 3; ++i)
        if (M[i][i] < M[m][m]) m = i;
    return normalize({V[0][m], V[1][m], V[2][m]});
}

// A calibrated line: the interpretation-plane normal. Each line votes once
// (unit weight) — the measured-best scheme for this detector (see header).
struct CalLine { Vec3 n; };

// Inlier count of axis d (|n.d| < tau means the line points at d).
int axisScore(const std::vector<CalLine>& lines, const Vec3& d, double tau) {
    int s = 0;
    for (const CalLine& l : lines)
        if (std::fabs(dot(l.n, d)) < tau) ++s;
    return s;
}

// Re-fit one axis as the direction most perpendicular to its inlier normals.
Vec3 refitAxis(const std::vector<CalLine>& lines, const Vec3& d, double tau) {
    double S[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    int cnt = 0;
    for (const CalLine& l : lines) {
        if (std::fabs(dot(l.n, d)) > tau) continue;
        const Vec3& n = l.n;
        S[0][0] += n.x * n.x; S[0][1] += n.x * n.y; S[0][2] += n.x * n.z;
        S[1][1] += n.y * n.y; S[1][2] += n.y * n.z; S[2][2] += n.z * n.z;
        ++cnt;
    }
    if (cnt < 2) return d;
    S[1][0] = S[0][1]; S[2][0] = S[0][2]; S[2][1] = S[1][2];
    return smallestEigenvector(S);
}

std::array<Vec3, 3> orthonormalize(Vec3 a, Vec3 b) {
    a = normalize(a);
    b = normalize({b.x - dot(b, a) * a.x, b.y - dot(b, a) * a.y, b.z - dot(b, a) * a.z});
    Vec3 c = normalize(cross(a, b));
    return {a, b, c};
}

std::array<Vec3, 3> refineTriad(const std::vector<CalLine>& lines,
                                std::array<Vec3, 3> tri, double tau) {
    for (int iter = 0; iter < 8; ++iter) {
        std::array<Vec3, 3> R{refitAxis(lines, tri[0], tau),
                              refitAxis(lines, tri[1], tau),
                              refitAxis(lines, tri[2], tau)};
        int sc[3];
        for (int a = 0; a < 3; ++a) sc[a] = axisScore(lines, R[a], tau);
        int order[3] = {0, 1, 2};
        std::sort(order, order + 3, [&](int a, int b) { return sc[a] > sc[b]; });
        tri = orthonormalize(R[order[0]], R[order[1]]);
    }
    return tri;
}

// Estimate the orthogonal Manhattan frame. Strong search (16k candidate pairs,
// 25 seeds) plus a vertical-prior seed; both measurably reduce the frame
// search failures that dominate indoor error.
bool estimateManhattan(const std::vector<CalLine>& lines, double tau,
                       std::array<Vec3, 3>& out) {
    const int N = (int)lines.size();
    if (N < 8) return false;
    std::mt19937 rng(12345);
    std::uniform_int_distribution<int> pick(0, N - 1);

    std::vector<Vec3> cand;
    int pairs = std::min(16000, N * (N - 1) / 2);
    cand.reserve(pairs);
    for (int t = 0; t < pairs; ++t) {
        int i = pick(rng), j = pick(rng);
        if (i == j) continue;
        Vec3 d = cross(lines[i].n, lines[j].n);
        if (norm(d) < 1e-6) continue;
        cand.push_back(normalize(d));
    }
    if (cand.empty()) return false;

    std::vector<int> sc(cand.size());
    for (std::size_t i = 0; i < cand.size(); ++i) sc[i] = axisScore(lines, cand[i], tau);
    std::vector<int> idx(cand.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&](int a, int b) { return sc[a] > sc[b]; });

    std::vector<Vec3> seeds;
    // Vertical prior: photos are usually roughly upright, so make sure the
    // strongest near-vertical candidate is tried as a first axis.
    for (int ii : idx)
        if (std::fabs(cand[ii].y) > 0.90) { seeds.push_back(cand[ii]); break; }
    for (int ii : idx) {
        const Vec3& d = cand[ii];
        bool dup = false;
        for (const Vec3& s : seeds)
            if (std::fabs(dot(d, s)) > 0.99) { dup = true; break; }
        if (!dup) seeds.push_back(d);
        if ((int)seeds.size() >= 25) break;
    }

    double best_total = -1;
    bool found = false;
    for (const Vec3& d1 : seeds) {
        // best near-orthogonal second axis among the candidates
        Vec3 d2{0, 0, 0};
        int best2 = -1;
        for (std::size_t ci = 0; ci < cand.size(); ++ci) {
            if (std::fabs(dot(cand[ci], d1)) > 0.26) continue;  // > ~75 deg apart
            if (sc[ci] > best2) { best2 = sc[ci]; d2 = cand[ci]; }
        }
        if (best2 < 0) {
            Vec3 t = std::fabs(d1.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
            d2 = normalize(cross(d1, t));
        }
        std::array<Vec3, 3> tri = refineTriad(lines, orthonormalize(d1, d2), tau);
        double total = 0;
        for (const CalLine& l : lines) {
            double best = 1e9;
            for (const Vec3& d : tri) best = std::min(best, std::fabs(dot(l.n, d)));
            if (best < tau) total += 1.0;
        }
        if (total > best_total) { best_total = total; out = tri; found = true; }
    }
    return found;
}

void drawLine(std::vector<unsigned char>& rgb, int w, int h, int x0, int y0, int x1, int y1,
              unsigned char r, unsigned char g, unsigned char b) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1, err = dx + dy;
    for (;;) {
        if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h) {
            std::size_t i = (std::size_t(y0) * w + x0) * 3;
            rgb[i] = r; rgb[i + 1] = g; rgb[i + 2] = b;
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string input, output;
    double focal = 0, cx = -1, cy = -1, min_len = 12.0, tau_deg = 2.0;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--focal" && i + 1 < argc) focal = std::atof(argv[++i]);
        else if (a == "--cx" && i + 1 < argc) cx = std::atof(argv[++i]);
        else if (a == "--cy" && i + 1 < argc) cy = std::atof(argv[++i]);
        else if (a == "--min-len" && i + 1 < argc) min_len = std::atof(argv[++i]);
        else if (a == "--tau" && i + 1 < argc) tau_deg = std::atof(argv[++i]);
        else if (!a.empty() && a[0] != '-') (input.empty() ? input : output) = a;
    }
    if (input.empty() || focal <= 0) {
        std::printf("Usage: %s <image> --focal F [--cx X --cy Y] [out.png]\n"
                    "  F = focal length in pixels; cx,cy default to the image centre\n",
                    argv[0]);
        return 1;
    }

    sweeplsd::GrayImage src = sweeplsd::loadGray(input);
    if (src.width == 0) { std::printf("Error: cannot load '%s'\n", input.c_str()); return 1; }
    if (cx < 0) cx = 0.5 * src.width;
    if (cy < 0) cy = 0.5 * src.height;

    std::vector<sweeplsd::LineSegment> segs =
        sweeplsd::detect(src, sweeplsd::Params::improved());

    // Calibrate: endpoint rays via K^-1, interpretation-plane normal per line.
    auto rayOf = [&](double x, double y) {
        return normalize({(x - cx) / focal, (y - cy) / focal, 1.0});
    };
    std::vector<CalLine> lines;
    std::vector<int> line_seg;  // index back into segs, for the visualization
    for (int i = 0; i < (int)segs.size(); ++i) {
        const auto& s = segs[i];
        if (std::hypot(s.x1 - s.x0, s.y1 - s.y0) < min_len) continue;
        Vec3 n = cross(rayOf(s.x0, s.y0), rayOf(s.x1, s.y1));
        if (norm(n) < 1e-9) continue;
        lines.push_back({normalize(n)});
        line_seg.push_back(i);
    }
    std::printf("Image: %s (%dx%d), f=%.1fpx | %zu segments, %zu used\n", input.c_str(),
                src.width, src.height, focal, segs.size(), lines.size());

    const double tau = std::sin(tau_deg * kPi / 180.0);
    std::array<Vec3, 3> frame;
    if (!estimateManhattan(lines, tau, frame)) {
        std::printf("Not enough lines to estimate a Manhattan frame.\n");
        return 1;
    }
    for (int k = 0; k < 3; ++k)
        std::printf("  axis %d: (%+.4f, %+.4f, %+.4f)  inliers=%d\n", k, frame[k].x,
                    frame[k].y, frame[k].z, axisScore(lines, frame[k], tau));

    // Visualization: segments coloured by their Manhattan axis (gray = none).
    if (output.empty()) {
        auto dot_pos = input.find_last_of('.');
        output = (dot_pos == std::string::npos ? input : input.substr(0, dot_pos)) +
                 "_manhattan.png";
    }
    const int w = src.width, h = src.height;
    std::vector<unsigned char> rgb(std::size_t(w) * h * 3);
    for (int i = 0; i < w * h; ++i) {
        unsigned char g = (unsigned char)(src.data[i] / 3);
        rgb[i * 3] = rgb[i * 3 + 1] = rgb[i * 3 + 2] = g;
    }
    const unsigned char col[4][3] = {
        {235, 64, 52}, {52, 200, 80}, {66, 135, 245}, {130, 130, 130}};
    for (std::size_t li = 0; li < lines.size(); ++li) {
        int a = 3;
        double best = tau;
        for (int k = 0; k < 3; ++k) {
            double r = std::fabs(dot(lines[li].n, frame[k]));
            if (r < best) { best = r; a = k; }
        }
        const auto& s = segs[line_seg[li]];
        drawLine(rgb, w, h, (int)std::lround(s.x0), (int)std::lround(s.y0),
                 (int)std::lround(s.x1), (int)std::lround(s.y1), col[a][0], col[a][1],
                 col[a][2]);
    }
    if (!sweeplsd::saveRgbPng(output, w, h, rgb)) {
        std::printf("Error writing %s\n", output.c_str());
        return 1;
    }
    std::printf("Output: %s\n", output.c_str());
    return 0;
}
