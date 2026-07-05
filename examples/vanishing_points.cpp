// Example application: vanishing-point estimation from SweepLSD line segments.
//
// Uses ONLY the public sweeplsd library (sweeplsd::detect + sweeplsd::io), to show the
// detector is reusable as a building block. Pipeline:
//   1. detect line segments with SweepLSD,
//   2. keep the longer ones (vanishing-point geometry wants reliable lines),
//   3. find up to K dominant vanishing points by sequential RANSAC on the
//      segments' homogeneous lines, refined by least squares (smallest
//      eigenvector of sum(l l^T) via a 3x3 Jacobi eigensolver),
//   4. print the vanishing points and save a visualization coloured by which
//      vanishing point each segment supports.
//
// A vanishing point v (homogeneous 3-vector; v[2]~0 means a point at infinity =
// a set of parallel lines) is the common intersection of image lines; a line l
// passes through v iff l . v = 0. Inliers are tested by an angle: the direction
// from a segment's midpoint toward v must match the segment's own direction.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"

namespace {

struct Vec3 { double x, y, z; };
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

struct Seg {
    double mx, my;    // midpoint
    double dx, dy;    // unit direction
    double len;
    Vec3 l;           // homogeneous line, normalized to unit 3-norm
    int vp = -1;      // assigned vanishing point index, -1 = none
};

// Direction from a segment's midpoint toward vanishing point v (unit). For a
// point at infinity the direction is just (v.x, v.y).
bool dirTowardVp(const Seg& s, const Vec3& v, double& ux, double& uy) {
    double scale = std::fabs(v.x) + std::fabs(v.y) + 1e-12;
    if (std::fabs(v.z) > 1e-9 * scale) {  // finite vanishing point
        ux = v.x / v.z - s.mx;
        uy = v.y / v.z - s.my;
    } else {  // at infinity (parallel pencil)
        ux = v.x;
        uy = v.y;
    }
    double n = std::hypot(ux, uy);
    if (n < 1e-12) return false;
    ux /= n; uy /= n;
    return true;
}

bool consistent(const Seg& s, const Vec3& v, double cos_th) {
    double ux, uy;
    if (!dirTowardVp(s, v, ux, uy)) return false;
    return std::fabs(ux * s.dx + uy * s.dy) >= cos_th;  // undirected angle
}

// Smallest-eigenvalue eigenvector of a 3x3 symmetric matrix (Jacobi rotations).
Vec3 smallestEigenvector(double A[3][3]) {
    double V[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    double M[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) M[i][j] = A[i][j];
    for (int sweep = 0; sweep < 50; ++sweep) {
        // largest off-diagonal
        int p = 0, q = 1;
        double mx = std::fabs(M[0][1]);
        if (std::fabs(M[0][2]) > mx) { mx = std::fabs(M[0][2]); p = 0; q = 2; }
        if (std::fabs(M[1][2]) > mx) { mx = std::fabs(M[1][2]); p = 1; q = 2; }
        if (mx < 1e-15) break;
        double theta = 0.5 * std::atan2(2 * M[p][q], M[q][q] - M[p][p]);
        double c = std::cos(theta), s = std::sin(theta);
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
    int small = 0;
    for (int i = 1; i < 3; ++i)
        if (M[i][i] < M[small][small]) small = i;
    return {V[0][small], V[1][small], V[2][small]};
}

Vec3 refine(const std::vector<Seg>& segs, const std::vector<int>& inliers) {
    double A[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    for (int idx : inliers) {
        const Vec3& l = segs[idx].l;
        double v[3] = {l.x, l.y, l.z};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) A[i][j] += v[i] * v[j];
    }
    return smallestEigenvector(A);
}

// Bresenham line into an interleaved RGB buffer.
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
    if (argc < 2) {
        std::printf("Usage: %s <input-image> [output.png] [--max-vp K] [--min-len L] [--angle DEG]\n",
                    argv[0]);
        return 1;
    }
    std::string input = argv[1], output;
    int max_vp = 3;
    double min_len = 25.0, angle_deg = 2.0;
    for (int i = 2; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--max-vp" && i + 1 < argc) max_vp = std::atoi(argv[++i]);
        else if (a == "--min-len" && i + 1 < argc) min_len = std::atof(argv[++i]);
        else if (a == "--angle" && i + 1 < argc) angle_deg = std::atof(argv[++i]);
        else if (!a.empty() && a[0] != '-') output = a;
    }
    if (output.empty()) {
        auto dot = input.find_last_of('.');
        output = (dot == std::string::npos ? input : input.substr(0, dot)) + "_vp.png";
    }

    sweeplsd::GrayImage src = sweeplsd::loadGray(input);
    if (src.width == 0) { std::printf("Error: cannot load '%s'\n", input.c_str()); return 1; }

    // 1-2. detect segments (link fragments for longer, more reliable lines) and
    //      keep the long ones.
    sweeplsd::Params params;
    params.link_collinear = true;
    std::vector<sweeplsd::LineSegment> raw = sweeplsd::detect(src, params);

    std::vector<Seg> segs;
    for (const auto& s : raw) {
        double dx = s.x1 - s.x0, dy = s.y1 - s.y0, len = std::hypot(dx, dy);
        if (len < min_len) continue;
        Vec3 a{s.x0, s.y0, 1.0}, b{s.x1, s.y1, 1.0};
        Vec3 l = cross(a, b);
        double ln = std::sqrt(l.x * l.x + l.y * l.y + l.z * l.z);
        if (ln < 1e-12) continue;
        Seg seg;
        seg.mx = 0.5 * (s.x0 + s.x1);
        seg.my = 0.5 * (s.y0 + s.y1);
        seg.dx = dx / len; seg.dy = dy / len; seg.len = len;
        seg.l = {l.x / ln, l.y / ln, l.z / ln};
        segs.push_back(seg);
    }
    std::printf("Image: %s (%dx%d) | segments: %zu total, %zu used (len >= %.0f)\n",
                input.c_str(), src.width, src.height, raw.size(), segs.size(), min_len);

    // 3. sequential RANSAC for up to max_vp vanishing points.
    const double cos_th = std::cos(angle_deg * 3.14159265358979 / 180.0);
    const int kIters = 2000;
    const std::size_t kMinSupport = 6;
    std::mt19937 rng(0);
    std::vector<Vec3> vps;

    std::vector<int> remaining;
    for (int i = 0; i < (int)segs.size(); ++i) remaining.push_back(i);

    for (int v = 0; v < max_vp && remaining.size() >= kMinSupport; ++v) {
        std::uniform_int_distribution<std::size_t> pick(0, remaining.size() - 1);
        std::vector<int> best_inliers;
        for (int it = 0; it < kIters; ++it) {
            std::size_t ia = pick(rng), ib = pick(rng);
            if (ia == ib) continue;
            Vec3 cand = cross(segs[remaining[ia]].l, segs[remaining[ib]].l);
            double nn = std::sqrt(cand.x * cand.x + cand.y * cand.y + cand.z * cand.z);
            if (nn < 1e-12) continue;
            std::vector<int> inl;
            for (int idx : remaining)
                if (consistent(segs[idx], cand, cos_th)) inl.push_back(idx);
            if (inl.size() > best_inliers.size()) best_inliers = std::move(inl);
        }
        if (best_inliers.size() < kMinSupport) break;

        Vec3 vp = refine(segs, best_inliers);  // least-squares over inliers
        // re-collect inliers with the refined vanishing point
        std::vector<int> inliers;
        for (int idx : remaining)
            if (consistent(segs[idx], vp, cos_th)) inliers.push_back(idx);
        if (inliers.size() < kMinSupport) break;

        for (int idx : inliers) segs[idx].vp = v;
        vps.push_back(vp);
        std::vector<int> rest;
        for (int idx : remaining)
            if (segs[idx].vp < 0) rest.push_back(idx);
        remaining.swap(rest);

        if (std::fabs(vp.z) > 1e-9 * (std::fabs(vp.x) + std::fabs(vp.y) + 1e-12))
            std::printf("  VP %d: (%.1f, %.1f)  support=%zu\n", v, vp.x / vp.z, vp.y / vp.z,
                        inliers.size());
        else
            std::printf("  VP %d: at infinity, direction (%.3f, %.3f)  support=%zu\n", v,
                        vp.x / std::hypot(vp.x, vp.y), vp.y / std::hypot(vp.x, vp.y), inliers.size());
    }

    // 4. visualization: segments coloured by vanishing point (gray = unassigned).
    const int w = src.width, h = src.height;
    std::vector<unsigned char> rgb(std::size_t(w) * h * 3);
    for (int i = 0; i < w * h; ++i) { unsigned char g = src.data[i] / 3; rgb[i*3] = rgb[i*3+1] = rgb[i*3+2] = g; }
    const unsigned char palette[][3] = {{235, 64, 52}, {52, 200, 80}, {66, 135, 245},
                                        {240, 200, 40}, {200, 80, 230}};
    // draw the used segments coloured by their assigned vanishing point
    {
        std::size_t si = 0;
        for (const auto& r : raw) {
            double len = std::hypot(r.x1 - r.x0, r.y1 - r.y0);
            if (len < min_len) continue;
            const Seg& seg = segs[si++];
            unsigned char cr = 130, cg = 130, cb = 130;  // unassigned
            if (seg.vp >= 0) { cr = palette[seg.vp % 5][0]; cg = palette[seg.vp % 5][1]; cb = palette[seg.vp % 5][2]; }
            drawLine(rgb, w, h, (int)std::lround(r.x0), (int)std::lround(r.y0),
                     (int)std::lround(r.x1), (int)std::lround(r.y1), cr, cg, cb);
        }
    }
    // mark finite vanishing points that fall near the image
    for (std::size_t v = 0; v < vps.size(); ++v) {
        const Vec3& vp = vps[v];
        if (std::fabs(vp.z) <= 1e-9 * (std::fabs(vp.x) + std::fabs(vp.y) + 1e-12)) continue;
        int px = (int)std::lround(vp.x / vp.z), py = (int)std::lround(vp.y / vp.z);
        if (px < -w || px > 2 * w || py < -h || py > 2 * h) continue;
        for (int d = -6; d <= 6; ++d) {
            drawLine(rgb, w, h, px - 6, py + d, px + 6, py + d, 255, 255, 255);
        }
    }
    if (!sweeplsd::saveRgbPng(output, w, h, rgb)) { std::printf("Error writing %s\n", output.c_str()); return 1; }
    std::printf("Output: %s (%zu vanishing point(s))\n", output.c_str(), vps.size());
    return 0;
}
