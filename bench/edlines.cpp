#include "edlines.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

// Compact EDLines-style detector (see edlines.hpp for scope/caveats):
//   1. Gaussian smooth, Sobel gradient (magnitude + horizontal/vertical class).
//   2. 1-pixel edges via threshold + non-maximum suppression.
//   3. Trace edges into ordered chains (broken at junctions).
//   4. Walk each chain, fitting straight line segments by least squares and
//      splitting where a point deviates by more than `fit_tol` pixels.

namespace edlines {

namespace {

struct Grid {
    int w = 0, h = 0;
    std::vector<float> d;
    Grid(int w_, int h_) : w(w_), h(h_), d(std::size_t(w_) * h_, 0.f) {}
    float& at(int x, int y) { return d[std::size_t(y) * w + x]; }
    float get(int x, int y) const {
        if (x < 0 || y < 0 || x >= w || y >= h) return 0.f;
        return d[std::size_t(y) * w + x];
    }
};

// 5x5 separable gaussian (sigma ~1), normalised, kept as float.
Grid gaussian(const sweeplsd::GrayImage& src) {
    const int w = src.width, h = src.height;
    const float k[5] = {1, 4, 6, 4, 1};  // sum 16
    Grid tmp(w, h), out(w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            float s = 0, wsum = 0;
            for (int dx = -2; dx <= 2; ++dx) {
                if (x + dx < 0 || x + dx >= w) continue;
                s += k[dx + 2] * src.at(x + dx, y);
                wsum += k[dx + 2];
            }
            tmp.at(x, y) = s / wsum;
        }
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            float s = 0, wsum = 0;
            for (int dy = -2; dy <= 2; ++dy) {
                if (y + dy < 0 || y + dy >= h) continue;
                s += k[dy + 2] * tmp.at(x, y + dy);
                wsum += k[dy + 2];
            }
            out.at(x, y) = s / wsum;
        }
    return out;
}

}  // namespace

std::vector<sweeplsd::LineSegment> detect(const sweeplsd::GrayImage& src, const Params& params) {
    const int w = src.width, h = src.height;
    if (w == 0 || h == 0) return {};

    Grid g = gaussian(src);

    // Sobel gradient: gx responds to vertical edges, gy to horizontal edges.
    Grid mag(w, h);
    std::vector<std::uint8_t> vertical(std::size_t(w) * h, 0);  // 1 = vertical edge
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float gx = (g.get(x + 1, y - 1) + 2 * g.get(x + 1, y) + g.get(x + 1, y + 1)) -
                       (g.get(x - 1, y - 1) + 2 * g.get(x - 1, y) + g.get(x - 1, y + 1));
            float gy = (g.get(x - 1, y + 1) + 2 * g.get(x, y + 1) + g.get(x + 1, y + 1)) -
                       (g.get(x - 1, y - 1) + 2 * g.get(x, y - 1) + g.get(x + 1, y - 1));
            mag.at(x, y) = std::fabs(gx) + std::fabs(gy);
            vertical[std::size_t(y) * w + x] = (std::fabs(gx) >= std::fabs(gy)) ? 1 : 0;
        }
    }

    // 1-pixel edges: threshold + non-maximum suppression along the gradient.
    std::vector<std::uint8_t> edge(std::size_t(w) * h, 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float m = mag.at(x, y);
            if (m < params.gradient_th) continue;
            bool is_max = vertical[std::size_t(y) * w + x]
                              ? (m >= mag.get(x - 1, y) && m >= mag.get(x + 1, y))
                              : (m >= mag.get(x, y - 1) && m >= mag.get(x, y + 1));
            if (is_max) edge[std::size_t(y) * w + x] = 1;
        }
    }

    auto isEdge = [&](int x, int y) {
        return x >= 0 && y >= 0 && x < w && y < h && edge[std::size_t(y) * w + x];
    };
    auto degree = [&](int x, int y) {
        int n = 0;
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx)
                if ((dx || dy) && isEdge(x + dx, y + dy)) ++n;
        return n;
    };

    // Trace edges into ordered chains, breaking at junctions (degree >= 3).
    struct Pt { int x, y; };
    std::vector<std::uint8_t> visited(std::size_t(w) * h, 0);
    auto markVisited = [&](int x, int y) { visited[std::size_t(y) * w + x] = 1; };
    auto isVisited = [&](int x, int y) { return visited[std::size_t(y) * w + x] != 0; };

    auto walkFrom = [&](int sx, int sy, std::vector<Pt>& chain) {
        int x = sx, y = sy;
        while (true) {
            markVisited(x, y);
            chain.push_back({x, y});
            if (degree(x, y) >= 3) break;  // stop at a junction
            int nx = -1, ny = -1;
            float best = -1.f;             // prefer the strongest unvisited neighbour
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    if (!dx && !dy) continue;
                    int cx = x + dx, cy = y + dy;
                    if (!isEdge(cx, cy) || isVisited(cx, cy)) continue;
                    if (degree(cx, cy) >= 3) continue;
                    if (mag.at(cx, cy) > best) { best = mag.at(cx, cy); nx = cx; ny = cy; }
                }
            if (nx < 0) break;
            x = nx; y = ny;
        }
    };

    std::vector<std::vector<Pt>> chains;
    // Start at endpoints (degree 1) first so chains run end-to-end, then sweep
    // the rest (closed loops / leftovers).
    for (int pass = 0; pass < 2; ++pass) {
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                if (!isEdge(x, y) || isVisited(x, y)) continue;
                if (degree(x, y) >= 3) continue;
                if (pass == 0 && degree(x, y) != 1) continue;
                std::vector<Pt> chain;
                walkFrom(x, y, chain);
                if ((int)chain.size() >= 2) chains.push_back(std::move(chain));
            }
    }

    // Least-squares line fitting with split-on-deviation.
    std::vector<sweeplsd::LineSegment> out;
    for (const auto& chain : chains) {
        std::size_t start = 0;
        double n = 0, sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
        auto add = [&](const Pt& p) {
            n += 1; sx += p.x; sy += p.y; sxx += double(p.x) * p.x;
            syy += double(p.y) * p.y; sxy += double(p.x) * p.y;
        };
        auto reset = [&]() { n = sx = sy = sxx = syy = sxy = 0; };
        // distance of point p to the current least-squares line.
        auto dist = [&](const Pt& p) -> double {
            double cx = sx / n, cy = sy / n;
            double a = sxx - sx * cx, b = sxy - sx * cy, c = syy - sy * cy;
            double theta = 0.5 * std::atan2(2 * b, a - c);  // principal axis angle
            double nx = -std::sin(theta), ny = std::cos(theta);  // line normal
            return std::fabs((p.x - cx) * nx + (p.y - cy) * ny);
        };
        auto emit = [&](std::size_t i0, std::size_t i1) {
            if (i1 < i0 + std::size_t(params.min_length)) return;
            const Pt& a = chain[i0];
            const Pt& b = chain[i1];
            int dx = b.x - a.x, dy = b.y - a.y;
            if (dx * dx + dy * dy >= params.min_length * params.min_length)
                out.push_back({float(a.x), float(a.y), float(b.x), float(b.y)});
        };

        for (std::size_t i = 0; i < chain.size(); ++i) {
            if (n >= 2 && dist(chain[i]) > params.fit_tol) {
                emit(start, i - 1);
                start = i;
                reset();
            }
            add(chain[i]);
        }
        if (n >= 2) emit(start, chain.size() - 1);
    }
    return out;
}

}  // namespace edlines
