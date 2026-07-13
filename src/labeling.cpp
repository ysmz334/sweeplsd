#include <algorithm>
#include <cfloat>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>

#include "stages.hpp"

// Stages 3 & 4 — connected-component labelling and line judgment
// (thesis §3.2.3 and §3.2.4).
//
// Edge "interior" pixels are grouped into labelled regions; endpoint candidates
// act as the cuts between regions. Each label accumulates the scatter moments
// (N, Sw, Sx, Sy, Sxx, Syy, Sxy) needed for the PCA test. When an interior
// pixel touches an endpoint candidate it records the segment start; the second
// such contact closes the segment and submits it to the line judgment.
//
// The work is driven one image row at a time (`Labeler::processRow`) so the
// same code serves both the multi-pass and the streaming one-pass detector.
//
// New in this version (accuracy improvements c-f):
//  - moments are accumulated in double so the sub-pixel NMS offsets
//    (improvement c) can be added directly; integer sums up to 2^53 stay exact,
//    so the unweighted classic path is numerically unchanged;
//  - per-label count of "strong" pixels for the streaming hysteresis
//    (improvement d);
//  - per-label histogram of the 4 direction classes for the coherence gate
//    (improvement e);
//  - per-label bounding-box extreme points so endpoints can be re-derived from
//    the projection extremes instead of the first/last contacts (improvement f);
//  - the NFA density estimate can forget exponentially (improvement g).

namespace sweeplsd {

namespace {

// Hot/cold split for cache locality. `LabelHot` is touched for every interior
// pixel (union-find resolve + moment accumulation); `LabelCold` only when a
// pixel touches an endpoint candidate, or on a merge.
struct LabelHot {
    int connect = 0;    // union-find parent; 0 = representative (root)
    int last_row = -1;  // recency (row, then x), used to pick the merge survivor
    int latest_x = -1;
    int pix_num = 0;            // unweighted pixel count (for thresholds / NFA)
    std::uint32_t strong_cnt = 0;   // pixels with power >= high threshold (hysteresis)
    double w_sum = 0;           // total weight Sigma w (== pix_num when unweighted)
    // weighted moments: Sigma(w*x), Sigma(w*x^2), ... (w = 1 unless weight_by_gradient)
    double x_sum = 0, x_sq_sum = 0, y_sum = 0, y_sq_sum = 0, xy_sum = 0;
    // bbox extreme points (integer pixel coords): the pixel that attained each
    // extreme, so improvement f can re-derive the endpoints from projections.
    int min_x = INT_MAX, min_x_y = 0, max_x = INT_MIN, max_x_y = 0;
    int min_y = INT_MAX, min_y_x = 0, max_y = INT_MIN, max_y_x = 0;
};
struct LabelCold {
    bool has_start = false;  // first endpoint of the segment has been seen
    int start_x = 0, start_y = 0;
};

struct LabelTable {
    std::vector<LabelHot> hot{LabelHot{}};    // index 0 = reserved "no label" sentinel
    std::vector<LabelCold> cold{LabelCold{}};
    int create() {
        hot.emplace_back();
        cold.emplace_back();
        return int(hot.size()) - 1;
    }
    // Union-find resolve with path compression.
    int find(int id) {
        int root = id;
        while (hot[root].connect != 0) root = hot[root].connect;
        while (hot[id].connect != 0) {
            int next = hot[id].connect;
            hot[id].connect = root;
            id = next;
        }
        return root;
    }
    LabelHot& h(int id) { return hot[id]; }
    LabelCold& c(int id) { return cold[id]; }
};

// Upper tail of the binomial: P(X >= k), X ~ Bin(n, p), in log space.
double binomialTail(int n, int k, double p) {
    if (k <= 0) return 1.0;
    if (k > n) return DBL_MIN;
    double logp = std::log(p), logq = std::log(1.0 - p);
    double logBin = std::lgamma(n + 1.0);
    double sum = 0.0;
    for (int j = k; j <= n; ++j) {
        double logterm = logBin - std::lgamma(j + 1.0) - std::lgamma(n - j + 1.0) +
                         j * logp + (n - j) * logq;
        double term = std::exp(logterm);
        sum += term;
        if (j > k && term < sum * 1e-12) break;
    }
    return sum > 0.0 ? sum : DBL_MIN;
}

double dirAngle(float ax, float ay, float bx, float by) {
    double d = std::fabs(double(ax) * bx + double(ay) * by);
    if (d > 1.0) d = 1.0;
    return std::acos(d);
}
double dist(float ax, float ay, float bx, float by) {
    return std::hypot(double(ax) - bx, double(ay) - by);
}

// Sub-pixel offset (dx, dy) in px for one edge pixel: delta is the 1/16 px
// offset along the NMS axis of the H/V direction `cls` (see kernels.hpp).
inline void subpixelOffset(int cls, std::int8_t delta, double& ox, double& oy) {
    constexpr double kInv16 = 1.0 / 16.0;
    double d = double(delta);
    if (cls == 1) { ox = d * kInv16; oy = 0.0; }        // Vertical edge: x offset
    else          { ox = 0.0;        oy = d * kInv16; } // Horizontal edge: y offset
}

}  // namespace

struct Labeler::Impl {
    int width, height;
    Params params;
    LabelTable labels;
    std::vector<int> prev_row;  // representative label per pixel, previous row
    std::vector<int> cur_row;
    std::vector<LineSegment> segments;  // finalized output
    bool want_ex = false;               // also collect per-segment scatter stats
    std::vector<LineSegmentEx> segments_ex;

    // Streaming NFA state (improvement 4): running edge-pixel density. Doubles
    // so improvement g (exponential forgetting) is a two-line change.
    double edge_count = 0, pixel_count = 0;
    int cur_y = 0;

    // Collinear-linking state (improvement 5).
    struct Active {
        LineSegment seg;
        float dirx, diry;
        int max_y;
        bool prov;  // provisional: every part was admitted below pixel_num_th
    };
    std::vector<Active> active;

    Impl(int w, int h, const Params& p, bool ex = false)
        : width(w), height(h), params(p), prev_row(w, 0), cur_row(w, 0), want_ex(ex) {}

    // Improvement 1: project the two raw endpoints onto the least-squares line
    // (centroid + principal direction (dx, dy) from the moments), so the
    // reported endpoints lie exactly on the fitted line — sub-pixel and
    // consistent.
    LineSegment fitEndpoints(double mux, double muy, double dx, double dy,
                             double sx, double sy, double ex, double ey) {
        auto proj = [&](double px, double py, float& ox, float& oy) {
            double t = (px - mux) * dx + (py - muy) * dy;
            ox = float(mux + t * dx);
            oy = float(muy + t * dy);
        };
        LineSegment s;
        proj(sx, sy, s.x0, s.y0);
        proj(ex, ey, s.x1, s.y1);
        return s;
    }

    // Improvement 4 (+g): a-contrario gate against the running edge density.
    bool passesNfa(int N, const LineSegment& s) {
        if (!params.use_nfa) return true;
        if (pixel_count <= 0) return true;
        double p = edge_count / pixel_count;
        if (p <= 0.0 || p >= 1.0) return true;
        double len = std::max(std::fabs(s.x1 - s.x0), std::fabs(s.y1 - s.y0)) + 1.0;
        int n = int(std::lround(len));
        int k = std::min(N, n);
        double log_nt = 2.5 * std::log10(double(width) * double(height));
        double log_nfa = log_nt + std::log10(binomialTail(n, k, p));
        return -log_nfa >= params.nfa_epsilon;
    }

    void pushOut(const LineSegment& s, const LineSegmentEx* ex = nullptr) {
        if (int(segments.size()) < params.max_segments) {
            segments.push_back(s);
            if (want_ex && ex) segments_ex.push_back(*ex);
        }
    }

    // Two-stage length threshold (see Params::link_admit_pix): fragments were
    // admitted into the linker below pixel_num_th; a chain that is still
    // provisional (no part ever cleared pixel_num_th on its own) is kept only
    // if its Chebyshev SPAN clears the threshold — the chain must evidence
    // actual length. Span, not accumulated pixel count, is the deliberate
    // choice: a pixel-count gate re-admits dense wiggly chains of noise
    // fragments (many pixels, short span) and measurably floods precision at
    // high noise. Anything the judge accepted at the full threshold is never
    // dropped here.
    void flushActive(const Active& a) {
        if (a.prov) {
            double cheb = std::max(std::fabs(double(a.seg.x1) - a.seg.x0),
                                   std::fabs(double(a.seg.y1) - a.seg.y0));
            if (cheb + 1.0 < double(params.pixel_num_th)) return;
        }
        pushOut(a.seg);
    }

    // Improvement 5: try to extend `s` by joining a collinear active segment.
    void emit(LineSegment s, bool prov, const LineSegmentEx* ex = nullptr) {
        if (!params.link_collinear) {
            pushOut(s, ex);
            return;
        }
        const double max_ang = params.link_max_angle_deg * 3.14159265358979 / 180.0;
        const int band = int(std::ceil(params.link_max_gap)) + 1;
        // flush un-extendable actives
        std::vector<Active> keep;
        keep.reserve(active.size());
        for (const Active& a : active) {
            if (a.max_y < cur_y - band) flushActive(a);
            else keep.push_back(a);
        }
        active.swap(keep);

        auto unit = [](const LineSegment& g, float& ux, float& uy) {
            double dx = double(g.x1) - g.x0, dy = double(g.y1) - g.y0;
            double L = std::hypot(dx, dy);
            ux = L > 0 ? float(dx / L) : 1.0f;
            uy = L > 0 ? float(dy / L) : 0.0f;
        };
        float sdx, sdy;
        unit(s, sdx, sdy);

        bool linked = true;
        while (linked) {
            linked = false;
            for (std::size_t i = 0; i < active.size(); ++i) {
                const Active& t = active[i];
                if (dirAngle(sdx, sdy, t.dirx, t.diry) > max_ang) continue;
                float se[2][2] = {{s.x0, s.y0}, {s.x1, s.y1}};
                float te[2][2] = {{t.seg.x0, t.seg.y0}, {t.seg.x1, t.seg.y1}};
                // Lateral consistency (see Params::link_lateral_tol): the
                // candidate must lie on s's LINE, not merely be parallel to
                // it, and vice versa — otherwise a gap wider than a thin
                // bar's width fuses the bar's two parallel flanks.
                {
                    double lat = 0;
                    for (int b = 0; b < 2; ++b) {
                        double vx = double(te[b][0]) - s.x0, vy = double(te[b][1]) - s.y0;
                        lat = std::max(lat, std::fabs(vx * sdy - vy * sdx));
                    }
                    for (int a2 = 0; a2 < 2; ++a2) {
                        double vx = double(se[a2][0]) - t.seg.x0, vy = double(se[a2][1]) - t.seg.y0;
                        lat = std::max(lat, std::fabs(vx * t.diry - vy * t.dirx));
                    }
                    if (lat > params.link_lateral_tol) continue;
                }
                double gap = 1e30;
                for (int a = 0; a < 2; ++a)
                    for (int b = 0; b < 2; ++b)
                        gap = std::min(gap, dist(se[a][0], se[a][1], te[b][0], te[b][1]));
                if (gap > params.link_max_gap) continue;
                LineSegment m;
                double best = -1;
                float pts[4][2] = {{se[0][0], se[0][1]}, {se[1][0], se[1][1]},
                                   {te[0][0], te[0][1]}, {te[1][0], te[1][1]}};
                for (int a = 0; a < 4; ++a)
                    for (int b = a + 1; b < 4; ++b) {
                        double d = dist(pts[a][0], pts[a][1], pts[b][0], pts[b][1]);
                        if (d > best) { best = d; m = {pts[a][0], pts[a][1], pts[b][0], pts[b][1]}; }
                    }
                float mdx, mdy;
                unit(m, mdx, mdy);
                if (dirAngle(mdx, mdy, sdx, sdy) > max_ang ||
                    dirAngle(mdx, mdy, t.dirx, t.diry) > max_ang)
                    continue;
                s = m;
                sdx = mdx;
                sdy = mdy;
                prov = prov && t.prov;
                active.erase(active.begin() + i);
                linked = true;
                break;
            }
        }
        int my = int(std::lround(std::max(s.y0, s.y1)));
        active.push_back({s, sdx, sdy, my, prov});
    }

    // Line judgment (thesis §3.2.4): enough pixels (criterion 1) + the new
    // hysteresis gate + elongated enough scatter (criterion 4, PCA), then
    // optional NFA gate, then emit.
    void judgeAndRegister(const LabelHot& L, int sx, int sy, int ex, int ey) {
        // Two-stage threshold: with linking on, admit smaller fragments into
        // the linker; pixel_num_th is enforced on the linked chain at flush.
        const int admit_th = params.link_collinear
                                 ? std::min(params.pixel_num_th, params.link_admit_pix)
                                 : params.pixel_num_th;
        if (L.pix_num < admit_th) return;
        if (params.use_hysteresis && int(L.strong_cnt) < params.hysteresis_strong_min) return;
        // (i) border margin: drop a segment whose bounding box reaches within
        // border_margin px of the frame (the 2x2 gradient bias fringes the image
        // edge). A pure integer bbox test on the label's own extremes, so it is
        // trivially bit-exact across SW / HLS / RTL.
        if (params.border_margin > 0 && L.min_x != INT_MAX &&
            (L.min_x < params.border_margin || L.max_x >= width - params.border_margin ||
             L.min_y < params.border_margin || L.max_y >= height - params.border_margin))
            return;

        // Centroid from the (weighted) moments; W == pix_num when unweighted.
        double W = L.w_sum;
        if (W <= 0.0) return;
        double mux = L.x_sum / W, muy = L.y_sum / W;
        double ma, mb, mc;
        if (params.weight_by_gradient) {
            // Normalised covariance (bounded; safe for large weights).
            ma = L.x_sq_sum / W - mux * mux;
            mb = L.xy_sum / W - mux * muy;
            mc = L.y_sq_sum / W - muy * muy;
        } else {
            // Un-normalised form (W == N); same scatter test as the original.
            ma = L.x_sq_sum * W - L.x_sum * L.x_sum;
            mb = L.xy_sum * W - L.x_sum * L.y_sum;
            mc = L.y_sq_sum * W - L.y_sum * L.y_sum;
        }
        double trace = ma + mc;
        double root = std::sqrt((ma - mc) * (ma - mc) + 4.0 * mb * mb);
        double denom = trace + root;
        if (denom <= 0.0) return;  // degenerate scatter
        if ((trace - root) / denom > params.aspect_th) return;  // not elongated

        // Improvement h: absolute perpendicular-spread (curve) rejection. The
        // smaller eigenvalue of the *normalised* covariance is the perpendicular
        // variance in px^2; a curved arc bows from its chord and inflates it.
        // (The aspect_th ratio above misses short low-curvature arcs because
        // their ev_max is small too.) Integer-friendly: ev_min <= th^2.
        if (params.max_perp_spread > 0.0) {
            double ncxx = L.x_sq_sum / W - mux * mux;
            double ncxy = L.xy_sum / W - mux * muy;
            double ncyy = L.y_sq_sum / W - muy * muy;
            double ntr = ncxx + ncyy;
            double nrt = std::sqrt((ncxx - ncyy) * (ncxx - ncyy) + 4.0 * ncxy * ncxy);
            double ev_min_norm = 0.5 * (ntr - nrt);
            if (ev_min_norm > params.max_perp_spread * params.max_perp_spread) return;
        }

        // Major-axis direction of the scatter, shared by the endpoint choice
        // below and the endpoint projection in fitEndpoints.
        const double theta = 0.5 * std::atan2(2.0 * mb, ma - mc);
        const double dx = std::cos(theta), dy = std::sin(theta);

        // Improvement f: choose the endpoint pair as the projection extremes
        // among the bbox extreme points and the two recorded contacts.
        double p0x = sx, p0y = sy, p1x = ex, p1y = ey;
        if (params.endpoint_from_bbox && L.min_x != INT_MAX) {
            const double cand[6][2] = {
                {double(sx), double(sy)},         {double(ex), double(ey)},
                {double(L.min_x), double(L.min_x_y)}, {double(L.max_x), double(L.max_x_y)},
                {double(L.min_y_x), double(L.min_y)}, {double(L.max_y_x), double(L.max_y)},
            };
            double tmin = DBL_MAX, tmax = -DBL_MAX;
            for (const auto& c : cand) {
                double t = (c[0] - mux) * dx + (c[1] - muy) * dy;
                if (t < tmin) { tmin = t; p0x = c[0]; p0y = c[1]; }
                if (t > tmax) { tmax = t; p1x = c[0]; p1y = c[1]; }
            }
        }

        LineSegment s = fitEndpoints(mux, muy, dx, dy, p0x, p0y, p1x, p1y);  // sub-pixel

        // Improvement j: the moments were accumulated on the gradient lattice,
        // which sits at the pixel CORNERS (x+0.5, y+0.5); translate the finished
        // segment back into pixel-centre coordinates (same fix as canonical LSD).
        const float lshift = params.lattice_half_shift ? 0.5f : 0.0f;
        s.x0 += lshift; s.y0 += lshift;
        s.x1 += lshift; s.y1 += lshift;
        if (!passesNfa(L.pix_num, s)) return;

        if (!want_ex) { emit(s, L.pix_num < params.pixel_num_th); return; }
        // Per-segment scatter statistics: normalised covariance (variance in
        // px^2) regardless of the weighting branch, so eigenvalues are
        // physically meaningful.
        double cxx = L.x_sq_sum / W - mux * mux;
        double cxy = L.xy_sum / W - mux * muy;
        double cyy = L.y_sq_sum / W - muy * muy;
        double tr = cxx + cyy, rt = std::sqrt((cxx - cyy) * (cxx - cyy) + 4.0 * cxy * cxy);
        double theta_n = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);  // from the normalised form
        LineSegmentEx e;
        e.seg = s;
        e.pix_num = int(L.pix_num);
        e.cx = float(mux) + lshift;  // centroid in the same (pixel-centre) frame
        e.cy = float(muy) + lshift;
        e.dir_x = float(std::cos(theta_n));
        e.dir_y = float(std::sin(theta_n));
        e.ev_max = float(0.5 * (tr + rt));
        e.ev_min = float(0.5 * (tr - rt));
        emit(s, L.pix_num < params.pixel_num_th, &e);
    }

    void accumulate(LabelHot& L, int x, int y, double w, double fx, double fy, bool strong) {
        L.pix_num += 1;
        L.strong_cnt += strong;
        L.w_sum += w;
        L.x_sum += w * fx;
        L.x_sq_sum += w * fx * fx;
        L.y_sum += w * fy;
        L.y_sq_sum += w * fy * fy;
        L.xy_sum += w * fx * fy;
        if (x < L.min_x) { L.min_x = x; L.min_x_y = y; }
        if (x > L.max_x) { L.max_x = x; L.max_x_y = y; }
        if (y < L.min_y) { L.min_y = y; L.min_y_x = x; }
        if (y > L.max_y) { L.max_y = y; L.max_y_x = x; }
        L.last_row = y;
        L.latest_x = x;
    }

    // Merge two labels, combining moments; returns the surviving label id. If
    // both halves already hold a start endpoint, the merge closes a segment.
    int merge(int l0, int l1) {
        LabelHot& a = labels.h(l0);
        LabelHot& b = labels.h(l1);
        bool keep0 = std::make_pair(a.last_row, a.latest_x) >= std::make_pair(b.last_row, b.latest_x);
        int main_id = keep0 ? l0 : l1;
        int other_id = keep0 ? l1 : l0;
        LabelHot& mh = labels.h(main_id);

        mh.pix_num = a.pix_num + b.pix_num;
        mh.strong_cnt = a.strong_cnt + b.strong_cnt;
        mh.w_sum = a.w_sum + b.w_sum;
        mh.x_sum = a.x_sum + b.x_sum;
        mh.x_sq_sum = a.x_sq_sum + b.x_sq_sum;
        mh.y_sum = a.y_sum + b.y_sum;
        mh.y_sq_sum = a.y_sq_sum + b.y_sq_sum;
        mh.xy_sum = a.xy_sum + b.xy_sum;
        const LabelHot& oh = labels.h(other_id);
        if (oh.min_x < mh.min_x) { mh.min_x = oh.min_x; mh.min_x_y = oh.min_x_y; }
        if (oh.max_x > mh.max_x) { mh.max_x = oh.max_x; mh.max_x_y = oh.max_x_y; }
        if (oh.min_y < mh.min_y) { mh.min_y = oh.min_y; mh.min_y_x = oh.min_y_x; }
        if (oh.max_y > mh.max_y) { mh.max_y = oh.max_y; mh.max_y_x = oh.max_y_x; }
        labels.h(other_id).connect = main_id;

        LabelCold& mc = labels.c(main_id);
        LabelCold& oc = labels.c(other_id);
        if (mc.has_start && oc.has_start) {
            judgeAndRegister(mh, labels.c(l0).start_x, labels.c(l0).start_y,
                             labels.c(l1).start_x, labels.c(l1).start_y);
        } else if (!mc.has_start && oc.has_start) {
            mc.has_start = true;
            mc.start_x = oc.start_x;
            mc.start_y = oc.start_y;
        }
        return main_id;
    }

    void processRow(int y, const Feature* above, const Feature* cur, const Feature* below,
                    const std::uint16_t* power, const std::uint8_t* dir,
                    const std::int8_t* delta) {
        std::fill(cur_row.begin(), cur_row.end(), 0);
        cur_y = y;
        const bool weighted = params.weight_by_gradient && power != nullptr;
        const bool hysteresis = params.use_hysteresis && power != nullptr;
        const bool subpix = params.subpixel_nms && delta != nullptr && dir != nullptr;
        const bool sparse = params.sparse_label_scan;
        const int strong_th = params.gradient_power_th;

        if (params.use_nfa) {  // running edge density for the a-contrario test
            if (params.nfa_window_rows > 0) {  // improvement g: forget exponentially
                double f = 1.0 - 1.0 / double(params.nfa_window_rows);
                edge_count *= f;
                pixel_count *= f;
            }
            pixel_count += width;
            int x = 0;
            for (; x + 8 <= width; x += 8) {  // zero-word skip (Feature::None == 0)
                std::uint64_t wd;
                std::memcpy(&wd, cur + x, 8);
                if (wd == 0) continue;
                for (int i = 0; i < 8; ++i) edge_count += (cur[x + i] != Feature::None);
            }
            for (; x < width; ++x) edge_count += (cur[x] != Feature::None);
        }

        // Per-pixel body, generated in two flavours: Checked=true for the two
        // border columns (neighbour reads clamp to the image; identical to the
        // original bounds-checked accesses), Checked=false for the interior
        // (1 <= x <= width-2), where all eight neighbours are in range and read
        // as plain array accesses.
        auto pixel = [&](int x, auto checked) {
            // `checked` is std::true_type / std::false_type; branch on its type
            // directly (a `constexpr bool` local tripped MSVC's constant
            // evaluation inside the nested lambdas).
            using Checked = decltype(checked);
            auto feat = [&](const Feature* row, int xx) -> int {
                if constexpr (Checked::value)
                    return (xx >= 0 && xx < width) ? int(row[xx]) : int(Feature::None);
                else
                    return int(row[xx]);
            };
            auto lab = [&](const std::vector<int>& r, int xx) -> int {
                if constexpr (Checked::value) return (xx >= 0 && xx < width) ? r[xx] : 0;
                else return r[xx];
            };
            // All eight neighbours, read once and reused by both the label
            // gather and the endpoint-contact test below.
            const int aL = feat(above, x - 1), aC = int(above[x]), aR = feat(above, x + 1);
            const int cL = feat(cur, x - 1), cR = feat(cur, x + 1);
            const int bL = feat(below, x - 1), bC = int(below[x]), bR = feat(below, x + 1);
            constexpr int kInt = int(Feature::Interior), kEnd = int(Feature::Endpoint);

            // Causal 8-neighbours that can already carry a label: NE, N, NW
            // (previous row = `above`) and W (this row = `cur`).
            struct Nb { bool edge; int label; };
            Nb neigh[4] = {
                {aR == kInt, lab(prev_row, x + 1)},  // NE
                {aC == kInt, prev_row[x]},           // N
                {aL == kInt, lab(prev_row, x - 1)},  // NW
                {cL == kInt, lab(cur_row, x - 1)},   // W
            };

            int edge_num = 0, label0 = 0, label1 = 0;
            for (const Nb& nb : neigh) {
                if (!nb.edge) continue;
                ++edge_num;
                int lr = labels.find(nb.label);
                if (label0 == 0) label0 = lr;
                else if (lr != label0 && label1 == 0) label1 = lr;
            }

            int center;
            if (edge_num == 0) center = labels.create();
            else if (label1 == 0) center = label0;
            else center = merge(label0, label1);

            double fx = x, fy = y;
            if (subpix && delta[x] != 0) {
                double ox, oy;
                subpixelOffset(dir[x] & 1, delta[x], ox, oy);
                fx += ox;
                fy += oy;
            }
            accumulate(labels.h(center), x, y, weighted ? double(power[x]) : 1.0, fx, fy,
                       hysteresis && power[x] >= strong_th);
            cur_row[x] = center;

            // Endpoint contact: Feature::Endpoint is the only value with bit 1
            // set, so OR-ing the eight neighbours and masking is the branch-free
            // equivalent of the eight == comparisons.
            const bool touches_end = ((aL | aC | aR | cL | cR | bL | bC | bR) & kEnd) != 0;
            if (touches_end) {
                LabelCold& Lc = labels.c(center);
                if (Lc.has_start) judgeAndRegister(labels.h(center), Lc.start_x, Lc.start_y, x, y);
                else { Lc.has_start = true; Lc.start_x = x; Lc.start_y = y; }
            }
        };

        int x = 0;
        while (x < width) {
            // Speed: 8 consecutive Feature::None bytes (== zero uint64) cannot
            // contain an Interior pixel — skip the whole word. Exact.
            if (sparse && x + 8 <= width) {
                std::uint64_t wd;
                std::memcpy(&wd, cur + x, 8);
                if (wd == 0) { x += 8; continue; }
            }
            const int x_end = sparse ? std::min(x + 8, width) : width;
            for (; x < x_end; ++x) {
                if (cur[x] != Feature::Interior) continue;
                if (x == 0 || x == width - 1) pixel(x, std::true_type{});
                else pixel(x, std::false_type{});
            }
        }
        prev_row.swap(cur_row);
    }
};

Labeler::Labeler(int width, int height, const Params& params, bool want_extended)
    : impl_(new Impl(width, height, params, want_extended)) {}
Labeler::~Labeler() = default;
void Labeler::processRow(int y, const Feature* above, const Feature* cur, const Feature* below,
                         const std::uint16_t* power, const std::uint8_t* dir,
                         const std::int8_t* delta) {
    impl_->processRow(y, above, cur, below, power, dir, delta);
}
std::vector<LineSegment> Labeler::takeSegments() {
    for (const auto& a : impl_->active) impl_->flushActive(a);  // flush remaining linked segments
    impl_->active.clear();
    return std::move(impl_->segments);
}
std::vector<LineSegmentEx> Labeler::takeSegmentsEx() {
    // Extended collection runs with linking disabled, so `active` is empty here.
    return std::move(impl_->segments_ex);
}

namespace {
template <class Take>
auto runLabeler(const Grid<Feature>& feat, const Params& params,
                const Grid<std::uint16_t>* power, const Grid<EdgeDir>* dir,
                const Grid<std::int8_t>* delta, bool want_ex, Take take) {
    const int w = feat.width, h = feat.height;
    Labeler labeler(w, h, params, want_ex);
    const std::vector<Feature> none(w, Feature::None);  // stand-in for rows outside the image
    auto row = [&](int y) { return (y >= 0 && y < h) ? &feat.at(0, y) : none.data(); };
    auto prow = [&](int y) { return power ? &power->at(0, y) : nullptr; };
    auto drow = [&](int y) {
        return dir ? reinterpret_cast<const std::uint8_t*>(&dir->at(0, y)) : nullptr;
    };
    auto srow = [&](int y) { return (delta && delta->width > 0) ? &delta->at(0, y) : nullptr; };
    for (int y = 0; y < h; ++y)
        labeler.processRow(y, row(y - 1), row(y), row(y + 1), prow(y), drow(y), srow(y));
    return take(labeler);
}
}  // namespace

std::vector<LineSegment> labelAndJudge(const Grid<Feature>& feat, const Params& params,
                                       const Grid<std::uint16_t>* power,
                                       const Grid<EdgeDir>* dir, const Grid<std::int8_t>* delta) {
    return runLabeler(feat, params, power, dir, delta, false,
                      [](Labeler& l) { return l.takeSegments(); });
}

std::vector<LineSegmentEx> labelAndJudgeEx(const Grid<Feature>& feat, const Params& params,
                                           const Grid<std::uint16_t>* power,
                                           const Grid<EdgeDir>* dir,
                                           const Grid<std::int8_t>* delta) {
    // Disable linking so moments map 1:1 to the emitted segments.
    Params p = params;
    p.link_collinear = false;
    return runLabeler(feat, p, power, dir, delta, true,
                      [](Labeler& l) { return l.takeSegmentsEx(); });
}

}  // namespace sweeplsd
