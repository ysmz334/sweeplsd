#pragma once

#include <cfloat>
#include <cmath>
#include <vector>

#include "../src/backend.hpp"
#include "sweeplsd/sweeplsd.hpp"

// Host-side finalisation of hardware SegmentRecords: the single
// floating-point step of the algorithm (projecting the raw endpoint contacts
// onto the PCA axis of the label's scatter), kept off-chip on purpose
// (DESIGN.md). Line-for-line replica of the accepted path of
// sweeplsd::Labeler::Impl::judgeAndRegister + fitEndpoints
// (../../src/labeling.cpp), baseline configuration — same expressions in the
// same order, so given a bit-exact record the resulting LineSegment is
// bit-identical to the software detector's. All the integer sums are far
// below 2^53, so their double images are exact.
//
// Host-only: this header is not synthesized.

namespace sweeplsd_hls {

// `endpoint_from_bbox` (improvement f) chooses the endpoint pair as the
// projection extremes among the record's four bbox extreme points and the two
// contacts — the same candidate order and strict comparisons as the golden
// model (ties keep the earlier candidate). `lattice_half_shift` (improvement
// j) translates the finished segment by (+0.5, +0.5) back into pixel-centre
// coordinates. Both default off = the baseline behaviour.
inline sweeplsd::LineSegment finalizeRecord(const SegmentRecord& r,
                                            bool endpoint_from_bbox = false,
                                            bool lattice_half_shift = false) {
    const double W = double(r.n);
    const double x_sum = double(r.x_sum), y_sum = double(r.y_sum);
    const double x_sq_sum = double(r.x_sq_sum), y_sq_sum = double(r.y_sq_sum);
    const double xy_sum = double(r.xy_sum);

    const double mux = x_sum / W, muy = y_sum / W;
    const double ma = x_sq_sum * W - x_sum * x_sum;
    const double mb = xy_sum * W - x_sum * y_sum;
    const double mc = y_sq_sum * W - y_sum * y_sum;

    const double theta = 0.5 * std::atan2(2.0 * mb, ma - mc);
    const double dx = std::cos(theta), dy = std::sin(theta);

    double p0x = double(r.sx), p0y = double(r.sy);
    double p1x = double(r.ex), p1y = double(r.ey);
    if (endpoint_from_bbox) {
        const double cand[6][2] = {
            {double(r.sx), double(r.sy)},         {double(r.ex), double(r.ey)},
            {double(r.min_x), double(r.min_x_y)}, {double(r.max_x), double(r.max_x_y)},
            {double(r.min_y_x), double(r.min_y)}, {double(r.max_y_x), double(r.max_y)},
        };
        double tmin = DBL_MAX, tmax = -DBL_MAX;
        for (const auto& c : cand) {
            const double t = (c[0] - mux) * dx + (c[1] - muy) * dy;
            if (t < tmin) { tmin = t; p0x = c[0]; p0y = c[1]; }
            if (t > tmax) { tmax = t; p1x = c[0]; p1y = c[1]; }
        }
    }

    auto proj = [&](double px, double py, float& ox, float& oy) {
        const double t = (px - mux) * dx + (py - muy) * dy;
        ox = float(mux + t * dx);
        oy = float(muy + t * dy);
    };
    sweeplsd::LineSegment s;
    proj(p0x, p0y, s.x0, s.y0);
    proj(p1x, p1y, s.x1, s.y1);
    if (lattice_half_shift) {
        s.x0 += 0.5f; s.y0 += 0.5f;
        s.x1 += 0.5f; s.y1 += 0.5f;
    }
    return s;
}

// Drain a record stream (up to and including the n==0 terminator) into
// finalised segments, capped like the software pushOut (Params::max_segments).
inline std::vector<sweeplsd::LineSegment> finalizeStream(hls::stream<SegmentRecord>& in,
                                                         int max_segments,
                                                         bool endpoint_from_bbox = false,
                                                         bool lattice_half_shift = false) {
    std::vector<sweeplsd::LineSegment> segs;
    while (true) {
        SegmentRecord r = in.read();
        if (r.n == 0) break;  // terminator
        if (int(segs.size()) < max_segments)
            segs.push_back(finalizeRecord(r, endpoint_from_bbox, lattice_half_shift));
    }
    return segs;
}

}  // namespace sweeplsd_hls
