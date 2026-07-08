#include <vector>

#include "kernels.hpp"
#include "stages.hpp"

// Stage 1c — threshold + non-maximum suppression + AND (thesis §3.2.1.5-3.2.1.7).
// Row-driven over the shared branch-free kernel (kernels.hpp) so it auto-vectorizes.
// With Params::use_hysteresis the threshold here is the LOW one; the per-label
// strong-pixel requirement (the other half of the hysteresis) lives in the
// labeling stage. With Params::subpixel_nms a second sparse sweep fills the
// per-edge-pixel sub-pixel offset row (improvement c).

namespace sweeplsd {

EdgeField extractEdges(const GradientField& grad, const Params& params) {
    const int w = grad.power.width, h = grad.power.height;
    const Grid<std::uint16_t>& p = grad.power;
    const std::vector<std::uint16_t> zero(w, 0);  // stand-in for rows outside the image
    auto prow = [&](int y) { return (y >= 0 && y < h) ? &p.at(0, y) : zero.data(); };
    const bool hyst = params.use_hysteresis;
    kernels::AdaptiveLowTh adapt;
    // (d) adaptive low threshold with a TWO-row lag: row y is thresholded with
    // the histogram of rows < y-1. The extra row (vs a one-row lag) is what
    // lets the FPGA run its 64-bin percentile scan over a full row instead of
    // the ~10-cycle inter-row flush gap; the decay is 1/256 per row so the
    // shift is sub-LSB. `th_pending` carries lowTh(H_{y-1}) to row y+1.
    int th_pending = params.hysteresis_low_th;

    EdgeField out;
    out.edge = Grid<std::uint8_t>(w, h, 0);
    if (params.subpixel_nms) out.delta = Grid<std::int8_t>(w, h, 0);
    for (int y = 0; y < h; ++y) {
        int th = params.gradient_power_th;
        if (hyst) {
            if (params.hysteresis_adaptive) {
                // Two-row lag (see the th_pending comment above): use the
                // threshold computed one row ago (= lowTh(H_{y-2})), then stage
                // lowTh(H_{y-1}) for the next row, then fold row y. Bit-identical
                // in both drivers and to the RTL histogram module.
                th = th_pending;
                th_pending = adapt.lowTh(params.hysteresis_low_th, params.gradient_power_th);
                adapt.update(prow(y), w);
            } else {
                th = params.hysteresis_low_th;
            }
        }
        const std::uint8_t* dir = reinterpret_cast<const std::uint8_t*>(&grad.dir.at(0, y));
        kernels::edgeRow(prow(y - 1), prow(y), prow(y + 1), dir, w, th,
                         params.nms_strict_tiebreak, &out.edge.at(0, y));
        if (params.subpixel_nms)
            kernels::nmsSubpixelRow(prow(y - 1), prow(y), prow(y + 1), dir,
                                    &out.edge.at(0, y), w, &out.delta.at(0, y));
    }
    return out;
}

}  // namespace sweeplsd
