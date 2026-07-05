#include <cstring>
#include <vector>

#include "kernels.hpp"
#include "sweeplsd/sweeplsd.hpp"
#include "stages.hpp"

// Streaming, single-pass SweepLSD — the line-buffer / ring-buffer design from the
// thesis (§3.3, §3.4), expressed readably.
//
// The whole pipeline runs as ONE downward sweep of the image. For each input
// row we advance every stage by one row, keeping only a few rows of each
// intermediate alive in a small ring buffer (memory is O(width), not the whole
// image). Because each stage needs a few neighbour rows, the stages run with a
// fixed vertical lag; when input row y has just arrived we can finalise:
//
//   gaussian row y-2   (needs src rows   y-4..y)
//   gradient row y-3   (needs gaussian   y-3..y-2)
//   edge     row y-4   (needs gradient   y-5..y-3)
//   feature  row y-6   (needs edge       y-8..y-4)
//   labeling row y-7   (needs feature    y-8..y-6)
//
// so labelling trails the input by 7 rows. After the last input row we keep
// sweeping with empty (zero) rows to flush the bottom of the image. Every
// per-pixel computation reuses the shared kernels in kernels.hpp, so the output
// is identical to the multi-pass detect().
//
// New rows carried to the labeling lag: the H/V direction row (interprets the
// sub-pixel offset axis) and the sub-pixel offset row itself. Both are
// 1 byte/px, so the O(width) memory claim is untouched.

namespace sweeplsd {

namespace {

// A ring buffer holding the most recent `cap` rows of one field. Rows are
// produced in increasing order; a slot remembers which absolute row it holds so
// stale rows read back as the zero value.
template <class T>
struct RowRing {
    int width, cap;
    std::vector<T> buf;
    std::vector<int> tag;  // absolute row index stored in each slot, or -1

    RowRing(int w, int c) : width(w), cap(c), buf(std::size_t(w) * c), tag(c, -1) {}
    int slot(int y) const { int s = y % cap; return s < 0 ? s + cap : s; }
    T* mutableRow(int y) { int s = slot(y); tag[s] = y; return &buf[std::size_t(s) * width]; }
    const T* rowPtr(int y) const {  // pointer to row y, or nullptr if not currently held
        if (y < 0) return nullptr;
        int s = slot(y);
        return tag[s] == y ? &buf[std::size_t(s) * width] : nullptr;
    }
};

}  // namespace

std::vector<LineSegment> detectOnePass(const GrayImage& src, const Params& params) {
    const int w = src.width, h = src.height;
    if (w == 0 || h == 0) return {};

    // Ring buffers, each just big enough for the neighbour rows its consumer
    // needs (capacities rounded up for clarity/safety).
    RowRing<std::uint8_t> srcR(w, 8);    // gaussian reads 5 rows
    RowRing<std::uint16_t> gaussR(w, 4); // gradient reads 2 rows
    RowRing<std::uint16_t> powR(w, 10);  // edge reads 3 rows; kept until labelling (7-row lag)
                                         // for weight_by_gradient and the hysteresis strong count
    RowRing<std::uint8_t> dirR(w, 10);   // edge reads 1 row; kept until labelling for the
                                         // sub-pixel interpretation (H/V axis)
    RowRing<std::uint8_t> edgeR(w, 8);   // feature reads 5 rows
    RowRing<std::int8_t> deltaR(w, 10);  // sub-pixel offsets, kept until labelling
    RowRing<std::uint8_t> featR(w, 4);   // labelling reads 3 rows (Feature as uint8)

    std::vector<std::uint16_t> vert_row(w);            // scratch for the gaussian vertical pass
    const std::vector<std::uint8_t> zero_row(w, 0);    // 8-bit stand-in for rows outside the image
    const std::vector<std::uint16_t> zero_row16(w, 0); // 16-bit stand-in (gaussian/power rows)

    kernels::AdaptiveLowTh adapt;  // same per-row update order as the multi-pass

    Labeler labeler(w, h, params);
    const std::vector<Feature> none_row(w, Feature::None);  // stand-in for rows outside the image
    auto featRow = [&](int rr) {
        const std::uint8_t* p = featR.rowPtr(rr);
        return p ? reinterpret_cast<const Feature*>(p) : none_row.data();
    };

    auto inImage = [&](int r) { return r >= 0 && r < h; };

    for (int y = 0; y < h + 7; ++y) {
        if (y < h)  // ingest source row y (rows are contiguous in GrayImage)
            std::memcpy(srcR.mutableRow(y), &src.at(0, y), std::size_t(w));

        if (int r = y - 2; inImage(r)) {  // gaussian (separable: vertical then horizontal)
            auto srcRow = [&](int ry) {
                const std::uint8_t* p = srcR.rowPtr(ry);
                return p ? p : zero_row.data();
            };
            kernels::gaussianVerticalRow(srcRow(r - 2), srcRow(r - 1), srcRow(r), srcRow(r + 1),
                                         srcRow(r + 2), w, vert_row.data());
            kernels::gaussianHorizontalRow(vert_row.data(), r, w, h, gaussR.mutableRow(r));
        }
        if (int r = y - 3; inImage(r)) {  // gradient (power + direction)
            auto grow = [&](int rr) {
                const std::uint16_t* p = gaussR.rowPtr(rr);
                return p ? p : zero_row16.data();
            };
            kernels::gradientRow(grow(r), grow(r + 1), w, powR.mutableRow(r),
                                 dirR.mutableRow(r));
        }
        if (int r = y - 4; inImage(r)) {  // edge (+ optional sub-pixel offsets)
            auto prow = [&](int rr) {
                const std::uint16_t* p = powR.rowPtr(rr);
                return p ? p : zero_row16.data();
            };
            int edge_th = params.gradient_power_th;
            if (params.use_hysteresis) {
                if (params.hysteresis_adaptive) {
                    adapt.update(prow(r), w);
                    edge_th = adapt.lowTh(params.hysteresis_low_th, params.gradient_power_th);
                } else {
                    edge_th = params.hysteresis_low_th;
                }
            }
            const std::uint8_t* dr = dirR.rowPtr(r);
            std::uint8_t* er = edgeR.mutableRow(r);
            kernels::edgeRow(prow(r - 1), prow(r), prow(r + 1), dr, w, edge_th,
                             params.nms_strict_tiebreak, er);
            if (params.subpixel_nms)
                kernels::nmsSubpixelRow(prow(r - 1), prow(r), prow(r + 1), dr, er, w,
                                        deltaR.mutableRow(r));
        }
        if (int r = y - 6; inImage(r)) {  // endpoint candidates
            auto erow = [&](int rr) {
                const std::uint8_t* p = edgeR.rowPtr(rr);
                return p ? p : zero_row.data();
            };
            kernels::featureRow(erow(r - 2), erow(r - 1), erow(r), erow(r + 1), erow(r + 2), w,
                                params.sparse_feature_scan, featR.mutableRow(r));
        }
        if (int r = y - 7; inImage(r)) {  // labelling + line judgment
            labeler.processRow(r, featRow(r - 1), featRow(r), featRow(r + 1), powR.rowPtr(r),
                               dirR.rowPtr(r),
                               params.subpixel_nms ? deltaR.rowPtr(r) : nullptr);
        }
    }
    return labeler.takeSegments();
}

}  // namespace sweeplsd
