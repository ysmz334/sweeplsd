#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "sweeplsd/image.hpp"
#include "sweeplsd/sweeplsd.hpp"

// Internal declarations for the four pipeline stages. Each is a pure function
// that consumes the previous stage's field(s) and produces the next one, which
// makes the data flow obvious and each stage independently testable. The
// thesis (§3.2.x) names the same four stages.

namespace sweeplsd {

// Quantised gradient direction (thesis §3.2.1.4). SweepLSD distinguishes only
// horizontal vs vertical edges; NMS competes along the dominant gradient axis.
enum class EdgeDir : std::uint8_t {
    Horizontal = 0,  // |dy| dominant: NMS along +-(0,1)
    Vertical = 1,    // |dx| dominant: NMS along +-(1,0)
};

struct GradientField {
    Grid<std::uint16_t> power;   // average of |dx|, |dy| on the gaussian
    Grid<EdgeDir> dir;           // dominant gradient orientation per pixel
};

// Stage-1c output. `delta` is the sub-pixel NMS offset per surviving edge pixel
// (signed, 1/16 px units, along the NMS axis of that pixel's H/V direction);
// only filled when Params::subpixel_nms is set.
struct EdgeField {
    Grid<std::uint8_t> edge;   // 0/1
    Grid<std::int8_t> delta;   // empty unless subpixel_nms
};

// Classification of a pixel in the endpoint-candidate map (thesis §3.2.2).
// Endpoints delimit line-segment regions; interior pixels are what gets labelled.
enum class Feature : std::uint8_t {
    None = 0,      // not an edge
    Interior = 1,  // edge pixel that continues a line
    Endpoint = 2,  // edge pixel that ends / branches / corners a line
};

// Stage 1 — edge extraction (thesis §3.2.1).
Grid<std::uint16_t> gaussianBlur(const GrayImage& src);
GradientField computeGradient(const Grid<std::uint16_t>& gaussian, const Params& params);
EdgeField extractEdges(const GradientField& grad, const Params& params);

// Stage 2 — endpoint-candidate extraction (thesis §3.2.2).
Grid<Feature> extractEndpointCandidates(const Grid<std::uint8_t>& edge, const Params& params);

// Stages 3 & 4 — connected-component labelling with per-label scatter moments,
// segments delimited by endpoint candidates, then PCA-based line judgment
// (thesis §3.2.3 and §3.2.4).
//
// The labeller is row-streaming so it can be driven by either the multi-pass
// detector (over a full Grid) or the one-pass detector (over ring buffers).
// `processRow` advances one image row, given raw pointers to the three feature
// rows it needs — `above` (y-1), `cur` (y), `below` (y+1) — each a contiguous
// array of `width` Features; pass an all-None row for y-1<0 or y+1>=h. Raw
// pointers (rather than a std::function) let the per-pixel accesses inline.
//
// `power` is the gradient-power row for `cur` (may be null) — read when
// Params::weight_by_gradient or Params::use_hysteresis is set.
// `dir` is the H/V direction row for `cur` (may be null) — read when
// Params::subpixel_nms is set (to pick each edge pixel's sub-pixel axis).
// `delta` is the sub-pixel offset row for `cur` (may be null) — read when
// Params::subpixel_nms is set.
class Labeler {
public:
    // `want_extended` makes the labeller also accumulate per-segment scatter
    // statistics (retrievable via takeSegmentsEx); off by default so the normal
    // path has no overhead.
    Labeler(int width, int height, const Params& params, bool want_extended = false);
    ~Labeler();
    void processRow(int y, const Feature* above, const Feature* cur, const Feature* below,
                    const std::uint16_t* power, const std::uint8_t* dir = nullptr,
                    const std::int8_t* delta = nullptr);
    std::vector<LineSegment> takeSegments();
    std::vector<LineSegmentEx> takeSegmentsEx();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Convenience wrapper: run the labeller over a fully materialised feature map.
// `power`/`dir`/`delta` (same size as feat) are only read when the matching
// Params flags are set; pass nullptr otherwise.
std::vector<LineSegment> labelAndJudge(const Grid<Feature>& feat, const Params& params,
                                       const Grid<std::uint16_t>* power = nullptr,
                                       const Grid<EdgeDir>* dir = nullptr,
                                       const Grid<std::int8_t>* delta = nullptr);

// Same, but returns segments with their scatter statistics (see LineSegmentEx).
std::vector<LineSegmentEx> labelAndJudgeEx(const Grid<Feature>& feat, const Params& params,
                                           const Grid<std::uint16_t>* power = nullptr,
                                           const Grid<EdgeDir>* dir = nullptr,
                                           const Grid<std::int8_t>* delta = nullptr);

}  // namespace sweeplsd
