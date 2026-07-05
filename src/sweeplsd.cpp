#include "sweeplsd/sweeplsd.hpp"

#include <algorithm>
#include <chrono>

#include "stages.hpp"

// Pipeline orchestration: chain the four stages (thesis §3.2) together. Each
// arrow is one full-image pass over the previous field.
//
//   gray -> gaussian -> gradient -> edge(+delta) -> endpoint candidates -> segments
//
// The labeling stage now also receives the gradient power (hysteresis /
// weighting), the direction map (coherence gate) and the sub-pixel offset map.

namespace sweeplsd {

std::vector<LineSegment> detect(const GrayImage& src, const Params& params) {
    Grid<std::uint16_t> gaussian = gaussianBlur(src);
    GradientField gradient = computeGradient(gaussian, params);
    EdgeField edge = extractEdges(gradient, params);
    Grid<Feature> feat = extractEndpointCandidates(edge.edge, params);
    return labelAndJudge(feat, params, &gradient.power, &gradient.dir,
                         params.subpixel_nms ? &edge.delta : nullptr);
}

std::vector<LineSegmentEx> detectEx(const GrayImage& src, const Params& params) {
    Grid<std::uint16_t> gaussian = gaussianBlur(src);
    GradientField gradient = computeGradient(gaussian, params);
    EdgeField edge = extractEdges(gradient, params);
    Grid<Feature> feat = extractEndpointCandidates(edge.edge, params);
    return labelAndJudgeEx(feat, params, &gradient.power, &gradient.dir,
                           params.subpixel_nms ? &edge.delta : nullptr);
}

namespace {
template <class F>
double medianMs(int runs, F&& fn) {
    std::vector<double> t;
    t.reserve(runs);
    for (int i = 0; i < runs; ++i) {
        auto t0 = std::chrono::steady_clock::now();
        fn();
        auto t1 = std::chrono::steady_clock::now();
        t.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
    std::sort(t.begin(), t.end());
    return t[t.size() / 2];
}
}  // namespace

std::vector<StageTiming> profileStages(const GrayImage& src, const Params& params, int runs) {
    Grid<std::uint16_t> gaussian;
    GradientField gradient;
    EdgeField edge;
    Grid<Feature> feat;
    std::vector<LineSegment> segs;

    double tg = medianMs(runs, [&] { gaussian = gaussianBlur(src); });
    double td = medianMs(runs, [&] { gradient = computeGradient(gaussian, params); });
    double te = medianMs(runs, [&] { edge = extractEdges(gradient, params); });
    double tf = medianMs(runs, [&] { feat = extractEndpointCandidates(edge.edge, params); });
    double tl = medianMs(runs, [&] {
        segs = labelAndJudge(feat, params, &gradient.power, &gradient.dir,
                             params.subpixel_nms ? &edge.delta : nullptr);
    });

    return {{"1. gaussian", tg}, {"2. gradient", td}, {"3. edge (thr+NMS)", te},
            {"4. endpoint feat", tf}, {"5. label+judge", tl}};
}

}  // namespace sweeplsd
