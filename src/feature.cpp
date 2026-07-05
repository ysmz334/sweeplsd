#include <vector>

#include "kernels.hpp"
#include "stages.hpp"

// Stage 2 — endpoint-candidate extraction (thesis §3.2.2). Row-driven so the
// 5x5 outer-ring thinning (kernels.hpp) auto-vectorizes; see kernels::featureRow.

namespace sweeplsd {

static_assert(sizeof(Feature) == 1, "featureRow writes Feature as bytes");

Grid<Feature> extractEndpointCandidates(const Grid<std::uint8_t>& edge, const Params& params) {
    const int w = edge.width, h = edge.height;
    const std::vector<std::uint8_t> zero(w, 0);  // stand-in for rows outside the image
    auto row = [&](int y) { return (y >= 0 && y < h) ? &edge.at(0, y) : zero.data(); };

    Grid<Feature> feat(w, h, Feature::None);
    for (int y = 0; y < h; ++y)
        kernels::featureRow(row(y - 2), row(y - 1), row(y), row(y + 1), row(y + 2), w,
                            params.sparse_feature_scan,
                            reinterpret_cast<std::uint8_t*>(&feat.at(0, y)));
    return feat;
}

}  // namespace sweeplsd
