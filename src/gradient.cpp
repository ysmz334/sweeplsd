#include <cstdlib>
#include <vector>

#include "kernels.hpp"
#include "stages.hpp"

// Stage 1b — gradient power and direction (thesis §3.2.1.2-3.2.1.4). Row-driven
// over the shared branch-free kernel (kernels.hpp) so it auto-vectorizes.
// The direction is quantised to horizontal / vertical.

namespace sweeplsd {

static_assert(sizeof(EdgeDir) == 1, "gradientRow writes EdgeDir as bytes");

GradientField computeGradient(const Grid<std::uint16_t>& g, const Params& /*params*/) {
    const int w = g.width, h = g.height;
    const std::vector<std::uint16_t> zero(w, 0);  // stand-in for the row past the bottom edge
    auto row = [&](int y) { return (y >= 0 && y < h) ? &g.at(0, y) : zero.data(); };

    GradientField out;
    out.power = Grid<std::uint16_t>(w, h);
    out.dir = Grid<EdgeDir>(w, h, EdgeDir::Horizontal);
    for (int y = 0; y < h; ++y)
        kernels::gradientRow(row(y), row(y + 1), w, &out.power.at(0, y),
                             reinterpret_cast<std::uint8_t*>(&out.dir.at(0, y)));
    return out;
}

}  // namespace sweeplsd
