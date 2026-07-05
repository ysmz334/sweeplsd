#include "kernels.hpp"
#include "stages.hpp"

// Stage 1a — Gaussian smoothing (thesis §3.2.1.1). Thin loop over the shared
// per-pixel kernel; see kernels.hpp for the kernel and the fixed-point details.

namespace sweeplsd {

Grid<std::uint16_t> gaussianBlur(const GrayImage& src) {
    const int w = src.width, h = src.height;
    const std::vector<std::uint8_t> zero(w, 0);  // stand-in for rows outside the image
    auto srcRow = [&](int y) { return (y >= 0 && y < h) ? &src.at(0, y) : zero.data(); };

    // Separable: vertical pass into a one-row scratch, horizontal pass straight
    // into the output row. Only the current row's vertical sums are ever needed,
    // so materialising a full-image vertical intermediate would just add one
    // full-image write + read of memory traffic.
    Grid<std::uint16_t> out(w, h);
    std::vector<std::uint16_t> vert(w);
    for (int y = 0; y < h; ++y) {
        kernels::gaussianVerticalRow(srcRow(y - 2), srcRow(y - 1), srcRow(y), srcRow(y + 1),
                                     srcRow(y + 2), w, vert.data());
        kernels::gaussianHorizontalRow(vert.data(), y, w, h, &out.at(0, y));
    }
    return out;
}

}  // namespace sweeplsd
