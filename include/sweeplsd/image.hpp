#pragma once

#include <cstdint>
#include <vector>

namespace sweeplsd {

// Dense row-major 2-D field. Rows are contiguous, so &g.at(0, y) is a valid
// pointer to the whole row y (the row-driven kernels rely on this).
template <class T>
struct Grid {
    int width = 0, height = 0;
    std::vector<T> data;

    Grid() = default;
    Grid(int w, int h) : width(w), height(h), data(std::size_t(w) * h) {}
    Grid(int w, int h, T fill) : width(w), height(h), data(std::size_t(w) * h, fill) {}

    T& at(int x, int y) { return data[std::size_t(y) * width + x]; }
    const T& at(int x, int y) const { return data[std::size_t(y) * width + x]; }
};

using GrayImage = Grid<std::uint8_t>;

}  // namespace sweeplsd
