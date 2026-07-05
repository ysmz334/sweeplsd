#pragma once

#include <string>
#include <vector>

#include "sweeplsd/image.hpp"
#include "sweeplsd/sweeplsd.hpp"

namespace sweeplsd {

// Load an image file (PNG/JPG/BMP/...) as 8-bit grayscale via stb_image.
// Returns an empty image (width==0) on failure.
GrayImage loadGray(const std::string& path);

// Render the detected segments as random-coloured lines over a dimmed copy of
// the source image and write it to `path` (format inferred from extension:
// .png/.bmp/.jpg). Returns false on failure.
bool saveSegmentVisualization(const std::string& path, const GrayImage& src,
                              const std::vector<LineSegment>& segments);

// Write a grayscale image / an interleaved RGB buffer as a PNG. Used by the
// stage-dump tool to visualize pipeline intermediates.
bool saveGrayPng(const std::string& path, const GrayImage& img);
bool saveRgbPng(const std::string& path, int width, int height,
                const std::vector<unsigned char>& rgb);

}  // namespace sweeplsd
