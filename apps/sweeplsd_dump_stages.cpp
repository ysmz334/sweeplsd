#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"
#include "stages.hpp"  // internal pipeline stages (needs src/ on the include path)

// Dump each pipeline intermediate as an image, for the explanatory document
// (docs/sweeplsd_explained.html). Not part of the library/CLI — a teaching aid.

namespace {
sweeplsd::GrayImage scaleToGray(const sweeplsd::Grid<std::uint16_t>& f, int shift) {
    sweeplsd::GrayImage g(f.width, f.height);
    for (int i = 0; i < f.width * f.height; ++i)
        g.data[i] = std::uint8_t(std::min(255, int(f.data[i]) >> shift));
    return g;
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::printf("Usage: %s <input-image> <output-dir>\n", argv[0]);
        return 1;
    }
    std::string in = argv[1], dir = argv[2];
    sweeplsd::GrayImage src = sweeplsd::loadGray(in);
    if (src.width == 0) { std::printf("Error: cannot load %s\n", in.c_str()); return 1; }
    sweeplsd::Params params = sweeplsd::Params::improved();  // the shipped configuration

    // Run the stages explicitly so each intermediate can be visualized.
    auto gaussian = sweeplsd::gaussianBlur(src);
    auto gradient = sweeplsd::computeGradient(gaussian, params);
    auto edge = sweeplsd::extractEdges(gradient, params);
    auto feat = sweeplsd::extractEndpointCandidates(edge.edge, params);
    auto segments = sweeplsd::detect(src, params);

    sweeplsd::saveGrayPng(dir + "/00_source.png", src);
    sweeplsd::saveGrayPng(dir + "/01_gaussian.png", scaleToGray(gaussian, 6));   // undo the x64 scaling
    sweeplsd::saveGrayPng(dir + "/02_power.png", scaleToGray(gradient.power, 2));

    sweeplsd::GrayImage edgeImg(edge.edge.width, edge.edge.height);
    for (int i = 0; i < edge.edge.width * edge.edge.height; ++i)
        edgeImg.data[i] = edge.edge.data[i] ? 255 : 0;
    sweeplsd::saveGrayPng(dir + "/03_edge.png", edgeImg);

    // Feature map: dim grey = interior edge, red = endpoint candidate.
    std::vector<unsigned char> rgb(std::size_t(feat.width) * feat.height * 3, 0);
    for (int i = 0; i < feat.width * feat.height; ++i) {
        if (feat.data[i] == sweeplsd::Feature::Interior) { rgb[i*3] = rgb[i*3+1] = rgb[i*3+2] = 190; }
        else if (feat.data[i] == sweeplsd::Feature::Endpoint) { rgb[i*3] = 255; rgb[i*3+1] = 40; rgb[i*3+2] = 40; }
    }
    sweeplsd::saveRgbPng(dir + "/04_feature.png", feat.width, feat.height, rgb);

    sweeplsd::saveSegmentVisualization(dir + "/05_segments.png", src, segments);
    std::printf("Wrote 6 stage images to %s (%zu segments)\n", dir.c_str(), segments.size());
    return 0;
}
