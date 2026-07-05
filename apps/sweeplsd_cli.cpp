#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"

namespace {

void printUsage(const char* prog) {
    std::cout <<
        "SweepLSD line segment detector (readable reproduction)\n"
        "Usage: " << prog << " <input-image> [output-image] [options]\n"
        "  input-image    path to a PNG/JPG/BMP image (read as grayscale)\n"
        "  output-image   visualization path (default: <input>_segments.png)\n"
        "Options (override the reference defaults):\n"
        "  --grad-th N     gradient power threshold     (default 256)\n"
        "  --pix-th N      minimum pixels per segment   (default 16)\n"
        "  --aspect-th F   max PCA eigenvalue ratio      (default 0.0078125)\n"
        "  --nfa           enable a-contrario (NFA) validation (suppresses weak alignments)\n"
        "  --link          enable gap-tolerant collinear linking (joins fragments)\n"
        "  --weight        weight the PCA fit by gradient strength (more accurate direction)\n";
}

std::string defaultOutput(const std::string& input) {
    std::string::size_type dot = input.find_last_of('.');
    std::string base = (dot == std::string::npos) ? input : input.substr(0, dot);
    return base + "_segments.png";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string input = argv[1];
    std::string output;
    sweeplsd::Params params;

    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&](double fallback) { return (i + 1 < argc) ? std::atof(argv[++i]) : fallback; };
        if (arg == "--grad-th") params.gradient_power_th = int(next(params.gradient_power_th));
        else if (arg == "--pix-th") params.pixel_num_th = int(next(params.pixel_num_th));
        else if (arg == "--aspect-th") params.aspect_th = next(params.aspect_th);
        else if (arg == "--nfa") params.use_nfa = true;
        else if (arg == "--link") params.link_collinear = true;
        else if (arg == "--weight") params.weight_by_gradient = true;
        else if (arg.size() && arg[0] != '-') output = arg;
        else { printUsage(argv[0]); return 1; }
    }
    if (output.empty()) output = defaultOutput(input);

    sweeplsd::GrayImage src = sweeplsd::loadGray(input);
    if (src.width == 0) {
        std::cerr << "Error: could not load image '" << input << "'\n";
        return 1;
    }

    auto t0 = std::chrono::steady_clock::now();
    std::vector<sweeplsd::LineSegment> segments = sweeplsd::detect(src, params);
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::cout << "Image:    " << input << " (" << src.width << "x" << src.height << ")\n";
    std::cout << "Segments: " << segments.size() << "\n";
    std::cout << "Time:     " << ms << " ms\n";

    if (!sweeplsd::saveSegmentVisualization(output, src, segments)) {
        std::cerr << "Error: could not write '" << output << "'\n";
        return 1;
    }
    std::cout << "Output:   " << output << "\n";
    return 0;
}
