// Dump the software detector's finalized segments as text (x0 y0 x1 y1 per
// line, float) — the "pure SW" column of the 3-level comparison renders
// (SW vs HLS-C vs RTL). Improved mode to match the corpus vectors and the
// board configuration.
//
//   dump_segments <image.png> > <name>_sw.txt

#include <cstdio>

#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: dump_segments <image.png>\n");
        return 2;
    }
    sweeplsd::GrayImage img = sweeplsd::loadGray(argv[1]);
    if (img.width == 0) {
        std::fprintf(stderr, "cannot load %s\n", argv[1]);
        return 1;
    }
    sweeplsd::Params params = sweeplsd::Params::improved();
    std::vector<sweeplsd::LineSegment> segs = sweeplsd::detect(img, params);
    for (const auto& s : segs)
        std::printf("%.4f %.4f %.4f %.4f\n", s.x0, s.y0, s.x1, s.y1);
    std::fprintf(stderr, "%s: %zu segments\n", argv[1], segs.size());
    return 0;
}
