// Software twin of the Atlys demo: renders the exact scene of
// rtl/boards/atlys/pattern_gen.v (bit-for-bit — same integer comparisons),
// runs the baseline software detector on it, and writes
//   <outdir>/scene.png            the scene as the hardware displays it
//   <outdir>/scene_sw_overlay.png detect() segments drawn hardware-style
//                                 (green, half-resolution mask) for a direct
//                                 visual comparison with a photo of the board
//   <outdir>/scene_sw_vis.png     the library's standard visualization
// and prints the segment count (= what the board's LEDs show).
//
// Usage: demo_scene <outdir> [width height]   (default 1280 720)

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"

using namespace sweeplsd;

// pattern_gen.v, transliterated (1280x720 scene, v2a)
static std::uint8_t scene(int x, int y) {
    bool b_h = (y >= 89) && (y <= 91) && (x >= 90) && (x <= 840);
    bool b_v = (x >= 119) && (x <= 121) && (y >= 150) && (y <= 630);
    int d45 = x - y - 150;
    bool b_45 = (d45 >= -2) && (d45 <= 2) && (x >= 210) && (x <= 690);
    int d23 = 2 * x + 3 * y - 2250;
    bool b_23 = (d23 >= -6) && (d23 <= 6) && (x >= 150) && (x <= 900);
    int d52 = 5 * x - 2 * y - 2100;
    bool b_52 = (d52 >= -9) && (d52 <= 9) && (y >= 90) && (y <= 660);
    int cdx = x - 720, cdy = y - 450;
    int c_d2 = cdx * cdx + cdy * cdy - 18225;
    bool b_circ = (c_d2 >= -540) && (c_d2 <= 540);
    return (b_h | b_v | b_45 | b_23 | b_52 | b_circ) ? 220 : 30;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: demo_scene <outdir> [w h]\n");
        return 1;
    }
    const std::string dir = argv[1];
    const int w = argc > 3 ? std::atoi(argv[2]) : 1280;
    const int h = argc > 3 ? std::atoi(argv[3]) : 720;

    GrayImage img(w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) img.at(x, y) = scene(x, y);
    saveGrayPng(dir + "/scene.png", img);

    Params params;  // baseline — exactly what the bitstream hardcodes
    std::vector<LineSegment> segs = detect(img, params);
    std::printf("baseline detect(): %zu segments (board LEDs show %zu & 0xFF = %zu)\n",
                segs.size(), segs.size(), segs.size() & 0xFF);

    // Hardware-style overlay: green half-resolution mask over the scene,
    // drawn with integer Bresenham on the RAW endpoint contacts — the same
    // rendering the board does (overlay_mask.v), for photo comparison.
    // detect() returns sub-pixel fitted endpoints; the board draws raw
    // contacts, so tiny differences at segment tips are expected.
    std::vector<std::uint8_t> mask(std::size_t(w / 2) * (h / 2), 0);
    auto plot = [&](int mx, int my) {
        if (mx >= 0 && mx < w / 2 && my >= 0 && my < h / 2)
            mask[std::size_t(my) * (w / 2) + mx] = 1;
    };
    for (const LineSegment& s : segs) {
        // (v2c j) the board draws endpoints at (c+1)>>1 — the mask cell
        // nearest the true edge position c+0.5 (overlay_mask.v cmap).
        int x0 = (int(s.x0) + 1) >> 1, y0 = (int(s.y0) + 1) >> 1;
        int x1 = (int(s.x1) + 1) >> 1, y1 = (int(s.y1) + 1) >> 1;
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        int x = x0, y = y0;
        while (true) {
            plot(x, y);
            if (x == x1 && y == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x += sx; }
            if (e2 <= dx) { err += dx; y += sy; }
        }
    }
    std::vector<unsigned char> rgb(std::size_t(w) * h * 3);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            std::uint8_t g = img.at(x, y);
            std::size_t i = (std::size_t(y) * w + x) * 3;
            if (mask[std::size_t(y / 2) * (w / 2) + (x / 2)]) {
                rgb[i] = 32; rgb[i + 1] = 255; rgb[i + 2] = 32;
            } else {
                rgb[i] = g; rgb[i + 1] = g; rgb[i + 2] = g;
            }
        }
    saveRgbPng(dir + "/scene_sw_overlay.png", w, h, rgb);

    saveSegmentVisualization(dir + "/scene_sw_vis.png", img, segs);
    return 0;
}
