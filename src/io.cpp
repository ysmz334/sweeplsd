#include "sweeplsd/io.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace sweeplsd {

GrayImage loadGray(const std::string& path) {
    int w = 0, h = 0, channels = 0;
    unsigned char* pixels = stbi_load(path.c_str(), &w, &h, &channels, 1);  // force 1 channel
    if (!pixels) return GrayImage{};

    GrayImage img(w, h);
    std::memcpy(img.data.data(), pixels, std::size_t(w) * h);
    stbi_image_free(pixels);
    return img;
}

namespace {

// Map a hue in [0,1) to a bright RGB colour (S=V=1).
void hueToRgb(double hue, unsigned char& r, unsigned char& g, unsigned char& b) {
    double h6 = hue * 6.0;
    int sector = int(h6) % 6;
    double frac = h6 - std::floor(h6);
    double x = 1.0 - frac;
    double vals[3] = {0, 0, 0};
    switch (sector) {
        case 0: vals[0] = 1;    vals[1] = frac; vals[2] = 0;    break;
        case 1: vals[0] = x;    vals[1] = 1;    vals[2] = 0;    break;
        case 2: vals[0] = 0;    vals[1] = 1;    vals[2] = frac; break;
        case 3: vals[0] = 0;    vals[1] = x;    vals[2] = 1;    break;
        case 4: vals[0] = frac; vals[1] = 0;    vals[2] = 1;    break;
        default: vals[0] = 1;   vals[1] = 0;    vals[2] = x;    break;
    }
    r = (unsigned char)(vals[0] * 255);
    g = (unsigned char)(vals[1] * 255);
    b = (unsigned char)(vals[2] * 255);
}

// Bresenham line into an interleaved RGB buffer.
void drawLine(std::vector<unsigned char>& rgb, int w, int h, int x0, int y0, int x1, int y1,
              unsigned char r, unsigned char g, unsigned char b) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h) {
            std::size_t idx = (std::size_t(y0) * w + x0) * 3;
            rgb[idx] = r;
            rgb[idx + 1] = g;
            rgb[idx + 2] = b;
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

}  // namespace

bool saveSegmentVisualization(const std::string& path, const GrayImage& src,
                              const std::vector<LineSegment>& segments) {
    const int w = src.width, h = src.height;
    if (w == 0 || h == 0) return false;

    // Dimmed grayscale background so the coloured segments stand out.
    std::vector<unsigned char> rgb(std::size_t(w) * h * 3);
    for (int i = 0; i < w * h; ++i) {
        unsigned char v = (unsigned char)(src.data[i] / 3);
        rgb[i * 3] = rgb[i * 3 + 1] = rgb[i * 3 + 2] = v;
    }

    std::srand(0);  // deterministic colours, like the reference
    for (const LineSegment& s : segments) {
        unsigned char r, g, b;
        hueToRgb((std::rand() % 360) / 360.0, r, g, b);
        drawLine(rgb, w, h, (int)std::lround(s.x0), (int)std::lround(s.y0),
                 (int)std::lround(s.x1), (int)std::lround(s.y1), r, g, b);
    }

    const std::string ext = path.size() >= 4 ? path.substr(path.size() - 4) : "";
    if (ext == ".bmp" || ext == ".BMP")
        return stbi_write_bmp(path.c_str(), w, h, 3, rgb.data()) != 0;
    if (ext == ".jpg" || ext == ".JPG")
        return stbi_write_jpg(path.c_str(), w, h, 3, rgb.data(), 90) != 0;
    return stbi_write_png(path.c_str(), w, h, 3, rgb.data(), w * 3) != 0;
}

bool saveGrayPng(const std::string& path, const GrayImage& img) {
    if (img.width == 0 || img.height == 0) return false;
    return stbi_write_png(path.c_str(), img.width, img.height, 1, img.data.data(), img.width) != 0;
}

bool saveRgbPng(const std::string& path, int width, int height,
                const std::vector<unsigned char>& rgb) {
    if (width == 0 || height == 0) return false;
    return stbi_write_png(path.c_str(), width, height, 3, rgb.data(), width * 3) != 0;
}

}  // namespace sweeplsd
