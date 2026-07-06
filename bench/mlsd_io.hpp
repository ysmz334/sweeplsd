// Tiny reader for M-LSD line files produced by ref/20260612/mlsd/mlsd_runner.py.
// Format: one segment per line, "x1 y1 x2 y2" (floats), in source-image pixels.
// Returns false if the file is absent/unreadable (so callers can skip M-LSD).
#pragma once
#include <fstream>
#include <string>
#include <vector>
#include "sweeplsd/sweeplsd.hpp"

inline bool readMlsdFile(const std::string& path, std::vector<sweeplsd::LineSegment>& out) {
    std::ifstream in(path);
    if (!in) return false;
    out.clear();
    float x0, y0, x1, y1;
    while (in >> x0 >> y0 >> x1 >> y1) out.push_back({x0, y0, x1, y1});
    return true;
}
