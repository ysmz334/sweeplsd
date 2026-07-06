// Reader for genuine-EDLines result files written by tools/edlines_real.exe
// (the authors' ED_Lib, MIT, built separately with MSVC + OpenCV — see
// tools/build_edlines_real.bat). Format: a header line "<count> <median_ms>",
// then one segment per line, "x0 y0 x1 y1" (sub-pixel floats, source-image
// pixel coordinates). Returns false if the file is absent/unreadable so
// callers can fall back to the bundled EDLines-style detector.
#pragma once
#include <fstream>
#include <string>
#include <vector>
#include "sweeplsd/sweeplsd.hpp"

inline bool readEdRealFile(const std::string& path, std::vector<sweeplsd::LineSegment>& out,
                           double* median_ms = nullptr) {
    std::ifstream in(path);
    if (!in) return false;
    long n = 0;
    double ms = 0;
    if (!(in >> n >> ms)) return false;
    if (median_ms) *median_ms = ms;
    out.clear();
    float x0, y0, x1, y1;
    while (in >> x0 >> y0 >> x1 >> y1) out.push_back({x0, y0, x1, y1});
    return true;
}
