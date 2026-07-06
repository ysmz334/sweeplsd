#pragma once

#include <vector>

#include "sweeplsd/image.hpp"
#include "sweeplsd/sweeplsd.hpp"

// EDLines-style line segment detector, used as a comparison baseline.
//
// This is a compact, OpenCV-free reimplementation in the spirit of EDLines
// (Akinlar & Topal, 2011): Edge-Drawing-style clean edge chains followed by
// least-squares line fitting with split-on-deviation. It is NOT the authors'
// library (which depends on OpenCV) and omits their NFA/Helmholtz validation,
// so treat it as "EDLines-style", not a bit-exact EDLines.

namespace edlines {

struct Params {
    int gradient_th = 32;   // minimum |gx|+|gy| to be an edge pixel
    int min_length = 10;    // shortest accepted segment (pixels along the chain)
    double fit_tol = 1.5;   // max point-to-line distance before splitting (px)
};

std::vector<sweeplsd::LineSegment> detect(const sweeplsd::GrayImage& src, const Params& params = {});

}  // namespace edlines
