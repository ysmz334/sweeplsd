#pragma once

// SweepLSD — a one-pass line segment detector. One sweep, all segments.
//
// The image is processed as a single top-to-bottom raster sweep: 5x5 gaussian
// -> 2x2 gradient -> threshold + NMS edge -> endpoint candidates -> streaming
// connected-component labelling with per-label scatter moments -> PCA line
// judgment. The per-pixel core is integer-only and needs O(width) memory
// (a few buffered rows), which is what makes the design FPGA-friendly.
//
// The algorithm was proposed as "OPLSD" in a 2014 master's thesis (Yoshiyasu
// Shimizu, Waseda University); this library is its from-scratch reimplementation
// plus measured improvements. `Params{}` reproduces the thesis behaviour;
// `Params::improved()` enables the improvements (see each flag below).
//
// Thesis section references (§...) throughout the sources point at that thesis.

#include <string>
#include <vector>

#include "sweeplsd/image.hpp"

namespace sweeplsd {

struct LineSegment {
    float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
};

// Extended result: the segment plus its scatter statistics (centroid, principal
// direction, eigenvalues of the pixel-scatter covariance).
struct LineSegmentEx {
    LineSegment seg;
    int pix_num = 0;
    float cx = 0, cy = 0;
    float dir_x = 1, dir_y = 0;
    float ev_max = 0, ev_min = 0;
};

struct StageTiming {
    std::string name;
    double ms = 0;
};

struct Params {
    // ---- original SweepLSD parameters (thesis §3.2) --------------------------
    int gradient_power_th = 256;   // edge threshold on the gradient power
    // Judgment criterion 1: minimum pixels per segment (thesis default 15).
    // This is the coverage knob: lowering it admits shorter structure lines,
    // which measurably helps downstream vanishing-point estimation in
    // cluttered scenes (NYU indoor median error 12.7deg -> 11.6 at 12 ->
    // 11.0 at 10; York Urban 1.04deg -> 1.01) at the cost of ~30% more
    // (short, real) segments on rich outdoor photos. The default stays at the
    // thesis value; VP-oriented applications should consider 10-12.
    int pixel_num_th = 15;
    double aspect_th = 0.05;       // judgment criterion 4: ev_min/ev_max bound
    int max_segments = 100000;

    // ---- previously added improvements ------------------------------------
    bool weight_by_gradient = false;  // improvement 3: gradient-weighted moments
    bool use_nfa = false;             // improvement 4: streaming a-contrario gate
    double nfa_epsilon = 0.0;
    bool link_collinear = false;      // improvement 5: collinear linking
    double link_max_angle_deg = 4.0;
    double link_max_gap = 4.0;

    // ---- new accuracy improvements (this work) -----------------------------
    // Two ideas were evaluated and DROPPED after testing (see the accuracy/speed
    // report's "improvement validity" section): a 4-direction gradient
    // quantisation (former (b)) and the direction-coherence gate that depended
    // on it (former (e)). On the thesis dataset and the synthetic-GT benchmark
    // the 4-direction NMS *fragmented* diagonal and repetitive-texture edges
    // (F-max fell ~0.4 vs ~0.9 at sigma=20; segment counts inflated up to 5x)
    // while giving no gain on downstream vanishing-point accuracy, so the
    // gradient direction stays quantised to H/V only. The remaining letters keep
    // their original identifiers.
    // (a) strict tie-break in NMS: gradient plateaus thin to 1px instead of 2.
    bool nms_strict_tiebreak = false;
    // (c) sub-pixel NMS: parabolic interpolation of the power across the
    //     gradient gives a per-edge-pixel offset accumulated into the moments.
    bool subpixel_nms = false;
    // (d) streaming hysteresis: extract edges at a LOW threshold, but require
    //     each label to contain >= hysteresis_strong_min pixels whose power
    //     reaches the main (high) threshold. 1-pass equivalent of Canny's
    //     2-threshold hysteresis, folded into the labeling stage.
    bool use_hysteresis = false;
    int hysteresis_low_th = 120;       // low-threshold floor (high = gradient_power_th)
    int hysteresis_strong_min = 3;
    // Adapt the low threshold to the image's noise floor (a decayed power
    // histogram, O(1) state, identical row order in both drivers): keeps the
    // hysteresis benefit on clean images without flooding noisy ones.
    bool hysteresis_adaptive = true;
    // (f) endpoints from projection extremes: instead of the first/last
    //     endpoint-candidate contacts, report the extreme points of the label's
    //     bounding box projected on the fitted axis (robust to >2 contacts).
    bool endpoint_from_bbox = false;
    // (g) local NFA: exponential forgetting of the edge-density estimate
    //     (window in rows; 0 = global running density as before).
    int nfa_window_rows = 0;
    // (h) curve rejection: reject a segment whose absolute perpendicular RMS
    //     spread sqrt(ev_min) exceeds this (px). A straight thin edge has a tiny
    //     constant perpendicular variance; a curved arc bows away from its chord
    //     and is rejected. Complements the *relative* aspect_th with an
    //     *absolute* bound (a short low-curvature arc passes the ratio test but
    //     not this). 0 = off. Once-per-segment; integer form is ev_min <= th^2.
    double max_perp_spread = 0.0;
    // (i) border guard: ignore edge pixels within this many pixels of the image
    //     frame during labelling, so the boundary artifact (the gaussian/gradient
    //     step at the very edge gets detected as a rectangle tracing the frame)
    //     is not emitted. 0 = off.
    int border_margin = 0;
    // (j) gradient-lattice half-pixel correction. The 2x2 gradient operator
    //     samples the gradient at the CORNER between four pixels, i.e. at
    //     (x+0.5, y+0.5) in pixel-centre coordinates, but the moments accumulate
    //     the integer indices (x, y) — so every emitted segment sits exactly
    //     half a pixel up-left of the true edge (0.71 px laterally on a 45°
    //     edge). Shifting the finished segment by (+0.5, +0.5) removes the bias;
    //     canonical LSD applies the identical correction to its output for the
    //     same reason ("points with an offset of (0.5,0.5), that should be
    //     added to output", lsd.c).
    bool lattice_half_shift = false;

    // ---- speed switches (exact: do not change the output) ------------------
    bool sparse_feature_scan = true;  // 8px zero-word skip in the endpoint stage
    bool sparse_label_scan = true;    // 8px zero-word skip in the labeling stage

    // All accuracy improvements on, with tuned defaults.
    static Params improved() {
        Params p;
        p.nms_strict_tiebreak = true;
        p.subpixel_nms = true;
        p.use_hysteresis = true;
        p.hysteresis_low_th = 120;
        p.hysteresis_strong_min = 3;
        p.endpoint_from_bbox = true;
        p.max_perp_spread = 1.0;  // (h) reject curved arcs (keeps straight lines)
        p.border_margin = 3;      // (i) drop the image-frame boundary artifact
                                  //     (the 2x2 gradient biases it ~2px in on
                                  //     the bottom/right edge, so 3 clears it)
        p.lattice_half_shift = true;  // (j) centre segments on the true edge
        return p;
    }
};

// Multi-pass reference pipeline (one full-image pass per stage).
std::vector<LineSegment> detect(const GrayImage& src, const Params& params = Params{});
std::vector<LineSegmentEx> detectEx(const GrayImage& src, const Params& params = Params{});

// Streaming one-pass pipeline (line buffers only, O(width) memory).
std::vector<LineSegment> detectOnePass(const GrayImage& src, const Params& params = Params{});

std::vector<StageTiming> profileStages(const GrayImage& src, const Params& params, int runs);

}  // namespace sweeplsd
