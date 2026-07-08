#pragma once

#include <cstdint>

// 128-bit integer support for the judgment products (T^2 up to ~2^118 at the
// documented widths): ap_(u)int<128> under Vitis HLS, the compiler's __int128
// on the tool-free host build (GCC/Clang — MinGW here; MSVC would need a
// small software wide-mul, TODO if ever needed). Included at file scope —
// ap_int.h drags in <cmath> and must never be included inside a namespace.
#if defined(__SYNTHESIS__) || __has_include(<ap_int.h>)
#include <ap_int.h>
#define SWEEPLSD_HLS_HAS_AP_INT 1
#elif !defined(__SIZEOF_INT128__)
#error "128-bit integer support required (GCC/Clang host or Vitis HLS ap_int)"
#endif

// Shared types and constants for the SweepLSD HLS core (see ../DESIGN.md).
//
// This directory is self-contained on purpose — it must synthesise without
// ../../src — so the few per-pixel definitions it shares with the software
// kernels are duplicated here and kept in lockstep by the parity testbench
// (hls/tb), which fails on the first mismatching pixel.

namespace sweeplsd_hls {

// Compile-time maximum image width: sizes every line buffer (BRAM depth).
// Runtime width may be anything <= this. 2048 covers the FullHD target.
constexpr int kMaxWidth = 2048;

// Gaussian kernel {16, 64, 96, 64, 16} (sum 256), single >>10 rescale in the
// horizontal pass — the thesis §3.2.1.1 fixed-point formulation.
constexpr int kGaussK0 = 16, kGaussK1 = 64, kGaussK2 = 96;
constexpr int kGaussShift = 10;

// Gradient power + H/V direction for one pixel (stage 1b -> 1c stream).
struct PowerDir {
    std::uint16_t power;  // (|dx|+|dy|+1)/2, <= 32640
    std::uint8_t dir;     // 1 = vertical edge (|dx| dominant), 0 = horizontal
};

// Sparse event record (front-end -> labelling back-end). One record per
// feature pixel plus one end-of-row marker per row; the y coordinate is
// implicit (the consumer counts end-of-row markers).
struct Event {
    std::uint8_t kind;    // see EventKind
    std::uint16_t x;      // column, < kMaxWidth (meaningful for feature kinds)
    std::uint8_t strong;  // (d) hysteresis: this pixel's power >= high threshold
                          //   (meaningful only for kEventInterior)
};

enum EventKind : std::uint8_t {
    kEventEndOfFrame = 0,
    kEventInterior = 1,  // == Feature::Interior
    kEventEndpoint = 2,  // == Feature::Endpoint
    kEventEndOfRow = 3,
};

// (d) streaming hysteresis configuration. `on` extracts edges at the LOW
// threshold (fixed `low`, or the adaptive percentile) instead of power_th,
// and rejects any segment with fewer than `strong_min` pixels reaching
// power_th (the HIGH threshold). Off = baseline (power_th everywhere).
struct HystCfg {
    bool on;
    bool adaptive;
    int low;
    int strong_min;
};

// Line-judgment aspect bound as an exact rational: accept a segment iff
// lambda_min/lambda_max <= kAspectNum/kAspectDen. The baseline parameter is
// 0.05 = 1/20 exactly, which turns the software's sqrt-based eigenvalue test
// into the pure-integer comparison
//   (kAspectDen - kAspectNum)^2 * T^2  <=  (kAspectDen + kAspectNum)^2 * R^2
// (T = trace, R^2 = (ma-mc)^2 + 4 mb^2; both sides fit in 128 bits because
// R <= T always holds for a PSD scatter matrix). See DESIGN.md.
constexpr int kAspectNum = 1, kAspectDen = 20;

// 128-bit judgment integers (see the file-scope include block above).
#if defined(SWEEPLSD_HLS_HAS_AP_INT)
using wide_t = ap_int<128>;
using uwide_t = ap_uint<128>;
#else
using wide_t = __int128;
using uwide_t = unsigned __int128;
#endif

}  // namespace sweeplsd_hls
