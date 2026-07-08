#pragma once

#include <cstdint>

#include "frontend.hpp"
#include "hls_stream_compat.hpp"
#include "hls_types.hpp"

// SweepLSD HLS back-end: event-driven connected-component labelling with
// per-label scatter moments, endpoint-delimited segment closure, and the
// pure-integer PCA line judgment (thesis §3.2.3-3.2.4; see ../DESIGN.md).
//
// Consumes the sparse event stream produced by the front-end (interior /
// endpoint pixels + row markers) and emits one SegmentRecord per accepted
// segment, terminated by one record with n == 0. The only floating-point
// step of the algorithm — projecting the endpoint contacts onto the fitted
// axis — is NOT done here; the host applies it to the record (see
// ../host/finalize.hpp), which reproduces the software output bit-exactly.
//
// Semantics match sweeplsd::Labeler (the golden model) exactly, including the
// merge-survivor rule, the segment emission order, and the judgment — with
// one deliberate hardware addition: labels are recycled through a free-list
// ring (the original ref implementation's scheme, made one row more
// conservative to be safe under full union-find path compression; see the
// scavenger comment in backend.cpp).

namespace sweeplsd_hls {

// Concurrent-label capacity (thesis table 3.4 budgets 960; the original
// software model ran the same image class with 512). Id 0 is the "no label"
// sentinel, so kMaxLabels-1 ids are usable.
constexpr int kMaxLabels = 1024;

// One accepted segment: the two raw endpoint contacts plus the exact integer
// scatter moments the host needs for the sub-pixel endpoint projection.
// Hardware widths (documented; the C model uses container types):
//   sx/ex: u11  sy/ey: u11  n: u18  x_sum/y_sum: u30  *_sq/xy: u41
//   bbox fields: u11 each
struct SegmentRecord {
    std::uint16_t sx, sy, ex, ey;
    std::uint32_t n;  // pixel count; n == 0 terminates the stream
    std::uint64_t x_sum, y_sum;
    std::uint64_t x_sq_sum, y_sq_sum, xy_sum;
    // (f) bbox extreme points: the PIXEL that attained each extreme of the
    // label's bounding box (companion coordinate included), so the host can
    // choose the endpoint pair as the projection extremes among these four
    // points and the two contacts. Always tracked (cheap); only the host
    // finalisation decides whether to use them (endpoint_from_bbox).
    std::uint16_t min_x, min_x_y, max_x, max_x_y;
    std::uint16_t min_y, min_y_x, max_y, max_y_x;
};

// Event stream in, accepted-segment records out. `pixel_num_th` is judgment
// criterion 1 (thesis default 15); the eigenvalue-ratio bound is the exact
// rational kAspectNum/kAspectDen in hls_types.hpp (= the baseline 0.05).
// `hyst.strong_min` gates (d) hysteresis (0 / off = no gate).
void sweeplsdBackend(hls::stream<Event>& events, hls::stream<SegmentRecord>& out,
                     int width, int height, int pixel_num_th, const HystCfg& hyst);

// Full core: front-end + back-end as one dataflow region.
void sweeplsdCore(hls::stream<std::uint8_t>& src, hls::stream<SegmentRecord>& out,
                  int width, int height, int power_th, bool strict, int pixel_num_th,
                  const HystCfg& hyst);

}  // namespace sweeplsd_hls

// Synthesis top at global scope — Vitis HLS set_top cannot name functions
// inside a namespace. Thin forwarding wrapper around sweeplsd_hls::sweeplsdCore.
void sweeplsd_core_top(hls::stream<std::uint8_t>& src,
                       hls::stream<sweeplsd_hls::SegmentRecord>& out, int width,
                       int height, int power_th, bool strict, int pixel_num_th,
                       bool hyst_on, bool hyst_adaptive, int hyst_low,
                       int hyst_strong_min);

namespace sweeplsd_hls {

#ifndef __SYNTHESIS__
// C-model diagnostics (not synthesized): high-water marks of the label table
// and the per-row event list, and the free-list-underflow count (must stay 0).
struct BackendStats {
    int max_live_labels = 0;
    int max_row_events = 0;
    int max_find_chain = 0;
    int freelist_underflows = 0;
};
BackendStats backendStats();
#endif

}  // namespace sweeplsd_hls
