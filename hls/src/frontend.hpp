#pragma once

#include <cstdint>

#include "hls_stream_compat.hpp"
#include "hls_types.hpp"

// SweepLSD HLS front-end: the four pixel-rate pipeline stages (thesis §3.2.1
// gaussian / gradient / edge and §3.2.2 endpoint candidates), each written as
// a streaming process with line buffers, II=1 per pixel. The final process
// compresses the dense feature map into the sparse event stream consumed by
// the labelling back-end (see ../DESIGN.md).
//
// Every stage consumes and produces exactly width*height items in raster
// order; the internal row/column lags are absorbed by each stage's own
// line buffers, and out-of-image taps read as zero — matching the software
// zero-row / bounds-checked-read conventions bit-exactly.

namespace sweeplsd_hls {

// Stage 1a: 5x5 separable gaussian, {16,64,96,64,16}, vertical then
// horizontal, single >>10 in the horizontal pass. Border (2 rows/cols on each
// side) emits 0, exactly like the software gaussianHorizontalRow.
void hlsGaussian(hls::stream<std::uint8_t>& in, hls::stream<std::uint16_t>& out,
                 int width, int height);

// Stage 1b: 2x2 gradient on gaussian rows y, y+1 -> power + H/V direction.
void hlsGradient(hls::stream<std::uint16_t>& in, hls::stream<PowerDir>& out,
                 int width, int height);

// Stage 1c: threshold + NMS along the H/V direction. `strict` is the
// improved-mode tie-break (baseline: false). With hysteresis on, the NMS uses
// the low threshold (fixed hyst_low or the adaptive percentile) while the
// emitted strong bit tests power_th; output packs {strong<<1 | edge}.
void hlsEdge(hls::stream<PowerDir>& in, hls::stream<std::uint8_t>& out,
             int width, int height, int power_th, bool strict,
             bool hyst_on, bool hyst_adaptive, int hyst_low, int edge_border = 3);

// Stage 2: 5x5 endpoint-candidate classification -> bits[1:0] = 0 none /
// 1 interior / 2 endpoint, bit2 = strong (hysteresis).
void hlsFeature(hls::stream<std::uint8_t>& in, hls::stream<std::uint8_t>& out,
                int width, int height);

// Dense feature map -> sparse event stream (+ end-of-row / end-of-frame
// markers) for the labelling back-end.
void hlsEvents(hls::stream<std::uint8_t>& in, hls::stream<Event>& out,
               int width, int height);

// Top level: the four stages plus the event compressor as one dataflow region.
void sweeplsdFrontend(hls::stream<std::uint8_t>& src, hls::stream<Event>& events,
                      int width, int height, int power_th, bool strict,
                      const HystCfg& hyst, int edge_border = 3);

}  // namespace sweeplsd_hls
