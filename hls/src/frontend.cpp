#include "frontend.hpp"

// Streaming implementations of the four pixel-rate stages. Structure shared
// by all of them:
//
//   for y in [0, height + rowLag]:      // rowLag flush rows sweep the lag out
//     for x in [0, width + colLag]:
//       read input pixel        (only while y < height && x < width)
//       line-buffer shift       (only while x < width)
//       window shift
//       emit output (y-rowLag, x-colLag)  (once y >= rowLag && x >= colLag)
//
// Out-of-image taps are zero by construction (line buffers zero-initialised,
// reads gated to x < width, inputs gated to the image extent), which
// reproduces the software's zero-row stand-ins and bounds-checked reads
// exactly. Each line buffer sees one read and one write per cycle (dual-port
// BRAM), so II=1 holds without partitioning.

namespace sweeplsd_hls {

namespace {

// Verbatim port of sweeplsd::kernels::endpointCore (see ../../src/kernels.hpp
// for the derivation; thesis §3.2.2 fig. 3.11): thin the 5x5 outer ring,
// count survivors, and report "endpoint candidate" unless the centre is a
// straight line passing through. Branch-free boolean algebra over the 24
// neighbour bits — synthesises to a few LUT levels. Parity-tested against the
// software original by hls/tb. Takes the window array directly (win[2+dy][2+dx]
// = the software's e(dx, dy)) — a capturing-lambda accessor lowers to
// pointer-to-pointer, which Vitis HLS rejects [HLS 214-134].
inline bool endpointCore(const std::uint8_t win[5][5]) {
    using U = std::uint8_t;
    U a = win[2][1], b = win[1][1], c = win[1][2], d = win[1][3];
    U eR = win[2][3], fR = win[3][3], gR = win[3][2], hR = win[3][1];

    U i = win[2][0], j = win[1][0], k = win[0][0], l = win[0][1];
    U m = win[0][2],  n = win[0][3],  o = win[0][4],  p = win[1][4];
    U q = win[2][4],   r = win[3][4],   s = win[4][4],   t = win[4][3];
    U u = win[4][2],   v = win[4][1],  w = win[4][0],  xx = win[3][0];

    U ab = a | b, bc = b | c, cd = c | d, de = d | eR;
    U ef = eR | fR, fg = fR | gR, gh = gR | hR, ha = hR | a;
    U hab = ha | b, bcd = bc | d, def = de | fR, fgh = fg | hR;

    i &= hab; m &= bcd; q &= def; u &= fgh;

    U hai = ha | i, abi = ab | i, bcm = bc | m, cdm = cd | m;
    U deq = de | q, efq = ef | q, fgu = fg | u, ghu = gh | u;

    j &= abi; l &= bcm; n &= cdm; p &= deq;
    r &= efq; t &= fgu; v &= ghu; xx &= hai;

    U xj = xx | j, ln = l | n, pr = p | r, tv = t | v;
    i &= (xj ^ 1); m &= (ln ^ 1); q &= (pr ^ 1); u &= (tv ^ 1);

    U jl = j & l, np = n & p, rt = r & t, vx = v & xx;
    U bjl = b | jl, dnp = d | np, frt = fR | rt, hvx = hR | vx;

    k &= bjl; o &= dnp; s &= frt; w &= hvx;
    j &= (k ^ 1); l &= (k ^ 1); n &= (o ^ 1); p &= (o ^ 1);
    r &= (s ^ 1); t &= (s ^ 1); v &= (w ^ 1); xx &= (w ^ 1);

    U count = U(i + j + k + l + m + n + o + p + q + r + s + t + u + v + w + xx);

    U xm = U((U(0) - j) & 0x01) ^ U((U(0) - k) & 0x03) ^ U((U(0) - l) & 0x07) ^
           U((U(0) - m) & 0x0f) ^ U((U(0) - n) & 0x1f) ^ U((U(0) - o) & 0x3f) ^
           U((U(0) - p) & 0x7f) ^ U((U(0) - q) & 0xff) ^ U((U(0) - r) & 0xfe) ^
           U((U(0) - s) & 0xfc) ^ U((U(0) - t) & 0xf8) ^ U((U(0) - u) & 0xf0) ^
           U((U(0) - v) & 0xe0) ^ U((U(0) - w) & 0xc0) ^ U((U(0) - xx) & 0x80);
    U pc = xm - ((xm >> 1) & 0x55);
    pc = (pc & 0x33) + ((pc >> 2) & 0x33);
    pc = U(pc + (pc >> 4)) & 0x0f;

    U straight = U(count == 2) & U(pc > 6);
    return straight == 0;
}

// (d) hysteresis adaptive low threshold — fixed-point twin of
// sweeplsd::kernels::AdaptiveLowTh (../../src/kernels.hpp), kept in lockstep
// by hls/tb. Decayed 64-bin power histogram; the low threshold is ~2x the
// 80th-percentile power. Integer only: counts x256, decay v-=v>>8, percentile
// via cum*5 >= total*4. A row's threshold is derived from the histogram of the
// rows TWO back (lowTh(H_{r-2})): the previous row's lowTh is staged into
// low_pending and consumed here. The two-row lag gives the FPGA a full row to
// run the 64-bin percentile scan instead of the ~10-cycle inter-row gap.
struct AdaptiveLowTh {
    static const int kBins = 64, kBinW = 64;
    static const std::uint32_t kUnit = 256;
    std::uint32_t bins[kBins];
    std::uint32_t total;
    void reset() {
        for (int b = 0; b < kBins; ++b) bins[b] = 0;
        total = 0;
    }
    void update(const std::uint16_t* power, int w) {
        for (int b = 0; b < kBins; ++b) bins[b] -= bins[b] >> 8;
        total -= total >> 8;
        for (int x = 0; x < w; x += 4) {
            int b = power[x] >> 6;
            bins[b < kBins ? b : kBins - 1] += kUnit;
            total += kUnit;
        }
    }
    int lowTh(int user_low, int high) const {
        if (total == 0) return user_low;
        const std::uint64_t want4 = std::uint64_t(total) * 4;
        std::uint64_t cum = 0;
        int b = 0;
        for (; b < kBins; ++b) {
            cum += bins[b];
            if (cum * 5 >= want4) break;
        }
        if (b >= kBins) b = kBins - 1;
        int th = 2 * (b * kBinW + kBinW / 2);
        if (th < user_low) th = user_low;
        if (th > high) th = high;
        return th;
    }
};

}  // namespace

// ---- Stage 1a: gaussian ---------------------------------------------------

void hlsGaussian(hls::stream<std::uint8_t>& in, hls::stream<std::uint16_t>& out,
                 int width, int height) {
    static std::uint8_t lb0[kMaxWidth], lb1[kMaxWidth], lb2[kMaxWidth], lb3[kMaxWidth];

init:
    for (int x = 0; x < width; ++x) {
#pragma HLS PIPELINE II = 1
        lb0[x] = lb1[x] = lb2[x] = lb3[x] = 0;
    }

rows:
    for (int y = 0; y <= height + 1; ++y) {
        // Horizontal window of vertical sums: after column x it holds the
        // sums for columns x-4 .. x.
        std::uint16_t vwin[5] = {0, 0, 0, 0, 0};
#pragma HLS ARRAY_PARTITION variable = vwin complete
    cols:
        for (int x = 0; x <= width + 1; ++x) {
#pragma HLS PIPELINE II = 1
            std::uint8_t cur = 0;
            if (y < height && x < width) cur = in.read();

            // Vertical 5-tap at row y-2 (lb3..lb0 hold source rows y-1..y-4).
            std::uint16_t vs = 0;
            if (x < width) {
                std::uint8_t r0 = lb0[x], r1 = lb1[x], r2 = lb2[x], r3 = lb3[x];
                vs = std::uint16_t(kGaussK0 * (r0 + cur) + kGaussK1 * (r1 + r3) +
                                   kGaussK2 * r2);  // <= 65280
                lb0[x] = r1;
                lb1[x] = r2;
                lb2[x] = r3;
                lb3[x] = cur;
            }
            vwin[0] = vwin[1];
            vwin[1] = vwin[2];
            vwin[2] = vwin[3];
            vwin[3] = vwin[4];
            vwin[4] = vs;

            if (y >= 2 && x >= 2) {
                const int r = y - 2, xc = x - 2;
                std::uint16_t g = 0;
                const bool border =
                    r < 2 || r >= height - 2 || xc < 2 || xc >= width - 2;
                if (!border) {
                    std::uint32_t s = std::uint32_t(kGaussK0) * (vwin[0] + vwin[4]) +
                                      std::uint32_t(kGaussK1) * (vwin[1] + vwin[3]) +
                                      std::uint32_t(kGaussK2) * vwin[2];
                    g = std::uint16_t(s >> kGaussShift);  // <= 16320
                }
                out.write(g);
            }
        }
    }
}

// ---- Stage 1b: gradient ---------------------------------------------------

void hlsGradient(hls::stream<std::uint16_t>& in, hls::stream<PowerDir>& out,
                 int width, int height) {
    static std::uint16_t lbg[kMaxWidth];

init:
    for (int x = 0; x < width; ++x) {
#pragma HLS PIPELINE II = 1
        lbg[x] = 0;
    }

rows:
    for (int y = 0; y <= height; ++y) {
        std::uint16_t g0_left = 0;  // gauss(y-1, x-1)
        std::uint16_t g1_left = 0;  // gauss(y,   x-1)
    cols:
        for (int x = 0; x <= width; ++x) {
#pragma HLS PIPELINE II = 1
            std::uint16_t g = 0;  // gauss(y, x); 0 past the bottom/right edge
            if (y < height && x < width) g = in.read();
            std::uint16_t g0_right = 0;  // gauss(y-1, x); 0 past the right edge
            if (x < width) {
                g0_right = lbg[x];
                lbg[x] = g;
            }
            if (y >= 1 && x >= 1) {
                // 2x2 window at (r = y-1, xc = x-1): g00 g10 / g01 g11.
                const int g00 = g0_left, g10 = g0_right;
                const int g01 = g1_left, g11 = g;
                int dx = (g10 + g11) - (g00 + g01);
                int dy = (g01 + g11) - (g00 + g10);
                if (dx < 0) dx = -dx;
                if (dy < 0) dy = -dy;
                PowerDir pd;
                pd.power = std::uint16_t((dx + dy + 1) >> 1);  // <= 32640
                pd.dir = std::uint8_t(dx > dy);
                out.write(pd);
            }
            g0_left = g0_right;
            g1_left = g;
        }
    }
}

// ---- Stage 1c: edge = threshold + NMS -------------------------------------

// Output packing: bit0 = edge (survives threshold + NMS), bit1 = strong
// ((d) hysteresis: centre power >= the HIGH threshold power_th). With
// hyst_on the NMS uses the LOW threshold (fixed hyst_low, or the adaptive
// percentile) while the strong bit still tests power_th; with hyst off the
// low threshold IS power_th and the strong bit is unused.
void hlsEdge(hls::stream<PowerDir>& in, hls::stream<std::uint8_t>& out,
             int width, int height, int power_th, bool strict,
             bool hyst_on, bool hyst_adaptive, int hyst_low) {
    static std::uint16_t lpa[kMaxWidth];  // power row y-2 (= row r-1)
    static std::uint16_t lpc[kMaxWidth];  // power row y-1 (= row r)
    static std::uint8_t ldir[kMaxWidth];  // dir   row y-1 (= row r)
    static AdaptiveLowTh adapt;
    static int low_pending;               // (d) staged lowTh(H_{r-1}); two-row lag

init:
    for (int x = 0; x < width; ++x) {
#pragma HLS PIPELINE II = 1
        lpa[x] = lpc[x] = 0;
        ldir[x] = 0;
    }
    adapt.reset();
    low_pending = hyst_low;

    const int s = strict ? 1 : 0;
rows:
    for (int y = 0; y <= height; ++y) {
        // Threshold for NMS of row r = y-1 = lowTh(H_{r-2}) (two-row lag): use
        // the value staged last row, then stage lowTh of the current histogram
        // (= H_{r-1}) for next row. Baseline / fixed-low need no state.
        int low_th = power_th;
        if (hyst_on) {
            if (hyst_adaptive) {
                low_th = low_pending;
                low_pending = adapt.lowTh(hyst_low, power_th);  // adapt == H_{r-1} here
            } else {
                low_th = hyst_low;
            }
        }
        if (y >= 1 && hyst_on && hyst_adaptive) {
            for (int b = 0; b < AdaptiveLowTh::kBins; ++b)
                adapt.bins[b] -= adapt.bins[b] >> 8;   // decay, then fold row r below
            adapt.total -= adapt.total >> 8;
        }
        std::uint16_t pc_m = 0;   // pc[xc-1]
        std::uint16_t pc_c = 0;   // pc[xc]
        std::uint16_t pa_d = 0;   // pa[xc]
        std::uint16_t pb_d = 0;   // pb[xc] (incoming row y, one column behind)
        std::uint8_t dir_d = 0;   // dir[xc]
    cols:
        for (int x = 0; x <= width; ++x) {
#pragma HLS PIPELINE II = 1
            std::uint16_t pw = 0;
            std::uint8_t dr = 0;
            if (y < height && x < width) {
                PowerDir pd = in.read();
                pw = pd.power;
                dr = pd.dir;
            }
            std::uint16_t pa_x = 0, pc_right = 0;
            std::uint8_t dir_x = 0;
            if (x < width) {
                pa_x = lpa[x];
                pc_right = lpc[x];     // power of row r = y-1 at column x
                dir_x = ldir[x];
                lpa[x] = pc_right;
                lpc[x] = pw;
                ldir[x] = dr;
                // (d) fold row r's power into the histogram (every 4th column,
                // same subsample as the software) — after this row's threshold
                // was already taken above.
                if (y >= 1 && hyst_on && hyst_adaptive && (x & 3) == 0) {
                    int b = pc_right >> 6;
                    adapt.bins[b < AdaptiveLowTh::kBins ? b : AdaptiveLowTh::kBins - 1] +=
                        AdaptiveLowTh::kUnit;
                    adapt.total += AdaptiveLowTh::kUnit;
                }
            }
            if (y >= 1 && x >= 1) {
                // NMS at (r = y-1, xc = x-1). Competitors: vertical edges
                // compete left/right on pc, horizontal edges up/down (pa, pb).
                const int c = pc_c;
                const bool isV = dir_d == 1;
                const int Pm = isV ? int(pc_m) : int(pa_d);
                const int Pp = isV ? int(pc_right) : int(pb_d);
                const std::uint8_t edge =
                    std::uint8_t((c >= low_th) & (c >= Pm + s) & (c >= Pp));
                const std::uint8_t strong = std::uint8_t(c >= power_th);
                out.write(std::uint8_t(edge | (strong << 1)));
            }
            pc_m = pc_c;
            pc_c = pc_right;
            pa_d = pa_x;
            dir_d = dir_x;
            pb_d = pw;
        }
    }
}

// ---- Stage 2: endpoint candidates -----------------------------------------

void hlsFeature(hls::stream<std::uint8_t>& in, hls::stream<std::uint8_t>& out,
                int width, int height) {
    static std::uint8_t lb0[kMaxWidth], lb1[kMaxWidth], lb2[kMaxWidth], lb3[kMaxWidth];

init:
    for (int x = 0; x < width; ++x) {
#pragma HLS PIPELINE II = 1
        lb0[x] = lb1[x] = lb2[x] = lb3[x] = 0;
    }

rows:
    for (int y = 0; y <= height + 1; ++y) {
        // 5x5 window: win[wy][wx] = {strong<<1 | edge} of (y-2 + (wy-2),
        // x-2 + (wx-2)) once the output position (y-2, x-2) is in frame; zero
        // outside the image. The strong bit ((d) hysteresis) rides through the
        // same buffers so it emerges aligned with the centre's classification.
        std::uint8_t win[5][5] = {};
#pragma HLS ARRAY_PARTITION variable = win complete dim = 0
    cols:
        for (int x = 0; x <= width + 1; ++x) {
#pragma HLS PIPELINE II = 1
            std::uint8_t e = 0;
            if (y < height && x < width) e = in.read();

            std::uint8_t t0 = 0, t1 = 0, t2 = 0, t3 = 0, t4 = 0;
            if (x < width) {
                t0 = lb0[x];  // row y-4
                t1 = lb1[x];  // row y-3
                t2 = lb2[x];  // row y-2
                t3 = lb3[x];  // row y-1
                t4 = e;       // row y
                lb0[x] = t1;
                lb1[x] = t2;
                lb2[x] = t3;
                lb3[x] = t4;
            }
            for (int wy = 0; wy < 5; ++wy) {
#pragma HLS UNROLL
                win[wy][0] = win[wy][1];
                win[wy][1] = win[wy][2];
                win[wy][2] = win[wy][3];
                win[wy][3] = win[wy][4];
            }
            win[0][4] = t0;
            win[1][4] = t1;
            win[2][4] = t2;
            win[3][4] = t3;
            win[4][4] = t4;

            if (y >= 2 && x >= 2) {
                std::uint8_t ewin[5][5];  // edge-bit-only view for endpointCore
                for (int wy = 0; wy < 5; ++wy)
                    for (int wx = 0; wx < 5; ++wx) ewin[wy][wx] = win[wy][wx] & 1;
                const std::uint8_t centre = ewin[2][2];
                const std::uint8_t strong = std::uint8_t((win[2][2] >> 1) & 1);
                const int isEnd = endpointCore(ewin) ? 1 : 0;
                // bits[1:0] = feature code 0/1/2, bit2 = strong
                out.write(std::uint8_t(int(centre) * (isEnd + 1) | (strong << 2)));
            }
        }
    }
}

// ---- Feature map -> sparse events ------------------------------------------

void hlsEvents(hls::stream<std::uint8_t>& in, hls::stream<Event>& out,
               int width, int height) {
rows:
    for (int y = 0; y < height; ++y) {
    cols:
        for (int x = 0; x < width; ++x) {
#pragma HLS PIPELINE II = 1
            std::uint8_t f = in.read();
            const std::uint8_t code = f & 3;      // 1 = Interior, 2 = Endpoint
            if (code != 0) {
                Event ev;
                ev.kind = code;
                ev.x = std::uint16_t(x);
                ev.strong = std::uint8_t((f >> 2) & 1);  // (d) hysteresis
                out.write(ev);
            }
        }
        out.write(Event{kEventEndOfRow, 0, 0});
    }
    out.write(Event{kEventEndOfFrame, 0, 0});
}

// ---- Top-level dataflow ------------------------------------------------------

void sweeplsdFrontend(hls::stream<std::uint8_t>& src, hls::stream<Event>& events,
                      int width, int height, int power_th, bool strict,
                      const HystCfg& hyst) {
#pragma HLS DATAFLOW
    static hls::stream<std::uint16_t> gauss_s;
    static hls::stream<PowerDir> grad_s;
    static hls::stream<std::uint8_t> edge_s;
    static hls::stream<std::uint8_t> feat_s;
#pragma HLS STREAM variable = gauss_s depth = 16
#pragma HLS STREAM variable = grad_s depth = 16
#pragma HLS STREAM variable = edge_s depth = 16
#pragma HLS STREAM variable = feat_s depth = 16

    hlsGaussian(src, gauss_s, width, height);
    hlsGradient(gauss_s, grad_s, width, height);
    hlsEdge(grad_s, edge_s, width, height, power_th, strict, hyst.on, hyst.adaptive,
            hyst.low);
    hlsFeature(edge_s, feat_s, width, height);
    hlsEvents(feat_s, events, width, height);
}

}  // namespace sweeplsd_hls
