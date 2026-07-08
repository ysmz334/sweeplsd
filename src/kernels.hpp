#pragma once

#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "sweeplsd/image.hpp"
#include "stages.hpp"

// Per-pixel kernels for the edge-extraction and endpoint stages, written once
// and shared by both the multi-pass detector (src/*.cpp loops over a Grid) and
// the streaming one-pass detector (src/sweeplsd_onepass.cpp loops over ring
// buffers).

// Compiler portability. SWEEPLSD_NOINLINE is performance-load-bearing, not
// cosmetic: featureRowInterior must stay out of large callers or GCC drops
// its auto-vectorization (see the comment at its definition).
#if defined(_MSC_VER)
#include <intrin.h>
#define SWEEPLSD_NOINLINE __declspec(noinline)
#else
#define SWEEPLSD_NOINLINE __attribute__((noinline))
#endif

namespace sweeplsd {
namespace kernels {

// Count trailing zeros of a non-zero 64-bit word (used to jump to the first
// non-zero byte of an 8-pixel group; assumes little-endian byte order, which
// holds on every supported target: x86-64 and AArch64).
inline int ctz64(std::uint64_t v) {
#if defined(_MSC_VER)
    unsigned long i;
    _BitScanForward64(&i, v);
    return int(i);
#else
    return __builtin_ctzll(v);
#endif
}

// Every stage is driven row-by-row over raw, contiguous row pointers (image
// borders handled outside the hot loop), so the inner loops stay branch-free
// and the compiler auto-vectorizes them to SSE/AVX2 — no explicit intrinsics.

// ---- Stage 1a: gaussian (thesis §3.2.1.1) --------------------------------
// Separable 5x5 with kernel {16,64,96,64,16} (sum 256). See the original
// comments: vertical pass keeps full precision in uint16 (16 AVX2 lanes), the
// horizontal pass weights in uint32 and applies the single >>10 rescale —
// bit-identical to a direct 5x5 convolution >>10.
inline const int* gaussWeights() {
    static const int k[3] = {16, 64, 96};
    return k;
}
constexpr int kGaussShift = 10;  // single rescale, applied in the horizontal pass

inline void gaussianVerticalRow(const std::uint8_t* r0, const std::uint8_t* r1,
                                const std::uint8_t* r2, const std::uint8_t* r3,
                                const std::uint8_t* r4, int w, std::uint16_t* __restrict out) {
    const int* k = gaussWeights();
    const int k0 = k[0], k1 = k[1], k2 = k[2];
    for (int x = 0; x < w; ++x) {
        std::uint16_t s = std::uint16_t(k0 * (r0[x] + r4[x]));
        s += std::uint16_t(k1 * (r1[x] + r3[x]));
        s += std::uint16_t(k2 * r2[x]);
        out[x] = s;  // <= 65280
    }
}

inline void gaussianHorizontalRow(const std::uint16_t* __restrict v, int y, int w, int h,
                                  std::uint16_t* __restrict out) {
    if (y < 2 || y >= h - 2 || w < 5) {
        for (int x = 0; x < w; ++x) out[x] = 0;
        return;
    }
    const int* k = gaussWeights();
    const std::uint32_t k0 = k[0], k1 = k[1], k2 = k[2];
    out[0] = out[1] = out[w - 2] = out[w - 1] = 0;
    for (int x = 2; x < w - 2; ++x) {
        std::uint32_t s = k0 * (v[x - 2] + v[x + 2]) + k1 * (v[x - 1] + v[x + 1]) + k2 * v[x];
        out[x] = std::uint16_t(s >> kGaussShift);  // <= 16320 (x64 fixed-point)
    }
}

// ---- Stage 1b: gradient (thesis §3.2.1.2-3.2.1.4) ------------------------
// 2x2 operator on the gaussian rows g0 (=row y) and g1 (=row y+1; pass a zero
// row at the bottom edge). Emits power = (|dx|+|dy|+1)/2 and the H/V direction
// (dir = 1 for a vertical edge, |dx| dominant; 0 for horizontal). The loop is
// branch-free => auto-vectorizes; only the last column (which would read x+1
// out of range) is handled separately.
inline void gradientRow(const std::uint16_t* g0, const std::uint16_t* g1, int w,
                        std::uint16_t* __restrict power, std::uint8_t* __restrict dir) {
    auto emit = [&](int x, int g00, int g10, int g01, int g11) {
        int dx = std::abs((g10 + g11) - (g00 + g01));
        int dy = std::abs((g01 + g11) - (g00 + g10));
        power[x] = std::uint16_t((dx + dy + 1) / 2);
        dir[x] = std::uint8_t(dx > dy);  // 1 = Vertical edge, 0 = Horizontal
    };
    for (int x = 0; x < w - 1; ++x) emit(x, g0[x], g0[x + 1], g1[x], g1[x + 1]);
    if (w > 0) emit(w - 1, g0[w - 1], 0, g1[w - 1], 0);
}

// Adaptive hysteresis low threshold (part of improvement d). A fixed low
// threshold below the image's noise floor floods the edge map with noise
// maxima (cost AND precision). This tracks an exponentially-decayed histogram
// of the gradient power (64 bins of width 64, subsampled every 4th pixel) and
// raises the low threshold to ~2x the 80th-percentile power — i.e. just above
// the bulk of the noise — clamped to [user low, high]. State is O(1) and the
// update order is the row order, identical in both drivers, so multi-pass and
// one-pass outputs stay bit-identical.
//
// FIXED-POINT (v2c step 3 / FPGA parity): counts are scaled x256 so the
// per-row decay `v -= v >> 8` (= v * 255/256 floored) keeps sub-unit
// precision without floats, and the 80th-percentile test avoids a divide by
// cross-multiplying `cum*5 >= total*4` (exactly 0.8, no rounding). This is
// what the HLS C model and the RTL histogram both compute bit-for-bit; the
// change vs the old float form is sub-LSB (an occasional one-bin threshold
// shift), see hls/DESIGN.md improvement (d).
struct AdaptiveLowTh {
    static constexpr int kBins = 64, kBinW = 64;  // covers power 0..4095
    static constexpr std::uint32_t kUnit = 256;   // fixed-point +1.0
    std::uint32_t bins[kBins] = {};
    std::uint32_t total = 0;

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
        // 80th percentile bin: first b with cumulative count >= 0.8*total,
        // written as cum*5 >= total*4 (integer, no divide).
        const std::uint64_t want4 = std::uint64_t(total) * 4;
        std::uint64_t cum = 0;
        int b = 0;
        for (; b < kBins; ++b) {
            cum += bins[b];
            if (cum * 5 >= want4) break;
        }
        if (b >= kBins) b = kBins - 1;
        int p80 = b * kBinW + kBinW / 2;
        int th = 2 * p80;
        if (th < user_low) th = user_low;
        if (th > high) th = high;
        return th;
    }
};

// ---- Stage 1c: edge = threshold + NMS + AND (thesis §3.2.1.5-3.2.1.7) ----
// Non-maximum suppression along the gradient. The competitor pair (Pm, Pp) per
// direction class:
//   Horizontal: (pa[x],   pb[x])     — up / down
//   Vertical:   (pc[x-1], pc[x+1])   — left / right
// The selection is branch-free (multiply-by-0/1 masks) so the interior loop
// auto-vectorizes.
//
// Tie handling (improvement a): with `strict`, the pixel must be STRICTLY
// greater than the minus-side neighbour (c >= Pm+1) and >= the plus side, so a
// 2px gradient plateau thins to a single pixel instead of surviving twice.
// With strict=false ties survive on both sides — bit-identical to the original.
inline void edgeRow(const std::uint16_t* pa, const std::uint16_t* pc, const std::uint16_t* pb,
                    const std::uint8_t* dir, int w, int power_th, bool strict,
                    std::uint8_t* __restrict out) {
    const int s = strict ? 1 : 0;
    auto at = [&](const std::uint16_t* row, int x) { return (x >= 0 && x < w) ? int(row[x]) : 0; };
    auto border = [&](int x) {  // bounds-checked, used for the 2 border columns only
        int c = at(pc, x);
        int isV = (dir[x] == 1), isH = (isV ^ 1);
        int Pm = isV * at(pc, x - 1) + isH * at(pa, x);
        int Pp = isV * at(pc, x + 1) + isH * at(pb, x);
        out[x] = std::uint8_t((c >= power_th) & (c >= Pm + s) & (c >= Pp));
    };
    if (w == 0) return;
    border(0);
    // Branch-free interior with plain array reads so it auto-vectorizes: the
    // competitor pair is selected by 0/1 masks per direction class.
    for (int x = 1; x < w - 1; ++x) {
        int c = pc[x];
        int isV = (dir[x] == 1), isH = (isV ^ 1);
        int Pm = isV * pc[x - 1] + isH * pa[x];
        int Pp = isV * pc[x + 1] + isH * pb[x];
        out[x] = std::uint8_t((c >= power_th) & (c >= Pm + s) & (c >= Pp));
    }
    if (w > 1) border(w - 1);
}

// Sub-pixel NMS offset (improvement c). For each SURVIVING edge pixel, fit a
// parabola through the three power samples along the NMS axis and store the
// vertex offset in 1/16 px units (int8, clamped to ±8 = ±0.5 px). The sweep is
// sparse (only out[x] != 0), so a branchy scalar loop is fine — edge pixels are
// typically 1-5% of the row.
inline void nmsSubpixelRow(const std::uint16_t* pa, const std::uint16_t* pc,
                           const std::uint16_t* pb, const std::uint8_t* dir,
                           const std::uint8_t* edge, int w, std::int8_t* __restrict delta) {
    std::memset(delta, 0, std::size_t(w));
    auto at = [&](const std::uint16_t* row, int x) { return (x >= 0 && x < w) ? int(row[x]) : 0; };
    auto one = [&](int x) {
        if (!edge[x]) return;
        int Pm, Pp;
        if (dir[x] == 1) { Pm = at(pc, x - 1); Pp = at(pc, x + 1); }  // Vertical
        else             { Pm = at(pa, x);     Pp = at(pb, x);     }  // Horizontal
        int c = pc[x];
        int den = 2 * c - Pm - Pp;  // >= 0 at a local max
        if (den <= 0) return;
        // float divide: same result as the int divide for our ranges, but much
        // cheaper than the 32-bit integer division on x86.
        int d16 = int(float(8 * (Pp - Pm)) / float(den));
        if (d16 > 8) d16 = 8;
        if (d16 < -8) d16 = -8;
        delta[x] = std::int8_t(d16);
    };
    // Edge rows are sparse. An edge byte is 0 or 1, so each set byte of the
    // 8-byte word carries exactly one bit: count-trailing-zeros jumps straight
    // to the next edge pixel and `wd &= wd - 1` consumes it — zero bytes are
    // never touched (the previous version still branched on all 8).
    int x = 0;
    for (; x + 8 <= w; x += 8) {
        std::uint64_t wd;
        std::memcpy(&wd, edge + x, 8);
        while (wd) {
            one(x + (ctz64(wd) >> 3));
            wd &= wd - 1;
        }
    }
    for (; x < w; ++x) one(x);
}

// ---- Stage 2: endpoint-candidate classification (thesis §3.2.2) ----------
// Thin the 5x5 outer ring, count the survivors, and flag the centre as an
// endpoint candidate unless it is a straight line passing through (exactly two
// survivors that are far apart on the ring).
//
// `endpointCore` is written entirely branch-free so the row driver below can be
// auto-vectorized. Bit-identical to the original boolean formulation.
//
// All values here are 0/1 flags, counts <= 16 and 8-bit masks, so the whole
// dataflow fits in uint8_t — which lets the vectorizer pack 2-4x more lanes
// per register than the equivalent int formulation.
template <class L>
inline bool endpointCore(L e) {
    using U = std::uint8_t;
    // Inner ring a..h (thesis fig. 3.11): a=left, b=TL, c=top, d=TR,
    // eR=right, fR=BR, gR=bottom, hR=BL.
    U a = U(e(-1, 0)), b = U(e(-1, -1)), c = U(e(0, -1)), d = U(e(1, -1));
    U eR = U(e(1, 0)), fR = U(e(1, 1)), gR = U(e(0, 1)), hR = U(e(-1, 1));

    // Outer ring i..xx (5x5 border) walking around from the left-mid pixel.
    U i = U(e(-2, 0)), j = U(e(-2, -1)), k = U(e(-2, -2)), l = U(e(-1, -2));
    U m = U(e(0, -2)),  n = U(e(1, -2)),  o = U(e(2, -2)),  p = U(e(2, -1));
    U q = U(e(2, 0)),   r = U(e(2, 1)),   s = U(e(2, 2)),   t = U(e(1, 2));
    U u = U(e(0, 2)),   v = U(e(-1, 2)),  w = U(e(-2, 2)),  xx = U(e(-2, 1));

    U ab = a | b, bc = b | c, cd = c | d, de = d | eR;
    U ef = eR | fR, fg = fR | gR, gh = gR | hR, ha = hR | a;
    U hab = ha | b, bcd = bc | d, def = de | fR, fgh = fg | hR;

    i &= hab; m &= bcd; q &= def; u &= fgh;

    U hai = ha | i, abi = ab | i, bcm = bc | m, cdm = cd | m;
    U deq = de | q, efq = ef | q, fgu = fg | u, ghu = gh | u;

    j &= abi; l &= bcm; n &= cdm; p &= deq;
    r &= efq; t &= fgu; v &= ghu; xx &= hai;

    // "a && !b" on 0/1 values, branch-free, is "a & (b ^ 1)".
    U xj = xx | j, ln = l | n, pr = p | r, tv = t | v;
    i &= (xj ^ 1); m &= (ln ^ 1); q &= (pr ^ 1); u &= (tv ^ 1);

    U jl = j & l, np = n & p, rt = r & t, vx = v & xx;
    U bjl = b | jl, dnp = d | np, frt = fR | rt, hvx = hR | vx;

    k &= bjl; o &= dnp; s &= frt; w &= hvx;
    j &= (k ^ 1); l &= (k ^ 1); n &= (o ^ 1); p &= (o ^ 1);
    r &= (s ^ 1); t &= (s ^ 1); v &= (w ^ 1); xx &= (w ^ 1);

    U count = U(i + j + k + l + m + n + o + p + q + r + s + t + u + v + w + xx);

    // Ring "thermometer" masks (i contributes 0): XOR of the present pixels'
    // masks; its popcount is the shorter arc distance between two survivors.
    // (0 - flag) is 0x00 or 0xff in uint8, so each term is mask-or-zero.
    U xm = U((U(0) - j) & 0x01) ^ U((U(0) - k) & 0x03) ^ U((U(0) - l) & 0x07) ^
           U((U(0) - m) & 0x0f) ^ U((U(0) - n) & 0x1f) ^ U((U(0) - o) & 0x3f) ^
           U((U(0) - p) & 0x7f) ^ U((U(0) - q) & 0xff) ^ U((U(0) - r) & 0xfe) ^
           U((U(0) - s) & 0xfc) ^ U((U(0) - t) & 0xf8) ^ U((U(0) - u) & 0xf0) ^
           U((U(0) - v) & 0xe0) ^ U((U(0) - w) & 0xc0) ^ U((U(0) - xx) & 0x80);
    U pc = xm - ((xm >> 1) & 0x55);            // SWAR popcount of the 8 bits
    pc = (pc & 0x33) + ((pc >> 2) & 0x33);
    pc = U(pc + (pc >> 4)) & 0x0f;

    U straight = U(count == 2) & U(pc > 6);
    return straight == 0;
}

// Classify a whole row into the endpoint-candidate map (0 = None, 1 = Interior,
// 2 = Endpoint), given the 5 edge rows e0..e4 (= rows y-2..y+2; pass a zero row
// for any outside the image).
//
// Sparse fast path (speed improvement, exact): out[x] = rows[2][x]*(isEnd+1),
// so wherever the CENTRE row is zero the output is zero regardless of the
// neighbourhood. Edge maps are typically 1-5% dense, so scanning the centre row
// 8 bytes at a time and skipping all-zero words removes ~95% of the
// endpointCore evaluations while producing bit-identical output. Non-zero words
// fall back to the branch-free 8px loop (still auto-vectorizable).
// The branch-free interior sweep, [x0, x1). Kept out-of-line on purpose
// (`noinline` is load-bearing): inlined into the (large) one-pass driver, GCC
// gives up on vectorizing the endpointCore dataflow and the stage runs ~10x
// slower; compiled standalone it vectorizes the same way for every caller.
// (`inline` stays for linkage — the attribute only forbids inlining the body.)
inline SWEEPLSD_NOINLINE void featureRowInterior(
        const std::uint8_t* e0, const std::uint8_t* e1, const std::uint8_t* e2,
        const std::uint8_t* e3, const std::uint8_t* e4, int x0, int x1,
        std::uint8_t* __restrict out) {
    // The rows must arrive as five separate pointers: passed as an array the
    // vectorizer cannot rule out aliasing through the extra indirection and
    // falls back to scalar code.
    const std::uint8_t* rows[5] = {e0, e1, e2, e3, e4};
    for (int x = x0; x < x1; ++x) {
        int isEnd = endpointCore([&](int dx, int dy) { return int(rows[dy + 2][x + dx]); }) ? 1 : 0;
        out[x] = std::uint8_t(int(rows[2][x]) * (isEnd + 1));  // 0 / 1 / 2
    }
}

inline void featureRow(const std::uint8_t* e0, const std::uint8_t* e1, const std::uint8_t* e2,
                       const std::uint8_t* e3, const std::uint8_t* e4, int w, bool sparse,
                       std::uint8_t* __restrict out) {
    const std::uint8_t* rows[5] = {e0, e1, e2, e3, e4};
    auto border = [&](int x) {
        int isEnd = endpointCore([&](int dx, int dy) {
                        int xx = x + dx;
                        return (xx >= 0 && xx < w) ? int(rows[dy + 2][xx]) : 0;
                    }) ? 1 : 0;
        out[x] = std::uint8_t(int(rows[2][x]) * (isEnd + 1));  // 0 / 1 / 2
    };
    if (w < 5) {
        for (int x = 0; x < w; ++x) border(x);
        return;
    }
    auto interior = [&](int x0, int x1) { featureRowInterior(e0, e1, e2, e3, e4, x0, x1, out); };
    border(0); border(1);
    const int xe = w - 2;
    int x = 2;
    bool use_sparse = false;
    if (sparse) {
        // Prescan: the span path only wins when most centre-row words are zero
        // (clustered edges in a mostly-empty row). Noise speckle at a few
        // percent density already touches most 8-byte words, in which case the
        // fully vectorized dense loop is faster — so measure first and choose.
        int zero_words = 0, words = 0;
        for (int p = 2; p + 8 <= xe; p += 8, ++words) {
            std::uint64_t wd;
            std::memcpy(&wd, e2 + p, 8);
            zero_words += (wd == 0);
        }
        use_sparse = words > 0 && zero_words * 4 >= words * 3;  // >= 75% zero
    }
    if (use_sparse) {
        // Span-based sparse scan: zero words of the CENTRE row become zero
        // output (memset); maximal runs of non-zero words are handed to the
        // branch-free interior loop in ONE call, so it still auto-vectorizes
        // over long spans instead of 8px crumbs.
        while (x + 8 <= xe) {
            std::uint64_t wd;
            std::memcpy(&wd, e2 + x, 8);
            if (wd == 0) {
                std::memset(out + x, 0, 8);
                x += 8;
                continue;
            }
            int span = x;
            x += 8;
            while (x + 8 <= xe) {
                std::memcpy(&wd, e2 + x, 8);
                if (wd == 0) break;
                x += 8;
            }
            interior(span, x);
        }
    }
    interior(x, xe);
    border(w - 2); border(w - 1);
}

}  // namespace kernels
}  // namespace sweeplsd
