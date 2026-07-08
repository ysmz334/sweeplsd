#include "backend.hpp"

#ifndef __SYNTHESIS__
#include <cassert>
#endif

// Event-driven labelling engine (thesis §3.2.3-3.2.4). Work is proportional
// to the number of feature pixels (1-5 % of the image on real photos), not to
// the pixel count — that is what lets this half of the pipeline run elastic
// behind the FIFO while the front-end holds II=1 (DESIGN.md).
//
// Semantics are a transliteration of sweeplsd::Labeler::Impl::processRow
// (../../src/labeling.cpp), baseline configuration, with two hardware
// substitutions, both output-invariant:
//
//  1. Row/feature buffers are validated by a stored row tag instead of being
//     cleared every row (clearing would cost O(width) per row and drag the
//     engine back to pixel rate). A cell is "present" only if its tag equals
//     the row being read; anything staler reads as empty, exactly like the
//     golden model's freshly-zeroed rows.
//
//  2. Labels are recycled through a free-list ring (the original reference
//     implementation's scheme) instead of growing the table without bound.
//     A label is freed once its last activity row is two full rows behind
//     the scan (see the scavenger below for why that is safe under full
//     union-find path compression). Ids never influence the output — the
//     merge-survivor rule orders by (last_row, latest_x) and segments are
//     emitted in scan order — so recycling preserves golden parity.
//
// Style note: state lives in file-scope statics and the steps are plain
// functions, not capturing lambdas — closures that capture arrays (or other
// closures) lower to pointer-to-pointer, which Vitis HLS rejects [HLS 214-134].

namespace sweeplsd_hls {

namespace {

// Per-label state. The C model keeps one struct array; hardware splits the
// fields into parallel BRAMs (hot: connect/recency/moments, cold: start
// contact — the golden model documents the same hot/cold split).
// Hardware widths: connect u10, last_row/latest_x u11, pix_num u18,
// x_sum/y_sum u30, squares/cross u41, start u11+u11, has_start 1 bit.
struct LabelState {
    std::uint16_t connect;  // 0 = root, else the label this one merged into
    std::int32_t last_row;  // most recent accumulate (row, then x) — recency
    std::int32_t latest_x;
    std::uint32_t pix_num;
    std::uint64_t x_sum, y_sum, x_sq_sum, y_sq_sum, xy_sum;
    std::uint32_t strong_cnt;  // (d) hysteresis: pixels with power >= high th
    bool has_start;
    std::uint16_t start_x, start_y;
    // (f) bbox extreme points (7 fields, u11 each in hardware). max_y needs
    // no field: rows arrive in order, so max_y == last_row at all times
    // (incl. merges — the survivor rule keeps the larger last_row, and the
    // golden merge's strict `>` tie-break can then never prefer the loser's
    // max_y point). max_y_x = x of the FIRST pixel of the newest row, which
    // is not latest_x (the LAST x of that row) — it rides the existing
    // first-touch-this-row branch in accumulate(). In hardware creation
    // co-occurs with the first accumulate, so the sentinels below reduce to
    // a direct write of (x, y).
    std::int32_t min_x, min_x_y, max_x, max_x_y;
    std::int32_t min_y, min_y_x, max_y_x;
};

inline wide_t mulw(std::uint64_t a, std::uint64_t b) {
    return wide_t(std::int64_t(a)) * wide_t(std::int64_t(b));
}

// ---- engine state (BRAM; sizes are the DESIGN.md memory budget) ------------
LabelState lab[kMaxLabels];
std::uint16_t freelist[kMaxLabels];    // ring of recyclable ids
int fl_head, fl_count;
std::uint16_t row_label[kMaxWidth];    // one label row, tag-validated:
std::int32_t row_tag[kMaxWidth];       //   tag y = cur row, y-1 = prev row
std::uint8_t feat_kind[3][kMaxWidth];  // feature rows y-1, y, y+1 (mod 3)
std::int32_t feat_tag[3][kMaxWidth];
std::uint16_t xlist[2][kMaxWidth];     // interior columns of row y (ping-pong:
std::uint8_t xstrong[2][kMaxWidth];    //   + each column's strong bit (d)
int xcount[2];                         //   row y is processed while y+1 ingests)
std::uint16_t touched[3][kMaxLabels];  // labels first-touched in row r (mod 3),
int tcount[3];                         //   consumed by the scavenger at row r+2
int g_width, g_height, g_pix_th;       // frame parameters
bool g_hyst_on;                        // (d) hysteresis gate active
int g_hyst_strong_min;                 //   reject labels with fewer strong pixels
int g_border;                          // (i) border margin: skip labelling within
                                       //   this many px of the frame (0 = off)
int g_mps_2sq;                         // (h) curve reject: 2*max_perp_spread^2 as an
                                       //   integer (0 = off; 2 for the default mps=1)

// Rotating mod-3 indices, advanced by increment-and-wrap. Never compute
// `row % 3` directly: a non-power-of-two remainder synthesises to a
// multi-cycle sequential divider [urem_.._36_seq], which would put ~36 dead
// cycles into every feature access of the event path.
int g_ing_b;                        // feature buffer receiving the ingest row
int g_buf_prev, g_buf_cur, g_buf_next;  // buffers of rows y-1 / y / y+1
int g_tl_cur;                       // touched-list index of the processed row

static_assert((kMaxLabels & (kMaxLabels - 1)) == 0,
              "kMaxLabels must be a power of two (free-list ring uses & mask)");
constexpr int kLabelMask = kMaxLabels - 1;

inline int rot3(int b) { return b == 2 ? 0 : b + 1; }

#ifndef __SYNTHESIS__
BackendStats g_stats;
#endif

// Feature value at (x, r) with the row's buffer index `b` supplied by the
// caller (see the rotating indices above); 0 (None) outside the image or when
// the cell is stale — identical to the golden model's zero-row stand-ins and
// bounds-checked reads.
int featAt(int b, int r, int x) {
    if (r < 0 || r >= g_height || x < 0 || x >= g_width) return 0;
    return feat_tag[b][x] == r ? int(feat_kind[b][x]) : 0;
}

// Label row cell of row r (only r = y-1 / y are ever live).
int labelAt(int r, int x) {
    if (x < 0 || x >= g_width) return 0;
    return row_tag[x] == r ? int(row_label[x]) : 0;
}

// Union-find resolve with full path compression (== golden find()).
int findRoot(int id) {
    int root = id;
#ifndef __SYNTHESIS__
    int depth = 0;
#endif
find_chase:
    while (lab[root].connect != 0) {
        root = lab[root].connect;
#ifndef __SYNTHESIS__
        if (++depth > g_stats.max_find_chain) g_stats.max_find_chain = depth;
#endif
    }
    int cur = id;
find_compress:
    while (lab[cur].connect != 0) {
        int nxt = lab[cur].connect;
        lab[cur].connect = std::uint16_t(root);
        cur = nxt;
    }
    return root;
}

int createLabel() {
    if (fl_count == 0) {
#ifndef __SYNTHESIS__
        ++g_stats.freelist_underflows;  // must never happen (checked by tb)
#endif
        return 0;
    }
    const int id = freelist[fl_head];
    fl_head = (fl_head + 1) & kLabelMask;
    --fl_count;
#ifndef __SYNTHESIS__
    const int live = kMaxLabels - 1 - fl_count;
    if (live > g_stats.max_live_labels) g_stats.max_live_labels = live;
#endif
    LabelState& L = lab[id];
    L.connect = 0;
    L.last_row = -1;
    L.latest_x = -1;
    L.pix_num = 0;
    L.x_sum = L.y_sum = L.x_sq_sum = L.y_sq_sum = L.xy_sum = 0;
    L.strong_cnt = 0;
    L.has_start = false;
    L.start_x = L.start_y = 0;
    L.min_x = INT32_MAX;  // any x wins strictly on the first accumulate
    L.max_x = -1;
    L.min_y = INT32_MAX;
    L.min_x_y = L.max_x_y = L.min_y_x = L.max_y_x = 0;
    return id;
}

// Line judgment (thesis §3.2.4), exact integer form. Accept iff
// pix_num >= th and lambda_min/lambda_max <= kAspectNum/kAspectDen, i.e.
//   (den-num)^2 * T^2 <= (den+num)^2 * R^2,
// T = ma+mc >= 0, R^2 = (ma-mc)^2 + 4 mb^2 <= T^2 (PSD), so both sides
// fit 128 bits. This replaces the software's sqrt with no approximation;
// the only conceivable divergence is an input landing exactly between
// 1/20 and double(0.05) (~3e-18 relative) — the parity testbench would
// catch it, and none occurs on the evaluation corpus.
void judgeAndEmit(hls::stream<SegmentRecord>& out, const LabelState& L, int sx,
                  int sy, int ex, int ey) {
// Once-per-segment unit (~10^3 calls/frame vs 10^6 pixels): keep it out of
// line and time-multiplex ONE multiplier through all nine wide products —
// fully parallel they cost ~177 DSPs (196% of an xc7a35t).
#pragma HLS INLINE off
#pragma HLS ALLOCATION operation instances=mul limit=1
    if (int(L.pix_num) < g_pix_th) return;
    // (d) hysteresis gate: a segment must contain enough strong pixels.
    if (g_hyst_on && int(L.strong_cnt) < g_hyst_strong_min) return;
    // (i) border margin: drop a segment whose bounding box reaches within
    // g_border px of the frame (the 2x2 gradient bias fringes the image edge).
    // A pure integer bbox test on the record's own extremes (max_y == last_row),
    // so it is trivially bit-exact across SW / HLS / RTL.
    if (g_border > 0 &&
        (int(L.min_x) < g_border || int(L.max_x) >= g_width - g_border ||
         int(L.min_y) < g_border || int(L.last_row) >= g_height - g_border))
        return;
    const wide_t ma = mulw(L.pix_num, L.x_sq_sum) - mulw(L.x_sum, L.x_sum);
    const wide_t mb = mulw(L.pix_num, L.xy_sum) - mulw(L.x_sum, L.y_sum);
    const wide_t mc = mulw(L.pix_num, L.y_sq_sum) - mulw(L.y_sum, L.y_sum);
    const wide_t T = ma + mc;
    if (T <= 0) return;  // degenerate scatter (golden: denom <= 0)
    const wide_t d = ma - mc;
    const uwide_t T2 = uwide_t(T) * uwide_t(T);
    const uwide_t R2 = uwide_t(d * d) + uwide_t(4) * uwide_t(mb * mb);
    constexpr int kRejN = (kAspectDen - kAspectNum) * (kAspectDen - kAspectNum);
    constexpr int kRejD = (kAspectDen + kAspectNum) * (kAspectDen + kAspectNum);
    if (uwide_t(kRejN) * T2 > uwide_t(kRejD) * R2) return;  // not elongated
    // (h) max_perp_spread: reject if the smaller eigenvalue of the NORMALISED
    // covariance exceeds max_perp_spread^2 (a curved arc bows off its chord and
    // inflates it). ev_min = 0.5*(T-R)/N^2, so ev_min > mps^2  <=>
    // A := T - 2*mps^2*N^2 > R = sqrt(R2); done sqrt-free as (A>0 && A^2 > R2),
    // reusing the aspect test's T and R2 (all terms stay < 2^128).
    if (g_mps_2sq > 0) {
        const wide_t N2 = mulw(L.pix_num, L.pix_num);
        const wide_t A = T - wide_t(g_mps_2sq) * N2;
        if (A > 0 && uwide_t(A) * uwide_t(A) > R2) return;  // too much perp spread
    }
    SegmentRecord rec;
    rec.sx = std::uint16_t(sx);
    rec.sy = std::uint16_t(sy);
    rec.ex = std::uint16_t(ex);
    rec.ey = std::uint16_t(ey);
    rec.n = L.pix_num;
    rec.x_sum = L.x_sum;
    rec.y_sum = L.y_sum;
    rec.x_sq_sum = L.x_sq_sum;
    rec.y_sq_sum = L.y_sq_sum;
    rec.xy_sum = L.xy_sum;
    rec.min_x = std::uint16_t(L.min_x);
    rec.min_x_y = std::uint16_t(L.min_x_y);
    rec.max_x = std::uint16_t(L.max_x);
    rec.max_x_y = std::uint16_t(L.max_x_y);
    rec.min_y = std::uint16_t(L.min_y);
    rec.min_y_x = std::uint16_t(L.min_y_x);
    rec.max_y = std::uint16_t(L.last_row);  // rows in order: max_y == last_row
    rec.max_y_x = std::uint16_t(L.max_y_x);
    out.write(rec);
}

// Merge two root labels; returns the surviving id. Transliterates the
// golden merge(): survivor = greater (last_row, latest_x); the loser's
// connect points at the survivor; a merge of two started halves closes a
// segment, judged on the *combined* moments with the contacts in (l0, l1)
// argument order.
int mergeLabels(hls::stream<SegmentRecord>& out, int l0, int l1) {
    const LabelState a = lab[l0];  // by value: the survivor aliases one of them
    const LabelState b = lab[l1];
    const bool keep0 = a.last_row > b.last_row ||
                       (a.last_row == b.last_row && a.latest_x >= b.latest_x);
    const int main_id = keep0 ? l0 : l1;
    const int other_id = keep0 ? l1 : l0;
    LabelState& M = lab[main_id];
    M.pix_num = a.pix_num + b.pix_num;
    M.x_sum = a.x_sum + b.x_sum;
    M.x_sq_sum = a.x_sq_sum + b.x_sq_sum;
    M.y_sum = a.y_sum + b.y_sum;
    M.y_sq_sum = a.y_sq_sum + b.y_sq_sum;
    M.xy_sum = a.xy_sum + b.xy_sum;
    M.strong_cnt = a.strong_cnt + b.strong_cnt;
    // (f) bbox union, loser into survivor; strict compares == the golden
    // merge (ties keep the survivor's point). The golden's max_y line can
    // never fire (survivor.last_row >= loser.last_row), so max_y_x stays.
    const LabelState& O = keep0 ? b : a;
    if (O.min_x < M.min_x) { M.min_x = O.min_x; M.min_x_y = O.min_x_y; }
    if (O.max_x > M.max_x) { M.max_x = O.max_x; M.max_x_y = O.max_x_y; }
    if (O.min_y < M.min_y) { M.min_y = O.min_y; M.min_y_x = O.min_y_x; }
    lab[other_id].connect = std::uint16_t(main_id);

    const LabelState& A0 = keep0 ? a : b;  // survivor's / loser's cold view
    const LabelState& O0 = keep0 ? b : a;
    if (A0.has_start && O0.has_start) {
        judgeAndEmit(out, M, a.start_x, a.start_y, b.start_x, b.start_y);
    } else if (!A0.has_start && O0.has_start) {
        M.has_start = true;
        M.start_x = O0.start_x;
        M.start_y = O0.start_y;
    }
    return main_id;
}

void accumulate(int id, int x, int y, int strong) {
    LabelState& L = lab[id];
    if (L.last_row != y) {  // first touch this row -> scavenger candidate
        touched[g_tl_cur][tcount[g_tl_cur]++] = std::uint16_t(id);
        L.max_y_x = x;  // (f) first pixel of the new bottom row (== golden
                        //     `y > max_y` branch, since max_y == last_row)
    }
    if (x < L.min_x) { L.min_x = x; L.min_x_y = y; }
    if (x > L.max_x) { L.max_x = x; L.max_x_y = y; }
    if (y < L.min_y) { L.min_y = y; L.min_y_x = x; }
    L.strong_cnt += std::uint32_t(strong);  // (d) hysteresis
    L.pix_num += 1;
    L.x_sum += std::uint64_t(x);
    L.x_sq_sum += std::uint64_t(x) * std::uint64_t(x);
    L.y_sum += std::uint64_t(y);
    L.y_sq_sum += std::uint64_t(y) * std::uint64_t(y);
    L.xy_sum += std::uint64_t(x) * std::uint64_t(y);
    L.last_row = y;
    L.latest_x = x;
}

// Free every label whose last activity row is `row` (== two rows behind
// the scan). Safety: a label can only be referenced (a) raw, from a row
// cell — cells written in row t hold ids whose last_row >= t (the write
// co-occurs with an accumulate), and only rows y-1/y are live — or
// (b) through a connect chain — every traversable chain node accumulated
// (as a merge survivor) no earlier than one row before any cell that can
// still reach it, so its last_row is also >= y-1. Hence last_row <= y-2
// implies unreachable. This is the original reference implementation's
// y_mod_2/latest_x recycling rule made one row more conservative, which
// is what full path compression (the golden semantics) requires.
void scavenge(int li, int row) {
    if (row < 0) return;
scavenge_loop:
    for (int i = 0; i < tcount[li]; ++i) {
        const int id = touched[li][i];
        if (lab[id].last_row == row) {  // final activity was `row` -> dead
            freelist[(fl_head + fl_count) & kLabelMask] = std::uint16_t(id);
            ++fl_count;
        }
    }
    tcount[li] = 0;
}

// One interior pixel — the golden pixel() body, baseline configuration.
void processPixel(hls::stream<SegmentRecord>& out, int y, int x, int strong) {
    const int aL = featAt(g_buf_prev, y - 1, x - 1), aC = featAt(g_buf_prev, y - 1, x),
              aR = featAt(g_buf_prev, y - 1, x + 1);
    const int cL = featAt(g_buf_cur, y, x - 1), cR = featAt(g_buf_cur, y, x + 1);
    const int bL = featAt(g_buf_next, y + 1, x - 1), bC = featAt(g_buf_next, y + 1, x),
              bR = featAt(g_buf_next, y + 1, x + 1);
    constexpr int kInt = int(kEventInterior), kEnd = int(kEventEndpoint);

    // Causal neighbours that can carry a label: NE, N, NW, W (golden order).
    const bool nedge[4] = {aR == kInt, aC == kInt, aL == kInt, cL == kInt};
    const int nlabel[4] = {labelAt(y - 1, x + 1), labelAt(y - 1, x),
                           labelAt(y - 1, x - 1), labelAt(y, x - 1)};

    int edge_num = 0, label0 = 0, label1 = 0;
gather:
    for (int i = 0; i < 4; ++i) {
        if (!nedge[i]) continue;
        ++edge_num;
        const int lr = findRoot(nlabel[i]);
        if (label0 == 0) label0 = lr;
        else if (lr != label0 && label1 == 0) label1 = lr;
    }

    int center;
    if (edge_num == 0) center = createLabel();
    else if (label1 == 0) center = label0;
    else center = mergeLabels(out, label0, label1);

    accumulate(center, x, y, strong);
    row_label[x] = std::uint16_t(center);
    row_tag[x] = y;

    // Endpoint contact: Endpoint (=2) is the only feature with bit 1 set.
    const bool touches_end = ((aL | aC | aR | cL | cR | bL | bC | bR) & kEnd) != 0;
    if (touches_end) {
        LabelState& L = lab[center];
        if (L.has_start) {
            judgeAndEmit(out, L, L.start_x, L.start_y, x, y);
        } else {
            L.has_start = true;
            L.start_x = std::uint16_t(x);
            L.start_y = std::uint16_t(y);
        }
    }
}

void processRow(hls::stream<SegmentRecord>& out, int y) {
    // Rotate the mod-3 indices instead of computing y % 3 (see above). Rows
    // are processed strictly in order y = 0, 1, 2, ...; the ingest buffer
    // g_ing_b currently holds row y+1.
    g_tl_cur = (y == 0) ? 0 : rot3(g_tl_cur);
    g_buf_next = g_ing_b;
    g_buf_cur = g_buf_next == 0 ? 2 : g_buf_next - 1;
    g_buf_prev = g_buf_cur == 0 ? 2 : g_buf_cur - 1;

    const int p = y & 1;
#ifndef __SYNTHESIS__
    if (xcount[p] > g_stats.max_row_events) g_stats.max_row_events = xcount[p];
#endif
row_loop:
    for (int i = 0; i < xcount[p]; ++i)
        processPixel(out, y, int(xlist[p][i]), int(xstrong[p][i]));
    xcount[p] = 0;
    scavenge(rot3(g_tl_cur), y - 2);  // (y-2) mod 3 == (y+1) mod 3
}

}  // namespace

#ifndef __SYNTHESIS__
BackendStats backendStats() { return g_stats; }
#endif

void sweeplsdBackend(hls::stream<Event>& events, hls::stream<SegmentRecord>& out,
                     int width, int height, int pixel_num_th, const HystCfg& hyst,
                     int border_margin, int mps_2sq) {
    g_width = width;
    g_height = height;
    g_pix_th = pixel_num_th;
    g_hyst_on = hyst.on;
    g_hyst_strong_min = hyst.strong_min;
    g_border = border_margin;
    g_mps_2sq = mps_2sq;
    fl_head = 0;
    fl_count = kMaxLabels - 1;  // ids 1..kMaxLabels-1; 0 = "no label"
    xcount[0] = xcount[1] = 0;
    tcount[0] = tcount[1] = tcount[2] = 0;
init_freelist:
    for (int i = 0; i < kMaxLabels - 1; ++i) freelist[i] = std::uint16_t(i + 1);
init_tags:
    for (int x = 0; x < width; ++x) {
#pragma HLS PIPELINE II = 1
        row_tag[x] = -1;
        feat_tag[0][x] = feat_tag[1][x] = feat_tag[2][x] = -1;
    }

    // Event loop: ingest row y+1 while processing row y.
    int ingest_y = 0;
    g_ing_b = 0;
    bool done = false;
event_loop:
    while (!done) {
        const Event e = events.read();
        switch (e.kind) {
            case kEventInterior:
            case kEventEndpoint: {
                feat_kind[g_ing_b][e.x] = e.kind;
                feat_tag[g_ing_b][e.x] = ingest_y;
                if (e.kind == kEventInterior) {
                    const int p = ingest_y & 1;
                    xstrong[p][xcount[p]] = e.strong;  // (d) hysteresis
                    xlist[p][xcount[p]++] = e.x;
                }
                break;
            }
            case kEventEndOfRow:
                if (ingest_y >= 1) processRow(out, ingest_y - 1);
                ++ingest_y;
                g_ing_b = rot3(g_ing_b);
                break;
            case kEventEndOfFrame:
            default:
                if (ingest_y >= 1) processRow(out, ingest_y - 1);  // last row; below = none
                done = true;
                break;
        }
    }

    SegmentRecord term = {};
    term.n = 0;  // terminator
    out.write(term);
}

void sweeplsdCore(hls::stream<std::uint8_t>& src, hls::stream<SegmentRecord>& out,
                  int width, int height, int power_th, bool strict, int pixel_num_th,
                  const HystCfg& hyst, int border_margin, int mps_2sq) {
#pragma HLS DATAFLOW
    static hls::stream<Event> ev;
#pragma HLS STREAM variable = ev depth = 2048
    sweeplsdFrontend(src, ev, width, height, power_th, strict, hyst);
    sweeplsdBackend(ev, out, width, height, pixel_num_th, hyst, border_margin, mps_2sq);
}

}  // namespace sweeplsd_hls

void sweeplsd_core_top(hls::stream<std::uint8_t>& src,
                       hls::stream<sweeplsd_hls::SegmentRecord>& out, int width,
                       int height, int power_th, bool strict, int pixel_num_th,
                       bool hyst_on, bool hyst_adaptive, int hyst_low,
                       int hyst_strong_min) {
#pragma HLS INTERFACE axis port = src
#pragma HLS INTERFACE axis port = out
#pragma HLS INTERFACE s_axilite port = width
#pragma HLS INTERFACE s_axilite port = height
#pragma HLS INTERFACE s_axilite port = power_th
#pragma HLS INTERFACE s_axilite port = strict
#pragma HLS INTERFACE s_axilite port = pixel_num_th
#pragma HLS INTERFACE s_axilite port = hyst_on
#pragma HLS INTERFACE s_axilite port = hyst_adaptive
#pragma HLS INTERFACE s_axilite port = hyst_low
#pragma HLS INTERFACE s_axilite port = hyst_strong_min
#pragma HLS INTERFACE s_axilite port = return
    sweeplsd_hls::HystCfg hyst{hyst_on, hyst_adaptive, hyst_low, hyst_strong_min};
    sweeplsd_hls::sweeplsdCore(src, out, width, height, power_th, strict, pixel_num_th,
                               hyst);
}
