// (B) vertical-stripe parallelism — design probe. Quantifies the ONE
// correctness cost of splitting the labelling back-end into two independent
// engines that each own a vertical column range [0,M) and [M,W): a connected
// component that crosses the boundary column M is labelled as two separate
// components (one per engine) instead of one, so it is judged as its two halves
// rather than as a whole.
//
// The two engines need NO cross-engine wiring: give both the GLOBAL width and
// global x coordinates, and route each data event to the engine whose range
// contains its x (row/frame markers go to both). An out-of-range neighbour is
// simply never ingested by an engine, so it reads as background there — exactly
// the boundary split. The (i) border margin, which uses the global x, stays
// correct (the internal boundary is not near x=0 or x=W, so it is never
// mistaken for a frame edge).
//
// This runs the bit-exact HLS C model: one single-engine pass = the golden
// segment set, then the two-engine split, and compares the accepted-segment
// SETS. It reports how many golden segments are unaffected, how many cross the
// boundary (the at-risk set), and the net change in accepted segments. No
// timing model here — that is the overflow half of the story (fifo_dropsim);
// this tool answers only "what does the split cost in fidelity?".
//
//   stripe_probe <img...|@manifest> [--boundary M] [--stripes 2|4] [--csv out.csv]
//
// --boundary M fixes the split column (default W/2). --stripes 4 splits into
// four equal ranges (three internal boundaries) to preview deeper parallelism.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "backend.hpp"
#include "frontend.hpp"
#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"

using namespace sweeplsd;
namespace H = sweeplsd_hls;

namespace {

int g_stripes = 2;      // number of vertical engines
int g_boundary = -1;    // fixed split column for --stripes 2 (default W/2)
// Per-engine event-FIFO drop model (same policy as fifo_dropsim / event_fifo.v,
// drop_mode=1). Cost constants are the calibrated real back-end AFTER the
// parallel-skip gather optimisation (commit 34e8442): ~Cavg 11.5, i.e. ing=2 +
// proc=13 per interior. Each stripe engine drains its own FIFO in the shared
// pixel clock; a busy stripe still overflows, so the split's overflow win
// depends on load balance, not just on halving the average.
int g_ing = 2;
int g_proc = 13;
int g_hblank = 280;     // 1080p30 horizontal blanking clocks / line
int g_depth = 2048;     // per-engine FIFO depth (data dropped at occ >= depth-8)

// A full record signature: two records are the same accepted segment iff every
// field matches (a segment wholly inside one stripe reproduces the golden record
// exactly; a boundary-crossing one differs in every field).
std::string key(const H::SegmentRecord& r) {
    char buf[256];
    std::snprintf(buf, sizeof buf,
                  "%u,%u,%u,%u,%u,%llu,%llu,%llu,%llu,%llu,%u,%u,%u,%u,%u,%u,%u,%u",
                  r.sx, r.sy, r.ex, r.ey, r.n,
                  (unsigned long long)r.x_sum, (unsigned long long)r.y_sum,
                  (unsigned long long)r.x_sq_sum, (unsigned long long)r.y_sq_sum,
                  (unsigned long long)r.xy_sum, r.min_x, r.min_x_y, r.max_x,
                  r.max_x_y, r.min_y, r.min_y_x, r.max_y, r.max_y_x);
    return buf;
}

// One data event with its row and produce-cycle (1080p30 geometry): the
// front-end is II=1 so the event at (x,y) is produced at y*(W+hblank)+x.
struct EvT {
    std::uint8_t kind;
    std::uint16_t x;
    std::uint8_t strong;
    int y;
    long pcyc;
};

// Run the back-end over an explicit event list; returns accepted records.
std::vector<H::SegmentRecord> runEventsRec(const std::vector<H::Event>& evs, int w, int h,
                                           const Params& p) {
    hls::stream<H::Event> es;
    for (const H::Event& e : evs) es.write(e);
    hls::stream<H::SegmentRecord> rec;
    const H::HystCfg hyst{p.use_hysteresis, p.hysteresis_adaptive, p.hysteresis_low_th,
                          p.hysteresis_strong_min};
    const int mps_2sq = int(2.0 * p.max_perp_spread * p.max_perp_spread + 0.5);
    H::sweeplsdBackend(es, rec, w, h, p.pixel_num_th, hyst, p.border_margin, mps_2sq);
    std::vector<H::SegmentRecord> out;
    while (!rec.empty()) {
        H::SegmentRecord r = rec.read();
        if (r.n == 0) break;
        out.push_back(r);
    }
    return out;
}

// Back-end over the events whose x is in [lo, hi) (markers always pass); global
// width so the border test / neighbour bookkeeping use true coordinates.
std::vector<H::SegmentRecord> runRange(const std::vector<H::Event>& evs, int lo, int hi,
                                       int w, int h, const Params& p) {
    std::vector<H::Event> sub;
    for (const H::Event& e : evs) {
        const bool data = (e.kind == H::kEventInterior || e.kind == H::kEventEndpoint);
        if (data && !(int(e.x) >= lo && int(e.x) < hi)) continue;
        sub.push_back(e);
    }
    return runEventsRec(sub, w, h, p);
}

// One engine's stream = the events with x in [lo,hi) plus every marker. Apply
// the live FIFO drop policy (data dropped once the engine's FIFO occupancy
// reaches depth-8; markers use the 8-slot reserve) using this engine's own
// per-row interior counts for the EOR processing burst, and return the SURVIVING
// events (to be run through the back-end). This is fifo_dropsim's timing model
// restricted to one stripe.
std::vector<H::Event> keptForEngine(const std::vector<EvT>& evx, int lo, int hi, int W, int H) {
    // this engine's interior count per row
    std::vector<long> irow(H + 1, 0);
    for (const EvT& e : evx)
        if (e.kind == H::kEventInterior && int(e.x) >= lo && int(e.x) < hi && e.y < H)
            irow[e.y]++;
    auto inRange = [&](const EvT& e) {
        const bool data = (e.kind == H::kEventInterior || e.kind == H::kEventEndpoint);
        return !data || (int(e.x) >= lo && int(e.x) < hi);
    };
    auto cost = [&](const EvT& e) -> long {
        if (e.kind == H::kEventEndOfRow)
            return g_ing + ((e.y >= 1 && e.y - 1 < H) ? long(g_proc) * irow[e.y - 1] : 0);
        return g_ing;  // data event or EOF
    };
    const long afull = g_depth - 8;
    std::deque<int> fifo;
    long be_free = 0;
    std::vector<H::Event> kept;
    // indices of in-range events, in stream order
    std::vector<int> idx;
    for (int i = 0; i < int(evx.size()); ++i)
        if (inRange(evx[i])) idx.push_back(i);
    std::vector<char> drop(idx.size(), 0);
    for (int k = 0; k < int(idx.size()); ++k) {
        const EvT& e = evx[idx[k]];
        const long pc = e.pcyc;
        while (!fifo.empty()) {
            int jk = fifo.front();
            long pop_cyc = std::max(be_free, evx[idx[jk]].pcyc);
            if (pop_cyc > pc) break;
            be_free = pop_cyc + cost(evx[idx[jk]]);
            fifo.pop_front();
        }
        const bool data = (e.kind == H::kEventInterior || e.kind == H::kEventEndpoint);
        if (data && long(fifo.size()) >= afull) drop[k] = 1;
        else fifo.push_back(k);
    }
    for (int k = 0; k < int(idx.size()); ++k)
        if (!drop[k]) kept.push_back(H::Event{evx[idx[k]].kind, evx[idx[k]].x, evx[idx[k]].strong});
    return kept;
}

struct Result {
    std::string name;
    int w = 0, h = 0;
    long golden = 0, split = 0;
    long unaffected = 0;     // golden records reproduced exactly by the split
    long crossing = 0;       // golden segments straddling a boundary (at-risk)
    long lost = 0;           // at-risk golden segments with no surviving half
    long halves = 0;         // extra records the split adds (surviving halves)
    long max_stripe_events = 0, min_stripe_events = 0;  // load balance (data ev)
    long surv_single = 0;    // survived under single-engine FIFO drop (current HW)
    long surv_split = 0;     // survived under per-engine FIFO drop (stripes)
};

Result probe(const std::string& name, const GrayImage& src, const Params& p) {
    Result R;
    R.name = name;
    R.w = src.width;
    R.h = src.height;
    const int W = src.width, Hh = src.height;

    // front-end -> full event vector
    hls::stream<std::uint8_t> ss;
    for (int y = 0; y < Hh; ++y)
        for (int x = 0; x < W; ++x) ss.write(src.at(x, y));
    hls::stream<H::Event> ev;
    const H::HystCfg hyst{p.use_hysteresis, p.hysteresis_adaptive, p.hysteresis_low_th,
                          p.hysteresis_strong_min};
    H::sweeplsdFrontend(ss, ev, W, Hh, p.gradient_power_th, p.nms_strict_tiebreak, hyst, 3);
    std::vector<H::Event> evs;
    std::vector<EvT> evx;
    const long Tline = W + g_hblank;
    int cy = 0;
    while (!ev.empty()) {
        H::Event e = ev.read();
        evs.push_back(e);
        EvT x;
        x.kind = e.kind;
        x.x = e.x;
        x.strong = e.strong;
        if (e.kind == H::kEventEndOfRow) {
            x.y = cy;
            x.pcyc = long(cy) * Tline + W;
            ++cy;
        } else if (e.kind == H::kEventEndOfFrame) {
            x.y = cy;
            x.pcyc = long(cy) * Tline + W + 1;
        } else {
            x.y = cy;
            x.pcyc = long(cy) * Tline + e.x;
        }
        evx.push_back(x);
    }

    // stripe boundaries
    std::vector<int> bnd;   // g_stripes+1 cut points 0 .. W
    if (g_stripes == 2 && g_boundary > 0) {
        bnd = {0, g_boundary, W};
    } else {
        for (int i = 0; i <= g_stripes; ++i) bnd.push_back(int((long(W) * i) / g_stripes));
    }

    // golden (single engine over the whole width)
    std::vector<H::SegmentRecord> golden = runRange(evs, 0, W, W, Hh, p);
    R.golden = long(golden.size());

    // split engines
    std::unordered_map<std::string, int> splitkeys;
    R.min_stripe_events = -1;
    for (int s = 0; s < g_stripes; ++s) {
        int lo = bnd[s], hi = bnd[s + 1];
        long dev = 0;
        for (const H::Event& e : evs)
            if ((e.kind == H::kEventInterior || e.kind == H::kEventEndpoint) &&
                int(e.x) >= lo && int(e.x) < hi)
                ++dev;
        R.max_stripe_events = std::max(R.max_stripe_events, dev);
        R.min_stripe_events = R.min_stripe_events < 0 ? dev : std::min(R.min_stripe_events, dev);
        std::vector<H::SegmentRecord> part = runRange(evs, lo, hi, W, Hh, p);
        R.split += long(part.size());
        for (const H::SegmentRecord& r : part) splitkeys[key(r)]++;
    }

    // compare sets
    std::unordered_map<std::string, int> gk;
    for (const H::SegmentRecord& r : golden) gk[key(r)]++;
    for (auto& kv : gk) {
        auto it = splitkeys.find(kv.first);
        int common = it == splitkeys.end() ? 0 : std::min(kv.second, it->second);
        R.unaffected += common;
    }

    // at-risk golden segments: bbox straddles any internal boundary
    for (const H::SegmentRecord& r : golden) {
        bool cross = false;
        for (int i = 1; i < int(bnd.size()) - 1; ++i)
            if (int(r.min_x) < bnd[i] && int(r.max_x) >= bnd[i]) cross = true;
        if (cross) R.crossing++;
    }
    // golden segments the split failed to reproduce (== crossing that lost both
    // halves, in the common case) and the extra half-records the split created
    R.lost = R.golden - R.unaffected;         // golden not reproduced exactly
    R.halves = R.split - R.unaffected;         // split records that are not golden

    // ---- overflow: survived segments under the live FIFO drop policy ----
    // single engine over the whole width == what the board runs today
    R.surv_single = long(runEventsRec(keptForEngine(evx, 0, W, W, Hh), W, Hh, p).size());
    // per-stripe engines, each dropping on its own FIFO; sum the survivors
    R.surv_split = 0;
    for (int s = 0; s < g_stripes; ++s)
        R.surv_split += long(runEventsRec(keptForEngine(evx, bnd[s], bnd[s + 1], W, Hh), W, Hh, p).size());
    return R;
}

std::vector<std::string> readManifest(const std::string& path) {
    std::vector<std::string> out;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
        std::size_t b = line.find_first_not_of(" \t");
        if (b == std::string::npos || line[b] == '#') continue;
        out.push_back(line.substr(b));
    }
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> imgs;
    std::string csv_path;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--boundary" && i + 1 < argc) { g_boundary = std::atoi(argv[++i]); continue; }
        if (a == "--stripes" && i + 1 < argc) { g_stripes = std::atoi(argv[++i]); continue; }
        if (a == "--ing" && i + 1 < argc) { g_ing = std::atoi(argv[++i]); continue; }
        if (a == "--proc" && i + 1 < argc) { g_proc = std::atoi(argv[++i]); continue; }
        if (a == "--depth" && i + 1 < argc) { g_depth = std::atoi(argv[++i]); continue; }
        if (a == "--csv" && i + 1 < argc) { csv_path = argv[++i]; continue; }
        if (!a.empty() && a[0] == '@') {
            for (std::string& s : readManifest(a.substr(1))) imgs.push_back(s);
            continue;
        }
        imgs.push_back(a);
    }
    if (imgs.empty()) {
        std::fprintf(stderr, "usage: stripe_probe <img...|@manifest> [--stripes N] "
                             "[--boundary M] [--csv out.csv]\n");
        return 2;
    }

    const Params p = Params::improved();
    std::FILE* csv = csv_path.empty() ? nullptr : std::fopen(csv_path.c_str(), "w");
    if (csv)
        std::fprintf(csv, "name,w,h,golden,split,unaffected,crossing,lost,halves,"
                          "max_stripe_ev,min_stripe_ev,balance,surv_single,surv_split\n");

    std::printf("# vertical-stripe probe (stripes=%d, front-end=improved, drop ing=%d proc=%d "
                "depth=%d)\n", g_stripes, g_ing, g_proc, g_depth);
    std::printf("%-16s %7s %8s %6s  %8s %8s %8s\n", "name", "golden", "crossing", "bal%",
                "survHW", "survStrp", "ideal");

    long t_gold = 0, t_split = 0, t_unaff = 0, t_cross = 0, t_lost = 0, t_half = 0;
    long t_maxev = 0, t_minev = 0, t_s1 = 0, t_s2 = 0;
    int n = 0;
    for (const std::string& path : imgs) {
        GrayImage img = loadGray(path);
        if (img.width == 0) { std::fprintf(stderr, "SKIP (load): %s\n", path.c_str()); continue; }
        if (img.width > H::kMaxWidth) { std::fprintf(stderr, "SKIP (width): %s\n", path.c_str()); continue; }
        std::string base = path;
        std::size_t s = base.find_last_of("/\\");
        if (s != std::string::npos) base = base.substr(s + 1);

        Result R = probe(base, img, p);
        double bal = R.max_stripe_events ? 100.0 * double(R.min_stripe_events) / double(R.max_stripe_events) : 0.0;
        std::printf("%-16s %7ld %8ld %5.1f  %8ld %8ld %8ld\n", R.name.c_str(),
                    R.golden, R.crossing, bal, R.surv_single, R.surv_split, R.golden);
        std::fflush(stdout);
        if (csv)
            std::fprintf(csv, "%s,%d,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%.1f,%ld,%ld\n", R.name.c_str(),
                         R.w, R.h, R.golden, R.split, R.unaffected, R.crossing, R.lost,
                         R.halves, R.max_stripe_events, R.min_stripe_events, bal,
                         R.surv_single, R.surv_split);
        t_gold += R.golden; t_split += R.split; t_unaff += R.unaffected;
        t_cross += R.crossing; t_lost += R.lost; t_half += R.halves;
        t_maxev += R.max_stripe_events; t_minev += R.min_stripe_events;
        t_s1 += R.surv_single; t_s2 += R.surv_split;
        ++n;
    }
    if (csv) std::fclose(csv);
    if (!n) return 1;

    std::fprintf(stderr,
                 "\n==== corpus (%d images, %d stripes) ====\n"
                 "golden segments        : %ld\n"
                 "split segments         : %ld  (net %+ld, %.2f%%)\n"
                 "reproduced exactly      : %ld  (%.2f%% of golden)\n"
                 "boundary-crossing (risk): %ld  (%.2f%% of golden)\n"
                 "golden not reproduced   : %ld  (%.2f%%)\n"
                 "extra half-records      : %ld\n"
                 "mean stripe balance     : %.1f%% (min/max data events)\n"
                 "---- overflow (drop ing=%d proc=%d depth=%d) ----\n"
                 "ideal (no drop)         : %ld\n"
                 "survived single-engine  : %ld  (%.1f%% of ideal)  [current HW]\n"
                 "survived %d-stripe       : %ld  (%.1f%% of ideal)  [+%ld vs HW]\n",
                 n, g_stripes, t_gold, t_split, t_split - t_gold,
                 100.0 * double(t_split - t_gold) / double(t_gold ? t_gold : 1),
                 t_unaff, 100.0 * double(t_unaff) / double(t_gold ? t_gold : 1),
                 t_cross, 100.0 * double(t_cross) / double(t_gold ? t_gold : 1),
                 t_lost, 100.0 * double(t_lost) / double(t_gold ? t_gold : 1),
                 t_half, 100.0 * double(t_minev) / double(t_maxev ? t_maxev : 1),
                 g_ing, g_proc, g_depth, t_gold,
                 t_s1, 100.0 * double(t_s1) / double(t_gold ? t_gold : 1),
                 g_stripes, t_s2, 100.0 * double(t_s2) / double(t_gold ? t_gold : 1),
                 t_s2 - t_s1);
    return 0;
}
