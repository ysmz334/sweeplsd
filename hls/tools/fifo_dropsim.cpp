// Step 1 of the FIFO-drop study: a row-granular producer -> FIFO -> back-end
// co-simulation that reports, per image, HOW MANY events the live path drops
// (interior vs endpoint, and in which rows) and HOW MANY segments are lost as a
// result (full stream vs dropped stream, both run through the real HLS
// back-end).
//
// This is NOT the RTL testbench: it drives the bit-exact HLS C model (same
// front-end + same sweeplsdBackend the RTL is verified against) and overlays a
// timing model of the elastic event FIFO (rtl/core/event_fifo.v, drop_mode=1):
//
//   producer  : the front-end is II=1, so the event at pixel (x,y) is produced
//               at cycle y*(W+hblank)+x; the end-of-row marker at the end of the
//               active line, then hblank idle clocks, then the next row. This is
//               the 1080p30 live geometry (thesis 1920x1080 on the board).
//   consumer  : the back-end FSM pops a data event in ~ING clocks (ingest) but,
//               on popping an end-of-row marker, stalls for the PROCESS burst of
//               the previous row (~PROC clocks per interior pixel + scavenge)
//               during which it pops nothing and the FIFO fills — the lumpy
//               drain that makes dense rows overflow (backend.v, backend.cpp).
//   FIFO      : depth 2048, data events dropped once occupancy >= 2040 (the
//               8-slot marker reserve); EOR/EOF markers are never dropped.
//
// The cost constants (ING, PROC, SCAV) are first-order; Step 2 calibrates them
// against a few RTL-testbench runs. Front-end + back-end config = the improved
// mode live_core.v drives (strict, adaptive hysteresis low=120, border 3,
// mps 1). Usage:
//
//   fifo_dropsim <img...|@manifest> [--hblank N] [--ing N] [--proc N]
//                [--scav N] [--depth N] [--csv out.csv]

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <string>
#include <vector>

#include "backend.hpp"
#include "frontend.hpp"
#include "sweeplsd/io.hpp"
#include "sweeplsd/sweeplsd.hpp"

using namespace sweeplsd;
namespace H = sweeplsd_hls;

namespace {

int g_hblank = 280;      // 1080p30 horizontal blanking clocks / line
int g_ing = 2;           // clocks to pop+ingest one data event
int g_proc = 15;         // clocks to process one interior pixel (per-pixel burst)
int g_scav = 0;          // extra clocks per processed row (scavenger); 0 = fold into proc
int g_depth = 2048;      // FIFO depth; data dropped at occupancy >= depth-8

struct Ev {
    std::uint8_t kind;
    std::uint16_t x;
    std::uint8_t strong;
    int y;        // row (from EOR count); for a marker, the row it closes/ends
    long pcyc;    // produce cycle
};

// Count accepted segments (records with n != 0 before the terminator).
long runBackend(const std::vector<Ev>& evs, const std::vector<char>& drop, int w, int h,
                const Params& p) {
    hls::stream<H::Event> es;
    for (std::size_t i = 0; i < evs.size(); ++i) {
        if (!drop.empty() && drop[i]) continue;  // thinned stream skips dropped data events
        es.write(H::Event{evs[i].kind, evs[i].x, evs[i].strong});
    }
    hls::stream<H::SegmentRecord> rec;
    const H::HystCfg hyst{p.use_hysteresis, p.hysteresis_adaptive, p.hysteresis_low_th,
                          p.hysteresis_strong_min};
    const int mps_2sq = int(2.0 * p.max_perp_spread * p.max_perp_spread + 0.5);
    H::sweeplsdBackend(es, rec, w, h, p.pixel_num_th, hyst, p.border_margin, mps_2sq);
    long n = 0;
    while (!rec.empty()) {
        H::SegmentRecord r = rec.read();
        if (r.n == 0) break;
        ++n;
    }
    return n;
}

struct Result {
    std::string name;
    int w = 0, h = 0;
    long data = 0, interior = 0, endpoint = 0;
    long drop_int = 0, drop_end = 0;
    long peak_occ = 0;
    int first_drop_row = -1, last_drop_row = -1, rows_with_drop = 0, max_row_drop = 0;
    long seg_full = 0, seg_drop = 0;
};

Result simulate(const std::string& name, const GrayImage& src, const Params& p) {
    Result R;
    R.name = name;
    R.w = src.width;
    R.h = src.height;
    const int W = src.width, Hh = src.height;
    const long Tline = W + g_hblank;

    // ---- front-end -> full event vector, with row + produce cycle ----
    hls::stream<std::uint8_t> src_s;
    for (int y = 0; y < Hh; ++y)
        for (int x = 0; x < W; ++x) src_s.write(src.at(x, y));
    hls::stream<H::Event> ev;
    const H::HystCfg hyst{p.use_hysteresis, p.hysteresis_adaptive, p.hysteresis_low_th,
                          p.hysteresis_strong_min};
    H::sweeplsdFrontend(src_s, ev, W, Hh, p.gradient_power_th, p.nms_strict_tiebreak, hyst);

    std::vector<Ev> evs;
    std::vector<long> interior_row(Hh, 0);
    int y = 0;
    while (!ev.empty()) {
        H::Event e = ev.read();
        Ev x;
        x.kind = e.kind;
        x.x = e.x;
        x.strong = e.strong;
        if (e.kind == H::kEventEndOfRow) {
            x.y = y;                       // marker ending row y
            x.pcyc = long(y) * Tline + W;  // end of the active line
            ++y;
        } else if (e.kind == H::kEventEndOfFrame) {
            x.y = y;
            x.pcyc = long(y) * Tline + W + 1;
        } else {
            x.y = y;
            x.pcyc = long(y) * Tline + e.x;
            if (e.kind == H::kEventInterior) {
                if (y < Hh) interior_row[y]++;
                R.interior++;
            } else {
                R.endpoint++;
            }
        }
        evs.push_back(x);
    }
    R.data = R.interior + R.endpoint;

    // ---- FIFO co-sim (event_fifo.v drop_mode=1) ----
    const long afull = g_depth - 8;   // data dropped at occupancy >= afull
    auto cost = [&](const Ev& e) -> long {
        if (e.kind == H::kEventEndOfRow) {
            // popping this marker triggers processRow(e.y - 1) in the back-end
            long burst = (e.y >= 1 && e.y - 1 < Hh) ? long(g_proc) * interior_row[e.y - 1] + g_scav : 0;
            return g_ing + burst;
        }
        if (e.kind == H::kEventEndOfFrame) return g_ing;
        return g_ing;  // data event
    };

    std::deque<int> fifo;
    long be_free = 0;                 // cycle the back-end can pop next
    std::vector<char> drop(evs.size(), 0);
    std::vector<int> row_drop(Hh, 0);
    for (std::size_t i = 0; i < evs.size(); ++i) {
        const long pc = evs[i].pcyc;
        // drain: pop everything the back-end finishes reading by cycle pc
        while (!fifo.empty()) {
            int j = fifo.front();
            long pop_cyc = std::max(be_free, evs[j].pcyc);
            if (pop_cyc > pc) break;
            be_free = pop_cyc + cost(evs[j]);
            fifo.pop_front();
        }
        const bool is_data =
            evs[i].kind == H::kEventInterior || evs[i].kind == H::kEventEndpoint;
        if (is_data && long(fifo.size()) >= afull) {
            drop[i] = 1;
            if (evs[i].kind == H::kEventInterior) R.drop_int++;
            else R.drop_end++;
            if (evs[i].y < Hh) row_drop[evs[i].y]++;
        } else {
            fifo.push_back(int(i));
        }
        R.peak_occ = std::max(R.peak_occ, long(fifo.size()));
    }

    for (int r = 0; r < Hh; ++r)
        if (row_drop[r] > 0) {
            if (R.first_drop_row < 0) R.first_drop_row = r;
            R.last_drop_row = r;
            R.rows_with_drop++;
            R.max_row_drop = std::max(R.max_row_drop, row_drop[r]);
        }

    // ---- segments: full stream vs dropped stream, through the real back-end ----
    R.seg_full = runBackend(evs, {}, W, Hh, p);
    R.seg_drop = runBackend(evs, drop, W, Hh, p);
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
        auto next = [&]() { return std::atoi(argv[++i]); };
        if (a == "--hblank") { g_hblank = next(); continue; }
        if (a == "--ing") { g_ing = next(); continue; }
        if (a == "--proc") { g_proc = next(); continue; }
        if (a == "--scav") { g_scav = next(); continue; }
        if (a == "--depth") { g_depth = next(); continue; }
        if (a == "--csv" && i + 1 < argc) { csv_path = argv[++i]; continue; }
        if (!a.empty() && a[0] == '@') {
            for (std::string& s : readManifest(a.substr(1))) imgs.push_back(s);
            continue;
        }
        imgs.push_back(a);
    }
    if (imgs.empty()) {
        std::fprintf(stderr, "usage: fifo_dropsim <img...|@manifest> [--hblank N --ing N "
                             "--proc N --scav N --depth N --csv out.csv]\n");
        return 2;
    }

    const Params p = Params::improved();
    std::FILE* csv = csv_path.empty() ? nullptr : std::fopen(csv_path.c_str(), "w");
    if (csv)
        std::fprintf(csv, "name,w,h,data,interior,endpoint,drop_int,drop_end,drop_pct,"
                          "peak_occ,rows_with_drop,first_drop_row,last_drop_row,max_row_drop,"
                          "seg_full,seg_drop,seg_lost,seg_lost_pct\n");

    std::printf("# FIFO drop co-sim (1080p30, hblank=%d, ING=%d PROC=%d SCAV=%d depth=%d)\n",
                g_hblank, g_ing, g_proc, g_scav, g_depth);
    std::printf("%-16s %7s %6s %6s %7s %6s  %8s %8s %8s  %5s\n", "name", "data", "dInt",
                "dEnd", "drop%", "peak", "segFull", "segDrop", "lost", "lost%");

    std::vector<Result> all;
    for (const std::string& path : imgs) {
        GrayImage img = loadGray(path);
        if (img.width == 0) { std::fprintf(stderr, "SKIP (load): %s\n", path.c_str()); continue; }
        if (img.width > H::kMaxWidth) {
            std::fprintf(stderr, "SKIP (width): %s\n", path.c_str());
            continue;
        }
        std::string base = path;
        std::size_t s = base.find_last_of("/\\");
        if (s != std::string::npos) base = base.substr(s + 1);

        Result R = simulate(base, img, p);
        long drop = R.drop_int + R.drop_end;
        double dpct = R.data ? 100.0 * double(drop) / double(R.data) : 0.0;
        long lost = R.seg_full - R.seg_drop;
        double lpct = R.seg_full ? 100.0 * double(lost) / double(R.seg_full) : 0.0;
        std::printf("%-16s %7ld %6ld %6ld %6.2f%% %6ld  %8ld %8ld %8ld %5.1f%%\n", R.name.c_str(),
                    R.data, R.drop_int, R.drop_end, dpct, R.peak_occ, R.seg_full, R.seg_drop,
                    lost, lpct);
        std::fflush(stdout);
        if (csv)
            std::fprintf(csv, "%s,%d,%d,%ld,%ld,%ld,%ld,%ld,%.3f,%ld,%d,%d,%d,%d,%ld,%ld,%ld,%.3f\n",
                         R.name.c_str(), R.w, R.h, R.data, R.interior, R.endpoint, R.drop_int,
                         R.drop_end, dpct, R.peak_occ, R.rows_with_drop, R.first_drop_row,
                         R.last_drop_row, R.max_row_drop, R.seg_full, R.seg_drop, lost, lpct);
        all.push_back(R);
    }
    if (csv) std::fclose(csv);
    if (all.empty()) return 1;

    // ---- corpus summary ----
    long tot_full = 0, tot_drop = 0, tot_lost = 0, tot_data = 0, tot_dev = 0;
    int imgs_dropping = 0, imgs_losing = 0;
    for (const Result& R : all) {
        tot_full += R.seg_full;
        tot_drop += R.seg_drop;
        tot_lost += R.seg_full - R.seg_drop;
        tot_data += R.data;
        tot_dev += R.drop_int + R.drop_end;
        if (R.drop_int + R.drop_end > 0) imgs_dropping++;
        if (R.seg_full - R.seg_drop > 0) imgs_losing++;
    }
    std::vector<const Result*> byloss;
    for (const Result& R : all) byloss.push_back(&R);
    std::sort(byloss.begin(), byloss.end(), [](const Result* a, const Result* b) {
        return (a->seg_full - a->seg_drop) > (b->seg_full - b->seg_drop);
    });

    std::fprintf(stderr,
                 "\n==== corpus (%zu images) ====\n"
                 "images that drop any event : %d / %zu\n"
                 "images that lose any segment: %d / %zu\n"
                 "total events %ld, dropped %ld (%.2f%%)\n"
                 "total segments full %ld, after drop %ld, lost %ld (%.2f%%)\n",
                 all.size(), imgs_dropping, all.size(), imgs_losing, all.size(), tot_data, tot_dev,
                 100.0 * double(tot_dev) / double(tot_data ? tot_data : 1), tot_full, tot_drop,
                 tot_lost, 100.0 * double(tot_lost) / double(tot_full ? tot_full : 1));
    std::fprintf(stderr, "\nworst 12 by segments lost:\n");
    for (int i = 0; i < 12 && i < int(byloss.size()); ++i) {
        const Result* R = byloss[i];
        long lost = R->seg_full - R->seg_drop;
        std::fprintf(stderr, "  %-16s lost=%ld / %ld (%.1f%%)  droppedEv=%ld (int %ld/end %ld)  "
                             "rows[%d..%d] peak_occ=%ld\n",
                     R->name.c_str(), lost, R->seg_full,
                     R->seg_full ? 100.0 * double(lost) / double(R->seg_full) : 0.0,
                     R->drop_int + R->drop_end, R->drop_int, R->drop_end, R->first_drop_row,
                     R->last_drop_row, R->peak_occ);
    }
    return 0;
}
