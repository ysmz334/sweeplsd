// Step 0 of the FIFO-drop study (see rtl/DESIGN.md "elastic buffer"): measure,
// per image, the DENSITY the labelling back-end has to swallow. It runs only
// the front-end (sweeplsdFrontend, bit-exact with the board's front-end) and
// tabulates the sparse event stream by row — no timing/blanking model yet, no
// drop estimate. That comes in Step 1; here we just look at how bursty the
// event stream is, to decide whether a drop model is even worth building.
//
// Front-end config = Params::improved() = exactly what boards/atlys/live_core.v
// drives on the FPGA (strict NMS + adaptive hysteresis low=120 + border 3 +
// mps 1). The event count depends on the front-end only; the back-end gates
// (strong_min, border, mps) do not change which events are produced.
//
//   density_probe <img...>            # per-image CSV to stdout + summary
//   density_probe @manifest.txt       # read image paths from a file (one/line)
//
// The two "budget" columns use the 1080p30 live-video geometry (the thesis
// 1920x1080 format on the board): one active line = width + 280 blanking
// clocks, and the back-end spends ~Cavg=17 clocks per data event (2 ingest +
// ~15 process; see backend.v). rows_over = rows whose data-event count alone
// exceeds one line's clock budget (width+280)/17 — a purely indicative "this
// row cannot be kept up with in real time" flag, NOT a drop count (the FIFO
// absorbs isolated bursts; sustained ones drop — that is Step 1's job).

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
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

// 1080p30 live geometry (thesis 1920x1080 on the board). Overridable so Step 1
// can reuse the same numbers for other modes.
int g_hblank = 280;   // horizontal blanking clocks per line
int g_cavg = 17;      // back-end clocks per data event (2 ingest + ~15 process)
int g_edge_border = 3;  // outer px zeroed at the edge stage (border-ring fix); 0 = pre-fix

struct RowStat {
    long interior = 0;
    long endpoint = 0;
    long data() const { return interior + endpoint; }
};

struct ImgStat {
    std::string name;
    int w = 0, h = 0;
    long interior = 0, endpoint = 0;
    long max_data_row = 0, max_int_row = 0;
    long p50 = 0, p90 = 0, p99 = 0;
    double mean = 0.0;
    long rows_over = 0;      // rows over the single-line budget (indicative)
    long budget_line = 0;    // (w + hblank) / cavg
    bool ok = true;
};

ImgStat probe(const std::string& name, const GrayImage& src) {
    ImgStat st;
    st.name = name;
    st.w = src.width;
    st.h = src.height;

    const Params p = Params::improved();
    hls::stream<std::uint8_t> src_s;
    for (int y = 0; y < src.height; ++y)
        for (int x = 0; x < src.width; ++x) src_s.write(src.at(x, y));

    hls::stream<H::Event> ev;
    const H::HystCfg hyst{p.use_hysteresis, p.hysteresis_adaptive, p.hysteresis_low_th,
                          p.hysteresis_strong_min};
    H::sweeplsdFrontend(src_s, ev, src.width, src.height, p.gradient_power_th,
                        p.nms_strict_tiebreak, hyst, g_edge_border);

    std::vector<RowStat> rows(src.height);
    int y = 0;
    bool eof = false;
    while (!ev.empty()) {
        H::Event e = ev.read();
        if (e.kind == H::kEventEndOfFrame) { eof = true; break; }
        if (e.kind == H::kEventEndOfRow) { ++y; continue; }
        if (y >= src.height) { st.ok = false; break; }
        if (e.kind == H::kEventInterior) rows[y].interior++;
        else if (e.kind == H::kEventEndpoint) rows[y].endpoint++;
    }
    if (!eof || y != src.height) st.ok = false;

    std::vector<long> per_row;
    per_row.reserve(rows.size());
    for (const RowStat& r : rows) {
        st.interior += r.interior;
        st.endpoint += r.endpoint;
        st.max_data_row = std::max(st.max_data_row, r.data());
        st.max_int_row = std::max(st.max_int_row, r.interior);
        per_row.push_back(r.data());
    }
    st.budget_line = (st.w + g_hblank) / g_cavg;
    for (long d : per_row)
        if (d > st.budget_line) st.rows_over++;

    if (!per_row.empty()) {
        std::vector<long> s = per_row;
        std::sort(s.begin(), s.end());
        auto pct = [&](double q) {
            std::size_t i = std::size_t(q * (s.size() - 1) + 0.5);
            return s[i];
        };
        st.p50 = pct(0.50);
        st.p90 = pct(0.90);
        st.p99 = pct(0.99);
        long tot = st.interior + st.endpoint;
        st.mean = double(tot) / double(per_row.size());
    }
    return st;
}

std::vector<std::string> readManifest(const std::string& path) {
    std::vector<std::string> out;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        // trim
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
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--hblank" && i + 1 < argc) { g_hblank = std::atoi(argv[++i]); continue; }
        if (a == "--cavg" && i + 1 < argc) { g_cavg = std::atoi(argv[++i]); continue; }
        if (a == "--edge-border" && i + 1 < argc) { g_edge_border = std::atoi(argv[++i]); continue; }
        if (!a.empty() && a[0] == '@') {
            for (std::string& s : readManifest(a.substr(1))) imgs.push_back(s);
            continue;
        }
        imgs.push_back(a);
    }
    if (imgs.empty()) {
        std::fprintf(stderr, "usage: density_probe [--hblank N] [--cavg N] [--edge-border N] "
                             "<img...|@manifest>\n");
        return 2;
    }

    std::printf("# front-end = Params::improved() (== live_core.v); geometry 1080p30 "
                "hblank=%d cavg=%d edge_border=%d  budget_line=(w+hblank)/cavg\n",
                g_hblank, g_cavg, g_edge_border);
    std::printf("name,w,h,interior,endpoint,data,dens_pct,mean_row,p50,p90,p99,max_row,"
                "max_int_row,budget_line,rows_over\n");

    std::vector<ImgStat> all;
    for (const std::string& path : imgs) {
        GrayImage img = loadGray(path);
        if (img.width == 0) {
            std::fprintf(stderr, "SKIP (cannot load): %s\n", path.c_str());
            continue;
        }
        if (img.width > H::kMaxWidth) {
            std::fprintf(stderr, "SKIP (width %d > kMaxWidth %d): %s\n", img.width,
                         H::kMaxWidth, path.c_str());
            continue;
        }
        // basename for readability
        std::string base = path;
        std::size_t slash = base.find_last_of("/\\");
        if (slash != std::string::npos) base = base.substr(slash + 1);

        ImgStat st = probe(base, img);
        long data = st.interior + st.endpoint;
        double dens = 100.0 * double(data) / (double(st.w) * double(st.h));
        std::printf("%s,%d,%d,%ld,%ld,%ld,%.3f,%.1f,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
                    st.name.c_str(), st.w, st.h, st.interior, st.endpoint, data, dens,
                    st.mean, st.p50, st.p90, st.p99, st.max_data_row, st.max_int_row,
                    st.budget_line, st.rows_over);
        std::fflush(stdout);
        if (!st.ok) std::fprintf(stderr, "WARN framing broken: %s\n", st.name.c_str());
        all.push_back(st);
    }

    if (all.empty()) return 1;

    // ---- corpus summary (to stderr so stdout stays clean CSV) ----
    auto agg = [&](auto getter) {
        std::vector<double> v;
        for (const ImgStat& s : all) v.push_back(double(getter(s)));
        std::sort(v.begin(), v.end());
        struct { double mn, med, p90, mx; } r{
            v.front(), v[v.size() / 2], v[std::size_t(0.9 * (v.size() - 1) + 0.5)], v.back()};
        return r;
    };
    auto md = agg([](const ImgStat& s) { return s.max_data_row; });
    auto mi = agg([](const ImgStat& s) { return s.max_int_row; });
    auto ro = agg([](const ImgStat& s) { return s.rows_over; });
    auto de = agg([](const ImgStat& s) {
        return 100.0 * double(s.interior + s.endpoint) / (double(s.w) * double(s.h));
    });
    long imgs_with_over = 0;
    for (const ImgStat& s : all)
        if (s.rows_over > 0) imgs_with_over++;

    std::fprintf(stderr,
                 "\n==== corpus summary over %zu images (budget_line ~%ld data-ev/row) ====\n"
                 "                     min      median     p90       max\n"
                 "density %%            %7.3f  %7.3f  %7.3f  %7.3f\n"
                 "max data / row     %7.0f  %7.0f  %7.0f  %7.0f\n"
                 "max interior / row %7.0f  %7.0f  %7.0f  %7.0f\n"
                 "rows over budget   %7.0f  %7.0f  %7.0f  %7.0f\n"
                 "images with any over-budget row: %ld / %zu\n",
                 all.size(), all.empty() ? 0 : all.front().budget_line,
                 de.mn, de.med, de.p90, de.mx, md.mn, md.med, md.p90, md.mx,
                 mi.mn, mi.med, mi.p90, mi.mx, ro.mn, ro.med, ro.p90, ro.mx,
                 imgs_with_over, all.size());

    // worst 8 images by max data/row
    std::vector<const ImgStat*> bymax;
    for (const ImgStat& s : all) bymax.push_back(&s);
    std::sort(bymax.begin(), bymax.end(),
              [](const ImgStat* a, const ImgStat* b) { return a->max_data_row > b->max_data_row; });
    std::fprintf(stderr, "\nworst images by peak data/row:\n");
    for (int i = 0; i < 8 && i < int(bymax.size()); ++i)
        std::fprintf(stderr, "  %-16s max_row=%ld  rows_over=%ld  dens=%.2f%%\n",
                     bymax[i]->name.c_str(), bymax[i]->max_data_row, bymax[i]->rows_over,
                     100.0 * double(bymax[i]->interior + bymax[i]->endpoint) /
                         (double(bymax[i]->w) * double(bymax[i]->h)));
    return 0;
}
