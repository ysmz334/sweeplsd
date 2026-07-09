// Judge datapath sizing probe: run the bit-exact HLS front-end + back-end over
// a corpus and report the LARGEST moment inputs (n, Sx, Sy, Sxx, Syy, Sxy) and
// derived products (ma, mb, mc, T, T^2, R^2, 361*T^2, 441*R^2) the judge
// actually sees. The current judge datapath is sized 128-bit for the worst-case
// n=2^18 cap; this measures the EMPIRICAL worst case so the datapath can be
// narrowed (rounding is acceptable — see the judge-simplification work).
//
//   moment_probe <img...|@manifest>
//
// Front-end config = Params::improved() (== the board). g_mmax accumulates
// across all images (corpus-wide maxima).

#include <cstdint>
#include <cstdio>
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

int bits_u64(std::uint64_t v) { int b = 0; while (v) { ++b; v >>= 1; } return b; }
int bits_u128(unsigned __int128 v) { int b = 0; while (v) { ++b; v >>= 1; } return b; }

void dump128(const char* name, unsigned __int128 v) {
    // print as decimal is awkward for 128-bit; report bit width + hi/lo 64
    std::uint64_t hi = std::uint64_t(v >> 64), lo = std::uint64_t(v);
    std::printf("  %-8s bits=%3d   (hi=%llu lo=%llu)\n", name, bits_u128(v),
                (unsigned long long)hi, (unsigned long long)lo);
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
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (!a.empty() && a[0] == '@') {
            for (std::string& s : readManifest(a.substr(1))) imgs.push_back(s);
        } else {
            imgs.push_back(a);
        }
    }
    if (imgs.empty()) {
        std::fprintf(stderr, "usage: moment_probe <img...|@manifest>\n");
        return 2;
    }

    const Params p = Params::improved();
    const H::HystCfg hyst{p.use_hysteresis, p.hysteresis_adaptive, p.hysteresis_low_th,
                          p.hysteresis_strong_min};
    const int mps_2sq = int(2.0 * p.max_perp_spread * p.max_perp_spread + 0.5);

    H::momentMaxReset();
    int n_img = 0;
    for (const std::string& path : imgs) {
        GrayImage img = loadGray(path);
        if (img.width == 0) { std::fprintf(stderr, "SKIP (load): %s\n", path.c_str()); continue; }
        if (img.width > H::kMaxWidth) { std::fprintf(stderr, "SKIP (width): %s\n", path.c_str()); continue; }
        hls::stream<std::uint8_t> ss;
        for (int y = 0; y < img.height; ++y)
            for (int x = 0; x < img.width; ++x) ss.write(img.at(x, y));
        hls::stream<H::Event> ev;
        H::sweeplsdFrontend(ss, ev, img.width, img.height, p.gradient_power_th,
                            p.nms_strict_tiebreak, hyst, 3);
        hls::stream<H::SegmentRecord> rec;
        H::sweeplsdBackend(ev, rec, img.width, img.height, p.pixel_num_th, hyst,
                           p.border_margin, mps_2sq);
        while (!rec.empty()) rec.read();
        ++n_img;
    }

    H::MomentMax m = H::momentMax();
    std::printf("=== judge moment maxima over %d images (%ld judge calls, %ld reached arith) ===\n",
                n_img, m.calls, m.arith);
    std::printf("inputs (u64):\n");
    std::printf("  n        max=%-12llu bits=%d\n", (unsigned long long)m.n, bits_u64(m.n));
    std::printf("  Sx       max=%-12llu bits=%d\n", (unsigned long long)m.xs, bits_u64(m.xs));
    std::printf("  Sy       max=%-12llu bits=%d\n", (unsigned long long)m.ys, bits_u64(m.ys));
    std::printf("  Sxx      max=%-12llu bits=%d\n", (unsigned long long)m.xss, bits_u64(m.xss));
    std::printf("  Syy      max=%-12llu bits=%d\n", (unsigned long long)m.yss, bits_u64(m.yss));
    std::printf("  Sxy      max=%-12llu bits=%d\n", (unsigned long long)m.xys, bits_u64(m.xys));
    std::printf("derived (magnitude):\n");
    dump128("ma", m.ma);
    dump128("mb", m.mb);
    dump128("mc", m.mc);
    dump128("T", m.T);
    dump128("T^2", m.T2);
    dump128("R^2", m.R2);
    dump128("361*T^2", m.lhs);
    dump128("441*R^2", m.rhs);
    return 0;
}
