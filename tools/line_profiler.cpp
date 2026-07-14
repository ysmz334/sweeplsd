// line_profiler — a tiny self-contained sampling line profiler for MinGW/-g
// Windows binaries. No cv2pdb, no Visual Studio: it launches the target,
// samples the primary thread's instruction pointer at ~1 kHz, and resolves each
// sample to a source file:line and function via `addr2line`, which reads the
// binary's DWARF debug info directly. It prints the hottest source lines, a
// per-file rollup, and the hottest functions.
//
// This exists because cv2pdb-converted PDBs sometimes fail to display .cpp
// sources in Visual Studio's profiler (VS wants a matching source checksum on
// the primary compiland source, which cv2pdb does not write). Reading the DWARF
// directly with addr2line side-steps that entirely.
//
//   line_profiler <target.exe built with -g> [target args...]
//
// Requirements: the TARGET must be compiled with -g and still contain DWARF
// (i.e. NOT run through cv2pdb, which strips it); `addr2line` must be on PATH
// (it ships with MinGW). ASLR is handled via the PE ImageBase slide, so no
// special link flags are needed. See tools/build_profile.bat and
// docs/profiling.md.

#include <windows.h>
#include <psapi.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>

static std::uint64_t peImageBase(const char* path) {
    FILE* f = std::fopen(path, "rb");
    if (!f) return 0;
    unsigned char dos[64];
    if (std::fread(dos, 1, 64, f) != 64) { std::fclose(f); return 0; }
    std::uint32_t e_lfanew = *reinterpret_cast<std::uint32_t*>(dos + 60);
    std::fseek(f, e_lfanew, SEEK_SET);
    unsigned char nt[264];
    size_t n = std::fread(nt, 1, sizeof(nt), f);
    std::fclose(f);
    if (n < 32) return 0;
    // 'PE\0\0'(4) + FileHeader(20) => OptionalHeader at offset 24; magic 0x20b
    // is PE32+, and ImageBase is the 8 bytes at optional-header offset 24.
    if (*reinterpret_cast<std::uint16_t*>(nt + 24) != 0x20b) return 0;
    return *reinterpret_cast<std::uint64_t*>(nt + 24 + 24);
}

static std::string basename(const std::string& p) {
    size_t s = p.find_last_of("/\\");
    return s == std::string::npos ? p : p.substr(s + 1);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: %s <target.exe (built with -g, DWARF intact)> [args...]\n", argv[0]);
        return 1;
    }
    std::string exe = argv[1];
    std::string cmd;
    for (int i = 1; i < argc; ++i) { cmd += '"'; cmd += argv[i]; cmd += '"'; if (i + 1 < argc) cmd += ' '; }

    STARTUPINFOA si{}; si.cb = sizeof(si);
    PROCESS_INFORMATION pi{};
    std::vector<char> cmdbuf(cmd.begin(), cmd.end()); cmdbuf.push_back(0);
    if (!CreateProcessA(exe.c_str(), cmdbuf.data(), nullptr, nullptr, FALSE, 0, nullptr, nullptr,
                        &si, &pi)) {
        std::printf("CreateProcess failed (%lu)\n", GetLastError());
        return 1;
    }

    const std::uint64_t imageBase = peImageBase(exe.c_str());
    std::uint64_t runtimeBase = 0, moduleSize = 0, total = 0, inMod = 0;
    std::unordered_map<std::uint64_t, std::uint64_t> hits;  // static addr -> count

    timeBeginPeriod(1);
    for (;;) {
        if (WaitForSingleObject(pi.hProcess, 0) == WAIT_OBJECT_0) break;
        if (!runtimeBase) {
            HMODULE mods[8]; DWORD need = 0;
            if (EnumProcessModules(pi.hProcess, mods, sizeof(mods), &need) && need >= sizeof(HMODULE)) {
                MODULEINFO mi{};
                if (GetModuleInformation(pi.hProcess, mods[0], &mi, sizeof(mi))) {
                    runtimeBase = reinterpret_cast<std::uint64_t>(mi.lpBaseOfDll);
                    moduleSize = mi.SizeOfImage;
                }
            }
        }
        if (SuspendThread(pi.hThread) != (DWORD)-1) {
            CONTEXT ctx{}; ctx.ContextFlags = CONTEXT_CONTROL;
            if (GetThreadContext(pi.hThread, &ctx)) {
                ++total;
                if (runtimeBase && ctx.Rip >= runtimeBase && ctx.Rip < runtimeBase + moduleSize) {
                    ++hits[ctx.Rip - runtimeBase + imageBase];  // undo ASLR slide
                    ++inMod;
                }
            }
            ResumeThread(pi.hThread);
        }
        Sleep(1);
    }
    timeEndPeriod(1);
    CloseHandle(pi.hThread); CloseHandle(pi.hProcess);

    if (hits.empty()) {
        std::printf("no in-module samples (total=%llu). Was the target built with -g?\n",
                    (unsigned long long)total);
        return 1;
    }

    std::vector<std::uint64_t> addrs; addrs.reserve(hits.size());
    for (auto& kv : hits) addrs.push_back(kv.first);
    std::string tmp = std::string(std::getenv("TEMP") ? std::getenv("TEMP") : ".") + "\\lineprof_addrs.txt";
    if (FILE* af = std::fopen(tmp.c_str(), "w")) {
        for (std::uint64_t a : addrs) std::fprintf(af, "0x%llx\n", (unsigned long long)a);
        std::fclose(af);
    }
    std::string a2l = "addr2line -C -f -e \"" + exe + "\" < \"" + tmp + "\"";
    FILE* p = _popen(a2l.c_str(), "r");
    if (!p) { std::printf("addr2line failed to launch — is MinGW on PATH?\n"); return 1; }

    std::unordered_map<std::string, std::uint64_t> byLine, byFunc, byFile;
    char buf[8192];
    for (size_t i = 0; i < addrs.size(); ++i) {
        if (!std::fgets(buf, sizeof(buf), p)) break;
        std::string func(buf); func.erase(func.find_last_not_of(" \r\n") + 1);
        if (!std::fgets(buf, sizeof(buf), p)) break;
        std::string loc(buf); loc.erase(loc.find_last_not_of(" \r\n") + 1);
        std::uint64_t c = hits[addrs[i]];
        if (func == "??") func = "[library / no-symbol]";
        std::string file = loc.substr(0, loc.find_last_of(':'));
        if (file.empty() || file == "??") file = "[library / no-line]";
        else file = basename(file);
        byLine[basename(loc)] += c;
        byFunc[func] += c;
        byFile[file] += c;
    }
    _pclose(p);

    auto topN = [](std::unordered_map<std::string, std::uint64_t>& m, size_t n) {
        std::vector<std::pair<std::string, std::uint64_t>> v(m.begin(), m.end());
        std::sort(v.begin(), v.end(), [](auto& a, auto& b) { return a.second > b.second; });
        if (v.size() > n) v.resize(n);
        return v;
    };
    auto pct = [&](std::uint64_t c) { return inMod ? 100.0 * double(c) / double(inMod) : 0.0; };

    std::printf("\nsampling line profile of %s\n", basename(exe).c_str());
    std::printf("samples: total=%llu  in-module=%llu (%.1f%%)  unique addrs=%zu\n",
                (unsigned long long)total, (unsigned long long)inMod,
                total ? 100.0 * inMod / total : 0.0, addrs.size());

    std::printf("\n== per-file rollup (self time) ==\n");
    for (auto& e : topN(byFile, 15))
        std::printf("  %7.1f%%  %8llu   %s\n", pct(e.second), (unsigned long long)e.second, e.first.c_str());

    std::printf("\n== hottest SOURCE LINES (self time) ==\n");
    std::printf("  %7s %9s   %s\n", "%", "samples", "file:line");
    for (auto& e : topN(byLine, 30))
        std::printf("  %6.1f%% %9llu   %s\n", pct(e.second), (unsigned long long)e.second, e.first.c_str());

    std::printf("\n== hottest FUNCTIONS (self time) ==\n");
    for (auto& e : topN(byFunc, 20))
        std::printf("  %6.1f%% %9llu   %s\n", pct(e.second), (unsigned long long)e.second, e.first.c_str());
    return 0;
}
