#pragma once

// hls::stream compatibility shim.
//
// Under Vitis HLS (csim/csynth/cosim) the real <hls_stream.h> is used. In the
// plain-CMake host build (`SWEEPLSD_BUILD_HLS`, the everyday tool-free check)
// this header provides a minimal drop-in with the same blocking-FIFO
// semantics, so hls/src compiles and runs bit-exactly with any C++17 compiler.

#if defined(__SYNTHESIS__) || __has_include(<hls_stream.h>)

#include <hls_stream.h>

#else

#include <cassert>
#include <cstddef>
#include <deque>

namespace hls {

template <class T>
class stream {
public:
    void write(const T& v) { q_.push_back(v); }
    T read() {
        assert(!q_.empty() && "hls::stream shim: read from empty stream");
        T v = q_.front();
        q_.pop_front();
        return v;
    }
    bool empty() const { return q_.empty(); }
    std::size_t size() const { return q_.size(); }

private:
    std::deque<T> q_;
};

}  // namespace hls

#endif
