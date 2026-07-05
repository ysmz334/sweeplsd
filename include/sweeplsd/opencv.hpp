#pragma once

// OpenCV adapter for SweepLSD — optional, header-only.
//
// The core `sweeplsd` library is deliberately dependency-free (it knows nothing
// about OpenCV). This header is the bridge for projects that DO live in an
// OpenCV world: it lets you feed a `cv::Mat` straight in and get lines back as
// `cv::Vec4f` (x0,y0,x1,y1) — the exact format `cv::LineSegmentDetector`
// produces — so SweepLSD can be a drop-in replacement for OpenCV's own LSD.
//
// How to use it:
//   * Include this header ONLY in translation units that already use OpenCV.
//   * It needs <opencv2/core.hpp> + <opencv2/imgproc.hpp> on the include path,
//     and your build links OpenCV (opencv_core + opencv_imgproc, or the
//     consolidated opencv_world). The core sweeplsd library stays OpenCV-free.
//   * In CMake: link `sweeplsd::opencv` (header-only INTERFACE target that pulls in
//     sweeplsd::sweeplsd) and add OpenCV yourself, e.g.
//         find_package(sweeplsd CONFIG REQUIRED)
//         find_package(OpenCV REQUIRED)
//         target_link_libraries(app PRIVATE sweeplsd::opencv ${OpenCV_LIBS})
//
// Everything here is inline (nothing to compile into the library), which also
// sidesteps ABI/toolchain mismatches — you compile it against whatever OpenCV
// your project already uses.

#include <cstdint>
#include <cstring>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "sweeplsd/sweeplsd.hpp"

namespace sweeplsd {

// ---- Type conversions -----------------------------------------------------

// cv::Mat -> GrayImage (8-bit grayscale, deep copy).
// Accepts single-channel, BGR(A), or non-8-bit input: colour is converted with
// the standard BT.601 weights (cv::cvtColor) and other depths are scaled to
// 8-bit (cv::convertTo). Throws cv::Exception on an empty/unsupported Mat.
inline GrayImage fromMat(const cv::Mat& m) {
    CV_Assert(!m.empty());
    cv::Mat gray;
    switch (m.channels()) {
        case 1: gray = m; break;
        case 3: cv::cvtColor(m, gray, cv::COLOR_BGR2GRAY); break;
        case 4: cv::cvtColor(m, gray, cv::COLOR_BGRA2GRAY); break;
        default: CV_Error(cv::Error::StsUnsupportedFormat,
                          "sweeplsd::fromMat: expected 1, 3, or 4 channels");
    }
    if (gray.depth() != CV_8U) {
        cv::Mat tmp;
        gray.convertTo(tmp, CV_8U);
        gray = tmp;
    }
    GrayImage out(gray.cols, gray.rows);
    for (int y = 0; y < gray.rows; ++y) {
        const std::uint8_t* src = gray.ptr<std::uint8_t>(y);
        std::memcpy(out.data.data() + std::size_t(y) * gray.cols, src, gray.cols);
    }
    return out;
}

// GrayImage -> cv::Mat (CV_8UC1, deep copy).
inline cv::Mat toMat(const GrayImage& g) {
    cv::Mat m(g.height, g.width, CV_8UC1);
    for (int y = 0; y < g.height; ++y)
        std::memcpy(m.ptr<std::uint8_t>(y),
                    g.data.data() + std::size_t(y) * g.width, g.width);
    return m;
}

// LineSegment <-> cv::Vec4f (x0, y0, x1, y1) — the layout used by
// cv::LineSegmentDetector, so these vectors interoperate with OpenCV LSD code.
inline cv::Vec4f toVec4f(const LineSegment& s) {
    return cv::Vec4f(s.x0, s.y0, s.x1, s.y1);
}
inline LineSegment fromVec4f(const cv::Vec4f& v) {
    return LineSegment{v[0], v[1], v[2], v[3]};
}
inline std::vector<cv::Vec4f> toVec4f(const std::vector<LineSegment>& segs) {
    std::vector<cv::Vec4f> out;
    out.reserve(segs.size());
    for (const LineSegment& s : segs) out.emplace_back(s.x0, s.y0, s.x1, s.y1);
    return out;
}

// ---- One-call detection on a cv::Mat --------------------------------------

// Run SweepLSD on an image and return lines as cv::Vec4f, like cv::LSD.
// `detect` is the multi-pass version; `detectOnePass` is the streaming,
// O(width)-memory one. Both call the dependency-free core after fromMat().
inline std::vector<cv::Vec4f> detect(const cv::Mat& image, const Params& params = {}) {
    return toVec4f(detect(fromMat(image), params));
}
inline std::vector<cv::Vec4f> detectOnePass(const cv::Mat& image, const Params& params = {}) {
    return toVec4f(detectOnePass(fromMat(image), params));
}

// Draw segments onto an image (helper). Default colour is red (BGR).
inline void drawSegments(cv::Mat& canvas, const std::vector<cv::Vec4f>& lines,
                         const cv::Scalar& color = cv::Scalar(0, 0, 255),
                         int thickness = 1) {
    for (const cv::Vec4f& l : lines)
        cv::line(canvas, cv::Point(cvRound(l[0]), cvRound(l[1])),
                 cv::Point(cvRound(l[2]), cvRound(l[3])), color, thickness, cv::LINE_AA);
}

// ---- Drop-in class mirroring cv::LineSegmentDetector ----------------------

// Same call shape as OpenCV's detector so existing code can switch with minimal
// edits:
//     cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector();
//     lsd->detect(gray, lines);            //  -> becomes:
//     sweeplsd::LineSegmentDetector lsd;
//     lsd.detect(gray, lines);             //  lines: Nx1 CV_32FC4, same layout
class LineSegmentDetector {
public:
    // `one_pass` selects the streaming O(width)-memory detector instead of the
    // multi-pass one (identical output, lower memory).
    explicit LineSegmentDetector(const Params& params = {}, bool one_pass = false)
        : params_(params), one_pass_(one_pass) {}

    // Detect into an OutputArray as an Nx1 CV_32FC4 matrix of (x0,y0,x1,y1),
    // exactly like cv::LineSegmentDetector::detect.
    void detect(cv::InputArray image, cv::OutputArray lines) const {
        GrayImage g = fromMat(image.getMat());
        std::vector<LineSegment> segs =
            one_pass_ ? sweeplsd::detectOnePass(g, params_) : sweeplsd::detect(g, params_);
        cv::Mat out(static_cast<int>(segs.size()), 1, CV_32FC4);
        for (int i = 0; i < static_cast<int>(segs.size()); ++i)
            out.at<cv::Vec4f>(i, 0) = toVec4f(segs[std::size_t(i)]);
        out.copyTo(lines);
    }

    // Convenience overload returning the lines directly.
    std::vector<cv::Vec4f> detect(cv::InputArray image) const {
        GrayImage g = fromMat(image.getMat());
        return toVec4f(one_pass_ ? sweeplsd::detectOnePass(g, params_)
                                 : sweeplsd::detect(g, params_));
    }

    // Draw, mirroring cv::LineSegmentDetector::drawSegments (lines: CV_32FC4).
    void drawSegments(cv::InputOutputArray image, cv::InputArray lines) const {
        cv::Mat canvas = image.getMat();
        cv::Mat lm = lines.getMat();
        const int n = lm.rows * lm.cols;
        for (int i = 0; i < n; ++i) {
            const cv::Vec4f l = lm.at<cv::Vec4f>(i);
            cv::line(canvas, cv::Point(cvRound(l[0]), cvRound(l[1])),
                     cv::Point(cvRound(l[2]), cvRound(l[3])),
                     cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }
    }

    const Params& params() const { return params_; }
    Params& params() { return params_; }

private:
    Params params_;
    bool one_pass_;
};

}  // namespace sweeplsd
