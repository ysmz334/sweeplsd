// Example: using SweepLSD inside an OpenCV pipeline via <sweeplsd/opencv.hpp>.
//
// This is the OpenCV counterpart of examples/vanishing_points.cpp. It shows the
// three ways the adapter lets you mix SweepLSD with cv::Mat / cv::Vec4f:
//   (a) the drop-in class sweeplsd::LineSegmentDetector, same call shape as
//       cv::LineSegmentDetector (detect/drawSegments, lines as Nx1 CV_32FC4);
//   (b) the one-call free function sweeplsd::detect(cv::Mat) -> vector<cv::Vec4f>;
//   (c) raw conversions sweeplsd::fromMat / toMat / toVec4f.
//
// It is built separately with MSVC + OpenCV (see tools/build_opencv_example.bat)
// because the rest of the repo builds with MinGW and the local OpenCV is an MSVC
// build — exactly like tools/edlines_real.cpp. The core sweeplsd library is still
// OpenCV-free; only this consumer links OpenCV.
//
//   opencv_detect <image> [out.png] [--one-pass] [--link] [--nfa]

#include <cstdio>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <sweeplsd/opencv.hpp>

int main(int argc, char** argv) {
    std::string input, out = "sweeplsd_opencv.png";
    bool one_pass = false;
    sweeplsd::Params params;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--one-pass") one_pass = true;
        else if (a == "--link") params.link_collinear = true;
        else if (a == "--nfa") params.use_nfa = true;
        else if (!a.empty() && a[0] != '-') {
            if (input.empty()) input = a; else out = a;
        }
    }
    if (input.empty()) {
        std::printf("Usage: %s <image> [out.png] [--one-pass] [--link] [--nfa]\n", argv[0]);
        return 1;
    }

    // Load with OpenCV as we would in any OpenCV program.
    cv::Mat src = cv::imread(input, cv::IMREAD_COLOR);
    if (src.empty()) {
        std::printf("Error: could not load '%s'\n", input.c_str());
        return 1;
    }

    // (a) Drop-in class, mirroring cv::LineSegmentDetector. fromMat() inside the
    //     adapter converts the BGR image to grayscale automatically.
    sweeplsd::LineSegmentDetector lsd(params, one_pass);
    cv::Mat lines;                 // Nx1 CV_32FC4, just like cv::LSD
    lsd.detect(src, lines);
    std::printf("sweeplsd::LineSegmentDetector: %d segments (%s)\n",
                lines.rows, one_pass ? "one-pass" : "multi-pass");

    // (b) Equivalent one-call free function returning cv::Vec4f directly.
    std::vector<cv::Vec4f> segs = sweeplsd::detect(src, params);

    // Draw on a dimmed copy and save with OpenCV.
    cv::Mat canvas;
    src.convertTo(canvas, -1, 0.5);          // dim background
    sweeplsd::drawSegments(canvas, segs, cv::Scalar(0, 0, 255), 1);
    if (!cv::imwrite(out, canvas)) {
        std::printf("Error: could not write '%s'\n", out.c_str());
        return 1;
    }
    std::printf("Wrote %s (%zu segments drawn)\n", out.c_str(), segs.size());
    return 0;
}
