// bench_frame_conv — Mat -> QImage conversion chain A/B.
//
// Usage: bench_frame_conv [width height iters]
//
// Compares the current product chain (cvtColor into a fresh Mat, wrap in a
// QImage, deep-copy) against the direct-write chain (allocate the QImage
// first, cvtColor straight into its buffer). Asserts byte-for-byte equality
// of the two results, then reports per-frame medians for both.

#include <QImage>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

// Reference implementation of the CURRENT product chain (frame_pipeline /
// file_frame_generator / display_strategy::mat_to_qimage).
QImage conv_old(const cv::Mat& frame) {
    cv::Mat rgb;
    if (frame.channels() == 1) {
        cv::cvtColor(frame, rgb, cv::COLOR_GRAY2RGB);
    } else {
        cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
    }
    QImage img(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step),
               QImage::Format_RGB888);
    return img.copy(); // deep copy (rgb is local)
}

// Candidate: the QImage owns its buffer from the start; cvtColor writes into
// it through a stride-aware view. One allocation, no second full-frame copy.
QImage conv_new(const cv::Mat& frame) {
    QImage dst(frame.cols, frame.rows, QImage::Format_RGB888);
    if (dst.isNull()) return QImage();
    cv::Mat dst_view(frame.rows, frame.cols, CV_8UC3, dst.bits(),
                     static_cast<size_t>(dst.bytesPerLine()));
    if (frame.channels() == 1) {
        cv::cvtColor(frame, dst_view, cv::COLOR_GRAY2RGB);
    } else {
        cv::cvtColor(frame, dst_view, cv::COLOR_BGR2RGB);
    }
    return dst;
}

bool qimage_equal(const QImage& a, const QImage& b) {
    if (a.size() != b.size() || a.format() != b.format()) return false;
    for (int y = 0; y < a.height(); ++y) {
        if (std::memcmp(a.constScanLine(y), b.constScanLine(y),
                        static_cast<size_t>(a.bytesPerLine())) != 0) {
            return false;
        }
    }
    return true;
}

double median_us(std::vector<double>& v) {
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

} // namespace

int main(int argc, char** argv) {
    const int w = argc > 1 ? std::atoi(argv[1]) : 1280;
    const int h = argc > 2 ? std::atoi(argv[2]) : 720;
    const int iters = argc > 3 ? std::atoi(argv[3]) : 500;

    // Deterministic synthetic frames (3-channel BGR and 1-channel gray).
    cv::Mat bgr(h, w, CV_8UC3), gray(h, w, CV_8UC1);
    for (int y = 0; y < h; ++y) {
        auto* rb = bgr.ptr<cv::Vec3b>(y);
        auto* rg = gray.ptr<std::uint8_t>(y);
        for (int x = 0; x < w; ++x) {
            const int v = (x * 7 + y * 13) & 0xFF;
            rb[x] = cv::Vec3b(static_cast<std::uint8_t>(v),
                              static_cast<std::uint8_t>(255 - v),
                              static_cast<std::uint8_t>((x ^ y) & 0xFF));
            rg[x] = static_cast<std::uint8_t>(v);
        }
    }

    // Correctness first: both channels, both sources.
    if (!qimage_equal(conv_old(bgr), conv_new(bgr))) {
        std::fprintf(stderr, "bench_frame_conv: BGR path MISMATCH\n");
        return 1;
    }
    if (!qimage_equal(conv_old(gray), conv_new(gray))) {
        std::fprintf(stderr, "bench_frame_conv: GRAY path MISMATCH\n");
        return 1;
    }
    std::printf("frame_conv: equality OK (%dx%d)\n", w, h);

    std::vector<double> t_old, t_new;
    t_old.reserve(iters);
    t_new.reserve(iters);
    for (int i = 0; i < iters; ++i) {
        // BGR is the dominant product path.
        auto a = std::chrono::steady_clock::now();
        const QImage q1 = conv_old(bgr);
        auto b = std::chrono::steady_clock::now();
        const QImage q2 = conv_new(bgr);
        auto c = std::chrono::steady_clock::now();
        t_old.push_back(std::chrono::duration<double, std::micro>(b - a).count());
        t_new.push_back(std::chrono::duration<double, std::micro>(c - b).count());
        (void)q1;
        (void)q2;
    }
    std::printf("frame_conv: old %.1f us/frame | new %.1f us/frame | speedup %.2fx\n",
                median_us(t_old), median_us(t_new), median_us(t_old) / median_us(t_new));
    return 0;
}
