#include "algo/calibration/blinking_detect.h"
#include "algo/calibration/intrinsic.h"
#include "multicam/calibration.h"

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <random>
#include <stdexcept>
#include <vector>

namespace {
using namespace gui::multicam;
using Windows = std::array<CountWindow, 3>;
constexpr int width = 1280, height = 720;

// The pre-optimization path is the oracle: retain the same count threshold, full image and fallback detector.
DetectedGroup serial_detection(const Windows &windows) {
    DetectedGroup out;
    out.start_us = windows[0].start_us;
    for (int i = 0; i < 3; ++i) {
        if (windows[i].start_us != out.start_us)
            throw std::runtime_error("Calibration windows are not simultaneous.");
        out.sizes[i] = windows[i].on.size();
        gui_algo::BlinkParams params;
        params.ratio_on = params.ratio_off = 1;
        const auto frame = gui_algo::build_blink_frame_from_counts(windows[i].on, windows[i].off, params);
        gui_algo::IntrinsicCalibration detector;
        detector.set_pattern(gui_algo::CalibrationPattern::Chessboard, 9, 6, 1);
        if (frame.valid) {
            auto detection = detector.detect_only(frame.frame, false);
            if (detection.found)
                out.corners[i] = std::move(detection.points);
        }
    }
    return out;
}

enum class Scene { Clean, Noisy, Dense, Ragged, Clutter };

Windows checkerboards(Scene scene) {
    Windows windows;
    for (int camera = 0; camera < 3; ++camera) {
        auto &window = windows[camera];
        window.start_us = 1200000;
        const int square = scene == Scene::Ragged ? 30 : 50;
        cv::Mat board(height, width, CV_8U, cv::Scalar(255));
        for (int r = 0; r < 7; ++r)
            for (int c = 0; c < 10; ++c)
                if ((r + c) % 2 == 0)
                    board(cv::Rect(280 + c * square, 160 + r * square, square, square)).setTo(0);
        if (scene != Scene::Clean) {
            const std::array<cv::Point2f, 4> source = {cv::Point2f(280, 160),
                                                       {float(280 + 10 * square), 160},
                                                       {float(280 + 10 * square), float(160 + 7 * square)},
                                                       {280, float(160 + 7 * square)}};
            auto destination = source;
            destination[0] += cv::Point2f(-18 + 11 * camera, 12 - 7 * camera);
            destination[1] += cv::Point2f(9 - 8 * camera, -10 + 5 * camera);
            destination[2] += cv::Point2f(20 - 9 * camera, 11 + 5 * camera);
            destination[3] += cv::Point2f(-11 + 6 * camera, -7 + 2 * camera);
            cv::warpPerspective(board, board, cv::getPerspectiveTransform(source.data(), destination.data()),
                                board.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(255));
        }
        if (scene == Scene::Clutter) {
            std::mt19937 clutter(942 + camera);
            for (int y = 0; y < height; y += 8)
                for (int x = 0; x < width; x += 8)
                    board(cv::Rect(x, y, 8, 8)).setTo((clutter() & 1u) ? 0 : 255);
        }
        window.on = cv::Mat(height, width, CV_32S);
        window.off = cv::Mat(height, width, CV_32S);
        std::mt19937 rng(42 + camera);
        const int background = scene == Scene::Dense ? 8 : 0;
        const int foreground = scene == Scene::Dense ? 60 : 30;
        for (int y = 0; y < height; ++y) {
            auto *on = window.on.ptr<int>(y);
            auto *off = window.off.ptr<int>(y);
            const auto *row = board.ptr<uchar>(y);
            for (int x = 0; x < width; ++x) {
                bool black = row[x] == 0;
                if (scene == Scene::Ragged && (rng() & 255u) < 55u)
                    black = !black;
                const int count = black ? foreground : background;
                on[x] = off[x] = count;
                if (scene == Scene::Noisy || scene == Scene::Dense) {
                    on[x] += int(rng() % 3);
                    off[x] += int(rng() % 3);
                    if ((rng() & 1023u) == 0)
                        on[x] = off[x] = black ? background : foreground;
                }
            }
        }
    }
    return windows;
}

void equivalent(const DetectedGroup &actual, const DetectedGroup &reference, bool all_found = true) {
    EXPECT_EQ(actual.start_us, reference.start_us);
    EXPECT_EQ(actual.sizes, reference.sizes);
    for (int camera = 0; camera < 3; ++camera) {
        SCOPED_TRACE(camera);
        if (all_found)
            ASSERT_EQ(reference.corners[camera].size(), 54);
        EXPECT_EQ(actual.corners[camera], reference.corners[camera]);
    }
}

struct Sample {
    double wall_ms, cpu_ms;
};
template <typename F> Sample measure(F &&f) {
    const auto wall = std::chrono::steady_clock::now();
    const auto cpu = std::clock();
    const auto result = f();
    for (const auto &corners : result.corners)
        EXPECT_TRUE(corners.empty() || corners.size() == 54);
    return {std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall).count(),
            1000. * (std::clock() - cpu) / CLOCKS_PER_SEC};
}
double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}
class OpenCvThreadScope {
  public:
    explicit OpenCvThreadScope(int threads) : previous_(cv::getNumThreads()) { cv::setNumThreads(threads); }
    ~OpenCvThreadScope() { cv::setNumThreads(previous_); }

  private:
    int previous_;
};
} // namespace

TEST(MultiCameraDetection, CleanFullResolutionCornersAndInputCountsAreUnchanged) {
    const auto windows = checkerboards(Scene::Clean);
    std::array<cv::Mat, 3> on, off;
    for (int camera = 0; camera < 3; ++camera) {
        on[camera] = windows[camera].on.clone();
        off[camera] = windows[camera].off.clone();
    }
    const auto reference = serial_detection(windows);
    const int threads = cv::getNumThreads();
    equivalent(detect_group(windows), reference);
    EXPECT_EQ(cv::getNumThreads(), threads);
    for (int camera = 0; camera < 3; ++camera) {
        EXPECT_EQ(cv::norm(on[camera], windows[camera].on, cv::NORM_INF), 0);
        EXPECT_EQ(cv::norm(off[camera], windows[camera].off, cv::NORM_INF), 0);
        EXPECT_EQ(reference.sizes[camera], cv::Size(width, height));
    }
}

TEST(MultiCameraDetection, NoisyAndDensePerspectiveBoardsMatchTheSerialDetector) {
    for (const auto scene : {Scene::Noisy, Scene::Dense}) {
        SCOPED_TRACE(static_cast<int>(scene));
        const auto windows = checkerboards(scene);
        equivalent(detect_group(windows), serial_detection(windows));
    }
}

TEST(MultiCameraDetection, RaggedSquaresKeepTheExistingBridgeFallback) {
    const auto windows = checkerboards(Scene::Ragged);
    gui_algo::BlinkParams params;
    params.ratio_on = params.ratio_off = 1;
    const auto frame = gui_algo::build_blink_frame_from_counts(windows[0].on, windows[0].off, params);
    ASSERT_TRUE(frame.valid);
    std::vector<cv::Point2f> raw;
    EXPECT_FALSE(cv::findChessboardCorners(frame.frame, {9, 6}, raw, cv::CALIB_CB_FILTER_QUADS));
    equivalent(detect_group(windows), serial_detection(windows));
}

TEST(MultiCameraDetection, DenseClutterRetainsFailureResultsWithoutPartialCornerSets) {
    const auto windows = checkerboards(Scene::Clutter);
    const auto reference = serial_detection(windows);
    equivalent(detect_group(windows), reference, false);
    for (const auto &corners : reference.corners)
        EXPECT_TRUE(corners.empty());
}

TEST(MultiCameraDetection, MissingBoardDoesNotSuppressTheOtherCameras) {
    auto windows = checkerboards(Scene::Dense);
    windows[0].on.setTo(60);
    windows[0].off.setTo(60);
    const auto actual = detect_group(windows);
    equivalent(actual, serial_detection(windows), false);
    EXPECT_TRUE(actual.corners[0].empty());
    EXPECT_EQ(actual.corners[1].size(), 54);
    EXPECT_EQ(actual.corners[2].size(), 54);
}

TEST(MultiCameraDetection, RetainsTheExistingRelaxedOnAndOffPolarityThresholds) {
    auto windows = checkerboards(Scene::Clean);
    windows[0].off.setTo(0);
    windows[1].on.setTo(0);
    for (int camera = 0; camera < 2; ++camera) {
        EXPECT_FALSE(gui_algo::build_blink_frame_from_counts(windows[camera].on, windows[camera].off).valid);
        gui_algo::BlinkParams params;
        params.ratio_on = params.ratio_off = 1;
        EXPECT_TRUE(
            gui_algo::build_blink_frame_from_counts(windows[camera].on, windows[camera].off, params).valid);
    }
    equivalent(detect_group(windows), serial_detection(windows));
}

TEST(MultiCameraDetection, RejectsNonCommonTimeWindowsAndHandlesAbsentCounts) {
    auto windows = checkerboards(Scene::Clean);
    windows[2].start_us += calibration_window_us;
    EXPECT_THROW(detect_group(windows), std::runtime_error);
    windows[2].start_us = windows[0].start_us;
    windows[0].on.release();
    windows[1].off = cv::Mat::zeros(height / 2, width, CV_32S);
    windows[2].off = cv::Mat::zeros(height, width, CV_8U);
    const auto actual = detect_group(windows);
    equivalent(actual, serial_detection(windows), false);
    for (const auto &corners : actual.corners)
        EXPECT_TRUE(corners.empty());
}

TEST(MultiCameraDetection, EachCompletedGroupHasNoHistoryOrDeferredCandidateResults) {
    auto windows = checkerboards(Scene::Noisy);
    for (int group = 0; group < 4; ++group) {
        for (auto &window : windows)
            window.start_us = 1200000 + group * calibration_window_us;
        const auto reference = serial_detection(windows);
        equivalent(detect_group(windows), reference);
        // Once the call returns, no detector job may keep reading these shared count matrices.
        for (auto &window : windows) {
            window.on.setTo(0);
            window.off.setTo(0);
        }
        const auto empty = detect_group(windows);
        for (const auto &corners : empty.corners)
            EXPECT_TRUE(corners.empty());
        windows = checkerboards(Scene::Noisy);
    }
}

TEST(MultiCameraDetectionPerformance, ReportsWallAndProcessCpuWithoutTimingAssertions) {
    const int configured_threads = cv::getNumThreads();
    std::fprintf(stderr, "[detection benchmark] OpenCV=%s configured_threads=%d, 1280x720, 3x100ms\n",
                 CV_VERSION, configured_threads);
    std::vector<int> limits{1};
    if (configured_threads != 1)
        limits.push_back(configured_threads);
    for (const int threads : limits) {
        OpenCvThreadScope scope(threads);
        for (const auto scene : {Scene::Clean, Scene::Dense, Scene::Ragged, Scene::Clutter}) {
            const auto windows = checkerboards(scene);
            const auto reference = serial_detection(windows);
            equivalent(detect_group(windows), reference, scene != Scene::Clutter);
            double events = 0;
            for (const auto &window : windows)
                events += cv::sum(window.on)[0] + cv::sum(window.off)[0];
            std::vector<double> serial_wall, serial_cpu, group_wall, group_cpu;
            for (int round = 0; round < 7; ++round) {
                const auto serial = [&] { return measure([&] { return serial_detection(windows); }); };
                const auto grouped = [&] { return measure([&] { return detect_group(windows); }); };
                Sample a, b;
                if (round % 2 == 0) {
                    a = serial();
                    b = grouped();
                } else {
                    b = grouped();
                    a = serial();
                }
                serial_wall.push_back(a.wall_ms);
                serial_cpu.push_back(a.cpu_ms);
                group_wall.push_back(b.wall_ms);
                group_cpu.push_back(b.cpu_ms);
            }
            std::fprintf(stderr,
                         "[detection benchmark] scene=%s threads=%d total_counts=%.0f "
                         "serial_wall_ms=%.3f serial_cpu_ms=%.3f group_wall_ms=%.3f group_cpu_ms=%.3f "
                         "group_wall_range_ms=[%.3f,%.3f], median of 7 alternating rounds\n",
                         scene == Scene::Clean    ? "clean"
                         : scene == Scene::Dense  ? "dense"
                         : scene == Scene::Ragged ? "ragged"
                                                  : "clutter",
                         threads, events, median(serial_wall), median(serial_cpu), median(group_wall),
                         median(group_cpu), *std::min_element(group_wall.begin(), group_wall.end()),
                         *std::max_element(group_wall.begin(), group_wall.end()));
        }
    }
    EXPECT_EQ(cv::getNumThreads(), configured_threads);
}
