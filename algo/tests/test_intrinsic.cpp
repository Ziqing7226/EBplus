// algo/tests/test_intrinsic.cpp — unit tests for IntrinsicCalibration (Zhang
// two-pass method).
//
// Locks: the chessboard object-point formula, the detect_only/accept split
// (detect does not accumulate), the duplicate-pose rejection used by the
// wizard, the two-pass calibration with per-view outlier rejection
// (bad views dropped, per-view RMS reported), and the square-size contract
// (a size-only reconfiguration keeps observations; run() solves with the
// CURRENT size; 0 mm fails cleanly). Full blink-frame detection is
// covered by test_blinking_detect.cpp.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <algorithm>
#include <random>
#include <vector>

#include <opencv2/core.hpp>
#include "algo/common/opencv5_compat.h"
#include <opencv2/imgproc.hpp>

#include "algo/calibration/intrinsic.h"

using gui_algo::CalibrationPattern;
using gui_algo::IntrinsicCalibration;

namespace {

// Synthesizes N views of a chessboard (inner corners 9×6, 20 mm squares)
// projected through a known pinhole camera, with optional per-view corruption
// (a large offset on selected views simulates bad detections).
struct SyntheticViews {
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<cv::Point3f> object_grid;
    cv::Mat K;
};

SyntheticViews make_views(std::size_t n_views,
                          const std::vector<bool>& corrupt = {},
                          double noise_px = 0.3) {
    constexpr int kCols = 9, kRows = 6;
    constexpr float kSquareMm = 20.0f;
    const double fx = 500.0, fy = 500.0, cx = 320.0, cy = 240.0;

    SyntheticViews sv;
    // cv::Mat_'s comma initializer is deprecated in OpenCV 5 — fill by hand.
    sv.K = cv::Mat_<double>(3, 3);
    {
        double* k = sv.K.ptr<double>();
        const double vals[9] = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
        for (int i = 0; i < 9; ++i) k[i] = vals[i];
    }

    // Object grid: 9×6 inner corners at (c·20, r·20, 0) mm.
    for (int r = 0; r < kRows; ++r)
        for (int c = 0; c < kCols; ++c)
            sv.object_grid.emplace_back(c * kSquareMm, r * kSquareMm, 0.0f);

    std::mt19937 rng(1234);
    std::normal_distribution<double> noise(0.0, noise_px);
    const cv::Mat no_dist = cv::Mat::zeros(1, 5, CV_64F);

    for (std::size_t i = 0; i < n_views; ++i) {
        // Varied, plausible poses (small rotations, camera at ~500 mm distance).
        const cv::Vec3d rvec(0.15 + 0.25 * static_cast<double>(i),
                             -0.05 * static_cast<double>(i),
                             0.02 * static_cast<double>(i));
        const cv::Vec3d tvec(-40.0 * static_cast<double>(i % 3),
                             25.0 * static_cast<double>((i / 3) % 2),
                             500.0);

        std::vector<cv::Point2f> pts;
        cv::projectPoints(sv.object_grid, rvec, tvec, sv.K, no_dist, pts);
        for (auto& p : pts) {
            p.x += static_cast<float>(noise(rng));
            p.y += static_cast<float>(noise(rng));
        }
        // Corrupt the view if requested by shuffling the 2D point order, so
        // the point-to-grid correspondence is wrong. A pixel shift is a bad
        // choice here: the solver can fold a uniform shift into the principal
        // point (which LM basin it picks differs per platform), making the
        // rejection outcome platform-dependent. A shuffled correspondence is
        // genuinely unfittable — the corrupted views' RMS stays an order of
        // magnitude above the clean ones on every toolchain.
        if (i < corrupt.size() && corrupt[i]) {
            std::shuffle(pts.begin(), pts.end(), rng);
        }
        sv.image_points.push_back(std::move(pts));
    }
    return sv;
}

void feed(IntrinsicCalibration& cal, const SyntheticViews& sv) {
    cal.set_pattern(CalibrationPattern::Chessboard, 9, 6, 20.0f);
    // detect_only records image_size_ on first call (mirrors the wizard, which
    // always detects before accumulating); the blank frame must not be found.
    auto res = cal.detect_only(cv::Mat(480, 640, CV_8UC1, cv::Scalar(0)), false);
    ASSERT_FALSE(res.found);
    for (const auto& pts : sv.image_points) cal.accept(pts);
}

} // namespace

TEST(IntrinsicCalibration, ChessboardObjectGridFormula) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::Chessboard, 9, 6, 20.0f);
    const auto g = cal.object_grid();
    ASSERT_EQ(g.size(), 54u);
    EXPECT_NEAR(g[0].x, 0, 1e-4f);           EXPECT_NEAR(g[0].y, 0, 1e-4f);
    EXPECT_NEAR(g[1].x, 20, 1e-4f);          EXPECT_NEAR(g[1].y, 0, 1e-4f);
    EXPECT_NEAR(g[8].x, 160, 1e-4f);         EXPECT_NEAR(g[8].y, 0, 1e-4f);
    EXPECT_NEAR(g[9].x, 0, 1e-4f);           EXPECT_NEAR(g[9].y, 20, 1e-4f);
    EXPECT_NEAR(g[53].x, 160, 1e-4f);        EXPECT_NEAR(g[53].y, 100, 1e-4f);
}

TEST(IntrinsicCalibration, DetectOnlyDoesNotAccumulate) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::Chessboard, 9, 6, 20.0f);
    cv::Mat black(128, 128, CV_8UC1, cv::Scalar(0));
    auto res = cal.detect_only(black, false);
    EXPECT_FALSE(res.found);
    EXPECT_EQ(cal.frame_count(), 0u);
}

TEST(IntrinsicCalibration, AcceptAccumulatesAndDuplicateDetected) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::Chessboard, 9, 6, 20.0f);
    // Synthesize 54 ordered points matching the 9×6 grid (row-major, *10 px scale).
    std::vector<cv::Point2f> pts;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 9; ++c)
            pts.emplace_back(c * 20.f * 10.f, r * 20.f * 10.f);
    cal.accept(pts);
    EXPECT_EQ(cal.frame_count(), 1u);
    // Identical points → duplicate of the stored pose.
    EXPECT_TRUE(cal.is_duplicate_pose(pts, 10.0));
    // Translated points → a different pose.
    std::vector<cv::Point2f> shifted = pts;
    for (auto& p : shifted) { p.x += 50; p.y += 50; }
    EXPECT_FALSE(cal.is_duplicate_pose(shifted, 10.0));
}

TEST(IntrinsicCalibration, RejectsEmptyFrame) {
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::Chessboard, 9, 6, 20.0f);
    auto res = cal.detect_only(cv::Mat(), false);
    EXPECT_FALSE(res.found);
    EXPECT_TRUE(res.points.empty());
}

TEST(IntrinsicCalibration, TwoPassCalibrationRecoversIntrinsics) {
    SyntheticViews sv = make_views(8);
    IntrinsicCalibration cal;
    feed(cal, sv);

    const auto r = cal.run();
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(r.kept_frames, 8u);
    EXPECT_EQ(r.removed_frames, 0u);
    EXPECT_EQ(r.frames_used, 8u);
    EXPECT_EQ(r.per_view_rms.size(), 8u);
    // 0.3 px noise → RMS well under 2 px.
    EXPECT_LT(r.rms, 2.0);
    // Camera matrix recovered near ground truth (fx = fy = 500, cx = 320, cy = 240).
    EXPECT_NEAR(r.K.at<double>(0, 0), 500.0, 30.0);
    EXPECT_NEAR(r.K.at<double>(1, 1), 500.0, 30.0);
    EXPECT_NEAR(r.K.at<double>(0, 2), 320.0, 30.0);
    EXPECT_NEAR(r.K.at<double>(1, 2), 240.0, 30.0);
}

TEST(IntrinsicCalibration, OutlierViewIsRemoved) {
    // One corrupted view (of 8) must be dropped by the default mean+2σ rule.
    const std::vector<bool> corrupt = {false, false, false, true, false, false, false, false};
    SyntheticViews sv = make_views(8, corrupt);
    IntrinsicCalibration cal;
    feed(cal, sv);

    const auto r = cal.run();
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(r.removed_frames, 1u);
    EXPECT_EQ(r.kept_frames, 7u);
    EXPECT_EQ(r.frames_used, 7u);
    EXPECT_EQ(r.per_view_rms.size(), 7u);
    // The corrupted view is flagged in selected_views (index 3).
    ASSERT_EQ(r.selected_views.size(), 8u);
    EXPECT_FALSE(r.selected_views[3]);
    EXPECT_LT(r.rms, 2.0);
}

TEST(IntrinsicCalibration, NegativeOutlierThresholdKeepsAll) {
    const std::vector<bool> corrupt = {false, false, false, true, false, false};
    SyntheticViews sv = make_views(6, corrupt);
    IntrinsicCalibration cal;
    feed(cal, sv);
    cal.set_outlier_threshold(-1.0);  // keep all views

    const auto r = cal.run();
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(r.removed_frames, 0u);
    EXPECT_EQ(r.kept_frames, 6u);
}

TEST(IntrinsicCalibration, TooFewViewsAfterRejectionFails) {
    // 6 views, 4 of them heavily corrupted; an aggressive outlier threshold
    // (0.1σ) rejects them → only 2 survive → run() errors.
    const std::vector<bool> corrupt = {false, true, true, true, true, false};
    SyntheticViews sv = make_views(6, corrupt);
    IntrinsicCalibration cal;
    feed(cal, sv);
    cal.set_outlier_threshold(0.1);

    const auto r = cal.run();
    EXPECT_FALSE(r.ok);
    EXPECT_FALSE(r.error.empty());
    EXPECT_LT(r.kept_frames, 3u);
}

TEST(IntrinsicCalibration, FewerThanThreeFramesFails) {
    SyntheticViews sv = make_views(2);
    IntrinsicCalibration cal;
    feed(cal, sv);

    const auto r = cal.run();
    EXPECT_FALSE(r.ok);
    EXPECT_FALSE(r.error.empty());
}

// A square-size-only set_pattern call must NOT clear the accepted captures:
// the size is a pure world-scale factor and the observations are pixel data,
// so the wizard can (re)enter the measured value at any point of a session.
TEST(IntrinsicCalibration, SquareSizeRescaleKeepsObservations) {
    SyntheticViews sv = make_views(5);
    IntrinsicCalibration cal;
    feed(cal, sv);
    ASSERT_EQ(cal.frame_count(), 5u);

    cal.set_pattern(CalibrationPattern::Chessboard, 9, 6, 5.0f);
    EXPECT_EQ(cal.frame_count(), 5u);

    const auto r = cal.run();
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(r.frames_used, 5u);
    // K is world-scale invariant — the rescaled solve recovers the same K.
    EXPECT_NEAR(r.K.at<double>(0, 0), 500.0, 30.0);
    EXPECT_NEAR(r.K.at<double>(0, 2), 320.0, 30.0);
}

// run() must rebuild the object grids from the CURRENT square size: views
// synthesized against a 20 mm grid but solved after re-configuring to 5 mm
// recover translations shrunk by 5/20 (proving the final value is the one
// used, not a per-capture frozen copy).
TEST(IntrinsicCalibration, RunUsesCurrentSquareSize) {
    SyntheticViews sv = make_views(4);
    IntrinsicCalibration cal;
    feed(cal, sv);
    cal.set_pattern(CalibrationPattern::Chessboard, 9, 6, 5.0f);

    const auto r = cal.run();
    ASSERT_TRUE(r.ok) << r.error;
    ASSERT_EQ(r.tvecs.size(), 4u);
    // Ground-truth tvec z = 500 mm at 20 mm squares → 125 mm at 5 mm squares.
    for (const auto& tv : r.tvecs) {
        EXPECT_NEAR(tv.at<double>(2), 125.0, 10.0);
    }
}

// The "not measured" default (0 mm) must fail cleanly with an actionable
// message instead of feeding cv::calibrateCamera a degenerate all-origin
// object grid.
TEST(IntrinsicCalibration, RunFailsWhileSquareSizeNotSet) {
    SyntheticViews sv = make_views(4);
    IntrinsicCalibration cal;
    feed(cal, sv);
    cal.set_pattern(CalibrationPattern::Chessboard, 9, 6, 0.0f);

    const auto r = cal.run();
    EXPECT_FALSE(r.ok);
    EXPECT_NE(r.error.find("Square size"), std::string::npos);
}

// Corner-bridge fallback: a real thresholded blink frame carries
// salt-and-pepper square damage whose ragged quad contours the raw
// FILTER_QUADS pass cannot link into a board. detect_only must then dilate
// the board regions and retry. The rescued corners are noisier than a raw
// pass (a few px at this harsh corruption level) — acceptable, because the
// solver's per-view outlier rejection drops views that disagree with the
// majority. (Clean synthetic erosion does NOT reproduce the failure —
// OpenCV links pristine quads even when their corners no longer touch.)
TEST(IntrinsicCalibration, DilationFallbackRescuesErodedCorners) {
    constexpr int kSq = 30;          // px per square (10×7 squares → 9×6 corners)
    constexpr int kOx = 60, kOy = 40;
    cv::Mat board(280, 400, CV_8UC1, cv::Scalar(255));
    for (int r = 0; r < 7; ++r) {
        for (int c = 0; c < 10; ++c) {
            if (((r + c) & 1) == 0) {
                cv::rectangle(board, {kOx + c * kSq, kOy + r * kSq},
                              {kOx + (c + 1) * kSq - 1, kOy + (r + 1) * kSq - 1},
                              cv::Scalar(0), cv::FILLED);
            }
        }
    }
    // Ground-truth inner corners (row-major from the top-left).
    std::vector<cv::Point2f> gt;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 9; ++c)
            gt.emplace_back(static_cast<float>(kOx + (c + 1) * kSq),
                            static_cast<float>(kOy + (r + 1) * kSq));

    // Sanity: the clean board detects via the raw pass.
    IntrinsicCalibration cal;
    cal.set_pattern(CalibrationPattern::Chessboard, 9, 6, 20.0f);
    const auto clean = cal.detect_only(board, false);
    ASSERT_TRUE(clean.found);

    // Corrupt with salt-and-pepper raggedness (~16% of pixels flipped). A
    // dedicated RNG (not cv::theRNG(), whose state the sanity check above
    // consumes) keeps the corruption deterministic.
    cv::Mat noise(board.size(), CV_8UC1);
    cv::RNG rng(42);
    rng.fill(noise, cv::RNG::UNIFORM, 0, 256);
    cv::Mat eroded;
    cv::bitwise_xor(board, noise < 40, eroded);

    // Pin the premise: raw OpenCV cannot link this frame, so a successful
    // detect_only below necessarily comes from the fallback.
    {
        std::vector<cv::Point2f> raw_corners;
        EXPECT_FALSE(cv::findChessboardCorners(eroded, {9, 6}, raw_corners,
                                               cv::CALIB_CB_FILTER_QUADS))
            << "test premise broken: raw pass unexpectedly links this frame";
    }

    const auto res = cal.detect_only(eroded, false);
    ASSERT_TRUE(res.found) << "fallback must rescue the ragged board";
    ASSERT_EQ(res.points.size(), gt.size());
    // Compare as sets (corner ORDER may flip under the fallback): every
    // ground-truth corner must have a detected corner nearby.
    double worst = 0.0, sum = 0.0;
    for (const auto& g : gt) {
        double best = 1e9;
        for (const auto& p : res.points) best = std::min(best, cv::norm(p - g));
        worst = std::max(worst, best);
        sum += best;
    }
    std::fprintf(stderr, "[test] corner error: mean=%.2f px worst=%.2f px\n",
                 sum / gt.size(), worst);
    EXPECT_LT(sum / gt.size(), 2.0) << "mean corner error of the rescued view";
    EXPECT_LT(worst, 6.0) << "worst corner error of the rescued view";
}
