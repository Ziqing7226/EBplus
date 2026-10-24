// gui/calibration/coverage_hull.h — global coverage overlay for the
// calibration wizard's aim view.
//
// Distortion parameters are conditioned by views near the EDGES of the field
// of view, so the wizard shows the user which sensor region the accepted
// captures already cover: the convex hull of all accepted corners plus its
// area fraction of the sensor. Pure computation, unit-tested in
// gui/tests/test_coverage_hull.cpp; rendering lives in CalibrationCameraView.

#ifndef GUI_CALIBRATION_COVERAGE_HULL_H
#define GUI_CALIBRATION_COVERAGE_HULL_H

#include <algorithm>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
// OpenCV 5.0 moved the 2D geometry helpers (convexHull, contourArea,
// boundingRect, minAreaRect, ...) from imgproc to the new 2d module; the
// imgproc.hpp of 5.x no longer declares them. Include 2d.hpp on v5+;
// CV_MAJOR_VERSION comes from the core.hpp include above.
#if CV_MAJOR_VERSION >= 5
#include <opencv2/2d.hpp>
#endif

namespace gui {

struct CoverageOverlay {
    /// Convex hull of the union of all accepted corners (sensor px);
    /// empty when fewer than 3 points exist in total.
    std::vector<cv::Point2f> hull;
    /// Hull area / sensor area, clamped to [0, 1].
    double coverage{0.0};
};

/// @brief Computes the convex hull over the union of every accepted view's
/// corners and its area fraction of the sensor. Guidance only — no gating.
inline CoverageOverlay compute_coverage_overlay(
    const std::vector<std::vector<cv::Point2f>>& views, int sensor_w, int sensor_h) {
    CoverageOverlay out;
    if (sensor_w <= 0 || sensor_h <= 0) return out;

    std::size_t total = 0;
    for (const auto& v : views) total += v.size();
    if (total < 3) return out;

    std::vector<cv::Point2f> all;
    all.reserve(total);
    for (const auto& v : views) all.insert(all.end(), v.begin(), v.end());

    cv::convexHull(all, out.hull);
    const double sensor_area = static_cast<double>(sensor_w) * sensor_h;
    out.coverage = std::clamp(cv::contourArea(out.hull) / sensor_area, 0.0, 1.0);
    return out;
}

} // namespace gui

#endif // GUI_CALIBRATION_COVERAGE_HULL_H
