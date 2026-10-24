// algo/calibration/intrinsic.cpp

#include "intrinsic.h"

#include <algorithm>
#include <cmath>
#include <filesystem>

#include <opencv2/imgproc.hpp>
// OpenCV 5.0 split calib3d and imgproc — see opencv5_compat.h.
#include "algo/common/opencv5_compat.h"

#include <opencv2/core/persistence.hpp>

#include "blinking_detect.h"

namespace gui_algo {

namespace {
// Radial-distortion-friendly grid validation: every row and column must stay// approximately straight — the cosine of the angle between two consecutive
// vectors of three consecutive points must stay above cos_max_angle (standard
// points-on-grid radial check; perspective keeps lines straight, so this only
// rejects genuinely mis-fit point sets). cos(π/9) ≈ 0.94 (~20° bend) is enough
// once the corners come from the raw FILTER_QUADS pass (subpixel refinement
// on the binary blink frame is what previously bent the rows out of shape).
bool points_on_grid_radial(const std::vector<cv::Point2f>& pts, int cols, int rows,
                           float cos_max_angle) {
    if (static_cast<int>(pts.size()) != cols * rows || cols < 3 || rows < 3) return false;
    const auto angle_ok = [cos_max_angle](const cv::Point2f& a, const cv::Point2f& b,
                                          const cv::Point2f& c) {
        const cv::Point2f v1 = b - a;
        const cv::Point2f v2 = c - b;
        const double n1 = cv::norm(v1), n2 = cv::norm(v2);
        if (n1 < 1e-6 || n2 < 1e-6) return true;  // degenerate segment — skip
        const double dot = v1.x * v2.x + v1.y * v2.y;
        return dot / (n1 * n2) >= static_cast<double>(cos_max_angle);
    };
    // Rows (row-major layout: point r*cols+c).
    for (int r = 0; r < rows; ++r) {
        for (int c = 1; c < cols - 1; ++c) {
            if (!angle_ok(pts[r * cols + c - 1], pts[r * cols + c], pts[r * cols + c + 1]))
                return false;
        }
    }
    // Columns.
    for (int c = 0; c < cols; ++c) {
        for (int r = 1; r < rows - 1; ++r) {
            if (!angle_ok(pts[(r - 1) * cols + c], pts[r * cols + c], pts[(r + 1) * cols + c]))
                return false;
        }
    }
    return true;
}

// Dilation fallback for binary blink frames whose square contours are too
// ragged for the raw FILTER_QUADS pass (real thresholded blink frames carry
// salt-and-pepper damage; small ~30 px squares are hit hardest). Dilating
// the black (board) regions with a small odd kernel smooths the quad
// contours and partially restores the threshold-eroded square edges, so the
// rescued corners are if anything CLOSER to the true grid than the raw
// frame's — no geometric compensation is applied (a uniform-shift model was
// tried and made accuracy worse: dilation deforms quads non-uniformly).
// The frame is board-black (0) on white (255).
cv::Mat bridge_corners(const cv::Mat& gray, int k) {
    cv::Mat black;
    cv::threshold(gray, black, 128, 255, cv::THRESH_BINARY_INV);
    cv::dilate(black, black, cv::getStructuringElement(cv::MORPH_RECT, {k, k}));
    cv::Mat out;
    cv::bitwise_not(black, out);
    return out;
}

// The chessboard detector was reworked between OpenCV releases: on 4.13+ and
// 5.x the dilate-only fallback no longer rescues salt-and-pepper-damaged
// binary frames (the raw pass still fails on them, see
// DilationFallbackRescuesErodedCorners), while on <= 4.12 dilate works and a
// median pass was never reached. The fallback therefore keeps dilate first
// (older OpenCV behavior is bit-identical) and adds a median variant — the
// canonical salt-and-pepper remover — for the newer detectors, plus a
// morphological close (fills the speckle holes in the board regions without
// fattening them) under OpenCV 5 only.
cv::Mat bridge_corners_median(const cv::Mat& gray, int k) {
    cv::Mat black, filtered, out;
    cv::threshold(gray, black, 128, 255, cv::THRESH_BINARY_INV);
    cv::medianBlur(black, filtered, k);
    cv::bitwise_not(filtered, out);
    return out;
}

#if CV_MAJOR_VERSION >= 5
cv::Mat bridge_corners_close(const cv::Mat& gray, int k) {
    cv::Mat black, closed, out;
    cv::threshold(gray, black, 128, 255, cv::THRESH_BINARY_INV);
    cv::morphologyEx(black, closed, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, {k, k}));
    cv::bitwise_not(closed, out);
    return out;
}
#endif
} // namespace

IntrinsicCalibration::IntrinsicCalibration() = default;
IntrinsicCalibration::~IntrinsicCalibration() = default;

void IntrinsicCalibration::set_pattern(CalibrationPattern pattern,
                                       int cols, int rows,
                                       float square_size_mm) {
    // For chessboard, (cols, rows) IS the OpenCV inner-corner count — the same
    // convention as cv::findChessboardCorners' patternSize, the wizard's UI
    // and BlinkingChessboardDisplay (which draws (cols+1)×(rows+1) squares).
    // For circle grids, the count is the number of circles per row/column.
    cv::Size new_bs = cv::Size(std::max(cols, 1), std::max(rows, 1));
    // Pattern type / corner-count changes invalidate accumulated observations
    // (a different point count or correspondence order would feed
    // cv::calibrateCamera inconsistent sets → throw or wrong result). The
    // square size does NOT: it is a uniform world-scale factor, and run()
    // rebuilds every view's object grid from the current value, so the wizard
    // can re-scale mid-session without losing the already-accepted captures.
    if (pattern_ != pattern || board_size_ != new_bs) {
        image_points_.clear();
        image_size_ = cv::Size(0, 0);
    }
    pattern_ = pattern;
    board_size_ = new_bs;
    square_size_mm_ = square_size_mm;
}

void IntrinsicCalibration::set_outlier_threshold(double ths) {
    outlier_ths_ = ths;
}

std::vector<cv::Point3f> IntrinsicCalibration::make_object_grid() const {
    std::vector<cv::Point3f> pts;
    pts.reserve(static_cast<std::size_t>(board_size_.width) *
                static_cast<std::size_t>(board_size_.height));
    // Both supported patterns are rectangular grids with the user-entered
    // square (or circle-spacing) size as the cell edge.
    for (int r = 0; r < board_size_.height; ++r) {
        for (int c = 0; c < board_size_.width; ++c) {
            pts.emplace_back(c * square_size_mm_, r * square_size_mm_, 0.0f);
        }
    }
    return pts;
}

DetectionResult IntrinsicCalibration::detect_only(const cv::Mat& frame, bool annotate) {
    DetectionResult result;
    if (frame.empty()) {
        result.found = false;
        return result;
    }
    if (image_size_.area() == 0) {
        image_size_ = frame.size();
    }

    cv::Mat gray;
    if (frame.channels() == 1) {
        gray = frame;
    } else {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }

    std::vector<cv::Point2f> corners;
    bool found = false;

    switch (pattern_) {
        case CalibrationPattern::Chessboard:
            // The blink frame is already a clean binary chessboard, so a plain
            // FILTER_QUADS search suffices — adaptive thresholding a binary
            // frame can distort square sizes and break detection on real
            // noisy blink frames. cornerSubPix is intentionally NOT applied:
            // on a binary frame its 11×11 gradient window drags corners off
            // the grid lines (the radial straightness check then rejects
            // valid detections on real recordings).
            found = cv::findChessboardCorners(gray, board_size_, corners,
                                              cv::CALIB_CB_FILTER_QUADS);
            if (found) {
                // Reject mis-fit point sets (straight rows/columns check).
                if (!points_on_grid_radial(corners, board_size_.width,
                                           board_size_.height,
                                           static_cast<float>(std::cos(3.14159 / 9)))) {
                    found = false;
                    corners.clear();
                }
            }
            if (!found) {
                // Corner-bridge fallback: real thresholded blink frames carry
                // salt-and-pepper square damage that breaks the raw quad
                // linking (see bridge_corners). Dilate the board regions and
                // retry with growing kernels; rescued views still go through
                // the straightness gate, and the solver's outlier rejection
                // drops any that disagree with the majority.
                const auto try_bridge = [&](const cv::Mat& bridged) {
                    if (cv::findChessboardCorners(bridged, board_size_, corners,
                                                  cv::CALIB_CB_FILTER_QUADS) &&
                        points_on_grid_radial(corners, board_size_.width,
                                              board_size_.height,
                                              static_cast<float>(std::cos(3.14159 / 9)))) {
                        found = true;
                        return;
                    }
                    corners.clear();
                };
                for (const int k : {3, 5}) {
                    try_bridge(bridge_corners(gray, k));
                    if (found) break;
                }
                // Newer detectors (OpenCV 4.13+/5.x) need real speckle
                // removal; on older OpenCV dilate already rescued the frame,
                // so this pass is never reached and behavior is unchanged.
                for (const int k : {3, 5}) {
                    try_bridge(bridge_corners_median(gray, k));
                    if (found) break;
                }
#if CV_MAJOR_VERSION >= 5
                for (const int k : {3, 5}) {
                    try_bridge(bridge_corners_close(gray, k));
                    if (found) break;
                }
#endif
            }
            break;
        case CalibrationPattern::CircleGrid:
            found = cv::findCirclesGrid(gray, board_size_, corners);
            break;
    }

    result.found = found;
    result.points = corners;

    if (annotate) {
        if (frame.channels() == 1) {
            cv::cvtColor(frame, result.image, cv::COLOR_GRAY2BGR);
        } else {
            result.image = frame.clone();
        }
        cv::drawChessboardCorners(result.image, board_size_, corners, found);
    }
    // NOTE: no accumulation here — the caller decides via accept().
    return result;
}

void IntrinsicCalibration::accept(const std::vector<cv::Point2f>& points) {
    image_points_.push_back(points);
}

void IntrinsicCalibration::remove_last_frame() {
    if (!image_points_.empty()) {
        image_points_.pop_back();
    }
}

DetectionResult IntrinsicCalibration::add_frame(const cv::Mat& frame, bool annotate) {
    DetectionResult result = detect_only(frame, annotate);
    if (result.found) {
        accept(result.points);
    }
    return result;
}

bool IntrinsicCalibration::is_duplicate_pose(
    const std::vector<cv::Point2f>& points, double threshold_px) const {
    if (points.empty()) return false;
    for (const auto& stored : image_points_) {
        if (stored.size() != points.size()) continue;
        double sum = 0.0;
        for (std::size_t i = 0; i < points.size(); ++i) {
            const double dx = stored[i].x - points[i].x;
            const double dy = stored[i].y - points[i].y;
            sum += std::sqrt(dx * dx + dy * dy);
        }
        const double mean = sum / static_cast<double>(points.size());
        if (mean < threshold_px) return true;
    }
    return false;
}

IntrinsicResult IntrinsicCalibration::run() {
    IntrinsicResult result;
    if (image_points_.size() < 3) {
        result.error = "Need at least 3 valid frames; got " +
                       std::to_string(image_points_.size());
        return result;
    }
    if (image_size_.area() == 0) {
        result.error = "Image size not yet known";
        return result;
    }
    if (square_size_mm_ <= 0.0f) {
        // A zero square collapses every object point to the origin — the
        // per-view homographies degenerate and calibrateCamera throws or
        // returns nonsense. Fail with an actionable message instead (the
        // wizard surfaces it on the status line / warning box).
        result.error = "Square size is not set (0 mm) — measure one "
                       "chessboard square edge and enter it before "
                       "calibrating.";
        return result;
    }

    try {
        const std::size_t n_views = image_points_.size();

        // All views share the board geometry (a pattern/corner-count change
        // clears observations in set_pattern), so ONE object grid — built
        // from the CURRENT square size — serves every view. Rebuilding here
        // (instead of freezing grids at accept() time) is what lets the user
        // re-scale the board mid-session and still get a calibration that
        // uses the final value.
        const std::vector<cv::Point3f> grid = make_object_grid();
        const std::vector<std::vector<cv::Point3f>>
            object_points(n_views, grid);

        // ---- Pass 1: coarse fit. K3 + aspect ratio fixed for a stable initial
        // estimate and reliable per-view poses (standard Zhang two-pass practice).
        cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);
        std::vector<cv::Mat> rvecs, tvecs;
        cv::calibrateCamera(object_points, image_points_, image_size_, K, dist,
                            rvecs, tvecs,
                            cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_FIX_K3);

        // ---- Per-view RMS reprojection errors (pass-1 parameters). ----
        std::vector<double> per_view_rms(n_views, 0.0);
        for (std::size_t i = 0; i < n_views; ++i) {
            std::vector<cv::Point2f> proj;
            cv::projectPoints(object_points[i], rvecs[i], tvecs[i], K, dist, proj);
            double e2 = 0.0;
            const std::size_t n = image_points_[i].size();
            for (std::size_t k = 0; k < n; ++k) {
                const double dx = proj[k].x - image_points_[i][k].x;
                const double dy = proj[k].y - image_points_[i][k].y;
                e2 += dx * dx + dy * dy;
            }
            per_view_rms[i] = std::sqrt(e2 / static_cast<double>(std::max<std::size_t>(1, n)));
        }

        // ---- Outlier rejection: drop views with error > mean + ths·std. ----
        std::vector<bool> selected(n_views, true);
        if (outlier_ths_ > 0.0) {
            double sum = 0.0, sumsq = 0.0;
            for (double e : per_view_rms) { sum += e; sumsq += e * e; }
            const double mean = sum / static_cast<double>(n_views);
            const double var = std::max(0.0, sumsq / static_cast<double>(n_views) - mean * mean);
            const double ths = mean + outlier_ths_ * std::sqrt(var);
            for (std::size_t i = 0; i < n_views; ++i) {
                if (per_view_rms[i] > ths) selected[i] = false;
            }
        }
        result.kept_frames = static_cast<std::size_t>(
            std::count(selected.begin(), selected.end(), true));
        result.removed_frames = n_views - result.kept_frames;
        if (result.kept_frames < 3) {
            result.error = "Outlier rejection removed too many views (" +
                           std::to_string(result.removed_frames) +
                           " of " + std::to_string(n_views) +
                           "); at least 3 must remain. Capture more frames.";
            return result;  // ok stays false
        }

        // ---- Pass 2: refit on kept views with K3 free. ----
        {
            std::vector<std::vector<cv::Point2f>> kept_pts;
            std::vector<std::vector<cv::Point3f>> kept_obj;
            kept_pts.reserve(result.kept_frames);
            kept_obj.reserve(result.kept_frames);
            for (std::size_t i = 0; i < n_views; ++i) {
                if (!selected[i]) continue;
                kept_pts.push_back(image_points_[i]);
                kept_obj.push_back(grid);
            }
            std::vector<cv::Mat> kept_rvecs, kept_tvecs;
            cv::calibrateCamera(kept_obj, kept_pts, image_size_, K, dist,
                                kept_rvecs, kept_tvecs, cv::CALIB_FIX_ASPECT_RATIO);

            // Per-view RMS with the FINAL parameters (kept views only).
            std::vector<double> final_rms;
            final_rms.reserve(result.kept_frames);
            for (std::size_t i = 0; i < result.kept_frames; ++i) {
                std::vector<cv::Point2f> proj;
                cv::projectPoints(kept_obj[i], kept_rvecs[i], kept_tvecs[i], K, dist, proj);
                double e2 = 0.0;
                const std::size_t n = kept_pts[i].size();
                for (std::size_t k = 0; k < n; ++k) {
                    const double dx = proj[k].x - kept_pts[i][k].x;
                    const double dy = proj[k].y - kept_pts[i][k].y;
                    e2 += dx * dx + dy * dy;
                }
                final_rms.push_back(std::sqrt(e2 / static_cast<double>(std::max<std::size_t>(1, n))));
            }

            double rms = 0.0;
            for (double e : final_rms) rms += e * e;
            rms = std::sqrt(rms / static_cast<double>(result.kept_frames));

            result.rms = rms;
            result.K = K;
            result.dist_coeffs = dist;
            result.rvecs = std::move(kept_rvecs);
            result.tvecs = std::move(kept_tvecs);
            result.frames_used = result.kept_frames;
            result.per_view_rms = std::move(final_rms);
            result.selected_views = std::move(selected);
            // Set ok only after every potentially-throwing operation succeeded.
            result.ok = true;
        }
    } catch (const cv::Exception& e) {
        result.error = e.what();
    } catch (const std::exception& e) {
        result.error = e.what();
    }
    return result;
}

void IntrinsicCalibration::reset() {
    image_points_.clear();
    image_size_ = cv::Size(0, 0);
}

bool load_intrinsics_yml(const std::string& path,
                         cv::Mat& K, cv::Mat& dist_coeffs,
                         cv::Size& image_size) {
    // Missing file: fail WITHOUT opening a cv::FileStorage — its open failure
    // prints an [ERROR] persistence line to stderr on every call, and the
    // undistort path is (re)applied to every Preprocessor instance whenever
    // the shared default is forwarded, so an uncalibrated system would spew
    // the same error at startup. A silent false lets the caller decide
    // whether a missing file is even noteworthy.
    if (path.empty() || !std::filesystem::exists(path)) return false;
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    cv::Mat k_in, dist_in;
    int w = 0, h = 0;
    fs["image_width"]  >> w;
    fs["image_height"] >> h;
    fs["camera_matrix"]           >> k_in;
    fs["distortion_coefficients"] >> dist_in;
    fs.release();
    if (k_in.empty() || dist_in.empty() || w <= 0 || h <= 0) return false;
    // Normalise to the canonical types used by IntrinsicCalibration /
    // cv::undistortPoints (CV_64F for K, CV_64F 1xN for dist).
    if (k_in.type() != CV_64F) {
        cv::Mat tmp;
        k_in.convertTo(tmp, CV_64F);
        k_in = tmp;
    }
    if (dist_in.type() != CV_64F) {
        cv::Mat tmp;
        dist_in.convertTo(tmp, CV_64F);
        dist_in = tmp;
    }
    if (k_in.rows != 3 || k_in.cols != 3) return false;
    K = k_in;
    dist_coeffs = dist_in;
    image_size = cv::Size(w, h);
    return true;
}

} // namespace gui_algo
