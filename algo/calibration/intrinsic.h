// algo/calibration/intrinsic.h — intrinsic camera calibration (Zhang method).
//
// Detection: a BLINKING CHESSBOARD is displayed on screen (alternating with a
// blank frame at 10 ms). Pixels on black squares toggle dark↔light every cycle
// and fire both polarities; over a capture window covering ≥ one blink cycle
// they form a filled checkerboard in the binary "blink frame"
// (see blinking_detect.h), which is fed to cv::findChessboardCorners.
//
// Solving: TWO-PASS calibration. Pass 1 fixes K3 + aspect ratio for a stable
// initial fit and computes per-view reprojection errors; views whose error
// exceeds mean + outlier_ths·std are removed; pass 2 re-runs calibrateCamera
// on the kept views with K3 free. An optional fronto-parallel refinement pass
// (undistort → homography → re-detect subpix) is planned as a later phase.
//
// The algorithm is frame-driven: the GUI wizard feeds accumulated binary blink
// frames via add_frame()/detect_only()+accept(). Detection runs synchronously
// on the caller's thread; run() performs the (two-pass) bundle adjustment.

#ifndef GUI_ALGO_CALIBRATION_INTRINSIC_H
#define GUI_ALGO_CALIBRATION_INTRINSIC_H

#include <cstddef>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace gui_algo {

/// @brief Pattern type for calibration board detection.
enum class CalibrationPattern {
    Chessboard,     ///< Blinking chessboard (inner corners); the wizard's default.
    CircleGrid      ///< Regular circle grid (static detection, kept for reuse).
};

/// @brief Result of a single frame's corner detection.
struct DetectionResult {
    bool found{false};
    cv::Mat image;                    ///< Annotated copy (for wizard preview)
    std::vector<cv::Point2f> points;  ///< Detected corner coordinates
};

/// @brief Result of the full intrinsic calibration (two-pass).
struct IntrinsicResult {
    bool ok{false};
    double rms{0.0};                   ///< Overall RMS reprojection error (kept views)
    cv::Mat K;                         ///< 3x3 camera matrix
    cv::Mat dist_coeffs;               ///< 1x5 distortion [k1,k2,p1,p2,k3]
    std::vector<cv::Mat> rvecs;        ///< Per-KEPT-view rotation vectors
    std::vector<cv::Mat> tvecs;        ///< Per-KEPT-view translation vectors
    std::size_t frames_used{0};        ///< == kept_frames
    std::size_t kept_frames{0};        ///< Views surviving outlier rejection
    std::size_t removed_frames{0};     ///< Views rejected by outlier rule
    std::vector<bool> selected_views;  ///< Per ORIGINAL input view (survived?)
    std::vector<double> per_view_rms;  ///< RMS per KEPT view (parallel to rvecs)
    std::string error;                 ///< Empty when ok==true
};

/// @brief Intrinsic calibration accumulator.
class IntrinsicCalibration {
public:
    IntrinsicCalibration();
    ~IntrinsicCalibration();

    /// @brief Configures the board geometry. Must be called before add_frame().
    /// Changing the pattern type or corner count clears accumulated
    /// observations (they would feed cv::calibrateCamera inconsistent point
    /// sets otherwise). The square size is a pure world-scale factor — pixel
    /// observations stay valid when it changes, so it never clears anything;
    /// run() rebuilds the object grid from the CURRENT value, which is what
    /// the final calibration uses.
    void set_pattern(CalibrationPattern pattern,
                     int cols, int rows,
                     float square_size_mm);

    /// @brief Sets the outlier-rejection threshold: views with per-view RMS
    /// reprojection error above mean + ths·std are dropped before pass 2.
    /// ths <= 0 keeps all views. Default 2.0.
    void set_outlier_threshold(double ths);

    /// @brief Attempts to detect the calibration pattern in @p frame and, on
    /// success, accumulates the observation. Convenience wrapper around
    /// detect_only() + accept().
    DetectionResult add_frame(const cv::Mat& frame, bool annotate = true);

    /// @brief Runs pattern detection only — does NOT accumulate. Lets the
    /// caller reject a frame (duplicate pose, insufficient coverage, …)
    /// before committing it via accept(). Records image_size_ on first call.
    DetectionResult detect_only(const cv::Mat& frame, bool annotate = true);

    /// @brief Commits an already-detected point set as a calibration
    /// observation. @p points must come from a successful detect_only() on a
    /// frame of the configured geometry.
    void accept(const std::vector<cv::Point2f>& points);

    /// Adds stored full-resolution corners without running image detection.
    bool accept_observation(const std::vector<cv::Point2f>& points, cv::Size image_size);

    /// @brief Removes the most recently accepted observation (image + object
    /// points). Used by the wizard's "Delete this capture" button. No-op if empty.
    void remove_last_frame();

    /// @brief Returns true if @p points match (within @p threshold_px mean
    /// Euclidean distance) any already-accepted observation.
    bool is_duplicate_pose(const std::vector<cv::Point2f>& points,
                           double threshold_px) const;

    /// @brief Runs the two-pass calibration (see header comment).
    IntrinsicResult run();

    /// @brief Discards all collected observations.
    void reset();

    std::size_t frame_count() const { return image_points_.size(); }
    cv::Size image_size() const { return image_size_; }

    /// @brief The object-point grid for the currently configured geometry
    /// (read-only). Exposed so the wizard/tests can verify the configured
    /// board matches the displayed pattern.
    std::vector<cv::Point3f> object_grid() const { return make_object_grid(); }

private:
    CalibrationPattern pattern_{CalibrationPattern::Chessboard};
    cv::Size board_size_{0, 0};    ///< Inner-corner count for chessboard (== OpenCV patternSize), (cols, rows) for circles
    float square_size_mm_{1.0f};
    double outlier_ths_{2.0};
    cv::Size image_size_{0, 0};

    std::vector<std::vector<cv::Point2f>> image_points_;

    std::vector<cv::Point3f> make_object_grid() const;
};

/// @brief Loads intrinsic calibration (K, distCoeffs, image_size) from a YAML
///        file written by CalibrationWizard::on_intrinsic_save() or any
///        OpenCV-compatible YAML with the same keys:
///          image_width, image_height, camera_matrix, distortion_coefficients
///        (rms optional). Used by the calibration wizard (round-trip) and by
///        the Preprocessor undistort stage (consumes the wizard's output).
/// @return true on success; on failure leaves outputs unchanged.
bool load_intrinsics_yml(const std::string& path,
                         cv::Mat& K, cv::Mat& dist_coeffs,
                         cv::Size& image_size);

} // namespace gui_algo

#endif // GUI_ALGO_CALIBRATION_INTRINSIC_H
