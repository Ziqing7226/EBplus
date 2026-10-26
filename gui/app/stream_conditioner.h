// gui/app/stream_conditioner.h — the single shared event-conditioning stage.
//
// One instance per event SOURCE (live: CameraController's CD callback; file:
// FileFrameGenerator::render_frame) applies the whole conditioning pipeline
// ONCE per batch and hands every consumer (display frame renderers, processed
// recording, algorithm instances) the SAME output span. Before this, the
// FilterChain ran twice in live mode and the noise filter / downsample /
// undistort ran once per consumer (display + every algorithm instance) on the
// same events.
//
// Pipeline (order fixed by design 2026-08-19):
//   1. unified ROI crop      — highest priority. ROI mode: keep+shift to
//                              ROI-relative coordinates (live hardware I_ROI
//                              already dropped outside events; the bounds test
//                              is a no-op there). RONI: keep outside the rect,
//                              coordinates stay ABSOLUTE (the kept region is
//                              not a rectangle, ROI-relative geometry is
//                              undefined — mirrors AlgoBridge's RONI
//                              pass-through).
//   2. FilterChain value     — polarity filter/invert, before the noise
//      stages                 filter so polarity matching sees the filtered
//                              polarity.
//   3. noise filter          — the expensive stage, now computed once; maps
//                              sized to the stage-1 output geometry.
//   4. 1/4 downsample thin   — one parity decision in the canonical
//                              coordinate system. Coordinate HALVING stays
//                              per-consumer (halving backends shift coords
//                              themselves; the display never halves).
//   5. undistort             — single LUT at the stage-1 geometry (K adjusted
//                              by the ROI origin; factor 1 — halving is
//                              downstream).
//   6. FilterChain geometry  — flips mirror within the stage-1 output
//      stages                 geometry (ROI dims in ROI mode, sensor dims in
//                              RONI — geometry follows the coordinate system).
//                              AFTER undistort: a mirrored frame must be
//                              undistorted with the physical K first (exact
//                              only when the principal point is centered).
//
// Consumers get the canonical ROI-relative stream; AlgoInstance no longer
// crops (set_unified_roi only resizes backends).
//
// Thread safety: mutated from the GUI thread (init / set_roi / set_param),
// consumed on the SDK data thread (live) or the GUI timer thread (file
// render). An internal mutex serialises apply() against mutations; the
// FilterChain has its own lock (never taken while holding ours in reverse
// order — the chain knows nothing of this class).

#ifndef GUI_APP_STREAM_CONDITIONER_H
#define GUI_APP_STREAM_CONDITIONER_H

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/events/event_cd.h>

#include "algo/cv/noise_filter.h"
#include "algo_bridge/filter_chain.h"

namespace gui {

class StreamConditioner {
public:
    /// @brief Sets the source geometry (sensor or file dimensions). Call on
    /// camera connect / file load. Rebuilds geometry-dependent state.
    void init(int sensor_w, int sensor_h);

    /// @brief Attaches the FilterChain (Display Transform). May be null.
    /// The chain object is shared; its geometry stages are re-geometried to
    /// this conditioner's output dims at every geometry change (init /
    /// set_roi) and when the chain is attached — never on the batch path.
    void set_filter_chain(FilterChain* fc);

    /// @brief Unified ROI. ROI mode (roni=false): crop+shift to ROI-relative.
    /// RONI (roni=true): drop inside-rect, keep absolute coordinates.
    void set_roi(bool enabled, int x0, int y0, int x1, int y1, bool roni);

    /// @brief preproc_* parameter (same keys the panel emits). Returns false
    /// for non-preproc keys.
    bool set_param(const std::string& key, const std::string& value);
    /// @brief Current preproc_* value ("" if unknown). Round-trips params so
    /// configs capture the global conditioning state.
    std::string get_param(const std::string& key) const;

    /// @brief Clears the noise filter's temporal state (file seek/loop,
    /// source restart). Geometry-dependent state (LUT) is kept.
    void reset_temporal();

    /// @brief Resets every per-source conditioning decision taken through
    /// set_param / set_roi (noise filter + params, 1/4 thin, undistort,
    /// ROI rect). Called by CameraController::teardown() so a newly
    /// connected camera starts from defaults instead of inheriting the
    /// previous camera's settings. GUI thread only.
    void reset_session_state();

    /// @brief True when any stage (ROI / chain / filter / thin / undistort)
    /// would modify the stream.
    bool active() const;

    /// @brief Conditions one batch. The returned span points into internal
    /// buffers owned by this object — valid until the next apply() call on
    /// the same instance, and must not be mutated by callers.
    std::pair<const Metavision::EventCD*, std::size_t>
    apply(const Metavision::EventCD* begin, const Metavision::EventCD* end);

    /// @brief Output geometry (what consumers should assume): ROI dims in
    /// ROI mode, source dims otherwise.
    int out_width() const;
    int out_height() const;

private:
    // Guarded by mutex_ -----------------------------------------------------
    int sensor_w_{0}, sensor_h_{0};
    bool roi_enabled_{false};
    bool roi_roni_{false};
    int roi_x0_{0}, roi_y0_{0}, roi_x1_{0}, roi_y1_{0};

    // Noise filter (stage 3) — one instance, maps at the output geometry.
    bool filter_enabled_{false};
    gui_algo::NoiseFilter::Mode filter_mode_{gui_algo::NoiseFilter::Mode::KNoise};
    std::unique_ptr<gui_algo::NoiseFilter> filter_;
    std::unordered_map<std::string, std::string> filter_params_;

    bool downsample_enabled_{false};  // stage 4 (thin only; halving downstream)

    // Undistort (stage 5) — moved from the per-instance Preprocessor; factor
    // is always 1 (halving happens per consumer, after this stage).
    bool undistort_enabled_{false};
    std::string undistort_path_;
    cv::Mat undistort_K_;
    cv::Mat undistort_dist_;
    bool undistort_lut_valid_{false};
    bool undistort_lut_failed_{false};
    int undistort_eff_w_{0}, undistort_eff_h_{0};
    std::vector<cv::Point2f> undistort_lut_;

    mutable std::mutex mutex_;

    // Scratch buffers — only touched inside apply() (serialised by mutex_).
    std::vector<Metavision::EventCD> roi_buf_;
    std::vector<Metavision::EventCD> fc_buf_;
    std::vector<gui_algo::Event> work_buf_;
    std::vector<Metavision::EventCD> geo_buf_;

    FilterChain* fc_{nullptr};  // not owned; has its own lock

    /// Pushes the current output geometry into fc_->set_geometry (flip
    /// mirror axes). Called only from geometry MUTATION points (init /
    /// set_roi / set_filter_chain, all GUI thread) — apply() relies on it
    /// being current and never touches the chain geometry.
    void push_geometry_locked();

    void rebuild_filter_locked(int w, int h);
    void rebuild_undistort_lut_locked(int w, int h);
};

} // namespace gui

#endif // GUI_APP_STREAM_CONDITIONER_H
