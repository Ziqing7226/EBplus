// gui/algo_bridge/backends/analytics_backends.cpp — frame producers + analyzers
// (design §3.4). Split from the former algo_backend.cpp monolith.
// Contains: EventToVideo, SensorSelfTest.

#include "algo_bridge/algo_backend.h"
#include "algo_bridge/backends/backend_common.h"

#include <cmath>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "algo/analytics/event_to_video.h"
#include "algo/analytics/sensor_self_test.h"

using namespace gui::backend_detail;

namespace gui {

class EventToVideoBackend final : public AlgoBackend {
    int sensor_w_{0}, sensor_h_{0};
    ProcessRegion roi_;
    // Current param values (re-applied after ROI rebuild so the fresh
    // EventToVideo instance inherits every setting, including E2VID model
    // path and post-processing params — otherwise ROI changes would silently
    // drop the E2VID configuration and fall back to the heuristic path).
    gui_algo::EventToVideo::Mode mode_{gui_algo::EventToVideo::Mode::E2VID};
    int output_fps_{30};
    // BardowVariational params.
    float window_ms_{50.0F};
    float delta_t_ms_{15.0F};
    float theta_{0.22F};
    int num_iterations_{100};
    float lambda1_{0.02F}, lambda2_{0.05F}, lambda3_{0.02F};
    float lambda4_{0.2F}, lambda5_{0.1F}, lambda6_{1.0F};
    float decay_tau_ms_{0.0F};
    // InteractingMaps params.
    float relaxation_step_{0.1F};
    int im_iterations_{50};
    float fov_deg_{60.0F};
    // E2VID params. Each DL mode (2=E2VID, 3=E2VID+, 4=FireNet+,
    // 5=HyperE2VID) has its own model path so all weight sets can be
    // installed side by side; switching modes swaps the loaded model.
    std::string model_paths_[4];
    int e2vid_num_bins_{5};
    int e2vid_device_{0};  ///< 0=Auto, 1=CPU, 2=GPU (§4.4.2-GPU)
    bool e2vid_auto_hdr_{false};
    float unsharp_amount_{0.3F};
    float unsharp_sigma_{1.0F};
    float bilateral_sigma_{0.0F};
    std::unique_ptr<gui_algo::EventToVideo> algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::Event> roi_events_;
    Preprocessor preproc_;
public:
    EventToVideoBackend(int w, int h) : sensor_w_(w), sensor_h_(h) {
        roi_.compute(sensor_w_, sensor_h_);
        preproc_.halve_coords_ = true;
        rebuild();
    }
    /// Index into model_paths_[] for DL modes (2..5), -1 for non-DL modes.
    static int dl_index(gui_algo::EventToVideo::Mode m) {
        return gui_algo::EventToVideo::mode_is_dl(m) ? static_cast<int>(m) - 2
                                                     : -1;
    }
    /// Resolves the user's device policy for the current mode. FireNet+
    /// executes WRONG on the OpenVINO GPU plugin (upstream 2026.4 iGPU
    /// issue: the output collapses to a near-constant image — std 0.005 vs
    /// 0.118 on CPU; the FP32 precision hint does not help; ORT-CPU and
    /// OV-CPU agree bit-for-bit) — Auto therefore resolves to CPU for that
    /// mode. An explicit GPU choice is still honored and visible in the
    /// status line (dev=gpu).
    int resolved_device() const {
        if (mode_ == gui_algo::EventToVideo::Mode::FireNetPlus &&
            e2vid_device_ == 0) {
            return 1;  // CPU
        }
        return e2vid_device_;
    }
    void rebuild() {
        const int aw = roi_.enabled ? roi_.rw : sensor_w_;
        const int ah = roi_.enabled ? roi_.rh : sensor_h_;
        preproc_.init(aw, ah);
        const int f = preproc_.factor();
        algo_ = std::make_unique<gui_algo::EventToVideo>(aw / f, ah / f, mode_, output_fps_);
        // BardowVariational
        algo_->set_window_ms(window_ms_);
        algo_->set_delta_t_ms(delta_t_ms_);
        algo_->set_theta(theta_);
        algo_->set_num_iterations(num_iterations_);
        algo_->set_lambda1(lambda1_);
        algo_->set_lambda2(lambda2_);
        algo_->set_lambda3(lambda3_);
        algo_->set_lambda4(lambda4_);
        algo_->set_lambda5(lambda5_);
        algo_->set_lambda6(lambda6_);
        algo_->set_decay_tau_ms(decay_tau_ms_);
        // InteractingMaps
        algo_->set_relaxation_step(relaxation_step_);
        algo_->set_im_iterations(im_iterations_);
        algo_->set_fov_deg(fov_deg_);
        // E2VID family: load the current mode's ONNX model if a path is set.
        // An empty path keeps the heuristic fallback (BUG-G9: comment
        // corrected — the model IS loaded here in rebuild(), not deferred).
        // Device policy goes BEFORE the model load so the fresh instance
        // selects its runtime on the first load (no double reload).
        algo_->set_e2v_device(resolved_device());
        const int di = dl_index(mode_);
        if (di >= 0) algo_->set_model_path(model_paths_[di]);
        algo_->set_e2vid_num_bins(e2vid_num_bins_);
        // Re-sync from the algo: when a model is loaded, set_num_bins ignores
        // the caller's value and uses model_num_bins_ (e2vid_inference.h).
        // Without this re-sync, e2vid_num_bins_ would stay at the stale
        // default/user value and get_param("num_bins") would return the
        // wrong value (BUG-N11).
        e2vid_num_bins_ = algo_->e2vid_num_bins();
        algo_->set_e2vid_auto_hdr(e2vid_auto_hdr_);
        // 1/4 downsample is handled by the shared Preprocessor (which halves
        // event coordinates before they reach the algo). The algo's internal
        // downsample flags must stay OFF to avoid double-halving.
        algo_->set_e2vid_downsample(false);
        algo_->set_downsample(false);
        algo_->set_unsharp_amount(unsharp_amount_);
        algo_->set_unsharp_sigma(unsharp_sigma_);
        algo_->set_bilateral_sigma(bilateral_sigma_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (preproc_.set_param(k, v)) {
            if (k == "preproc_downsample") rebuild();
            return;
        }
        // Snapshot ROI state BEFORE any param mutation so the rebuild-skip
        // logic below compares old vs. new against the true prior state.
        // Without this, roi_enabled toggling would read the NEW roi_.enabled
        // for both old_aw and new_aw, falsely skipping rebuild() and leaving
        // the algorithm with mismatched dimensions vs. incoming events (N2).
        const bool prev_roi_enabled = roi_.enabled;
        const int prev_roi_rw = roi_.rw;
        const int prev_roi_rh = roi_.rh;
        bool need_rebuild = false;
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 5) {
                mode_ = static_cast<gui_algo::EventToVideo::Mode>(m);
                if (algo_) {
                    algo_->set_mode(mode_);
                    // Switching between DL modes swaps the loaded model
                    // (each mode has its own weights; an empty path for the
                    // target mode unloads and falls back to heuristic).
                    const int di = dl_index(mode_);
                    if (di >= 0) {
                        // Re-resolve the device for the new mode BEFORE the
                        // load (FireNet+ Auto resolves to CPU).
                        algo_->set_e2v_device(resolved_device());
                        algo_->set_model_path(model_paths_[di]);
                        // Re-sync: the newly loaded model dictates num_bins
                        // (same contract as the model_path branch, BUG-N11).
                        e2vid_num_bins_ = algo_->e2vid_num_bins();
                    }
                }
            }
        } else if (k == "output_fps") {
            output_fps_ = to_i(v);
            if (algo_) algo_->set_output_fps(output_fps_);
        } else if (k == "window_ms") {
            window_ms_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_window_ms(window_ms_);
        } else if (k == "delta_t_ms") {
            delta_t_ms_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_delta_t_ms(delta_t_ms_);
        } else if (k == "theta") {
            theta_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_theta(theta_);
        } else if (k == "num_iterations") {
            num_iterations_ = to_i(v);
            if (algo_) algo_->set_num_iterations(num_iterations_);
        } else if (k == "lambda1") {
            lambda1_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda1(lambda1_);
        } else if (k == "lambda2") {
            lambda2_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda2(lambda2_);
        } else if (k == "lambda3") {
            lambda3_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda3(lambda3_);
        } else if (k == "lambda4") {
            lambda4_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda4(lambda4_);
        } else if (k == "lambda5") {
            lambda5_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda5(lambda5_);
        } else if (k == "lambda6") {
            lambda6_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda6(lambda6_);
        } else if (k == "decay_tau_ms") {
            decay_tau_ms_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_decay_tau_ms(decay_tau_ms_);
        } else if (k == "relaxation_step") {
            relaxation_step_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_relaxation_step(relaxation_step_);
        } else if (k == "im_iterations") {
            im_iterations_ = to_i(v);
            if (algo_) algo_->set_im_iterations(im_iterations_);
        } else if (k == "fov_deg") {
            fov_deg_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_fov_deg(fov_deg_);
        } else if (k == "model_path" || k == "e2vid_plus_model_path" ||
                   k == "firenet_plus_model_path" ||
                   k == "hypere2vid_model_path") {
            // Per-DL-mode weight paths. The key determines which slot is
            // written (mode 2 keeps the legacy "model_path" key for config
            // backward compatibility); only the CURRENT mode's path is
            // applied to the live algo.
            const int slot = (k == "model_path") ? 0
                             : (k == "e2vid_plus_model_path") ? 1
                             : (k == "firenet_plus_model_path") ? 2
                                                                : 3;
            model_paths_[slot] = v;
            if (algo_ && dl_index(mode_) == slot) {
                algo_->set_model_path(v);
                // num_bins is dictated by the loaded model (rpg_e2vid:
                // model.num_bins). Sync the persisted value so subsequent
                // ROI rebuilds keep the model's channel count.
                e2vid_num_bins_ = algo_->e2vid_num_bins();
            }
        } else if (k == "device") {
            e2vid_device_ = to_i(v);
            // set_device reloads the model from the cached path when the
            // policy changes, so the runtime selection takes effect.
            if (algo_) algo_->set_e2v_device(resolved_device());
        } else if (k == "num_bins") {
            e2vid_num_bins_ = to_i(v);
            if (algo_) {
                algo_->set_e2vid_num_bins(e2vid_num_bins_);
                // Re-sync: the algo ignores the caller's value when a model
                // is loaded (model_num_bins_ takes precedence). Without this,
                // e2vid_num_bins_ would retain the stale user/cache value
                // and get_param("num_bins") would return the wrong value
                // (BUG-N11).
                e2vid_num_bins_ = algo_->e2vid_num_bins();
            }
        } else if (k == "auto_hdr") {
            e2vid_auto_hdr_ = to_b(v);
            if (algo_) algo_->set_e2vid_auto_hdr(e2vid_auto_hdr_);
        } else if (k == "unsharp_amount") {
            unsharp_amount_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_unsharp_amount(unsharp_amount_);
        } else if (k == "unsharp_sigma") {
            unsharp_sigma_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_unsharp_sigma(unsharp_sigma_);
        } else if (k == "bilateral_sigma") {
            bilateral_sigma_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_bilateral_sigma(bilateral_sigma_);
        } else if (k == "downsample") {
            // Backward compat: the old per-algo "downsample" key is forwarded
            // to the shared preproc stage (preproc_downsample). The per-algo
            // flag stays OFF to avoid double-halving (BUG-R2). Must rebuild
            // directly (not via need_rebuild + N2 dimension check) because
            // the preproc factor change affects algorithm dimensions (aw/f,
            // ah/f in rebuild()) regardless of ROI dimensions — the N2 check
            // only compares ROI dims and would falsely skip the rebuild.
            // Skip rebuild if the value didn't change (avoids unnecessary
            // ONNX reload on duplicate set_param calls).
            const std::string prev = preproc_.get_param("preproc_downsample");
            preproc_.set_param("preproc_downsample", v);
            if (prev != v) rebuild();
            return;
        }
        // Phase 2.6: roi_* keys intentionally not handled (legacy per-backend
        // ROI deleted; unified ROI at the source).
        if (need_rebuild) {
            // Only rebuild (which reloads the ONNX model) when the effective
            // dimensions actually change. ROI position changes (x, y) don't
            // affect the algorithm dimensions — they only shift which events
            // are cropped — so skip the expensive rebuild (N2).
            // Use the snapshot taken at entry — roi_.enabled may have been
            // toggled by this very call, so reading it now would compare the
            // NEW state against itself and falsely skip rebuild().
            const int old_aw = prev_roi_enabled ? prev_roi_rw : sensor_w_;
            const int old_ah = prev_roi_enabled ? prev_roi_rh : sensor_h_;
            roi_.compute(sensor_w_, sensor_h_);
            const int new_aw = roi_.enabled ? roi_.rw : sensor_w_;
            const int new_ah = roi_.enabled ? roi_.rh : sensor_h_;
            if (new_aw != old_aw || new_ah != old_ah) {
                rebuild();
            }
        }
    }
    std::string get_param(const std::string& k) const override {
        auto pp = preproc_.get_param(k); if (!pp.empty()) return pp;
        if (k == "mode") return from_i(static_cast<int>(mode_));
        if (k == "output_fps") return from_i(output_fps_);
        if (k == "window_ms") return from_d(window_ms_);
        if (k == "delta_t_ms") return from_d(delta_t_ms_);
        if (k == "theta") return from_d(theta_);
        if (k == "num_iterations") return from_i(num_iterations_);
        if (k == "lambda1") return from_d(lambda1_);
        if (k == "lambda2") return from_d(lambda2_);
        if (k == "lambda3") return from_d(lambda3_);
        if (k == "lambda4") return from_d(lambda4_);
        if (k == "lambda5") return from_d(lambda5_);
        if (k == "lambda6") return from_d(lambda6_);
        if (k == "decay_tau_ms") return from_d(decay_tau_ms_);
        if (k == "relaxation_step") return from_d(relaxation_step_);
        if (k == "im_iterations") return from_i(im_iterations_);
        if (k == "fov_deg") return from_d(fov_deg_);
        if (k == "model_path") return model_paths_[0];
        if (k == "e2vid_plus_model_path") return model_paths_[1];
        if (k == "firenet_plus_model_path") return model_paths_[2];
        if (k == "hypere2vid_model_path") return model_paths_[3];
        if (k == "model_loaded") {
            // Pseudo-param (not registered) for the panel's one-shot error
            // hint (§五-H1): only meaningful in DL modes; empty = N/A.
            return (gui_algo::EventToVideo::mode_is_dl(mode_) && algo_)
                       ? from_b(algo_->e2vid_model_loaded()) : std::string{};
        }
        if (k == "num_bins") return from_i(e2vid_num_bins_);
        if (k == "device") return from_i(e2vid_device_);
        if (k == "auto_hdr") return from_b(e2vid_auto_hdr_);
        if (k == "unsharp_amount") return from_d(unsharp_amount_);
        if (k == "unsharp_sigma") return from_d(unsharp_sigma_);
        if (k == "bilateral_sigma") return from_d(bilateral_sigma_);
        // Phase 2.6: roi_* keys intentionally not handled.
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        const auto* ev = as_events(passthrough_.data());
        std::size_t n = passthrough_.size();
        if (roi_.enabled && roi_.rw > 0 && roi_.rh > 0) {
            crop_to_roi(ev, n, roi_, &preproc_, roi_events_);
            ev = roi_events_.data();
            n = roi_events_.size();
        } else if (preproc_.active() && n > 0) {
            auto [p, m] = preproc_.apply(ev, n);
            roi_events_.assign(p, p + m);
            ev = roi_events_.data();
            n = m;
        }
        algo_->process(ev, n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.has_frame = true;
        cv::Mat frame = algo_->get_frame();
        const int f = preproc_.factor();
        if (f > 1 && !frame.empty()) {
            const int aw = roi_.enabled ? roi_.rw : sensor_w_;
            const int ah = roi_.enabled ? roi_.rh : sensor_h_;
            cv::resize(frame, frame, cv::Size(aw, ah), 0, 0, cv::INTER_NEAREST);
        }
        r.frame = frame.clone();
        r.status = "e2v: " + std::to_string(algo_->width()) + "x" +
                   std::to_string(algo_->height()) +
                   (roi_.enabled ? " (ROI)" : " (full)");
        // §五-H1: a failed ONNX load silently falls back to the heuristic
        // path — the status line must show which reconstruction is running
        // so users don't mistake heuristic output for "E2VID quality".
        // dev= shows the active neural runtime (gpu = OpenVINO GPU plugin,
        // cpu = ONNX Runtime) — the Auto policy can silently pick either.
        if (gui_algo::EventToVideo::mode_is_dl(algo_->mode())) {
            if (algo_->e2vid_model_loaded()) {
                r.status += " model=loaded dev=" + algo_->e2v_active_runtime();
            } else {
                r.status += " model=heuristic";
            }
        }
        return r;
    }
    void reset() override {
        if (algo_) algo_->reset();
        passthrough_.clear();
        roi_events_.clear();
    }
    void set_sensor_dimensions(int w, int h) override {
        sensor_w_ = w;
        sensor_h_ = h;
        roi_.compute(sensor_w_, sensor_h_);
        rebuild();
    }
};


/// SensorSelfTest backend — per-pixel refractory-period heatmap + bad-pixel
/// detection (design §4.4.8). Processes the FULL sensor (no ROI, no
/// preprocessing) because the self-test must cover every pixel. Produces a
/// CV_8UC3 heatmap frame: red = never triggered (suspected bad pixel),
/// grayscale = exponential mapping of min inter-event interval (brighter =
/// shorter refractory period).
class SensorSelfTestBackend final : public AlgoBackend {
    int sensor_w_{0}, sensor_h_{0};
    std::unique_ptr<gui_algo::SensorSelfTest> algo_;
    std::vector<Metavision::EventCD> passthrough_;
    // Flag set by the close handler via set_param("__final_report", "1")
    // so the next pull_result() returns the full report in r.status for
    // the close dialog. Cleared after one use. The report itself is
    // computed fresh at that moment (O(N log N) — too expensive per frame).
    bool final_report_requested_{false};
public:
    explicit SensorSelfTestBackend(int w, int h) : sensor_w_(w), sensor_h_(h) {
        rebuild();
    }
    void rebuild() {
        algo_ = std::make_unique<gui_algo::SensorSelfTest>(sensor_w_, sensor_h_);
        passthrough_.clear();
        final_report_requested_ = false;
    }
    void set_param(const std::string& k, const std::string& v) override {
        // No user-tunable parameters — the self-test is fully automatic.
        // The close handler uses "__final_report" to request the full report.
        if (k == "__final_report" && v == "1") {
            final_report_requested_ = true;
        }
    }
    std::string get_param(const std::string& /*k*/) const override {
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_->process(as_events(passthrough_.data()), passthrough_.size());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.has_frame = true;
        // render() returns a freshly allocated Mat — no clone needed.
        r.frame = algo_->render();
        // Close handler requested the full report — compute it fresh and
        // return it in r.status for the dialog. This is a one-shot flag.
        if (final_report_requested_) {
            final_report_requested_ = false;
            r.status = algo_->report();
            return r;
        }
        // Per-frame status: concise one-line summary (not the full report,
        // which would flood the AlgoWindow status label every frame).
        // Computed on the pull side on purpose: in live mode the push rate
        // (per USB batch, ~hundreds/s) is far higher than the display rate,
        // so caching this on the push side would burn MORE CPU on the
        // hotter thread.
        const auto c = algo_->count_pixels();
        r.status = "Triggered: " + std::to_string(c.triggered) + "/" +
                   std::to_string(c.total) + " | Measured: " +
                   std::to_string(c.measured) + " | Bad: " +
                   std::to_string(c.bad);
        return r;
    }
    void reset() override {
        if (algo_) algo_->reset();
        passthrough_.clear();
        final_report_requested_ = false;
    }
    void set_sensor_dimensions(int w, int h) override {
        sensor_w_ = w;
        sensor_h_ = h;
        rebuild();
    }
};

// --- Per-category factory (called by create_algo_backend in backend_factory.cpp)
std::unique_ptr<AlgoBackend> create_analytics_backend(const std::string& name,
                                          int width, int height) {
    if (name == "event_to_video")              return std::make_unique<EventToVideoBackend>(width, height);
    if (name == "sensor_self_test")            return std::make_unique<SensorSelfTestBackend>(width, height);
    return nullptr;
}

} // namespace gui
