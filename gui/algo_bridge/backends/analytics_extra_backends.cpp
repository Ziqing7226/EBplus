// gui/algo_bridge/backends/analytics_extra_backends.cpp — FreqDetector,
// FrequencyMap (design §3.4). Split from the former algo_backend.cpp monolith.

#include "algo_bridge/algo_backend.h"
#include "algo_bridge/backends/backend_common.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "algo/analytics/freq_detector.h"
#include "algo/cv/frequency_map.h"

using namespace gui::backend_detail;

namespace gui {

// §4.4.6 Auto Bias Controller: removed from the algorithm pipeline
// (2026-08-21) — camera-level feature in CameraController now.

// ===========================================================================
// Group F: Event-vector (process returns vector<Event>)
// ===========================================================================


// ===========================================================================
// Group E: Analyzers (process events, produce text/point overlay)
// ===========================================================================

/// FreqDetector backend — detected light sources as overlay boxes + frequency
/// labels drawn on the main display (design §4.4.7). No internal ROI filter:
/// with the unified ROI active the incoming stream is already ROI-relative,
/// and the OverlayStrategy shifts coordinates by the ROI origin, so a second
/// absolute-rect filter would be inconsistent. During the analyzer's
/// initialization phase the result carries a centered hint explaining the
/// feature and the keep-still requirement instead of sources.
class FreqDetectorBackend final : public AlgoBackend {
    gui_algo::FreqDetector algo_;
    std::vector<gui_algo::LightSource> last_;
    // Persisted so set_sensor_dimensions() rebuilds keep the user's settings
    // instead of reverting to ctor defaults.
    float f_min_{100.0f};
    float f_max_{10000.0f};
    float bin_dt_us_{50.0f};
    int heatmap_threshold_{50};
    int min_events_{3};
    int region_radius_{1};
    float peak_alpha_{5.0f};
    float first_analysis_s_{5.0f};
    float max_duration_s_{20.0f};
    float update_interval_s_{2.0f};
public:
    explicit FreqDetectorBackend(int w, int h) : algo_(w, h) {}
    void set_param(const std::string& k, const std::string& v) override {
        if (k == "fmin") { f_min_ = static_cast<float>(to_d(v)); algo_.set_f_min(f_min_); }
        else if (k == "fmax") { f_max_ = static_cast<float>(to_d(v)); algo_.set_f_max(f_max_); }
        else if (k == "bin_dt_us") { bin_dt_us_ = static_cast<float>(to_d(v)); algo_.set_bin_dt_us(bin_dt_us_); }
        else if (k == "heatmap_threshold") { heatmap_threshold_ = to_i(v); algo_.set_heatmap_threshold(heatmap_threshold_); }
        else if (k == "min_events") { min_events_ = to_i(v); algo_.set_min_cc_area(min_events_); }
        else if (k == "region_radius") { region_radius_ = to_i(v); algo_.set_region_radius(region_radius_); }
        else if (k == "peak_alpha") { peak_alpha_ = static_cast<float>(to_d(v)); algo_.set_peak_alpha(peak_alpha_); }
        else if (k == "first_analysis") { first_analysis_s_ = static_cast<float>(to_d(v)); algo_.set_first_analysis_s(first_analysis_s_); }
        else if (k == "max_duration") { max_duration_s_ = static_cast<float>(to_d(v)); algo_.set_max_duration_s(max_duration_s_); }
        else if (k == "update_interval_s") { update_interval_s_ = static_cast<float>(to_d(v)); algo_.set_update_interval_s(update_interval_s_); }
    }
    std::string get_param(const std::string& k) const override {
        if (k == "fmin") return from_d(algo_.f_min());
        if (k == "fmax") return from_d(algo_.f_max());
        if (k == "bin_dt_us") return from_d(algo_.bin_dt_us());
        if (k == "heatmap_threshold") return from_i(algo_.heatmap_threshold());
        if (k == "min_events") return from_i(algo_.min_cc_area());
        if (k == "region_radius") return from_i(algo_.region_radius());
        if (k == "peak_alpha") return from_d(algo_.peak_alpha());
        if (k == "first_analysis") return from_d(algo_.first_analysis_s());
        if (k == "max_duration") return from_d(algo_.max_duration_s());
        if (k == "update_interval_s") return from_d(algo_.update_interval_s());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        // Zero-copy feed: gui_algo::Event is layout-compatible with
        // Metavision::EventCD (static_assert in as_events).
        algo_.process(as_events(b), static_cast<std::size_t>(e - b));
        if (algo_.should_analyze()) last_ = algo_.analyze();
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        if (algo_.phase() == gui_algo::FreqDetector::Phase::Initializing) {
            r.hint = "LED frequency measurement (spectral method)\n"
                     "Keep the camera and the light source relatively still (" +
                     std::to_string(
                         static_cast<int>(std::ceil(algo_.init_remaining_s()))) +
                     " s)";
            return r;
        }
        int id = 0;
        for (const auto& src : last_) {
            OverlayBox box;
            box.x = src.u0; box.y = src.v0;
            box.w = std::max(src.w, 6); box.h = std::max(src.h, 6);
            box.id = id++;
            r.boxes.push_back(box);
            OverlayText t;
            t.x = src.u0 + box.w + 4;
            t.y = src.v0 + src.h / 2;
            t.text = std::to_string(static_cast<int>(std::lround(src.blink_freq_hz))) + " Hz";
            r.texts.push_back(t);
        }
        return r;
    }
    void reset() override { algo_.reset(); last_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        // Rebuild at the new dimensions and re-apply the persisted settings.
        algo_ = gui_algo::FreqDetector(w, h);
        algo_.set_f_min(f_min_);
        algo_.set_f_max(f_max_);
        algo_.set_bin_dt_us(bin_dt_us_);
        algo_.set_heatmap_threshold(heatmap_threshold_);
        algo_.set_min_cc_area(min_events_);
        algo_.set_region_radius(region_radius_);
        algo_.set_peak_alpha(peak_alpha_);
        algo_.set_first_analysis_s(first_analysis_s_);
        algo_.set_max_duration_s(max_duration_s_);
        algo_.set_update_interval_s(update_interval_s_);
        last_.clear();
    }
};


/// FrequencyMapBackend — per-pixel flicker frequency map (Standalone frame:
/// Jet colormap) + clustered light sources (circles + Hz labels).
/// analyze() (a full-sensor BFS cluster sweep) is throttled to ~4 Hz so the
/// GUI thread is not starved; between runs the last frame + sources are reused.
class FrequencyMapBackend final : public AlgoBackend {
    gui_algo::FrequencyMap algo_;
    gui_algo::FrequencyMapParams params_;  ///< Persisted across set_sensor_dimensions() rebuilds.
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    /// Minimum interval between analyze() runs (ms).
    int update_interval_ms_{250};
    std::chrono::steady_clock::time_point last_analyze_{};
    cv::Mat cached_frame_;
    std::vector<gui_algo::FlickerSource> cached_sources_;
public:
    FrequencyMapBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "update_interval_ms") update_interval_ms_ = std::max(50, to_i(v));
        else if (k == "filter_length") params_.frequency_filter_length = to_i(v);
        else if (k == "period_diff_thresh_us") params_.period_diff_thresh_us = to_i(v);
        else if (k == "min_freq_hz") params_.min_freq_hz = static_cast<float>(to_d(v));
        else if (k == "max_freq_hz") params_.max_freq_hz = static_cast<float>(to_d(v));
        else if (k == "max_freq_diff") params_.max_cluster_frequency_diff = static_cast<float>(to_d(v));
        else if (k == "min_cluster_size") params_.min_cluster_size = to_i(v);
        else return;
        algo_.set_params(params_);
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "update_interval_ms") return from_i(update_interval_ms_);
        if (k == "filter_length") return from_i(params_.frequency_filter_length);
        if (k == "period_diff_thresh_us") return from_i(static_cast<int>(params_.period_diff_thresh_us));
        if (k == "min_freq_hz") return from_d(params_.min_freq_hz);
        if (k == "max_freq_hz") return from_d(params_.max_freq_hz);
        if (k == "max_freq_diff") return from_d(params_.max_cluster_frequency_diff);
        if (k == "min_cluster_size") return from_i(params_.min_cluster_size);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        algo_.process(ev, ev + n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_analyze_).count();
        if (cached_frame_.empty() || elapsed_ms >= update_interval_ms_) {
            algo_.analyze();
            last_analyze_ = now;
            // Frequency map → Jet heatmap at sensor resolution (the
            // EventDisplayWidget texture uses GL linear filtering, so the
            // widget upscales smoothly — a CPU-side upscale would only add
            // a per-update multi-megapixel resize/copy). Fixed-band
            // normalization: the confirmed-frequency range [min_freq_hz,
            // max_freq_hz] maps to the full colormap (higher = redder/
            // hotter); unconfirmed pixels (0) are blacked out after the
            // LUT so the background reads as empty rather than Jet's
            // dark-blue tail.
            const cv::Mat f = algo_.frequency_hz();
            cv::Mat u8 = cv::Mat::zeros(f.size(), CV_8UC1);
            const float lo = params_.min_freq_hz;
            const float span = std::max(1.0f, params_.max_freq_hz - lo);
            for (int y = 0; y < f.rows; ++y) {
                const float* frow = f.ptr<float>(y);
                std::uint8_t* orow = u8.ptr<std::uint8_t>(y);
                for (int x = 0; x < f.cols; ++x) {
                    if (frow[x] <= 0.0f) continue;  // unconfirmed → black
                    float v = (frow[x] - lo) / span;
                    v = std::min(1.0f, std::max(0.0f, v));
                    orow[x] = static_cast<std::uint8_t>(24.0f + v * 231.0f);
                }
            }
            cv::applyColorMap(u8, cached_frame_, cv::COLORMAP_JET);
            cv::Mat unconfirmed;
            cv::compare(u8, 0, unconfirmed, cv::CMP_EQ);
            cached_frame_.setTo(cv::Scalar(0, 0, 0), unconfirmed);
            // Sources are annotated directly into the frame (white circles +
            // shadowed text) — nothing goes to the sidebar text list.
            cached_sources_ = algo_.sources();
            for (const auto& src : cached_sources_) {
                const cv::Point c(static_cast<int>(std::lround(src.x)),
                                  static_cast<int>(std::lround(src.y)));
                const int rad = std::max(3, static_cast<int>(std::lround(src.radius_px)));
                cv::circle(cached_frame_, c, rad, cv::Scalar(255, 255, 255), 1);
                const std::string label =
                    std::to_string(static_cast<int>(std::lround(src.frequency_hz))) + " Hz";
                const cv::Point tp(c.x + rad + 3, c.y + 4);
                cv::putText(cached_frame_, label, tp + cv::Point(1, 1),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
                cv::putText(cached_frame_, label, tp,
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            }
        }
        r.frame = cached_frame_;
        r.has_frame = true;
        return r;
    }
    void reset() override {
        algo_.reset();
        passthrough_.clear();
        roi_buf_.clear();
        cached_frame_ = cv::Mat();
        cached_sources_.clear();
        last_analyze_ = {};
    }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        algo_ = gui_algo::FrequencyMap(w, h);
        algo_.set_params(params_);
        reset();
    }
};


// --- Per-category factory (called by create_algo_backend in backend_factory.cpp)
std::unique_ptr<AlgoBackend> create_analytics_extra_backend(const std::string& name,
                                          int width, int height) {
    if (name == "freq_detector")               return std::make_unique<FreqDetectorBackend>(width, height);
    if (name == "frequency_map")               return std::make_unique<FrequencyMapBackend>(width, height);
    return nullptr;
}

} // namespace gui
