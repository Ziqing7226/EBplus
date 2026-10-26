// gui/app/stream_conditioner.cpp — see stream_conditioner.h for the pipeline
// description. The undistort LUT build is compiled here so the header stays
// cheap; it moved from the per-instance Preprocessor, which no longer
// undistorts.

#include "stream_conditioner.h"

#include <filesystem>
#include <cstdio>

// OpenCV 5.0 moved undistortPoints from calib3d to the geometry module —
// include the version-correct set (see opencv5_compat.h).
#include "algo/common/opencv5_compat.h"

#include "algo/common/event.h"
#include "algo/calibration/intrinsic.h"
#include "algo_bridge/backends/backend_common.h"

namespace gui {

using backend_detail::to_i;
using backend_detail::to_b;
using backend_detail::from_b;
using backend_detail::apply_noise_filter_param;
using backend_detail::get_noise_filter_param;

void StreamConditioner::set_filter_chain(FilterChain* fc) {
    std::lock_guard<std::mutex> lk(mutex_);
    fc_ = fc;
    // A freshly attached chain needs this conditioner's output geometry for
    // its flip stages before the first batch runs.
    push_geometry_locked();
}

void StreamConditioner::push_geometry_locked() {
    if (!fc_) return;
    const int w = (roi_enabled_ && !roi_roni_) ? roi_x1_ - roi_x0_ : sensor_w_;
    const int h = (roi_enabled_ && !roi_roni_) ? roi_y1_ - roi_y0_ : sensor_h_;
    if (w > 0 && h > 0) fc_->set_geometry(w, h);
}

void StreamConditioner::init(int sensor_w, int sensor_h) {
    std::lock_guard<std::mutex> lk(mutex_);
    sensor_w_ = sensor_w;
    sensor_h_ = sensor_h;
    undistort_lut_valid_ = false;  // geometry changed → LUT needs rebuild
    undistort_lut_failed_ = false; // re-arm: new geometry gets one attempt
    const int w = (roi_enabled_ && !roi_roni_) ? roi_x1_ - roi_x0_ : sensor_w_;
    const int h = (roi_enabled_ && !roi_roni_) ? roi_y1_ - roi_y0_ : sensor_h_;
    rebuild_filter_locked(w, h);
    push_geometry_locked();
}

void StreamConditioner::set_roi(bool enabled, int x0, int y0, int x1, int y1,
                                bool roni) {
    std::lock_guard<std::mutex> lk(mutex_);
    roi_enabled_ = enabled;
    roi_roni_ = roni;
    roi_x0_ = x0;
    roi_y0_ = y0;
    roi_x1_ = x1;
    roi_y1_ = y1;
    undistort_lut_valid_ = false;
    undistort_lut_failed_ = false;  // geometry change re-arms the breaker
    const int w = (enabled && !roni) ? x1 - x0 : sensor_w_;
    const int h = (enabled && !roni) ? y1 - y0 : sensor_h_;
    rebuild_filter_locked(w, h);
    push_geometry_locked();
}

bool StreamConditioner::set_param(const std::string& k, const std::string& v) {
    std::lock_guard<std::mutex> lk(mutex_);
    const int w = (roi_enabled_ && !roi_roni_) ? roi_x1_ - roi_x0_ : sensor_w_;
    const int h = (roi_enabled_ && !roi_roni_) ? roi_y1_ - roi_y0_ : sensor_h_;
    if (k == "preproc_filter_enabled") {
        filter_enabled_ = to_b(v);
        rebuild_filter_locked(w, h);
        return true;
    }
    if (k == "preproc_downsample") {
        downsample_enabled_ = to_b(v);
        return true;
    }
    if (k == "preproc_undistort_enabled") {
        undistort_enabled_ = to_b(v);
        undistort_lut_failed_ = false;  // re-arm on toggle
        if (undistort_enabled_ && undistort_K_.empty()) {
            std::fprintf(stderr,
                         "StreamConditioner: undistort enabled but no "
                         "intrinsics loaded from '%s' — stage is a no-op "
                         "until a valid calibration file is set\n",
                         undistort_path_.c_str());
        }
        return true;
    }
    if (k == "preproc_undistort_path") {
        undistort_path_ = v;
        cv::Mat K, dist;
        cv::Size sz;
        if (gui_algo::load_intrinsics_yml(v, K, dist, sz)) {
            undistort_K_ = K;
            undistort_dist_ = dist;
        } else {
            // Same silence rule as the old Preprocessor: a missing file
            // while undistort is disabled is the normal uncalibrated state.
            const bool silent =
                !undistort_enabled_ && !v.empty() && !std::filesystem::exists(v);
            if (!silent) {
                std::fprintf(stderr,
                             "StreamConditioner: failed to load intrinsics "
                             "from '%s' — undistort disabled\n", v.c_str());
            }
            undistort_K_.release();
            undistort_dist_.release();
        }
        undistort_lut_valid_ = false;
        undistort_lut_failed_ = false;
        return true;
    }
    static const std::string pfp = "preproc_filter_";
    if (k.size() > pfp.size() && k.compare(0, pfp.size(), pfp) == 0) {
        const std::string bare = k.substr(pfp.size());
        filter_params_[bare] = v;
        if (bare == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 8) {
                filter_mode_ = static_cast<gui_algo::NoiseFilter::Mode>(m);
                rebuild_filter_locked(w, h);
            }
        } else if (filter_) {
            apply_noise_filter_param(*filter_, bare, v);
        }
        return true;
    }
    return false;
}

std::string StreamConditioner::get_param(const std::string& k) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (k == "preproc_filter_enabled") return from_b(filter_enabled_);
    if (k == "preproc_downsample") return from_b(downsample_enabled_);
    if (k == "preproc_undistort_enabled") return from_b(undistort_enabled_);
    if (k == "preproc_undistort_path") return undistort_path_;
    static const std::string pfp = "preproc_filter_";
    if (k.size() > pfp.size() && k.compare(0, pfp.size(), pfp) == 0) {
        const std::string bare = k.substr(pfp.size());
        auto it = filter_params_.find(bare);
        if (it != filter_params_.end()) return it->second;
        if (filter_) return get_noise_filter_param(*filter_, bare);
    }
    return {};
}

void StreamConditioner::reset_temporal() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (filter_) filter_->reset();
}

void StreamConditioner::reset_session_state() {
    std::lock_guard<std::mutex> lk(mutex_);
    // Drop every per-source decision: ROI rect, noise filter (+ its params),
    // 1/4 thin, undistort. Geometry-only FilterChain stages (flip /
    // polarity) are deliberately NOT touched here — their checkbox state
    // lives in the PreprocessingPanel, which owns the enable/replay cycle
    // across disconnects.
    roi_enabled_ = false;
    roi_roni_ = false;
    roi_x0_ = roi_y0_ = roi_x1_ = roi_y1_ = 0;
    filter_enabled_ = false;
    filter_params_.clear();
    downsample_enabled_ = false;
    undistort_enabled_ = false;
    undistort_path_.clear();
    undistort_lut_valid_ = false;
    undistort_lut_failed_ = false;
    undistort_K_.release();
    rebuild_filter_locked(sensor_w_, sensor_h_);
    push_geometry_locked();
}

bool StreamConditioner::active() const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (roi_enabled_) return true;
    if (filter_enabled_ || downsample_enabled_) return true;
    if (undistort_enabled_ && !undistort_K_.empty()) return true;
    if (fc_ && fc_->has_enabled()) return true;
    return false;
}

int StreamConditioner::out_width() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return (roi_enabled_ && !roi_roni_) ? roi_x1_ - roi_x0_ : sensor_w_;
}

int StreamConditioner::out_height() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return (roi_enabled_ && !roi_roni_) ? roi_y1_ - roi_y0_ : sensor_h_;
}

void StreamConditioner::rebuild_filter_locked(int w, int h) {
    if (filter_enabled_ && w > 0 && h > 0) {
        filter_ = std::make_unique<gui_algo::NoiseFilter>(w, h, filter_mode_);
        for (const auto& kv : filter_params_) {
            apply_noise_filter_param(*filter_, kv.first, kv.second);
        }
    } else {
        filter_.reset();
    }
}

std::pair<const Metavision::EventCD*, std::size_t>
StreamConditioner::apply(const Metavision::EventCD* begin,
                         const Metavision::EventCD* end) {
    std::lock_guard<std::mutex> lk(mutex_);
    std::size_t n = static_cast<std::size_t>(end - begin);
    if (n == 0) return {begin, 0};

    const bool fc_value = fc_ && fc_->has_enabled(FilterChain::Group::ValueOnly);
    const bool fc_geo = fc_ && fc_->has_enabled(FilterChain::Group::GeometryOnly);
    const bool undistort_on = undistort_enabled_ && !undistort_K_.empty() &&
                              !undistort_dist_.empty();
    if (!roi_enabled_ && !fc_value && !filter_ && !downsample_enabled_ &&
        !undistort_on && !fc_geo) {
        return {begin, n};  // nothing to do — zero-copy passthrough
    }

    // --- Stage 1: unified ROI (highest priority) ---------------------------
    const Metavision::EventCD* p = begin;
    if (roi_enabled_) {
        roi_buf_.clear();
        roi_buf_.reserve(n);
        if (!roi_roni_) {
            // Keep inside the rect, shift to ROI-relative coordinates. On a
            // live camera with hardware I_ROI the drop is redundant (the
            // sensor already discarded outside events) but harmless.
            const int x0 = roi_x0_, y0 = roi_y0_;
            for (std::size_t i = 0; i < n; ++i) {
                const auto& ev = begin[i];
                if (ev.x >= x0 && ev.x < roi_x1_ && ev.y >= y0 && ev.y < roi_y1_) {
                    roi_buf_.push_back(ev);
                    roi_buf_.back().x =
                        static_cast<std::uint16_t>(ev.x - x0);
                    roi_buf_.back().y =
                        static_cast<std::uint16_t>(ev.y - y0);
                }
            }
        } else {
            // RONI: drop INSIDE the rect; coordinates stay absolute.
            for (std::size_t i = 0; i < n; ++i) {
                const auto& ev = begin[i];
                const bool inside = ev.x >= roi_x0_ && ev.x < roi_x1_ &&
                                    ev.y >= roi_y0_ && ev.y < roi_y1_;
                if (!inside) roi_buf_.push_back(ev);
            }
        }
        p = roi_buf_.data();
        n = roi_buf_.size();
        if (n == 0) return {p, 0};
    }

    // --- Stage 2: FilterChain value stages (polarity) ----------------------
    if (fc_value) {
        fc_buf_.clear();
        fc_->process_value(p, p + n, fc_buf_);
        p = fc_buf_.data();
        n = fc_buf_.size();
        if (n == 0) return {p, 0};
    }

    // gui_algo::Event is layout-compatible with Metavision::EventCD
    // (static_assert in algo/common/event.h).
    auto* evp = reinterpret_cast<gui_algo::Event*>(const_cast<Metavision::EventCD*>(p));
    bool in_work = false;  // events currently live in work_buf_

    // --- Stage 3: noise filter (once) ---------------------------------------
    if (filter_) {
        if (!in_work) {
            work_buf_.assign(evp, evp + n);
            in_work = true;
        }
        n = filter_->filter(work_buf_.data(), n);
        if (n == 0) return {reinterpret_cast<const Metavision::EventCD*>(
                               work_buf_.data()), 0};
    }

    // --- Stage 4: 1/4 downsample thin (halving stays per consumer) ---------
    if (downsample_enabled_) {
        if (!in_work) {
            work_buf_.assign(evp, evp + n);
            in_work = true;
        }
        std::size_t kept = 0;
        for (std::size_t i = 0; i < n; ++i) {
            if ((work_buf_[i].x & 1) == 0 && (work_buf_[i].y & 1) == 0) {
                work_buf_[kept] = work_buf_[i];
                ++kept;
            }
        }
        n = kept;
        if (n == 0) return {reinterpret_cast<const Metavision::EventCD*>(
                               work_buf_.data()), 0};
    }

    // --- Stage 5: undistort (single LUT, factor 1) --------------------------
    if (undistort_on) {
        const int w = (roi_enabled_ && !roi_roni_) ? roi_x1_ - roi_x0_ : sensor_w_;
        const int h = (roi_enabled_ && !roi_roni_) ? roi_y1_ - roi_y0_ : sensor_h_;
        if (!undistort_lut_failed_ && !undistort_lut_valid_) {
            try {
                rebuild_undistort_lut_locked(w, h);
            } catch (const cv::Exception& e) {
                std::fprintf(stderr,
                             "StreamConditioner: undistort LUT rebuild "
                             "failed: %s\n", e.what());
                undistort_lut_valid_ = false;
                undistort_lut_failed_ = true;  // one attempt per config
            }
        }
        if (undistort_lut_valid_ && undistort_eff_w_ > 0 && undistort_eff_h_ > 0) {
            if (!in_work) {
                work_buf_.assign(evp, evp + n);
                in_work = true;
            }
            const int eff_w = undistort_eff_w_;
            const int eff_h = undistort_eff_h_;
            std::size_t kept = 0;
            for (std::size_t i = 0; i < n; ++i) {
                const int x = work_buf_[i].x;
                const int y = work_buf_[i].y;
                if (x < 0 || y < 0 || x >= eff_w || y >= eff_h) continue;
                const cv::Point2f& mapped =
                    undistort_lut_[static_cast<std::size_t>(y) * eff_w + x];
                const int nx = static_cast<int>(std::lround(mapped.x));
                const int ny = static_cast<int>(std::lround(mapped.y));
                if (nx < 0 || ny < 0 || nx >= eff_w || ny >= eff_h) continue;
                work_buf_[kept] = work_buf_[i];
                work_buf_[kept].x = static_cast<std::uint16_t>(nx);
                work_buf_[kept].y = static_cast<std::uint16_t>(ny);
                ++kept;
            }
            n = kept;
            if (n == 0) return {reinterpret_cast<const Metavision::EventCD*>(
                                   work_buf_.data()), 0};
        }
    }

    const Metavision::EventCD* out = in_work
        ? reinterpret_cast<const Metavision::EventCD*>(work_buf_.data())
        : p;

    // --- Stage 6: FilterChain geometry stages (flips, after undistort) -----
    if (fc_geo) {
        // The flips mirror within this stream's coordinate system (ROI dims
        // in ROI mode, source dims otherwise) — the geometry is kept current
        // by push_geometry_locked() at every geometry mutation point
        // (init / set_roi / set_filter_chain, GUI thread); the batch path
        // never touches it.
        geo_buf_.clear();
        fc_->process_geometry(out, out + n, geo_buf_);
        out = geo_buf_.data();
        n = geo_buf_.size();
    }
    return {out, n};
}

void StreamConditioner::rebuild_undistort_lut_locked(int w, int h) {
    undistort_lut_valid_ = false;
    undistort_lut_.clear();
    undistort_eff_w_ = 0;
    undistort_eff_h_ = 0;
    if (undistort_K_.empty() || undistort_dist_.empty()) return;
    if (w <= 0 || h <= 0) return;

    // Adjust K from sensor resolution to the stage-1 coordinate system:
    // ROI mode shifts the origin by (roi_x0_, roi_y0_); RONI keeps ABSOLUTE
    // coordinates, so K must stay unadjusted there (the rect origin is
    // meaningless for the coordinate system). Factor is always 1 here
    // (halving is per consumer, downstream). cv::undistortPoints with
    // P = K_adj returns undistorted pixels in the same adjusted system —
    // directly indexable by event coordinates.
    cv::Mat K_adj = undistort_K_.clone();
    if (K_adj.type() != CV_64F) K_adj.convertTo(K_adj, CV_64F);
    cv::Mat dist = undistort_dist_;
    if (dist.type() != CV_64F) dist.convertTo(dist, CV_64F);
    const bool roi_relative = roi_enabled_ && !roi_roni_;
    const double ox = roi_relative ? static_cast<double>(roi_x0_) : 0.0;
    const double oy = roi_relative ? static_cast<double>(roi_y0_) : 0.0;
    K_adj.at<double>(0, 2) -= ox;
    K_adj.at<double>(1, 2) -= oy;

    std::vector<cv::Point2f> pts;
    pts.reserve(static_cast<std::size_t>(w) * h);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            pts.emplace_back(static_cast<float>(x), static_cast<float>(y));
        }
    }
    std::vector<cv::Point2f> undistorted;
    cv::undistortPoints(pts, undistorted, K_adj, dist, cv::noArray(), K_adj);
    undistort_lut_ = std::move(undistorted);
    undistort_eff_w_ = w;
    undistort_eff_h_ = h;
    undistort_lut_valid_ = true;
}

} // namespace gui
