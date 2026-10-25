// algo/cv/time_surface.h — Time Surface window.
//
// Design §4.3.27. 直接引用 OpenEB 源码实现：
//   - Metavision::MostRecentTimestampBuffer 作为时间戳缓冲区
//   - Metavision::TimeSurfaceProcessor 用事件更新缓冲区
//   - MostRecentTimestampBuffer::generate_img_time_surface* 生成线性衰减灰度图
//   - 在灰度图基础上叠加伪彩色调色板 (Gray/Hot/Plasma/Turbo)
//
// OpenEB 的线性衰减公式 (mostrecent_timestamp_buffer_impl.h):
//   ratio = 255 / delta_t
//   pixel = (delta_t - (last_ts - ts)) * ratio   // clamped to [0, 255]
// 即 last_ts → 255 (亮), last_ts - delta_t → 0 (暗)。
//
// 指数衰减模式对齐 dv-processing Accumulator 的 EXPONENTIAL 分支
// (ref/dv-processing-master .../core/frame/accumulator.hpp:119-154):
//   decay():     potential = (p - neutral) * exp(-(t - t_last) / tau) + neutral
//   contribute(): potential = clamp(potential + eventContribution, min, max)
// accumulate() 对每个事件先 decay 再 contribute，电位在事件间持续累积；
// generateFrame() 时做同步衰减 (synchronousDecay) 到 current_t_ 再归一化。
// 默认参数与 dv 一致: eventContribution=0.15, neutral=0, [min,max]=[0,1]。
// MostRecentTimestampBuffer 充当 dv 的 decayTimeSurface_ (每像素最近事件时间)。

#ifndef GUI_ALGO_CV_TIME_SURFACE_H
#define GUI_ALGO_CV_TIME_SURFACE_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>
#include <metavision/sdk/core/preprocessors/time_surface_processor.h>
#include <metavision/sdk/core/utils/mostrecent_timestamp_buffer.h>

#include "algo/common/event.h"

namespace gui_algo {

/// @brief Time Surface renderer — 直接引用 OpenEB MostRecentTimestampBuffer。
class TimeSurface {
public:
    enum class Channels {
        Merged = 1,   ///< 1 channel: both polarities share one buffer.
        Split = 2,    ///< 2 channels: separate ON/OFF buffers.
    };

    enum class Palette {
        Gray,
        Hot,
        Plasma,
        Turbo,
    };

    enum class Decay {
        Linear,       ///< OpenEB linear window (hard cut to 0 at the tail).
        Exponential,  ///< dv-processing Accumulator EXPONENTIAL: per-event
                       ///< decay + contribute, synchronous decay at render.
    };

    /// @brief Constructs the time surface.
    /// @param width,height Sensor dimensions.
    /// @param channels Merged or split polarity buffers.
    /// @param decay_time_us Linear decay time window in us (OpenEB delta_t).
    /// @param palette Pseudo-color palette.
    /// @param refresh_rate_hz Target render refresh rate in Hz.
    /// @param decay Linear (default, unchanged behavior) or Exponential.
    /// @param tau_us Exponential decay time constant in us (dv decayParam).
    TimeSurface(int width, int height,
                Channels channels = Channels::Merged,
                Metavision::timestamp decay_time_us = 100000,
                Palette palette = Palette::Hot,
                int refresh_rate_hz = 30,
                Decay decay = Decay::Linear,
                Metavision::timestamp tau_us = 100000)
        : width_(width), height_(height), channels_(channels),
          decay_time_us_(clamp_decay(decay_time_us)),
          palette_(palette),
          refresh_rate_hz_(clamp_refresh(refresh_rate_hz)),
          decay_(decay),
          tau_us_(clamp_decay(tau_us)),
          // OpenEB MostRecentTimestampBuffer: rows=height, cols=width,
          // channels = 1 (merged) or 2 (split polarity).
          ts_buf_(height, width, static_cast<int>(channels_)) {
        ts_buf_.set_to(-1);  // -1 = sentinel for "never hit" (0 is a legal ts)
        alloc_potential();
        rebuild_palette_lut();
    }

    /// @brief Updates the time surface with a batch of events.
    /// Linear 模式仅更新 MostRecentTimestampBuffer (OpenEB TimeSurfaceProcessor
    /// 内联逻辑)。Exponential 模式对齐 dv Accumulator::accumulate(): 对每个
    /// 事件先 decay (按 exp(-dt/tau) 衰减已有电位) 再 contribute (叠加
    /// eventContribution_，clamp 到 [min, max])，电位在事件间持续累积。
    void process(const Event* events, std::size_t n) {
        if (events == nullptr || n == 0) return;
        const int ch = static_cast<int>(channels_);
        const bool exp = (decay_ == Decay::Exponential);
        const float inv_tau = exp ? (1.0f / static_cast<float>(tau_us_)) : 0.0f;
        for (std::size_t i = 0; i < n; ++i) {
            const Event& e = events[i];
            if (e.x >= width_ || e.y >= height_) continue;
            const int c = (ch == 1) ? 0 : (e.p & 1);
            const Metavision::timestamp prev_t = ts_buf_.at(e.y, e.x, c);
            ts_buf_.at(e.y, e.x, c) = e.t;
            if (e.t > current_t_) current_t_ = e.t;
            if (exp) {
                // dv accumulator.hpp decay() + contribute() (lines 119-154).
                float* row = potential_surface_.ptr<float>(e.y);
                const int idx = (ch == 1) ? e.x : (e.x + c * width_);
                float pot = row[idx];
                if (prev_t >= 0 && e.t > prev_t) {
                    const float dt = static_cast<float>(e.t - prev_t);
                    pot = (pot - neutral_potential_) * std::exp(-dt * inv_tau)
                          + neutral_potential_;
                }
                pot = std::min(std::max(pot + event_contribution_, min_potential_),
                               max_potential_);
                row[idx] = pot;
            }
        }
    }

    /// @brief Renders the time-decay encoded pseudo-color image (CV_8UC3).
    /// Linear 模式用 OpenEB MostRecentTimestampBuffer::generate_img_time_surface*
    /// 生成 CV_8UC1 线性衰减灰度图；Exponential 模式对齐 dv Accumulator
    /// generateFrame() (synchronousDecay=true): 将累积电位同步衰减到 current_t_
    /// 后按 dv 公式归一化到 [0,255]；最后用调色板映射为伪彩色。
    cv::Mat render() const {
        cv::Mat img(height_, width_, CV_8UC3, cv::Scalar(0, 0, 0));
        if (width_ <= 0 || height_ <= 0) return img;
        if (current_t_ < 0) return img;  // no events yet: all pixels "never hit"

        // OpenEB 线性衰减灰度图生成；指数模式按 dv 累加器归一化。
        cv::Mat gray;
        if (decay_ == Decay::Exponential) {
            // dv Accumulator generateFrame() (accumulator.hpp:276-290):
            // synchronousDecay=true — 将每像素电位衰减到 current_t_ 后归一化。
            // 不修改 potential_surface_/ts_buf_ (render 是只读快照)：显示值
            //   display = (pot - neutral) * exp(-(current_t - last)/tau) + neutral
            // 下一次 process() 仍从 last (事件时间) 衰减，电位状态一致。
            const float inv_tau = 1.0f / static_cast<float>(tau_us_);
            const double scale = 255.0 / static_cast<double>(max_potential_ - min_potential_);
            const double shift = -static_cast<double>(min_potential_) * scale;
            const int ch = static_cast<int>(channels_);
            if (ch == 1) {
                gray.create(height_, width_, CV_8UC1);
                gray.setTo(0);
                for (int y = 0; y < height_; ++y) {
                    const auto* pot_row = potential_surface_.ptr<float>(y);
                    auto* dst = gray.ptr<std::uint8_t>(y);
                    for (int x = 0; x < width_; ++x) {
                        const Metavision::timestamp last = ts_buf_.at(y, x, 0);
                        if (last < 0) { dst[x] = 0; continue; }
                        float v = pot_row[x];
                        if (current_t_ > last) {
                            const float dt = static_cast<float>(current_t_ - last);
                            v = (v - neutral_potential_) * std::exp(-dt * inv_tau)
                                + neutral_potential_;
                        }
                        dst[x] = cv::saturate_cast<std::uint8_t>(v * scale + shift);
                    }
                }
            } else {
                // Split: 产生并排 [OFF | ON] 灰度图，复用下方 Split 合并路径。
                gray.create(height_, width_ * 2, CV_8UC1);
                gray.setTo(0);
                for (int y = 0; y < height_; ++y) {
                    const auto* pot_row = potential_surface_.ptr<float>(y);
                    auto* dst = gray.ptr<std::uint8_t>(y);
                    for (int c = 0; c < 2; ++c) {
                        for (int x = 0; x < width_; ++x) {
                            const Metavision::timestamp last = ts_buf_.at(y, x, c);
                            const int dst_x = c * width_ + x;
                            if (last < 0) { dst[dst_x] = 0; continue; }
                            float v = pot_row[x + c * width_];
                            if (current_t_ > last) {
                                const float dt = static_cast<float>(current_t_ - last);
                                v = (v - neutral_potential_) * std::exp(-dt * inv_tau)
                                    + neutral_potential_;
                            }
                            dst[dst_x] = cv::saturate_cast<std::uint8_t>(v * scale + shift);
                        }
                    }
                }
            }
        } else if (channels_ == Channels::Merged) {
            // 合并极性: generate_img_time_surface_collapsing_channels
            ts_buf_.generate_img_time_surface_collapsing_channels(
                current_t_, decay_time_us_, gray);
        } else {
            // 分极性: generate_img_time_surface 输出并排 (W*2) 灰度图。
            // 我们只取左半 (OFF) 和右半 (ON) 合并为单帧彩色。
            ts_buf_.generate_img_time_surface(
                current_t_, decay_time_us_, gray);
        }

        if (gray.empty()) return img;

        if (channels_ == Channels::Split && gray.cols == width_ * 2) {
            // OpenEB split 模式输出 [OFF | ON] 并排。取两半叠加。
            cv::Mat off_roi(gray, cv::Rect(0, 0, width_, height_));
            cv::Mat on_roi(gray, cv::Rect(width_, 0, width_, height_));
            for (int y = 0; y < height_; ++y) {
                auto* dst = img.ptr<cv::Vec3b>(y);
                const auto* off = off_roi.ptr<std::uint8_t>(y);
                const auto* on = on_roi.ptr<std::uint8_t>(y);
                for (int x = 0; x < width_; ++x) {
                    const cv::Vec3b c_off = palette_lut_[off[x]];
                    const cv::Vec3b c_on = palette_lut_[on[x]];
                    // Per-channel max (§四-M2): cv::Vec3b + is a SATURATING
                    // add, so (c_off+c_on)*0.5 halved the brightness of
                    // single-polarity full-scale pixels (255 -> 128).
                    dst[x] = cv::Vec3b(std::max(c_off[0], c_on[0]),
                                       std::max(c_off[1], c_on[1]),
                                       std::max(c_off[2], c_on[2]));
                }
            }
        } else {
            // Merged: 灰度 → 伪彩色映射。
            for (int y = 0; y < height_; ++y) {
                auto* dst = img.ptr<cv::Vec3b>(y);
                const auto* src = gray.ptr<std::uint8_t>(y);
                for (int x = 0; x < width_; ++x) {
                    dst[x] = palette_lut_[src[x]];
                }
            }
        }
        return img;
    }

    /// @brief Returns the minimum render interval in us from refresh_rate_hz.
    Metavision::timestamp refresh_interval_us() const {
        return static_cast<Metavision::timestamp>(1.0e6 / refresh_rate_hz_);
    }

    void set_channels(Channels c) {
        if (c != channels_) {
            channels_ = c;
            ts_buf_.create(height_, width_, static_cast<int>(channels_));
            ts_buf_.set_to(-1);  // -1 = "never hit" sentinel
            alloc_potential();
        }
    }
    Channels channels() const { return channels_; }

    void set_decay_time_us(Metavision::timestamp us) {
        decay_time_us_ = clamp_decay(us);
    }
    Metavision::timestamp decay_time_us() const { return decay_time_us_; }

    void set_decay(Decay d) { decay_ = d; }
    Decay decay() const { return decay_; }

    void set_tau_us(Metavision::timestamp us) {
        tau_us_ = clamp_decay(us);
    }
    Metavision::timestamp tau_us() const { return tau_us_; }

    void set_palette(Palette p) {
        palette_ = p;
        rebuild_palette_lut();
    }
    Palette palette() const { return palette_; }

    void set_refresh_rate_hz(int hz) { refresh_rate_hz_ = clamp_refresh(hz); }
    int refresh_rate_hz() const { return refresh_rate_hz_; }

    /// @brief Clears the timestamp buffer and accumulated potential.
    void reset() {
        ts_buf_.set_to(-1);  // -1 = "never hit" sentinel (0 is a legal ts)
        if (!potential_surface_.empty())
            potential_surface_.setTo(neutral_potential_);
        current_t_ = -1;
    }

    int width() const { return width_; }
    int height() const { return height_; }

private:
    static Metavision::timestamp clamp_decay(Metavision::timestamp us) {
        if (us < 10000) return 10000;
        if (us > 5000000) return 5000000;
        return us;
    }
    static int clamp_refresh(int hz) {
        if (hz < 10) return 10;
        if (hz > 120) return 120;
        return hz;
    }

    /// @brief Rebuilds the 256-entry palette LUT from map_color(). Entries
    /// are computed by the very same function the per-pixel loops used to
    /// call, so palette_lut_[g] is bit-identical to map_color(g / 255.0).
    void rebuild_palette_lut() {
        palette_lut_.resize(256);
        for (int i = 0; i < 256; ++i) {
            palette_lut_[static_cast<std::size_t>(i)] = map_color(i / 255.0);
        }
    }

    /// @brief Allocates the Exponential-mode potential surface.
    /// CV_32F, (height, width) for Merged or (height, width*2) for Split
    /// with [0,width)=OFF, [width,2*width)=ON. Initialized to neutral.
    void alloc_potential() {
        const int ch = static_cast<int>(channels_);
        potential_surface_.create(height_, width_ * ch, CV_32F);
        potential_surface_.setTo(neutral_potential_);
    }

    /// @brief Maps a normalized value in [0, 1] to a palette color.
    cv::Vec3b map_color(double v) const {
        if (v < 0.0) v = 0.0;
        if (v > 1.0) v = 1.0;
        const auto b8 = [](double x) {
            return static_cast<std::uint8_t>(x * 255.0 + 0.5);
        };
        switch (palette_) {
            case Palette::Gray: {
                const std::uint8_t g = b8(v);
                return cv::Vec3b(g, g, g);
            }
            case Palette::Hot: {
                double r = 0.0, g = 0.0, b = 0.0;
                if (v < 0.33) {
                    r = v / 0.33;
                } else if (v < 0.66) {
                    r = 1.0;
                    g = (v - 0.33) / 0.33;
                } else {
                    r = 1.0;
                    g = 1.0;
                    b = (v - 0.66) / 0.34;
                }
                return cv::Vec3b(b8(b), b8(g), b8(r));
            }
            case Palette::Plasma: {
                double r = v;
                double g = std::max(0.0, (v - 0.5) * 2.0);
                double b = std::max(0.0, 0.5 - 0.5 * v);
                return cv::Vec3b(b8(b), b8(g), b8(r));
            }
            case Palette::Turbo: {
                double r = 0.0, g = 0.0, b = 0.0;
                if (v < 0.25) {
                    b = 1.0;
                    g = v / 0.25;
                } else if (v < 0.5) {
                    b = 1.0 - (v - 0.25) / 0.25;
                    g = 1.0;
                } else if (v < 0.75) {
                    g = 1.0;
                    r = (v - 0.5) / 0.25;
                } else {
                    r = 1.0;
                    g = 1.0 - (v - 0.75) / 0.25;
                }
                return cv::Vec3b(b8(b), b8(g), b8(r));
            }
        }
        return cv::Vec3b(0, 0, 0);
    }

    int width_;
    int height_;
    Channels channels_;
    Metavision::timestamp decay_time_us_;
    Palette palette_;
    int refresh_rate_hz_;
    Decay decay_;
    Metavision::timestamp tau_us_;  // dv decayParam: exp decay time constant
    // dv Accumulator defaults (accumulator.hpp:198-213).
    float event_contribution_{0.15f};   // contribution per event
    float neutral_potential_{0.0f};     // decay asymptote
    float max_potential_{1.0f};         // upper clamp
    float min_potential_{0.0f};         // lower clamp
    /// Accumulated potential surface (Exponential mode only). CV_32F,
    /// (height, width) for Merged or (height, width*2) for Split.
    cv::Mat potential_surface_;
    /// OpenEB MostRecentTimestampBuffer — 直接引用，不自行实现时间戳缓冲区。
    /// 兼作 dv decayTimeSurface_ (每像素最近事件时间)。
    Metavision::MostRecentTimestampBuffer ts_buf_;
    /// 256-entry pseudo-color LUT: palette_lut_[g] == map_color(g / 255.0).
    /// Rebuilt whenever the palette changes so the per-pixel render loops
    /// skip the branches/divisions of map_color (bit-identical output).
    std::vector<cv::Vec3b> palette_lut_;
    Metavision::timestamp current_t_{-1};  // -1 = no event processed yet
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_TIME_SURFACE_H
