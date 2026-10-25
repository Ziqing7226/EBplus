// algo/analytics/sensor_self_test.h — sensor bad-pixel & refractory-period analyzer.
//
// Design §4.4.8. Per-pixel sensor self-test: tracks the minimum inter-event
// interval (refractory period) for every pixel and flags pixels that never
// fire as suspected bad pixels. Intended to be run while the user shakes the
// camera to stimulate event generation across the full sensor.
//
// Algorithm complexity: O(1) per event (two array reads + one write + one
// comparison). The per-pixel minimum interval is updated in place; no
// sorting or histogram binning is performed during event processing.
//
// Visualization: a grayscale heatmap where brighter = shorter refractory
// period. Pixels that never triggered any event are shown in red (suspected
// bad pixels). The interval-to-brightness mapping uses an exponential curve
// precomputed as a LUT so render() is O(N) with a trivial constant.
//
// Header-only, matching hot_pixel_filter.h conventions.

#ifndef GUI_ALGO_ANALYTICS_SENSOR_SELF_TEST_H
#define GUI_ALGO_ANALYTICS_SENSOR_SELF_TEST_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"

namespace gui_algo {

/// @brief Per-pixel sensor self-test: bad-pixel detection + refractory-period
///        estimation.
class SensorSelfTest {
public:
    /// Sentinel marking a pixel that has never seen an event.
    static constexpr Metavision::timestamp kNeverSeen =
        std::numeric_limits<Metavision::timestamp>::min();
    /// Sentinel marking a pixel that has seen events but only one so far
    /// (no inter-event interval could be computed yet).
    static constexpr Metavision::timestamp kNoInterval =
        std::numeric_limits<Metavision::timestamp>::max();

    /// @brief Constructs the analyzer for a sensor of the given dimensions.
    /// @param width,height Sensor dimensions in pixels.
    explicit SensorSelfTest(int width, int height)
        : width_(width), height_(height),
          last_ts_(static_cast<std::size_t>(width) * height, kNeverSeen),
          min_interval_(static_cast<std::size_t>(width) * height, kNoInterval) {
        build_lut();
    }

    int width() const { return width_; }
    int height() const { return height_; }

    /// @brief Feeds a batch of events and updates per-pixel state. O(n).
    void process(const Event* events, std::size_t n) {
        if (events == nullptr || n == 0) return;
        for (std::size_t i = 0; i < n; ++i) {
            const Event& e = events[i];
            if (e.x >= width_ || e.y >= height_) continue;
            const std::size_t idx =
                static_cast<std::size_t>(e.y) * width_ + e.x;
            const Metavision::timestamp prev = last_ts_[idx];
            if (prev != kNeverSeen && e.t > prev) {
                const Metavision::timestamp dt = e.t - prev;
                if (dt < min_interval_[idx]) {
                    min_interval_[idx] = dt;
                }
            }
            last_ts_[idx] = e.t;
        }
    }

    /// @brief Renders the refractory-period heatmap as an HxW CV_8UC3 image.
    /// Red = never triggered (suspected bad pixel); grayscale = exponential
    /// mapping of min interval (brighter = shorter refractory period).
    cv::Mat render() const {
        cv::Mat img(height_, width_, CV_8UC3);
        // Row-pointer traversal over the same flat indices (idx = y*width+x)
        // as before — identical writes, minus the per-pixel at<> bounds
        // checks and the div/mod index arithmetic.
        for (int y = 0; y < height_; ++y) {
            auto* row = img.ptr<cv::Vec3b>(y);
            const std::size_t base = static_cast<std::size_t>(y) * width_;
            for (int x = 0; x < width_; ++x) {
                const std::size_t idx = base + static_cast<std::size_t>(x);
                if (last_ts_[idx] == kNeverSeen) {
                    // Suspected bad pixel — red.
                    row[x] = cv::Vec3b(0, 0, 255);
                } else if (min_interval_[idx] == kNoInterval) {
                    // Only one event — insufficient data, render dark.
                    row[x] = cv::Vec3b(0, 0, 0);
                } else {
                    const int v = lut_[interval_to_lut_index(min_interval_[idx])];
                    row[x] = cv::Vec3b(v, v, v);
                }
            }
        }
        return img;
    }

    // --- Statistics -------------------------------------------------------

    /// @brief Lightweight per-pixel counts (O(N), no sort). Used for the
    /// per-frame status label so we don't pay the O(N log N) sort cost of
    /// compute_stats() every frame.
    struct PixelCounts {
        std::size_t total{0};       ///< width * height
        std::size_t triggered{0};   ///< pixels with ≥ 1 event
        std::size_t measured{0};    ///< pixels with ≥ 2 events (valid interval)
        std::size_t bad{0};         ///< pixels with 0 events
    };

    /// @brief Counts pixels in each category. O(N) with a trivial constant
    /// (comparisons + increments only, no sorting).
    PixelCounts count_pixels() const {
        PixelCounts c;
        c.total = static_cast<std::size_t>(width_) * height_;
        for (std::size_t idx = 0; idx < c.total; ++idx) {
            if (last_ts_[idx] != kNeverSeen) {
                ++c.triggered;
                if (min_interval_[idx] != kNoInterval) ++c.measured;
            } else {
                ++c.bad;
            }
        }
        return c;
    }

    /// @brief Summary statistics over all pixels with a valid min interval.
    struct Stats {
        std::size_t total_pixels{0};       ///< width * height
        std::size_t triggered_pixels{0};   ///< pixels with ≥ 1 event
        std::size_t measured_pixels{0};    ///< pixels with ≥ 2 events (valid interval)
        std::size_t bad_pixels{0};         ///< pixels with 0 events
        Metavision::timestamp min_us{0};   ///< shortest min-interval seen
        Metavision::timestamp max_us{0};   ///< longest min-interval seen
        double mean_us{0.0};
        double median_us{0.0};
        double p90_us{0.0};
    };

    /// @brief Computes summary statistics (one-time O(N log N) for the sort
    /// needed by median/p90). Called at report time, not per-event.
    Stats compute_stats() const {
        Stats s;
        s.total_pixels = static_cast<std::size_t>(width_) * height_;
        std::vector<Metavision::timestamp> intervals;
        intervals.reserve(s.total_pixels / 4);
        for (std::size_t idx = 0; idx < s.total_pixels; ++idx) {
            if (last_ts_[idx] != kNeverSeen) {
                ++s.triggered_pixels;
                if (min_interval_[idx] != kNoInterval) {
                    intervals.push_back(min_interval_[idx]);
                }
            } else {
                ++s.bad_pixels;
            }
        }
        s.measured_pixels = intervals.size();
        if (intervals.empty()) return s;
        std::sort(intervals.begin(), intervals.end());
        s.min_us = intervals.front();
        s.max_us = intervals.back();
        double sum = 0.0;
        for (auto v : intervals) sum += static_cast<double>(v);
        s.mean_us = sum / static_cast<double>(intervals.size());
        s.median_us = static_cast<double>(intervals[intervals.size() / 2]);
        const std::size_t p90_idx =
            static_cast<std::size_t>(intervals.size() * 0.9);
        s.p90_us = static_cast<double>(
            intervals[std::min(p90_idx, intervals.size() - 1)]);
        return s;
    }

    /// @brief Returns the coordinates of all suspected bad pixels (never
    /// triggered). The vector is capped at @p max_count entries; if there are
    /// more, only the first @p max_count are returned.
    std::vector<std::pair<int, int>> bad_pixel_coords(
        std::size_t max_count = 200) const {
        std::vector<std::pair<int, int>> coords;
        const std::size_t total = static_cast<std::size_t>(width_) * height_;
        for (std::size_t idx = 0; idx < total && coords.size() < max_count; ++idx) {
            if (last_ts_[idx] == kNeverSeen) {
                coords.emplace_back(static_cast<int>(idx % width_),
                                    static_cast<int>(idx / width_));
            }
        }
        return coords;
    }

    /// @brief Formats a human-readable report string for the close dialog.
    std::string report() const {
        const Stats s = compute_stats();
        std::string r = "Sensor Self-Test Report\n";
        r += "========================\n\n";
        r += "Sensor: " + std::to_string(width_) + "x" +
             std::to_string(height_) + " (" +
             std::to_string(s.total_pixels) + " pixels)\n";
        r += "Triggered pixels: " + std::to_string(s.triggered_pixels) +
             " / " + std::to_string(s.total_pixels) + "\n";
        r += "Pixels with measured interval: " +
             std::to_string(s.measured_pixels) + "\n";
        r += "Suspected bad pixels: " + std::to_string(s.bad_pixels) + "\n\n";
        if (s.measured_pixels > 0) {
            r += "Refractory period (min interval per pixel):\n";
            r += "  Min:    " + std::to_string(s.min_us) + " us\n";
            r += "  Max:    " + std::to_string(s.max_us) + " us\n";
            r += "  Mean:   " + std::to_string(s.mean_us) + " us\n";
            r += "  Median: " + std::to_string(s.median_us) + " us\n";
            r += "  P90:    " + std::to_string(s.p90_us) + " us\n";
            r += "  (P90 = 90% of pixels have refractory period <= this value)\n";
        } else {
            r += "No inter-event intervals measured.\n";
            r += "Shake the camera more to stimulate event generation.\n";
        }
        const auto coords = bad_pixel_coords(50);
        if (!coords.empty()) {
            r += "\nSuspected bad pixel coordinates (x, y):\n";
            for (const auto& [x, y] : coords) {
                r += "  (" + std::to_string(x) + ", " + std::to_string(y) + ")\n";
            }
            if (s.bad_pixels > coords.size()) {
                r += "  ... and " + std::to_string(s.bad_pixels - coords.size()) +
                     " more.\n";
            }
        }
        return r;
    }

    /// @brief Resets all per-pixel state.
    void reset() {
        std::fill(last_ts_.begin(), last_ts_.end(), kNeverSeen);
        std::fill(min_interval_.begin(), min_interval_.end(), kNoInterval);
    }

private:
    /// Upper bound of the interval range mapped by the LUT (us). Intervals
    /// longer than this are clamped to the darkest gray.
    static constexpr int kMaxIntervalUs = 10000;
    /// Exponential decay constant: v = 255 * exp(-dt / tau).
    /// tau = 2000 → dt=1 maps to ~255, dt=10000 maps to ~2.
    static constexpr double kTau = 2000.0;

    void build_lut() {
        lut_.resize(kMaxIntervalUs + 1);
        for (int dt = 0; dt <= kMaxIntervalUs; ++dt) {
            const double v = 255.0 * std::exp(-static_cast<double>(dt) / kTau);
            int iv = static_cast<int>(std::lround(v));
            if (iv < 0) iv = 0;
            if (iv > 255) iv = 255;
            lut_[static_cast<std::size_t>(dt)] =
                static_cast<std::uint8_t>(iv);
        }
    }

    int interval_to_lut_index(Metavision::timestamp dt) const {
        if (dt < 0) return 0;
        if (dt > kMaxIntervalUs) return kMaxIntervalUs;
        return static_cast<int>(dt);
    }

    int width_;
    int height_;
    std::vector<Metavision::timestamp> last_ts_;
    std::vector<Metavision::timestamp> min_interval_;
    std::vector<std::uint8_t> lut_;  ///< exponential interval→brightness LUT
};

} // namespace gui_algo

#endif // GUI_ALGO_ANALYTICS_SENSOR_SELF_TEST_H
