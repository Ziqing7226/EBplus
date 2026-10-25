// algo/common/lif_integrator.h — Leaky Integrate-and-Fire neuron integrator.
//
// ✅ 移植自 jAER BlurringTunnelFilter / LIFNeuron. Models a 2D grid of
// leaky integrate-and-fire neurons driven by events: each event adds +1.0 to
// the membrane potential of the corresponding cell (regardless of polarity,
// per jAER BlurringTunnelFilter), which then leaks exponentially toward 0.
// When a cell's potential crosses the firing threshold, it emits a spike and
// either resets to reset_value (legacy mode, when jump_after_firing_percent
// == 0) or drops by max(jump_after_firing_percent * threshold / 100, 1.0)
// (jAER behaviour, retaining residual potential). Initial membrane potential
// is set to initial_potential_percent * threshold / 100 — 有意修正 jAER
// LIFNeuron.reset 漏除 100 的上游 bug（jAER 实为 MPInitialPercnetTh *
// MPThreshold，默认 50×15=750，导致神经元首个事件即发放；本实现按百分数
// 语义计算，默认 50% → 7.5）。Used by ClusterLIF
// (4.3.18) and related clustering algorithms. Header-only.

#ifndef GUI_ALGO_COMMON_LIF_INTEGRATOR_H
#define GUI_ALGO_COMMON_LIF_INTEGRATOR_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <metavision/sdk/base/utils/timestamp.h>

namespace gui_algo {

/// @brief 2D grid of leaky integrate-and-fire neurons driven by events.
class LifIntegrator {
public:
    /// @brief Constructs the neuron grid.
    /// @param width,height Grid (sensor) dimensions.
    /// @param tau_us Membrane time constant (leak). Larger = slower decay.
    /// @param threshold Firing threshold. Crossing it emits a spike + reset.
    /// @param reset_value Potential applied after a spike when
    ///        jump_after_firing_percent == 0 (legacy full-reset mode).
    /// @param decay_step_us Discrete leak granularity (unused; the value is
    /// accepted for call-site compatibility but not stored).
    /// @param initial_potential_percent Initial MP as a percent of threshold
    ///        (jAER MPInitialPercnetTh，但有意修正其漏除 100 的上游 bug，
    ///        见头注释). 0 → start at reset_value (legacy).
    /// @param jump_after_firing_percent MP drop after firing as a percent of
    ///        threshold (jAER MPJumpAfterFiringPercentTh). 0 → legacy
    ///        full-reset to reset_value; >0 → MP -= max(pct*th/100, 1.0),
    ///        matching jAER LIFNeuron.reduceMPafterFiring (floor of 1.0).
    LifIntegrator(int width, int height,
                  Metavision::timestamp tau_us = 10000,
                  double threshold = 1.0,
                  double reset_value = 0.0,
                  Metavision::timestamp decay_step_us = 1000,
                  double initial_potential_percent = 0.0,
                  double jump_after_firing_percent = 0.0)
        : width_(width), height_(height), tau_us_(tau_us),
          threshold_(threshold), reset_value_(reset_value),
          initial_potential_percent_(initial_potential_percent),
          jump_after_firing_percent_(jump_after_firing_percent),
          initial_potential_(compute_initial_potential()),
          potential_(static_cast<std::size_t>(width) * height,
                     initial_potential_),
          last_ts_(static_cast<std::size_t>(width) * height, -1) {}

    /// @brief Feeds an event. Returns true if this cell fired on this event.
    /// Mirrors jAER LIFNeuron.addEvent: incrementMP then conditional fire.
    /// A cell fires at most once per event (jAER checks once after the
    /// increment); the residual MP may remain above threshold and fire again
    /// on a subsequent event.
    bool add_event(std::uint16_t x, std::uint16_t y, short p,
                   Metavision::timestamp t) {
        if (x >= width_ || y >= height_) return false;
        const std::size_t idx = static_cast<std::size_t>(y) * width_ + x;
        if (t > latest_t_) latest_t_ = t;
        // Apply leak since the last update at this cell (decays toward 0).
        const auto prev = last_ts_[idx];
        last_ts_[idx] = t;
        if (prev >= 0 && t > prev) {
            const double dt = static_cast<double>(t - prev);
            potential_[idx] *= std::exp(-dt / static_cast<double>(tau_us_));
        } else if (prev >= 0 && t < prev) {
            // Non-monotonic timestamp (time regression): reset potential,
            // matching jAER BlurringTunnelFilter's MP=0 on backward time.
            potential_[idx] = 0.0;
        }
        // Integrate: every event adds +1.0 regardless of polarity (per jAER
        // BlurringTunnelFilter, which feeds all neighbours uniformly).
        (void)p;
        potential_[idx] += 1.0;
        // Fire only when jump_after_firing_percent_ > 0 (jAER guards on
        // MPDecreaseArterFiringPercentTh > 0). When 0, use the legacy
        // full-reset path so existing callers keep their semantics.
        if (potential_[idx] >= threshold_) {
            if (jump_after_firing_percent_ > 0.0) {
                reduce_mp_after_firing(idx);
                return true;
            }
            potential_[idx] = reset_value_;
            return true;
        }
        return false;
    }

    /// @brief Globally leaks all neurons by @p dt_us. Call periodically to
    /// prevent stale potentials from accumulating on quiet cells.
    /// Also synchronises every cell's last-update timestamp to @p now so a
    /// later add_event() does not decay the same interval a second time
    /// (§四-低3). @p now defaults to the latest event timestamp seen so far.
    void leak_global(Metavision::timestamp dt_us,
                     Metavision::timestamp now = -1) {
        if (dt_us <= 0) return;
        if (now < 0) now = latest_t_;
        const double decay = std::exp(-static_cast<double>(dt_us) /
                                      static_cast<double>(tau_us_));
        for (auto& v : potential_) {
            v *= decay;
        }
        if (now >= 0) {
            std::fill(last_ts_.begin(), last_ts_.end(), now);
        }
    }

    /// @brief Returns the membrane potential at (x, y).
    double potential(int x, int y) const {
        return potential_[static_cast<std::size_t>(y) * width_ + x];
    }

    /// @brief Returns a flat potential image (CV_32F-compatible layout).
    const std::vector<double>& potential_grid() const { return potential_; }

    void clear() {
        std::fill(potential_.begin(), potential_.end(), initial_potential_);
        std::fill(last_ts_.begin(), last_ts_.end(),
                  static_cast<Metavision::timestamp>(-1));
        latest_t_ = -1;
    }

    int width() const { return width_; }
    int height() const { return height_; }
    double threshold() const { return threshold_; }
    Metavision::timestamp tau_us() const { return tau_us_; }
    double reset_value() const { return reset_value_; }
    double initial_potential_percent() const {
        return initial_potential_percent_;
    }
    double jump_after_firing_percent() const {
        return jump_after_firing_percent_;
    }

    void set_threshold(double t) {
        threshold_ = t;
        // Recompute the initial potential so future clear() calls use the
        // new threshold (existing potentials are left untouched, mirroring
        // jAER setMPThreshold which only updates thresholdMP).
        initial_potential_ = compute_initial_potential();
    }
    void set_tau_us(Metavision::timestamp tau) {
        tau_us_ = tau;
    }
    void set_initial_potential_percent(double p) {
        initial_potential_percent_ = p;
        initial_potential_ = compute_initial_potential();
    }
    void set_jump_after_firing_percent(double p) {
        jump_after_firing_percent_ = p;
    }

private:
    /// @brief Computes the per-cell initial MP from the current threshold and
    /// initial_potential_percent_ as percent * threshold / 100. 有意修正
    /// jAER LIFNeuron.reset 漏除 100 的上游 bug（jAER 实为
    /// MPInitialPercnetTh * MPThreshold，默认 750 而非 7.5）。
    /// Falls back to reset_value_ when the percent is non-positive.
    double compute_initial_potential() const {
        if (initial_potential_percent_ <= 0.0) return reset_value_;
        return initial_potential_percent_ * threshold_ / 100.0;
    }

    /// @brief Drops MP after firing (jAER LIFNeuron.reduceMPafterFiring):
    /// MPjump = max(threshold * pct / 100, 1.0); MP -= MPjump.
    void reduce_mp_after_firing(std::size_t idx) {
        double mp_jump = threshold_ * jump_after_firing_percent_ / 100.0;
        if (mp_jump < 1.0) mp_jump = 1.0;
        potential_[idx] -= mp_jump;
    }

    int width_;
    int height_;
    Metavision::timestamp tau_us_;
    double threshold_;
    double reset_value_;
    double initial_potential_percent_;
    double jump_after_firing_percent_;
    double initial_potential_;
    std::vector<double> potential_;
    std::vector<Metavision::timestamp> last_ts_;
    Metavision::timestamp latest_t_{-1};  ///< newest event timestamp seen
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_LIF_INTEGRATOR_H
