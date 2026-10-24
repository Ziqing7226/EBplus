// algo/cv/corner_detector.h — event-native corner detection and tracking.
//
// Three modes:
//   EndStopped      — ✅ 移植自 jAER EndStoppedOrientationLabeler. End-stopped
//                     cell simulation: for each event a 4-bin orientation is
//                     computed from a 2-channel (polarity) time surface via
//                     PCA of the 3x3 neighbour timestamp deltas; the receptive
//                     field is then walked along both the orientation and its
//                     opposite — if the run of recently-active pixels ends on
//                     BOTH sides within end_stopped_distance, a corner is
//                     emitted (a line ending in both directions).
//   TypeCoincidence — ✅ 移植自 jAER TypeCoincidenceFilter. Each event is
//                     oriented via the same PCA; if any neighbour in a
//                     configurable radius stores an ORTHOGONAL orientation
//                     ((ori+numOri/2)%numOri) whose timestamp is within
//                     coincidence_window_us, a corner is emitted (a corner is
//                     where two orthogonal edges coincide spatio-temporally).
//   Harris          — frame-based Harris on the accumulation frame
//                     (cv::cornerHarris) mapped back to event positions.
//                     (untouched, self-developed design §4.3.12)
//   Arc             — ✅ 移植自 dv-processing ArcCornerDetector (Arc*). For
//                     each event, two rings (radii 3 and 4) of the
//                     same-polarity time surface are searched for a
//                     contiguous arc of recent timestamps (spread <
//                     arc_corner_range_us) whose length is within
//                     [0.125, 0.4] x circumference, while every timestamp
//                     outside the arc is older than the arc minimum. The
//                     continuous response (min-inside minus max-outside, in
//                     us, averaged over both rings) goes to Corner::strength.
//                     Diffs from dv: is_recent pre-gate per event, only the
//                     small radius 3/4 templates (dv defaults to 5/6).
// Detected corners are tracked by nearest-neighbour matching; tracks shorter
// than min_track_len are suppressed. Output: vector<Corner>. Header-only.

#ifndef GUI_ALGO_CV_CORNER_DETECTOR_H
#define GUI_ALGO_CV_CORNER_DETECTOR_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
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

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief A detected and tracked corner.
struct Corner {
    float x{0.0F};
    float y{0.0F};
    float strength{0.0F};
    int track_id{-1};
    std::vector<cv::Point2f> trajectory;
};

/// @brief Multi-mode corner detector with nearest-neighbour tracking.
class CornerDetector {
public:
    enum class Mode { EndStopped, TypeCoincidence, Harris, Arc };

    CornerDetector(int width, int height, Mode mode = Mode::EndStopped)
        : width_(width), height_(height), mode_(mode) {
        reset();
    }

    // Parameters (defaults per design §4.3.12) ----------------------------
    void set_mode(Mode m) { mode_ = m; }
    void set_accumulation_ms(double v) { accumulation_ms_ = clamp_d(v, 1.0, 100.0); }
    void set_threshold(double v) { threshold_ = clamp_d(v, 1e-6, 1.0); }
    void set_track_radius_px(int v) { track_radius_px_ = clamp_i(v, 1, 30); }
    void set_min_track_len(int v) { min_track_len_ = clamp_i(v, 1, 100); }
    void set_output_hz(int v) { output_hz_ = clamp_i(v, 10, 500); }

    // TypeCoincidence params (✅ 移植自 jAER TypeCoincidenceFilter) ----------
    /// @brief jAER minDtThreshold: temporal coincidence window in us.
    void set_coincidence_window_us(int v) { coincidence_window_us_ = clamp_i(v, 1, 1000000); }
    /// @brief jAER dist: neighbourhood radius (pixels) searched for an
    ///        orthogonal-orientation coincidence.
    void set_neighborhood_radius(int v) { neighborhood_radius_ = clamp_i(v, 1, 5); }

    // EndStopped params (✅ 移植自 jAER EndStoppedOrientationLabeler) --------
    /// @brief jAER endStoppedLength: half-RF length walked along the orientation.
    void set_end_stopped_distance(int v) { end_stopped_distance_ = clamp_i(v, 1, 6); }
    /// @brief jAER maxDtToUse: max age (us) for a pixel to count as active.
    void set_max_age_us(int v) { max_age_us_ = clamp_i(v, 1000, 1000000); }

    // Arc params (✅ 移植自 dv-processing ArcCornerDetector, ring radii 3/4) -
    /// @brief dv cornerRange: max timestamp spread (us) allowed within a
    ///        corner arc. Default 5000 matches dv's documented code sample
    ///        (docs/.../feature_detection/sample1.cpp).
    void set_arc_corner_range_us(int v) { arc_corner_range_us_ = clamp_i(v, 100, 1000000); }
    /// @brief Min corner response (us) to emit. dv has no such threshold —
    ///        it emits any corner whose response is positive by construction;
    ///        the default 1us mirrors that, raise it to suppress weak corners.
    void set_arc_min_response_us(double v) { arc_min_response_us_ = clamp_d(v, 0.0, 1e9); }

    // Shared orientation params (jAER NUM_TYPES = 4) ----------------------
    /// @brief Number of orientation bins. Clamped to exactly 4: the
    /// EndStopped walker tables kBaseDx/kBaseDy have only 4 entries and are
    /// indexed with %4, so values > 4 would misalign bin semantics (§四-低6).
    void set_num_orientations(int v) { num_orientations_ = clamp_i(v, 4, 4); }

    Mode mode() const { return mode_; }
    double accumulation_ms() const { return accumulation_ms_; }
    double threshold() const { return threshold_; }
    int track_radius_px() const { return track_radius_px_; }
    int min_track_len() const { return min_track_len_; }
    int output_hz() const { return output_hz_; }
    int width() const { return width_; }
    int height() const { return height_; }
    int coincidence_window_us() const { return coincidence_window_us_; }
    int neighborhood_radius() const { return neighborhood_radius_; }
    int end_stopped_distance() const { return end_stopped_distance_; }
    int max_age_us() const { return max_age_us_; }
    int num_orientations() const { return num_orientations_; }
    int arc_corner_range_us() const { return arc_corner_range_us_; }
    double arc_min_response_us() const { return arc_min_response_us_; }

    /// @brief Accumulates events; runs detection + tracking when the
    ///        accumulation window elapses, emitting corners at output_hz.
    void process(const Event* events, std::size_t count) {
        ensure_mats();
        ensure_event_state();
        for (std::size_t i = 0; i < count; ++i) {
            const Event& e = events[i];
            if (e.x >= width_ || e.y >= height_) continue;
            const int x = static_cast<int>(e.x);
            const int y = static_cast<int>(e.y);
            // Frame accumulation (used by Harris; harmless for event modes).
            float& a = accum_.at<float>(y, x);
            if (a < 1e6F) a += 1.0F;
            if (e.p) {
                float& on = on_.at<float>(y, x);
                if (on < 1e6F) on += 1.0F;
            } else {
                float& off = off_.at<float>(y, x);
                if (off < 1e6F) off += 1.0F;
            }
            // Event-native corner detection (per-event).
            if (mode_ == Mode::TypeCoincidence) {
                process_event_type_coincidence(e, x, y);
            } else if (mode_ == Mode::EndStopped) {
                process_event_end_stopped(e, x, y);
            } else if (mode_ == Mode::Arc) {
                process_event_arc(e, x, y);
            }
            if (e.t > last_event_t_) last_event_t_ = e.t;
        }
        if (last_event_t_ - last_frame_t_ >= accum_us()) {
            detect_and_track();
            last_frame_t_ = last_event_t_;
        }
        if (last_event_t_ - last_emit_t_ >= emit_us()) {
            emit_corners();
            last_emit_t_ = last_event_t_;
        }
    }

    /// @brief Processes an event packet.
    void process(EventPacket& events) {
        process(events.data(), events.size());
    }

    /// @brief Returns the most recently emitted corners.
    const std::vector<Corner>& corners() const { return corners_; }

    void reset() {
        accum_ = cv::Mat();
        on_ = cv::Mat();
        off_ = cv::Mat();
        ori_surface_.clear();
        last_ori_.clear();
        last_ori_t_.clear();
        event_detected_.clear();
        tracks_.clear();
        corners_.clear();
        last_event_t_ = 0;
        last_frame_t_ = 0;
        last_emit_t_ = 0;
        next_track_id_ = 0;
    }

private:
    static int clamp_i(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    static double clamp_d(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    Metavision::timestamp accum_us() const {
        return static_cast<Metavision::timestamp>(accumulation_ms_ * 1000.0);
    }
    Metavision::timestamp emit_us() const {
        return static_cast<Metavision::timestamp>(1e6 / static_cast<double>(output_hz_));
    }

    void ensure_mats() {
        if (accum_.empty()) {
            accum_ = cv::Mat::zeros(height_, width_, CV_32FC1);
            on_ = cv::Mat::zeros(height_, width_, CV_32FC1);
            off_ = cv::Mat::zeros(height_, width_, CV_32FC1);
        }
    }

    void ensure_event_state() {
        if (ori_surface_.empty()) {
            ori_surface_.assign(
                static_cast<std::size_t>(2) * static_cast<std::size_t>(width_)
                    * static_cast<std::size_t>(height_),
                -1);  // -1 = never seen (0 is a legal timestamp, §四-低8)
            last_ori_.assign(
                static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_),
                -1);
            last_ori_t_.assign(
                static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_),
                0);
        }
    }

    /// @brief Two-channel (polarity) time-surface index. @p p must be 0 or 1.
    std::size_t idx_of(int x, int y, int p) const {
        return static_cast<std::size_t>(p) * static_cast<std::size_t>(width_)
                   * static_cast<std::size_t>(height_)
             + static_cast<std::size_t>(y) * static_cast<std::size_t>(width_)
             + static_cast<std::size_t>(x);
    }

    /// @brief Per-orientation along-direction unit offsets (jAER baseOffsets).
    /// ori 0 = horizontal (1,0), 1 = 45 deg (1,1), 2 = vertical (0,1),
    /// 3 = 135 deg (-1,1). EndStopped walks these integer offsets.
    static constexpr int kBaseDx[4] = {1, 1, 0, -1};
    static constexpr int kBaseDy[4] = {0, 1, 1, 1};

    /// Arc* ring templates (dv internal::CircleCoordinates), (dx,dy) pairs
    /// ordered circularly. Only the small radii 3/4 are ported — dv supports
    /// 3..7 and defaults to 5/6, but the per-event cost grows with ring
    /// length and the SDK thread budget does not allow it.
    static constexpr int kArcCircle3[16][2] = {
        {0, 3},  {1, 3},  {2, 2},  {3, 1},  {3, 0},  {3, -1}, {2, -2}, {1, -3},
        {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}};
    static constexpr int kArcCircle4[20][2] = {
        {0, 4},  {1, 4},  {2, 3},  {3, 2},  {4, 1},  {4, 0},  {4, -1},
        {3, -2}, {2, -3}, {1, -4}, {0, -4}, {-1, -4}, {-2, -3}, {-3, -2},
        {-4, -1}, {-4, 0}, {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}};

    /// @brief PCA orientation from the 3x3 same-polarity time surface (same
    ///        method as orientation_filter.h). Returns a bin in
    ///        [0,num_orientations_) or -1 if too few recent neighbours.
    int compute_orientation_pca(const Event& e) const {
        const double win = static_cast<double>(ori_time_window_us_);
        const int pol = e.p ? 1 : 0;
        double sx = 0.0, sy = 0.0, sxx = 0.0, syy = 0.0, sxy = 0.0, wsum = 0.0;
        int recent = 0;
        for (int dy = -1; dy <= 1; ++dy) {
            const int ny = static_cast<int>(e.y) + dy;
            if (ny < 0 || ny >= height_) continue;
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;  // exclude centre
                const int nx = static_cast<int>(e.x) + dx;
                if (nx < 0 || nx >= width_) continue;
                const Metavision::timestamp lt = ori_surface_[idx_of(nx, ny, pol)];
                if (lt < 0) continue;  // -1 = never seen
                const double diff = static_cast<double>(e.t - lt);
                if (diff < 0.0 || diff > win) continue;
                const double w = 1.0 - diff / win;     // freshness weight
                const double px = static_cast<double>(nx);
                const double py = static_cast<double>(ny);
                sx += w * px;
                sy += w * py;
                sxx += w * px * px;
                syy += w * py * py;
                sxy += w * px * py;
                wsum += w;
                ++recent;
            }
        }
        if (recent < ori_min_neighbors_ || wsum <= 0.0) return -1;
        const double cxx = sxx / wsum - (sx / wsum) * (sx / wsum);
        const double cyy = syy / wsum - (sy / wsum) * (sy / wsum);
        const double cxy = sxy / wsum - (sx / wsum) * (sy / wsum);
        double theta = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);  // [-pi/2, pi/2]
        if (theta < 0.0) theta += CV_PI;                        // [0, pi)
        const double deg = theta * 180.0 / CV_PI;
        const double bin = 180.0 / static_cast<double>(num_orientations_);
        int q = static_cast<int>(std::floor(deg / bin + 0.5));
        if (q >= num_orientations_) q = 0;
        return q;
    }

    /// @brief True if pixel (x,y) has any-polarity activity within max_age_us.
    bool is_recent(int x, int y, Metavision::timestamp t) const {
        const Metavision::timestamp lt0 = ori_surface_[idx_of(x, y, 0)];
        const Metavision::timestamp lt1 = ori_surface_[idx_of(x, y, 1)];
        Metavision::timestamp lt = lt0;
        if (lt1 > lt) lt = lt1;
        if (lt < 0) return false;  // -1 = never seen
        const Metavision::timestamp dt = t - lt;
        return dt >= 0 && dt <= max_age_us_;
    }

    // ✅ 移植自 jAER TypeCoincidenceFilter -------------------------------
    // For each event: orient it via PCA, then search a (2r+1)^2 neighbourhood
    // for a pixel whose stored orientation is orthogonal ((ori+numOri/2)%numOri)
    // and whose timestamp is within coincidence_window_us. If found, emit a
    // corner. The per-pixel (orientation, timestamp) map is single-channel
    // (any polarity), per the port spec.
    void process_event_type_coincidence(const Event& e, int x, int y) {
        const int ori = compute_orientation_pca(e);
        // Update the polarity time surface (after PCA, before storing).
        const int pol = e.p ? 1 : 0;
        ori_surface_[idx_of(x, y, pol)] = e.t;

        bool corner = false;
        if (ori >= 0) {
            const int orth = (ori + num_orientations_ / 2) % num_orientations_;
            const int r = neighborhood_radius_;
            for (int dy = -r; dy <= r && !corner; ++dy) {
                const int ny = y + dy;
                if (ny < 0 || ny >= height_) continue;
                for (int dx = -r; dx <= r && !corner; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    const int nx = x + dx;
                    if (nx < 0 || nx >= width_) continue;
                    const std::size_t ni =
                        static_cast<std::size_t>(ny) * static_cast<std::size_t>(width_)
                        + static_cast<std::size_t>(nx);
                    if (last_ori_[ni] != orth) continue;
                    const Metavision::timestamp dt = e.t - last_ori_t_[ni];
                    if (dt >= 0 && dt <= coincidence_window_us_) {
                        corner = true;
                    }
                }
            }
        }
        // Store current (orientation, timestamp) at pixel.
        const std::size_t ci =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(width_)
            + static_cast<std::size_t>(x);
        last_ori_[ci] = ori;
        last_ori_t_[ci] = e.t;
        if (corner) {
            event_detected_.push_back(
                Detected{static_cast<float>(x), static_cast<float>(y), 1.0F});
        }
    }

    // ✅ 移植自 jAER EndStoppedOrientationLabeler ------------------------
    // For each event: orient it via PCA, then walk integer offsets along the
    // orientation and its opposite. A line ending exists in a direction if the
    // run of recently-active pixels stops within end_stopped_distance (at least
    // `threshold` active pixels, then an inactive/OOB pixel). If BOTH sides
    // have line endings, a corner is emitted.
    void process_event_end_stopped(const Event& e, int x, int y) {
        const int ori = compute_orientation_pca(e);
        const int pol = e.p ? 1 : 0;
        ori_surface_[idx_of(x, y, pol)] = e.t;
        if (ori < 0) return;

        const int opp = (ori + num_orientations_ / 2) % num_orientations_;
        const int dx0 = kBaseDx[ori % 4];
        const int dy0 = kBaseDy[ori % 4];
        const int dx1 = kBaseDx[opp % 4];
        const int dy1 = kBaseDy[opp % 4];

        const bool end0 = has_line_ending(x, y, dx0, dy0, e.t);
        const bool end1 = has_line_ending(x, y, dx1, dy1, e.t);
        if (end0 && end1) {
            event_detected_.push_back(
                Detected{static_cast<float>(x), static_cast<float>(y), 1.0F});
        }
    }

    /// @brief Walks (dx,dy) from (x,y). A line ending exists if the run of
    ///        recently-active pixels (d=1..) has length in [min_run, dmax] and
    ///        is followed by an inactive / out-of-bounds pixel.
    bool has_line_ending(int x, int y, int dx, int dy,
                         Metavision::timestamp t) const {
        const int dmax = end_stopped_distance_;
        const int min_run = std::max(1, static_cast<int>(threshold_));
        int runlen = 0;
        for (int d = 1; d <= dmax + 1; ++d) {
            const int px = x + d * dx;
            const int py = y + d * dy;
            if (px < 0 || px >= width_ || py < 0 || py >= height_) break;
            if (!is_recent(px, py, t)) break;
            ++runlen;
        }
        return runlen >= min_run && runlen <= dmax;
    }

    // ✅ 移植自 dv-processing ArcCornerDetector (Arc*) ------------------------
    // Ported differences from dv (arc_corner_detector.hpp):
    //  - is_recent pre-gate: dv evaluates the full ring logic on EVERY event
    //    (~10x the EndStopped per-event cost), which would saturate the SDK
    //    thread; here the ring evaluation only runs for pixels that already
    //    showed any-polarity activity within max_age_us (2 lookups, same
    //    recency test as EndStopped's walker, checked against the surface
    //    state before the current event is written).
    //  - Only ring radii 3/4 (dv default 5/6), see kArcCircle3/4.
    //  - Our time surface stores -1 for never-seen (dv: 0); values are
    //    clamped to 0 on read so the timestamp arithmetic matches dv.
    void process_event_arc(const Event& e, int x, int y) {
        // Pre-gate: only evaluate pixels that showed any-polarity activity
        // within max_age_us BEFORE this event (2 lookups, same recency test
        // as EndStopped's walker). The surface itself is updated for every
        // event, as in dv — the gate only skips the ring evaluation.
        const bool recent = is_recent(x, y, e.t);
        const int pol = e.p ? 1 : 0;
        ori_surface_[idx_of(x, y, pol)] = e.t;
        if (!recent) return;
        // dv restricts the roi by radius2: the full radius-4 ring must be
        // inside the frame.
        constexpr int kRMax = 4;
        if (x < kRMax || x >= width_ - kRMax || y < kRMax || y >= height_ - kRMax) return;

        const auto ts_at = [&](int dx, int dy) -> Metavision::timestamp {
            const Metavision::timestamp v = ori_surface_[idx_of(x + dx, y + dy, pol)];
            return v < 0 ? 0 : v;  // dv semantics: never-seen reads as 0
        };

        const int (*rings[2])[2] = {kArcCircle3, kArcCircle4};
        const int ring_len[2] = {16, 20};
        double response = 0.0;
        bool is_corner = false;
        for (int ci = 0; ci < 2; ++ci) {
            const int (*ring)[2] = rings[ci];
            const int n = ring_len[ci];
            // The arc must contain the ring's max timestamp (dv: find it first).
            int max_i = 0;
            Metavision::timestamp max_v = ts_at(ring[0][0], ring[0][1]);
            for (int i = 1; i < n; ++i) {
                const Metavision::timestamp v = ts_at(ring[i][0], ring[i][1]);
                if (v > max_v) { max_v = v; max_i = i; }
            }
            if (max_v == 0) continue;  // dv: ring untouched, circle skipped
            // expandArc: grow a contiguous arc around max_i while timestamps
            // stay within arc_corner_range_us of the arc minimum.
            int begin = max_i, end = max_i, arc_size = 1;
            Metavision::timestamp min_in = max_v;
            bool begin_found = false, end_found = false;
            do {
                if (!begin_found) {
                    const int cand = (begin == 0) ? n - 1 : begin - 1;
                    const Metavision::timestamp v = ts_at(ring[cand][0], ring[cand][1]);
                    const Metavision::timestamp d = v > min_in ? v - min_in : min_in - v;
                    if (d < arc_corner_range_us_) {
                        begin = cand;
                        ++arc_size;
                        if (v < min_in) min_in = v;
                    } else {
                        begin_found = true;
                    }
                }
                if (!end_found) {
                    const int cand = (end + 1 == n) ? 0 : end + 1;
                    const Metavision::timestamp v = ts_at(ring[cand][0], ring[cand][1]);
                    const Metavision::timestamp d = v > min_in ? v - min_in : min_in - v;
                    if (d < arc_corner_range_us_) {
                        end = cand;
                        ++arc_size;
                        if (v < min_in) min_in = v;
                    } else {
                        end_found = true;
                    }
                }
            } while (arc_size < n && !(begin_found && end_found) && begin != end);

            if (!arc_size_ok(arc_size, n)) {
                response = 0.0;
                is_corner = false;
                break;
            }
            // checkSurroundingTimestamps: no timestamp outside the arc may
            // be newer than the arc minimum (early-out on first violation,
            // as in dv).
            Metavision::timestamp max_out = std::numeric_limits<Metavision::timestamp>::min();
            for (int i = (end + 1 == n) ? 0 : end + 1; i != begin;
                 i = (i + 1 == n) ? 0 : i + 1) {
                const Metavision::timestamp v = ts_at(ring[i][0], ring[i][1]);
                if (v > max_out) max_out = v;
                if (v > min_in) break;
            }
            if (min_in > max_out) {
                // dv response: (min inside - max outside), averaged over rings.
                response += static_cast<double>(min_in - max_out) / 2.0;
                is_corner = true;
            } else {
                response = 0.0;
                is_corner = false;
                break;
            }
        }
        if (is_corner && response >= arc_min_response_us_) {
            event_detected_.push_back(Detected{static_cast<float>(x),
                                               static_cast<float>(y),
                                               static_cast<float>(response)});
        }
    }

    /// @brief dv ArcLimits: arc length within [0.125, 0.4] x circumference;
    ///        the inverted arc is accepted as well (dv semantics).
    static bool arc_size_ok(int arc_size, int circumference) {
        const int min_size = static_cast<int>(std::lround(0.125 * circumference));
        const int max_size = static_cast<int>(std::lround(0.4 * circumference));
        if (arc_size >= min_size && arc_size <= max_size) return true;
        if (circumference < arc_size) return false;
        const int inverted = circumference - arc_size;
        return inverted >= min_size && inverted <= max_size;
    }

    struct Detected { float x, y, strength; };

    void detect_and_track() {
        std::vector<Detected> detected;
        switch (mode_) {
            case Mode::EndStopped:
            case Mode::TypeCoincidence:
            case Mode::Arc:
                detected.swap(event_detected_);
                break;
            case Mode::Harris:
                detect_harris(detected);
                break;
        }
        track(detected);
        // Reset accumulation frames after detection (Harris only needs this,
        // but resetting for all modes is harmless and keeps surfaces bounded).
        accum_.setTo(0.0F);
        on_.setTo(0.0F);
        off_.setTo(0.0F);
    }

    void detect_harris(std::vector<Detected>& out) {
        // Restrict the dense scan to the active region: cornerHarris +
        // dilate + the per-pixel collect_maxima loop are O(W×H) per
        // accumulation window, which at full sensor (1280×720) saturates the
        // pipeline every 10 ms window regardless of event count — the GUI
        // froze (latent defect exposed by the §五-A1 label fix; the true
        // Harris path had never run in the GUI before). Cost must be
        // proportional to activity, not resolution.
        cv::Mat active8;
        cv::compare(accum_, 0.0F, active8, cv::CMP_GT);
        cv::Rect bb = cv::boundingRect(active8);
        if (bb.empty()) return;
        // Pad so the 3x3 Sobel taps and the dilate kernel see the same
        // neighbourhood a full-frame scan would (clamped to the frame).
        constexpr int kPad = 8;
        bb = cv::Rect(std::max(0, bb.x - kPad), std::max(0, bb.y - kPad),
                      std::min(width_ - bb.x, bb.width + 2 * kPad),
                      std::min(height_ - bb.y, bb.height + 2 * kPad));
        cv::Mat strength;
        cv::cornerHarris(accum_(bb), strength, 3, 3, 0.04);
        cv::normalize(strength, strength, 0.0, 1.0, cv::NORM_MINMAX);
        collect_maxima(strength, out, bb.x, bb.y);
    }

    void collect_maxima(const cv::Mat& strength, std::vector<Detected>& out,
                        int x_off = 0, int y_off = 0) {
        const double thr = threshold_;
        const int sw = strength.cols;
        const int sh = strength.rows;
        int ksize = std::min(15, std::max(3, track_radius_px_ * 2 + 1));
        if ((ksize & 1) == 0) ++ksize;  // ensure odd
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(ksize, ksize));
        cv::Mat dil;
        cv::dilate(strength, dil, kernel);
        cv::Mat mask = (strength == dil) & (strength > thr);
        for (int y = 0; y < sh; ++y) {
            for (int x = 0; x < sw; ++x) {
                if (mask.at<uchar>(y, x)) {
                    out.push_back(Detected{static_cast<float>(x + x_off),
                                           static_cast<float>(y + y_off),
                                           strength.at<float>(y, x)});
                }
            }
        }
    }

    struct Track {
        int id{-1};
        float x{0.0F}, y{0.0F};
        float strength{0.0F};
        std::vector<cv::Point2f> traj;
        int len{0};
        int miss{0};
        bool updated{false};
    };

    void track(const std::vector<Detected>& detected) {
        for (auto& t : tracks_) t.updated = false;
        const float r = static_cast<float>(track_radius_px_);
        const float r2 = r * r;
        for (const auto& d : detected) {
            int best = -1;
            float best_d2 = r2;
            for (int k = 0; k < static_cast<int>(tracks_.size()); ++k) {
                if (!tracks_[k].updated) {
                    const float dx = tracks_[k].x - d.x;
                    const float dy = tracks_[k].y - d.y;
                    const float d2 = dx * dx + dy * dy;
                    if (d2 < best_d2) { best_d2 = d2; best = k; }
                }
            }
            if (best < 0) {
                Track t;
                t.id = next_track_id_++;
                t.x = d.x;
                t.y = d.y;
                t.strength = d.strength;
                t.traj.push_back(cv::Point2f(d.x, d.y));
                t.len = 1;
                t.updated = true;
                tracks_.push_back(t);
            } else {
                Track& t = tracks_[best];
                const float a = 0.4F;
                t.x = t.x * (1.0F - a) + d.x * a;
                t.y = t.y * (1.0F - a) + d.y * a;
                t.strength = d.strength;
                t.traj.push_back(cv::Point2f(t.x, t.y));
                if (t.traj.size() > 200) t.traj.erase(t.traj.begin());
                ++t.len;
                t.updated = true;
                t.miss = 0;
            }
        }
        // Age and prune unmatched tracks.
        std::vector<Track> kept;
        kept.reserve(tracks_.size());
        for (auto& t : tracks_) {
            if (!t.updated) ++t.miss;
            if (t.miss <= 3) kept.push_back(std::move(t));
        }
        tracks_.swap(kept);
    }

    void emit_corners() {
        corners_.clear();
        for (const auto& t : tracks_) {
            if (t.len < min_track_len_) continue;
            Corner c;
            c.x = t.x;
            c.y = t.y;
            c.strength = t.strength;
            c.track_id = t.id;
            c.trajectory = t.traj;
            corners_.push_back(c);
        }
    }

    int width_;
    int height_;
    Mode mode_;
    double accumulation_ms_{10.0};
    double threshold_{0.1};
    int track_radius_px_{5};
    int min_track_len_{10};
    int output_hz_{100};

    // TypeCoincidence / EndStopped params (jAER ports).
    int coincidence_window_us_{10000};   // jAER minDtThreshold
    int neighborhood_radius_{1};         // jAER dist
    int num_orientations_{4};            // jAER NUM_TYPES
    int end_stopped_distance_{3};        // jAER endStoppedLength
    int max_age_us_{40000};              // jAER maxDtToUse
    int ori_time_window_us_{10000};      // PCA neighbour freshness window
    int ori_min_neighbors_{2};           // PCA min recent neighbours
    int arc_corner_range_us_{5000};      // dv cornerRange (sample default)
    double arc_min_response_us_{1.0};    // dv: none (any positive response)

    // Harris accumulation frames.
    cv::Mat accum_;
    cv::Mat on_;
    cv::Mat off_;

    // Event-native state.
    std::vector<Metavision::timestamp> ori_surface_;  // 2 * w * h (polarity)
    std::vector<int> last_ori_;                       // w * h, last bin or -1
    std::vector<Metavision::timestamp> last_ori_t_;   // w * h
    std::vector<Detected> event_detected_;            // per-window corners

    std::vector<Track> tracks_;
    std::vector<Corner> corners_;
    Metavision::timestamp last_event_t_{0};
    Metavision::timestamp last_frame_t_{0};
    Metavision::timestamp last_emit_t_{0};
    int next_track_id_{0};
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_CORNER_DETECTOR_H
