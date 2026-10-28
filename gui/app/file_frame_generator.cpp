// gui/app/file_frame_generator.cpp

#include "file_frame_generator.h"

#include <algorithm>

#include <opencv2/imgproc.hpp>

#include "algo_bridge/filter_chain.h"
#include "frame_mode_renderer.h"

namespace gui {

void FileFrameGenerator::set_algo_feed_callback(
    std::function<std::pair<const Metavision::EventCD*, std::size_t>()> cb) {
    algo_feed_cb_ = std::move(cb);
}

FileFrameGenerator::FileFrameGenerator(QObject* parent) : QObject(parent) {
    timer_.setTimerType(Qt::PreciseTimer);
    connect(&timer_, &QTimer::timeout, this, &FileFrameGenerator::on_timer);
}

FileFrameGenerator::~FileFrameGenerator() {
    timer_.stop();
}

void FileFrameGenerator::add_events(const Metavision::EventCD* begin,
                                    const Metavision::EventCD* end) {
    if (begin == nullptr || end == nullptr || begin >= end) return;
    bool just_truncated = false;
    bool pause_loader = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // OOM guard (audit §六-C2): never let the buffer grow past
        // kMaxBufferedEvents. A batch that would cross the cap is only
        // appended up to the cap; the rest is dropped and reported once.
        const std::size_t room = kMaxBufferedEvents - events_.size();
        const std::size_t n = static_cast<std::size_t>(end - begin);
        if (room > 0) {
            const Metavision::EventCD* append_end =
                begin + std::min(n, room);
            events_.insert(events_.end(), begin, append_end);
            // Duration = last buffered event timestamp. Updated atomically
            // so on_timer() (GUI thread) can read it without locking.
            const Metavision::timestamp last_t = (append_end - 1)->t;
            Metavision::timestamp cur =
                duration_us_.load(std::memory_order_relaxed);
            while (last_t > cur) {
                if (duration_us_.compare_exchange_weak(
                        cur, last_t, std::memory_order_relaxed)) {
                    break;
                }
            }
            if (streaming_) {
                // A pending reposition is satisfied once events at/after the
                // target have arrived.
                if (awaiting_seek_.load(std::memory_order_acquire) &&
                    last_t >= seek_target_.load(std::memory_order_relaxed)) {
                    awaiting_seek_.store(false, std::memory_order_release);
                }
                // Bounded buffer: drop events far behind the cursor, then
                // pause the loader at the high-water span. Events arrive
                // time-sorted, so the hold-back horizon is a scan from the
                // previous trim position; the vector prefix is erased in
                // batches to keep the amortized cost low.
                const Metavision::timestamp holdback_t =
                    cursor_us_.load(std::memory_order_relaxed) - kStreamHoldBackUs;
                while (trim_start_ < events_.size() &&
                       events_[trim_start_].t < holdback_t) {
                    ++trim_start_;
                }
                constexpr std::size_t kTrimCompactEvents = 4'000'000;
                if (trim_start_ >= kTrimCompactEvents) {
                    events_.erase(events_.begin(),
                                  events_.begin() + static_cast<std::ptrdiff_t>(trim_start_));
                    trim_start_ = 0;
                }
                if (!stream_paused_.load(std::memory_order_acquire) &&
                    last_t - cursor_us_.load(std::memory_order_relaxed) >
                        kStreamHighWaterUs) {
                    stream_paused_.store(true, std::memory_order_release);
                    pause_loader = true;
                }
            }
        }
        if (n > room && !truncated_) {
            truncated_ = true;
            just_truncated = true;
        }
    }
    // Emit outside the lock; Qt queues this to GUI-thread listeners.
    if (just_truncated) {
        emit buffer_truncated();
    }
    if (pause_loader) {
        emit stream_pause_requested();
    }
}

void FileFrameGenerator::set_geometry(long width, long height) {
    if (width <= 0 || height <= 0) return;
    const bool changed = width_ != width || height_ != height;
    width_ = width;
    height_ = height;
    if (changed || frame_.empty()) {
        frame_.create(static_cast<int>(height_), static_cast<int>(width_), CV_8UC3);
    }
    conditioner_.init(static_cast<int>(width), static_cast<int>(height));
    // Recompute the software ROI rect against the new sensor size.
    set_display_roi(roi_enabled_, roi_x_, roi_y_, roi_w_, roi_h_, roi_roni_);
}

void FileFrameGenerator::set_display_roi(bool enabled, int x, int y, int w, int h,
                                         bool roni) {
    roi_enabled_ = enabled;
    roi_roni_ = roni;
    roi_x_ = x; roi_y_ = y; roi_w_ = w; roi_h_ = h;
    // Compute the rect (auto-center on -1, clamp to sensor), mirroring
    // ProcessRegion::compute and CameraController::set_unified_roi.
    const int sw = width_ > 0 ? static_cast<int>(width_) : 1280;
    const int sh = height_ > 0 ? static_cast<int>(height_) : 720;
    const int rw = (w <= 0) ? sw : std::min(w, sw);
    const int rh = (h <= 0) ? sh : std::min(h, sh);
    const int rx = (x < 0) ? (sw - rw) / 2 : std::min(std::max(0, x), sw - rw);
    const int ry = (y < 0) ? (sh - rh) / 2 : std::min(std::max(0, y), sh - rh);
    roi_x0_ = rx; roi_y0_ = ry;
    roi_x1_ = rx + rw; roi_y1_ = ry + rh;
    conditioner_.set_roi(enabled, roi_x0_, roi_y0_, roi_x1_, roi_y1_, roni);
    // Display placement: the frame stays FULL-sensor (ROI content shifted
    // back to its absolute position for rendering; the rest stays the
    // palette background — pre-rework visuals the overlay / Zoom-to-ROI /
    // Replace-composite strategies are built around). RONI keeps absolute
    // coordinates, so no shift there.
    set_render_origin(enabled && !roni, roi_x0_, roi_y0_);
}

void FileFrameGenerator::set_fps(std::uint16_t fps) {
    if (fps == 0) fps = 1;
    fps_ = fps;
    if (timer_.isActive()) {
        timer_.setInterval(1000 / static_cast<int>(fps_));
    }
}

void FileFrameGenerator::set_accumulation_time_us(Metavision::timestamp us) {
    if (us < 1) us = 1;
    accumulation_us_ = us;
}

void FileFrameGenerator::set_color_palette(Metavision::ColorPalette palette) {
    palette_ = palette;
}

void FileFrameGenerator::set_duration_us(Metavision::timestamp us) {
    Metavision::timestamp cur = duration_us_.load(std::memory_order_relaxed);
    while (us > cur) {
        if (duration_us_.compare_exchange_weak(cur, us,
                                               std::memory_order_relaxed)) {
            break;
        }
    }
}

void FileFrameGenerator::play() {
    if (playing_) return;
    if (width_ <= 0 || height_ <= 0) return;
    // If at or past EOF, restart from the beginning. Only meaningful once
    // loading is complete — a cursor parked at the buffer top while the
    // file is still streaming is NOT EOF (audit §六-P2).
    const Metavision::timestamp dur = duration_us_.load(std::memory_order_relaxed);
    if (dur > 0 && cursor_us_ >= dur &&
        loading_complete_.load(std::memory_order_acquire)) {
        // Same contract as seek()/looped(): stateful algorithms must reset
        // before events from the beginning of the file arrive (audit §六-P4).
        if (streaming_) {
            loader_reposition(0);
        } else {
            cursor_us_ = 0;
            emit seeked(0);
        }
    }
    playing_ = true;
    timer_.start(1000 / static_cast<int>(fps_));
}

void FileFrameGenerator::pause() {
    if (!playing_) return;
    timer_.stop();
    playing_ = false;
}

void FileFrameGenerator::seek(Metavision::timestamp t_us) {
    if (t_us < 0) t_us = 0;
    // Clamp to the known duration so a Step past EOF can't park the cursor
    // beyond the last event (the next Play would then restart from 0 as if
    // EOF had been reached — audit §六-P5).
    const Metavision::timestamp dur = duration_us_.load(std::memory_order_relaxed);
    if (dur > 0 && t_us > dur) t_us = dur;
    if (streaming_) {
        // Streaming mode: only a window around the cursor is buffered. A
        // target inside the buffer is a plain cursor move; anything else
        // needs the loader to reposition (which clears the buffer).
        bool inside = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            inside = trim_start_ < events_.size() &&
                     t_us >= events_[trim_start_].t &&
                     t_us <= events_.back().t;
        }
        if (!inside) {
            loader_reposition(t_us);
            return;
        }
        stream_paused_.store(false, std::memory_order_release);
        emit stream_resume_requested();
    }
    cursor_us_ = t_us;
    // Display filter temporal state must not span a backward time jump
    // (Phase 2.5) — stale timestamp surfaces would suppress events.
    conditioner_.reset_temporal();
    // Notify listeners so stateful algorithms can reset their temporal state
    // before the new (possibly earlier) events arrive. Without this, a
    // backward seek leaves algorithm timestamps ahead of the new events,
    // causing them to be ignored and the output to freeze — the same issue
    // as looped() but triggered by user-initiated cursor jumps.
    emit seeked(t_us);
    // Render immediately so the user sees the seeked frame.
    render_frame(cursor_us_, cursor_us_ + accumulation_us_);
    if (width_ > 0 && height_ > 0 && !frame_.empty()) {
        cv::Mat rgb;
        cv::cvtColor(frame_, rgb, cv::COLOR_BGR2RGB);
        QImage img(rgb.data, rgb.cols, rgb.rows,
                   static_cast<int>(rgb.step), QImage::Format_RGB888);
        QImage copy = img.copy();
        emit frame_ready(std::move(copy), cursor_us_);
    }
    emit position_changed(cursor_us_,
                          duration_us_.load(std::memory_order_relaxed));
}

Metavision::timestamp FileFrameGenerator::duration_us() const {
    return duration_us_.load(std::memory_order_relaxed);
}

std::size_t FileFrameGenerator::event_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return events_.size();
}

void FileFrameGenerator::clear() {
    timer_.stop();
    playing_ = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        events_.clear();
        truncated_ = false;
    }
    duration_us_.store(0, std::memory_order_relaxed);
    // A new file is about to stream in: suspend EOF handling until the
    // loader signals completion (audit §六-P2).
    loading_complete_.store(false, std::memory_order_release);
    cursor_us_ = 0;
    // Streaming state (bounded-buffer playback).
    trim_start_ = 0;
    stream_paused_.store(false, std::memory_order_release);
    awaiting_seek_.store(false, std::memory_order_release);
    seek_target_.store(0, std::memory_order_relaxed);
}

bool FileFrameGenerator::is_truncated() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return truncated_;
}

void FileFrameGenerator::loader_reposition(Metavision::timestamp t_us) {
    // Drop everything — the loader will re-stream from the new position.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        events_.clear();
        trim_start_ = 0;
        truncated_ = false;
    }
    cursor_us_ = t_us;
    seek_target_.store(t_us, std::memory_order_relaxed);
    awaiting_seek_.store(true, std::memory_order_release);
    stream_paused_.store(false, std::memory_order_release);
    // Display filter temporal state must not span the time jump (Phase 2.5)
    // and stateful algorithms reset via seeked(), same contract as seek().
    conditioner_.reset_temporal();
    emit seeked(t_us);
    emit loader_seek_requested(t_us);
}

void FileFrameGenerator::maybe_resume_loader() {
    if (!stream_paused_.load(std::memory_order_acquire)) return;
    Metavision::timestamp newest = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!events_.empty()) newest = events_.back().t;
    }
    const Metavision::timestamp cur = cursor_us_.load(std::memory_order_relaxed);
    const Metavision::timestamp dur = duration_us_.load(std::memory_order_relaxed);
    // Resume when the buffered span drains below the low-water mark, or when
    // the loader is close enough to the file end that it must run to EOF.
    if (newest - cur < kStreamLowWaterUs || (dur > 0 && newest >= dur)) {
        stream_paused_.store(false, std::memory_order_release);
        emit stream_resume_requested();
    }
}

void FileFrameGenerator::on_timer() {
    if (width_ <= 0 || height_ <= 0) return;
    // A loader reposition (out-of-buffer seek / loop wrap / EOF restart) is
    // in flight: hold the last frame until events at the new position arrive.
    if (awaiting_seek_.load(std::memory_order_acquire)) return;

    const Metavision::timestamp dur = duration_us_.load(std::memory_order_relaxed);

    // EOF / buffer-wait check (audit §六-P2). duration_us_ is only the max
    // timestamp buffered SO FAR, so a cursor at/past it means one of two
    // things:
    //   - loading complete  → genuine EOF: stop (emit eof_reached) or,
    //     in loop mode, wrap to 0 (emit looped).
    //   - still loading     → the cursor merely caught up with the read
    //     progress: wait silently (no advance, no EOF, NO WRAP) until more
    //     events are buffered or loading completes. playing_ stays true.
    //
    // The wait applies to loop mode too (§12.2-A revisited): wrapping while
    // loading replays the buffered prefix over and over — with a tiny
    // accumulation window the cursor outruns the loader within the first
    // milliseconds and the first window's frame is re-emitted repeatedly
    // (user report: "开头反复闪烁同一个累积帧"). The edcfbf3 concern (loop
    // never wraps if loading_complete_ is never set) is handled by the
    // loader's reliable completion signal (camera_controller.cpp:268);
    // a genuinely wedged loader stalls playback instead of flashing —
    // the lesser evil.
    if (dur > 0 && cursor_us_ >= dur) {
        if (!loading_complete_.load(std::memory_order_acquire)) {
            // Streaming mode: the loader may be parked at the high-water
            // mark — resume it so it can reach EOF; without this the
            // catch-up wait would never end.
            maybe_resume_loader();
            return;
        }
        if (loop_) {
            // Display filter temporal state must not span the loop wrap
            // (Phase 2.5) — event time jumps back to the start of the file.
            conditioner_.reset_temporal();
            emit looped();
            if (streaming_) {
                // The start of the file is no longer buffered: reposition.
                loader_reposition(0);
                return;
            }
            cursor_us_ = 0;
        } else {
            timer_.stop();
            playing_ = false;
            emit eof_reached();
            return;
        }
    } else if (streaming_) {
        maybe_resume_loader();
    }

    const Metavision::timestamp start = cursor_us_;
    const Metavision::timestamp end = start + accumulation_us_;

    // Render events in [start, end) to frame_.
    render_frame(start, end);

    // Convert BGR → RGB and emit. The QImage owns its buffer and cvtColor
    // writes straight into it — one allocation, no second full-frame copy.
    if (!frame_.empty()) {
        QImage img(frame_.cols, frame_.rows, QImage::Format_RGB888);
        if (!img.isNull()) {
            cv::Mat dst_view(frame_.rows, frame_.cols, CV_8UC3, img.bits(),
                             static_cast<size_t>(img.bytesPerLine()));
            cv::cvtColor(frame_, dst_view, cv::COLOR_BGR2RGB);
            emit frame_ready(std::move(img), start);
        }
    }

    cursor_us_ = end;
    emit position_changed(cursor_us_, dur);
}

void FileFrameGenerator::render_frame(Metavision::timestamp start_us,
                                      Metavision::timestamp end_us) {
    if (frame_.empty()) {
        if (width_ > 0 && height_ > 0) {
            frame_.create(static_cast<int>(height_),
                          static_cast<int>(width_), CV_8UC3);
        } else {
            return;
        }
    }

    // Use the Metavision color palette (same as CDFrameGenerator).
    const cv::Vec3b bg   = Metavision::get_bgr_color(palette_, Metavision::ColorType::Background);
    const cv::Vec3b on   = Metavision::get_bgr_color(palette_, Metavision::ColorType::Positive);
    const cv::Vec3b off  = Metavision::get_bgr_color(palette_, Metavision::ColorType::Negative);
    frame_.setTo(cv::Scalar(bg[0], bg[1], bg[2]));

    // Collect the buffered events in [start_us, end_us) — raw; conditioning
    // (ROI / FilterChain / noise filter / downsample / undistort) runs below,
    // once, for BOTH the rendered pixels and the algorithm feed.
    auto window_events = std::make_shared<std::vector<Metavision::EventCD>>();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!events_.empty()) {
            // Events are sorted by timestamp (SDK guarantee). Binary search
            // for the window boundaries.
            auto begin_it = std::lower_bound(
                events_.begin(), events_.end(), start_us,
                [](const Metavision::EventCD& e, Metavision::timestamp t) {
                    return e.t < t;
                });
            auto end_it = std::lower_bound(
                events_.begin(), events_.end(), end_us,
                [](const Metavision::EventCD& e, Metavision::timestamp t) {
                    return e.t < t;
                });
            window_events->assign(begin_it, end_it);
        }
    }

    // Shared conditioning (2026-08-19): ONE pass per rendered window —
    // unified ROI (crop+shift / RONI drop-inside) → FilterChain value stages
    // → noise filter → 1/4 thin → undistort → FilterChain flips. The SAME
    // output feeds the rendered pixels AND events_window_ready; algorithm
    // instances no longer re-run the filter per instance.
    const auto [cond_p, cond_n] = conditioner_.apply(
        window_events->data(), window_events->data() + window_events->size());
    // The conditioned span points into the conditioner's buffers — copy back
    // into the shared_ptr the signal owns. This vector is the CANONICAL
    // (ROI-relative) stream the algorithm feed emits.
    window_events->assign(cond_p, cond_p + cond_n);

    // DISPLAY placement: the frame stays FULL-sensor; shift the conditioned
    // events back by the ROI origin for rendering only (no-op when the ROI
    // is off or in RONI mode). The rest of the frame keeps the palette
    // background — the pre-rework visuals the ROI overlay / Zoom-to-ROI
    // crop / Replace-composite strategies are built around.
    const std::vector<Metavision::EventCD>* draw_events = window_events.get();
    if (render_origin_active_ && !window_events->empty()) {
        render_shift_buf_.clear();
        render_shift_buf_.reserve(window_events->size());
        const std::uint16_t ox = static_cast<std::uint16_t>(render_origin_x_);
        const std::uint16_t oy = static_cast<std::uint16_t>(render_origin_y_);
        for (const auto& ev : *window_events) {
            render_shift_buf_.push_back(ev);
            render_shift_buf_.back().x = ev.x + ox;
            render_shift_buf_.back().y = ev.y + oy;
        }
        draw_events = &render_shift_buf_;
    }
    const int h = static_cast<int>(height_);
    const int w = static_cast<int>(width_);
    // Non-integration frame modes: feed the window events to the shared
    // frame-mode renderer and use its generated frame for display. Otherwise
    // keep the palette event render (Integration mode).
    if (frame_mode_renderer_ && frame_mode_renderer_->active()) {
        frame_mode_renderer_->add_events(draw_events->data(),
                                         draw_events->data() + draw_events->size());
        cv::Mat mode_frame = frame_mode_renderer_->generate();
        if (!mode_frame.empty() && mode_frame.size() == frame_.size()) {
            frame_ = mode_frame;
        } else {
            frame_.setTo(cv::Scalar(bg[0], bg[1], bg[2]));
        }
    } else {
        for (const auto& ev : *draw_events) {
            if (ev.x < 0 || ev.x >= w || ev.y < 0 || ev.y >= h) continue;
            frame_.ptr<cv::Vec3b>(static_cast<int>(ev.y))[ev.x] = ev.p ? on : off;
        }
    }

    // Emit the CONDITIONED (canonical, ROI-relative) events in this window
    // so algorithm instances can process them synchronously with the
    // displayed frame — backends run at ROI dims, so unlike the display
    // feed above these are NOT shifted back. Emitted before frame_ready so
    // results are ready when the frame is displayed.
    emit events_window_ready(window_events, start_us);

    // P6-A file-mode write-back: if a stream-filter algorithm exposed an
    // output span for this window (hot_pixel_filter removed events, EIS
    // compensated coordinates), REPAINT the frame from those events so the
    // filtering is visible during playback, matching the live mode.
    if (algo_feed_cb_) {
        auto [fb, fn] = algo_feed_cb_();
        if (fb != nullptr && fn > 0) {
            const Metavision::EventCD* draw2 = fb;
            std::vector<Metavision::EventCD> shifted;
            if (render_origin_active_) {
                shifted.reserve(fn);
                for (std::size_t i = 0; i < fn; ++i) {
                    shifted.push_back(fb[i]);
                    shifted.back().x = fb[i].x + static_cast<std::uint16_t>(render_origin_x_);
                    shifted.back().y = fb[i].y + static_cast<std::uint16_t>(render_origin_y_);
                }
                draw2 = shifted.data();
            }
            frame_.setTo(cv::Scalar(bg[0], bg[1], bg[2]));
            for (std::size_t i = 0; i < fn; ++i) {
                const auto& ev = draw2[i];
                if (ev.x >= static_cast<std::uint16_t>(w) ||
                    ev.y >= static_cast<std::uint16_t>(h)) continue;
                frame_.ptr<cv::Vec3b>(ev.y)[ev.x] = ev.p ? on : off;
            }
        }
    }
}

} // namespace gui
