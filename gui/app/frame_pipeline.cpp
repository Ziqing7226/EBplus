// gui/app/frame_pipeline.cpp

#include "frame_pipeline.h"

#include <opencv2/imgproc.hpp>

namespace gui {

FramePipeline::FramePipeline(QObject* parent) : QObject(parent) {
    // Forward FileFrameGenerator signals to FramePipeline signals.
    connect(&file_generator_, &FileFrameGenerator::frame_ready,
            this, &FramePipeline::frame_ready);
    connect(&file_generator_, &FileFrameGenerator::position_changed,
            this, &FramePipeline::file_position_changed);
    connect(&file_generator_, &FileFrameGenerator::eof_reached,
            this, &FramePipeline::file_eof_reached);
    connect(&file_generator_, &FileFrameGenerator::looped,
            this, &FramePipeline::file_looped);
    connect(&file_generator_, &FileFrameGenerator::seeked,
            this, &FramePipeline::file_seeked);
    connect(&file_generator_, &FileFrameGenerator::events_window_ready,
            this, &FramePipeline::events_window_ready);
    connect(&file_generator_, &FileFrameGenerator::buffer_truncated,
            this, &FramePipeline::file_buffer_truncated);

    // File-path frame-mode rendering: render_frame feeds the shared renderer.
    file_generator_.set_frame_mode_renderer(&frame_mode_renderer_);
    // Stateful frame modes (TimeDecay / EventsIntegration) must not span a
    // backward time jump: reset on seek and loop.
    connect(this, &FramePipeline::file_seeked, this,
            [this](Metavision::timestamp) { frame_mode_renderer_.reset(); });
    connect(this, &FramePipeline::file_looped, this,
            [this] { frame_mode_renderer_.reset(); });

    // Live-mode tick for non-integration frame modes: generate the current
    // mode's frame at fps from the accumulated events.
    frame_mode_timer_ = new QTimer(this);
    frame_mode_timer_->setTimerType(Qt::PreciseTimer);
    connect(frame_mode_timer_, &QTimer::timeout,
            this, &FramePipeline::on_frame_mode_tick);
}

FramePipeline::~FramePipeline() {
    stop();
}

std::uint16_t FramePipeline::clamp_fps(std::uint16_t fps) const {
    if (fps < 1) fps = 1;
    if (fps > fps_limit_) fps = fps_limit_;
    return fps;
}

bool FramePipeline::start(long width, long height,
                          std::uint16_t fps,
                          Metavision::timestamp accumulation_time_us) {
    if (is_running()) {
        return false;
    }
    if (width <= 0 || height <= 0) {
        return false;
    }
    width_  = width;
    height_ = height;
    fps_    = clamp_fps(fps);
    accumulation_us_ = accumulation_time_us;
    file_mode_ = false;
    generator_ = std::make_unique<gui_algo::FrameGenerator>(width_, height_);
    recreate_window();
    frame_mode_renderer_.set_geometry(static_cast<int>(width_),
                                      static_cast<int>(height_));
    frame_mode_renderer_.set_mode(frame_mode_);
    frame_mode_renderer_.reset();
    update_frame_mode_timer();
    return window_id_ >= 0;
}

bool FramePipeline::start_file(long width, long height,
                               std::uint16_t fps,
                               Metavision::timestamp accumulation_time_us) {
    if (is_running()) {
        return false;
    }
    if (width <= 0 || height <= 0) {
        return false;
    }
    width_  = width;
    height_ = height;
    fps_    = clamp_fps(fps);
    accumulation_us_ = accumulation_time_us;
    file_mode_ = true;
    file_generator_.clear();
    file_generator_.set_geometry(width_, height_);
    file_generator_.set_fps(fps_);
    file_generator_.set_accumulation_time_us(accumulation_us_);
    frame_mode_renderer_.set_geometry(static_cast<int>(width_),
                                      static_cast<int>(height_));
    frame_mode_renderer_.set_mode(frame_mode_);
    frame_mode_renderer_.reset();
    return true;
}

void FramePipeline::recreate_window() {
    if (!generator_) {
        return;
    }
    if (window_id_ >= 0) {
        generator_->remove_window(window_id_);
        window_id_ = -1;
    }
    gui_algo::FrameGenerator::WindowParams params;
    params.fps = fps_;
    params.accumulation_time_us = accumulation_us_;

    // The callback runs on CDFrameGenerator's internal thread. We must deep
    // copy the cv::Mat before emitting, since the SDK reuses it.
    window_id_ = generator_->add_window(
        "main", params,
        [this](Metavision::timestamp ts, cv::Mat& frame) {
            // Non-integration frame modes emit via the mode tick instead.
            if (frame_mode_ != FrameMode::Integration) {
                return;
            }
            if (frame.empty()) {
                return;
            }
            try {
                // The QImage owns its buffer from the start and cvtColor
                // writes straight into it through a stride-aware view — one
                // allocation and no second full-frame copy (the emitted
                // image uniquely owns its data, so the SDK-thread/GUI-thread
                // handoff stays safe without the former deep copy).
                QImage img(frame.cols, frame.rows, QImage::Format_RGB888);
                if (!img.isNull()) {
                    cv::Mat dst_view(frame.rows, frame.cols, CV_8UC3, img.bits(),
                                     static_cast<size_t>(img.bytesPerLine()));
                    if (frame.channels() == 1) {
                        cv::cvtColor(frame, dst_view, cv::COLOR_GRAY2RGB);
                    } else {
                        cv::cvtColor(frame, dst_view, cv::COLOR_BGR2RGB);
                    }
                    emit frame_ready(std::move(img), ts);
                }
            } catch (const std::exception&) {
                // Swallow — the SDK thread must not propagate exceptions
            } catch (...) {}
        });
}

void FramePipeline::stop() {
    if (frame_mode_timer_) frame_mode_timer_->stop();
    if (file_mode_) {
        file_generator_.pause();
        file_generator_.clear();
        file_mode_ = false;
    }
    if (generator_) {
        generator_->remove_window(window_id_);
        generator_.reset();
    }
    window_id_ = -1;
    frame_mode_renderer_.reset();
}

void FramePipeline::add_events(const Metavision::EventCD* begin,
                               const Metavision::EventCD* end) {
    if (file_mode_) {
        file_generator_.add_events(begin, end);
    } else if (generator_) {
        // Live mode: events arrive ALREADY conditioned (unified ROI →
        // polarity stages → noise filter → 1/4 thin → undistort → flips —
        // CameraController::conditioner_), in ROI-RELATIVE coordinates when
        // the unified ROI is active. The DISPLAY renders on the full-sensor
        // frame (ROI content at its absolute position, the rest stays
        // background — the Zoom-to-ROI checkbox crops the finished frame),
        // so shift events back by the ROI origin for the display feed.
        const Metavision::EventCD* out_b = begin;
        const Metavision::EventCD* out_e = end;
        if (display_roi_active_) {
            const std::uint16_t ox = static_cast<std::uint16_t>(display_roi_x0_);
            const std::uint16_t oy = static_cast<std::uint16_t>(display_roi_y0_);
            const int fw = static_cast<int>(width_);
            const int fh = static_cast<int>(height_);
            display_shift_buf_.clear();
            display_shift_buf_.reserve(static_cast<std::size_t>(end - begin));
            for (const auto* p = begin; p != end; ++p) {
                const int nx = p->x + ox;
                const int ny = p->y + oy;
                // A rect change can race one batch against the new origin —
                // skip events that would land outside the frame (the SDK
                // generator has no per-event bounds check).
                if (nx < 0 || nx >= fw || ny < 0 || ny >= fh) continue;
                display_shift_buf_.push_back(*p);
                display_shift_buf_.back().x = static_cast<std::uint16_t>(nx);
                display_shift_buf_.back().y = static_cast<std::uint16_t>(ny);
            }
            out_b = display_shift_buf_.data();
            out_e = display_shift_buf_.data() + display_shift_buf_.size();
        }
        // Processed-stream recording + the frame renderers all see the same
        // ABSOLUTE-coordinate display span (pre-rework recording semantics).
        std::lock_guard<std::mutex> lk(processed_mutex_);
        if (processed_listener_) processed_listener_(out_b, out_e);
        // Non-integration frame modes feed the frame-mode renderer (their
        // tick emits frame_ready); the CDFrameGenerator path is bypassed.
        if (frame_mode_ != FrameMode::Integration) {
            frame_mode_renderer_.add_events(out_b, out_e);
            if (out_e > out_b) {
                last_ev_ts_.store((out_e - 1)->t, std::memory_order_relaxed);
            }
        } else {
            generator_->add_events(out_b, out_e);
        }
    }
}

void FramePipeline::set_display_roi_origin(bool active, int x0, int y0) {
    display_roi_active_ = active;
    display_roi_x0_ = x0;
    display_roi_y0_ = y0;
    file_generator_.set_render_origin(active, x0, y0);
}

void FramePipeline::set_accumulation_time_us(Metavision::timestamp us) {
    if (us == accumulation_us_) return;
    accumulation_us_ = us;
    if (file_mode_) {
        file_generator_.set_accumulation_time_us(us);
    } else if (generator_) {
        generator_->set_accumulation_time_us(us);
    }
    emit accumulation_time_changed(us);
}

void FramePipeline::set_fps(std::uint16_t fps) {
    fps = clamp_fps(fps);
    if (fps == fps_) return;
    fps_ = fps;
    if (file_mode_) {
        file_generator_.set_fps(fps);
    } else if (generator_) {
        recreate_window();
    }
    update_frame_mode_timer();
    emit fps_changed(fps_);
}

void FramePipeline::set_fps_limit(std::uint16_t limit) {
    if (limit < 1) limit = 1;
    if (limit == fps_limit_) return;
    fps_limit_ = limit;
    emit fps_limit_changed(fps_limit_);
    if (fps_ > fps_limit_) {
        set_fps(fps_limit_);
    }
}

void FramePipeline::set_color_palette(Metavision::ColorPalette palette) {
    palette_ = palette;
    frame_mode_renderer_.set_palette(palette);
    if (generator_) {
        generator_->set_color_palette(palette);
    }
    // File mode: forward to FileFrameGenerator so render_frame() uses the
    // palette selected in DisplayPanel (matches CDFrameGenerator behavior).
    file_generator_.set_color_palette(palette);
}

void FramePipeline::set_frame_mode(FrameMode mode) {
    if (frame_mode_ == mode) return;
    frame_mode_ = mode;
    frame_mode_renderer_.set_mode(mode);
    frame_mode_renderer_.set_geometry(static_cast<int>(width_),
                                      static_cast<int>(height_));
    frame_mode_renderer_.reset();
    update_frame_mode_timer();
    emit frame_mode_changed(mode);
}

void FramePipeline::set_frame_decay_time_us(Metavision::timestamp us) {
    frame_mode_renderer_.set_decay_time_us(us);
}

void FramePipeline::update_frame_mode_timer() {
    if (!frame_mode_timer_) return;
    frame_mode_timer_->stop();
    // Live mode + non-integration frame modes: generate at fps on a timer.
    if (!file_mode_ && frame_mode_ != FrameMode::Integration && width_ > 0) {
        frame_mode_timer_->start(std::max(1, 1000 / static_cast<int>(fps_)));
    }
}

void FramePipeline::on_frame_mode_tick() {
    cv::Mat frame = frame_mode_renderer_.generate();
    if (frame.empty() || frame.channels() == 0) {
        return;
    }
    try {
        cv::Mat rgb;
        if (frame.channels() == 1) {
            cv::cvtColor(frame, rgb, cv::COLOR_GRAY2RGB);
        } else {
            cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
        }
        QImage img(rgb.data, rgb.cols, rgb.rows,
                   static_cast<int>(rgb.step), QImage::Format_RGB888);
        emit frame_ready(img.copy(), last_ev_ts_.load(std::memory_order_relaxed));
    } catch (const std::exception&) {
        // Swallow — the tick must not propagate exceptions
    } catch (...) {}
}

void FramePipeline::set_file_filter_chain(FilterChain* fc) {
    file_generator_.set_filter_chain(fc);
}

// --- File playback control ---

void FramePipeline::play_file() {
    if (file_mode_) file_generator_.play();
}

void FramePipeline::pause_file() {
    if (file_mode_) file_generator_.pause();
}

void FramePipeline::seek_file(Metavision::timestamp t_us) {
    if (file_mode_) file_generator_.seek(t_us);
}

void FramePipeline::set_file_loop(bool on) {
    if (file_mode_) file_generator_.set_loop(on);
}

void FramePipeline::set_file_duration_us(Metavision::timestamp us) {
    if (file_mode_) file_generator_.set_duration_us(us);
}

void FramePipeline::set_file_loading_complete(bool complete) {
    if (file_mode_) file_generator_.set_loading_complete(complete);
}

Metavision::timestamp FramePipeline::file_position_us() const {
    if (file_mode_) return file_generator_.position_us();
    return 0;
}

Metavision::timestamp FramePipeline::file_duration_us() const {
    if (file_mode_) return file_generator_.duration_us();
    return 0;
}

} // namespace gui
