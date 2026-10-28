// gui/app/frame_pipeline.h — Qt-facing wrapper around the frame generator.
//
// Lives on the GUI thread. Supports two backends:
//
//   Live mode  — gui_algo::FrameGenerator (wraps Metavision::CDFrameGenerator).
//                Used for live cameras. Shows the latest accumulation window
//                of the real-time event stream.
//
//   File mode  — FileFrameGenerator. Used for file playback. Buffers all
//                events and replays them at fps*window/1e6 speed, enabling
//                slow-motion and fast-forward. Loop = cursor reset (no file
//                reopen), seek = set cursor, pause = stop timer.
//
// Single source of truth for display parameters:
//   - fps_               : display frame rate (clamped to [1, fps_limit_])
//   - accumulation_us_   : per-frame event accumulation window (μs)
//   - fps_limit_         : user-configurable upper bound on fps (default 60)
//
// Both DisplayPanel and PlaybackControls read from and write to the pipeline.

#ifndef GUI_APP_FRAME_PIPELINE_H
#define GUI_APP_FRAME_PIPELINE_H

#include <QObject>
#include <QImage>
#include <QTimer>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include <metavision/sdk/base/utils/timestamp.h>
#include <metavision/sdk/base/events/event_cd.h>

#include "algo/common/frame_generator.h"
#include "file_frame_generator.h"
#include "frame_mode_renderer.h"

namespace gui {

class FramePipeline : public QObject {
    Q_OBJECT
public:
    explicit FramePipeline(QObject* parent = nullptr);
    ~FramePipeline();

    /// @brief Starts the pipeline in live mode (CDFrameGenerator).
    bool start(long width, long height,
               std::uint16_t fps,
               Metavision::timestamp accumulation_time_us);

    /// @brief Starts the pipeline in file mode (FileFrameGenerator). Events
    /// are buffered and replayed at fps*window/1e6 speed.
    bool start_file(long width, long height,
                    std::uint16_t fps,
                    Metavision::timestamp accumulation_time_us);

    void stop();
    bool is_running() const { return file_mode_ || window_id_ >= 0; }

    /// @brief Thread-safe: called from the SDK CD callback. In live mode,
    /// forwards to CDFrameGenerator. In file mode, buffers into FileFrameGenerator.
    void add_events(const Metavision::EventCD* begin, const Metavision::EventCD* end);

    void set_accumulation_time_us(Metavision::timestamp us);
    void set_fps(std::uint16_t fps);
    void set_fps_limit(std::uint16_t limit);
    void set_color_palette(Metavision::ColorPalette palette);

    /// @brief Selects the display frame mode (DisplayPanel "Frame mode").
    /// Integration keeps the existing CDFrameGenerator / palette-render paths;
    /// the other modes route events through FrameModeRenderer (live: a timer
    /// at fps; file: per-window inside render_frame).
    void set_frame_mode(FrameMode mode);
    FrameMode frame_mode() const { return frame_mode_; }
    /// @brief Decay time (µs) for the TimeDecay / EventsIntegration modes.
    void set_frame_decay_time_us(Metavision::timestamp us);

    /// @brief Sets the FilterChain for file-mode display filtering. Applied
    /// per-frame in FileFrameGenerator::render_frame() so flip etc.
    /// take effect immediately during file playback. Must be called before
    /// start_file() to ensure the pointer is available when rendering begins.
    void set_file_filter_chain(FilterChain* fc);

    /// @brief Forwards a preproc_* conditioning parameter to the FILE
    /// generator's StreamConditioner (the live conditioner is fed directly by
    /// CameraController::set_conditioner_param).
    void set_file_conditioner_param(const std::string& key,
                                    const std::string& value) {
        file_generator_.set_conditioner_param(key, value);
    }

    /// @brief Clears the file conditioner's temporal state (seek/loop).
    void reset_file_conditioner() { file_generator_.reset_conditioner(); }
    /// Per-source reset (teardown): clears the file conditioner state and
    /// any display ROI left over from the previous source.
    void reset_file_session_state() {
        file_generator_.reset_conditioner();
        set_display_roi_origin(false, 0, 0);
    }

    /// @brief Forwards the per-window algorithm feed hook to the file
    /// generator (P6-A file-mode write-back): the callback is invoked inside
    /// render_frame() so a stream-filter algorithm's output repaints the
    /// displayed frame during playback.
    void set_file_algo_feed_callback(
        std::function<std::pair<const Metavision::EventCD*, std::size_t>()> cb) {
        file_generator_.set_algo_feed_callback(std::move(cb));
    }

    /// @brief Tells the display path where the unified ROI sits. When the
    /// ROI is active the display renders on the FULL-sensor frame (ROI
    /// content shifted back to its absolute position; the rest stays the
    /// palette background — pre-rework visuals, kept because the ROI
    /// overlay / Zoom-to-ROI crop / Replace-composite strategies are all
    /// built around full frames). The canonical conditioned stream (what
    /// algorithms and the conditioner itself use) stays ROI-relative — the
    /// shift is display-only. No-op semantics for RONI (absolute already).
    /// Live: applied in add_events(); file: forwarded to the generator.
    void set_display_roi_origin(bool active, int x0, int y0);

    /// @brief Sets the file-mode software ROI (Phase 2.6): same semantics as
    /// the hardware ROI — the FileFrameGenerator only renders/emits events
    /// inside the rect (or outside it when @p roni is true, Phase 2.6 debug
    /// D-5). Live mode ignores this (hardware ROI handles it).
    void set_file_roi(bool enabled, int x, int y, int w, int h, bool roni = false) {
        file_generator_.set_display_roi(enabled, x, y, w, h, roni);
    }

    /// @brief Reads the file-mode software ROI state (computed rect).
    void file_roi(bool& enabled, int& x0, int& y0, int& x1, int& y1) const {
        file_generator_.display_roi(enabled, x0, y0, x1, y1);
    }

    /// @brief True when the file-mode software ROI is in RONI mode.
    bool file_roi_roni() const { return file_generator_.display_roi_roni(); }

    /// @brief Registers a listener for the display-path event stream
    /// (processed-stream recording). The listener receives every batch the
    /// display sees — the conditioned span (raw when no stage is active, so
    /// recording stays continuous across toggles), on the SDK CD thread
    /// under processed_mutex_. Live mode only (file-source recording is
    /// blocked upstream).
    void set_processed_events_listener(
        std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)> cb) {
        std::lock_guard<std::mutex> lk(processed_mutex_);
        processed_listener_ = std::move(cb);
    }

    // --- File playback control (file mode only) ---

    void play_file();
    void pause_file();
    void seek_file(Metavision::timestamp t_us);
    void set_file_loop(bool on);
    void set_file_duration_us(Metavision::timestamp us);
    /// @brief Enables bounded-buffer (streaming) file playback: the loader
    /// streams only a window ahead of the cursor; large files open instantly
    /// and memory stays bounded. Set once per file connect.
    void set_file_streaming(bool on) { file_generator_.set_streaming(on); }
    /// @brief Tells the FileFrameGenerator whether the file loader has
    /// finished streaming the whole file into the buffer (audit §六-P2).
    /// Called by CameraController when the SDK file camera hits EOF.
    void set_file_loading_complete(bool complete);
    Metavision::timestamp file_position_us() const;
    Metavision::timestamp file_duration_us() const;

    std::uint16_t fps() const { return fps_; }
    Metavision::timestamp accumulation_time_us() const { return accumulation_us_; }
    std::uint16_t fps_limit() const { return fps_limit_; }

signals:
    void frame_ready(QImage frame, Metavision::timestamp ts);
    void fps_changed(std::uint16_t fps);
    void accumulation_time_changed(Metavision::timestamp us);
    void fps_limit_changed(std::uint16_t limit);
    /// Emitted when the display frame mode changes (DisplayPanel selector).
    void frame_mode_changed(FrameMode mode);
    /// File mode: emitted after each frame with cursor position + duration.
    void file_position_changed(Metavision::timestamp pos, Metavision::timestamp dur);
    /// File mode: emitted when playback reaches the end of the buffer.
    void file_eof_reached();

    /// File mode: emitted when the cursor wraps to 0 on loop. Algorithms
    /// with temporal state must reset to avoid stale-state freezes.
    void file_looped();

    /// File mode: emitted when seek() moves the cursor. Stateful algorithms
    /// whose internal timestamps are ahead of the new cursor position must
    /// be reset to avoid the same stale-state freeze as file_looped().
    /// Emitted BEFORE the seeked frame is rendered so the reset takes
    /// effect before new events are pushed.
    void file_seeked(Metavision::timestamp t_us);

    /// File mode: emitted with the events in the current accumulation window
    /// [start, end) so algorithm instances can process them synchronously
    /// with the displayed frame. Emitted before frame_ready.
    void events_window_ready(std::shared_ptr<std::vector<Metavision::EventCD>> events,
                             Metavision::timestamp ts);

    /// File mode: emitted once when the FileFrameGenerator's event buffer
    /// hits its hard cap and further events are dropped (audit §六-C2).
    void file_buffer_truncated();

    /// File mode, streaming playback: the loader must be stopped / started /
    /// repositioned (OSC seek + start) — see FileFrameGenerator's streaming
    /// block. CameraController owns the file Camera and acts on these.
    void loader_pause_requested();
    void loader_resume_requested();
    void loader_seek_requested(Metavision::timestamp t_us);

private:
    void recreate_window();
    std::uint16_t clamp_fps(std::uint16_t fps) const;
    /// @brief Starts/stops the live-mode frame-mode tick based on the current
    /// mode + fps. No-op in file mode (render_frame drives generation there).
    void update_frame_mode_timer();

private slots:
    /// @brief Live-mode tick for non-integration frame modes: generate the
    /// current mode's frame from the accumulated events and emit frame_ready.
    void on_frame_mode_tick();

private:
    // Live mode backend
    std::unique_ptr<gui_algo::FrameGenerator> generator_;
    int window_id_{-1};

    // File mode backend
    FileFrameGenerator file_generator_;
    bool file_mode_{false};

    long width_{0};
    long height_{0};
    std::uint16_t fps_{30};
    Metavision::timestamp accumulation_us_{33000};
    std::uint16_t fps_limit_{60};

    // Frame-mode rendering (non-integration modes).
    FrameMode frame_mode_{FrameMode::Integration};
    FrameModeRenderer frame_mode_renderer_;
    QTimer* frame_mode_timer_{nullptr};
    /// Color palette forwarded to the TimeDecay visualization.
    Metavision::ColorPalette palette_{Metavision::ColorPalette::Dark};
    /// Timestamp of the last event fed to the renderer (SDK thread) — used as
    /// the frame timestamp by the live mode tick (GUI thread).
    std::atomic<Metavision::timestamp> last_ev_ts_{0};

    // Display-side ROI placement: origin to add back to the conditioned
    // ROI-relative events for the DISPLAY feeds (live). GUI-thread setter,
    // SDK-thread reader — plain ints are set before the pipeline starts or
    // between batches from the GUI thread; add_events tolerates either value.
    bool display_roi_active_{false};
    int display_roi_x0_{0};
    int display_roi_y0_{0};
    std::vector<Metavision::EventCD> display_shift_buf_;

    /// Processed-stream listener (processed-stream recording), invoked under
    /// processed_mutex_. Events arrive already conditioned
    /// (CameraController::conditioner_).
    std::mutex processed_mutex_;
    std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)>
        processed_listener_;
};

} // namespace gui

#endif // GUI_APP_FRAME_PIPELINE_H
