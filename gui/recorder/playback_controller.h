// gui/recorder/playback_controller.h — file playback control (design §3.3.2).
//
// Coordinates play / pause / loop / seek for file sources. All playback
// control is delegated to the FileFrameGenerator (via FramePipeline), which
// buffers all events and replays them at a user-controlled rate:
//
//   playback_rate = fps * accumulation_time_us / 1e6
//
// This enables true slow-motion (rate < 1) and fast-forward (rate > 1)
// playback, which is impossible with the SDK's CDFrameGenerator alone.
//
// Key simplifications vs. the CDFrameGenerator-based approach:
//   - Loop = cursor reset (no file reopen, no SDK terminal-state issues)
//   - Seek = set cursor + render immediately (no OSC seek, works after EOF)
//   - Play after EOF = restart timer (no file reopen)
//   - No probe timer needed (FileFrameGenerator emits position_changed)
//   - No real_time_playback flip detection (rate is fully controlled by
//     the FileFrameGenerator's QTimer, not by SDK delivery speed)
//
// For live cameras the PlaybackController is hidden (no file = no playback).

#ifndef GUI_RECORDER_PLAYBACK_CONTROLLER_H
#define GUI_RECORDER_PLAYBACK_CONTROLLER_H

#include <QObject>
#include <QPointer>
#include <QString>
#include <cstdint>
#include <utility>

#include <metavision/sdk/base/utils/timestamp.h>

namespace gui {

class CameraController;

class PlaybackController : public QObject {
    Q_OBJECT
public:
    explicit PlaybackController(QObject* parent = nullptr);

    /// @brief Binds to a camera controller (required before any playback op).
    void set_camera(CameraController* controller);

    /// @brief Opens @p path for playback. Reads all events as fast as
    /// possible (real_time_playback=false) into the FileFrameGenerator,
    /// then auto-starts playback.
    bool open_file(const QString& path);
    void play();
    void pause();
    void toggle_play_pause();
    bool seek(Metavision::timestamp t_us);
    void set_loop(bool on);

    // --- Linked parameter setters ---
    // All three delegate to the FramePipeline. The FramePipeline emits signals
    // that sync both the DisplayPanel and PlaybackControls UIs. The controller
    // listens to those signals to recompute the multiplier.

    void set_time_window_us(Metavision::timestamp us);
    void set_frame_rate(std::uint16_t fps);
    void set_multiplier(double m);

    bool loop() const { return loop_; }

    Metavision::timestamp time_window_us() const;
    std::uint16_t frame_rate() const;
    /// multiplier = fps * accumulation_time_us / 1e6 (full precision).
    double multiplier() const;
    const QString& current_file() const { return path_; }
    Metavision::timestamp duration_us() const;
    Metavision::timestamp position_us() const;
    /// @brief Timeline range {first, last} event timestamp (us) of the open
    /// file, {0, 0} when nothing is open. Cut output keeps source
    /// timestamps (no re-basing), so cut dialogs need the real range.
    std::pair<Metavision::timestamp, Metavision::timestamp> time_range_us()
        const;

signals:
    void opened(Metavision::timestamp duration_us);
    void closed();
    void state_changed(bool playing);
    void position_changed(Metavision::timestamp pos_us, Metavision::timestamp dur_us);
    void multiplier_changed(double m);
    void loop_changed(bool on);
    void error(const QString& msg);

private:
    /// True when a file source is connected and playback ops are meaningful.
    /// Internal helper only — never called from outside this class.
    bool available() const;

    /// @brief Called when the FileFrameGenerator reaches EOF (loop off).
    void on_file_eof();
    /// @brief Called when the FileFrameGenerator emits a position update.
    void on_file_position_changed(Metavision::timestamp pos, Metavision::timestamp dur);
    /// @brief Called when the FramePipeline's fps or accumulation changes.
    void on_pipeline_param_changed();
    Metavision::timestamp query_duration() const;

    QPointer<CameraController> controller_;
    QString path_;
    bool loop_{false};
    bool playing_{false};
    /// True when the last stop was caused by EOF (not user pause). play()
    /// resets the cursor to 0 when this is set.
    bool at_eof_{false};
    Metavision::timestamp duration_us_{0};
};

} // namespace gui

#endif // GUI_RECORDER_PLAYBACK_CONTROLLER_H
