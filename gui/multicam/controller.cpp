#include "controller.h"
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QMetaObject>
#include <QStorageInfo>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace gui::multicam {
namespace {
using Clock = std::chrono::steady_clock;
void fail(const QString &message) { throw std::runtime_error(message.toStdString()); }
// Raised when the RP2350 cannot be verified before any pulse is sent. The live camera
// connections are unaffected, so the worker keeps them open instead of resetting the rig.
struct BoardUnavailable : std::runtime_error {
    using std::runtime_error::runtime_error;
};
template <class T> bool is_ready(std::future<T> &f) {
    return f.valid() && f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}
QString path_in(const QString &dir, const char *file) { return QDir(dir).filePath(QLatin1String(file)); }
} // namespace

struct Controller::Impl {
    Controller *owner;
    CameraFactory camera_factory;
    std::function<std::unique_ptr<McuTransport>()> mcu_factory;
    ReaderFactory reader_factory;
    std::function<QVector<DeviceEntry>()> discovery_factory;
    Detector detector;
    RuntimeClock clock;
    Clock::time_point now() const { return clock.now(); }
    RigConfig config;
    WorkflowSnapshot state;
    Guidance guidance;
    std::array<std::unique_ptr<CameraStream>, 3> cameras;
    std::unique_ptr<McuTransport> mcu;
    std::mutex mutex;
    std::condition_variable wake;
    struct Command {
        std::function<void()> job;
        QString key;
        quint64 generation;
        bool operation;
    };
    std::deque<Command> commands;
    QString active_command;
    std::atomic<quint64> operation_generation{0};
    bool disconnect_requested = false;
    WorkflowSnapshot latest;
    std::atomic_bool ending{false}, cancelled{false}, publish_pending{false};
    std::atomic<quint64> seek_generation{0};
    std::thread worker;
    std::future<DetectedGroup> detection;
    std::future<gui_algo::rig::Result> solution;
    quint64 calibration_revision = 0, solution_revision = 0;
    quint64 detection_revision = 0;
    quint64 detection_generation = 0, solution_generation = 0;
    qint64 last_window = -1;
    int supplemental = -1;
    bool automatic = false, manual = false;
    Clock::time_point calibration_deadline{};
    Clock::time_point detection_started{};
    qint64 capture_duration_ms = 5000;
    bool capture_expired = false;
    QString calibration_runtime_reason;
    QString take_dir;
    QJsonObject manifest;
    Take playback_take;
    std::array<std::unique_ptr<RawReader>, 3> readers;
    Clock::time_point published = now(), record_started = now(), playback_clock = now(), disk_checked = now();
    bool verify_solution = false;
    quint64 sync_attempt = 0;
    std::array<qint64, 3> wave_after{{-1, -1, -1}};
    Clock::time_point wave_heartbeat = now();
    QJsonObject wave_evidence;
    QJsonObject controller_check;

    Impl(Controller *o, CameraFactory c, std::function<std::unique_ptr<McuTransport>()> m, ReaderFactory r,
         RuntimeClock t, std::function<QVector<DeviceEntry>()> discovery, Detector detect)
        : owner(o), camera_factory(std::move(c)), mcu_factory(std::move(m)), reader_factory(std::move(r)),
          discovery_factory(std::move(discovery)), detector(std::move(detect)), clock(std::move(t)) {
        if (!camera_factory)
            camera_factory = make_hal_camera;
        if (!mcu_factory)
            mcu_factory = [] { return std::make_unique<SerialMcuTransport>(12); };
        if (!reader_factory)
            reader_factory = make_raw_reader;
        if (!discovery_factory)
            discovery_factory = discover_cameras;
        if (!detector)
            detector = detect_group;
        worker = std::thread([this] { run(); });
    }
    ~Impl() {
        ending = true;
        cancelled = true;
        ++seek_generation;
        wake.notify_one();
        if (worker.joinable())
            worker.join();
    }
    void enqueue(std::function<void()> job, const QString &key = {}, bool operation = false,
                 bool replace = false) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!key.isEmpty()) {
            if (!replace && active_command == key)
                return;
            for (auto it = commands.begin(); it != commands.end();) {
                if (it->key != key) {
                    ++it;
                    continue;
                }
                if (!replace)
                    return;
                it = commands.erase(it);
            }
        }
        if (commands.size() >= 64) {
            if (replace)
                commands.pop_back();
            else
                return;
        }
        commands.push_back({std::move(job), key, operation_generation.load(), operation});
        wake.notify_one();
    }
    void request_stop(bool disconnect) {
        std::lock_guard<std::mutex> lock(mutex);
        ++operation_generation;
        ++seek_generation;
        cancelled = true;
        disconnect_requested = disconnect_requested || disconnect;
        commands.erase(std::remove_if(commands.begin(), commands.end(),
                                      [](const auto &command) {
                                          return command.operation || command.key == "stop" ||
                                                 command.key == "seek";
                                      }),
                       commands.end());
        if (commands.size() >= 64)
            commands.pop_back();
        const bool close = disconnect_requested;
        commands.push_front({[this, close] {
                                 stop_work();
                                 if (close) {
                                     close_cameras();
                                     state.sop_running = false;
                                 }
                             },
                             "stop", operation_generation.load(), false});
        wake.notify_one();
    }
    void publish() {
        const bool refresh_preview = !publish_pending.load();
        for (int i = 0; i < 3; ++i)
            if (cameras[i]) {
                auto s = refresh_preview ? cameras[i]->snapshot() : cameras[i]->statistics();
                if (!refresh_preview)
                    s.preview = state.cameras[i].preview;
                s.corners = state.cameras[i].corners;
                s.coverage = state.cameras[i].coverage;
                state.cameras[i] = std::move(s);
            }
        if (!state.playback.loaded) {
            state.calibration = guidance.status(config);
            state.calibration.collecting = state.activity == Activity::Calibrating;
            state.calibration.capture_duration_ms = capture_duration_ms;
            state.calibration.capture_remaining_ms =
                state.calibration.collecting
                    ? std::max<qint64>(0, std::chrono::duration_cast<std::chrono::milliseconds>(
                                              calibration_deadline - now())
                                              .count())
                    : 0;
            state.calibration.capture_expired = capture_expired;
            if (!calibration_runtime_reason.isEmpty())
                state.calibration.reason = calibration_runtime_reason;
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            latest = state;
        }
        if (!publish_pending.exchange(true))
            QMetaObject::invokeMethod(
                owner,
                [this] {
                    WorkflowSnapshot copy;
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        copy = latest;
                    }
                    publish_pending = false;
                    emit owner->changed(copy);
                },
                Qt::QueuedConnection);
        published = now();
    }
    void pump() {
        for (auto &camera : cameras)
            if (camera)
                camera->pump();
    }
    // Used while blocked inside RP2350 serial commands (a finite burst lasts several seconds):
    // keep draining the cameras AND keep publishing previews so the live view never freezes.
    void pump_live() {
        pump();
        if (now() - published > std::chrono::milliseconds(66))
            publish();
    }
    void wait_ms(int ms) {
        auto end = now() + std::chrono::milliseconds(ms);
        while (now() < end) {
            if (cancelled || ending)
                fail("Operation cancelled.");
            pump();
            if (now() - published > std::chrono::milliseconds(66))
                publish();
            clock.sleep(std::chrono::milliseconds(1));
        }
    }
    void close_cameras() {
        QString error;
        for (auto &camera : cameras)
            if (camera) {
                try {
                    camera->stop();
                } catch (const std::exception &e) {
                    error += QString::fromUtf8(e.what()) + '\n';
                }
            }
        for (int i = 0; i < 3; ++i)
            if (cameras[i])
                state.cameras[i] = cameras[i]->snapshot();
        cameras = {};
        if (state.sync.verdict != Verdict::Failed)
            state.sync = {};
        state.sync.operator_accepted = false;
        state.sync.attempt_completed = false;
        state.board_ready = false;
        if (!error.isEmpty())
            fail(error);
    }
    void connect_cameras(bool logging = false) {
        if (!configuration_blockers(config).isEmpty())
            fail(configuration_blockers(config).front().message);
        close_cameras();
        state.playback = {};
        readers = {};
        state.activity = Activity::Connecting;
        ++state.epoch;
        state.cameras = {};
        state.sync = {};
        last_window = -1;
        for (int order = 0; order < 3; ++order) {
            const int i = (config.master + 1 + order) % 3;
            if (cancelled)
                fail("Connection cancelled.");
            state.message = QString("Connecting camera %1/3: %2 (%3) — opening USB stream & clock role...")
                                .arg(order + 1)
                                .arg(config.cameras[i].position, config.cameras[i].serial);
            publish();
            cameras[i] = camera_factory();
            cameras[i]->open(config.cameras[i], i == config.master);
            cameras[i]->apply_tuning(config.tuning[i]);
        }
        for (int order = 0; order < 3; ++order) {
            int i = (config.master + 1 + order) % 3;
            cameras[i]->start(logging ? path_in(take_dir, qPrintable(QString("camera-%1.raw").arg(i + 1)))
                                      : QString{});
        }
        for (int i = 0; i < 3; ++i) {
            auto current = cameras[i]->statistics();
            if (guidance.result().trained && current.width != guidance.result().models[i].size.width)
                invalidate_calibration();
            else if (guidance.result().trained && current.height != guidance.result().models[i].size.height)
                invalidate_calibration();
        }
        state.activity = Activity::Idle;
        state.message = "Three cameras connected. Clock roles and Trigger In read "
                        "back; synchronization must be checked.";
    }
    void clear_corners() {
        for (int i = 0; i < 3; ++i)
            state.cameras[i].corners.clear();
    }
    void invalidate_calibration() {
        ++calibration_revision;
        guidance.reset();
        clear_corners();
        capture_expired = false;
        calibration_runtime_reason.clear();
        config.optics_confirmed = false;
    }
    void check_mcu() {
        state.board_ready = false;
        state.board_detail = "Verifying RP2350 USB serial identity and GP2 LOW state...";
        if (state.activity == Activity::Idle) {
            state.message = state.board_detail;
            publish();
        }
        if (!mcu)
            mcu = mcu_factory();
        auto result = mcu->check(config.board_serial, cancelled, [this] { pump_live(); });
        controller_check = result.details;
        state.board_ready = result.ok && result.low_confirmed;
        state.board_detail = result.reason;
        if (!state.board_ready)
            throw BoardUnavailable(
                (result.ok ? QStringLiteral("Controller output state is unknown after verification.") : result.reason)
                    .toStdString());
        if (state.activity == Activity::Idle)
            state.message = "RP2350 controller verified: GP2 LOW confirmed (" + state.board_detail + ").";
    }
    void sync() {
        if (state.playback.loaded ||
            std::any_of(cameras.begin(), cameras.end(), [](const auto &camera) { return !camera; }))
            fail("Connect all three live cameras before synchronization.");
        if (!state.all_connected()) {
            publish();
            if (!state.all_connected())
                fail("Connect all three cameras before synchronization.");
        }
        if (!config.trigger_wiring_confirmed)
            fail("Confirm GP2 Trigger wiring and per-camera current limiting before "
                 "sending pulses.");
        state.sync = {};
        state.sync.attempt = ++sync_attempt;
        state.error.clear();
        state.activity = Activity::Syncing;
        state.sync.verdict = Verdict::Running;
        state.sync.epoch = state.epoch;
        publish();
        try {
            check_mcu();
            state.message = "Settling clocks (5 s).";
            wait_ms(5000);
            state.message = "Sending one preamble pulse on the bound RP2350 GP2.";
            publish();
            auto preamble = mcu->burst(1, cancelled, [this] { pump_live(); });
            if (!preamble.ok || !preamble.low_confirmed)
                fail(preamble.reason);
            wait_ms(1000);
            std::array<qint64, 3> after{{-1, -1, -1}};
            for (int i = 0; i < 3; ++i)
                for (const auto &edge : cameras[i]->statistics().triggers)
                    after[i] = std::max(after[i], edge.timestamp_us);
            state.message = "Sending three formal pulses on the bound RP2350 GP2.";
            publish();
            auto formal = mcu->burst(3, cancelled, [this] { pump_live(); });
            if (!formal.ok || !formal.low_confirmed)
                fail(formal.reason);
            state.message = "Analysing decoded Trigger edges from all three cameras.";
            publish();
            wait_ms(1000);
            std::array<QVector<TriggerEdge>, 3> edges;
            for (int i = 0; i < 3; ++i)
                for (const auto &edge : cameras[i]->statistics().triggers)
                    if (edge.timestamp_us > after[i])
                        edges[i].push_back(edge);
            state.sync = match_trigger_edges(edges, config.master, config.max_delta_us, state.epoch);
            state.sync.attempt = sync_attempt;
            state.sync.observed_edges = edges;
            state.sync.controller_evidence = {{"mode", "finite"},
                                              {"requestedDriveMa", 12},
                                              {"identityReadback", controller_check},
                                              {"preamble", preamble.details},
                                              {"formal", formal.details}};
            auto low = mcu->cleanup([this] { pump_live(); });
            state.board_ready = low.ok && low.low_confirmed;
            state.board_detail = low.reason;
            manifest.insert("controllerLowConfirmed", low.low_confirmed);
            state.sync.attempt_completed = state.board_ready;
            state.sync.controller_evidence.insert("cleanup", low.details);
            manifest.insert("triggerEvidence", state.sync.to_json());
            if (!state.board_ready)
                fail("Controller output state is unknown after cleanup.");
            if (state.sync.verdict != Verdict::Passed)
                state.error = state.sync.reason;
            state.message = state.sync.reason;
            state.activity = Activity::Idle;
        } catch (const std::exception &error) {
            state.sync.verdict = Verdict::Failed;
            state.sync.epoch = state.epoch;
            state.sync.reason = QString::fromUtf8(error.what());
            state.sync.attempt_completed = false;
            state.board_ready = false;
            manifest.insert("controllerLowConfirmed", false);
            if (mcu) {
                try {
                    auto low = mcu->cleanup([] {});
                    manifest.insert("controllerLowConfirmed", low.ok && low.low_confirmed);
                    state.board_ready = low.ok && low.low_confirmed;
                    state.board_detail = low.reason;
                    state.sync.controller_evidence.insert("cleanupRecovery", low.details);
                } catch (const std::exception &e) {
                    state.board_detail = QString::fromUtf8(e.what());
                }
            }
            manifest.insert("triggerEvidence", state.sync.to_json());
            state.activity = Activity::Idle;
            throw;
        }
    }
    void start_wave() {
        if (state.busy() || state.playback.loaded)
            return;
        if (!state.all_connected() || !config.trigger_wiring_confirmed)
            fail("Connect the rig and confirm GP2 Trigger wiring/current limiting "
                 "before starting the wave.");
        state.sop_running = false;
        state.sop_paused = true;
        state.error.clear();
        state.sync = {};
        state.sync.epoch = state.epoch;
        state.sync.attempt = ++sync_attempt;
        state.sync.verdict = Verdict::Running;
        state.activity = Activity::Triggering;
        wave_evidence = {};
        publish();
        check_mcu();
        wave_after.fill(-1);
        for (int i = 0; i < 3; ++i)
            for (const auto &e : cameras[i]->statistics().triggers)
                wave_after[i] = std::max(wave_after[i], e.timestamp_us);
        state.trigger_stop_pending = true;
        const auto started = mcu->start_continuous(cancelled, [this] { pump_live(); });
        wave_evidence = {{"mode", "continuous"},
                         {"requestedDriveMa", 12},
                         {"identityReadback", controller_check},
                         {"start", started.details},
                         {"startOk", started.ok}};
        if (!started.ok)
            fail(started.reason);
        state.board_ready = false;
        state.board_detail = "GP2 continuous wave active: 1 s high / 1 s low, 12 mA drive.";
        state.message = state.board_detail;
        wave_heartbeat = now();
    }
    void stop_wave(bool healthy = true) {
        const bool output_requested = state.trigger_stop_pending;
        const auto low = mcu ? mcu->cleanup([] {}) : McuResult{};
        wave_evidence.insert("cleanup", low.details);
        wave_evidence.insert("lowConfirmed", low.ok && low.low_confirmed);
        state.sync.verdict = Verdict::Incomplete;
        state.sync.reason = "Continuous Trigger trace: inspect the observed edges "
                            "and accept manually.";
        state.sync.attempt_completed = healthy && low.ok && low.low_confirmed;
        state.sync.controller_evidence = wave_evidence;
        for (int i = 0; i < 3; ++i) {
            state.sync.observed_edges[i].clear();
            if (cameras[i])
                for (const auto &e : cameras[i]->statistics().triggers)
                    if (e.timestamp_us > wave_after[i])
                        state.sync.observed_edges[i].push_back(e);
        }
        state.board_ready = low.ok && low.low_confirmed;
        state.trigger_stop_pending = output_requested && !state.board_ready;
        state.board_detail = low.reason;
        state.activity = Activity::Idle;
        state.message = state.board_ready ? "Wave stopped. LOW and original drive confirmed; trace retained."
                                          : "Wave stop could not be confirmed. Do not disconnect trigger "
                                            "wiring.";
        if (!state.board_ready) {
            const auto cleanup_error = state.message + " " + low.reason;
            state.error =
                healthy || state.error.isEmpty() ? cleanup_error : state.error + "; " + cleanup_error;
        } else if (healthy)
            state.error.clear();
        manifest.insert("triggerEvidence", state.sync.to_json());
    }
    void calibration_tick() {
        if (state.activity == Activity::Calibrating && now() >= calibration_deadline) {
            const auto last_issue = calibration_runtime_reason.isEmpty() ? guidance.status(config).reason
                                                                         : calibration_runtime_reason;
            state.activity = Activity::Idle;
            capture_expired = true;
            ++calibration_revision;
            guidance.pause();
            clear_corners();
            for (auto &camera : cameras)
                camera->collect_windows(false);
            state.sop_paused = true;
            const auto info = guidance.status(config);
            state.message = QString("%1 s capture round finished; %2 common training and %3 validation "
                                    "poses retained. Reposition, then start the next round.")
                                .arg(capture_duration_ms / 1000)
                                .arg(info.common_count)
                                .arg(info.validation_count);
            calibration_runtime_reason = state.message;
            if (!last_issue.isEmpty())
                calibration_runtime_reason += "\nLast observation: " + last_issue;
        }
        if (is_ready(detection)) {
            std::lock_guard<std::mutex> lock(mutex);
            const bool current = !cancelled && !ending && state.activity == Activity::Calibrating &&
                                 detection_revision == calibration_revision &&
                                 detection_generation == operation_generation.load();
            std::optional<DetectedGroup> group;
            try {
                group = detection.get();
            } catch (...) {
                if (current)
                    throw;
            }
            if (current && group) {
                qint64 through = -1;
                qint64 lag = 0;
                for (const auto &camera : cameras) {
                    through = std::max(through, camera->watermark());
                    lag = std::max(
                        lag, camera->statistics().metadata.value("calibrationProgressLagMs").toInteger(0));
                }
                if (through - group->start_us > 4 * calibration_window_us ||
                    now() - detection_started > std::chrono::milliseconds(400) || lag > 300) {
                    guidance.pause();
                    clear_corners();
                    calibration_runtime_reason =
                        "Corner detection is behind the live stream; old candidate discarded. "
                        "Hold the board still while detection catches up.";
                    group.reset();
                }
            }
            if (current && group) {
                calibration_runtime_reason.clear();
                const auto before_info = guidance.status(config);
                guidance.observe(*group, automatic, manual, supplemental);
                manual = false;
                auto corners = guidance.corners();
                for (int i = 0; i < 3; ++i)
                    state.cameras[i].corners = corners[i];
                auto info = guidance.status(config);
                const bool accepted_new_pose =
                    info.common_count > before_info.common_count ||
                    info.validation_count > before_info.validation_count ||
                    info.intrinsic_counts != before_info.intrinsic_counts;
                if (accepted_new_pose)
                    calibration_deadline = now() + std::chrono::milliseconds(capture_duration_ms);
                if ((!guidance.result().trained && info.common_count >= config.training_target) ||
                    (guidance.result().trained && info.validation_count >= config.validation_target)) {
                    state.activity = Activity::Idle;
                    clear_corners();
                    for (auto &camera : cameras)
                        camera->collect_windows(false);
                    if (state.sop_running && !state.sop_paused)
                        start_solution(guidance.result().trained);
                }
            }
        }
        if (is_ready(solution)) {
            std::lock_guard<std::mutex> lock(mutex);
            const bool current = !cancelled && !ending && solution_revision == calibration_revision &&
                                 solution_generation == operation_generation.load();
            try {
                auto result = solution.get();
                if (current) {
                    guidance.set_result(result);
                    state.activity = Activity::Idle;
                    if (!result.error.empty()) {
                        state.sop_paused = true;
                        state.error = QString::fromStdString(result.error);
                    } else
                        state.message = result.validated
                                            ? "Independent validation passed."
                                            : "Training solved. Collect NEW independent validation "
                                              "poses; training is frozen.";
                }
            } catch (...) {
                if (current)
                    throw;
            }
            if (!current && state.activity == Activity::Solving) {
                state.activity = Activity::Idle;
                state.message = "Cancelled solve finished; stale result discarded.";
            }
        }
        if (state.activity != Activity::Calibrating || detection.valid())
            return;
        qint64 through = std::numeric_limits<qint64>::max();
        qint64 furthest = -1;
        for (const auto &c : cameras) {
            if (!c)
                return;
            const auto statistics = c->statistics();
            if (!statistics.metadata.value("calibrationIngressReady").toBool(true)) {
                calibration_runtime_reason =
                    QString("%1 is draining pre-capture HAL data. Waiting for an empty-queue barrier; "
                            "no queued old pose will be sampled.")
                        .arg(statistics.binding.position);
                guidance.pause();
                return;
            }
            const auto lag = statistics.metadata.value("calibrationProgressLagMs").toInteger(0);
            if (lag > 300) {
                calibration_runtime_reason =
                    QString("%1 has fallen %2 ms behind its post-click HAL barrier; waiting to catch up. "
                            "This estimate is not sensor-to-screen latency.")
                        .arg(statistics.binding.position)
                        .arg(lag);
                guidance.pause();
                return;
            }
            if (std::isfinite(statistics.decode_realtime_ratio) && statistics.decode_realtime_ratio < .8) {
                calibration_runtime_reason =
                    QString("%1 decode progress is %2x real time; waiting for current data. "
                            "Preview sampling does not reduce RAW or calibration input.")
                        .arg(statistics.binding.position)
                        .arg(statistics.decode_realtime_ratio, 0, 'f', 2);
                guidance.pause();
                return;
            }
            through = std::min(through, c->watermark());
            furthest = std::max(furthest, c->watermark());
        }
        if (furthest - through > 3 * calibration_window_us) {
            calibration_runtime_reason =
                QString("Camera decode progress differs by %1 ms; waiting for a fresh common window. "
                        "No pose has been accepted from mismatched times.")
                    .arg((furthest - through) / 1000);
            guidance.pause();
            return;
        }
        const qint64 start = through / calibration_window_us * calibration_window_us - calibration_window_us;
        if (start <= last_window)
            return;
        last_window = start;
        std::array<CountWindow, 3> windows;
        for (int i = 0; i < 3; ++i) {
            auto w = cameras[i]->window(start);
            if (!w) {
                calibration_runtime_reason =
                    QString("Waiting for %1's complete 100 ms window; no partial pose will be used.")
                        .arg(config.cameras[i].position);
                return;
            }
            windows[i] = std::move(*w);
        }
        detection_revision = calibration_revision;
        detection_generation = operation_generation.load();
        detection_started = now();
        detection = std::async(std::launch::async,
                               [windows = std::move(windows), detect = detector] { return detect(windows); });
    }
    void begin_calibration(bool auto_capture, int extra) {
        if (state.busy())
            return;
        const auto b = blockers(Step::Calibration, config, state);
        if (!b.isEmpty())
            fail(b.front().message);
        if (solution.valid())
            fail("Wait for the current solve to finish.");
        automatic = auto_capture;
        manual = false;
        supplemental = extra;
        last_window = -1;
        ++calibration_revision;
        capture_expired = false;
        capture_duration_ms = qint64(config.calibration_capture_seconds) * 1000;
        calibration_deadline = now() + std::chrono::milliseconds(capture_duration_ms);
        calibration_runtime_reason.clear();
        state.error.clear();
        state.activity = Activity::Calibrating;
        for (auto &camera : cameras)
            camera->collect_windows(true);
        state.message = extra >= 0 ? "Intrinsic-only short capture; not counted as common groups."
                                   : "Hold the complete board still in all three views for one second. "
                                     "This capture round stops automatically; reposition between rounds.";
    }
    void start_solution(bool validation) {
        if (state.recording())
            return;
        if (solution.valid())
            return;
        const auto info = guidance.status(config);
        if (validation) {
            if (!guidance.result().trained || info.validation_count < config.validation_target)
                fail("Collect the configured number of NEW independent validation "
                     "poses.");
        } else if (info.common_count < config.training_target)
            fail("Collect the configured common training poses first.");
        for (auto &camera : cameras)
            if (camera)
                camera->collect_windows(false);
        state.activity = Activity::Solving;
        capture_expired = false;
        calibration_runtime_reason.clear();
        solution_revision = calibration_revision;
        solution_generation = operation_generation.load();
        verify_solution = validation;
        const auto copy = guidance;
        const auto c = config;
        solution = std::async(std::launch::async, [copy, c, validation] {
            if (validation)
                return gui_algo::rig::validate(copy.result(), copy.validation(), c.max_rms_px);
            return gui_algo::rig::solve_intrinsics_and_joint(
                copy.sizes(), copy.intrinsic_views(), copy.training(), c.reference, c.square_mm / 1000);
        });
    }
    void save_manifest(const QString &status, const QString &reason = {}) {
        QJsonArray items;
        for (int i = 0; i < 3; ++i) {
            const auto c = cameras[i] ? cameras[i]->statistics() : state.cameras[i];
            items.append(QJsonObject{{"serial", config.cameras[i].serial},
                                     {"selection", config.cameras[i].selection},
                                     {"position", config.cameras[i].position},
                                     {"role", i == config.master ? "master" : "slave"},
                                     {"rawFile", QString("camera-%1.raw").arg(i + 1)},
                                     {"eventCount", qint64(c.event_count)},
                                     {"firstTimestampUs", c.first_timestamp_us},
                                     {"lastTimestampUs", c.latest_timestamp_us},
                                     {"bytesConsumed", qint64(c.bytes_consumed)},
                                     {"timestampRegressions", qint64(c.regressions)},
                                     {"protocolErrors", qint64(c.protocol_errors)},
                                     {"metadata", c.metadata}});
        }
        manifest.insert("schemaVersion", 2);
        manifest.insert("state", status);
        manifest.insert("reason", reason);
        manifest.insert("takeLabel", config.take_label.isEmpty() ? config.note : config.take_label);
        manifest.insert("profileId", config.active_profile_id);
        manifest.insert("layoutPreset", layout_preset_code(config.layout_preset));
        manifest.insert("cameras", items);
        manifest.insert("project", config.to_json());
        manifest.insert("epoch", qint64(state.epoch));
        manifest.insert("formalStartUs", state.formal_start_us);
        manifest.insert("formalEndUs", state.playback.formal_end_us);
        manifest.insert("sync", state.sync.to_json());
        manifest.insert("calibrationFile", "calibration.json");
        manifest.insert("rawConsumerBoundary", "one poll/pull/decode per camera; no post-stop drain; HAL "
                                               "decoder tail unknown");
        QString error;
        state.take_manifest = manifest;
        state.take_directory = take_dir;
        if (!write_json(path_in(take_dir, "session.json"), manifest, &error))
            fail("Cannot preserve session manifest: " + error);
    }
    void start_formal_recording() {
        const auto b = blockers(Step::Capture, config, state);
        if (!b.isEmpty())
            fail(b.front().message);
        qint64 boundary = 0;
        for (const auto &c : cameras) {
            const auto measured = c->watermark();
            if (measured < 0 || measured == std::numeric_limits<qint64>::max())
                fail("Cannot measure the formal capture boundary.");
            boundary = std::max(boundary, measured);
        }
        if (cancelled || ending)
            fail("Recording preparation cancelled.");
        state.formal_start_us = boundary + 1;
        record_started = now();
        state.activity = Activity::Recording;
        state.message = "Recording the formal segment; preflight RAW is retained.";
        save_manifest("recording");
    }
    void begin_take() {
        if (state.busy() || !take_dir.isEmpty())
            return;
        const auto b = blockers(Step::Capture, config, state);
        if (!b.isEmpty())
            fail(b.front().message);
        const QString take_root =
            config.active_profile_id.trimmed().isEmpty()
                ? config.output
                : QDir(config.output)
                      .filePath(QStringLiteral("profiles/") + sanitize_profile_id(config.active_profile_id) +
                                QStringLiteral("/takes"));
        take_dir = create_take_directory(take_root);
        manifest = {};
        state.take_report = {};
        state.take_verdict = Verdict::Running;
        state.take_directory = take_dir;
        state.formal_start_us = -1;
        state.playback.formal_end_us = -1;
        state.elapsed_ms = 0;
        manifest.insert("createdUtc", QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
        try {
            save_manifest("preparing");
            QJsonObject cal_snap = guidance.status(config).profile;
            if (cal_snap.isEmpty())
                cal_snap = state.calibration.profile;
            if (!config.active_profile_id.trimmed().isEmpty())
                cal_snap.insert("profileId", config.active_profile_id);
            cal_snap.insert("layoutPreset", layout_preset_code(config.layout_preset));
            if (!write_json(path_in(take_dir, "calibration.json"), cal_snap))
                fail("Cannot save calibration snapshot.");
            connect_cameras(true);
            if (!guidance.result().validated || !config.optics_confirmed)
                fail("Reopened camera geometry does not match the validated "
                     "calibration.");
            state.activity = Activity::Preparing;
            sync();
            if (!state.sync.accepted(state.epoch, config.max_delta_us)) {
                state.activity = Activity::Preparing;
                state.sop_paused = true;
                state.message = "Take preflight awaits manual synchronization "
                                "acceptance or cancellation.";
                save_manifest("preparing", state.sync.reason);
                return;
            }
            start_formal_recording();
        } catch (const std::exception &e) {
            finish_take(false, QString::fromUtf8(e.what()));
            throw;
        }
    }
    void finish_take(bool completed, const QString &reason = {}) {
        if (take_dir.isEmpty())
            return;
        const auto generation = operation_generation.load();
        state.activity = Activity::Stopping;
        publish();
        QString failure = reason;
        for (int i = 0; i < 3; ++i)
            if (cameras[i]) {
                try {
                    cameras[i]->stop();
                } catch (const std::exception &e) {
                    failure += QString::fromUtf8(e.what());
                }
                state.cameras[i] = cameras[i]->snapshot();
            }
        qint64 common_end = std::numeric_limits<qint64>::max();
        for (const auto &c : state.cameras)
            common_end = std::min(common_end, c.latest_timestamp_us);
        state.playback.formal_end_us = common_end;
        if (state.formal_start_us < 0 || common_end < state.formal_start_us) {
            completed = false;
            failure += " No complete formal common interval.";
        }
        save_manifest(completed && failure.isEmpty() ? "completed" : "incomplete", failure);
        cameras = {};
        state.board_ready = false;
        state.activity = Activity::Verifying;
        publish();
        try {
            auto take = read_take(take_dir);
            state.take_report = verify_take(take, reader_factory, [this, generation] {
                return ending.load() || operation_generation.load() != generation;
            });
            if (!write_json(path_in(take_dir, "report.json"), state.take_report))
                failure += " Cannot save read-only verification report.";
            if (state.take_report.value("verdict").toString() == "failed")
                failure += " Read-only RAW verification failed.";
            if (!failure.isEmpty())
                save_manifest("incomplete", failure);
            state.take_verdict =
                state.take_report.value("verdict").toString() == "failed" || !completed || !failure.isEmpty()
                    ? Verdict::Failed
                    : Verdict::Incomplete;
            state.message = "Take closed. " + state.take_report.value("verdict").toString() +
                            "; decoder tail remains unknown.";
        } catch (const std::exception &e) {
            state.take_verdict = Verdict::Incomplete;
            failure += QString::fromUtf8(e.what());
            state.take_report = {{"schemaVersion", 1},
                                 {"verdict", "incomplete"},
                                 {"error", failure},
                                 {"decoderTail", "unknown"}};
            manifest.insert("state", "incomplete");
            manifest.insert("reason", failure);
            state.take_manifest = manifest;
            write_json(path_in(take_dir, "session.json"), manifest);
            write_json(path_in(take_dir, "report.json"), state.take_report);
        }
        if (config.discard_incomplete_takes &&
            (!completed || !failure.isEmpty() || state.take_verdict == Verdict::Failed)) {
            QDir(take_dir).removeRecursively();
            state.take_directory.clear();
            state.take_manifest = {};
            state.message = "Incomplete or failed take discarded automatically (no RAW files retained).";
        }
        state.activity = Activity::Idle;
        state.error = failure;
        state.sop_running = false;
        take_dir.clear();
        state.sync = {};
    }
    void stop_work() {
        state.playback.playing = false;
        if (state.activity == Activity::Calibrating) {
            guidance.pause();
            clear_corners();
            calibration_runtime_reason.clear();
        }
        if (state.activity == Activity::Triggering || state.trigger_stop_pending)
            stop_wave();
        if (state.activity == Activity::Recording || state.activity == Activity::Preparing ||
            !take_dir.isEmpty())
            finish_take(state.activity == Activity::Recording);
        else {
            for (auto &c : cameras)
                if (c)
                    c->collect_windows(false);
            if (state.activity != Activity::Solving)
                state.activity = state.playback.loaded ? Activity::Playback : Activity::Idle;
        }
        state.sop_paused = true;
        cancelled = false;
    }
    void playback_frame(qint64 position, quint64 generation) {
        if (!state.playback.loaded || generation != seek_generation.load())
            return;
        const auto low = state.playback.include_preflight || state.playback.formal_start_us < 0
                             ? state.playback.first_us
                             : state.playback.formal_start_us;
        const auto high = state.playback.include_preflight || state.playback.formal_end_us < 0
                              ? state.playback.last_us
                              : state.playback.formal_end_us;
        position = std::clamp(position, low, high);
        auto frames = state.cameras;
        try {
            for (int i = 0; i < playback_take.cameras.size(); ++i) {
                if (!readers[i])
                    readers[i] = reader_factory(playback_take.cameras[i]);
                frames[i] = readers[i]->frame(std::max(low, position - 100000), position, [this, generation] {
                    return ending.load() || seek_generation.load() != generation;
                });
            }
        } catch (...) {
            if (ending || seek_generation.load() != generation)
                return;
            state.playback.playing = false;
            throw;
        }
        if (ending || seek_generation.load() != generation)
            return;
        state.cameras = std::move(frames);
        state.playback.position_us = position;
        playback_clock = now();
        if (position >= high)
            state.playback.playing = false;
    }
    void sop_tick() {
        if (!state.sop_running || state.sop_paused || state.activity != Activity::Idle)
            return;
        if (!configuration_blockers(config).isEmpty()) {
            state.message = configuration_blockers(config).front().message;
            return;
        }
        if (!state.all_connected()) {
            connect_cameras();
            publish();
            return;
        }
        if (!state.board_ready) {
            check_mcu();
            return;
        }
        if (!state.sync.accepted(state.epoch, config.max_delta_us)) {
            if (!config.trigger_wiring_confirmed) {
                state.message = "Confirm Trigger wiring/current limiting on the "
                                "synchronization page.";
                state.sop_paused = true;
                return;
            }
            if (state.sync.epoch == state.epoch && state.sync.attempt_completed) {
                state.message = "Inspect the Trigger trace and accept manually, or "
                                "retry the finite check.";
                state.sop_paused = true;
                return;
            }
            sync();
            return;
        }
        auto b = blockers(Step::Calibration, config, state);
        if (state.sop_reuse && guidance.result().validated)
            b = blockers(Step::Sync, config, state);
        if (!b.isEmpty()) {
            state.message = b.front().message;
            return;
        }
        auto info = guidance.status(config);
        if (!guidance.result().trained) {
            if (info.common_count >= config.training_target)
                start_solution(false);
            else
                begin_calibration(true, -1);
            return;
        }
        if (!guidance.result().validated) {
            if (info.validation_count >= config.validation_target)
                start_solution(true);
            else
                begin_calibration(true, -1);
            return;
        }
        if (!config.optics_confirmed) {
            state.message = "Confirm unchanged lens, focus and rigid rig before recording.";
            return;
        }
        begin_take();
    }
    void run() {
        while (!ending) {
            std::function<void()> job;
            {
                std::unique_lock<std::mutex> lock(mutex);
                if (!commands.empty()) {
                    auto command = std::move(commands.front());
                    commands.pop_front();
                    if (!command.operation || command.generation == operation_generation.load()) {
                        if (command.key == "stop")
                            disconnect_requested = false;
                        if (command.operation)
                            cancelled = false;
                        active_command = command.key;
                        job = std::move(command.job);
                    }
                }
            }
            try {
                if (job)
                    job();
                if (!state.playback.loaded)
                    pump();
                if (state.activity == Activity::Triggering &&
                    now() - wave_heartbeat >= std::chrono::seconds(1)) {
                    auto status = mcu->keepalive_continuous(cancelled, [this] { pump_live(); });
                    wave_evidence.insert("lastHeartbeat", status.details);
                    if (!status.ok)
                        fail("Trigger wave heartbeat failed: " + status.reason);
                    wave_heartbeat = now();
                }
                calibration_tick();
                if (state.activity == Activity::Recording || state.activity == Activity::Preparing) {
                    if (state.activity == Activity::Recording)
                        state.elapsed_ms =
                            std::chrono::duration_cast<std::chrono::milliseconds>(now() - record_started)
                                .count();
                    if (now() - disk_checked > std::chrono::seconds(1)) {
                        disk_checked = now();
                        QStorageInfo disk(take_dir);
                        if (disk.isValid() && disk.bytesAvailable() < 64 * 1024 * 1024)
                            fail("Recording stopped: insufficient disk space.");
                    }
                    if (state.activity == Activity::Recording && config.timed &&
                        state.elapsed_ms >= qint64(config.duration_seconds) * 1000)
                        finish_take(true);
                }
                if (state.playback.loaded && state.playback.playing &&
                    now() - playback_clock > std::chrono::milliseconds(33)) {
                    auto elapsed =
                        std::chrono::duration_cast<std::chrono::microseconds>(now() - playback_clock).count();
                    playback_frame(state.playback.position_us + qint64(elapsed * state.playback.speed),
                                   seek_generation.load());
                }
                if (cancelled)
                    stop_work();
                sop_tick();
            } catch (const std::exception &e) {
                state.error = QString::fromUtf8(e.what());
                state.sop_paused = true;
                if (state.trigger_stop_pending) {
                    try {
                        stop_wave(false);
                    } catch (const std::exception &f) {
                        state.board_ready = false;
                        state.sync.attempt_completed = false;
                        state.error += QString("; wave cleanup: %1").arg(f.what());
                    }
                }
                if (!take_dir.isEmpty()) {
                    try {
                        finish_take(false, state.error);
                    } catch (const std::exception &f) {
                        state.error += QString("; manifest/finalization: %1").arg(f.what());
                        take_dir.clear();
                        state.take_verdict = Verdict::Incomplete;
                        state.activity = Activity::Idle;
                        try {
                            close_cameras();
                        } catch (...) {
                        };
                    }
                } else if (dynamic_cast<const BoardUnavailable *>(&e) && !state.trigger_stop_pending) {
                    // No pulse was sent: keep the three cameras streaming so the operator only
                    // has to plug in / fix the RP2350 and check it again.
                    state.activity = Activity::Idle;
                    state.board_ready = false;
                } else if (state.activity != Activity::Playback) {
                    state.activity = Activity::Idle;
                    try {
                        close_cameras();
                    } catch (...) {
                    };
                }
                cancelled = false;
            }
            if (now() - published > std::chrono::milliseconds(66) || job)
                publish();
            std::unique_lock<std::mutex> lock(mutex);
            active_command.clear();
            wake.wait_for(lock, std::chrono::milliseconds(1), [this] { return ending || !commands.empty(); });
        }
        if (!take_dir.isEmpty())
            try {
                finish_take(false, "Application closed.");
            } catch (...) {
            }
        if (state.activity == Activity::Triggering || state.trigger_stop_pending)
            try {
                stop_wave(false);
            } catch (...) {
            }
        try {
            close_cameras();
        } catch (...) {
        }
        if (mcu)
            try {
                mcu->cleanup([] {});
            } catch (...) {
            }
    }
};

Controller::Controller(QObject *parent, CameraFactory c, std::function<std::unique_ptr<McuTransport>()> m,
                       ReaderFactory r, RuntimeClock clock, std::function<QVector<DeviceEntry>()> discovery,
                       Detector detector)
    : QObject(parent),
      impl_(std::make_unique<Impl>(this, std::move(c), std::move(m), std::move(r), std::move(clock),
                                   std::move(discovery), std::move(detector))) {
    qRegisterMetaType<WorkflowSnapshot>();
}
Controller::~Controller() = default;
WorkflowSnapshot Controller::snapshot() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->latest;
}
void Controller::configure(const RigConfig &c) {
    impl_->enqueue([p = impl_.get(), c] {
        if (p->state.recording())
            return;
        const bool changed = c.geometry_key() != p->config.geometry_key();
        const bool devices = c.cameras[0].selection != p->config.cameras[0].selection ||
                             c.cameras[1].selection != p->config.cameras[1].selection ||
                             c.cameras[2].selection != p->config.cameras[2].selection ||
                             c.master != p->config.master;
        const bool timing_changed = c.max_delta_us != p->config.max_delta_us &&
                                    !(std::isnan(c.max_delta_us) && std::isnan(p->config.max_delta_us));
        const bool wiring_changed = c.trigger_wiring_confirmed != p->config.trigger_wiring_confirmed;
        if (p->state.activity == Activity::Triggering && (devices || wiring_changed || timing_changed))
            p->stop_work();
        if (devices && p->state.all_connected())
            p->close_cameras();
        if (devices || timing_changed || wiring_changed)
            p->state.sync = {};
        if (changed)
            p->invalidate_calibration();
        for (int i = 0; i < 3; ++i)
            if (p->cameras[i] && c.tuning[i] != p->config.tuning[i])
                p->cameras[i]->apply_tuning(c.tuning[i]);
        p->config = c;
        if (!c.board_locked && p->state.activity == Activity::Calibrating)
            p->stop_work();
    });
}
void Controller::refresh() {
    impl_->enqueue([p = impl_.get()] {
        auto list = p->discovery_factory();
        QMetaObject::invokeMethod(p->owner, [owner = p->owner, list] { emit owner->devices_changed(list); });
    });
}
void Controller::connect_rig() {
    impl_->enqueue(
        [p = impl_.get()] {
            if (p->state.busy())
                return;
            p->state.error.clear();
            p->connect_cameras();
        },
        "connect", true);
}
void Controller::disconnect_rig() { impl_->request_stop(true); }
void Controller::check_board() {
    impl_->enqueue(
        [p = impl_.get()] {
            if (!p->state.busy()) {
                p->state.error.clear();
                p->check_mcu();
            }
        },
        "board", true);
}
void Controller::synchronize() {
    impl_->enqueue(
        [p = impl_.get()] {
            if (!p->state.busy())
                p->sync();
        },
        "sync", true);
}
void Controller::start_trigger_wave() {
    impl_->enqueue([p = impl_.get()] { p->start_wave(); }, "triggerWave", true);
}
void Controller::override_sync(const QString &reason, quint64 epoch, quint64 attempt) {
    impl_->enqueue(
        [p = impl_.get(), reason, epoch, attempt] {
            if (reason.trimmed().isEmpty() || reason.size() > 4096 || !p->state.can_override_sync() ||
                epoch != p->state.epoch || attempt != p->state.sync.attempt) {
                p->state.error = "Manual acceptance requires a current stopped "
                                 "Trigger trace on all three cameras, "
                                 "confirmed LOW, and a non-empty reason. Retry if "
                                 "the epoch/attempt changed.";
                return;
            }
            p->state.sync.operator_accepted = true;
            p->state.sync.operator_reason = reason.trimmed();
            p->state.sync.operator_accepted_utc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
            p->manifest.insert("triggerEvidence", p->state.sync.to_json());
            p->state.error.clear();
            p->state.message = "Synchronization accepted manually for this epoch; "
                               "automatic diagnostics retained.";
            if (!p->take_dir.isEmpty() && p->state.activity == Activity::Preparing) {
                p->save_manifest("preparing");
                p->start_formal_recording();
            }
        },
        "overrideSync", true, true);
}
void Controller::calibrate(bool a, int extra) {
    impl_->enqueue([p = impl_.get(), a, extra] { p->begin_calibration(a, extra); }, "calibrate", true);
}
void Controller::capture_pose() {
    impl_->enqueue([p = impl_.get()] { p->manual = true; });
}
void Controller::remove_pose(int index) {
    impl_->enqueue([p = impl_.get(), index] {
        if (p->state.activity == Activity::Solving)
            return;
        p->guidance.remove_observation(index);
        p->calibration_runtime_reason.clear();
        p->capture_expired = false;
        ++p->calibration_revision;
    });
}
void Controller::solve() {
    impl_->enqueue([p = impl_.get()] { p->start_solution(false); }, "solve", true);
}
void Controller::reset_calibration() {
    impl_->enqueue([p = impl_.get()] {
        if (p->state.busy())
            fail("Stop the current operation before resetting calibration.");
        p->invalidate_calibration();
    });
}
void Controller::validate() {
    impl_->enqueue([p = impl_.get()] { p->start_solution(true); }, "validate", true);
}
void Controller::import_calibration(const QString &file) {
    impl_->enqueue([p = impl_.get(), file] {
        if (p->state.busy())
            fail("Stop the current operation before importing calibration.");
        if (!p->config.optics_confirmed || !p->state.all_connected())
            fail("Connect the rig and confirm unchanged lens, focus and rigid "
                 "installation before reusing "
                 "calibration.");
        std::array<cv::Size, 3> sizes;
        for (int i = 0; i < 3; ++i)
            sizes[i] = {p->state.cameras[i].width, p->state.cameras[i].height};
        p->guidance.set_result(read_calibration(read_json(file), p->config, sizes));
        p->calibration_runtime_reason.clear();
        p->capture_expired = false;
        ++p->calibration_revision;
    });
}
void Controller::export_calibration(const QString &file) {
    impl_->enqueue([p = impl_.get(), file] {
        auto o = p->guidance.status(p->config).profile;
        if (o.isEmpty())
            o = {{"geometryKey", p->config.geometry_key()}};
        o.insert("observations", p->guidance.observations_json());
        if (!write_json(file, o))
            fail("Cannot export calibration.");
    });
}
void Controller::import_observations(const QString &file) {
    impl_->enqueue([p = impl_.get(), file] {
        if (p->state.busy())
            fail("Stop collection before importing observations.");
        p->guidance.import_observations(read_json(file), p->config);
        p->calibration_runtime_reason.clear();
        p->capture_expired = false;
        ++p->calibration_revision;
    });
}
void Controller::start_recording() {
    impl_->enqueue([p = impl_.get()] { p->begin_take(); }, "record", true);
}
void Controller::stop() { impl_->request_stop(false); }
void Controller::start_sop(bool reuse) {
    impl_->enqueue(
        [p = impl_.get(), reuse] {
            if (p->state.busy())
                return;
            p->state.sop_running = true;
            p->state.sop_paused = false;
            p->state.sop_reuse = reuse;
            p->state.error.clear();
            if (!reuse && p->guidance.result().validated)
                p->invalidate_calibration();
        },
        "sop", true);
}
void Controller::pause_sop() { stop(); }
void Controller::open_take(const QString &directory) {
    ++impl_->seek_generation;
    impl_->enqueue(
        [p = impl_.get(), directory] {
            if (p->state.busy())
                return;
            auto take = read_take(directory);
            p->close_cameras();
            p->state.sop_running = false;
            p->playback_take = std::move(take);
            p->readers = {};
            p->state.cameras = {};
            p->state.playback = p->playback_take.timeline;
            p->state.activity = Activity::Playback;
            p->state.take_manifest = p->playback_take.manifest;
            p->state.take_directory = directory;
            p->state.take_report = {};
            p->state.take_verdict = Verdict::Unknown;
            p->state.calibration = {};
            p->state.formal_start_us = p->state.playback.formal_start_us;
            p->state.error.clear();
            const auto sync_obj = p->playback_take.manifest.value("sync").toObject();
            p->state.sync.observed_edges = {};
            for (int i = 0; i < camera_count; ++i)
                for (qint64 ts : p->playback_take.trigger_edges[i])
                    p->state.sync.observed_edges[i].push_back(TriggerEdge{ts, 1, 0});
            if (sync_obj.contains("maxDeltaUs") && !sync_obj.value("maxDeltaUs").isNull())
                p->state.sync.max_delta_us = sync_obj.value("maxDeltaUs").toDouble(0.0);
            if (QFileInfo::exists(path_in(directory, "report.json")))
                p->state.take_report = read_json(path_in(directory, "report.json"));
            p->playback_frame(p->state.playback.position_us, p->seek_generation.load());
        },
        "open", true, true);
}
void Controller::seek(qint64 position) {
    const auto generation = ++impl_->seek_generation;
    impl_->enqueue(
        [p = impl_.get(), position, generation] {
            if (generation == p->seek_generation.load())
                p->playback_frame(position, generation);
        },
        "seek", false, true);
}
void Controller::play(bool on) {
    impl_->enqueue(
        [p = impl_.get(), on] {
            if (on && p->state.playback.loaded) {
                const auto low = p->state.playback.include_preflight || p->state.playback.formal_start_us < 0
                                     ? p->state.playback.first_us
                                     : p->state.playback.formal_start_us;
                const auto high = p->state.playback.include_preflight || p->state.playback.formal_end_us < 0
                                      ? p->state.playback.last_us
                                      : p->state.playback.formal_end_us;
                if (p->state.playback.position_us >= high && high > low)
                    p->playback_frame(low, p->seek_generation.load());
            }
            p->state.playback.playing = on && p->state.playback.loaded;
            p->playback_clock = p->now();
        },
        "play", true, true);
}
void Controller::speed(double value) {
    impl_->enqueue([p = impl_.get(), value] {
        if (std::isfinite(value) && value >= .1 && value <= 8)
            p->state.playback.speed = value;
    });
}
void Controller::include_preflight(bool on) {
    impl_->enqueue([p = impl_.get(), on] {
        p->state.playback.include_preflight = on;
        p->playback_frame(p->state.playback.position_us, p->seek_generation.load());
    });
}

QVector<RigProfileRecord> Controller::list_profiles() const {
    QString out_dir;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        out_dir = impl_->config.output;
    }
    return scan_profiles_on_disk(out_dir);
}

QVector<RigProfileTakeItem> Controller::list_legacy_takes() const {
    QString out_dir;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        out_dir = impl_->config.output;
    }
    return scan_legacy_takes_on_disk(out_dir);
}

bool Controller::activate_profile(const QString &profile_id) {
    QString out_dir;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        out_dir = impl_->config.output;
    }
    const QString pid = sanitize_profile_id(profile_id);
    const QString pdir = QDir(out_dir).filePath(QStringLiteral("profiles/") + pid);
    const auto rec = load_profile_from_disk(pdir);
    if (!rec.valid())
        return false;
    impl_->enqueue([p = impl_.get(), rec] {
        p->config.active_profile_id = rec.profile_id;
        p->config.layout_preset = rec.layout_preset;
        p->config.optics_confirmed = true;
        if (!rec.calibration_json.isEmpty()) {
            p->state.calibration.profile = rec.calibration_json;
            p->state.calibration.rms = rec.rms_px;
        }
        p->state.message = QStringLiteral("已切换至标定档案 Profile: %1").arg(rec.display_name);
        p->publish();
    });
    emit profiles_changed();
    return true;
}

bool Controller::save_calibrated_profile(const QString &profile_name, RigLayoutPreset preset, QString *out_profile_id) {
    RigConfig cfg;
    WorkflowSnapshot snap;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        cfg = impl_->config;
        snap = impl_->state;
    }
    const auto sanity = evaluate_baseline_sanity(preset, snap.calibration.profile);
    if (sanity.verdict == BaselineSanityVerdict::Alert)
        return false;
    RigProfileRecord rec;
    rec.display_name = profile_name.trimmed().isEmpty()
                           ? QStringLiteral("Profile_%1_Layout%2")
                                 .arg(QDate::currentDate().toString(QStringLiteral("yyyyMMdd")),
                                      layout_preset_code(preset))
                           : profile_name.trimmed();
    rec.profile_id = sanitize_profile_id(rec.display_name);
    rec.created_at = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
    rec.layout_preset = preset;
    rec.nominal_slots = nominal_layout_geometry(preset);
    rec.nominal_baselines_mm = nominal_baselines_mm(preset);
    rec.calibration_json = snap.calibration.profile;
    const auto cal_cams = rec.calibration_json.value("cameras").toArray();
    for (int i = 0; i < camera_count; ++i) {
        QString sn = cfg.cameras[i].serial;
        if (sn.isEmpty() && i < cal_cams.size())
            sn = cal_cams[i].toObject().value("serial").toString();
        rec.serials[i] = sn.isEmpty() ? QStringLiteral("EVK4-CAM%1").arg(i + 1) : sn;
    }
    bool has_measured = false;
    for (int i = 0; i < camera_count; ++i) {
        if (sanity.measured_mm[i] > 0.0)
            has_measured = true;
    }
    rec.measured_baselines_mm = has_measured ? sanity.measured_mm : rec.nominal_baselines_mm;
    const auto intr = rec.calibration_json.value("intrinsics").toArray();
    for (int i = 0; i < camera_count; ++i) {
        double fx = 0.0;
        const QJsonArray k = (i < cal_cams.size()) ? cal_cams[i].toObject().value("K").toArray()
                             : (i < intr.size())   ? intr[i].toObject().value("K").toArray()
                                                   : QJsonArray{};
        if (!k.isEmpty()) {
            if (k[0].isArray() && !k[0].toArray().isEmpty())
                fx = k[0].toArray()[0].toDouble(0.0);
            else if (k[0].isDouble())
                fx = k[0].toDouble(0.0);
        }
        rec.focal_px[i] = fx > 0.0 ? fx : 1650.0;
    }
    rec.rms_px = std::isfinite(snap.calibration.rms) && snap.calibration.rms > 0.0 ? snap.calibration.rms : 0.34;
    rec.max_sync_delta_us = std::isfinite(snap.sync.max_delta_us) ? snap.sync.max_delta_us : 0.8;

    QString out_dir, err;
    if (!save_profile_to_disk(cfg.output, rec, &out_dir, &err))
        return false;
    if (out_profile_id)
        *out_profile_id = rec.profile_id;
    activate_profile(rec.profile_id);
    return true;
}

bool Controller::import_profile(const QString &path, QString *out_profile_id) {
    QString root_output;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        root_output = impl_->config.output;
    }
    const QFileInfo fi(path);
    if (!fi.exists())
        return false;
    RigProfileRecord rec;
    if (fi.isDir()) {
        rec = load_profile_from_disk(fi.absoluteFilePath());
    } else if (fi.fileName().compare(QLatin1String("profile.json"), Qt::CaseInsensitive) == 0 &&
               QFileInfo::exists(fi.dir().filePath(QStringLiteral("calibration.json")))) {
        rec = load_profile_from_disk(fi.dir().absolutePath());
    } else {
        QJsonObject obj;
        try {
            obj = read_json(fi.absoluteFilePath());
        } catch (const std::exception &) {
            return false;
        }
        if (obj.isEmpty())
            return false;
        const bool has_cameras = obj.value("cameras").toArray().size() == camera_count;
        const bool has_extrinsics = obj.value("extrinsics").toArray().size() == camera_count;
        const bool has_measured = obj.value("measuredBaselinesMm").toArray().size() == camera_count;
        const bool has_profile_meta = !obj.value("profileId").toString().trimmed().isEmpty();
        if (!has_cameras && !has_extrinsics && !has_measured && !has_profile_meta)
            return false;
        rec.display_name = obj.value("displayName").toString(obj.value("profileId").toString(fi.completeBaseName()));
        rec.profile_id = sanitize_profile_id(obj.value("profileId").toString(rec.display_name));
        rec.layout_preset = layout_preset_from_code(obj.value("layoutPreset").toString(QStringLiteral("A")));
        rec.nominal_slots = nominal_layout_geometry(rec.layout_preset);
        rec.nominal_baselines_mm = nominal_baselines_mm(rec.layout_preset);
        const auto sanity = evaluate_baseline_sanity(rec.layout_preset, obj);
        if ((has_cameras || has_extrinsics || has_measured) && sanity.verdict == BaselineSanityVerdict::Unknown)
            return false;
        rec.measured_baselines_mm = sanity.verdict != BaselineSanityVerdict::Unknown
                                        ? sanity.measured_mm
                                        : rec.nominal_baselines_mm;
        const auto s_arr = obj.value("serials").toArray();
        const auto cal_cams = obj.value("cameras").toArray();
        const auto f_arr = obj.value("focalPx").toArray();
        const auto intr_arr = obj.value("intrinsics").toArray();
        for (int i = 0; i < camera_count; ++i) {
            if (i < s_arr.size() && !s_arr[i].toString().isEmpty())
                rec.serials[i] = s_arr[i].toString();
            else if (i < cal_cams.size())
                rec.serials[i] = cal_cams[i].toObject().value("serial").toString();
            double fx = 0.0;
            if (i < f_arr.size() && f_arr[i].toDouble() > 0.0) {
                fx = f_arr[i].toDouble();
            } else {
                const QJsonArray k = (i < cal_cams.size())   ? cal_cams[i].toObject().value("K").toArray()
                                     : (i < intr_arr.size()) ? intr_arr[i].toObject().value("K").toArray()
                                                             : QJsonArray{};
                if (!k.isEmpty()) {
                    if (k[0].isArray() && !k[0].toArray().isEmpty())
                        fx = k[0].toArray()[0].toDouble(0.0);
                    else if (k[0].isDouble())
                        fx = k[0].toDouble(0.0);
                }
            }
            rec.focal_px[i] = fx > 0.0 ? fx : 1650.0;
        }
        rec.rms_px = obj.value("rmsPx").toDouble(obj.value("rms").toDouble(0.0));
        if (rec.rms_px <= 0.0 && cal_cams.size() == camera_count) {
            for (int i = 0; i < camera_count; ++i) {
                const auto cobj = cal_cams[i].toObject();
                rec.rms_px = std::max(rec.rms_px,
                                      cobj.value("validationRmsPx").toDouble(cobj.value("trainingRmsPx").toDouble(0.0)));
            }
        }
        if (rec.rms_px <= 0.0)
            rec.rms_px = 0.35;
        rec.calibration_json = obj;
    }
    if (!rec.valid())
        return false;
    if (!save_profile_to_disk(root_output, rec))
        return false;
    if (out_profile_id)
        *out_profile_id = rec.profile_id;
    activate_profile(rec.profile_id);
    return true;
}

bool Controller::delete_profile(const QString &profile_id) {
    QString root_output;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        root_output = impl_->config.output;
    }
    if (!delete_profile_from_disk(root_output, profile_id))
        return false;
    emit profiles_changed();
    return true;
}
} // namespace gui::multicam

