#include "multicam/controller.h"
#include "multicam/workspace.h"
#include <QApplication>
#include <QCryptographicHash>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QKeyEvent>
#include <QLineEdit>
#include <QListWidget>
#include <QTemporaryDir>
#include <QtEndian>
#include <atomic>
#include <chrono>
#include <gtest/gtest.h>
#include <thread>

using namespace gui::multicam;
namespace {
struct FakeRig {
    std::atomic<qint64> time{100000};
    std::array<CameraSnapshot, 3> state;
    QVector<int> starts;
    QVector<int> open_order;
    std::atomic_bool reject_board{false}, low{true}, unplug{false};
    std::atomic_bool hold_check{false}, check_entered{false}, bounded_history{false};
    std::atomic_bool cleanup_throws{false}, burst_low{true};
    std::atomic_bool invalid_watermark{false};
    std::atomic_bool supply_windows{false};
    std::atomic_bool calibration_ingress_ready{true};
    std::atomic<double> decode_ratio{std::numeric_limits<double>::quiet_NaN()};
    std::atomic<qint64> calibration_progress_lag{0};
    std::array<std::atomic<qint64>, 3> timestamp_offsets{};
    std::atomic_int formal_edges{6}, wave_starts{0}, wave_heartbeats{0};
    std::atomic_bool wave_active{false}, heartbeat_fails{false};
    std::atomic_int opens{0}, pulses{0}, stops{0}, checks{0}, cancelled_checks{0};
    std::atomic_int rendered_snapshots{0}, statistic_snapshots{0};
    RuntimeClock clock() {
        return {
            [this] { return std::chrono::steady_clock::time_point(std::chrono::microseconds(time.load())); },
            [this](auto duration) {
                time += std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
            }};
    }
};
class FakeCamera final : public CameraStream {
  public:
    explicit FakeCamera(std::shared_ptr<FakeRig> rig) : rig_(std::move(rig)) {}
    void open(const CameraBinding &b, bool) override {
        i_ = b.serial.toInt();
        ++rig_->opens;
        rig_->open_order.push_back(i_);
        auto &s = rig_->state[i_];
        s = {};
        s.connected = true;
        s.binding = b;
        s.width = 1280;
        s.height = 720;
    }
    void start(const QString &raw) override {
        running_ = true;
        rig_->starts.push_back(i_);
        if (!raw.isEmpty()) {
            QFile f(raw);
            if (!f.open(QIODevice::WriteOnly) || f.write("test RAW fixture") < 0)
                throw std::runtime_error("mock write failed");
        }
    }
    void pump() override {
        if (!running_)
            return;
        if (rig_->unplug)
            throw std::runtime_error("injected disconnect");
        auto &s = rig_->state[i_];
        if (s.first_timestamp_us < 0)
            s.first_timestamp_us = rig_->time.load();
        s.latest_timestamp_us = rig_->time.load() + rig_->timestamp_offsets[i_].load();
        ++s.event_count;
    }
    void stop() override {
        if (running_) {
            running_ = false;
            ++rig_->stops;
            rig_->state[i_].connected = false;
        }
    }
    CameraSnapshot snapshot() const override {
        ++rig_->rendered_snapshots;
        return rig_->state[i_];
    }
    CameraSnapshot statistics() const override {
        ++rig_->statistic_snapshots;
        auto result = rig_->state[i_];
        result.metadata.insert("calibrationIngressReady", rig_->calibration_ingress_ready.load());
        result.decode_realtime_ratio = rig_->decode_ratio.load();
        result.metadata.insert("calibrationProgressLagMs", rig_->calibration_progress_lag.load());
        return result;
    }
    qint64 watermark() const override {
        return rig_->invalid_watermark ? -1 : rig_->time.load() + rig_->timestamp_offsets[i_].load();
    }
    std::optional<CountWindow> window(qint64 start) const override {
        if (!rig_->supply_windows || !counting_)
            return {};
        return CountWindow{start, cv::Mat::zeros(720, 1280, CV_32S), cv::Mat::zeros(720, 1280, CV_32S)};
    }
    void collect_windows(bool enabled) override { counting_ = enabled; }

  private:
    std::shared_ptr<FakeRig> rig_;
    int i_ = 0;
    bool running_ = false;
    bool counting_ = false;
};
class FakeMcu final : public McuTransport {
  public:
    explicit FakeMcu(std::shared_ptr<FakeRig> rig) : rig_(std::move(rig)) {}
    McuResult check(const QString &, std::atomic_bool &cancelled, const Pump &) override {
        ++rig_->checks;
        rig_->check_entered = true;
        while (rig_->hold_check && !cancelled)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (cancelled) {
            ++rig_->cancelled_checks;
            return {false, false, "cancelled", {}};
        }
        return {!rig_->reject_board, rig_->low, "mock bound board", {}};
    }
    McuResult burst(int n, std::atomic_bool &cancelled, const Pump &pump) override {
        if (cancelled)
            return {false, false, "cancelled", {}};
        rig_->pulses += n;
        const auto first = rig_->time.load();
        for (int i = 0; i < 3; ++i) {
            auto &edges = rig_->state[i].triggers;
            if (rig_->bounded_history && n == 1 && edges.isEmpty())
                for (int j = 0; j < 4096; ++j)
                    edges.push_back({first - 4096 + j, 0, 1});
            for (int j = 0; j < (n == 3 ? rig_->formal_edges.load() : n * 2); ++j) {
                edges.push_back({first + j * 1000000 + i, j % 2 ? 0 : 1, 0});
                if (rig_->bounded_history && edges.size() > 4096)
                    edges.removeFirst();
            }
        }
        rig_->time += n * 2000000;
        pump();
        return {true, rig_->burst_low, "mock burst", {{"pulses", n}}};
    }
    McuResult cleanup(const Pump &) override {
        if (rig_->cleanup_throws)
            throw std::runtime_error("injected cleanup failure");
        if (rig_->low)
            rig_->wave_active = false;
        return {rig_->low, rig_->low, "mock cleanup", {}};
    }
    McuResult start_continuous(std::atomic_bool &cancelled, const Pump &pump) override {
        if (cancelled)
            return {false, false, "cancelled", {}};
        ++rig_->wave_starts;
        rig_->wave_active = true;
        const auto first = rig_->time.load();
        for (int i = 0; i < 3; ++i)
            for (int edge = 0; edge < 4; ++edge)
                rig_->state[i].triggers.push_back({first + edge * 1000000 + i, edge % 2, 0});
        rig_->time += 4000000;
        pump();
        return {true, false, "mock wave active", {{"active", true}, {"drive", 12}}};
    }
    McuResult keepalive_continuous(std::atomic_bool &, const Pump &) override {
        ++rig_->wave_heartbeats;
        return {!rig_->heartbeat_fails && rig_->wave_active, false, "mock heartbeat", {}};
    }

  private:
    std::shared_ptr<FakeRig> rig_;
};
class FakeReader : public RawReader {
  public:
    explicit FakeReader(TakeCamera camera) : camera_(std::move(camera)) {}
    CameraSnapshot frame(qint64, qint64 target, const std::function<bool()> &cancelled) override {
        if (cancelled())
            throw std::runtime_error("cancelled");
        CameraSnapshot s;
        s.binding.serial = camera_.serial;
        s.latest_timestamp_us = target;
        s.event_count = camera_.count;
        s.first_timestamp_us = camera_.first;
        s.metadata.insert("protocolValidationAvailable", true);
        return s;
    }
    CameraSnapshot scan(const std::function<bool()> &cancelled) override {
        return frame(0, camera_.last, cancelled);
    }

  private:
    TakeCamera camera_;
};
bool until(const std::function<bool()> &condition, int ms = 4000) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) {
        QApplication::processEvents();
        if (condition())
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return condition();
}
RigConfig configuration(const QString &output) {
    RigConfig c;
    for (int i = 0; i < 3; ++i)
        c.cameras[i] = {QString("vendor:silky:%1").arg(i), QString::number(i), c.cameras[i].position};
    c.master = 1;
    c.roles_confirmed = true;
    c.trigger_wiring_confirmed = true;
    c.optics_confirmed = true;
    c.board_locked = true;
    c.square_mm = 25;
    c.max_delta_us = 5;
    c.max_rms_px = .5;
    c.output = output;
    return c;
}
QJsonObject profile(const RigConfig &c) {
    gui_algo::rig::Result r;
    r.trained = r.validated = true;
    r.reference = c.reference;
    r.square_m = .025;
    for (int i = 0; i < 3; ++i) {
        r.models[i] = {{1280, 720},
                       (cv::Mat_<double>(3, 3) << 760, 0, 640, 0, 760, 360, 0, 0, 1),
                       cv::Mat::zeros(1, 5, CV_64F)};
        r.training_rms[i] = r.validation_rms[i] = .1;
    }
    for (int i = 0; i < c.training_target; ++i) {
        r.training_ids.push_back(i);
        r.training_views.push_back({.1, .1, .1});
    }
    for (int i = 0; i < c.validation_target; ++i)
        r.validation_views.push_back({.1, .1, .1});
    return calibration_json(r, c);
}
std::unique_ptr<Controller> controller(const std::shared_ptr<FakeRig> &rig, ReaderFactory readers = {},
                                       Controller::Detector detector = {}) {
    if (!readers)
        readers = [](const auto &c) { return std::make_unique<FakeReader>(c); };
    return std::make_unique<Controller>(
        nullptr, [rig] { return std::make_unique<FakeCamera>(rig); },
        [rig] { return std::make_unique<FakeMcu>(rig); }, std::move(readers), rig->clock(),
        [] { return QVector<DeviceEntry>{}; }, std::move(detector));
}
void connect_and_sync(Controller &c, const RigConfig &config) {
    c.configure(config);
    c.connect_rig();
    ASSERT_TRUE(until([&] { return c.snapshot().all_connected(); }));
    c.check_board();
    ASSERT_TRUE(until([&] { return c.snapshot().board_ready; }));
    c.synchronize();
    ASSERT_TRUE(until([&] { return c.snapshot().sync.verdict == Verdict::Passed; }))
        << c.snapshot().error.toStdString();
}

TEST(MultiCameraRuntime, SlowGuiCoalescesPreviewWorkWhileEventConsumptionContinues) {
    QTemporaryDir output;
    auto rig = std::make_shared<FakeRig>();
    auto c = controller(rig);
    c->configure(configuration(output.path()));
    c->connect_rig();
    ASSERT_TRUE(until([&] { return c->snapshot().all_connected(); }));
    // Deliberately do not process GUI events while advancing the injected acquisition clock.
    rig->time += 100000;
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    const auto renders = rig->rendered_snapshots.load();
    const auto reads = rig->statistic_snapshots.load();
    const auto count = c->snapshot().cameras[0].event_count;
    rig->time += 100000;
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    EXPECT_EQ(rig->rendered_snapshots.load(), renders);
    EXPECT_GT(rig->statistic_snapshots.load(), reads);
    EXPECT_GT(c->snapshot().cameras[0].event_count, count);
    QCoreApplication::processEvents();
    rig->time += 100000;
    ASSERT_TRUE(until([&] { return rig->rendered_snapshots.load() > renders; }));
}

TEST(MultiCameraRuntime, CancelAndRestartDiscardsOldDetectionAndItsException) {
    for (bool throw_old_result : {false, true}) {
        struct Gate {
            std::atomic_int entered{0};
            std::atomic_bool first{false}, finish{false};
        };
        auto gate = std::make_shared<Gate>();
        auto rig = std::make_shared<FakeRig>();
        rig->supply_windows = true;
        auto c = controller(rig, {}, [gate, throw_old_result](const auto &windows) {
            const int attempt = ++gate->entered;
            while (!gate->finish && !(attempt == 1 && gate->first))
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if (attempt == 1 && throw_old_result)
                throw std::runtime_error("stale detection failure");
            DetectedGroup result;
            result.start_us = windows[0].start_us;
            for (int camera = 0; camera < 3; ++camera) {
                result.sizes[camera] = {1280, 720};
                for (int y = 0; y < 6; ++y)
                    for (int x = 0; x < 9; ++x)
                        result.corners[camera].emplace_back(250 + x * 45, 160 + y * 45);
            }
            return result;
        });
        struct Release {
            std::shared_ptr<Gate> gate;
            ~Release() { gate->finish = true; }
        } release{gate};
        QTemporaryDir output;
        connect_and_sync(*c, configuration(output.path()));
        c->calibrate(false);
        ASSERT_TRUE(until([&] { return gate->entered == 1; }));
        c->stop();
        ASSERT_TRUE(until([&] { return !c->snapshot().busy(); }));
        c->calibrate(false);
        ASSERT_TRUE(until([&] { return c->snapshot().activity == Activity::Calibrating; }));
        gate->first = true;
        ASSERT_TRUE(until([&] { return gate->entered >= 2; }));
        rig->time += 100000;
        const auto through = rig->time.load();
        ASSERT_TRUE(until([&] { return c->snapshot().cameras[0].latest_timestamp_us >= through; }));
        const auto state = c->snapshot();
        EXPECT_TRUE(state.cameras[0].corners.isEmpty());
        EXPECT_EQ(state.calibration.stable_progress, 0);
        EXPECT_EQ(state.calibration.common_count, 0);
        EXPECT_TRUE(state.error.isEmpty());
    }
}

TEST(MultiCameraRuntime, CalibrationRoundExpiresWithoutAcceptingLateDetectionAndCanResume) {
    struct Gate {
        std::atomic_int entered{0};
        std::atomic_bool release{false};
    };
    auto gate = std::make_shared<Gate>();
    auto rig = std::make_shared<FakeRig>();
    rig->supply_windows = true;
    auto c = controller(rig, {}, [gate](const auto &windows) {
        ++gate->entered;
        while (!gate->release)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        DetectedGroup g;
        g.start_us = windows[0].start_us;
        for (int i = 0; i < 3; ++i) {
            g.sizes[i] = {1280, 720};
            for (int y = 0; y < 6; ++y)
                for (int x = 0; x < 9; ++x)
                    g.corners[i].emplace_back(250 + 45 * x, 160 + 45 * y);
        }
        return g;
    });
    struct Release {
        std::shared_ptr<Gate> gate;
        ~Release() { gate->release = true; }
    } release{gate};
    QTemporaryDir output;
    auto config = configuration(output.path());
    config.calibration_capture_seconds = 3;
    connect_and_sync(*c, config);
    c->calibrate(true);
    ASSERT_TRUE(until([&] { return gate->entered == 1; }));
    rig->time += 1000000;
    ASSERT_TRUE(until([&] { return c->snapshot().calibration.capture_remaining_ms <= 2000; }));
    EXPECT_TRUE(c->snapshot().calibration.collecting);
    rig->time += 2100000;
    ASSERT_TRUE(until([&] { return c->snapshot().calibration.capture_expired; }));
    EXPECT_FALSE(c->snapshot().busy());
    EXPECT_EQ(c->snapshot().calibration.common_count, 0);
    EXPECT_EQ(c->snapshot().calibration.capture_remaining_ms, 0);
    EXPECT_TRUE(c->snapshot().sop_paused);
    gate->release = true;
    rig->time += 100000;
    ASSERT_TRUE(until([&] { return c->snapshot().cameras[0].latest_timestamp_us >= rig->time; }));
    EXPECT_TRUE(c->snapshot().cameras[0].corners.isEmpty());
    c->calibrate(true);
    ASSERT_TRUE(until([&] { return c->snapshot().calibration.collecting; }));
    EXPECT_FALSE(c->snapshot().calibration.capture_expired);
    EXPECT_GT(c->snapshot().calibration.capture_remaining_ms, 0);
    c->stop();
    ASSERT_TRUE(until([&] { return !c->snapshot().busy(); }));
}

TEST(MultiCameraRuntime, NoCandidateIsDispatchedUntilAllPostClickIngressBarriersAreReady) {
    auto rig = std::make_shared<FakeRig>();
    rig->supply_windows = true;
    rig->calibration_ingress_ready = false;
    auto calls = std::make_shared<std::atomic_int>(0);
    auto c = controller(rig, {}, [calls](const auto &windows) {
        ++*calls;
        DetectedGroup result;
        result.start_us = windows[0].start_us;
        return result;
    });
    QTemporaryDir output;
    connect_and_sync(*c, configuration(output.path()));
    c->calibrate(true);
    ASSERT_TRUE(until([&] { return c->snapshot().calibration.collecting; }));
    rig->time += 200000;
    ASSERT_TRUE(until([&] { return c->snapshot().calibration.reason.contains("draining"); }));
    EXPECT_EQ(*calls, 0);
    EXPECT_EQ(c->snapshot().calibration.common_count, 0);
    rig->calibration_ingress_ready = true;
    rig->time += 100000;
    ASSERT_TRUE(until([&] { return *calls > 0; }));
    c->stop();
    ASSERT_TRUE(until([&] { return !c->snapshot().busy(); }));
}

TEST(MultiCameraRuntime, BehindRealTimeOrMisalignedDecodersCannotStartDetection) {
    for (const auto &reason : {QString("0.50x"), QString("1500 ms behind"), QString("differs by 500 ms")}) {
        auto rig = std::make_shared<FakeRig>();
        rig->supply_windows = true;
        auto calls = std::make_shared<std::atomic_int>(0);
        auto c = controller(rig, {}, [calls](const auto &windows) {
            ++*calls;
            DetectedGroup result;
            result.start_us = windows[0].start_us;
            return result;
        });
        QTemporaryDir output;
        const auto config = configuration(output.path());
        connect_and_sync(*c, config);
        // Configure each scenario before the capture command, not halfway through a simulated tick.
        rig->decode_ratio = reason == "0.50x" ? .5 : 1.;
        rig->calibration_progress_lag = reason == "1500 ms behind" ? 1500 : 0;
        rig->timestamp_offsets[0] = reason == "differs by 500 ms" ? 500000 : 0;
        c->calibrate(true);
        ASSERT_TRUE(until([&] { return c->snapshot().calibration.reason.contains(reason); }))
            << reason.toStdString();
        EXPECT_EQ(*calls, 0);
        rig->decode_ratio = 1;
        rig->calibration_progress_lag = 0;
        rig->timestamp_offsets[0] = 0;
        rig->time += 100000;
        c->configure(config); // A command barrier also forces publication with the injected clock paused.
        ASSERT_TRUE(until([&] { return *calls > 0; }));
        c->stop();
        ASSERT_TRUE(until([&] { return !c->snapshot().busy(); }));
    }
}

TEST(MultiCameraRuntime, CandidateThatBecomesOldDuringDetectionCannotUpdateCornersOrStability) {
    struct Gate {
        std::atomic_int entered{0};
        std::atomic_bool first{false}, release{false};
    };
    auto gate = std::make_shared<Gate>();
    auto rig = std::make_shared<FakeRig>();
    rig->supply_windows = true;
    auto c = controller(rig, {}, [gate](const auto &windows) {
        const int call = ++gate->entered;
        while (!gate->release && !(call == 1 && gate->first))
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        DetectedGroup group;
        group.start_us = windows[0].start_us;
        for (int i = 0; i < 3; ++i) {
            group.sizes[i] = {1280, 720};
            for (int y = 0; y < 6; ++y)
                for (int x = 0; x < 9; ++x)
                    group.corners[i].emplace_back(250 + x * 45, 160 + y * 45);
        }
        return group;
    });
    struct Release {
        std::shared_ptr<Gate> gate;
        ~Release() { gate->release = true; }
    } release{gate};
    QTemporaryDir output;
    connect_and_sync(*c, configuration(output.path()));
    c->calibrate(true);
    ASSERT_TRUE(until([&] { return gate->entered == 1; }));
    rig->time += 500000;
    gate->first = true;
    ASSERT_TRUE(until([&] { return gate->entered >= 2; }));
    rig->time += 100000;
    ASSERT_TRUE(until([&] { return c->snapshot().calibration.reason.contains("old candidate discarded"); }));
    EXPECT_TRUE(c->snapshot().cameras[0].corners.isEmpty());
    EXPECT_EQ(c->snapshot().calibration.common_count, 0);
    EXPECT_EQ(c->snapshot().calibration.stable_progress, 0);
    c->stop();
    ASSERT_TRUE(until([&] { return !c->snapshot().busy(); }));
}
bool write_playback_take(const QString &directory, int count = 3) {
    QFile raw(QDir(directory).filePath("camera.raw"));
    if (!raw.open(QIODevice::WriteOnly) || raw.write("fixture") != 7)
        return false;
    raw.close();
    QJsonArray cameras;
    for (int i = 0; i < count; ++i)
        cameras.append(QJsonObject{{"serial", QString::number(i)},
                                   {"rawFile", "camera.raw"},
                                   {"eventCount", 10},
                                   {"firstTimestampUs", 1000000},
                                   {"lastTimestampUs", 20000000}});
    return write_json(QDir(directory).filePath("session.json"),
                      {{"state", "completed"}, {"cameras", cameras}});
}
bool write_evt2(const QString &path, bool invalid_coordinate = false) {
    QFile raw(path);
    if (!raw.open(QIODevice::WriteOnly))
        return false;
    QByteArray bytes("% format EVT2\n% geometry 640x480\n% integrator_name Prophesee\n"
                     "% plugin_name hal_plugin_prophesee\n% end\n");
    for (quint32 i = 0; i < 3; ++i) {
        const quint32 t = 1000000 + i * 50000;
        const quint32 x = invalid_coordinate ? 700 : (i + 1) * 100;
        for (const quint32 word : {(8u << 28) | (t >> 6), (1u << 28) | ((t & 63u) << 22) | (x << 11) | 20u}) {
            const auto little = qToLittleEndian(word);
            bytes.append(reinterpret_cast<const char *>(&little), sizeof(little));
        }
    }
    return raw.write(bytes) == bytes.size();
}
TEST(MultiCameraRuntime, StartsSlavesBeforeMasterAndReconnectInvalidatesEvidence) {
    QTemporaryDir dir;
    auto rig = std::make_shared<FakeRig>();
    auto c = controller(rig);
    connect_and_sync(*c, configuration(dir.path()));
    ASSERT_GE(rig->starts.size(), 3);
    EXPECT_EQ(rig->starts[2], 1);
    EXPECT_EQ(rig->open_order, (QVector<int>{2, 0, 1}));
    const auto epoch = c->snapshot().epoch;
    c->connect_rig();
    ASSERT_TRUE(until([&] { return c->snapshot().epoch > epoch && c->snapshot().all_connected(); }));
    EXPECT_EQ(c->snapshot().sync.verdict, Verdict::Unknown);
}
TEST(MultiCameraRuntime, EveryTakeReopensAndUsesMeasuredPreflightBoundary) {
    QTemporaryDir dir;
    auto rig = std::make_shared<FakeRig>();
    auto c = controller(rig);
    auto config = configuration(dir.path());
    connect_and_sync(*c, config);
    const auto old_epoch = c->snapshot().epoch;
    const auto file = dir.filePath("profile.json");
    ASSERT_TRUE(write_json(file, profile(config)));
    c->import_calibration(file);
    ASSERT_TRUE(until([&] { return c->snapshot().calibration.verdict == Verdict::Passed; }))
        << c->snapshot().error.toStdString();
    c->start_recording();
    ASSERT_TRUE(until([&] { return c->snapshot().activity == Activity::Recording; }))
        << c->snapshot().error.toStdString();
    auto s = c->snapshot();
    EXPECT_GT(s.epoch, old_epoch);
    EXPECT_EQ(rig->opens, 6);
    EXPECT_EQ(rig->pulses, 8);
    EXPECT_NE(s.formal_start_us, 14000000);
    EXPECT_GT(s.formal_start_us, 20000000);
    rig->time += 2000000;
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    c->stop();
    ASSERT_TRUE(until([&] { return !c->snapshot().recording() && !c->snapshot().take_report.isEmpty(); }));
    s = c->snapshot();
    EXPECT_EQ(s.take_manifest.value("state").toString(), "completed");
    EXPECT_TRUE(s.take_report.value("countsAndProtocolPass").toBool());
    EXPECT_EQ(s.take_report.value("decoderTail").toString(), "unknown");
    EXPECT_EQ(s.take_verdict, Verdict::Incomplete);
    EXPECT_TRUE(QFile::exists(QDir(s.take_directory).filePath("calibration.json")));
    auto take = read_take(s.take_directory);
    EXPECT_LT(take.timeline.first_us, take.timeline.formal_start_us);
    EXPECT_GE(take.timeline.formal_end_us, take.timeline.formal_start_us);
}
TEST(MultiCameraRuntime, UnknownOutputStateFailsSynchronization) {
    QTemporaryDir dir;
    auto rig = std::make_shared<FakeRig>();
    auto c = controller(rig);
    auto config = configuration(dir.path());
    c->configure(config);
    c->connect_rig();
    ASSERT_TRUE(until([&] { return c->snapshot().all_connected(); }));
    rig->low = false;
    c->synchronize();
    ASSERT_TRUE(until([&] { return !c->snapshot().error.isEmpty(); }));
    EXPECT_EQ(c->snapshot().sync.verdict, Verdict::Failed);
    EXPECT_FALSE(c->snapshot().board_ready);
    EXPECT_EQ(rig->pulses.load(), 0);
}
TEST(MultiCameraRuntime, MissingBoardKeepsCamerasConnectedAndReportsTheReason) {
    QTemporaryDir dir;
    auto rig = std::make_shared<FakeRig>();
    auto c = controller(rig);
    c->configure(configuration(dir.path()));
    c->connect_rig();
    ASSERT_TRUE(until([&] { return c->snapshot().all_connected(); }));
    const auto epoch = c->snapshot().epoch;
    rig->reject_board = true;
    c->check_board();
    ASSERT_TRUE(until([&] { return !c->snapshot().error.isEmpty(); }));
    EXPECT_FALSE(c->snapshot().board_ready);
    EXPECT_TRUE(c->snapshot().all_connected());
    EXPECT_EQ(c->snapshot().epoch, epoch);
    EXPECT_EQ(rig->stops.load(), 0);
    EXPECT_EQ(rig->pulses.load(), 0);
    rig->reject_board = false;
    c->check_board();
    ASSERT_TRUE(until([&] { return c->snapshot().board_ready; }));
    EXPECT_TRUE(c->snapshot().error.isEmpty());
    EXPECT_EQ(rig->opens.load(), 3);
}
TEST(MultiCameraRuntime, ManualSyncAcceptancePreservesFailureAndExpiresOnRetryOrReconnect) {
    QTemporaryDir dir;
    auto rig = std::make_shared<FakeRig>();
    rig->formal_edges = 4;
    auto c = controller(rig);
    const auto config = configuration(dir.path());
    c->configure(config);
    c->connect_rig();
    ASSERT_TRUE(until([&] { return c->snapshot().all_connected(); }));
    c->synchronize();
    ASSERT_TRUE(until([&] { return c->snapshot().sync.attempt_completed; }));
    auto s = c->snapshot();
    ASSERT_EQ(s.sync.verdict, Verdict::Failed);
    ASSERT_TRUE(s.can_override_sync());
    ASSERT_EQ(s.sync.observed_edges[0].size(), 4);
    const auto reason = s.sync.reason;
    c->override_sync(" ", s.epoch, s.sync.attempt);
    ASSERT_TRUE(until([&] { return c->snapshot().error.contains("non-empty reason"); }));
    EXPECT_FALSE(c->snapshot().sync.operator_accepted);
    c->override_sync("Observed aligned post-startup edges", s.epoch, s.sync.attempt);
    ASSERT_TRUE(until([&] { return c->snapshot().sync.operator_accepted; }));
    s = c->snapshot();
    EXPECT_TRUE(s.sync.accepted(s.epoch, config.max_delta_us));
    EXPECT_EQ(s.sync.verdict, Verdict::Failed);
    EXPECT_EQ(s.sync.reason, reason);
    EXPECT_FALSE(s.sync.to_json().value("timingMeasured").toBool());
    EXPECT_EQ(s.sync.to_json().value("acceptanceSource").toString(), "manual");
    EXPECT_FALSE(blockers(Step::Capture, config, s).isEmpty());
    const auto old_attempt = s.sync.attempt;
    c->synchronize();
    ASSERT_TRUE(until(
        [&] { return c->snapshot().sync.attempt > old_attempt && c->snapshot().sync.attempt_completed; }));
    EXPECT_FALSE(c->snapshot().sync.operator_accepted);
    c->override_sync("stale dialog", s.epoch, old_attempt);
    ASSERT_TRUE(until([&] { return c->snapshot().error.contains("epoch/attempt"); }));
    EXPECT_FALSE(c->snapshot().sync.operator_accepted);
    s = c->snapshot();
    c->override_sync("Reviewed retry", s.epoch, s.sync.attempt);
    ASSERT_TRUE(until([&] { return c->snapshot().sync.operator_accepted; }));
    c->connect_rig();
    ASSERT_TRUE(until([&] { return c->snapshot().epoch > s.epoch && c->snapshot().all_connected(); }));
    EXPECT_FALSE(c->snapshot().sync.operator_accepted);
    EXPECT_FALSE(c->snapshot().can_override_sync());
}
TEST(MultiCameraRuntime, EveryTakeRequiresFreshManualAcceptanceAndPersistsTheOverride) {
    QTemporaryDir dir;
    auto rig = std::make_shared<FakeRig>();
    auto c = controller(rig);
    const auto config = configuration(dir.path());
    connect_and_sync(*c, config);
    const auto file = dir.filePath("profile.json");
    ASSERT_TRUE(write_json(file, profile(config)));
    c->import_calibration(file);
    ASSERT_TRUE(until([&] { return c->snapshot().calibration.verdict == Verdict::Passed; }));
    const auto old_epoch = c->snapshot().epoch;
    rig->formal_edges = 4;
    c->start_recording();
    ASSERT_TRUE(until(
        [&] { return c->snapshot().activity == Activity::Preparing && c->snapshot().can_override_sync(); }));
    auto s = c->snapshot();
    EXPECT_GT(s.epoch, old_epoch);
    EXPECT_LT(s.formal_start_us, 0);
    EXPECT_EQ(s.elapsed_ms, 0);
    EXPECT_EQ(read_json(QDir(s.take_directory).filePath("session.json")).value("state"), "preparing");
    c->override_sync("This Take preflight trace reviewed", s.epoch, s.sync.attempt);
    ASSERT_TRUE(until([&] { return c->snapshot().activity == Activity::Recording; }));
    s = c->snapshot();
    EXPECT_GE(s.formal_start_us, 0);
    auto evidence = s.take_manifest.value("sync").toObject();
    EXPECT_EQ(evidence.value("verdict").toString(), "failed");
    EXPECT_EQ(evidence.value("acceptanceSource").toString(), "manual");
    EXPECT_EQ(evidence.value("manualOverride").toObject().value("reason").toString(),
              "This Take preflight trace reviewed");
    rig->time += 1000000;
    c->stop();
    ASSERT_TRUE(until([&] { return !c->snapshot().recording(); }));
    EXPECT_TRUE(c->snapshot()
                    .take_manifest.value("sync")
                    .toObject()
                    .value("manualOverride")
                    .toObject()
                    .value("accepted")
                    .toBool());
    EXPECT_FALSE(c->snapshot().sync.operator_accepted);
}
TEST(MultiCameraRuntime, ContinuousWaveStopsBeforeManualAcceptanceAndDoesNotAutoRestart) {
    QTemporaryDir dir;
    auto rig = std::make_shared<FakeRig>();
    auto c = controller(rig);
    connect_and_sync(*c, configuration(dir.path()));
    c->start_trigger_wave();
    ASSERT_TRUE(until([&] { return c->snapshot().activity == Activity::Triggering; }));
    EXPECT_TRUE(rig->wave_active);
    EXPECT_TRUE(c->snapshot().trigger_stop_pending);
    EXPECT_FALSE(c->snapshot().can_override_sync());
    for (int i = 0; i < 20; ++i)
        c->start_trigger_wave();
    rig->time += 2000000;
    ASSERT_TRUE(until([&] { return rig->wave_heartbeats.load() > 0; }));
    EXPECT_EQ(rig->wave_starts.load(), 1);
    c->stop();
    ASSERT_TRUE(until([&] { return !c->snapshot().busy() && c->snapshot().board_ready; }));
    EXPECT_FALSE(rig->wave_active);
    auto s = c->snapshot();
    EXPECT_TRUE(s.can_override_sync());
    EXPECT_EQ(s.sync.observed_edges[0].size(), 4);
    EXPECT_EQ(s.sync.verdict, Verdict::Incomplete);
    c->override_sync("Continuous trace visually aligned", s.epoch, s.sync.attempt);
    ASSERT_TRUE(until([&] { return c->snapshot().sync.operator_accepted; }));
    EXPECT_EQ(rig->wave_starts.load(), 1);
}
TEST(MultiCameraRuntime, ContinuousWaveRequiresWiringConfirmationAndStopsOnHeartbeatFailure) {
    QTemporaryDir dir;
    auto rig = std::make_shared<FakeRig>();
    auto c = controller(rig);
    auto config = configuration(dir.path());
    config.trigger_wiring_confirmed = false;
    c->configure(config);
    c->connect_rig();
    ASSERT_TRUE(until([&] { return c->snapshot().all_connected(); }));
    c->start_trigger_wave();
    ASSERT_TRUE(until([&] { return !c->snapshot().error.isEmpty(); }));
    EXPECT_EQ(rig->wave_starts.load(), 0);
    config.trigger_wiring_confirmed = true;
    connect_and_sync(*c, config);
    c->start_trigger_wave();
    ASSERT_TRUE(until([&] { return rig->wave_active.load(); }));
    rig->heartbeat_fails = true;
    rig->time += 2000000;
    ASSERT_TRUE(until([&] { return !rig->wave_active.load() && !c->snapshot().error.isEmpty(); }));
    EXPECT_FALSE(c->snapshot().can_override_sync());
}
TEST(MultiCameraRuntime, UnconfirmedContinuousStopBlocksFurtherWorkUntilCleanupSucceeds) {
    QTemporaryDir dir;
    auto rig = std::make_shared<FakeRig>();
    auto c = controller(rig);
    connect_and_sync(*c, configuration(dir.path()));
    c->start_trigger_wave();
    ASSERT_TRUE(until([&] { return rig->wave_active.load(); }));
    rig->low = false;
    c->stop();
    ASSERT_TRUE(until(
        [&] { return c->snapshot().activity == Activity::Idle && c->snapshot().trigger_stop_pending; }));
    EXPECT_TRUE(c->snapshot().busy());
    EXPECT_FALSE(c->snapshot().can_override_sync());
    rig->low = true;
    c->stop();
    ASSERT_TRUE(until([&] { return !c->snapshot().trigger_stop_pending && c->snapshot().board_ready; }));
    EXPECT_FALSE(rig->wave_active);
}
TEST(MultiCameraRuntime, ContinuousWaveIsStoppedOnDisconnectDestructionAndPageDeparture) {
    QTemporaryDir dir;
    {
        auto rig = std::make_shared<FakeRig>();
        auto c = controller(rig);
        connect_and_sync(*c, configuration(dir.path()));
        c->start_trigger_wave();
        ASSERT_TRUE(until([&] { return rig->wave_active.load(); }));
        c->disconnect_rig();
        ASSERT_TRUE(until([&] { return !rig->wave_active.load() && !c->snapshot().all_connected(); }));
        EXPECT_FALSE(c->snapshot().can_override_sync());
    }
    {
        auto rig = std::make_shared<FakeRig>();
        auto c = controller(rig);
        connect_and_sync(*c, configuration(dir.path()));
        c->start_trigger_wave();
        ASSERT_TRUE(until([&] { return rig->wave_active.load(); }));
        c.reset();
        EXPECT_FALSE(rig->wave_active);
    }
    {
        auto rig = std::make_shared<FakeRig>();
        bool refreshed = false;
        auto c = controller(rig);
        QObject::connect(c.get(), &Controller::devices_changed, [&] { refreshed = true; });
        Workspace ui(nullptr, c.get());
        ASSERT_TRUE(until([&] { return refreshed; }));
        connect_and_sync(*c, configuration(dir.path()));
        auto *steps = ui.findChild<QListWidget *>("multiSteps");
        ASSERT_NE(steps, nullptr);
        steps->setCurrentRow(int(Step::Sync));
        c->start_trigger_wave();
        ASSERT_TRUE(
            until([&] { return c->snapshot().activity == Activity::Triggering && rig->wave_active.load(); }));
        QApplication::processEvents();
        steps->setCurrentRow(int(Step::Calibration));
        ASSERT_TRUE(
            until([&] { return !rig->wave_active.load() && c->snapshot().activity == Activity::Idle; }));
    }
}
TEST(MultiCameraRuntime, UnknownBurstLowAndThrowingCleanupCannotPass) {
    QTemporaryDir dir;
    auto rig = std::make_shared<FakeRig>();
    rig->burst_low = false;
    rig->cleanup_throws = true;
    auto c = controller(rig);
    c->configure(configuration(dir.path()));
    c->connect_rig();
    ASSERT_TRUE(until([&] { return c->snapshot().all_connected(); }));
    c->synchronize();
    ASSERT_TRUE(until([&] { return !c->snapshot().error.isEmpty(); }));
    EXPECT_EQ(c->snapshot().sync.verdict, Verdict::Failed);
    EXPECT_FALSE(c->snapshot().board_ready);
    EXPECT_EQ(rig->pulses.load(), 1);
}
TEST(MultiCameraRuntime, FullBoundedTriggerHistoryStillSelectsFormalEdges) {
    QTemporaryDir dir;
    auto rig = std::make_shared<FakeRig>();
    rig->bounded_history = true;
    auto c = controller(rig);
    connect_and_sync(*c, configuration(dir.path()));
    EXPECT_EQ(c->snapshot().sync.verdict, Verdict::Passed);
    EXPECT_EQ(c->snapshot().sync.matched[0], 6);
    EXPECT_EQ(c->snapshot().cameras[0].triggers.size(), 4096);
}
TEST(MultiCameraRuntime, DuplicateActiveSynchronizationDoesNotQueueMorePulses) {
    QTemporaryDir dir;
    auto rig = std::make_shared<FakeRig>();
    rig->hold_check = true;
    auto c = controller(rig);
    c->configure(configuration(dir.path()));
    c->connect_rig();
    ASSERT_TRUE(until([&] { return c->snapshot().all_connected(); }));
    c->synchronize();
    ASSERT_TRUE(until([&] { return rig->check_entered.load(); }));
    for (int i = 0; i < 100; ++i)
        c->synchronize();
    c->export_calibration(dir.filePath("barrier.json"));
    rig->hold_check = false;
    ASSERT_TRUE(until([&] { return QFileInfo::exists(dir.filePath("barrier.json")); }));
    EXPECT_EQ(c->snapshot().sync.verdict, Verdict::Passed);
    EXPECT_EQ(rig->checks.load(), 1);
    EXPECT_EQ(rig->pulses.load(), 4);
}
TEST(MultiCameraRuntime, StopDiscardsPendingOperationsAndNewRequestCannotClearActiveCancellation) {
    QTemporaryDir dir;
    auto rig = std::make_shared<FakeRig>();
    rig->hold_check = true;
    auto c = controller(rig);
    c->configure(configuration(dir.path()));
    c->connect_rig();
    ASSERT_TRUE(until([&] { return c->snapshot().all_connected(); }));
    c->check_board();
    ASSERT_TRUE(until([&] { return rig->check_entered.load(); }));
    c->synchronize();
    c->connect_rig();
    c->stop();
    c->synchronize();
    ASSERT_TRUE(until([&] { return rig->cancelled_checks.load() == 1; }));
    c->stop();
    rig->hold_check = false;
    c->export_calibration(dir.filePath("barrier.json"));
    ASSERT_TRUE(until([&] { return QFileInfo::exists(dir.filePath("barrier.json")); }));
    EXPECT_EQ(rig->pulses.load(), 0);
    EXPECT_EQ(rig->opens.load(), 3);
}
TEST(MultiCameraRuntime, ActiveTakeRejectsReconnectBoardCheckAndSynchronization) {
    QTemporaryDir dir;
    auto rig = std::make_shared<FakeRig>();
    auto c = controller(rig);
    auto config = configuration(dir.path());
    connect_and_sync(*c, config);
    const auto file = dir.filePath("profile.json");
    ASSERT_TRUE(write_json(file, profile(config)));
    c->import_calibration(file);
    ASSERT_TRUE(until([&] { return c->snapshot().calibration.verdict == Verdict::Passed; }));
    c->start_recording();
    ASSERT_TRUE(until([&] { return c->snapshot().activity == Activity::Recording; }));
    const auto epoch = c->snapshot().epoch;
    const auto checks = rig->checks.load();
    c->connect_rig();
    c->check_board();
    c->synchronize();
    c->open_take(dir.filePath("nonexistent"));
    c->export_calibration(dir.filePath("barrier.json"));
    ASSERT_TRUE(until([&] { return QFileInfo::exists(dir.filePath("barrier.json")); }));
    EXPECT_EQ(c->snapshot().activity, Activity::Recording);
    EXPECT_EQ(c->snapshot().epoch, epoch);
    EXPECT_EQ(rig->opens.load(), 6);
    EXPECT_EQ(rig->checks.load(), checks);
    EXPECT_EQ(rig->pulses.load(), 8);
    rig->time += 1000000;
    c->stop();
    ASSERT_TRUE(until([&] { return !c->snapshot().recording(); }));
}
TEST(MultiCameraRuntime, UnapprovedSerialIsRejectedWithoutOpeningAnyPort) {
    SerialMcuTransport mcu;
    std::atomic_bool cancelled{false};
    const auto result = mcu.check("different-usb-serial", cancelled, [] {});
    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.low_confirmed);
    EXPECT_TRUE(result.reason.contains("fixed approved"));
}
TEST(MultiCameraRuntime, MacSerialAliasesDoNotCreateTwoBoards) {
    const QString cu = "/dev/cu.usbmodem1401", tty = "/dev/tty.usbmodem1401";
    EXPECT_EQ(distinct_serial_port_paths({cu, tty, cu}), QStringList{cu});
    EXPECT_EQ(distinct_serial_port_paths({tty, cu}), QStringList{cu});
    EXPECT_EQ(distinct_serial_port_paths({tty}), QStringList{tty});
    EXPECT_EQ(distinct_serial_port_paths({cu, tty, "/dev/cu.usbmodem1501"}).size(), 2);
    EXPECT_EQ(distinct_serial_port_paths({"/dev/ttyACM0", "/dev/ttyACM1"}).size(), 2);
    EXPECT_TRUE(distinct_serial_port_paths({}).isEmpty());
}
TEST(MultiCameraRuntime, CancelledNativeCheckReturnsBeforeSerialEnumeration) {
    SerialMcuTransport mcu;
    std::atomic_bool cancelled{true};
    bool pumped = false;
    const auto result = mcu.check("36f9f437460d3ba8", cancelled, [&] { pumped = true; });
    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.low_confirmed);
    EXPECT_TRUE(result.reason.contains("cancelled"));
    EXPECT_FALSE(pumped);
}
TEST(MultiCameraRuntime, ThrowingMcuFactoryIsReportedOnWorkerInsteadOfTerminating) {
    auto rig = std::make_shared<FakeRig>();
    Controller c(
        nullptr, [rig] { return std::make_unique<FakeCamera>(rig); },
        []() -> std::unique_ptr<McuTransport> { throw std::runtime_error("injected MCU factory"); },
        [](const auto &camera) { return std::make_unique<FakeReader>(camera); }, rig->clock(),
        [] { return QVector<DeviceEntry>{}; });
    c.check_board();
    ASSERT_TRUE(until([&] { return c.snapshot().error.contains("injected MCU factory"); }));
    EXPECT_FALSE(c.snapshot().board_ready);
}
TEST(MultiCameraRuntime, DiscoveryCanBeInjectedWithoutHardwareEnumeration) {
    auto rig = std::make_shared<FakeRig>();
    std::atomic_int enumerated{0};
    Controller c(
        nullptr, [rig] { return std::make_unique<FakeCamera>(rig); },
        [rig] { return std::make_unique<FakeMcu>(rig); },
        [](const auto &camera) { return std::make_unique<FakeReader>(camera); }, rig->clock(),
        [&] {
            ++enumerated;
            return QVector<DeviceEntry>{{"fixture:silky:0", "0", "offline fixture"}};
        });
    QVector<DeviceEntry> found;
    QObject::connect(&c, &Controller::devices_changed, &c, [&](const auto &devices) { found = devices; });
    c.refresh();
    ASSERT_TRUE(until([&] { return !found.isEmpty(); }));
    EXPECT_EQ(enumerated.load(), 1);
    EXPECT_EQ(found.front().selection, "fixture:silky:0");
}
TEST(MultiCameraRuntime, CancellationDuringTakePreflightPreservesIncompleteTakeWithoutFormalCapture) {
    QTemporaryDir dir;
    auto rig = std::make_shared<FakeRig>();
    auto c = controller(rig);
    const auto config = configuration(dir.path());
    connect_and_sync(*c, config);
    ASSERT_TRUE(write_json(dir.filePath("profile.json"), profile(config)));
    c->import_calibration(dir.filePath("profile.json"));
    ASSERT_TRUE(until([&] { return c->snapshot().calibration.verdict == Verdict::Passed; }));
    rig->check_entered = false;
    rig->hold_check = true;
    c->start_recording();
    ASSERT_TRUE(until([&] { return rig->check_entered.load(); }));
    c->stop();
    ASSERT_TRUE(until([&] { return !c->snapshot().recording() && !c->snapshot().take_report.isEmpty(); }));
    const auto state = c->snapshot();
    EXPECT_EQ(rig->pulses.load(), 4);
    EXPECT_EQ(rig->opens.load(), 6);
    EXPECT_EQ(state.formal_start_us, -1);
    EXPECT_EQ(state.take_manifest.value("state").toString(), "incomplete");
    EXPECT_FALSE(state.board_ready);
    for (int i = 1; i <= 3; ++i)
        EXPECT_TRUE(QFileInfo::exists(QDir(state.take_directory).filePath(QString("camera-%1.raw").arg(i))));
}
TEST(MultiCameraRuntime, UnknownWatermarkCannotStartFormalCapture) {
    QTemporaryDir dir;
    auto rig = std::make_shared<FakeRig>();
    auto c = controller(rig);
    const auto config = configuration(dir.path());
    connect_and_sync(*c, config);
    ASSERT_TRUE(write_json(dir.filePath("profile.json"), profile(config)));
    c->import_calibration(dir.filePath("profile.json"));
    ASSERT_TRUE(until([&] { return c->snapshot().calibration.verdict == Verdict::Passed; }));
    rig->invalid_watermark = true;
    c->start_recording();
    ASSERT_TRUE(until([&] { return !c->snapshot().take_report.isEmpty(); }));
    EXPECT_EQ(c->snapshot().formal_start_us, -1);
    EXPECT_EQ(c->snapshot().take_manifest.value("state").toString(), "incomplete");
    EXPECT_TRUE(c->snapshot().error.contains("formal capture boundary"));
}
TEST(MultiCameraRuntime, FailedVerificationOrReportWriteCannotLeaveACompletedManifest) {
    class Reader final : public FakeReader {
      public:
        Reader(const TakeCamera &camera, bool mismatch) : FakeReader(camera), mismatch_(mismatch) {}
        CameraSnapshot scan(const std::function<bool()> &cancelled) override {
            auto result = FakeReader::scan(cancelled);
            if (mismatch_)
                ++result.event_count;
            return result;
        }

      private:
        bool mismatch_;
    };
    for (const bool report_write_failure : {false, true}) {
        QTemporaryDir dir;
        auto rig = std::make_shared<FakeRig>();
        auto c = controller(rig, [report_write_failure](const TakeCamera &camera) {
            if (report_write_failure)
                QDir().mkdir(QFileInfo(camera.path).dir().filePath("report.json"));
            return std::make_unique<Reader>(camera, !report_write_failure);
        });
        const auto config = configuration(dir.path());
        connect_and_sync(*c, config);
        ASSERT_TRUE(write_json(dir.filePath("profile.json"), profile(config)));
        c->import_calibration(dir.filePath("profile.json"));
        ASSERT_TRUE(until([&] { return c->snapshot().calibration.verdict == Verdict::Passed; }));
        c->start_recording();
        ASSERT_TRUE(until([&] { return c->snapshot().activity == Activity::Recording; }));
        rig->time += 1000000;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        c->stop();
        ASSERT_TRUE(
            until([&] { return !c->snapshot().recording() && !c->snapshot().take_report.isEmpty(); }));
        const auto state = c->snapshot();
        EXPECT_EQ(state.take_manifest.value("state").toString(), "incomplete");
        EXPECT_EQ(state.take_verdict, Verdict::Failed);
        EXPECT_EQ(read_json(QDir(state.take_directory).filePath("session.json")).value("state").toString(),
                  "incomplete");
        EXPECT_FALSE(state.error.isEmpty());
    }
}
TEST(MultiCameraRuntime, MissingLegacyMetadataIsUnknownNotFailedAndCancellationStopsBeforeOpening) {
    Take take;
    TakeCamera camera;
    camera.serial = "0";
    take.cameras.push_back(camera);
    int opened = 0;
    ReaderFactory factory = [&](const TakeCamera &item) {
        ++opened;
        return std::make_unique<FakeReader>(item);
    };
    auto report = verify_take(take, factory, [] { return false; });
    EXPECT_EQ(report.value("verdict").toString(), "incomplete");
    EXPECT_FALSE(report.value("countsAndProtocolPass").toBool());
    EXPECT_EQ(report.value("cameras").toArray()[0].toObject().value("countsAndBoundsMatch").toString(),
              "unknown");
    opened = 0;
    report = verify_take(take, factory, [] { return true; });
    EXPECT_EQ(opened, 0);
    EXPECT_TRUE(report.value("cancelled").toBool());
    EXPECT_EQ(report.value("verdict").toString(), "incomplete");
}
TEST(MultiCameraRuntime, PartialLegacyBoundsAndNullCountsStayUnknown) {
    QTemporaryDir dir;
    ASSERT_TRUE(write_playback_take(dir.path()));
    auto manifest = read_json(dir.filePath("session.json"));
    auto rows = manifest.value("cameras").toArray();
    auto first = rows[0].toObject();
    first.insert("eventCount", QJsonValue::Null);
    rows[0] = first;
    auto second = rows[1].toObject();
    second.remove("firstTimestampUs");
    rows[1] = second;
    auto third = rows[2].toObject();
    third.remove("lastTimestampUs");
    rows[2] = third;
    manifest.insert("cameras", rows);
    ASSERT_TRUE(write_json(dir.filePath("session.json"), manifest));
    const auto take = read_take(dir.path());
    EXPECT_FALSE(take.cameras[0].count_recorded);
    EXPECT_EQ(take.cameras[1].first, -1);
    EXPECT_EQ(take.cameras[2].last, -1);
    EXPECT_EQ(take.timeline.formal_start_us, -1);
    const auto report = verify_take(
        take, [](const auto &camera) { return std::make_unique<FakeReader>(camera); }, [] { return false; });
    EXPECT_EQ(report.value("verdict").toString(), "incomplete");
    first.insert("eventCount", -2);
    rows[0] = first;
    manifest.insert("cameras", rows);
    ASSERT_TRUE(write_json(dir.filePath("session.json"), manifest));
    EXPECT_THROW(read_take(dir.path()), std::runtime_error);
}
TEST(MultiCameraRuntime, SupersededSeekIsAtomicAndLatestSurvivesQueueFlood) {
    struct Gate {
        std::atomic_bool entered{false};
    };
    class Reader final : public FakeReader {
      public:
        Reader(TakeCamera camera, std::shared_ptr<Gate> gate)
            : FakeReader(camera), second_(camera.serial == "1"), gate_(std::move(gate)) {}
        CameraSnapshot frame(qint64 start, qint64 target, const std::function<bool()> &cancelled) override {
            if (second_ && target == 2000000) {
                gate_->entered = true;
                while (!cancelled())
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            return FakeReader::frame(start, target, cancelled);
        }

      private:
        bool second_;
        std::shared_ptr<Gate> gate_;
    };
    QTemporaryDir dir;
    ASSERT_TRUE(write_playback_take(dir.path()));
    auto gate = std::make_shared<Gate>();
    auto c = controller(std::make_shared<FakeRig>(),
                        [gate](const auto &camera) { return std::make_unique<Reader>(camera, gate); });
    bool mixed = false;
    QObject::connect(c.get(), &Controller::changed, c.get(), [&](const auto &s) {
        if (s.playback.loaded)
            for (const auto &camera : s.cameras)
                mixed = mixed || camera.latest_timestamp_us != s.playback.position_us;
    });
    c->open_take(dir.path());
    ASSERT_TRUE(until([&] { return c->snapshot().playback.loaded; }));
    c->seek(2000000);
    ASSERT_TRUE(until([&] { return gate->entered.load(); }));
    for (int i = 0; i < 128; ++i)
        c->seek(3000000 + i);
    ASSERT_TRUE(until([&] { return c->snapshot().playback.position_us == 3000127; }));
    EXPECT_FALSE(mixed);
    EXPECT_TRUE(c->snapshot().error.isEmpty());
    for (const auto &camera : c->snapshot().cameras)
        EXPECT_EQ(camera.latest_timestamp_us, 3000127);
}
TEST(MultiCameraRuntime, OpeningLegacyTakeClearsPreviousReportAndStopPausesPlayback) {
    QTemporaryDir first, second;
    ASSERT_TRUE(write_playback_take(first.path()));
    ASSERT_TRUE(write_playback_take(second.path()));
    ASSERT_TRUE(write_json(first.filePath("report.json"), {{"verdict", "failed"}}));
    auto c = controller(std::make_shared<FakeRig>());
    c->open_take(first.path());
    ASSERT_TRUE(until([&] { return !c->snapshot().take_report.isEmpty(); }));
    c->open_take(second.path());
    ASSERT_TRUE(until([&] { return c->snapshot().take_directory == second.path(); }));
    EXPECT_TRUE(c->snapshot().take_report.isEmpty());
    EXPECT_EQ(c->snapshot().calibration.verdict, Verdict::Unknown);
    c->play(true);
    ASSERT_TRUE(until([&] { return c->snapshot().playback.playing; }));
    c->stop();
    ASSERT_TRUE(until([&] { return !c->snapshot().playback.playing; }));
    EXPECT_EQ(c->snapshot().activity, Activity::Playback);
}
TEST(MultiCameraRuntime, ThreeCameraOfflineTakeCannotEnterLiveSynchronization) {
    QTemporaryDir dir;
    ASSERT_TRUE(write_playback_take(dir.path()));
    auto rig = std::make_shared<FakeRig>();
    auto c = controller(rig);
    c->open_take(dir.path());
    ASSERT_TRUE(until([&] { return c->snapshot().playback.loaded; }));
    EXPECT_FALSE(c->snapshot().all_connected());
    c->check_board();
    ASSERT_TRUE(until([&] { return c->snapshot().board_ready; }));
    c->synchronize();
    ASSERT_TRUE(until([&] { return c->snapshot().error.contains("live cameras"); }));
    EXPECT_TRUE(c->snapshot().playback.loaded);
    EXPECT_EQ(rig->pulses.load(), 0);
    EXPECT_EQ(rig->opens.load(), 0);
}
TEST(MultiCameraRuntime, NativeOfflineReplayRetainsTimestampsAndCanWidenTheSameFrameReadOnly) {
    QTemporaryDir dir;
    const auto path = dir.filePath("synthetic.raw");
    ASSERT_TRUE(write_evt2(path));
    QFile raw(path);
    ASSERT_TRUE(raw.open(QIODevice::ReadOnly));
    const auto before = QCryptographicHash::hash(raw.readAll(), QCryptographicHash::Sha256);
    raw.close();
    ASSERT_TRUE(QFile::setPermissions(path, QFileDevice::ReadOwner));
    TakeCamera camera;
    camera.path = path;
    camera.serial = "offline";
    camera.first = 1000000;
    camera.last = 1100000;
    camera.count = 3;
    camera.count_recorded = true;
    const auto modified = QFileInfo(path).lastModified();
    const auto entries = QDir(dir.path()).entryList(QDir::Files);
    {
        auto reader = make_raw_reader(camera);
        const auto narrow = reader->frame(1050000, 1100000, [] { return false; });
        const auto wide = reader->frame(1000000, 1100000, [] { return false; });
        EXPECT_EQ(wide.first_timestamp_us, 1000000);
        EXPECT_EQ(wide.latest_timestamp_us, 1100000);
        EXPECT_FALSE(wide.connected);
        EXPECT_NE(narrow.preview.pixel(100, 15), wide.preview.pixel(100, 15));
        const auto back = reader->frame(1000000, 1000000, [] { return false; });
        EXPECT_EQ(back.latest_timestamp_us, 1000000);
        EXPECT_EQ(back.event_count, 1);
    }
    Take take;
    take.cameras.push_back(camera);
    const auto report = verify_take(take, {}, [] { return false; });
    EXPECT_FALSE(report.value("countsAndProtocolPass").toBool());
    EXPECT_EQ(report.value("verdict").toString(), "incomplete");
    EXPECT_TRUE(report.value("cameras").toArray()[0].toObject().value("countsAndBoundsMatch").toBool());
    EXPECT_FALSE(
        report.value("cameras").toArray()[0].toObject().value("protocolValidationAvailable").toBool());
    ASSERT_TRUE(raw.open(QIODevice::ReadOnly));
    EXPECT_EQ(QCryptographicHash::hash(raw.readAll(), QCryptographicHash::Sha256), before);
    EXPECT_EQ(QFileInfo(path).lastModified(), modified);
    EXPECT_EQ(QDir(dir.path()).entryList(QDir::Files), entries);
}
TEST(MultiCameraRuntime, NativeOfflineVerificationRejectsOutOfBoundsEvents) {
    QTemporaryDir dir;
    TakeCamera camera;
    camera.path = dir.filePath("invalid.raw");
    ASSERT_TRUE(write_evt2(camera.path, true));
    camera.serial = "offline";
    camera.first = 1000000;
    camera.last = 1100000;
    camera.count = 3;
    camera.count_recorded = true;
    Take take;
    take.cameras.push_back(camera);
    const auto report = verify_take(take, {}, [] { return false; });
    EXPECT_FALSE(report.value("countsAndProtocolPass").toBool());
    EXPECT_EQ(report.value("verdict").toString(), "failed");
    EXPECT_EQ(report.value("cameras").toArray()[0].toObject().value("protocolErrors").toInteger(), 3);
}
TEST(MultiCameraRuntime, ImportedProfileChecksResolutionAndPerViewThreshold) {
    auto c = configuration("output");
    auto json = profile(c);
    std::array<cv::Size, 3> sizes{{{1280, 720}, {1280, 720}, {1280, 720}}};
    EXPECT_TRUE(read_calibration(json, c, sizes).validated);
    sizes[2] = {640, 480};
    EXPECT_THROW(read_calibration(json, c, sizes), std::runtime_error);
    sizes[2] = {1280, 720};
    auto views = json.value("validationViews").toArray();
    views[0] = QJsonArray{.1, .1, 2.};
    json.insert("validationViews", views);
    EXPECT_THROW(read_calibration(json, c, sizes), std::runtime_error);
}
TEST(MultiCameraRuntime, PreciseSeekDoesNotStartPlayback) {
    QTemporaryDir dir;
    QFile raw(dir.filePath("camera.raw"));
    ASSERT_TRUE(raw.open(QIODevice::WriteOnly));
    raw.write("raw");
    raw.close();
    QJsonObject camera{{"serial", "0"},
                       {"rawFile", "camera.raw"},
                       {"eventCount", 10},
                       {"firstTimestampUs", qint64(1000000)},
                       {"lastTimestampUs", qint64(20000000)}};
    write_json(dir.filePath("session.json"), {{"state", "completed"}, {"cameras", QJsonArray{camera}}});
    auto rig = std::make_shared<FakeRig>();
    auto c = controller(rig);
    Workspace ui(nullptr, c.get());
    c->open_take(dir.path());
    ASSERT_TRUE(until([&] { return c->snapshot().playback.loaded; }));
    ui.findChild<QListWidget *>("multiSteps")->setCurrentRow(4);
    auto *jump = ui.findChild<QLineEdit *>("multiJump");
    ASSERT_NE(jump, nullptr);
    jump->setText("5.123456");
    QKeyEvent key(QEvent::KeyPress, Qt::Key_Return, Qt::NoModifier);
    QApplication::sendEvent(jump, &key);
    ASSERT_TRUE(until([&] { return c->snapshot().playback.position_us == 5123456; }));
    EXPECT_FALSE(c->snapshot().playback.playing);
}
TEST(MultiCameraRuntime, NativeReplayOfExistingTriCaptureTakeIsReadOnly) {
    const auto path = qEnvironmentVariable("EBPLUS_TEST_TAKE");
    if (path.isEmpty())
        GTEST_SKIP() << "Set EBPLUS_TEST_TAKE to validate existing RAW files.";
    const auto take = read_take(path);
    QVector<QDateTime> modified;
    for (const auto &camera : take.cameras)
        modified.push_back(QFileInfo(camera.path).lastModified());
    auto report = verify_take(take, {}, [] { return false; });
    EXPECT_TRUE(report.value("countsAndProtocolPass").toBool()) << QJsonDocument(report).toJson().constData();
    for (int i = 0; i < take.cameras.size(); ++i) {
        auto reader = make_raw_reader(take.cameras[i]);
        auto target = take.timeline.first_us + (take.timeline.last_us - take.timeline.first_us) / 2;
        auto first = reader->frame(target - 100000, target, [] { return false; });
        EXPECT_FALSE(first.preview.isNull());
        auto back =
            reader->frame(take.timeline.first_us, take.timeline.first_us + 100000, [] { return false; });
        EXPECT_LE(back.latest_timestamp_us, take.timeline.first_us + 100000);
        EXPECT_EQ(QFileInfo(take.cameras[i].path).lastModified(), modified[i]);
    }
}
} // namespace
