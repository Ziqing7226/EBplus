#pragma once
#include "calibration.h"
#include "mcu_transport.h"
#include "take.h"
#include <QObject>
#include <chrono>
#include <thread>

namespace gui::multicam {
struct RuntimeClock {
    std::function<std::chrono::steady_clock::time_point()> now = [] {
        return std::chrono::steady_clock::now();
    };
    std::function<void(std::chrono::milliseconds)> sleep = [](auto duration) {
        std::this_thread::sleep_for(duration);
    };
};
class Controller final : public QObject {
    Q_OBJECT
  public:
    using Detector = std::function<DetectedGroup(const std::array<CountWindow, 3> &)>;
    explicit Controller(QObject *parent = nullptr, CameraFactory cameras = {},
                        std::function<std::unique_ptr<McuTransport>()> mcu = {}, ReaderFactory readers = {},
                        RuntimeClock clock = {}, std::function<QVector<DeviceEntry>()> discovery = {},
                        Detector detector = {});
    ~Controller() override;
    void configure(const RigConfig &);
    void refresh();
    void connect_rig();
    void disconnect_rig();
    void check_board();
    void synchronize();
    void start_trigger_wave();
    void override_sync(const QString &reason, quint64 expected_epoch, quint64 expected_attempt);
    void calibrate(bool automatic, int supplemental = -1);
    void capture_pose();
    void remove_pose(int index = -1);
    void reset_calibration();
    void solve();
    void validate();
    void import_calibration(const QString &);
    void export_calibration(const QString &);
    void import_observations(const QString &);
    void start_recording();
    void stop();
    void start_sop(bool reuse);
    void pause_sop();
    void open_take(const QString &);
    void seek(qint64 absolute_us);
    void play(bool);
    void speed(double);
    void include_preflight(bool);
    WorkflowSnapshot snapshot() const;

    QVector<RigProfileRecord> list_profiles() const;
    QVector<RigProfileTakeItem> list_legacy_takes() const;
    bool activate_profile(const QString &profile_id);
    bool save_calibrated_profile(const QString &profile_name, RigLayoutPreset preset, QString *out_profile_id = nullptr);
    bool import_profile(const QString &path, QString *out_profile_id = nullptr);
    bool delete_profile(const QString &profile_id);

  signals:
    void changed(const gui::multicam::WorkflowSnapshot &);
    void devices_changed(const QVector<gui::multicam::DeviceEntry> &);
    void profiles_changed();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace gui::multicam
