#pragma once

#include <QImage>
#include <QJsonObject>
#include <QMetaType>
#include <QPointF>
#include <QRectF>
#include <QString>
#include <QStringList>
#include <QVector>
#include <QVector3D>
#include <array>
#include <limits>
#include <opencv2/core.hpp>

namespace gui::multicam {

constexpr int camera_count = 3;
constexpr qint64 calibration_window_us = 100000;

enum class Step { Devices, Sync, Calibration, Capture, Review };
enum class WorkspaceModule { Capture = 0, Calibrate = 1, Library = 2 };
enum class RigLayoutPreset {
    LayoutA_Collinear = 0,
    LayoutB_LShape = 1,
    LayoutC_LowTriangle = 2
};
enum class BaselineSanityVerdict { Unknown = -1, Pass = 0, Warning = 1, Alert = 2 };
enum class Activity {
    Idle,
    Connecting,
    Syncing,
    Triggering,
    Calibrating,
    Solving,
    Preparing,
    Recording,
    Stopping,
    Verifying,
    Playback
};
enum class Verdict { Unknown, Running, Passed, Failed, Incomplete };

struct NominalSlotGeometry {
    QString role_name;
    QString hole_id;
    QVector3D position_mm{0.0f, 0.0f, 0.0f};
};

struct BaselineSanityCheck {
    BaselineSanityVerdict verdict = BaselineSanityVerdict::Unknown;
    std::array<double, camera_count> nominal_mm{100.0, 50.0, 150.0};
    std::array<double, camera_count> measured_mm{0.0, 0.0, 0.0};
    std::array<double, camera_count> abs_errors_mm{0.0, 0.0, 0.0};
    double max_abs_error_mm = 0.0;
    QString summary;
};

struct CameraBinding {
    QString selection;
    QString serial;
    QString position;
};

struct CameraTuning {
    bool auto_bias = true;
    double rate_min_mev = 18.0;
    double rate_max_mev = 22.0;
    int diff_offset = 0;
    int diff_on_offset = 0;
    int diff_off_offset = 0;
    int fo_offset = 0;
    int hpf_offset = 0;
    int refr_offset = 0;
    bool stc_enabled = false;
    int stc_type = 1; // 0 = STC_CUT_TRAIL, 1 = STC_KEEP_TRAIL, 2 = TRAIL
    int stc_threshold_us = 1000;
    bool afk_enabled = false;
    int afk_low_hz = 90;
    int afk_high_hz = 110;
    bool erc_enabled = false;
    int erc_rate_mev = 20;
    bool operator==(const CameraTuning &other) const {
        return auto_bias == other.auto_bias && rate_min_mev == other.rate_min_mev &&
               rate_max_mev == other.rate_max_mev && diff_offset == other.diff_offset &&
               diff_on_offset == other.diff_on_offset && diff_off_offset == other.diff_off_offset &&
               fo_offset == other.fo_offset && hpf_offset == other.hpf_offset &&
               refr_offset == other.refr_offset && stc_enabled == other.stc_enabled &&
               stc_type == other.stc_type && stc_threshold_us == other.stc_threshold_us &&
               afk_enabled == other.afk_enabled && afk_low_hz == other.afk_low_hz &&
               afk_high_hz == other.afk_high_hz && erc_enabled == other.erc_enabled &&
               erc_rate_mev == other.erc_rate_mev;
    }
    bool operator!=(const CameraTuning &other) const { return !(*this == other); }
};

struct RigConfig {
    QString project = QStringLiteral("Tri-rig");
    QString active_profile_id;
    RigLayoutPreset layout_preset = RigLayoutPreset::LayoutA_Collinear;
    QString take_label;
    std::array<CameraBinding, camera_count> cameras;
    std::array<CameraTuning, camera_count> tuning;
    int master = 1;
    int reference = 1;
    bool roles_confirmed = false;
    bool trigger_wiring_confirmed = false;
    bool optics_confirmed = false;
    double max_delta_us = std::numeric_limits<double>::quiet_NaN();
    double max_rms_px = std::numeric_limits<double>::quiet_NaN();
    QString output;
    double square_mm = 0;
    int training_target = 50;
    int validation_target = 10;
    int calibration_capture_seconds = 5;
    bool timed = false;
    int duration_seconds = 60;
    bool discard_incomplete_takes = false;
    QString note;
    QString board_serial = QStringLiteral("36f9f437460d3ba8");
    bool board_locked = false;
    QString board_display;
    bool board_fullscreen = false;
    int board_square_px = 0;
    QSize board_logical_size;
    double board_device_pixel_ratio = 0;
    QString board_measured_utc;

    RigConfig();
    QJsonObject to_json() const;
    static RigConfig from_json(const QJsonObject &object);
    QString geometry_key() const;
};

struct TriggerEdge {
    qint64 timestamp_us = 0;
    int polarity = 0;
    int channel = 0;
};

struct SyncEvidence {
    Verdict verdict = Verdict::Unknown;
    quint64 epoch = 0;
    quint64 attempt = 0;
    double max_delta_us = 0;
    std::array<int, camera_count> matched{};
    std::array<int, camera_count> extras{};
    std::array<std::array<qint64, 6>, camera_count> delta_us{};
    QString reason;
    // True when the cameras report the GP2 rising edge as polarity 0 (inverting Trigger In front-end).
    bool polarity_inverted = false;
    bool attempt_completed = false;
    bool operator_accepted = false;
    QString operator_reason;
    QString operator_accepted_utc;
    std::array<QVector<TriggerEdge>, camera_count> observed_edges;
    QJsonObject controller_evidence;
    bool timing_measured() const;
    bool accepted(quint64 current_epoch, double tolerance_us) const;
    QJsonObject to_json() const;
};

struct CameraSnapshot {
    CameraBinding binding;
    bool connected = false;
    int width = 0;
    int height = 0;
    quint64 event_count = 0;
    quint64 bytes_consumed = 0;
    quint64 regressions = 0;
    quint64 protocol_errors = 0;
    qint64 first_timestamp_us = -1;
    qint64 latest_timestamp_us = -1;
    double event_rate = 0;
    double byte_rate = 0;
    double decode_realtime_ratio = std::numeric_limits<double>::quiet_NaN();
    qint64 decode_age_ms = -1;
    QImage preview;
    QVector<TriggerEdge> triggers;
    QVector<QPointF> corners;
    std::array<int, 9> coverage{};
    double stable_progress = 0;
    QString detail;
    QJsonObject metadata;
};

struct CalibrationStatus {
    QStringList observation_labels;
    Verdict verdict = Verdict::Unknown;
    bool collecting = false;
    qint64 capture_remaining_ms = 0;
    qint64 capture_duration_ms = 5000;
    bool capture_expired = false;
    bool training_complete = false;
    bool board_locked = false;
    std::array<int, camera_count> intrinsic_counts{};
    int common_count = 0;
    int validation_count = 0;
    double rms = std::numeric_limits<double>::quiet_NaN();
    QString geometry_key;
    QString guide = QStringLiteral("Open and measure the calibration board.");
    QString reason;
    int target_camera = 1;
    double stable_progress = 0;
    QRectF target_region;
    std::array<std::array<int, 9>, 3> coverage{};
    std::array<QVector<QPointF>, 3> validation_projected;
    std::array<QVector<QPointF>, 3> validation_observed;
    QJsonObject profile;
};

struct PlaybackStatus {
    bool loaded = false;
    bool playing = false;
    bool include_preflight = false;
    double speed = 1;
    qint64 position_us = 0;
    qint64 first_us = 0;
    qint64 last_us = 0;
    qint64 formal_start_us = -1;
    qint64 formal_end_us = -1;
    std::array<QVector<QPointF>, camera_count> event_rate_series;
};

struct RigProfileTakeItem {
    QString take_id;
    QString take_dir;
    QString label;
    QString created_at;
    double duration_s = 0.0;
    Verdict verdict = Verdict::Unknown;
    double max_delta_us = 0.0;
    bool is_legacy = false;
    QString profile_id;
};

struct RigProfileRecord {
    QString profile_id;
    QString display_name;
    QString created_at;
    RigLayoutPreset layout_preset = RigLayoutPreset::LayoutA_Collinear;
    std::array<QString, camera_count> serials;
    std::array<NominalSlotGeometry, camera_count> nominal_slots;
    std::array<double, camera_count> nominal_baselines_mm{100.0, 50.0, 150.0};
    std::array<double, camera_count> measured_baselines_mm{0.0, 0.0, 0.0};
    std::array<double, camera_count> focal_px{0.0, 0.0, 0.0};
    double rms_px = 0.0;
    double max_sync_delta_us = 0.0;
    QJsonObject calibration_json;
    QString directory_path;
    QVector<RigProfileTakeItem> takes;

    bool valid() const { return !profile_id.trimmed().isEmpty(); }
    BaselineSanityCheck sanity_check() const;
};

struct WorkflowSnapshot {
    Activity activity = Activity::Idle;
    std::array<CameraSnapshot, camera_count> cameras;
    quint64 epoch = 0;
    bool board_ready = false;
    bool trigger_stop_pending = false;
    QString board_detail;
    SyncEvidence sync;
    CalibrationStatus calibration;
    QString take_directory;
    QJsonObject take_manifest;
    QJsonObject take_report;
    Verdict take_verdict = Verdict::Unknown;
    qint64 elapsed_ms = 0;
    qint64 formal_start_us = -1;
    bool sop_running = false;
    bool sop_paused = false;
    bool sop_reuse = false;
    QString message;
    QString error;
    PlaybackStatus playback;

    bool all_connected() const;
    bool busy() const;
    bool recording() const;
    bool can_override_sync() const;
};

enum class BlockerKind { InvalidField, Action, Running };

struct Blocker {
    Step step;
    QString field;
    QString message;
    BlockerKind kind = BlockerKind::Action;
};

struct GuidanceContext {
    bool secondary_display_selected = false;
    bool board_open = false;
};

struct StageGate {
    QVector<Blocker> pending;
    bool complete = false;
    QString status;
    QString next_action;
    bool can_advance() const { return pending.isEmpty(); }
};

QVector<Blocker> configuration_blockers(const RigConfig &config);
QVector<Blocker> blockers(Step step, const RigConfig &config, const WorkflowSnapshot &state);
StageGate stage_gate(Step step, const RigConfig &, const WorkflowSnapshot &, const GuidanceContext & = {});
QString blocker_kind_text(BlockerKind);
QString verdict_text(Verdict verdict);
QString step_text(Step step);

std::array<NominalSlotGeometry, camera_count> nominal_layout_geometry(RigLayoutPreset preset);
std::array<double, camera_count> nominal_baselines_mm(RigLayoutPreset preset);
BaselineSanityCheck evaluate_baseline_sanity(RigLayoutPreset preset, const QJsonObject &calibration_profile);
BaselineSanityCheck evaluate_baseline_sanity(
    const std::array<double, camera_count> &nominal_mm,
    const std::array<double, camera_count> &measured_mm);
QString layout_preset_code(RigLayoutPreset preset);
RigLayoutPreset layout_preset_from_code(const QString &code);
QString layout_preset_title(RigLayoutPreset preset);
QString sanitize_profile_id(const QString &raw_name);
bool save_profile_to_disk(const QString &root_output_dir, const RigProfileRecord &record, QString *out_dir = nullptr, QString *error = nullptr);
RigProfileRecord load_profile_from_disk(const QString &profile_dir);
QVector<RigProfileRecord> scan_profiles_on_disk(const QString &root_output_dir);
QVector<RigProfileTakeItem> scan_legacy_takes_on_disk(const QString &root_output_dir);
bool delete_profile_from_disk(const QString &root_output_dir, const QString &profile_id, QString *error = nullptr);

} // namespace gui::multicam

Q_DECLARE_METATYPE(gui::multicam::WorkflowSnapshot)
Q_DECLARE_METATYPE(gui::multicam::RigConfig)

