#include "types.h"

#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QRegularExpression>
#include <QSet>
#include <algorithm>
#include <cmath>

namespace gui::multicam {
namespace {
QString verdict_code(Verdict v) {
    switch (v) {
    case Verdict::Passed:
        return QStringLiteral("passed");
    case Verdict::Failed:
        return QStringLiteral("failed");
    case Verdict::Incomplete:
        return QStringLiteral("incomplete");
    case Verdict::Running:
        return QStringLiteral("running");
    default:
        return QStringLiteral("unknown");
    }
}
} // namespace

RigConfig::RigConfig() {
    cameras[0].position = QStringLiteral("Left");
    cameras[1].position = QStringLiteral("Center");
    cameras[2].position = QStringLiteral("Right");
    output = QDir::homePath() + QStringLiteral("/TriEventCaptures");
}

QJsonObject RigConfig::to_json() const {
    QJsonArray bindings, tunings;
    for (int i = 0; i < camera_count; ++i) {
        const auto &c = cameras[i];
        bindings.append(
            QJsonObject{{"selection", c.selection}, {"serial", c.serial}, {"position", c.position}});
        const auto &t = tuning[i];
        tunings.append(QJsonObject{{"autoBias", t.auto_bias},
                                   {"rateMinMev", t.rate_min_mev},
                                   {"rateMaxMev", t.rate_max_mev},
                                   {"diffOffset", t.diff_offset},
                                   {"diffOnOffset", t.diff_on_offset},
                                   {"diffOffOffset", t.diff_off_offset},
                                   {"foOffset", t.fo_offset},
                                   {"hpfOffset", t.hpf_offset},
                                   {"refrOffset", t.refr_offset},
                                   {"stcEnabled", t.stc_enabled},
                                   {"stcType", t.stc_type},
                                   {"stcThresholdUs", t.stc_threshold_us},
                                   {"afkEnabled", t.afk_enabled},
                                   {"afkLowHz", t.afk_low_hz},
                                   {"afkHighHz", t.afk_high_hz},
                                   {"ercEnabled", t.erc_enabled},
                                   {"ercRateMev", t.erc_rate_mev}});
    }
    QJsonObject o{{"schemaVersion", 1},
                  {"project", project},
                  {"activeProfileId", active_profile_id},
                  {"layoutPreset", layout_preset_code(layout_preset)},
                  {"takeLabel", take_label},
                  {"cameras", bindings},
                  {"cameraTuning", tunings},
                  {"master", master},
                  {"reference", reference},
                  {"output", output},
                  {"squareMm", square_mm},
                  {"trainingTarget", training_target},
                  {"validationTarget", validation_target},
                  {"calibrationCaptureSeconds", calibration_capture_seconds},
                  {"timed", timed},
                  {"durationSeconds", duration_seconds},
                  {"discardIncompleteTakes", discard_incomplete_takes},
                  {"note", note},
                  {"boardSerial", board_serial},
                  {"triggerDriveMa", 12},
                  {"triggerWiringConfirmed", trigger_wiring_confirmed},
                  {"boardDisplay", board_display},
                  {"boardFullscreen", board_fullscreen}};
    o.insert("boardMeasurement", QJsonObject{{"squareLogicalPx", board_square_px},
                                             {"width", board_logical_size.width()},
                                             {"height", board_logical_size.height()},
                                             {"devicePixelRatio", board_device_pixel_ratio},
                                             {"measuredUtc", board_measured_utc}});
    if (std::isfinite(max_delta_us))
        o.insert("maxDeltaUs", max_delta_us);
    if (std::isfinite(max_rms_px))
        o.insert("maxRmsPx", max_rms_px);
    return o;
}

RigConfig RigConfig::from_json(const QJsonObject &o) {
    RigConfig c;
    c.project = o.value("project").toString(c.project);
    c.active_profile_id = o.value("activeProfileId").toString();
    c.layout_preset = layout_preset_from_code(o.value("layoutPreset").toString());
    c.take_label = o.value("takeLabel").toString();
    const auto a = o.value("cameras").toArray();
    for (int i = 0; i < std::min(camera_count, static_cast<int>(a.size())); ++i) {
        const auto b = a[i].toObject();
        c.cameras[i].selection = b.value("selection").toString();
        c.cameras[i].serial = b.value("serial").toString();
    }
    c.master = o.value("master").toInt(1);
    c.reference = o.value("reference").toInt(1);
    c.output = o.value("output").toString(c.output);
    c.square_mm = o.value("squareMm").toDouble();
    c.max_delta_us = o.value("maxDeltaUs").toDouble(std::numeric_limits<double>::quiet_NaN());
    c.max_rms_px = o.value("maxRmsPx").toDouble(std::numeric_limits<double>::quiet_NaN());
    c.training_target = o.value("trainingTarget").toInt(50);
    c.validation_target = o.value("validationTarget").toInt(10);
    c.calibration_capture_seconds = o.value("calibrationCaptureSeconds").toInt(5);
    c.timed = o.value("timed").toBool();
    c.duration_seconds = o.value("durationSeconds").toInt(60);
    c.discard_incomplete_takes = o.value("discardIncompleteTakes").toBool(false);
    const auto ta = o.value("cameraTuning").toArray();
    for (int i = 0; i < std::min(camera_count, static_cast<int>(ta.size())); ++i) {
        const auto tobj = ta[i].toObject();
        auto &t = c.tuning[i];
        t.auto_bias = tobj.value("autoBias").toBool(t.auto_bias);
        t.rate_min_mev = tobj.value("rateMinMev").toDouble(t.rate_min_mev);
        t.rate_max_mev = tobj.value("rateMaxMev").toDouble(t.rate_max_mev);
        t.diff_offset = tobj.value("diffOffset").toInt(t.diff_offset);
        t.diff_on_offset = tobj.value("diffOnOffset").toInt(t.diff_on_offset);
        t.diff_off_offset = tobj.value("diffOffOffset").toInt(t.diff_off_offset);
        t.fo_offset = tobj.value("foOffset").toInt(t.fo_offset);
        t.hpf_offset = tobj.value("hpfOffset").toInt(t.hpf_offset);
        t.refr_offset = tobj.value("refrOffset").toInt(t.refr_offset);
        t.stc_enabled = tobj.value("stcEnabled").toBool(t.stc_enabled);
        t.stc_type = tobj.value("stcType").toInt(t.stc_type);
        t.stc_threshold_us = tobj.value("stcThresholdUs").toInt(t.stc_threshold_us);
        t.afk_enabled = tobj.value("afkEnabled").toBool(t.afk_enabled);
        t.afk_low_hz = tobj.value("afkLowHz").toInt(t.afk_low_hz);
        t.afk_high_hz = tobj.value("afkHighHz").toInt(t.afk_high_hz);
        t.erc_enabled = tobj.value("ercEnabled").toBool(t.erc_enabled);
        t.erc_rate_mev = tobj.value("ercRateMev").toInt(t.erc_rate_mev);
    }
    c.note = o.value("note").toString();
    c.board_serial = o.value("boardSerial").toString(c.board_serial);
    c.board_display = o.value("boardDisplay").toString();
    c.board_fullscreen = o.value("boardFullscreen").toBool();
    return c;
}

QString RigConfig::geometry_key() const {
    QJsonArray a;
    for (const auto &c : cameras)
        a.append(QJsonObject{{"serial", c.serial}, {"selection", c.selection}});
    const QJsonObject identity{{"serials", a}, {"reference", reference}, {"squareMm", square_mm}};
    return QString::fromLatin1(
        QCryptographicHash::hash(QJsonDocument(identity).toJson(QJsonDocument::Compact),
                                 QCryptographicHash::Sha256)
            .toHex());
}

bool SyncEvidence::accepted(quint64 current_epoch, double tolerance_us) const {
    if (epoch != current_epoch || !std::isfinite(tolerance_us) || tolerance_us < 0)
        return false;
    const bool manual = operator_accepted && attempt_completed && attempt > 0 &&
                        !operator_reason.trimmed().isEmpty() && !operator_accepted_utc.isEmpty();
    return manual || (verdict == Verdict::Passed && max_delta_us <= tolerance_us);
}

bool SyncEvidence::timing_measured() const {
    return verdict != Verdict::Running &&
           std::all_of(matched.begin(), matched.end(), [](int n) { return n == 6; });
}
QJsonObject SyncEvidence::to_json() const {
    const bool measured = timing_measured();
    QJsonArray counts, extra, offsets, observed;
    for (int i = 0; i < camera_count; ++i) {
        counts.append(matched[i]);
        extra.append(extras[i]);
        QJsonArray row;
        for (auto delta : delta_us[i])
            row.append(delta);
        offsets.append(row);
        QJsonArray records;
        for (const auto &edge : observed_edges[i])
            records.append(QJsonObject{
                {"timestampUs", edge.timestamp_us}, {"polarity", edge.polarity}, {"channel", edge.channel}});
        observed.append(records);
    }
    return {{"verdict", verdict_code(verdict)},
            {"epoch", static_cast<qint64>(epoch)},
            {"attempt", static_cast<qint64>(attempt)},
            {"attemptCompleted", attempt_completed},
            {"acceptanceSource", operator_accepted            ? "manual"
                                 : verdict == Verdict::Passed ? "automatic"
                                                              : "none"},
            {"manualOverride", QJsonObject{{"accepted", operator_accepted},
                                           {"reason", operator_reason},
                                           {"acceptedUtc", operator_accepted_utc}}},
            {"observedRecords", observed},
            {"controller", controller_evidence},
            {"timingMeasured", measured},
            {"maxDeltaUs", measured ? QJsonValue(max_delta_us) : QJsonValue(QJsonValue::Null)},
            {"matchedFormalEdges", counts},
            {"extraRecordsClassified", measured},
            {"extraRecords", measured ? QJsonValue(extra) : QJsonValue(QJsonValue::Null)},
            {"reason", reason},
            {"triggerPolarityInverted", measured ? QJsonValue(polarity_inverted) : QJsonValue(QJsonValue::Null)},
            {"edgeDeltaFromMasterUs", measured ? QJsonValue(offsets) : QJsonValue(QJsonValue::Null)}};
}

bool WorkflowSnapshot::all_connected() const {
    if (playback.loaded)
        return false;
    for (const auto &c : cameras)
        if (!c.connected)
            return false;
    return true;
}
bool WorkflowSnapshot::recording() const {
    return activity == Activity::Preparing || activity == Activity::Recording ||
           activity == Activity::Stopping || activity == Activity::Verifying ||
           (take_verdict == Verdict::Running && !take_directory.isEmpty());
}
bool WorkflowSnapshot::busy() const {
    return trigger_stop_pending || recording() || activity == Activity::Connecting ||
           activity == Activity::Syncing || activity == Activity::Triggering ||
           activity == Activity::Calibrating || activity == Activity::Solving;
}
bool WorkflowSnapshot::can_override_sync() const {
    const bool waiting_take = activity == Activity::Preparing && formal_start_us < 0;
    if ((!waiting_take && busy()) || playback.loaded || !all_connected() || !board_ready || epoch == 0 ||
        sync.attempt == 0 || sync.epoch != epoch || !sync.attempt_completed || sync.operator_accepted ||
        (sync.verdict != Verdict::Failed && sync.verdict != Verdict::Incomplete))
        return false;
    return std::all_of(sync.observed_edges.begin(), sync.observed_edges.end(), [](const auto &edges) {
        return std::count_if(edges.begin(), edges.end(), [](const auto &e) { return e.channel == 0; }) >= 2;
    });
}

QVector<Blocker> configuration_blockers(const RigConfig &c) {
    QVector<Blocker> b;
    QSet<QString> seen;
    for (int i = 0; i < camera_count; ++i) {
        const auto serial = c.cameras[i].serial.trimmed();
        if (serial.isEmpty() || seen.contains(serial))
            b.push_back({Step::Devices, QStringLiteral("camera%1").arg(i),
                         serial.isEmpty()
                             ? QStringLiteral("%1: select a camera serial.").arg(c.cameras[i].position)
                             : QStringLiteral("%1: serial %2 is already assigned; "
                                              "select another camera.")
                                   .arg(c.cameras[i].position, serial),
                         BlockerKind::InvalidField});
        seen.insert(serial);
    }
    if (c.master < 0 || c.master >= camera_count)
        b.push_back({Step::Devices, "master", "Select the clock Master.", BlockerKind::InvalidField});
    if (c.reference < 0 || c.reference >= camera_count)
        b.push_back(
            {Step::Devices, "reference", "Select the geometry reference.", BlockerKind::InvalidField});
    if (!c.roles_confirmed)
        b.push_back(
            {Step::Devices, "roles", "Confirm camera placement, clock roles and geometry reference."});
    if (c.project.trimmed().isEmpty())
        b.push_back({Step::Devices, "project", "Enter a project name.", BlockerKind::InvalidField});
    if (!std::isfinite(c.max_delta_us) || c.max_delta_us < 0)
        b.push_back({Step::Devices, "delta", "Enter a non-negative maximum time difference (us).",
                     BlockerKind::InvalidField});
    if (!std::isfinite(c.max_rms_px) || c.max_rms_px <= 0)
        b.push_back({Step::Devices, "rms", "Enter a positive maximum RMS (px).", BlockerKind::InvalidField});
    if (c.training_target < 3 || c.training_target > 500)
        b.push_back({Step::Calibration, "trainingTarget", "Set 3-500 common training poses.",
                     BlockerKind::InvalidField});
    if (c.validation_target < 3 || c.validation_target > 100)
        b.push_back({Step::Calibration, "validationTarget", "Set 3-100 independent validation poses.",
                     BlockerKind::InvalidField});
    if (c.calibration_capture_seconds < 3 || c.calibration_capture_seconds > 30)
        b.push_back({Step::Calibration, "captureDuration", "Set a 3-30 second calibration capture round.",
                     BlockerKind::InvalidField});
    return b;
}

QVector<Blocker> blockers(Step step, const RigConfig &c, const WorkflowSnapshot &s) {
    QVector<Blocker> b;
    if (s.trigger_stop_pending)
        b.push_back(
            {Step::Sync, "stopTrigger", "Retry Stop Trigger: LOW / drive restoration not confirmed."});
    if (step == Step::Review) {
        if (!s.playback.loaded)
            b.push_back({Step::Review, "openTake", "Open a completed or incomplete take."});
        return b;
    }
    b += configuration_blockers(c);
    for (int i = 0; i < camera_count; ++i)
        if (!s.cameras[i].connected || s.playback.loaded)
            b.push_back({Step::Devices, "connect",
                         QStringLiteral("%1 (%2): connect the configured camera.")
                             .arg(c.cameras[i].position, c.cameras[i].serial.isEmpty()
                                                             ? QStringLiteral("serial not selected")
                                                             : c.cameras[i].serial)});
    if (!s.board_ready)
        b.push_back({Step::Devices, "checkBoard",
                     s.board_detail.isEmpty() ? QStringLiteral("Check the bound RP2350 controller.")
                                              : s.board_detail});
    if (step == Step::Devices)
        return b;
    if (!c.trigger_wiring_confirmed)
        b.push_back(
            {Step::Sync, "triggerWiring", "Confirm GP2 Trigger wiring and per-camera current limiting."});
    if (!s.sync.accepted(s.epoch, c.max_delta_us))
        b.push_back({Step::Sync,
                     s.can_override_sync() && c.trigger_wiring_confirmed ? "overrideSync" : "runSync",
                     s.can_override_sync() && c.trigger_wiring_confirmed
                         ? QStringLiteral("Inspect the retained Trigger traces; accept "
                                          "manually with a reason, or rerun the finite check.")
                     : s.sync.attempt_completed && s.sync.epoch == s.epoch
                         ? QStringLiteral("Check wiring / current limiting, then "
                                          "retry the finite check; "
                                          "manual acceptance still requires "
                                          "per-camera traces and confirmed LOW.")
                         : QStringLiteral("Run a finite Trigger check for the "
                                          "current connection epoch.")});
    if (step == Step::Sync)
        return b;
    if (step == Step::Calibration) {
        if (!std::isfinite(c.square_mm) || c.square_mm <= 0)
            b.push_back({Step::Calibration, "square", "Measure and enter the displayed square edge (mm).",
                         BlockerKind::InvalidField});
        if (!c.board_locked)
            b.push_back({Step::Calibration, "lockBoard", "Lock the measured calibration display size."});
        return b;
    }
    if (s.calibration.verdict != Verdict::Passed || s.calibration.geometry_key != c.geometry_key() ||
        !std::isfinite(s.calibration.rms) || s.calibration.rms > c.max_rms_px || !c.optics_confirmed)
        b.push_back({Step::Calibration, "calibration",
                     "Approve the current rig calibration, then confirm unchanged "
                     "optics / rig."});
    if (c.output.trimmed().isEmpty())
        b.push_back(
            {Step::Capture, "output", "Enter the recording output directory.", BlockerKind::InvalidField});
    if (c.timed && (c.duration_seconds < 1 || c.duration_seconds > 7200))
        b.push_back({Step::Capture, "duration", "Recording duration must be 1-7200 seconds.",
                     BlockerKind::InvalidField});
    return b;
}

namespace {
bool calibration_approved(const RigConfig &c, const WorkflowSnapshot &s) {
    return s.calibration.verdict == Verdict::Passed && s.calibration.geometry_key == c.geometry_key() &&
           std::isfinite(s.calibration.rms) && std::isfinite(c.max_rms_px) &&
           s.calibration.rms <= c.max_rms_px;
}

Blocker calibration_action(const RigConfig &c, const WorkflowSnapshot &s, const GuidanceContext &context) {
    const auto &cal = s.calibration;
    if (calibration_approved(c, s))
        return {Step::Calibration, "calibration", "Confirm lens, focus and rigid assembly are unchanged."};
    if (!c.board_locked && !context.secondary_display_selected)
        return {Step::Calibration, "display", "Select a secondary display for the blinking board."};
    if (!c.board_locked && !context.board_open)
        return {Step::Calibration, "showBoard", "Open the blinking board on the selected secondary display."};
    if (!std::isfinite(c.square_mm) || c.square_mm <= 0)
        return {Step::Calibration, "square", "Measure a displayed square; enter its edge in mm.",
                BlockerKind::InvalidField};
    if (!c.board_locked)
        return {Step::Calibration, "lockBoard", "Lock the measured display size."};
    if (!cal.training_complete && cal.common_count < c.training_target)
        return {Step::Calibration, "collect",
                QStringLiteral("Collect common training poses: %1 / %2 (all three cameras).")
                    .arg(cal.common_count)
                    .arg(c.training_target)};
    if (!cal.training_complete)
        return {Step::Calibration, "solve", "Solve the common training poses."};
    if (cal.geometry_key != c.geometry_key())
        return {Step::Calibration, "resetCal",
                "Rig identity / square size changed: reset and collect training "
                "again."};
    if (cal.validation_count < c.validation_target)
        return {Step::Calibration, "collect",
                QStringLiteral("Collect NEW independent validation poses: %1 / %2; "
                               "training is frozen.")
                    .arg(cal.validation_count)
                    .arg(c.validation_target)};
    return {Step::Calibration, "validate",
            cal.verdict == Verdict::Failed
                ? QStringLiteral("Validation failed: inspect RMS / error overlays; "
                                 "replace hold-out poses and validate again.")
            : cal.verdict == Verdict::Passed
                ? QStringLiteral("RMS %1 px exceeds / lacks the project limit %2 px: "
                                 "inspect errors and validate again.")
                      .arg(cal.rms)
                      .arg(c.max_rms_px)
                : QStringLiteral("Validate the frozen calibration against the "
                                 "project RMS limit.")};
}
} // namespace

StageGate stage_gate(Step step, const RigConfig &c, const WorkflowSnapshot &s,
                     const GuidanceContext &context) {
    StageGate gate;
    if (step == Step::Calibration) {
        gate.pending = blockers(Step::Sync, c, s);
        if (!calibration_approved(c, s) || !c.optics_confirmed)
            gate.pending.push_back(calibration_action(c, s, context));
    } else if (step == Step::Capture) {
        const auto take_state = s.take_manifest.value("state").toString();
        const bool closed = !s.take_directory.isEmpty() && !s.recording() &&
                            (take_state == "completed" || take_state == "incomplete");
        if (closed) {
            if (s.trigger_stop_pending)
                gate.pending.push_back({Step::Sync, "stopTrigger",
                                        "Retry Stop Trigger: LOW / drive restoration not confirmed."});
            gate.complete = s.take_manifest.value("state").toString() == "completed" &&
                            s.take_report.value("countsAndProtocolPass").toBool() &&
                            s.take_verdict != Verdict::Failed;
            gate.status = gate.complete ? QStringLiteral("Take closed; decoder tail unknown")
                          : s.take_verdict == Verdict::Failed
                              ? QStringLiteral("Failed Take; inspect report")
                              : QStringLiteral("Incomplete Take; inspect report");
        } else {
            gate.pending = blockers(Step::Capture, c, s);
            // Replace the legacy calibration summary with the exact next calibration
            // action.
            for (auto &item : gate.pending)
                if (item.step == Step::Calibration && item.field == "calibration")
                    item = calibration_action(c, s, context);
            if (gate.pending.isEmpty())
                gate.pending.push_back({Step::Capture, "record",
                                        "Start a new Take: fresh Trigger preflight, "
                                        "then formal recording."});
        }
    } else {
        gate.pending = blockers(step, c, s);
    }
    const bool conditions_complete = gate.pending.isEmpty();
    if (s.trigger_stop_pending) {
        // The stop retry remains actionable even though all other operations are
        // busy.
        gate.pending.erase(std::remove_if(gate.pending.begin(), gate.pending.end(),
                                          [](const auto &b) { return b.field == "stopTrigger"; }),
                           gate.pending.end());
        gate.pending.prepend(
            {Step::Sync, "stopTrigger", "Retry Stop Trigger: LOW / drive restoration not confirmed."});
    } else if (s.busy()) {
        QString message;
        QString field = "stop";
        Step target = step;
        switch (s.activity) {
        case Activity::Connecting:
            target = Step::Devices;
            message = "Connecting cameras; wait or cancel.";
            break;
        case Activity::Syncing:
            target = Step::Sync;
            message = "Trigger preflight running; wait or cancel.";
            break;
        case Activity::Triggering:
            target = Step::Sync;
            field = "stopTrigger";
            message = "Continuous Trigger running; stop and confirm LOW before continuing.";
            break;
        case Activity::Calibrating:
            target = Step::Calibration;
            message = s.calibration.training_complete
                          ? QStringLiteral("Independent validation %1 / %2; hold still. "
                                           "This capture round stops automatically.")
                                .arg(s.calibration.validation_count)
                                .arg(c.validation_target)
                          : QStringLiteral("Common training %1 / %2; hold still. "
                                           "This capture round stops automatically.")
                                .arg(s.calibration.common_count)
                                .arg(c.training_target);
            break;
        case Activity::Solving:
            target = Step::Calibration;
            message = "Solving / validating calibration; wait for the result.";
            break;
        case Activity::Preparing:
            if (s.can_override_sync() && c.trigger_wiring_confirmed) {
                target = Step::Sync;
                field = "overrideSync";
                message = "New Take preflight failed: inspect traces and accept "
                          "manually with a reason, or cancel; formal time is zero.";
            } else {
                target = Step::Capture;
                message = "New Take preflight running; formal time has not started.";
            }
            break;
        case Activity::Recording:
            target = Step::Capture;
            message = "Recording; stop to close RAWs and verify the Take.";
            break;
        case Activity::Stopping:
            target = Step::Capture;
            message = "Stopping; wait for LOW and RAW closure.";
            break;
        default:
            target = Step::Capture;
            message = "Verifying closed RAWs; wait for the report.";
            break;
        }
        gate.pending.prepend({target, field, message, BlockerKind::Running});
    }
    if (step != Step::Capture)
        gate.complete = conditions_complete && !s.trigger_stop_pending;
    else if (!gate.pending.isEmpty())
        gate.complete = false;
    const bool failed = (step == Step::Sync && s.sync.verdict == Verdict::Failed &&
                         !s.sync.accepted(s.epoch, c.max_delta_us)) ||
                        (step == Step::Calibration && s.calibration.verdict == Verdict::Failed &&
                         !calibration_approved(c, s)) ||
                        (step == Step::Capture && s.take_verdict == Verdict::Failed);
    if (s.trigger_stop_pending)
        gate.status = QStringLiteral("Stop unconfirmed");
    else if (s.busy())
        gate.status = QStringLiteral("Running");
    else if (gate.status.isEmpty())
        gate.status = failed          ? QStringLiteral("Failed")
                      : gate.complete ? (step == Step::Sync && s.sync.operator_accepted
                                             ? QStringLiteral("Accepted manually")
                                             : QStringLiteral("Complete"))
                                      : QStringLiteral("Pending");
    if (step == Step::Calibration && !gate.complete && !s.busy()) {
        const auto action = calibration_action(c, s, context);
        const auto phase =
            action.field == "display" || action.field == "showBoard"  ? QStringLiteral("1/7 Display / board")
            : action.field == "square" || action.field == "lockBoard" ? QStringLiteral("2/7 Measure / lock")
            : action.field == "collect" && !s.calibration.training_complete
                ? QStringLiteral("3/7 Common training")
            : action.field == "solve" || action.field == "resetCal" ? QStringLiteral("4/7 Solve training")
            : action.field == "collect"  ? QStringLiteral("5/7 Independent validation")
            : action.field == "validate" ? QStringLiteral("6/7 Validate")
                                         : QStringLiteral("7/7 Confirm optics / rig");
        gate.status += " | " + phase;
    }
    gate.next_action =
        gate.pending.isEmpty()
            ? (step == Step::Review ? QStringLiteral("Inspect the timeline and retained report.")
                                    : QStringLiteral("Continue to %1.").arg(step_text(Step(int(step) + 1))))
            : gate.pending.front().message;
    return gate;
}

QString blocker_kind_text(BlockerKind kind) {
    switch (kind) {
    case BlockerKind::InvalidField:
        return QStringLiteral("Fix");
    case BlockerKind::Running:
        return QStringLiteral("Running");
    default:
        return QStringLiteral("To do");
    }
}

QString verdict_text(Verdict v) {
    switch (v) {
    case Verdict::Passed:
        return QStringLiteral("Passed");
    case Verdict::Failed:
        return QStringLiteral("Failed");
    case Verdict::Running:
        return QStringLiteral("Running");
    case Verdict::Incomplete:
        return QStringLiteral("Incomplete");
    default:
        return QStringLiteral("Not verified");
    }
}
QString step_text(Step s) {
    switch (s) {
    case Step::Devices:
        return QStringLiteral("Devices & project");
    case Step::Sync:
        return QStringLiteral("Synchronization");
    case Step::Calibration:
        return QStringLiteral("Guided calibration");
    case Step::Capture:
        return QStringLiteral("Recording");
    default:
        return QStringLiteral("Playback & report");
    }
}

std::array<NominalSlotGeometry, camera_count> nominal_layout_geometry(RigLayoutPreset preset) {
    switch (preset) {
    case RigLayoutPreset::LayoutB_LShape:
        return {{
            {QStringLiteral("Left"), QStringLiteral("L+90"), QVector3D(0.0f, 90.0f, 0.0f)},
            {QStringLiteral("Center"), QStringLiteral("O"), QVector3D(0.0f, 0.0f, 0.0f)},
            {QStringLiteral("Right"), QStringLiteral("L+120"), QVector3D(120.0f, 0.0f, 0.0f)},
        }};
    case RigLayoutPreset::LayoutC_LowTriangle:
        return {{
            {QStringLiteral("Left"), QStringLiteral("T-L"), QVector3D(-75.0f, -93.675f, 0.0f)},
            {QStringLiteral("Center"), QStringLiteral("O"), QVector3D(0.0f, 0.0f, 0.0f)},
            {QStringLiteral("Right"), QStringLiteral("T-R"), QVector3D(75.0f, -93.675f, 0.0f)},
        }};
    case RigLayoutPreset::LayoutA_Collinear:
    default:
        return {{
            {QStringLiteral("Left"), QStringLiteral("C-100"), QVector3D(-100.0f, 0.0f, 0.0f)},
            {QStringLiteral("Center"), QStringLiteral("O"), QVector3D(0.0f, 0.0f, 0.0f)},
            {QStringLiteral("Right"), QStringLiteral("C+50"), QVector3D(50.0f, 0.0f, 0.0f)},
        }};
    }
}

std::array<double, camera_count> nominal_baselines_mm(RigLayoutPreset preset) {
    switch (preset) {
    case RigLayoutPreset::LayoutB_LShape:
        return {90.0, 120.0, 150.0};
    case RigLayoutPreset::LayoutC_LowTriangle:
        return {120.0, 120.0, 150.0};
    case RigLayoutPreset::LayoutA_Collinear:
    default:
        return {100.0, 50.0, 150.0};
    }
}

BaselineSanityCheck evaluate_baseline_sanity(
    const std::array<double, camera_count> &nominal_mm,
    const std::array<double, camera_count> &measured_mm) {
    BaselineSanityCheck out;
    out.nominal_mm = nominal_mm;
    out.measured_mm = measured_mm;
    bool has_measured = true;
    for (int i = 0; i < camera_count; ++i) {
        if (!std::isfinite(measured_mm[i]) || measured_mm[i] <= 0.0) {
            has_measured = false;
            break;
        }
    }
    if (!has_measured) {
        out.verdict = BaselineSanityVerdict::Unknown;
        out.summary = QStringLiteral("等待标定求解以比对设计基线与实测平移模长 ||T||");
        return out;
    }
    double max_err = 0.0;
    for (int i = 0; i < camera_count; ++i) {
        out.abs_errors_mm[i] = std::abs(measured_mm[i] - nominal_mm[i]);
        max_err = std::max(max_err, out.abs_errors_mm[i]);
    }
    out.max_abs_error_mm = max_err;
    if (max_err <= 5.0) {
        out.verdict = BaselineSanityVerdict::Pass;
        out.summary = QStringLiteral("✓ 基线物理一致性校验通过 (最大偏差 %1 mm ≤ 5.0 mm)")
                          .arg(max_err, 0, 'f', 1);
    } else if (max_err <= 12.0) {
        out.verdict = BaselineSanityVerdict::Warning;
        out.summary = QStringLiteral("⚠ 基线轻微偏差 (最大偏差 %1 mm，5.0–12.0 mm)，建议复核视野边缘采样")
                          .arg(max_err, 0, 'f', 1);
    } else {
        out.verdict = BaselineSanityVerdict::Alert;
        out.summary = QStringLiteral("✗ 标定异常告警 (最大偏差 %1 mm > 12.0 mm)：请检查棋盘格方格尺寸或主板构型 A/B/C 是否选错！")
                          .arg(max_err, 0, 'f', 1);
    }
    return out;
}

BaselineSanityCheck evaluate_baseline_sanity(RigLayoutPreset preset, const QJsonObject &calibration_profile) {
    const auto nominal = nominal_baselines_mm(preset);
    std::array<double, camera_count> measured{0.0, 0.0, 0.0};
    const auto direct = calibration_profile.value("measuredBaselinesMm").toArray();
    if (direct.size() == camera_count) {
        for (int i = 0; i < camera_count; ++i)
            measured[i] = direct[i].toDouble(0.0);
        return evaluate_baseline_sanity(nominal, measured);
    }
    QJsonArray extrinsics = calibration_profile.value("cameras").toArray();
    if (extrinsics.size() != camera_count)
        extrinsics = calibration_profile.value("extrinsics").toArray();
    if (extrinsics.size() == camera_count) {
        const QString unit = calibration_profile.value("lengthUnit").toString().trimmed().toLower();
        const double scale_mm = (unit == QLatin1String("metre") || unit == QLatin1String("meter") ||
                                 unit == QLatin1String("m"))
                                    ? 1000.0
                                    : 1.0;
        std::array<std::array<double, 3>, camera_count> centers{};
        bool ok = true;
        for (int i = 0; i < camera_count; ++i) {
            const auto mat = extrinsics[i].toObject().value("T_camera_from_reference").toArray();
            double R[3][3]{};
            double t[3]{};
            if ((mat.size() == 16 || mat.size() == 12) && !mat[0].isArray()) {
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c)
                        R[r][c] = mat[r * 4 + c].toDouble(std::numeric_limits<double>::quiet_NaN());
                    t[r] = mat[r * 4 + 3].toDouble(std::numeric_limits<double>::quiet_NaN());
                }
                if (mat.size() == 16) {
                    const double b0 = mat[12].toDouble(1.0);
                    const double b1 = mat[13].toDouble(1.0);
                    const double b2 = mat[14].toDouble(1.0);
                    const double b3 = mat[15].toDouble(0.0);
                    if (!std::isfinite(b0) || !std::isfinite(b1) || !std::isfinite(b2) || !std::isfinite(b3) ||
                        std::abs(b0) > 1e-4 || std::abs(b1) > 1e-4 || std::abs(b2) > 1e-4 ||
                        std::abs(b3 - 1.0) > 1e-4) {
                        ok = false;
                        break;
                    }
                }
            } else if (mat.size() >= 3 && mat[0].isArray()) {
                for (int r = 0; r < 3; ++r) {
                    const auto row = mat[r].toArray();
                    if (row.size() < 4) {
                        ok = false;
                        break;
                    }
                    for (int c = 0; c < 3; ++c)
                        R[r][c] = row[c].toDouble(std::numeric_limits<double>::quiet_NaN());
                    t[r] = row[3].toDouble(std::numeric_limits<double>::quiet_NaN());
                }
                if (ok && mat.size() >= 4 && mat[3].isArray()) {
                    const auto row3 = mat[3].toArray();
                    if (row3.size() < 4 || std::abs(row3[0].toDouble(1.0)) > 1e-4 ||
                        std::abs(row3[1].toDouble(1.0)) > 1e-4 || std::abs(row3[2].toDouble(1.0)) > 1e-4 ||
                        std::abs(row3[3].toDouble(0.0) - 1.0) > 1e-4)
                        ok = false;
                }
            } else {
                ok = false;
            }
            if (!ok)
                break;
            for (int r = 0; r < 3; ++r) {
                if (!std::isfinite(t[r]))
                    ok = false;
                for (int c = 0; c < 3; ++c)
                    if (!std::isfinite(R[r][c]))
                        ok = false;
            }
            if (!ok)
                break;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    const double dot = R[0][r] * R[0][c] + R[1][r] * R[1][c] + R[2][r] * R[2][c];
                    if (std::abs(dot - (r == c ? 1.0 : 0.0)) > 1e-2)
                        ok = false;
                }
            }
            const double det = R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) -
                               R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0]) +
                               R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]);
            if (!ok || !std::isfinite(det) || std::abs(det - 1.0) > 1e-2) {
                ok = false;
                break;
            }
            centers[i][0] = -(R[0][0] * t[0] + R[1][0] * t[1] + R[2][0] * t[2]) * scale_mm;
            centers[i][1] = -(R[0][1] * t[0] + R[1][1] * t[1] + R[2][1] * t[2]) * scale_mm;
            centers[i][2] = -(R[0][2] * t[0] + R[1][2] * t[1] + R[2][2] * t[2]) * scale_mm;
        }
        if (ok) {
            const auto dist = [&](int a, int b) {
                const double dx = centers[a][0] - centers[b][0];
                const double dy = centers[a][1] - centers[b][1];
                const double dz = centers[a][2] - centers[b][2];
                return std::sqrt(dx * dx + dy * dy + dz * dz);
            };
            measured[0] = dist(0, 1);
            measured[1] = dist(1, 2);
            measured[2] = dist(0, 2);
        }
    }
    return evaluate_baseline_sanity(nominal, measured);
}

BaselineSanityCheck RigProfileRecord::sanity_check() const {
    return evaluate_baseline_sanity(nominal_baselines_mm, measured_baselines_mm);
}

QString layout_preset_code(RigLayoutPreset preset) {
    switch (preset) {
    case RigLayoutPreset::LayoutB_LShape:
        return QStringLiteral("B");
    case RigLayoutPreset::LayoutC_LowTriangle:
        return QStringLiteral("C");
    case RigLayoutPreset::LayoutA_Collinear:
    default:
        return QStringLiteral("A");
    }
}

RigLayoutPreset layout_preset_from_code(const QString &code) {
    const auto upper = code.trimmed().toUpper();
    if (upper == QLatin1String("B") || upper.contains(QLatin1String("LSHAPE")))
        return RigLayoutPreset::LayoutB_LShape;
    if (upper == QLatin1String("C") || upper.contains(QLatin1String("TRIANGLE")))
        return RigLayoutPreset::LayoutC_LowTriangle;
    return RigLayoutPreset::LayoutA_Collinear;
}

QString layout_preset_title(RigLayoutPreset preset) {
    switch (preset) {
    case RigLayoutPreset::LayoutB_LShape:
        return QStringLiteral("构型 B · 直角 L 形 (90 / 120 / 150 mm)");
    case RigLayoutPreset::LayoutC_LowTriangle:
        return QStringLiteral("构型 C · 低矮等腰三角形 (120 / 120 / 150 mm)");
    case RigLayoutPreset::LayoutA_Collinear:
    default:
        return QStringLiteral("构型 A · 非等距共线型 (100 / 50 / 150 mm)");
    }
}

QString sanitize_profile_id(const QString &raw_name) {
    QString s = raw_name.trimmed();
    s.replace(QRegularExpression(QStringLiteral(R"([\\/:*?"<>|\s]+)")), QStringLiteral("_"));
    while (s.startsWith('_'))
        s.remove(0, 1);
    while (s.endsWith('_'))
        s.chop(1);
    return s.isEmpty() ? QStringLiteral("Profile_Default") : s;
}

namespace {
QJsonObject read_json_obj_file(const QString &path) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly))
        return {};
    return QJsonDocument::fromJson(f.readAll()).object();
}

Verdict verdict_from_str(const QString &v) {
    const auto s = v.trimmed().toLower();
    if (s == QLatin1String("passed"))
        return Verdict::Passed;
    if (s == QLatin1String("failed"))
        return Verdict::Failed;
    if (s == QLatin1String("incomplete"))
        return Verdict::Incomplete;
    if (s == QLatin1String("running"))
        return Verdict::Running;
    return Verdict::Unknown;
}

RigProfileTakeItem parse_take_dir_item(const QString &take_dir, const QString &profile_id, bool is_legacy) {
    RigProfileTakeItem item;
    const QFileInfo fi(take_dir);
    item.take_id = fi.fileName();
    item.take_dir = fi.absoluteFilePath();
    item.profile_id = profile_id;
    item.is_legacy = is_legacy;
    auto manifest = read_json_obj_file(QDir(take_dir).filePath(QStringLiteral("session.json")));
    if (manifest.isEmpty())
        manifest = read_json_obj_file(QDir(take_dir).filePath(QStringLiteral("take.json")));
    auto report = read_json_obj_file(QDir(take_dir).filePath(QStringLiteral("sync-report.json")));
    if (report.isEmpty())
        report = manifest.value("sync").toObject();
    item.label = manifest.value("takeLabel").toString();
    if (item.label.isEmpty())
        item.label = manifest.value("note").toString();
    if (item.label.isEmpty())
        item.label = item.take_id;
    item.created_at = manifest.value("startedUtc").toString();
    if (item.created_at.isEmpty())
        item.created_at = fi.lastModified().toUTC().toString(Qt::ISODate);
    item.duration_s = manifest.value("elapsedMs").toDouble(0.0) / 1000.0;
    if (item.duration_s <= 0.0)
        item.duration_s = manifest.value("durationSeconds").toDouble(0.0);
    item.verdict = verdict_from_str(manifest.value("verdict").toString(
        manifest.value("state").toString() == QLatin1String("completed") ? QStringLiteral("passed")
                                                                         : report.value("verdict").toString()));
    item.max_delta_us = report.value("maxAbsDeltaUs").toDouble(0.0);
    return item;
}
} // namespace

bool save_profile_to_disk(const QString &root_output_dir, const RigProfileRecord &record, QString *out_dir, QString *error) {
    const QString pid = sanitize_profile_id(record.profile_id.isEmpty() ? record.display_name : record.profile_id);
    const QString pdir = QDir(root_output_dir).filePath(QStringLiteral("profiles/") + pid);
    if (!QDir().mkpath(QDir(pdir).filePath(QStringLiteral("takes")))) {
        if (error)
            *error = QStringLiteral("Unable to create profile directory: %1").arg(pdir);
        return false;
    }
    QJsonArray serials_arr, nom_arr, meas_arr, focal_arr;
    for (int i = 0; i < camera_count; ++i) {
        serials_arr.append(record.serials[i]);
        nom_arr.append(record.nominal_baselines_mm[i]);
        meas_arr.append(record.measured_baselines_mm[i]);
        focal_arr.append(record.focal_px[i]);
    }
    const QJsonObject meta{
        {"schemaVersion", 1},
        {"profileId", pid},
        {"displayName", record.display_name.isEmpty() ? pid : record.display_name},
        {"createdAt", record.created_at.isEmpty() ? QDateTime::currentDateTimeUtc().toString(Qt::ISODate) : record.created_at},
        {"layoutPreset", layout_preset_code(record.layout_preset)},
        {"serials", serials_arr},
        {"nominalBaselinesMm", nom_arr},
        {"measuredBaselinesMm", meas_arr},
        {"focalPx", focal_arr},
        {"rmsPx", record.rms_px},
        {"maxSyncDeltaUs", record.max_sync_delta_us},
    };
    QFile pf(QDir(pdir).filePath(QStringLiteral("profile.json")));
    if (!pf.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (error)
            *error = pf.errorString();
        return false;
    }
    pf.write(QJsonDocument(meta).toJson(QJsonDocument::Indented));
    pf.close();

    QJsonObject cal = record.calibration_json;
    cal.insert("profileId", pid);
    cal.insert("layoutPreset", layout_preset_code(record.layout_preset));
    cal.insert("nominalBaselinesMm", nom_arr);
    cal.insert("measuredBaselinesMm", meas_arr);
    QFile cf(QDir(pdir).filePath(QStringLiteral("calibration.json")));
    if (cf.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        cf.write(QJsonDocument(cal).toJson(QJsonDocument::Indented));
        cf.close();
    }
    if (out_dir)
        *out_dir = pdir;
    return true;
}

RigProfileRecord load_profile_from_disk(const QString &profile_dir) {
    RigProfileRecord rec;
    const QDir dir(profile_dir);
    if (!dir.exists())
        return rec;
    const auto meta = read_json_obj_file(dir.filePath(QStringLiteral("profile.json")));
    const auto cal = read_json_obj_file(dir.filePath(QStringLiteral("calibration.json")));
    if (meta.isEmpty() && cal.isEmpty())
        return rec;
    rec.directory_path = dir.absolutePath();
    rec.profile_id = meta.value("profileId").toString(dir.dirName());
    rec.display_name = meta.value("displayName").toString(rec.profile_id);
    rec.created_at = meta.value("createdAt").toString(cal.value("calibratedUtc").toString());
    rec.layout_preset = layout_preset_from_code(
        meta.value("layoutPreset").toString(cal.value("layoutPreset").toString(QStringLiteral("A"))));
    rec.nominal_slots = nominal_layout_geometry(rec.layout_preset);
    rec.nominal_baselines_mm = nominal_baselines_mm(rec.layout_preset);
    const auto s_arr = meta.value("serials").toArray();
    const auto cal_cams = cal.value("cameras").toArray();
    for (int i = 0; i < camera_count; ++i) {
        if (i < s_arr.size() && !s_arr[i].toString().isEmpty())
            rec.serials[i] = s_arr[i].toString();
        else if (i < cal_cams.size())
            rec.serials[i] = cal_cams[i].toObject().value("serial").toString();
    }
    const auto m_arr = meta.value("measuredBaselinesMm").toArray();
    if (m_arr.size() == camera_count) {
        for (int i = 0; i < camera_count; ++i)
            rec.measured_baselines_mm[i] = m_arr[i].toDouble();
    } else if (!cal.isEmpty()) {
        rec.measured_baselines_mm = evaluate_baseline_sanity(rec.layout_preset, cal).measured_mm;
    }
    const auto f_arr = meta.value("focalPx").toArray();
    const auto intr_arr = cal.value("intrinsics").toArray();
    for (int i = 0; i < camera_count; ++i) {
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
    rec.rms_px = meta.value("rmsPx").toDouble(cal.value("rms").toDouble(0.0));
    if (rec.rms_px <= 0.0 && cal_cams.size() == camera_count) {
        double max_rms = 0.0;
        for (int i = 0; i < camera_count; ++i) {
            const auto cobj = cal_cams[i].toObject();
            max_rms = std::max(max_rms, cobj.value("validationRmsPx").toDouble(cobj.value("trainingRmsPx").toDouble(0.0)));
        }
        rec.rms_px = max_rms;
    }
    rec.max_sync_delta_us = meta.value("maxSyncDeltaUs").toDouble(0.0);
    rec.calibration_json = cal;

    const QDir takes_dir(dir.filePath(QStringLiteral("takes")));
    if (takes_dir.exists()) {
        const auto entries = takes_dir.entryInfoList(QStringList{QStringLiteral("take-*")},
                                                     QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name | QDir::Reversed);
        for (const auto &entry : entries)
            rec.takes.push_back(parse_take_dir_item(entry.absoluteFilePath(), rec.profile_id, false));
    }
    return rec;
}

QVector<RigProfileRecord> scan_profiles_on_disk(const QString &root_output_dir) {
    QVector<RigProfileRecord> out;
    const QDir profiles_dir(QDir(root_output_dir).filePath(QStringLiteral("profiles")));
    if (!profiles_dir.exists())
        return out;
    const auto entries = profiles_dir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    for (const auto &entry : entries) {
        auto rec = load_profile_from_disk(entry.absoluteFilePath());
        if (rec.valid())
            out.push_back(std::move(rec));
    }
    return out;
}

QVector<RigProfileTakeItem> scan_legacy_takes_on_disk(const QString &root_output_dir) {
    QVector<RigProfileTakeItem> out;
    const QDir root(root_output_dir);
    if (!root.exists())
        return out;
    const auto entries = root.entryInfoList(QStringList{QStringLiteral("take-*")},
                                            QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name | QDir::Reversed);
    for (const auto &entry : entries)
        out.push_back(parse_take_dir_item(entry.absoluteFilePath(), QStringLiteral("Legacy"), true));
    return out;
}

bool delete_profile_from_disk(const QString &root_output_dir, const QString &profile_id, QString *error) {
    const QString pid = sanitize_profile_id(profile_id);
    QDir pdir(QDir(root_output_dir).filePath(QStringLiteral("profiles/") + pid));
    if (!pdir.exists())
        return true;
    if (!pdir.removeRecursively()) {
        if (error)
            *error = QStringLiteral("Failed to delete profile directory: %1").arg(pdir.absolutePath());
        return false;
    }
    return true;
}
} // namespace gui::multicam


