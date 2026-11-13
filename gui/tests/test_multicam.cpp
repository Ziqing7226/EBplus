#include "main_window.h"
#include "multicam/workspace.h"
#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QSignalBlocker>
#include <QTemporaryDir>
#include <QToolButton>
#include <chrono>
#include <gtest/gtest.h>
#include <stdexcept>
#include <thread>

using namespace gui::multicam;
namespace {
RigConfig valid_config() {
    RigConfig c;
    for (int i = 0; i < 3; ++i)
        c.cameras[i] = {QString("vendor:silky:%1").arg(i), QString::number(i), c.cameras[i].position};
    c.roles_confirmed = true;
    c.trigger_wiring_confirmed = true;
    c.optics_confirmed = true;
    c.max_delta_us = 10;
    c.max_rms_px = .7;
    c.square_mm = 25;
    c.board_locked = true;
    return c;
}
DetectedGroup pose(qint64 time, float x = 250, float y = 160) {
    DetectedGroup g;
    g.start_us = time;
    for (int i = 0; i < 3; ++i) {
        g.sizes[i] = {1280, 720};
        for (int row = 0; row < 6; ++row)
            for (int col = 0; col < 9; ++col)
                g.corners[i].push_back({x + col * 45 + i * 8, y + row * 45});
    }
    return g;
}
void hold(Guidance &guide, qint64 start, bool automatic = true, int extra = -1) {
    for (int i = 0; i <= 10; ++i)
        guide.observe(pose(start + i * 100000), automatic, false, extra);
}
TEST(MultiCameraGates, MissingConditionsDoNotAcquireScientificDefaults) {
    RigConfig c;
    EXPECT_FALSE(std::isfinite(c.max_delta_us));
    EXPECT_FALSE(std::isfinite(c.max_rms_px));
    EXPECT_GE(configuration_blockers(c).size(), 6);
    EXPECT_FALSE(c.roles_confirmed);
}
TEST(MultiCameraGates, EpochAndOpticsAreRecordingGates) {
    auto c = valid_config();
    WorkflowSnapshot s;
    for (auto &camera : s.cameras)
        camera.connected = true;
    s.board_ready = true;
    s.epoch = 2;
    s.sync.verdict = Verdict::Passed;
    s.sync.epoch = 1;
    EXPECT_FALSE(blockers(Step::Sync, c, s).isEmpty());
    s.sync.epoch = 2;
    EXPECT_TRUE(blockers(Step::Sync, c, s).isEmpty());
    s.calibration.verdict = Verdict::Passed;
    s.calibration.geometry_key = c.geometry_key();
    s.calibration.rms = .3;
    EXPECT_TRUE(blockers(Step::Capture, c, s).isEmpty());
    c.optics_confirmed = false;
    EXPECT_FALSE(blockers(Step::Capture, c, s).isEmpty());
}
TEST(MultiCameraGates, DuplicateSerialsAreBlockedEvenWithDifferentPlugins) {
    auto c = valid_config();
    c.cameras[2].serial = c.cameras[0].serial;
    EXPECT_FALSE(configuration_blockers(c).isEmpty());
}
TEST(MultiCameraGates, ConfigurationRoundTripDoesNotTrustOldHumanConfirmations) {
    auto c = valid_config();
    auto loaded = RigConfig::from_json(c.to_json());
    EXPECT_EQ(loaded.geometry_key(), c.geometry_key());
    EXPECT_FALSE(loaded.optics_confirmed);
    EXPECT_FALSE(loaded.roles_confirmed);
    EXPECT_FALSE(loaded.board_locked);
}
TEST(MultiCameraGates, CalibrationCaptureDurationIsBoundedAndRoundTrips) {
    auto c = valid_config();
    c.calibration_capture_seconds = 7;
    EXPECT_EQ(RigConfig::from_json(c.to_json()).calibration_capture_seconds, 7);
    EXPECT_TRUE(configuration_blockers(c).isEmpty());
    for (int duration : {0, 2, 31, 1000000}) {
        c.calibration_capture_seconds = duration;
        const auto pending = configuration_blockers(c);
        EXPECT_TRUE(std::any_of(pending.begin(), pending.end(),
                                [](const auto &b) { return b.field == "captureDuration"; }));
    }
    EXPECT_EQ(RigConfig::from_json({}).calibration_capture_seconds, 5);
}
TEST(MultiCameraGates, PreflightCannotBeMistakenForIdle) {
    WorkflowSnapshot s;
    s.take_verdict = Verdict::Running;
    s.take_directory = "take";
    s.activity = Activity::Syncing;
    EXPECT_TRUE(s.recording());
    EXPECT_TRUE(s.busy());
}
TEST(MultiCameraGuide, StableForOneSecondThenAcceptsExactlyOnce) {
    Guidance g;
    for (int i = 0; i < 10; ++i)
        g.observe(pose(i * 100000), true, false);
    EXPECT_EQ(g.training().size(), 0);
    g.observe(pose(1000000), true, false);
    EXPECT_EQ(g.training().size(), 1);
    hold(g, 1100000);
    EXPECT_EQ(g.training().size(), 1);
    EXPECT_TRUE(g.status(valid_config()).reason.contains("Repeated"));
}
TEST(MultiCameraGuide, MovementAndMissingCommonFieldResetCountdown) {
    Guidance g;
    for (int i = 0; i < 9; ++i)
        g.observe(pose(i * 100000), true, false);
    auto moved = pose(900000, 265);
    g.observe(moved, true, false);
    EXPECT_LT(g.status(valid_config()).stable_progress, .1);
    auto missing = pose(1000000);
    missing.corners[0].clear();
    g.observe(missing, true, false);
    EXPECT_FALSE(g.stable());
    EXPECT_EQ(g.training().size(), 0);
    EXPECT_TRUE(g.status(valid_config()).reason.contains("Camera 1"));
}
TEST(MultiCameraGuide, SupplementalIsNotACommonGroup) {
    Guidance g;
    hold(g, 0, true, 0);
    EXPECT_TRUE(g.training().empty());
    auto s = g.status(valid_config());
    EXPECT_EQ(s.intrinsic_counts[0], 1);
    EXPECT_EQ(s.intrinsic_counts[1], 0);
    EXPECT_TRUE(s.reason.contains("Intrinsic-only"));
}
TEST(MultiCameraGuide, ManualAcceptCannotBypassStabilityOrCompleteness) {
    Guidance g;
    auto missing = pose(0);
    missing.corners[2].clear();
    g.observe(missing, false, true);
    EXPECT_TRUE(g.training().empty());
    g.observe(pose(100000), false, true);
    EXPECT_TRUE(g.training().empty());
    hold(g, 200000, false);
    EXPECT_TRUE(g.training().empty());
    g.observe(pose(1300000), false, true);
    EXPECT_EQ(g.training().size(), 1);
}
TEST(MultiCameraGuide, ValidationStaysSeparateAndDeleteRevokesApproval) {
    Guidance g;
    hold(g, 0);
    gui_algo::rig::Result solved;
    solved.trained = true;
    g.set_result(solved);
    for (int i = 0; i < 11; ++i)
        g.observe(pose(2000000 + i * 100000, 350), true, false);
    EXPECT_EQ(g.training().size(), 1);
    EXPECT_EQ(g.validation().size(), 1);
    solved.validated = true;
    g.set_result(solved);
    g.remove_last();
    EXPECT_FALSE(g.result().validated);
    EXPECT_TRUE(g.validation().empty());
    EXPECT_EQ(g.training().size(), 1);
}
TEST(MultiCameraSync, UniqueFormalEdgesAndExtrasHaveSeparateOutcomes) {
    std::array<QVector<TriggerEdge>, 3> edges;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 6; ++j)
            edges[i].push_back({10000000 + j * 1000000 + i * 2, j % 2 ? 0 : 1, 0});
    auto s = match_trigger_edges(edges, 1, 3, 9);
    EXPECT_EQ(s.verdict, Verdict::Passed);
    EXPECT_EQ(s.max_delta_us, 2);
    EXPECT_EQ(s.epoch, 9);
    edges[0].push_back({16000000, 0, 1});
    s = match_trigger_edges(edges, 1, 3, 9);
    EXPECT_EQ(s.verdict, Verdict::Passed);
    EXPECT_EQ(s.extras[0], 1);
    EXPECT_TRUE(s.reason.contains("not certified"));
}
TEST(MultiCameraSync, InvertedTriggerInputMatchesAndMixedPolarityIsRejected) {
    // Real rig data (2026-11-03): GP2 rise is decoded as polarity 0 on all three SilkyEvCam inputs,
    // with 999 872 us spacing and 0-1 us skew. That must pass and be flagged as inverted.
    std::array<QVector<TriggerEdge>, 3> edges;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 6; ++j)
            edges[i].push_back({8039292 + j * 999872 + (i == 1), j % 2 ? 1 : 0, 0});
    auto s = match_trigger_edges(edges, 1, 5, 1);
    EXPECT_EQ(s.verdict, Verdict::Passed) << s.reason.toStdString();
    EXPECT_TRUE(s.polarity_inverted);
    EXPECT_EQ(s.max_delta_us, 1);
    EXPECT_TRUE(s.to_json().value("triggerPolarityInverted").toBool());
    for (auto &e : edges[2])
        e.polarity = 1 - e.polarity;
    s = match_trigger_edges(edges, 1, 5, 1);
    EXPECT_EQ(s.verdict, Verdict::Failed);
    EXPECT_TRUE(s.reason.startsWith("Right camera")) << s.reason.toStdString();
    EXPECT_TRUE(s.reason.contains("polarity"));
    edges[2].remove(2);
    s = match_trigger_edges(edges, 1, 5, 1);
    EXPECT_TRUE(s.reason.startsWith("Right camera: 0 formal six-edge matches")) << s.reason.toStdString();
    EXPECT_TRUE(s.reason.contains("only 5 of 6")) << s.reason.toStdString();
}
TEST(MultiCameraSync, MissingAmbiguousAndOutOfToleranceAreNotPassed) {
    std::array<QVector<TriggerEdge>, 3> edges;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 6; ++j)
            edges[i].push_back({10000000 + j * 1000000 + i * 20, j % 2 ? 0 : 1, 0});
    EXPECT_EQ(match_trigger_edges(edges, 1, 10, 0).verdict, Verdict::Failed);
    edges[2].pop_back();
    EXPECT_EQ(match_trigger_edges(edges, 1, 100, 0).verdict, Verdict::Failed);
    for (auto &c : edges) {
        c.clear();
        for (int j = 0; j < 8; ++j)
            c.push_back({j * 1000000, j % 2 ? 0 : 1, 0});
    }
    EXPECT_EQ(match_trigger_edges(edges, 1, 100, 0).verdict, Verdict::Failed);
}
TEST(MultiCameraSync, UnmatchedRecordsDoNotSerializeZeroAsAMeasuredOffset) {
    std::array<QVector<TriggerEdge>, 3> edges;
    const auto unknown = match_trigger_edges(edges, 1, 10, 0).to_json();
    EXPECT_FALSE(unknown.value("timingMeasured").toBool());
    EXPECT_TRUE(unknown.value("maxDeltaUs").isNull());
    EXPECT_TRUE(unknown.value("edgeDeltaFromMasterUs").isNull());
    EXPECT_FALSE(unknown.value("extraRecordsClassified").toBool());
    EXPECT_TRUE(unknown.value("extraRecords").isNull());
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 6; ++j)
            edges[i].push_back({10000000 + j * 1000000 + i * 20, j % 2 ? 0 : 1, 0});
    const auto over_limit = match_trigger_edges(edges, 1, 10, 0).to_json();
    EXPECT_TRUE(over_limit.value("timingMeasured").toBool());
    EXPECT_EQ(over_limit.value("maxDeltaUs").toDouble(), 20);
}
TEST(MultiCameraMcu, RawReplHandlesEveryFragmentBoundary) {
    const QByteArray bytes = QByteArray("OKhello\n", 8) + char(4) + char(4) + ">";
    for (int split = 0; split < bytes.size(); ++split) {
        RawReplResponse r;
        r.feed(bytes.left(split));
        r.feed(bytes.mid(split));
        EXPECT_TRUE(r.complete()) << split;
        EXPECT_EQ(r.output(), "hello\n");
    }
}
TEST(MultiCameraMcu, RawReplRejectsErrorsAndUnboundedResponses) {
    RawReplResponse r;
    r.feed(QByteArray("OKout") + char(4) + "Traceback" + char(4));
    EXPECT_TRUE(r.failed());
    RawReplResponse n;
    n.feed("NO");
    EXPECT_TRUE(n.failed());
    RawReplResponse large;
    large.feed(QByteArray(1024 * 1024 + 1, 'x'));
    EXPECT_TRUE(large.failed());
}
TEST(MultiCameraMcu, ExplicitDiagnosticDriveMustReadBackAndRestore) {
    QJsonObject response{{"output", "low"},
                         {"gpio", 2},
                         {"pulses", 3},
                         {"expected_edges", 6},
                         {"high_us", 1000000},
                         {"low_us", 1000000},
                         {"drive_strength_mA_before", 4},
                         {"drive_strength_mA_during", 12},
                         {"drive_strength_mA_after", 4}};
    EXPECT_TRUE(valid_finite_burst_response(response, 3, 4, 12));
    EXPECT_FALSE(valid_finite_burst_response(response, 3, 4));
    response.insert("drive_strength_mA_after", 12);
    EXPECT_FALSE(valid_finite_burst_response(response, 3, 4, 12));
    response.insert("drive_strength_mA_after", 4);
    response.insert("drive_strength_mA_during", 4);
    EXPECT_FALSE(valid_finite_burst_response(response, 3, 4, 12));
    EXPECT_TRUE(valid_finite_burst_response(response, 3, 4));
    response.insert("drive_strength_mA_before", 8);
    EXPECT_FALSE(valid_finite_burst_response(response, 3, 4));
    response.insert("drive_strength_mA_before", 4);
    response.remove("drive_strength_mA_after");
    EXPECT_FALSE(valid_finite_burst_response(response, 3, 4, 12));
}
TEST(MultiCameraMcu, UnsupportedDiagnosticDriveIsRejectedWithoutHardware) {
    EXPECT_THROW(SerialMcuTransport(8), std::invalid_argument);
    EXPECT_THROW(SerialMcuTransport(-1), std::invalid_argument);
    EXPECT_NO_THROW(SerialMcuTransport{});
    EXPECT_NO_THROW(SerialMcuTransport{12});
}
TEST(MultiCameraTake, LegacyOrderIsPreservedAndUnknownGeometryIsNotInvented) {
    QTemporaryDir dir;
    QFile raw(dir.filePath("one.raw"));
    ASSERT_TRUE(raw.open(QIODevice::WriteOnly));
    raw.write("data");
    raw.close();
    QJsonArray cameras;
    for (int i = 0; i < 3; ++i)
        cameras.append(QJsonObject{{"serial", QString::number(3 - i)},
                                   {"role", i == 2 ? "master" : "slave"},
                                   {"rawFile", "/old/location/one.raw"},
                                   {"firstTimestampUs", qint64(100 + i * 10)},
                                   {"lastTimestampUs", qint64(500 + i * 10)},
                                   {"eventCount", 10}});
    ASSERT_TRUE(write_json(dir.filePath("session.json"), {{"state", "completed"}, {"cameras", cameras}}));
    auto take = read_take(dir.path());
    EXPECT_EQ(take.cameras[0].serial, "3");
    EXPECT_TRUE(take.cameras[0].position.isEmpty());
    EXPECT_EQ(take.timeline.formal_start_us, -1);
    EXPECT_TRUE(take.timeline.include_preflight);
    EXPECT_EQ(take.timeline.first_us, 100);
    EXPECT_EQ(take.timeline.last_us, 520);
}
TEST(MultiCameraTake, SymlinkCannotEscapeTheTake) {
    QTemporaryDir dir, out;
    QFile raw(out.filePath("outside.raw"));
    ASSERT_TRUE(raw.open(QIODevice::WriteOnly));
    raw.close();
    ASSERT_TRUE(QFile::link(raw.fileName(), dir.filePath("link.raw")));
    QJsonObject camera{
        {"serial", "1"}, {"rawFile", "link.raw"}, {"firstTimestampUs", 1}, {"lastTimestampUs", 2}};
    write_json(dir.filePath("session.json"), {{"state", "incomplete"}, {"cameras", QJsonArray{camera}}});
    EXPECT_THROW(read_take(dir.path()), std::runtime_error);
}
TEST(MultiCameraUI, BlockersLocateFieldsAndStepsRemainDistinct) {
    Workspace w;
    w.show();
    auto *list = w.findChild<QListWidget *>("multiBlockers");
    ASSERT_NE(list, nullptr);
    EXPECT_GT(list->count(), 0);
    auto *delta = w.findChild<QLineEdit *>("delta");
    ASSERT_NE(delta, nullptr);
    EXPECT_TRUE(delta->property("multiInvalid").toBool());
    for (int i = 0; i < list->count(); ++i)
        if (list->item(i)->data(Qt::UserRole).toString() == "delta") {
            ASSERT_TRUE(QMetaObject::invokeMethod(list, "itemClicked", Qt::DirectConnection,
                                                  Q_ARG(QListWidgetItem *, list->item(i))));
            EXPECT_EQ(w.current_step(), Step::Devices);
        }
    auto *steps = w.findChild<QListWidget *>("multiSteps");
    ASSERT_NE(steps, nullptr);
    steps->setCurrentRow(2);
    EXPECT_EQ(w.current_step(), Step::Calibration);
    EXPECT_EQ(steps->item(2)->data(Qt::UserRole).toString(), "current");
    EXPECT_EQ(steps->item(4)->data(Qt::UserRole).toString(), "pending");
    EXPECT_TRUE(steps->item(4)->text().contains("Pending"));
    steps->setCurrentRow(0);
    auto *first = list->item(0);
    WorkflowSnapshot snapshot;
    w.display_snapshot(snapshot);
    EXPECT_EQ(list->item(0), first);
    delta->setText("10");
    EXPECT_FALSE(delta->property("multiInvalid").toBool());
    EXPECT_FALSE(delta->toolTip().contains("Enter a non-negative"));
    w.toolbox_action()->setChecked(false);
    w.toolbox_action()->setChecked(true);
    EXPECT_TRUE(w.toolbox_action()->isChecked());
}
TEST(MultiCameraUI, RecordingBlocksModeSwitchAndFields) {
    Workspace w;
    WorkflowSnapshot s;
    s.activity = Activity::Recording;
    w.display_snapshot(s);
    EXPECT_FALSE(w.may_leave());
    EXPECT_FALSE(w.findChild<QLineEdit *>("project")->isEnabled());
    s.activity = Activity::Idle;
    w.display_snapshot(s);
    EXPECT_TRUE(w.may_leave());
}
TEST(MultiCameraUI, TriggerControlsAndManualAcceptanceHaveDistinctStates) {
    Workspace w;
    auto *wiring = w.findChild<QCheckBox *>("triggerWiring");
    auto *start = w.findChild<QToolButton *>("startTrigger");
    auto *stop = w.findChild<QToolButton *>("stopTrigger");
    auto *accept = w.findChild<QToolButton *>("overrideSync");
    auto *status = w.findChild<QLabel *>("multiSyncStatus");
    ASSERT_TRUE(wiring && start && stop && accept && status);
    EXPECT_FALSE(start->isEnabled());
    EXPECT_FALSE(stop->isEnabled());
    EXPECT_FALSE(accept->isEnabled());
    w.findChild<QLineEdit *>("delta")->setText("1");
    w.findChild<QLineEdit *>("rms")->setText("0.7");
    for (int i = 0; i < 3; ++i) {
        auto *camera = w.findChild<QComboBox *>(QString("camera%1").arg(i));
        camera->addItem(QString::number(i), QString("vendor:silky:%1").arg(i));
        camera->setItemData(camera->count() - 1, QString::number(i), Qt::UserRole + 1);
        camera->setCurrentIndex(camera->count() - 1);
    }
    w.findChild<QCheckBox *>("roles")->setChecked(true);
    wiring->setChecked(true);
    auto *steps = w.findChild<QListWidget *>("multiSteps");
    steps->setCurrentRow(1);
    WorkflowSnapshot s;
    s.epoch = s.sync.epoch = 1;
    s.sync.attempt = 1;
    s.sync.attempt_completed = true;
    s.sync.verdict = Verdict::Failed;
    s.sync.reason = "Missing initial edge; partial coverage";
    s.board_ready = true;
    for (int i = 0; i < 3; ++i) {
        s.cameras[i].connected = true;
        s.cameras[i].binding = {QString("vendor:silky:%1").arg(i), QString::number(i),
                                QStringList{"Left", "Center", "Right"}[i]};
        for (int edge = 0; edge < 4; ++edge)
            s.sync.observed_edges[i].push_back({8000000 + edge * 1000000 + (i == 1 ? 1 : 0), edge % 2, 0});
    }
    w.display_snapshot(s);
    EXPECT_TRUE(accept->isEnabled());
    EXPECT_TRUE(status->text().contains("not measured"));
    s.activity = Activity::Triggering;
    s.trigger_stop_pending = true;
    w.display_snapshot(s);
    EXPECT_TRUE(stop->isEnabled());
    EXPECT_FALSE(accept->isEnabled());
    EXPECT_FALSE(w.may_leave());
    s.activity = Activity::Idle;
    s.trigger_stop_pending = false;
    s.sync.operator_accepted = true;
    s.sync.operator_reason = "Later edges aligned";
    s.sync.operator_accepted_utc = "2026-10-20T09:00:00Z";
    w.display_snapshot(s);
    EXPECT_FALSE(accept->isEnabled());
    EXPECT_TRUE(status->text().contains("Accepted manually"));
    EXPECT_TRUE(status->text().contains(s.sync.reason));
    EXPECT_EQ(steps->item(1)->data(Qt::UserRole).toString(), "complete");
    EXPECT_FALSE(w.configuration().to_json().value("triggerWiringConfirmed").isNull());
    EXPECT_FALSE(RigConfig::from_json(w.configuration().to_json()).trigger_wiring_confirmed);
    if (const auto folder = qEnvironmentVariable("EBPLUS_TEST_SCREENSHOT_DIR"); !folder.isEmpty()) {
        ASSERT_TRUE(QDir().mkpath(folder));
        w.resize(1440, 900);
        w.show();
        steps->setCurrentRow(1);
        QApplication::processEvents();
        w.display_snapshot(s);
        QImage image(w.size(), QImage::Format_RGB32);
        w.render(&image);
        ASSERT_TRUE(image.save(QDir(folder).filePath("sync-manual-accepted.png")));
        w.resize(1180, 740);
        QApplication::processEvents();
        w.display_snapshot(s);
        QImage compact(w.size(), QImage::Format_RGB32);
        w.render(&compact);
        ASSERT_TRUE(compact.save(QDir(folder).filePath("sync-manual-compact.png")));
        w.resize(1440, 900);
        QApplication::processEvents();
        s.activity = Activity::Triggering;
        s.trigger_stop_pending = true;
        s.sync.operator_accepted = false;
        s.sync.attempt_completed = false;
        s.sync.verdict = Verdict::Running;
        for (int i = 0; i < 3; ++i) {
            s.cameras[i].triggers = s.sync.observed_edges[i];
            s.cameras[i].latest_timestamp_us = s.cameras[i].triggers.back().timestamp_us;
        }
        w.display_snapshot(s);
        w.render(&image);
        ASSERT_TRUE(image.save(QDir(folder).filePath("sync-continuous-running.png")));
    }
}
TEST(MultiCameraUI, SingleCameraLayoutCanBeRestored) {
    gui::MainWindow w;
    ASSERT_TRUE(QMetaObject::invokeMethod(&w, "set_multi_camera", Qt::DirectConnection, Q_ARG(bool, true)));
    EXPECT_NE(w.findChild<Workspace *>(), nullptr);
    ASSERT_TRUE(QMetaObject::invokeMethod(&w, "set_multi_camera", Qt::DirectConnection, Q_ARG(bool, false)));
    EXPECT_EQ(w.findChild<Workspace *>(), nullptr);
    w.close();
}
TEST(MultiCameraUI, PreviewOverlaysDoNotTouchDetectionInput) {
    std::array<CountWindow, 3> windows;
    for (auto &w : windows) {
        w.start_us = 100000;
        w.on = cv::Mat::zeros(720, 1280, CV_32S);
        w.off = w.on.clone();
        for (int r = 0; r < 7; ++r)
            for (int c = 0; c < 10; ++c)
                if ((r + c) % 2 == 0) {
                    cv::Rect square(240 + c * 55, 160 + r * 55, 55, 55);
                    w.on(square).setTo(100);
                    w.off(square).setTo(100);
                }
    }
    auto before = detect_group(windows);
    ASSERT_EQ(before.corners[0].size(), 54);
    auto count = windows[0].on.clone();
    Workspace ui;
    ui.resize(1280, 720);
    WorkflowSnapshot s;
    s.calibration.target_region = QRectF(.1, .1, .3, .3);
    for (auto &c : s.cameras) {
        c.width = 1280;
        c.height = 720;
        c.preview = QImage(640, 360, QImage::Format_RGB32);
        c.preview.fill(Qt::black);
        for (const auto &p : before.corners[0])
            c.corners.push_back({p.x, p.y});
    }
    ui.display_snapshot(s);
    ui.findChild<QListWidget *>("multiSteps")->setCurrentRow(2);
    QImage rendered(ui.size(), QImage::Format_RGB32);
    ui.render(&rendered);
    auto after = detect_group(windows);
    EXPECT_EQ(before.corners, after.corners);
    EXPECT_EQ(cv::norm(count, windows[0].on), 0);
}
TEST(MultiCameraUI, RenderAllWorkflowPages) {
    const auto folder = qEnvironmentVariable("EBPLUS_TEST_SCREENSHOT_DIR");
    if (folder.isEmpty())
        GTEST_SKIP() << "Set EBPLUS_TEST_SCREENSHOT_DIR to export Qt screenshots.";
    ASSERT_TRUE(QDir().mkpath(folder));
    gui::MainWindow window;
    window.resize(1440, 900);
    window.show();
    auto *action = window.findChild<QAction *>("actionMultiCamera");
    ASSERT_NE(action, nullptr);
    action->trigger();
    auto *workspace = window.findChild<Workspace *>();
    ASSERT_NE(workspace, nullptr);
    EXPECT_FALSE(workspace->isWindow());
    EXPECT_TRUE(workspace->isVisible());
    for (auto [mod, fname] : {std::pair{WorkspaceModule::Capture, "workspace-capture.png"},
                              std::pair{WorkspaceModule::Calibrate, "workspace-calibrate.png"},
                              std::pair{WorkspaceModule::Library, "workspace-library.png"}}) {
        workspace->select_workspace(mod);
        QApplication::processEvents();
        QImage ws_img(workspace->size(), QImage::Format_RGB32);
        workspace->render(&ws_img);
        ASSERT_TRUE(ws_img.save(QDir(folder).filePath(QString::fromLatin1(fname))));
    }
    workspace->select_workspace(WorkspaceModule::Calibrate);
    auto *steps = workspace->findChild<QListWidget *>("multiSteps");
    for (int i = 0; i < 5; ++i) {
        steps->setCurrentRow(i);
        QApplication::processEvents();
        QImage image(workspace->size(), QImage::Format_RGB32);
        workspace->render(&image);
        EXPECT_GT(workspace->findChild<QLineEdit *>("project")->width(), 160);
        ASSERT_TRUE(image.save(QDir(folder).filePath(QString("step-%1.png").arg(i + 1))));
    }
    window.close();
}

TEST(MultiCameraProfile, MainboardLayoutPresetsAndNominalBaselinesMatchSpec) {
    const auto geo_a = nominal_layout_geometry(RigLayoutPreset::LayoutA_Collinear);
    EXPECT_EQ(geo_a[0].hole_id, "C-100");
    EXPECT_FLOAT_EQ(geo_a[0].position_mm.x(), -100.0f);
    EXPECT_EQ(geo_a[1].hole_id, "O");
    EXPECT_EQ(geo_a[2].hole_id, "C+50");
    EXPECT_FLOAT_EQ(geo_a[2].position_mm.x(), 50.0f);
    const auto base_a = nominal_baselines_mm(RigLayoutPreset::LayoutA_Collinear);
    EXPECT_DOUBLE_EQ(base_a[0], 100.0);
    EXPECT_DOUBLE_EQ(base_a[1], 50.0);
    EXPECT_DOUBLE_EQ(base_a[2], 150.0);

    const auto geo_b = nominal_layout_geometry(RigLayoutPreset::LayoutB_LShape);
    EXPECT_EQ(geo_b[0].hole_id, "L+90");
    EXPECT_FLOAT_EQ(geo_b[0].position_mm.y(), 90.0f);
    EXPECT_EQ(geo_b[2].hole_id, "L+120");
    EXPECT_FLOAT_EQ(geo_b[2].position_mm.x(), 120.0f);
    const auto base_b = nominal_baselines_mm(RigLayoutPreset::LayoutB_LShape);
    EXPECT_DOUBLE_EQ(base_b[0], 90.0);
    EXPECT_DOUBLE_EQ(base_b[1], 120.0);
    EXPECT_DOUBLE_EQ(base_b[2], 150.0);

    const auto geo_c = nominal_layout_geometry(RigLayoutPreset::LayoutC_LowTriangle);
    EXPECT_EQ(geo_c[0].hole_id, "T-L");
    EXPECT_NEAR(geo_c[0].position_mm.y(), -93.675f, 1e-3f);
    EXPECT_EQ(geo_c[2].hole_id, "T-R");
    const auto base_c = nominal_baselines_mm(RigLayoutPreset::LayoutC_LowTriangle);
    EXPECT_DOUBLE_EQ(base_c[0], 120.0);
    EXPECT_DOUBLE_EQ(base_c[1], 120.0);
    EXPECT_DOUBLE_EQ(base_c[2], 150.0);
}

TEST(MultiCameraProfile, BaselineSanityCheckEvaluatesPassWarningAlertAndEmpty) {
    const auto empty_check = evaluate_baseline_sanity(RigLayoutPreset::LayoutA_Collinear, QJsonObject{});
    EXPECT_EQ(empty_check.verdict, BaselineSanityVerdict::Unknown);

    const auto pass_check = evaluate_baseline_sanity(
        {100.0, 50.0, 150.0}, {101.2, 49.4, 150.6});
    EXPECT_EQ(pass_check.verdict, BaselineSanityVerdict::Pass);
    EXPECT_NEAR(pass_check.max_abs_error_mm, 1.2, 1e-6);

    const auto warn_check = evaluate_baseline_sanity(
        {100.0, 50.0, 150.0}, {107.5, 50.2, 151.0});
    EXPECT_EQ(warn_check.verdict, BaselineSanityVerdict::Warning);
    EXPECT_NEAR(warn_check.max_abs_error_mm, 7.5, 1e-6);

    const auto alert_check = evaluate_baseline_sanity(
        {100.0, 50.0, 150.0}, {90.0, 120.0, 150.0});
    EXPECT_EQ(alert_check.verdict, BaselineSanityVerdict::Alert);
    EXPECT_GT(alert_check.max_abs_error_mm, 12.0);
}

TEST(MultiCameraProfile, SaveLoadScanProfileAndLegacyTakes) {
    QTemporaryDir temp;
    ASSERT_TRUE(temp.isValid());

    RigProfileRecord rec;
    rec.profile_id = "Lab_Bench_LayoutA";
    rec.display_name = "Lab Bench Layout A";
    rec.layout_preset = RigLayoutPreset::LayoutA_Collinear;
    rec.serials = {"00000327", "00000330", "00000328"};
    rec.nominal_baselines_mm = nominal_baselines_mm(rec.layout_preset);
    rec.measured_baselines_mm = {99.8, 50.1, 149.9};
    rec.focal_px = {1662.4, 1660.8, 1661.5};
    rec.rms_px = 0.34;
    rec.max_sync_delta_us = 0.8;

    QString profile_dir, err;
    ASSERT_TRUE(save_profile_to_disk(temp.path(), rec, &profile_dir, &err)) << err.toStdString();
    EXPECT_TRUE(QFileInfo::exists(QDir(profile_dir).filePath("profile.json")));
    EXPECT_TRUE(QFileInfo::exists(QDir(profile_dir).filePath("calibration.json")));

    const QString take_dir = create_take_directory(QDir(profile_dir).filePath("takes"));
    ASSERT_TRUE(write_json(QDir(take_dir).filePath("session.json"),
                           QJsonObject{{"schemaVersion", 2},
                                       {"state", "completed"},
                                       {"takeLabel", "hand_wave_01"},
                                       {"durationSeconds", 12}}));

    const QString leg_dir = create_take_directory(temp.path());
    ASSERT_TRUE(write_json(QDir(leg_dir).filePath("session.json"),
                           QJsonObject{{"schemaVersion", 2},
                                       {"state", "completed"},
                                       {"note", "legacy_run"}}));

    const auto profiles = scan_profiles_on_disk(temp.path());
    ASSERT_EQ(profiles.size(), 1);
    EXPECT_EQ(profiles[0].profile_id, "Lab_Bench_LayoutA");
    EXPECT_EQ(profiles[0].sanity_check().verdict, BaselineSanityVerdict::Pass);
    ASSERT_EQ(profiles[0].takes.size(), 1);
    EXPECT_EQ(profiles[0].takes[0].label, "hand_wave_01");

    const auto legacy = scan_legacy_takes_on_disk(temp.path());
    ASSERT_EQ(legacy.size(), 1);
    EXPECT_TRUE(legacy[0].is_legacy);
    EXPECT_EQ(legacy[0].label, "legacy_run");

    EXPECT_TRUE(delete_profile_from_disk(temp.path(), "Lab_Bench_LayoutA"));
    EXPECT_TRUE(scan_profiles_on_disk(temp.path()).isEmpty());
}

TEST(MultiCameraUI, ThreeWorkspaceModulesSeparateCaptureCalibrateAndLibrary) {
    Workspace ui;
    ui.resize(1360, 860);
    ui.show();
    QApplication::processEvents();

    ui.select_workspace(WorkspaceModule::Capture);
    QApplication::processEvents();
    EXPECT_EQ(ui.current_workspace(), WorkspaceModule::Capture);
    EXPECT_TRUE(ui.findChild<QWidget *>("multiCaptureControlBar")->isVisible());
    EXPECT_TRUE(ui.findChild<QWidget *>("multiCaptureProfileSidebar")->isVisible());
    EXPECT_FALSE(ui.findChild<QSlider *>("multiTimeline")->isVisible());
    EXPECT_FALSE(ui.findChild<QWidget *>("multiQuickAnalysisDock")->isVisible());

    auto *drawer_btn = ui.findChild<QToolButton *>("captureSyncDrawerToggleBtn");
    ASSERT_NE(drawer_btn, nullptr);
    drawer_btn->click();
    QApplication::processEvents();
    EXPECT_TRUE(ui.findChild<QWidget *>("multiCaptureSyncDrawer")->isVisible());

    ui.select_workspace(WorkspaceModule::Calibrate);
    QApplication::processEvents();
    EXPECT_EQ(ui.current_workspace(), WorkspaceModule::Calibrate);
    EXPECT_TRUE(ui.findChild<QWidget *>("multiCalibrateLeftPanel")->isVisible());
    EXPECT_FALSE(ui.findChild<QWidget *>("multiCaptureControlBar")->isVisible());
    EXPECT_FALSE(ui.findChild<QSlider *>("multiTimeline")->isVisible());

    ui.select_workspace(WorkspaceModule::Library);
    QApplication::processEvents();
    EXPECT_EQ(ui.current_workspace(), WorkspaceModule::Library);
    EXPECT_TRUE(ui.findChild<QWidget *>("multiLibrarySidebar")->isVisible());
    EXPECT_TRUE(ui.findChild<QSlider *>("multiTimeline")->isVisible());
    EXPECT_TRUE(ui.findChild<QWidget *>("multiQuickAnalysisDock")->isVisible());
    EXPECT_FALSE(ui.findChild<QWidget *>("multiCaptureControlBar")->isVisible());
    EXPECT_NE(ui.findChild<QToolButton *>("postTakeDiscardBtn"), nullptr);
}

TEST(MultiCameraProfile, RealCalibrationJsonInMetresAndInvalidRotationValidation) {
    auto make_cam = [](const QString &serial, double tx_m, double ty_m, double r00 = 1.0) {
        return QJsonObject{
            {"serial", serial},
            {"K", QJsonArray{1664.0, 0.0, 640.0, 0.0, 1662.0, 360.0, 0.0, 0.0, 1.0}},
            {"T_camera_from_reference",
             QJsonArray{r00, 0.0, 0.0, tx_m, 0.0, 1.0, 0.0, ty_m, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}},
        };
    };
    const QJsonObject real_cal{
        {"schemaVersion", 1},
        {"lengthUnit", "metre"},
        {"validationRmsPx", 0.31},
        {"cameras", QJsonArray{make_cam("00000327", 0.0, 0.0), make_cam("00000330", -0.1005, 0.0),
                               make_cam("00000328", -0.1502, 0.0)}},
    };
    const auto sanity = evaluate_baseline_sanity(RigLayoutPreset::LayoutA_Collinear, real_cal);
    EXPECT_EQ(sanity.verdict, BaselineSanityVerdict::Pass);
    EXPECT_NEAR(sanity.measured_mm[0], 100.5, 1e-3);
    EXPECT_NEAR(sanity.measured_mm[1], 49.7, 1e-3);
    EXPECT_NEAR(sanity.measured_mm[2], 150.2, 1e-3);

    const QJsonObject bad_rot_cal{
        {"schemaVersion", 1},
        {"lengthUnit", "metre"},
        {"cameras", QJsonArray{make_cam("00000327", 0.0, 0.0, 2.5), make_cam("00000330", -0.100, 0.0),
                               make_cam("00000328", -0.150, 0.0)}},
    };
    EXPECT_EQ(evaluate_baseline_sanity(RigLayoutPreset::LayoutA_Collinear, bad_rot_cal).verdict,
              BaselineSanityVerdict::Unknown);
}

TEST(MultiCameraProfile, ControllerBlocksAlertSaveAndValidatesImports) {
    QTemporaryDir temp;
    ASSERT_TRUE(temp.isValid());

    Controller ctrl;
    RigConfig cfg;
    cfg.output = temp.path();
    cfg.layout_preset = RigLayoutPreset::LayoutA_Collinear;
    ctrl.configure(cfg);

    const QString bad_json = temp.filePath("corrupt.json");
    {
        QFile f(bad_json);
        ASSERT_TRUE(f.open(QIODevice::WriteOnly));
        f.write("{ not valid json");
    }
    EXPECT_FALSE(ctrl.import_profile(bad_json));

    const QString arbitrary_json = temp.filePath("arbitrary.json");
    ASSERT_TRUE(write_json(arbitrary_json, QJsonObject{{"foo", 1}}));
    EXPECT_FALSE(ctrl.import_profile(arbitrary_json));

    RigProfileRecord src;
    src.profile_id = "Source_Profile_LayoutB";
    src.display_name = "Source Profile LayoutB";
    src.layout_preset = RigLayoutPreset::LayoutB_LShape;
    src.nominal_baselines_mm = nominal_baselines_mm(src.layout_preset);
    src.measured_baselines_mm = {90.2, 119.8, 150.1};
    src.focal_px = {1670.0, 1671.0, 1672.0};
    src.rms_px = 0.29;
    QString src_dir;
    ASSERT_TRUE(save_profile_to_disk(temp.filePath("external"), src, &src_dir));
    EXPECT_TRUE(ctrl.import_profile(QDir(src_dir).filePath("profile.json")));
    const auto imported = ctrl.list_profiles();
    ASSERT_EQ(imported.size(), 1);
    EXPECT_EQ(imported[0].sanity_check().verdict, BaselineSanityVerdict::Pass);
    EXPECT_NEAR(imported[0].focal_px[0], 1670.0, 1e-3);
}
} // namespace
int main(int argc, char **argv) {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    QApplication app(argc, argv);
    app.setOrganizationName("EBplusTests");
    app.setApplicationName("MultiCamera");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

