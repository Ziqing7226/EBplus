#include "calibration/blinking_chessboard_display.h"
#include "multicam/workspace.h"
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTextDocument>
#include <QTimer>
#include <QToolButton>
#include <gtest/gtest.h>
#include <stdexcept>

using namespace gui::multicam;
namespace {
RigConfig configured() {
    RigConfig c;
    for (int i = 0; i < camera_count; ++i)
        c.cameras[i] = {QString("test:%1").arg(i), QString("serial-%1").arg(i), c.cameras[i].position};
    c.roles_confirmed = c.trigger_wiring_confirmed = true;
    c.max_delta_us = 1;
    c.max_rms_px = .7;
    c.square_mm = 25;
    c.board_locked = true;
    return c;
}
WorkflowSnapshot connected(const RigConfig &c) {
    WorkflowSnapshot s;
    s.epoch = s.sync.epoch = 3;
    s.sync.verdict = Verdict::Passed;
    s.board_ready = true;
    for (int i = 0; i < camera_count; ++i) {
        s.cameras[i].binding = c.cameras[i];
        s.cameras[i].connected = true;
    }
    return s;
}
void approve(WorkflowSnapshot &s, const RigConfig &c) {
    s.calibration.verdict = Verdict::Passed;
    s.calibration.geometry_key = c.geometry_key();
    s.calibration.training_complete = true;
    s.calibration.common_count = c.training_target;
    s.calibration.validation_count = c.validation_target;
    s.calibration.rms = .3;
}
const Blocker *find(const QVector<Blocker> &items, const QString &field) {
    for (const auto &item : items)
        if (item.field == field)
            return &item;
    return nullptr;
}

class WorkflowUI : public ::testing::Test {
  protected:
    Controller controller{
        nullptr,
        []() -> std::unique_ptr<CameraStream> { throw std::runtime_error("No hardware in workflow tests"); },
        []() -> std::unique_ptr<McuTransport> { throw std::runtime_error("No MCU in workflow tests"); },
        {},
        {},
        [] { return QVector<DeviceEntry>{}; }};
    Workspace ui{nullptr, &controller};
    void SetUp() override {
        // Only synthetic snapshots drive these tests; no queued runtime publication
        // may replace them.
        QObject::disconnect(&controller, nullptr, &ui, nullptr);
        ui.resize(1180, 740);
        ui.show();
        QApplication::processEvents();
    }
    void select(Step step) { ui.findChild<QListWidget *>("multiSteps")->setCurrentRow(int(step)); }
    void configure_ui() {
        for (int i = 0; i < camera_count; ++i) {
            auto *combo = ui.findChild<QComboBox *>(QString("camera%1").arg(i));
            QSignalBlocker block(combo);
            combo->addItem(QString("serial-%1").arg(i), QString("test:%1").arg(i));
            combo->setItemData(combo->count() - 1, QString("serial-%1").arg(i), Qt::UserRole + 1);
            combo->setCurrentIndex(combo->count() - 1);
        }
        ui.findChild<QLineEdit *>("delta")->setText("1");
        ui.findChild<QLineEdit *>("rms")->setText("0.7");
        ui.findChild<QDoubleSpinBox *>("square")->setValue(25);
        ui.findChild<QCheckBox *>("roles")->setChecked(true);
        ui.findChild<QCheckBox *>("triggerWiring")->setChecked(true);
    }
    void click_blocker(const QString &key) {
        auto *list = ui.findChild<QListWidget *>("multiBlockers");
        for (int i = 0; i < list->count(); ++i)
            if (list->item(i)->data(Qt::UserRole).toString() == key) {
                ASSERT_TRUE(QMetaObject::invokeMethod(list, "itemClicked", Qt::DirectConnection,
                                                      Q_ARG(QListWidgetItem *, list->item(i))));
                return;
            }
        FAIL() << "Missing blocker: " << key.toStdString();
    }
};

TEST(WorkflowContract, MissingDeviceFieldsAreNamedAndActionsAreNotFieldErrors) {
    RigConfig c;
    WorkflowSnapshot s;
    auto gate = stage_gate(Step::Devices, c, s);
    for (int i = 0; i < camera_count; ++i) {
        const auto *item = find(gate.pending, QString("camera%1").arg(i));
        ASSERT_NE(item, nullptr);
        EXPECT_EQ(item->kind, BlockerKind::InvalidField);
        EXPECT_TRUE(item->message.contains(c.cameras[i].position));
    }
    for (const auto &key : {"delta", "rms"}) {
        ASSERT_NE(find(gate.pending, key), nullptr);
        EXPECT_EQ(find(gate.pending, key)->kind, BlockerKind::InvalidField);
    }
    for (const auto &key : {"roles", "connect", "checkBoard"}) {
        ASSERT_NE(find(gate.pending, key), nullptr);
        EXPECT_EQ(find(gate.pending, key)->kind, BlockerKind::Action);
    }
    c = configured();
    c.cameras[2].serial = c.cameras[0].serial;
    gate = stage_gate(Step::Devices, c, s);
    EXPECT_TRUE(find(gate.pending, "camera2")->message.contains("Right"));
    EXPECT_TRUE(find(gate.pending, "camera2")->message.contains("already assigned"));
    c = configured();
    s = connected(c);
    EXPECT_TRUE(stage_gate(Step::Devices, c, s).complete);
    s.cameras[1].connected = false;
    gate = stage_gate(Step::Devices, c, s);
    ASSERT_EQ(gate.pending.size(), 1);
    EXPECT_TRUE(gate.next_action.contains("Center"));
}

TEST(WorkflowContract, CalibrationHasSevenConcretePhasesAndNeverApprovesStaleGeometry) {
    auto c = configured();
    auto s = connected(c);
    c.board_locked = false;
    c.square_mm = 0;
    GuidanceContext context;
    auto gate = stage_gate(Step::Calibration, c, s, context);
    ASSERT_EQ(gate.pending.size(), 1);
    EXPECT_EQ(gate.pending.front().field, "display");
    EXPECT_TRUE(gate.status.contains("1/7"));
    context.secondary_display_selected = true;
    EXPECT_EQ(stage_gate(Step::Calibration, c, s, context).pending.front().field, "showBoard");
    context.board_open = true;
    gate = stage_gate(Step::Calibration, c, s, context);
    EXPECT_EQ(gate.pending.front().field, "square");
    EXPECT_EQ(gate.pending.front().kind, BlockerKind::InvalidField);
    c.square_mm = 25;
    EXPECT_EQ(stage_gate(Step::Calibration, c, s, context).pending.front().field, "lockBoard");
    c.board_locked = true;
    gate = stage_gate(Step::Calibration, c, s, context);
    EXPECT_TRUE(gate.status.contains("3/7"));
    EXPECT_TRUE(gate.next_action.contains("0 / 50"));
    s.calibration.common_count = c.training_target;
    gate = stage_gate(Step::Calibration, c, s, context);
    EXPECT_EQ(gate.pending.front().field, "solve");
    EXPECT_TRUE(gate.status.contains("4/7"));
    s.calibration.training_complete = true;
    s.calibration.geometry_key = c.geometry_key();
    gate = stage_gate(Step::Calibration, c, s, context);
    EXPECT_TRUE(gate.status.contains("5/7"));
    EXPECT_TRUE(gate.next_action.contains("NEW independent"));
    EXPECT_TRUE(gate.next_action.contains("0 / 10"));
    s.calibration.validation_count = c.validation_target;
    gate = stage_gate(Step::Calibration, c, s, context);
    EXPECT_EQ(gate.pending.front().field, "validate");
    EXPECT_TRUE(gate.status.contains("6/7"));
    approve(s, c);
    gate = stage_gate(Step::Calibration, c, s, context);
    EXPECT_EQ(gate.pending.front().field, "calibration");
    EXPECT_TRUE(gate.status.contains("7/7"));
    c.optics_confirmed = true;
    EXPECT_TRUE(stage_gate(Step::Calibration, c, s, context).complete);
    c.board_locked = false;
    EXPECT_TRUE(stage_gate(Step::Calibration, c, s, context).complete); // imported validated profile
    c.board_locked = true;
    s.calibration.geometry_key = "stale";
    gate = stage_gate(Step::Calibration, c, s, context);
    EXPECT_FALSE(gate.complete);
    EXPECT_EQ(gate.pending.front().field, "resetCal");
    s.calibration.geometry_key = c.geometry_key();
    s.calibration.rms = 2;
    EXPECT_FALSE(stage_gate(Step::Calibration, c, s, context).can_advance());
    EXPECT_FALSE(blockers(Step::Capture, c, s).isEmpty());
}

TEST(WorkflowContract, InvalidConfigurationTargetsRetainTheirOwningPageAndField) {
    auto c = configured();
    c.project.clear();
    c.master = -1;
    c.reference = camera_count;
    c.training_target = 2;
    c.validation_target = 101;
    const auto pending = configuration_blockers(c);
    for (const auto &key : {"project", "master", "reference", "trainingTarget", "validationTarget"}) {
        const auto *item = find(pending, key);
        ASSERT_NE(item, nullptr);
        EXPECT_EQ(item->kind, BlockerKind::InvalidField);
        EXPECT_EQ(item->step, QString(key).endsWith("Target") ? Step::Calibration : Step::Devices);
    }
    c = configured();
    const auto s = connected(c);
    // Collection entry stays open while the UI transition still requires training/validation.
    EXPECT_TRUE(blockers(Step::Calibration, c, s).isEmpty());
    EXPECT_FALSE(stage_gate(Step::Calibration, c, s).can_advance());
}

TEST(WorkflowContract, SyncFailureOffersEvidenceReviewButStopAndEpochRemainHardGates) {
    auto c = configured();
    auto s = connected(c);
    c.trigger_wiring_confirmed = false;
    EXPECT_NE(find(stage_gate(Step::Sync, c, s).pending, "triggerWiring"), nullptr);
    c.trigger_wiring_confirmed = true;
    s.sync.verdict = Verdict::Failed;
    s.sync.attempt = 1;
    s.sync.attempt_completed = true;
    auto gate = stage_gate(Step::Sync, c, s);
    EXPECT_EQ(gate.pending.front().field, "runSync");
    for (auto &edges : s.sync.observed_edges)
        edges = {{1, 1, 0}, {2, 0, 0}};
    gate = stage_gate(Step::Sync, c, s);
    EXPECT_EQ(gate.pending.front().field, "overrideSync");
    EXPECT_TRUE(gate.next_action.contains("retained"));
    s.sync.operator_accepted = true;
    s.sync.operator_reason = "Observed correspondence";
    s.sync.operator_accepted_utc = "2026-10-20T12:00:00Z";
    EXPECT_TRUE(stage_gate(Step::Sync, c, s).complete);
    s.sync.epoch--;
    EXPECT_FALSE(stage_gate(Step::Sync, c, s).complete);
    s.sync.epoch = s.epoch;
    s.trigger_stop_pending = true;
    for (int i = 0; i < 5; ++i) {
        gate = stage_gate(Step(i), c, s);
        ASSERT_FALSE(gate.pending.isEmpty());
        EXPECT_EQ(gate.pending.front().field, "stopTrigger");
        EXPECT_TRUE(gate.next_action.contains("LOW"));
        EXPECT_FALSE(gate.complete);
        EXPECT_FALSE(gate.can_advance());
    }
    EXPECT_FALSE(s.can_override_sync());
}

TEST(WorkflowContract, RecordingReadinessAndNewTakePreflightAreDistinctFromOfflineReview) {
    auto c = configured();
    auto s = connected(c);
    approve(s, c);
    c.optics_confirmed = true;
    c.output.clear();
    c.timed = true;
    c.duration_seconds = 0;
    auto gate = stage_gate(Step::Capture, c, s);
    for (const auto &key : {"output", "duration"}) {
        ASSERT_NE(find(gate.pending, key), nullptr);
        EXPECT_EQ(find(gate.pending, key)->kind, BlockerKind::InvalidField);
    }
    c.output = "/tmp/test-takes";
    c.duration_seconds = 60;
    EXPECT_TRUE(blockers(Step::Capture, c, s).isEmpty());
    gate = stage_gate(Step::Capture, c, s);
    EXPECT_FALSE(gate.complete);
    ASSERT_EQ(gate.pending.size(), 1);
    EXPECT_EQ(gate.pending.front().field, "record");
    EXPECT_TRUE(gate.next_action.contains("fresh Trigger preflight"));
    s.activity = Activity::Preparing;
    EXPECT_EQ(stage_gate(Step::Capture, c, s).pending.front().kind, BlockerKind::Running);
    s = {};
    gate = stage_gate(Step::Review, RigConfig{}, s);
    ASSERT_EQ(gate.pending.size(), 1);
    EXPECT_EQ(gate.pending.front().field, "openTake");
    s.playback.loaded = true;
    EXPECT_TRUE(stage_gate(Step::Review, RigConfig{}, s).complete);
    s.playback.loaded = false;
    s.take_directory = "/tmp/take";
    s.take_manifest = {{"state", "incomplete"}};
    s.take_verdict = Verdict::Failed;
    gate = stage_gate(Step::Capture, RigConfig{}, s);
    EXPECT_TRUE(gate.can_advance()); // a failed closed Take must still be reviewable
    EXPECT_FALSE(gate.complete);
}

TEST(WorkflowContract, EveryBusyOperationExplainsWhyNextMustWait) {
    const auto c = configured();
    for (auto activity : {Activity::Connecting, Activity::Syncing, Activity::Triggering,
                          Activity::Calibrating, Activity::Solving, Activity::Preparing, Activity::Recording,
                          Activity::Stopping, Activity::Verifying}) {
        auto s = connected(c);
        s.activity = activity;
        for (int i = 0; i < 5; ++i) {
            const auto gate = stage_gate(Step(i), c, s);
            ASSERT_FALSE(gate.pending.isEmpty());
            EXPECT_EQ(gate.pending.front().kind, BlockerKind::Running);
            EXPECT_FALSE(gate.next_action.isEmpty());
            EXPECT_FALSE(gate.can_advance());
        }
    }
}

TEST(WorkflowContract, BusyCalibrationPreservesCompletedPrerequisitesWithoutUnlockingNext) {
    const auto c = configured();
    auto s = connected(c);
    s.activity = Activity::Calibrating;
    for (auto step : {Step::Devices, Step::Sync}) {
        const auto gate = stage_gate(step, c, s);
        EXPECT_TRUE(gate.complete);
        EXPECT_FALSE(gate.can_advance());
        ASSERT_FALSE(gate.pending.isEmpty());
        EXPECT_EQ(gate.pending.front().kind, BlockerKind::Running);
    }
    EXPECT_FALSE(stage_gate(Step::Calibration, c, s).complete);
    EXPECT_FALSE(stage_gate(Step::Capture, c, s).complete);
}

TEST_F(WorkflowUI, NextExplainsMissingInputsAndLocatesAcrossPagesWithoutRedActionBoxes) {
    auto *delta = ui.findChild<QLineEdit *>("delta");
    EXPECT_TRUE(delta->property("multiInvalid").toBool());
    EXPECT_FALSE(ui.findChild<QCheckBox *>("roles")->property("multiInvalid").toBool());
    select(Step::Capture);
    auto *next = ui.findChild<QToolButton *>("next");
    auto *reason = ui.findChild<QLabel *>("multiNextReason");
    EXPECT_FALSE(next->isEnabled());
    EXPECT_FALSE(reason->text().isEmpty());
    EXPECT_TRUE(ui.findChild<QToolButton *>("multiPendingDetails")->toolTip().contains("remaining"));
    click_blocker("delta");
    EXPECT_EQ(ui.current_step(), Step::Devices);
    EXPECT_EQ(ui.focusWidget(), delta);
    configure_ui();
    auto s = connected(ui.configuration());
    ui.display_snapshot(s);
    EXPECT_TRUE(next->isEnabled());
    next->click();
    EXPECT_EQ(ui.current_step(), Step::Sync);
    select(Step::Calibration);
    auto *list = ui.findChild<QListWidget *>("multiBlockers");
    auto *first = list->item(0);
    ui.display_snapshot(s);
    EXPECT_EQ(first, list->item(0));
    click_blocker("display");
    EXPECT_EQ(ui.focusWidget(), ui.findChild<QComboBox *>("display"));
    EXPECT_FALSE(ui.findChild<QCheckBox *>("lockBoard")->property("multiInvalid").toBool());
}

TEST_F(WorkflowUI, CalibrationCompletionUsesActualGeometryRmsAndOpticsGate) {
    configure_ui();
    auto c = ui.configuration();
    auto s = connected(c);
    approve(s, c);
    ui.display_snapshot(s);
    select(Step::Calibration);
    EXPECT_FALSE(ui.findChild<QToolButton *>("next")->isEnabled());
    click_blocker("calibration");
    ui.findChild<QCheckBox *>("calibration")->setChecked(true);
    EXPECT_TRUE(ui.findChild<QToolButton *>("next")->isEnabled());
    s.calibration.geometry_key = "stale";
    ui.display_snapshot(s);
    EXPECT_FALSE(ui.findChild<QToolButton *>("next")->isEnabled());
    auto *steps = ui.findChild<QListWidget *>("multiSteps");
    EXPECT_NE(steps->item(2)->data(Qt::UserRole).toString(), "complete");
    s.calibration.verdict = Verdict::Failed;
    ui.display_snapshot(s);
    EXPECT_TRUE(steps->item(2)->isSelected());
    EXPECT_TRUE(steps->item(2)->text().contains("Failed"));
    EXPECT_TRUE(steps->item(2)->data(Qt::UserRole + 1).toBool());
}

TEST_F(WorkflowUI, CollectingShowsLiveCountsAndRetainsCompletedStepColors) {
    configure_ui();
    auto s = connected(ui.configuration());
    s.activity = Activity::Calibrating;
    s.calibration.collecting = true;
    s.calibration.common_count = 32;
    s.calibration.stable_progress = .62;
    s.calibration.guide = "Keep the full board visible in Left, Center and Right.";
    s.calibration.reason = "Hold still for one second; the current pose is not yet accepted.";
    ui.display_snapshot(s);
    select(Step::Calibration);
    auto *steps = ui.findChild<QListWidget *>("multiSteps");
    EXPECT_EQ(steps->item(0)->data(Qt::UserRole).toString(), "complete");
    EXPECT_EQ(steps->item(1)->data(Qt::UserRole).toString(), "complete");
    EXPECT_TRUE(steps->item(2)->isSelected());
    EXPECT_EQ(steps->item(3)->data(Qt::UserRole).toString(), "pending");
    EXPECT_FALSE(steps->item(3)->text().contains("Running"));
    auto *reason = ui.findChild<QLabel *>("multiNextReason");
    EXPECT_TRUE(reason->text().contains("not yet accepted"));
    EXPECT_FALSE(ui.findChild<QToolButton *>("next")->isEnabled());
    if (const auto folder = qEnvironmentVariable("EBPLUS_TEST_SCREENSHOT_DIR"); !folder.isEmpty()) {
        QDir().mkpath(folder);
        QApplication::processEvents();
        QImage image(ui.size(), QImage::Format_RGB32);
        ui.render(&image);
        EXPECT_TRUE(image.save(QDir(folder).filePath("workflow-calibrating.png")));
    }
}

TEST_F(WorkflowUI, StopRetryIsFirstAndBusyButtonsExplainTheirDisabling) {
    auto s = WorkflowSnapshot{};
    s.trigger_stop_pending = true;
    ui.display_snapshot(s);
    auto *list = ui.findChild<QListWidget *>("multiBlockers");
    ASSERT_GT(list->count(), 0);
    EXPECT_EQ(list->item(0)->data(Qt::UserRole).toString(), "stopTrigger");
    ui.findChild<QToolButton *>("locateNext")->click();
    EXPECT_EQ(ui.current_step(), Step::Sync);
    EXPECT_EQ(ui.focusWidget(), ui.findChild<QToolButton *>("stopTrigger"));
    for (const auto &id : {"next", "record", "collect", "solve", "connect", "overrideSync", "leave"}) {
        auto *button = ui.findChild<QToolButton *>(id);
        EXPECT_FALSE(button->isEnabled());
        EXPECT_FALSE(button->toolTip().isEmpty());
    }
    s.trigger_stop_pending = false;
    s.activity = Activity::Calibrating;
    ui.display_snapshot(s);
    EXPECT_EQ(list->item(0)->data(Qt::UserRole + 2).toInt(), int(BlockerKind::Running));
    EXPECT_FALSE(ui.findChild<QCheckBox *>("roles")->property("multiInvalid").toBool());
}

TEST_F(WorkflowUI, HiddenPagesAreLazyAndUnchangedListsAndReportKeepTheirIdentity) {
    WorkflowSnapshot s;
    s.calibration.observation_labels = {"Training #1"};
    s.take_report = {{"verdict", "incomplete"}};
    ui.display_snapshot(s);
    auto *report = ui.findChild<QPlainTextEdit *>("multiReports");
    EXPECT_TRUE(report->toPlainText().isEmpty());
    select(Step::Review);
    EXPECT_TRUE(report->toPlainText().contains("incomplete"));
    const int revision = report->document()->revision();
    ui.display_snapshot(s);
    EXPECT_EQ(report->document()->revision(), revision);
    select(Step::Calibration);
    QListWidget *observations = nullptr;
    for (auto *list : ui.findChildren<QListWidget *>())
        if (list->count() == 1 && list->item(0)->text() == "Training #1")
            observations = list;
    ASSERT_NE(observations, nullptr);
    auto *item = observations->item(0);
    for (int i = 0; i < 5; ++i)
        ui.display_snapshot(s);
    EXPECT_EQ(observations->item(0), item);
}

TEST_F(WorkflowUI, CompactFiveStageGuidanceAndNextStayOutsidePageScroll) {
    const auto folder = qEnvironmentVariable("EBPLUS_TEST_SCREENSHOT_DIR");
    if (!folder.isEmpty())
        ASSERT_TRUE(QDir().mkpath(folder));
    for (int i = 0; i < 5; ++i) {
        select(Step(i));
        QApplication::processEvents();
        auto *page = ui.findChild<QWidget *>("multiPage" + QString::number(i));
        auto *scroll = qobject_cast<QScrollArea *>(page->parentWidget()->parentWidget());
        ASSERT_NE(scroll, nullptr);
        scroll->verticalScrollBar()->setValue(scroll->verticalScrollBar()->maximum());
        for (const auto &id : {"multiStageTitle", "multiStageStatus", "multiStageAction", "multiNextReason",
                               "calibrationPrimary"}) {
            auto *widget = ui.findChild<QWidget *>(id);
            ASSERT_NE(widget, nullptr);
            EXPECT_TRUE(widget->isVisible());
            EXPECT_FALSE(scroll->isAncestorOf(widget));
            const QRect bounds(widget->mapTo(&ui, QPoint{}), widget->size());
            EXPECT_TRUE(ui.rect().contains(bounds)) << id;
        }
        EXPECT_TRUE(ui.findChild<QLabel *>("multiStageTitle")->text().startsWith(QString::number(i + 1)));
        if (!folder.isEmpty()) {
            QImage image(ui.size(), QImage::Format_RGB32);
            ui.render(&image);
            EXPECT_TRUE(image.save(QDir(folder).filePath(QString("workflow-compact-%1.png").arg(i + 1))));
        }
    }
}

TEST_F(WorkflowUI, PreviewTilesKeepSensorAspectAndMaximizeOnlyPresentation) {
    auto s = connected(configured());
    for (auto &camera : s.cameras) {
        camera.width = 1280;
        camera.height = 720;
        camera.preview = QImage(640, 360, QImage::Format_RGB32);
        camera.preview.fill(Qt::darkGreen);
        camera.event_rate = 2400000;
        camera.byte_rate = 12000000;
        camera.decode_realtime_ratio = .52;
        camera.decode_age_ms = 14;
    }
    ui.display_snapshot(s);
    QApplication::processEvents();
    auto *area = ui.findChild<QWidget *>("multiPreviewArea");
    ASSERT_NE(area, nullptr);
    for (int i = 0; i < 3; ++i) {
        auto *preview = ui.findChild<QWidget *>(QString("multiPreview%1").arg(i));
        ASSERT_NE(preview, nullptr);
        const auto canvas = preview->property("multiCanvasRect").toRect();
        EXPECT_NEAR(double(canvas.width()) / canvas.height(), 1280. / 720., .025);
        EXPECT_LT(preview->height(), preview->width() * .57 + 108);
        EXPECT_TRUE(preview->toolTip().contains("2.40 Mev/s"));
        EXPECT_TRUE(preview->toolTip().contains("not USB link speed"));
        EXPECT_TRUE(preview->toolTip().contains("14 ms"));
    }
    auto *maximize = ui.findChild<QToolButton *>("multiPreviewMaximize1");
    ASSERT_NE(maximize, nullptr);
    EXPECT_TRUE(maximize->toolTip().contains("Maximize"));
    maximize->click();
    QApplication::processEvents();
    EXPECT_EQ(area->property("multiMaximizedCamera").toInt(), 1);
    EXPECT_FALSE(ui.findChild<QWidget *>("multiPreview0")->isVisible());
    EXPECT_TRUE(ui.findChild<QWidget *>("multiPreview1")->isVisible());
    EXPECT_FALSE(ui.findChild<QWidget *>("multiPreview2")->isVisible());
    EXPECT_TRUE(maximize->toolTip().contains("Restore"));
    auto *scroll = ui.findChild<QScrollArea *>("multiPreviewScroll");
    auto *expanded = ui.findChild<QWidget *>("multiPreview1");
    EXPECT_LE(expanded->height(), scroll->viewport()->height());
    EXPECT_LE(expanded->width(), scroll->viewport()->width());
    ui.resize(ui.width(), 650);
    QApplication::processEvents();
    EXPECT_LE(expanded->height(), scroll->viewport()->height());
    ui.resize(ui.width(), 740);
    QApplication::processEvents();
    s.cameras[0].event_count = 12345;
    s.cameras[2].event_count = 54321;
    ui.display_snapshot(s);
    EXPECT_TRUE(ui.findChild<QWidget *>("multiPreview0")->accessibleName().contains("12345"));
    EXPECT_TRUE(ui.findChild<QWidget *>("multiPreview2")->accessibleName().contains("54321"));
    maximize->click();
    QApplication::processEvents();
    EXPECT_EQ(area->property("multiMaximizedCamera").toInt(), -1);
    for (int i = 0; i < 3; ++i)
        EXPECT_TRUE(ui.findChild<QWidget *>(QString("multiPreview%1").arg(i))->isVisible());
    ui.resize(1600, 850);
    QApplication::processEvents();
    const int wide_columns = area->property("multiPreviewColumns").toInt();
    ui.resize(980, 740);
    QApplication::processEvents();
    EXPECT_LE(area->property("multiPreviewColumns").toInt(), wide_columns);
    EXPECT_FALSE(ui.findChild<QScrollArea *>("multiPreviewScroll")->horizontalScrollBar()->isVisible());
}

TEST_F(WorkflowUI, AllThreeNormalPreviewsFitViewportDuringActiveCalibration) {
    configure_ui();
    auto state = connected(ui.configuration());
    state.activity = Activity::Calibrating;
    state.calibration.collecting = true;
    state.calibration.capture_remaining_ms = 3200;
    state.calibration.guide = "Keep the complete board in all three camera views.";
    state.calibration.reason = "Hold still for one second; all 54 corners must be detected.";
    for (auto &camera : state.cameras) {
        camera.width = 1280;
        camera.height = 720;
        camera.preview = QImage(640, 360, QImage::Format_RGB32);
        camera.preview.fill(QColor(27, 37, 49));
    }
    ui.display_snapshot(state);
    select(Step::Calibration);
    auto *scroll = ui.findChild<QScrollArea *>("multiPreviewScroll");
    ASSERT_NE(scroll, nullptr);
    const auto folder = qEnvironmentVariable("EBPLUS_TEST_SCREENSHOT_DIR");
    for (const auto size : {QSize(1600, 900), QSize(1180, 740), QSize(980, 650)}) {
        ui.resize(size);
        QApplication::processEvents();
        ASSERT_TRUE(ui.findChild<QLabel *>("multiGuide")->isVisible());
        EXPECT_EQ(scroll->verticalScrollBar()->maximum(), 0) << size.width() << "x" << size.height();
        EXPECT_EQ(scroll->horizontalScrollBar()->maximum(), 0);
        for (int i = 0; i < camera_count; ++i) {
            auto *tile = ui.findChild<QWidget *>(QString("multiPreview%1").arg(i));
            ASSERT_TRUE(tile->isVisible());
            const QRect bounds(tile->mapTo(scroll->viewport(), QPoint{}), tile->size());
            EXPECT_TRUE(scroll->viewport()->rect().contains(bounds))
                << size.width() << "x" << size.height() << " camera " << i;
            const QRect canvas = tile->property("multiCanvasRect").toRect();
            EXPECT_NEAR(double(canvas.width()) / canvas.height(), 1280. / 720., .025);
        }
        if (!folder.isEmpty()) {
            ASSERT_TRUE(QDir().mkpath(folder));
            QImage image(ui.size(), QImage::Format_RGB32);
            ui.render(&image);
            EXPECT_TRUE(image.save(QDir(folder).filePath(
                QString("preview-fit-calibrating-%1x%2.png").arg(size.width()).arg(size.height()))));
        }
    }
}

TEST_F(WorkflowUI, GuidedSubstepsKeepNecessaryFieldsAndMainActionInViewWithoutPageScroll) {
    configure_ui();
    auto state = connected(ui.configuration());
    approve(state, ui.configuration());
    ui.findChild<QCheckBox *>("calibration")->setChecked(true);
    state.playback.loaded = false;
    state.playback.first_us = 1000000;
    state.playback.last_us = 5000000;
    state.take_directory = "/tmp/synthetic-take";
    state.take_report = {{"countsAndProtocolPass", true}};
    for (auto &camera : state.cameras) {
        camera.width = 1280;
        camera.height = 720;
        camera.preview = QImage(640, 360, QImage::Format_RGB32);
        camera.preview.fill(QColor(27, 37, 49));
    }
    ui.display_snapshot(state);
    const std::array<QVector<QStringList>, 5> controls{
        QVector<QStringList>{{"camera0", "camera1", "camera2", "master", "reference", "roles"},
                             {"project", "delta", "rms"},
                             {"multiJourneyResult0"}},
        QVector<QStringList>{
            {"triggerWiring", "multiSyncInfo"}, {"startTrigger", "stopTrigger"}, {"multiJourneyResult1"}},
        QVector<QStringList>{{"display", "boardFullscreen"},
                             {"square", "lockBoard"},
                             {"captureDuration", "multiCalibrationCounts"},
                             {"multiCalibrationCounts"},
                             {"captureDuration", "multiCalibrationCounts"},
                             {"validationView"},
                             {"calibration"}},
        QVector<QStringList>{{"multiJourneyResult3"},
                             {"output", "chooseOutput", "timedRecording", "duration"},
                             {"multiRecordStatus"}},
        QVector<QStringList>{{"lastTake"}, {"multiJourneyResult4", "includePreflight"}, {"multiReports"}}};
    const auto folder = qEnvironmentVariable("EBPLUS_TEST_SCREENSHOT_DIR");
    if (!folder.isEmpty())
        ASSERT_TRUE(QDir().mkpath(folder));
    for (int stage = 0; stage < 5; ++stage) {
        state.playback.loaded = stage == int(Step::Review);
        ui.display_snapshot(state);
        select(Step(stage));
        auto *selector = ui.findChild<QComboBox *>(QString("multiSubstep%1").arg(stage));
        ASSERT_NE(selector, nullptr);
        for (int phase = 0; phase < controls[stage].size(); ++phase) {
            selector->setCurrentIndex(phase);
            QApplication::processEvents();
            auto *page = ui.findChild<QWidget *>("multiPage" + QString::number(stage));
            auto *scroll = qobject_cast<QScrollArea *>(page->parentWidget()->parentWidget());
            ASSERT_NE(scroll, nullptr);
            EXPECT_EQ(scroll->verticalScrollBar()->maximum(), 0) << stage << ":" << phase;
            for (const auto &id : controls[stage][phase]) {
                auto *field = ui.findChild<QWidget *>(id);
                ASSERT_NE(field, nullptr) << id.toStdString();
                EXPECT_TRUE(field->isVisible()) << id.toStdString();
                const QRect bounds(field->mapTo(scroll->viewport(), QPoint{}), field->size());
                EXPECT_TRUE(scroll->viewport()->rect().contains(bounds)) << id.toStdString();
            }
            auto *primary = ui.findChild<QToolButton *>("calibrationPrimary");
            EXPECT_TRUE(primary->isVisible());
            EXPECT_TRUE(ui.rect().contains(QRect(primary->mapTo(&ui, QPoint{}), primary->size())));
            EXPECT_FALSE(ui.findChild<QToolButton *>("next")->isVisible());
            if (!folder.isEmpty()) {
                QImage image(ui.size(), QImage::Format_RGB32);
                ui.render(&image);
                EXPECT_TRUE(
                    image.save(QDir(folder).filePath(QString("guided-%1-%2.png").arg(stage).arg(phase))));
            }
        }
    }
}

TEST_F(WorkflowUI, DeviceJourneyCompletesFieldsBeforeOfferingConnectionAndLocatesReopenedFields) {
    auto *primary = ui.findChild<QToolButton *>("calibrationPrimary");
    EXPECT_EQ(primary->property("multiAction").toString(), "camera0");
    configure_ui();
    EXPECT_EQ(primary->property("multiAction").toString(), "phase:1");
    primary->click();
    EXPECT_EQ(ui.findChild<QComboBox *>("multiSubstep0")->currentIndex(), 1);
    EXPECT_TRUE(ui.findChild<QLineEdit *>("delta")->isVisible());
    EXPECT_EQ(primary->property("multiAction").toString(), "phase:2");
    primary->click();
    EXPECT_EQ(primary->property("multiAction").toString(), "connect");
    auto state = connected(ui.configuration());
    ui.display_snapshot(state);
    EXPECT_EQ(primary->property("multiAction").toString(), "next");
    primary->click();
    EXPECT_EQ(ui.current_step(), Step::Sync);
    select(Step::Capture);
    ui.findChild<QLineEdit *>("delta")->clear();
    click_blocker("delta");
    EXPECT_EQ(ui.current_step(), Step::Devices);
    EXPECT_EQ(ui.findChild<QComboBox *>("multiSubstep0")->currentIndex(), 1);
    EXPECT_EQ(ui.focusWidget(), ui.findChild<QLineEdit *>("delta"));
    select(Step::Calibration);
    auto *advanced = ui.findChild<QComboBox *>("multiSubstep2");
    advanced->setCurrentIndex(7);
    for (const auto &id :
         {"trainingTarget", "validationTarget", "captureScope", "importObs", "exportCal", "resetCal"})
        EXPECT_TRUE(ui.findChild<QWidget *>(id)->isVisible()) << id;
}

TEST_F(WorkflowUI, FreshTakePreflightKeepsEligibleManualReviewAccessibleWhilePreparing) {
    configure_ui();
    auto state = connected(ui.configuration());
    state.activity = Activity::Preparing;
    state.formal_start_us = -1;
    state.sync.verdict = Verdict::Failed;
    state.sync.attempt = 2;
    state.sync.attempt_completed = true;
    state.sync.reason = "Only the initial edge is missing; inspect retained evidence.";
    for (auto &edges : state.sync.observed_edges)
        edges = {{1000000, 1, 0}, {2000000, 0, 0}};
    ASSERT_TRUE(state.can_override_sync());
    select(Step::Sync);
    ui.display_snapshot(state);
    EXPECT_EQ(ui.findChild<QComboBox *>("multiSubstep1")->currentIndex(), 2);
    auto *primary = ui.findChild<QToolButton *>("calibrationPrimary");
    EXPECT_TRUE(primary->isVisible());
    EXPECT_TRUE(primary->isEnabled());
    EXPECT_EQ(primary->property("multiAction").toString(), "overrideSync");
    EXPECT_TRUE(primary->text().contains("accept manually"));
    EXPECT_TRUE(ui.findChild<QToolButton *>("stop")->isVisible());
    state.sync.observed_edges[0].clear();
    ui.display_snapshot(state);
    EXPECT_NE(primary->property("multiAction").toString(), "overrideSync");
}

TEST_F(WorkflowUI, LoadedTakeKeepsOpenAnotherTakeVisibleAndIdleGated) {
    WorkflowSnapshot state;
    state.playback.loaded = true;
    state.playback.first_us = 1000000;
    state.playback.last_us = 5000000;
    state.take_directory = "/tmp/loaded-take-a";
    ui.display_snapshot(state);
    select(Step::Review);
    ui.findChild<QComboBox *>("multiSubstep4")->setCurrentIndex(0);
    QApplication::processEvents();
    auto *open = ui.findChild<QToolButton *>("openTake");
    ASSERT_NE(open, nullptr);
    EXPECT_TRUE(open->isVisible());
    EXPECT_TRUE(open->isEnabled());
    EXPECT_TRUE(open->text().contains("Open another Take"));
    EXPECT_EQ(ui.findChild<QToolButton *>("calibrationPrimary")->property("multiAction").toString(),
              "phase:1");
    state.activity = Activity::Verifying;
    ui.display_snapshot(state);
    EXPECT_TRUE(open->isVisible());
    EXPECT_FALSE(open->isEnabled());
}

TEST_F(WorkflowUI, ClosedTakeKeepsNextRecordingVisibleAfterFreshRigAndSyncReadiness) {
    configure_ui();
    ui.findChild<QCheckBox *>("calibration")->setChecked(true);
    auto state = connected(ui.configuration());
    approve(state, ui.configuration());
    state.take_directory = "/tmp/closed-take-a";
    state.take_manifest = {{"state", "completed"}};
    state.take_report = {{"countsAndProtocolPass", true}};
    state.take_verdict = Verdict::Passed;
    state.cameras[0].connected = false;
    ui.display_snapshot(state);
    select(Step::Capture);
    ui.findChild<QComboBox *>("multiSubstep3")->setCurrentIndex(2);
    QApplication::processEvents();
    auto *record = ui.findChild<QToolButton *>("record");
    ASSERT_NE(record, nullptr);
    EXPECT_TRUE(record->isVisible());
    EXPECT_FALSE(record->isEnabled());
    state.cameras[0].connected = true;
    ++state.epoch;
    ui.display_snapshot(state);
    EXPECT_FALSE(record->isEnabled()); // reconnect invalidates the old synchronization evidence
    state.sync.epoch = state.epoch;
    state.sync.verdict = Verdict::Passed;
    ui.display_snapshot(state);
    EXPECT_TRUE(record->isVisible());
    EXPECT_TRUE(record->isEnabled());
    EXPECT_TRUE(record->text().contains("New preflight + record"));
    EXPECT_EQ(ui.findChild<QToolButton *>("calibrationPrimary")->property("multiAction").toString(), "next");
}

TEST_F(WorkflowUI, UnconfirmedTriggerStopOverridesStaleCalibrationGuidance) {
    configure_ui();
    auto state = connected(ui.configuration());
    state.trigger_stop_pending = true;
    state.calibration.reason = "Old reason: move the board.";
    state.calibration.capture_expired = true;
    ui.display_snapshot(state);
    select(Step::Calibration);
    auto *primary = ui.findChild<QToolButton *>("calibrationPrimary");
    EXPECT_EQ(primary->property("multiAction").toString(), "stopTrigger");
    EXPECT_TRUE(primary->text().contains("Retry Stop"));
    EXPECT_TRUE(primary->isEnabled());
    auto *reason = ui.findChild<QLabel *>("multiNextReason");
    EXPECT_TRUE(reason->isVisible());
    EXPECT_TRUE(reason->text().contains("LOW / drive"));
    EXPECT_FALSE(reason->text().contains("move the board"));
}

TEST_F(WorkflowUI, CalibrationPrimaryChecklistCountdownAndAdvancedControlsAreExplicit) {
    configure_ui();
    auto s = connected(ui.configuration());
    ui.display_snapshot(s);
    select(Step::Calibration);
    auto *primary = ui.findChild<QToolButton *>("calibrationPrimary");
    auto *checklist = ui.findChild<QListWidget *>("multiCalibrationChecklist");
    ASSERT_NE(primary, nullptr);
    ASSERT_NE(checklist, nullptr);
    EXPECT_EQ(checklist->count(), 7);
    EXPECT_EQ(primary->property("multiAction").toString(), "display");
    primary->click();
    EXPECT_EQ(ui.focusWidget(), ui.findChild<QComboBox *>("display"));
    auto *duration = ui.findChild<QSpinBox *>("captureDuration");
    ASSERT_NE(duration, nullptr);
    EXPECT_EQ(duration->minimum(), 3);
    EXPECT_EQ(duration->maximum(), 30);
    EXPECT_EQ(duration->value(), 5);
    duration->setValue(7);
    EXPECT_EQ(ui.configuration().calibration_capture_seconds, 7);
    auto *advanced = ui.findChild<QWidget *>("calibrationAdvanced");
    EXPECT_FALSE(advanced->isVisible());
    ui.findChild<QComboBox *>("multiSubstep2")->setCurrentIndex(7);
    EXPECT_TRUE(advanced->isVisible());
    for (const auto &id : {"manual", "capturePose", "removePose", "resetCal", "importCal", "importObs"})
        EXPECT_TRUE(advanced->isAncestorOf(ui.findChild<QToolButton *>(id)));
    s.activity = Activity::Calibrating;
    s.calibration.collecting = true;
    s.calibration.capture_duration_ms = 7000;
    s.calibration.capture_remaining_ms = 3200;
    s.calibration.reason = "Waiting for a common 100 ms window; decoder skew: 3700 ms.";
    ui.display_snapshot(s);
    EXPECT_EQ(primary->property("multiAction").toString(), "pauseCapture");
    EXPECT_TRUE(primary->text().contains("3.2 s"));
    EXPECT_FALSE(duration->isEnabled());
    EXPECT_TRUE(ui.findChild<QLabel *>("multiStageAction")->text().contains("decoder skew"));
    s.activity = Activity::Idle;
    s.calibration.collecting = false;
    s.calibration.capture_expired = true;
    s.calibration.capture_remaining_ms = 0;
    ui.display_snapshot(s);
    EXPECT_TRUE(ui.findChild<QLabel *>("multiCaptureStatus")->text().contains("Round ended"));
    EXPECT_NE(primary->property("multiAction").toString(), "pauseCapture");
    EXPECT_TRUE(duration->isEnabled());
    auto *storage = ui.findChild<QLabel *>("multiCalibrationStorage");
    EXPECT_TRUE(storage->text().contains("memory"));
    EXPECT_TRUE(storage->text().contains("does not record RAW"));
    EXPECT_TRUE(storage->text().contains(ui.configuration().output));
    EXPECT_NE(ui.findChild<QToolButton *>("openOutput"), nullptr);
}

TEST_F(WorkflowUI, ClearObservationsRequiresConfirmationAndNeverDeletesFiles) {
    select(Step::Calibration);
    ui.findChild<QComboBox *>("multiSubstep2")->setCurrentIndex(7);
    bool asked = false;
    QTimer timeout;
    timeout.setSingleShot(true);
    QObject::connect(&timeout, &QTimer::timeout, &ui, [] {
        if (auto *dialog = QApplication::activeModalWidget())
            dialog->close();
    });
    timeout.start(2000);
    QTimer::singleShot(0, &ui, [&] {
        auto *dialog = qobject_cast<QMessageBox *>(QApplication::activeModalWidget());
        if (dialog) {
            asked = true;
            EXPECT_TRUE(dialog->text().contains("No RAW, JSON or Take files will be deleted"));
            EXPECT_EQ(dialog->standardButton(dialog->defaultButton()), QMessageBox::No);
            dialog->reject();
        }
    });
    ui.findChild<QToolButton *>("resetCal")->click();
    timeout.stop();
    EXPECT_TRUE(asked);
}

TEST_F(WorkflowUI, EnteringCalibrationCaptureSelectsCapturePhaseWithoutStealingLaterNavigation) {
    configure_ui();
    select(Step::Calibration);
    auto state = connected(ui.configuration());
    state.activity = Activity::Calibrating;
    state.calibration.collecting = true;
    ui.display_snapshot(state);
    auto *substep = ui.findChild<QComboBox *>("multiSubstep2");
    EXPECT_EQ(substep->currentIndex(), 2);
    substep->setCurrentIndex(7);
    ui.display_snapshot(state);
    EXPECT_EQ(substep->currentIndex(), 7);
    state.activity = Activity::Idle;
    state.calibration.training_complete = true;
    ui.display_snapshot(state);
    state.activity = Activity::Calibrating;
    ui.display_snapshot(state);
    EXPECT_EQ(substep->currentIndex(), 4);
}

TEST(WorkflowPattern, LowerPlacementKeepsSquareScaleAndSingleCameraDefault) {
    gui::BlinkingChessboardDisplay board;
    board.resize(1100, 800);
    board.show();
    QApplication::processEvents();
    const QRect original = board.board_rect();
    const int square = board.square_px();
    EXPECT_DOUBLE_EQ(board.vertical_placement(), .5);
    EXPECT_LE(std::abs(original.center().y() - board.rect().center().y()), 1);
    int geometry_changes = 0;
    QObject::connect(&board, &gui::BlinkingChessboardDisplay::pattern_layout_changed,
                     [&] { ++geometry_changes; });
    board.set_vertical_placement(.65);
    const QRect lower = board.board_rect();
    EXPECT_GT(lower.top(), original.top());
    EXPECT_EQ(lower.size(), original.size());
    EXPECT_EQ(board.square_px(), square);
    EXPECT_TRUE(board.rect().contains(lower));
    EXPECT_EQ(geometry_changes, 1);
    board.set_vertical_placement(.65);
    EXPECT_EQ(geometry_changes, 1);
    EXPECT_TRUE(board.findChildren<QLabel *>().isEmpty());
}
} // namespace
