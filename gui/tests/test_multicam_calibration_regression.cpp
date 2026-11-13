#include "gui/multicam/calibration.h"
#include <QElapsedTimer>
#include <QJsonArray>
#include <algorithm>
#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace gui::multicam;

RigConfig config() {
    RigConfig c;
    for (int i = 0; i < 3; ++i)
        c.cameras[i].serial = QString::number(i + 1);
    c.square_mm = 25;
    c.max_rms_px = .5;
    c.training_target = c.validation_target = 3;
    return c;
}

DetectedGroup group(qint64 time, float x = 250) {
    DetectedGroup g;
    g.start_us = time;
    for (int i = 0; i < 3; ++i) {
        g.sizes[i] = {1280, 720};
        for (int row = 0; row < 6; ++row)
            for (int col = 0; col < 9; ++col)
                g.corners[i].push_back({x + col * 45 + i * 8, 160.f + row * 45});
    }
    return g;
}

void hold(Guidance &g, qint64 start, float x = 250, int extra = -1) {
    for (int k = 0; k <= 10; ++k)
        g.observe(group(start + k * 100000, x), true, false, extra);
}

gui_algo::rig::Result result(const RigConfig &c) {
    gui_algo::rig::Result r;
    r.trained = r.validated = true;
    r.reference = c.reference;
    r.square_m = c.square_mm / 1000;
    for (int i = 0; i < 3; ++i) {
        r.models[i] = {{1280, 720},
                       (cv::Mat_<double>(3, 3) << 760, 0, 640, 0, 760, 360, 0, 0, 1),
                       cv::Mat::zeros(1, 5, CV_64F)};
        r.training_rms[i] = r.validation_rms[i] = .1;
    }
    for (int k = 0; k < c.training_target; ++k) {
        r.training_ids.push_back(k + 1);
        r.training_views.push_back({.1, .1, .1});
    }
    for (int k = 0; k < c.validation_target; ++k)
        r.validation_views.push_back({.1, .1, .1});
    return r;
}

const std::array<cv::Size, 3> sizes{{{1280, 720}, {1280, 720}, {1280, 720}}};

void camera_field(QJsonObject &profile, const QString &key, const QJsonValue &value) {
    auto cameras = profile.value("cameras").toArray();
    auto camera = cameras[0].toObject();
    camera.insert(key, value);
    cameras[0] = camera;
    profile.insert("cameras", cameras);
}

TEST(MultiCameraCalibrationRegression, RejectsMalformedProfileRmsAndPinholeMatrices) {
    const auto c = config();
    const auto profile = calibration_json(result(c), c);
    ASSERT_TRUE(read_calibration(profile, c, sizes).validated);
    for (const auto &field : {"trainingRmsPx", "validationRmsPx"}) {
        auto bad = profile;
        camera_field(bad, field, -1);
        EXPECT_THROW(read_calibration(bad, c, sizes), std::runtime_error);
    }
    for (int index : {1, 3, 6, 7, 8}) {
        auto bad = profile;
        auto k = bad.value("cameras").toArray()[0].toObject().value("K").toArray();
        k[index] = .25;
        camera_field(bad, "K", k);
        EXPECT_THROW(read_calibration(bad, c, sizes), std::runtime_error);
    }
    auto bad = profile;
    auto rows = bad.value("validationViews").toArray();
    rows[0] = QJsonArray{-.1, .1, .1};
    bad.insert("validationViews", rows);
    EXPECT_THROW(read_calibration(bad, c, sizes), std::runtime_error);
}

TEST(MultiCameraCalibrationRegression, ProfileReportsMustAgreeWithTheirCountsAndAggregates) {
    const auto c = config();
    const auto profile = calibration_json(result(c), c);
    auto bad = profile;
    camera_field(bad, "validationRmsPx", .2);
    EXPECT_THROW(read_calibration(bad, c, sizes), std::runtime_error);
    bad = profile;
    bad.insert("trainingCommonCount", std::numeric_limits<int>::max());
    EXPECT_THROW(read_calibration(bad, c, sizes), std::runtime_error);
    bad = profile;
    bad.insert("trainingViews", QJsonArray{QJsonArray{.1, .1, .1}});
    EXPECT_THROW(read_calibration(bad, c, sizes), std::runtime_error);
    bad = profile;
    bad.remove("trainingViews");
    EXPECT_THROW(read_calibration(bad, c, sizes), std::runtime_error);
    bad = profile;
    bad.insert("trainingViews", QJsonArray{});
    EXPECT_THROW(read_calibration(bad, c, sizes), std::runtime_error);
    auto invalid_config = c;
    invalid_config.max_rms_px = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(read_calibration(profile, invalid_config, sizes), std::runtime_error);
    auto invalid_sizes = sizes;
    invalid_sizes[0] = {0, 720};
    EXPECT_THROW(read_calibration(profile, c, invalid_sizes), std::runtime_error);
}

TEST(MultiCameraCalibrationRegression, InvalidCornersCannotStartAStableCountdown) {
    Guidance g;
    for (int k = 0; k <= 10; ++k) {
        auto detected = group(k * 100000);
        detected.corners[0][0].x = std::numeric_limits<float>::quiet_NaN();
        EXPECT_NO_THROW(g.observe(detected, true, true));
    }
    EXPECT_TRUE(g.training().empty());
    EXPECT_FALSE(g.stable());
    auto detected = group(2000000);
    EXPECT_NO_THROW(g.observe(detected, true, true, 3));
    EXPECT_TRUE(g.training().empty());
}

TEST(MultiCameraCalibrationRegression, ResolutionChangesCannotRelabelOldObservations) {
    Guidance g;
    hold(g, 0);
    const auto before = g.observations_json();
    for (int k = 0; k <= 10; ++k) {
        auto detected = group(2000000 + k * 100000, 350);
        detected.sizes[0] = {640, 480};
        g.observe(detected, true, false);
    }
    EXPECT_EQ(g.training().size(), 1);
    EXPECT_EQ(g.observations_json(), before);
    EXPECT_FALSE(g.stable());
}

TEST(MultiCameraCalibrationRegression, SwitchingFromSupplementalNeedsAFullCommonStableSecond) {
    Guidance g;
    for (int k = 0; k < 10; ++k) {
        auto detected = group(k * 100000);
        detected.corners[1].clear();
        detected.corners[2].clear();
        g.observe(detected, true, false, 0);
    }
    g.observe(group(1000000), true, false);
    EXPECT_TRUE(g.training().empty());
    EXPECT_EQ(g.status(config()).stable_progress, 0);
    for (int k = 11; k <= 20; ++k)
        g.observe(group(k * 100000), true, false);
    EXPECT_EQ(g.training().size(), 1);
}

TEST(MultiCameraCalibrationRegression, ReversedCornerOrderDoesNotMimicMotionOrANewPose) {
    Guidance g;
    for (int k = 0; k <= 20; ++k) {
        auto detected = group(k * 100000);
        if (k % 2)
            for (auto &points : detected.corners)
                std::reverse(points.begin(), points.end());
        g.observe(detected, true, false);
    }
    EXPECT_EQ(g.training().size(), 1);
    EXPECT_TRUE(g.status(config()).reason.contains("Repeated"));
}

TEST(MultiCameraCalibrationRegression, SlowCumulativeMotionResetsTheAnchor) {
    Guidance g;
    for (int k = 0; k <= 10; ++k)
        g.observe(group(k * 100000, 250 + .3f * k), true, true);
    EXPECT_TRUE(g.training().empty());
    EXPECT_LT(g.status(config()).stable_progress, .5);
}

TEST(MultiCameraCalibrationRegression, AddingHeldOutDataRevokesTheOldVerdict) {
    const auto c = config();
    Guidance g;
    g.set_result(result(c));
    hold(g, 2000000, 350);
    EXPECT_EQ(g.validation().size(), 1);
    EXPECT_TRUE(g.result().trained);
    EXPECT_FALSE(g.result().validated);
    EXPECT_TRUE(g.result().validation_views.empty());
    EXPECT_EQ(g.result().validation_rms[0], 0);
    EXPECT_NE(g.status(c).verdict, Verdict::Passed);
}

TEST(MultiCameraCalibrationRegression, SupplementalChangesRevokeTheFittedIntrinsics) {
    const auto c = config();
    Guidance g;
    g.set_result(result(c));
    hold(g, 2000000, 350, 0);
    EXPECT_FALSE(g.result().trained);
    EXPECT_FALSE(g.result().validated);
    EXPECT_TRUE(g.validation().empty());
    EXPECT_EQ(g.intrinsic_views()[0].size(), 1);
    EXPECT_EQ(g.status(c).common_count, 0);
}

TEST(MultiCameraCalibrationRegression, ImportingAProfileDropsUnrelatedLocalObservationBindings) {
    const auto c = config();
    Guidance g;
    hold(g, 0);
    auto imported = read_calibration(calibration_json(result(c), c), c, sizes);
    g.set_result(imported);
    EXPECT_TRUE(g.training().empty());
    EXPECT_TRUE(g.validation().empty());
    EXPECT_EQ(g.status(c).common_count, c.training_target);
    EXPECT_TRUE(g.status(c).observation_labels.isEmpty());
    g.remove_observation(0);
    EXPECT_TRUE(g.result().validated);
}

TEST(MultiCameraCalibrationRegression, DeletingHeldOutDataClearsTheWholeValidationReport) {
    const auto c = config();
    Guidance g;
    g.set_result(result(c));
    hold(g, 2000000, 350);
    auto checked = g.result();
    checked.validated = true;
    checked.validation_views = {{{.1, .1, .1}}};
    checked.validation_ids = {g.validation()[0].id};
    checked.validation_rms.fill(.1);
    g.set_result(checked);
    ASSERT_EQ(g.validation().size(), 1);
    g.remove_observation(0);
    EXPECT_FALSE(g.result().validated);
    EXPECT_TRUE(g.result().validation_views.empty());
    EXPECT_EQ(g.result().validation_rms[0], 0);
}

TEST(MultiCameraCalibrationRegression, MalformedObservationImportIsAtomicAndResolutionBound) {
    const auto c = config();
    Guidance g;
    hold(g, 0);
    const auto observations = g.observations_json();
    QJsonObject exported{{"geometryKey", c.geometry_key()}, {"observations", observations}};
    Guidance imported;
    imported.import_observations(exported, c);
    ASSERT_EQ(imported.training().size(), 1);
    auto bad = observations;
    auto rows = bad.value("training").toArray();
    auto row = rows[0].toObject();
    auto cameras = row.value("corners").toArray();
    auto points = cameras[0].toArray();
    points[0] = QJsonArray{5000, 160};
    cameras[0] = points;
    row.insert("corners", cameras);
    rows[0] = row;
    bad.insert("training", rows);
    exported.insert("observations", bad);
    EXPECT_THROW(g.import_observations(exported, c), std::runtime_error);
    EXPECT_EQ(g.observations_json(), observations);
    bad = observations;
    auto bound_sizes = bad.value("sizes").toArray();
    bound_sizes[0] = QJsonArray{1920, 1080};
    bad.insert("sizes", bound_sizes);
    exported.insert("observations", bad);
    EXPECT_THROW(g.import_observations(exported, c), std::runtime_error);
    EXPECT_EQ(g.observations_json(), observations);
}
TEST(MultiCameraCalibrationRegression, IndexedDeletionPreservesMonotonicIdsAndInvalidatesTraining) {
    const auto c = config();
    Guidance g;
    hold(g, 0, 250);
    hold(g, 2000000, 350);
    ASSERT_EQ(g.training().size(), 2);
    auto solved = result(c);
    solved.training_ids = {g.training()[0].id, g.training()[1].id};
    solved.training_observations = g.training();
    g.set_result(solved);
    hold(g, 4000000, 450);
    ASSERT_EQ(g.validation().size(), 1);
    g.remove_observation(0);
    EXPECT_FALSE(g.result().trained);
    EXPECT_TRUE(g.validation().empty());
    ASSERT_EQ(g.training().size(), 1);
    EXPECT_EQ(g.training()[0].id, 2);
    hold(g, 6000000, 550);
    ASSERT_EQ(g.training().size(), 2);
    EXPECT_EQ(g.training()[1].id, 4);
    EXPECT_EQ(g.status(c).observation_labels.size(), 2);
}
TEST(MultiCameraCalibrationRegression, IndexedSupplementalDeletionInvalidatesTheFit) {
    const auto c = config();
    Guidance g;
    hold(g, 0, 250);
    hold(g, 2000000, 350, 2);
    auto solved = result(c);
    solved.training_ids = {g.training()[0].id};
    solved.training_observations = g.training();
    g.set_result(solved);
    ASSERT_EQ(g.status(c).observation_labels.size(), 2);
    g.remove_observation(1);
    EXPECT_FALSE(g.result().trained);
    EXPECT_TRUE(g.validation().empty());
    EXPECT_EQ(g.training().size(), 1);
    EXPECT_EQ(g.intrinsic_views()[2].size(), 1);
    EXPECT_EQ(g.status(c).observation_labels.size(), 1);
}
TEST(MultiCameraCalibrationRegression, ProfileRoundTripPreservesTrainingCornerExclusion) {
    const auto c = config();
    auto solved = result(c);
    const auto training = group(0);
    for (int i = 0; i < 3; ++i)
        solved.intrinsic_training_views[i].push_back(training.corners[i]);
    const auto imported = read_calibration(calibration_json(solved, c), c, sizes);
    EXPECT_EQ(imported.intrinsic_training_views, solved.intrinsic_training_views);
    Guidance g;
    g.set_result(imported);
    hold(g, 2000000);
    EXPECT_TRUE(g.validation().empty());
    hold(g, 4000000, 350);
    EXPECT_EQ(g.validation().size(), 1);
    EXPECT_FALSE(g.result().validated);
    const auto reloaded = read_calibration(calibration_json(imported, c), c, sizes);
    EXPECT_EQ(reloaded.intrinsic_training_views, solved.intrinsic_training_views);
}
TEST(MultiCameraCalibrationRegression, EmbeddedObservationsMustMatchTheProfileResolution) {
    const auto c = config();
    Guidance g;
    hold(g, 0, 250);
    hold(g, 2000000, 350);
    hold(g, 4000000, 450);
    auto profile = calibration_json(result(c), c);
    profile.insert("observations", g.observations_json());
    EXPECT_TRUE(read_calibration(profile, c, sizes).validated);
    auto observations = g.observations_json();
    auto bound_sizes = observations.value("sizes").toArray();
    bound_sizes[0] = QJsonArray{1920, 1080};
    observations.insert("sizes", bound_sizes);
    profile.insert("observations", observations);
    EXPECT_THROW(read_calibration(profile, c, sizes), std::runtime_error);
}
TEST(MultiCameraCalibrationRegression, StabilityToleranceScalesWithSensorResolution) {
    Guidance g;
    for (int k = 0; k <= 10; ++k) {
        auto detected = group(k * 100000);
        for (int i = 0; i < 3; ++i) {
            detected.sizes[i] = {640, 360};
            for (auto &p : detected.corners[i]) {
                p *= .5f;
                if (k > 0)
                    p.x += 1.2f;
            }
        }
        g.observe(detected, true, false);
    }
    EXPECT_TRUE(g.training().empty());
    EXPECT_NEAR(g.status(config()).stable_progress, .9, 1e-6);
}
TEST(MultiCameraCalibrationRegression, SupplementalGuideDoesNotRequireOtherCameraDetections) {
    Guidance g;
    auto detected = group(0);
    detected.corners[1].clear();
    detected.corners[2].clear();
    g.observe(detected, false, false, 0);
    EXPECT_EQ(g.status(config()).target_camera, 0);
    EXPECT_FALSE(g.status(config()).guide.contains("restore"));
    detected.start_us = 100000;
    detected.corners[0].clear();
    g.observe(detected, false, false, 0);
    EXPECT_TRUE(g.status(config()).guide.contains("Camera 1"));
}

std::vector<Metavision::EventCD> event_load(int count, qint64 start = 100000) {
    std::vector<Metavision::EventCD> events;
    events.reserve(count);
    for (int i = 0; i < count; ++i)
        events.emplace_back((i * 37) % 1280, (i * 13) % 720, i % 2, start + i / 10);
    return events;
}

TEST(MultiCameraPerformance, FullResolutionCountsAreIndependentOfBoundedPreview) {
    LiveEventBuffer buffer(1280, 720);
    buffer.collect_windows(true, 0);
    const auto events = event_load(1000000);
    buffer.consume(events.data(), events.data() + events.size());
    const auto window = buffer.window(100000, 200000);
    ASSERT_TRUE(window);
    cv::Mat on = cv::Mat::zeros(720, 1280, CV_32S), off = on.clone();
    for (const auto &event : events)
        ++(event.p ? on : off).at<int>(event.y, event.x);
    EXPECT_EQ(cv::norm(on, window->on, cv::NORM_INF), 0);
    EXPECT_EQ(cv::norm(off, window->off, cv::NORM_INF), 0);
    EXPECT_EQ(buffer.statistics().event_count, events.size());
    EXPECT_EQ(buffer.statistics().first_timestamp_us, events.front().t);
    EXPECT_EQ(buffer.statistics().latest_timestamp_us, events.back().t);
    QImage expected(640, 360, QImage::Format_RGB32);
    expected.fill(QColor(27, 37, 49));
    for (auto event = events.end() - 8192; event != events.end(); ++event)
        expected.setPixel(event->x / 2, event->y / 2, event->p ? qRgb(220, 237, 239) : qRgb(67, 153, 197));
    EXPECT_EQ(buffer.preview(), expected);
}

TEST(MultiCameraPerformance, PreviewIsCachedAndPublishedImagesRemainImmutable) {
    LiveEventBuffer buffer(1280, 720);
    auto events = event_load(64);
    buffer.consume(events.data(), events.data() + events.size());
    const auto before = buffer.preview();
    const auto pixels = before.copy();
    EXPECT_EQ(before.cacheKey(), buffer.preview().cacheKey());
    events = event_load(20000, 200000);
    buffer.consume(events.data(), events.data() + events.size());
    EXPECT_NE(before.cacheKey(), buffer.preview().cacheKey());
    EXPECT_EQ(before, pixels);
    EXPECT_FALSE(buffer.window(100000, 200000));
}

TEST(MultiCameraPerformance, CompletedWindowsShareStorageAndSurviveBoundedRetention) {
    LiveEventBuffer buffer(1280, 720);
    buffer.collect_windows(true, 0);
    const auto first = event_load(200, 100000);
    buffer.consume(first.data(), first.data() + first.size());
    const auto window = buffer.window(100000, 200000);
    ASSERT_TRUE(window);
    const auto again = buffer.window(100000, 200000);
    ASSERT_TRUE(again);
    EXPECT_EQ(window->on.data, again->on.data);
    const auto original = window->on.clone();
    for (int i = 2; i < 9; ++i) {
        const auto next = event_load(100, i * 100000);
        buffer.consume(next.data(), next.data() + next.size());
    }
    EXPECT_FALSE(buffer.window(100000, 900000));
    EXPECT_TRUE(buffer.window(800000, 900000));
    EXPECT_EQ(cv::norm(window->on, original, cv::NORM_INF), 0);
    buffer.collect_windows(false, 900000);
    EXPECT_FALSE(buffer.window(800000, 900000));
    EXPECT_EQ(cv::norm(window->on, original, cv::NORM_INF), 0);
}

TEST(MultiCameraPerformance, FourCompletedWindowsRemainAvailableWithThreeHundredMsCameraSkew) {
    for (const qint64 skew : {200000, 299999, 300000}) {
        LiveEventBuffer fast(1280, 720), slow(1280, 720);
        fast.collect_windows(true, 0);
        slow.collect_windows(true, 0);
        const qint64 fast_time = 1000001, slow_time = fast_time - skew;
        const qint64 common =
            slow_time / calibration_window_us * calibration_window_us - calibration_window_us;
        for (qint64 t = 100000; t <= fast_time; t += 100000) {
            auto events = event_load(10, t);
            fast.consume(events.data(), events.data() + events.size());
            if (t <= slow_time)
                slow.consume(events.data(), events.data() + events.size());
        }
        const auto a = fast.window(common, fast_time);
        const auto b = slow.window(common, slow_time);
        ASSERT_TRUE(a) << "skew=" << skew;
        ASSERT_TRUE(b) << "skew=" << skew;
        EXPECT_EQ(a->start_us, b->start_us);
        EXPECT_EQ(cv::norm(a->on, b->on, cv::NORM_INF), 0);
        EXPECT_FALSE(fast.window(100000, fast_time));
    }
}

TEST(MultiCameraPerformance, CaptureIngressBarrierExcludesBacklogAndItsPartialWindow) {
    LiveEventBuffer buffer(1280, 720);
    buffer.request_collection(true, 0);
    for (qint64 t : {100000, 200000, 300000}) {
        const auto queued = event_load(100, t);
        buffer.consume(queued.data(), queued.data() + queued.size());
        EXPECT_FALSE(buffer.window(t, t + calibration_window_us));
    }
    EXPECT_EQ(buffer.statistics().event_count, 300);
    ASSERT_TRUE(buffer.ingress_drained(350000));
    EXPECT_FALSE(buffer.ingress_drained(360000));
    for (qint64 t : {350000, 400000}) {
        const auto current = event_load(100, t);
        buffer.consume(current.data(), current.data() + current.size());
    }
    EXPECT_FALSE(buffer.window(300000, 500000));
    const auto fresh = buffer.window(400000, 500000);
    ASSERT_TRUE(fresh);
    EXPECT_EQ(cv::sum(fresh->on)[0] + cv::sum(fresh->off)[0], 100);
    buffer.request_collection(true, 500000);
    EXPECT_FALSE(buffer.window(400000, 600000));
    buffer.request_collection(false, 600000);
    EXPECT_FALSE(buffer.ingress_drained(700000));
    EXPECT_EQ(buffer.statistics().event_count, 500);
}

TEST(MultiCameraPerformance, InvalidOrLateEventsCannotMutateSealedDetectionInput) {
    LiveEventBuffer buffer(1280, 720);
    buffer.collect_windows(true, 0);
    const auto events = event_load(200);
    buffer.consume(events.data(), events.data() + events.size());
    const auto window = buffer.window(100000, 200000);
    ASSERT_TRUE(window);
    const double count = cv::sum(window->on)[0] + cv::sum(window->off)[0];
    Metavision::EventCD late(1, 1, 1, 150000);
    EXPECT_THROW(buffer.consume(&late, &late + 1), std::runtime_error);
    EXPECT_EQ(cv::sum(window->on)[0] + cv::sum(window->off)[0], count);
    Metavision::EventCD backwards(1, 1, 1, 1);
    EXPECT_THROW(buffer.consume(&backwards, &backwards + 1), std::runtime_error);
    EXPECT_EQ(buffer.statistics().regressions, 1);
    LiveEventBuffer bad_coordinates(1280, 720);
    Metavision::EventCD outside(1280, 0, 1, 100000);
    EXPECT_THROW(bad_coordinates.consume(&outside, &outside + 1), std::runtime_error);
}

TEST(MultiCameraPerformance, CachedCalibrationStatusTracksObservationResultAndConfigChanges) {
    const auto c = config();
    Guidance guidance;
    EXPECT_EQ(guidance.status(c).common_count, 0);
    hold(guidance, 0);
    EXPECT_EQ(guidance.status(c).common_count, 1);
    EXPECT_EQ(guidance.status(c).observation_labels.size(), 1);
    guidance.remove_last();
    EXPECT_EQ(guidance.status(c).common_count, 0);
    auto r = result(c);
    guidance.set_result(r);
    EXPECT_EQ(guidance.status(c).profile, calibration_json(r, c));
    auto changed = c;
    changed.optics_confirmed = !c.optics_confirmed;
    EXPECT_EQ(guidance.status(changed).profile, calibration_json(r, changed));
    changed.max_rms_px = .01;
    EXPECT_EQ(guidance.status(changed).verdict, Verdict::Failed);
    r.error = "injected solver failure";
    guidance.set_result(r);
    EXPECT_EQ(guidance.status(c).reason, QString::fromStdString(r.error));
    guidance.reset();
    EXPECT_TRUE(guidance.status(c).profile.isEmpty());
}

TEST(MultiCameraCalibrationRegression, PauseKeepsObservationsButRestartsTheOneSecondStabilityGate) {
    Guidance guidance;
    for (int i = 0; i < 10; ++i)
        guidance.observe(group(i * 100000), true, false);
    EXPECT_NEAR(guidance.status(config()).stable_progress, .9, 1e-6);
    guidance.pause();
    EXPECT_EQ(guidance.status(config()).stable_progress, 0);
    guidance.observe(group(1000000), true, false);
    EXPECT_TRUE(guidance.training().empty());
    EXPECT_EQ(guidance.status(config()).stable_progress, 0);
    hold(guidance, 1100000);
    EXPECT_EQ(guidance.training().size(), 1);
    guidance.pause();
    EXPECT_EQ(guidance.training().size(), 1);
}

TEST(MultiCameraPerformance, SyntheticHotPathBenchmarkRetainsExactEventTotals) {
    const auto events = event_load(1000000);
    std::vector<double> legacy_ms, buffered_ms;
    auto legacy = [&] {
        CameraSnapshot stats;
        std::map<qint64, CountWindow> windows;
        for (const auto &event : events) {
            stats.latest_timestamp_us = event.t;
            ++stats.event_count;
            auto bucket = event.t / calibration_window_us * calibration_window_us;
            auto it = windows.find(bucket);
            if (it == windows.end())
                it = windows
                         .emplace(bucket, CountWindow{bucket, cv::Mat::zeros(720, 1280, CV_32S),
                                                      cv::Mat::zeros(720, 1280, CV_32S)})
                         .first;
            auto &pixel = (event.p ? it->second.on : it->second.off).at<int>(event.y, event.x);
            if (pixel < std::numeric_limits<int>::max())
                ++pixel;
        }
        EXPECT_EQ(stats.event_count, events.size());
    };
    auto buffered = [&] {
        LiveEventBuffer buffer(1280, 720);
        buffer.collect_windows(true, 0);
        buffer.consume(events.data(), events.data() + events.size());
        EXPECT_EQ(buffer.statistics().event_count, events.size());
    };
    for (int round = 0; round < 7; ++round) {
        auto measure = [](auto work) {
            QElapsedTimer timer;
            timer.start();
            work();
            return timer.nsecsElapsed() / 1e6;
        };
        if (round % 2) {
            buffered_ms.push_back(measure(buffered));
            legacy_ms.push_back(measure(legacy));
        } else {
            legacy_ms.push_back(measure(legacy));
            buffered_ms.push_back(measure(buffered));
        }
    }
    std::sort(legacy_ms.begin(), legacy_ms.end());
    std::sort(buffered_ms.begin(), buffered_ms.end());
    std::cout << "[BENCH] 1M full-resolution events: reference median=" << legacy_ms[3]
              << " ms; buffered median=" << buffered_ms[3] << " ms\n";
    const auto c = config();
    auto r = result(c);
    for (int camera = 0; camera < 3; ++camera)
        for (int view = 0; view < 50; ++view)
            r.intrinsic_training_views[camera].push_back(group(0).corners[camera]);
    Guidance guidance;
    guidance.set_result(r);
    guidance.status(c);
    QElapsedTimer timer;
    timer.start();
    for (int i = 0; i < 100; ++i)
        EXPECT_FALSE(calibration_json(r, c).isEmpty());
    const double uncached = timer.nsecsElapsed() / 1e6;
    timer.restart();
    for (int i = 0; i < 100; ++i)
        EXPECT_EQ(guidance.status(c).common_count, c.training_target);
    std::cout << "[BENCH] 100 report reads: serialize=" << uncached
              << " ms; cached status=" << timer.nsecsElapsed() / 1e6 << " ms\n";
}
} // namespace
