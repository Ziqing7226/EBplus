#include "multicam/camera_stream.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <mutex>
#include <thread>

using namespace gui::multicam;
namespace {
struct StreamAudit {
    std::atomic_int pumps{0}, renders{0}, stops{0};
    std::atomic_bool block{false}, fail{false};
    std::mutex mutex;
    std::thread::id consumer;
    bool wrong_thread = false;
    void visit() {
        std::lock_guard<std::mutex> lock(mutex);
        if (consumer == std::thread::id{})
            consumer = std::this_thread::get_id();
        wrong_thread |= consumer != std::this_thread::get_id();
    }
};
class AuditedStream final : public CameraStream {
  public:
    explicit AuditedStream(std::shared_ptr<StreamAudit> audit) : audit_(std::move(audit)) {}
    void open(const CameraBinding &, bool) override {
        audit_->visit();
        state_.connected = true;
        state_.width = 1280;
        state_.height = 720;
    }
    void start(const QString &) override { audit_->visit(); }
    void pump() override {
        audit_->visit();
        while (audit_->block)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (audit_->fail)
            throw std::runtime_error("decoder failure");
        ++audit_->pumps;
        state_.event_count += 100;
        state_.bytes_consumed += 200;
        state_.latest_timestamp_us += 10000;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    void stop() override {
        audit_->visit();
        ++audit_->stops;
        state_.connected = false;
    }
    CameraSnapshot snapshot() const override {
        audit_->visit();
        ++audit_->renders;
        auto out = state_;
        out.preview = QImage(16, 9, QImage::Format_RGB32);
        out.preview.fill(QColor(int(out.event_count % 255), 0, 0));
        return out;
    }
    CameraSnapshot statistics() const override {
        audit_->visit();
        return state_;
    }
    qint64 watermark() const override { return state_.latest_timestamp_us; }
    std::optional<CountWindow> window(qint64 start) const override {
        if (!counting_ || start < 0)
            return {};
        return CountWindow{start, cv::Mat::ones(2, 2, CV_32S), cv::Mat::ones(2, 2, CV_32S)};
    }
    void collect_windows(bool enabled) override {
        audit_->visit();
        counting_ = enabled;
    }

  private:
    std::shared_ptr<StreamAudit> audit_;
    CameraSnapshot state_;
    bool counting_ = false;
};
bool wait_for(const std::function<bool()> &ready) {
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!ready() && std::chrono::steady_clock::now() < until)
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    return ready();
}
auto stream(const std::shared_ptr<StreamAudit> &audit) {
    auto result = make_buffered_camera(std::make_unique<AuditedStream>(audit));
    result->open({}, false);
    result->start();
    return result;
}
} // namespace

TEST(MultiCameraConsumer, SlowCameraCannotBlockOtherConsumerOrSnapshot) {
    auto slow = std::make_shared<StreamAudit>(), fast = std::make_shared<StreamAudit>();
    slow->block = true;
    auto a = stream(slow), b = stream(fast);
    struct Unblock {
        std::shared_ptr<StreamAudit> audit;
        ~Unblock() { audit->block = false; }
    } unblock{slow};
    ASSERT_TRUE(wait_for([&] { return fast->pumps >= 20; }));
    const auto count = b->statistics().event_count;
    EXPECT_TRUE(a->snapshot().connected);
    a->pump(); // Coordinator health checks must never wait for the slow decoder.
    ASSERT_TRUE(wait_for([&] { return b->statistics().event_count > count + 1000; }));
    slow->block = false;
    a->stop();
    b->stop();
    EXPECT_FALSE(slow->wrong_thread);
    EXPECT_FALSE(fast->wrong_thread);
    EXPECT_NE(slow->consumer, fast->consumer);
    EXPECT_NE(slow->consumer, std::this_thread::get_id());
}

TEST(MultiCameraConsumer, LatestImageIsSampledAndStatisticsDoNotRender) {
    auto audit = std::make_shared<StreamAudit>();
    auto camera = stream(audit);
    const auto image = camera->snapshot().preview;
    const auto original = image.copy();
    for (int i = 0; i < 1000; ++i) {
        camera->statistics();
        camera->snapshot();
    }
    ASSERT_TRUE(wait_for([&] { return audit->pumps >= 30; }));
    EXPECT_LT(audit->renders.load(), audit->pumps.load() / 2);
    EXPECT_EQ(image, original);
    ASSERT_TRUE(wait_for([&] { return std::isfinite(camera->snapshot().decode_realtime_ratio); }));
    const auto measured = camera->snapshot();
    EXPECT_GT(measured.byte_rate, 0);
    EXPECT_TRUE(std::isfinite(measured.decode_realtime_ratio));
    EXPECT_GE(measured.decode_age_ms, 0);
    camera->stop();
    EXPECT_FALSE(audit->wrong_thread);
}

TEST(MultiCameraConsumer, SealedWindowsAreBoundedAndRetainedAfterEviction) {
    auto audit = std::make_shared<StreamAudit>();
    auto camera = stream(audit);
    camera->collect_windows(true);
    ASSERT_TRUE(wait_for([&] { return camera->watermark() >= 300000; }));
    std::optional<CountWindow> saved;
    ASSERT_TRUE(wait_for([&] {
        auto through = camera->watermark();
        saved =
            camera->window(through / calibration_window_us * calibration_window_us - calibration_window_us);
        return bool(saved);
    }));
    const auto original = saved->on.clone();
    const auto past = saved->start_us;
    ASSERT_TRUE(wait_for([&] { return camera->watermark() >= past + 1000000; }));
    EXPECT_FALSE(camera->window(past));
    EXPECT_EQ(cv::norm(saved->on, original), 0);
    camera->collect_windows(false);
    EXPECT_FALSE(camera->window(camera->watermark() / calibration_window_us * calibration_window_us -
                                calibration_window_us));
}

TEST(MultiCameraConsumer, DecoderFailureReachesCoordinatorAndStopStillWorks) {
    auto audit = std::make_shared<StreamAudit>();
    auto camera = stream(audit);
    ASSERT_TRUE(wait_for([&] { return audit->pumps >= 2; }));
    audit->fail = true;
    ASSERT_TRUE(wait_for([&] {
        try {
            camera->pump();
            return false;
        } catch (const std::runtime_error &) {
            return true;
        }
    }));
    EXPECT_NO_THROW(camera->stop());
    EXPECT_FALSE(camera->snapshot().connected);
    EXPECT_FALSE(audit->wrong_thread);
}
