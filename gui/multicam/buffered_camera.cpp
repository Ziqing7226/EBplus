#include "camera_stream.h"

#include <chrono>
#include <condition_variable>
#include <deque>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace gui::multicam {
namespace {
using Clock = std::chrono::steady_clock;

// The HAL consumer owns its decoder on one thread. Only latest immutable images and sealed
// count windows cross into the coordinator; RAW buffers are never dropped by this mailbox.
class BufferedCamera final : public CameraStream {
  public:
    explicit BufferedCamera(std::unique_ptr<CameraStream> source) : source_(std::move(source)) {
        if (!source_)
            throw std::invalid_argument("Missing camera stream.");
        worker_ = std::thread([this] { run(); });
    }
    ~BufferedCamera() override {
        try {
            stop();
        } catch (...) {
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ending_ = true;
        }
        wake_.notify_one();
        worker_.join();
    }
    void open(const CameraBinding &binding, bool master) override {
        invoke([this, binding, master] {
            source_->open(binding, master);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                error_ = {};
            }
            refresh(true);
        });
    }
    void start(const QString &path) override {
        invoke([this, path] {
            source_->start(path);
            running_ = true;
            rate_start_ = Clock::now();
            rate_state_ = source_->statistics();
            published_ = {};
            refresh(true);
        });
    }
    void pump() override {
        std::exception_ptr error;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            error = error_;
        }
        if (error)
            std::rethrow_exception(error);
    }
    void stop() override {
        invoke([this] {
            running_ = false;
            counting_ = false;
            try {
                source_->stop();
            } catch (...) {
                refresh(true);
                throw;
            }
            refresh(true);
        });
    }
    CameraSnapshot snapshot() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        auto result = latest_;
        if (received_data_)
            result.decode_age_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - last_data_).count();
        return result;
    }
    CameraSnapshot statistics() const override {
        auto result = snapshot();
        result.preview = {};
        return result;
    }
    qint64 watermark() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return watermark_;
    }
    std::optional<CountWindow> window(qint64 start) const override {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = windows_.find(start);
        return it == windows_.end() ? std::nullopt : std::optional<CountWindow>(it->second);
    }
    void collect_windows(bool enabled) override {
        invoke([this, enabled] {
            source_->collect_windows(enabled);
            counting_ = enabled;
            refresh(false);
        });
    }
    void apply_tuning(const CameraTuning &tuning) override {
        invoke([this, tuning] {
            source_->apply_tuning(tuning);
            refresh(false);
        });
    }

  private:
    void invoke(std::function<void()> action) {
        auto task = std::make_shared<std::packaged_task<void()>>(std::move(action));
        auto result = task->get_future();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            commands_.push_back([task] { (*task)(); });
        }
        wake_.notify_one();
        result.get();
    }
    bool refresh(bool image) {
        auto state = image ? source_->snapshot() : source_->statistics();
        const auto now = Clock::now();
        const auto through = source_->watermark();
        std::map<qint64, CountWindow> windows;
        if (counting_ && through >= calibration_window_us) {
            const auto last = through / calibration_window_us * calibration_window_us - calibration_window_us;
            for (int i = 3; i >= 0; --i) {
                auto window = source_->window(last - i * calibration_window_us);
                if (window)
                    windows.emplace(window->start_us, std::move(*window));
            }
        }
        const double seconds = std::chrono::duration<double>(now - rate_start_).count();
        if (running_ && seconds >= 0.25) {
            event_rate_ = double(state.event_count - rate_state_.event_count) / seconds;
            byte_rate_ = double(state.bytes_consumed - rate_state_.bytes_consumed) / seconds;
            realtime_ratio_ = rate_state_.latest_timestamp_us >= 0 && state.latest_timestamp_us >= 0
                                  ? double(state.latest_timestamp_us - rate_state_.latest_timestamp_us) /
                                        (seconds * 1000000)
                                  : std::numeric_limits<double>::quiet_NaN();
            rate_state_ = state;
            rate_state_.preview = {};
            rate_start_ = now;
        }
        state.event_rate = event_rate_;
        state.byte_rate = byte_rate_;
        state.decode_realtime_ratio = realtime_ratio_;
        std::lock_guard<std::mutex> lock(mutex_);
        const bool progressed =
            state.bytes_consumed != latest_.bytes_consumed || state.event_count != latest_.event_count;
        if (progressed) {
            last_data_ = now;
            received_data_ = true;
        }
        if (!image)
            state.preview = latest_.preview;
        latest_ = std::move(state);
        watermark_ = through;
        windows_ = std::move(windows);
        if (image)
            published_ = now;
        return progressed;
    }
    void run() {
        while (true) {
            std::function<void()> command;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                if (!running_ && commands_.empty() && !ending_)
                    wake_.wait(lock, [this] { return ending_ || !commands_.empty(); });
                if (ending_)
                    break;
                if (!commands_.empty()) {
                    command = std::move(commands_.front());
                    commands_.pop_front();
                }
            }
            if (command)
                command();
            if (!running_)
                continue;
            bool progressed = false;
            try {
                source_->pump();
                // Preview sampling is independent of full-resolution calibration accumulation.
                progressed = refresh(Clock::now() - published_ >= std::chrono::milliseconds(100));
            } catch (...) {
                running_ = false;
                std::lock_guard<std::mutex> lock(mutex_);
                error_ = std::current_exception();
            }
            if (!progressed) {
                std::unique_lock<std::mutex> lock(mutex_);
                wake_.wait_for(lock, std::chrono::milliseconds(1),
                               [this] { return ending_ || !commands_.empty(); });
            }
        }
    }

    std::unique_ptr<CameraStream> source_;
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::deque<std::function<void()>> commands_;
    std::thread worker_;
    bool ending_ = false, running_ = false, counting_ = false, received_data_ = false;
    std::exception_ptr error_;
    CameraSnapshot latest_, rate_state_;
    std::map<qint64, CountWindow> windows_;
    qint64 watermark_ = -1;
    Clock::time_point published_{}, rate_start_ = Clock::now(), last_data_{};
    double event_rate_ = 0, byte_rate_ = 0;
    double realtime_ratio_ = std::numeric_limits<double>::quiet_NaN();
};
} // namespace

std::unique_ptr<CameraStream> make_buffered_camera(std::unique_ptr<CameraStream> source) {
    return std::make_unique<BufferedCamera>(std::move(source));
}
} // namespace gui::multicam
