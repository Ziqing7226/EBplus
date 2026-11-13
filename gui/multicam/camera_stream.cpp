#include "camera_stream.h"

#include "algo/analytics/auto_bias_controller.h"
#include "gui/app/bias_applier.h"
#include <QElapsedTimer>
#include <QFile>
#include <QSet>
#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <metavision/hal/device/device.h>
#include <metavision/hal/device/device_discovery.h>
#include <metavision/hal/facilities/i_antiflicker_module.h>
#include <metavision/hal/facilities/i_camera_synchronization.h>
#include <metavision/hal/facilities/i_erc_module.h>
#include <metavision/hal/facilities/i_event_decoder.h>
#include <metavision/hal/facilities/i_event_trail_filter_module.h>
#include <metavision/hal/facilities/i_events_stream.h>
#include <metavision/hal/facilities/i_events_stream_decoder.h>
#include <metavision/hal/facilities/i_geometry.h>
#include <metavision/hal/facilities/i_hw_identification.h>
#include <metavision/hal/facilities/i_ll_biases.h>
#include <metavision/hal/facilities/i_plugin_software_info.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/utils/decoder_protocol_violation.h>
#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/events/event_ext_trigger.h>
#include <sstream>
#include <stdexcept>

namespace gui::multicam {
namespace {
void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}

class HalCamera final : public CameraStream {
  public:
    ~HalCamera() override {
        try {
            stop();
        } catch (...) {
        }
    }
    void open(const CameraBinding &binding, bool master) override {
        state_.binding = binding;
        device_ = Metavision::DeviceDiscovery::open(binding.selection.toStdString());
        require(bool(device_), "Unable to open the selected camera.");
        stream_ = device_->get_facility<Metavision::I_EventsStream>();
        decoder_ = device_->get_facility<Metavision::I_EventsStreamDecoder>();
        auto *cd = device_->get_facility<Metavision::I_EventDecoder<Metavision::EventCD>>();
        auto *trigger = device_->get_facility<Metavision::I_EventDecoder<Metavision::EventExtTrigger>>();
        auto *geometry = device_->get_facility<Metavision::I_Geometry>();
        auto *hardware = device_->get_facility<Metavision::I_HW_Identification>();
        auto *plugin = device_->get_facility<Metavision::I_PluginSoftwareInfo>();
        auto *sync = device_->get_facility<Metavision::I_CameraSynchronization>();
        auto *input = device_->get_facility<Metavision::I_TriggerIn>();
        biases_ = device_->get_facility<Metavision::I_LL_Biases>();
        trail_ = device_->get_facility<Metavision::I_EventTrailFilterModule>();
        afk_ = device_->get_facility<Metavision::I_AntiFlickerModule>();
        erc_ = device_->get_facility<Metavision::I_ErcModule>();
        require(stream_ && decoder_ && cd && trigger && geometry && hardware && plugin && sync && input,
                "Camera lacks a required HAL stream/sync/trigger/identity facility.");
        base_biases_.clear();
        bias_ranges_.clear();
        if (biases_) {
            try {
                for (const auto &[name, val] : biases_->get_all_biases()) {
                    base_biases_[name] = val;
                    Metavision::LL_Bias_Info info;
                    if (biases_->get_bias_info(name, info))
                        bias_ranges_[name] = info.get_bias_range();
                }
                bias_applier_.attach(biases_);
            } catch (...) {
            }
        }
        const auto serial = QString::fromStdString(hardware->get_serial());
        const auto full = QString::fromStdString(hardware->get_integrator()) + ':' +
                          QString::fromStdString(plugin->get_plugin_name()) + ':' + serial;
        require(serial == binding.serial && full == binding.selection,
                "Hardware identity does not match the selected full device identity.");
        require(QString::fromStdString(plugin->get_plugin_name()).contains("silky", Qt::CaseInsensitive),
                "This workspace requires three SilkyEvCam devices.");
        require(!decoder_->is_time_shifting_enabled(), "Timestamp shifting is forbidden for rig capture.");
        state_.width = geometry->get_width();
        state_.height = geometry->get_height();
        require(state_.width > 0 && state_.height > 0, "Invalid camera geometry.");
        events_ = std::make_unique<LiveEventBuffer>(state_.width, state_.height);
        require(master ? sync->set_mode_master() : sync->set_mode_slave(), "Cannot set clock role.");
        const auto expected = master ? Metavision::I_CameraSynchronization::SyncMode::MASTER
                                     : Metavision::I_CameraSynchronization::SyncMode::SLAVE;
        require(sync->get_mode() == expected, "Clock role readback mismatch.");
        require(input->enable(Metavision::I_TriggerIn::Channel::Main) &&
                    input->is_enabled(Metavision::I_TriggerIn::Channel::Main),
                "Trigger In readback failed.");
        int usb_speed_mbps = 0;
        QString connection_type = QStringLiteral("USB");
        try {
            connection_type = QString::fromStdString(hardware->get_connection_type());
            for (const auto &[k, v] : hardware->get_system_info()) {
                const QString key = QString::fromStdString(k);
                if (key.endsWith(QStringLiteral("Speed"), Qt::CaseInsensitive)) {
                    bool ok = false;
                    const int parsed = QString::fromStdString(v).trimmed().toInt(&ok);
                    if (ok && parsed > 0)
                        usb_speed_mbps = parsed;
                }
            }
        } catch (...) {
        }
        const QString usb_protocol =
            usb_speed_mbps >= 10000 ? QStringLiteral("USB 3.1 (10 Gbps)")
            : usb_speed_mbps >= 5000 ? QStringLiteral("USB 3.0 (5 Gbps)")
            : usb_speed_mbps == 480  ? QStringLiteral("USB 2.0 (480 Mbps)")
            : usb_speed_mbps > 0     ? QStringLiteral("USB 1.x (%1 Mbps)").arg(usb_speed_mbps)
                                     : connection_type;
        state_.metadata = {{"fullSerial", full},
                           {"serial", serial},
                           {"modeReadback", master ? "master" : "slave"},
                           {"sensor", QString::fromStdString(hardware->get_sensor_info().name_)},
                           {"plugin", QString::fromStdString(plugin->get_plugin_name())},
                           {"encoding", QString::fromStdString(hardware->get_current_data_encoding_format())},
                           {"connectionType", connection_type},
                           {"usbSpeedMbps", usb_speed_mbps},
                           {"usbProtocol", usb_protocol},
                           {"timeShiftingEnabled", false},
                           {"triggerInEnabled", true},
                           {"triggerChannel", 0}};
        refresh_bias_metadata();
        cd->add_event_buffer_callback([this](const auto *begin, const auto *end) {
            try {
                events_->consume(begin, end);
                if (auto_bias_enabled_ && begin != end && bias_applier_.attached()) {
                    std::uint32_t n_on = 0, n_off = 0;
                    for (auto *p = begin; p != end; ++p)
                        if (p->p)
                            ++n_on;
                        else
                            ++n_off;
                    const qint64 t_last = (end - 1)->t;
                    auto_bias_.accumulate(t_last, n_on, n_off);
                    if (last_auto_bias_tick_us_ < 0 ||
                        t_last - last_auto_bias_tick_us_ >= gui_algo::AutoBiasController::kTickUs) {
                        last_auto_bias_tick_us_ = t_last;
                        const auto cmd = auto_bias_.update(t_last);
                        if (cmd.active) {
                            if (cmd.home)
                                bias_applier_.home(auto_bias_.max_step());
                            else
                                bias_applier_.apply(cmd.delta_on, cmd.delta_off);
                            refresh_bias_metadata();
                        }
                    }
                }
            } catch (const std::exception &e) {
                callback_error_ = e.what();
            }
        });
        trigger->add_event_buffer_callback([this](const auto *begin, const auto *end) {
            for (auto *p = begin; p != end; ++p) {
                state_.triggers.push_back({p->t, p->p, p->id});
                if (state_.triggers.size() > 4096)
                    state_.triggers.remove(0, state_.triggers.size() - 4096);
                if (trigger_file_.isOpen()) {
                    const auto row = QByteArray::number(p->t) + ',' + QByteArray::number(p->p) + ',' +
                                     QByteArray::number(p->id) + '\n';
                    if (trigger_file_.write(row) != row.size())
                        callback_error_ = "Trigger evidence write failed.";
                }
            }
        });
        decoder_->add_protocol_violation_callback([this](auto violation) {
            ++state_.protocol_errors;
            std::ostringstream name;
            name << violation;
            state_.detail = QString::fromStdString(name.str());
            if (violation != Metavision::DecoderProtocolViolation::NonContinuousTimeHigh)
                callback_error_ = "Fatal decoder protocol violation: " + state_.detail;
        });
        state_.connected = true;
    }
    void start(const QString &path) override {
        if (!path.isEmpty()) {
            require(stream_->log_raw_data(path.toStdString()), "Cannot start RAW logging.");
            logging_ = true;
            trigger_file_.setFileName(path + ".triggers.csv");
            require(trigger_file_.open(QIODevice::WriteOnly), "Cannot create Trigger evidence.");
            require(trigger_file_.write("timestamp_us,polarity,channel\n") > 0,
                    "Cannot write Trigger header.");
        }
        stream_->start();
        running_ = true;
    }
    void pump() override {
        if (!running_)
            return;
        QElapsedTimer budget;
        budget.start();
        // Drain bursts without a per-buffer sleep, but give the other two cameras and commands a turn.
        for (int buffers = 0; buffers < 64 && (buffers == 0 || budget.elapsed() < 4); ++buffers) {
            const auto ready = stream_->poll_buffer();
            require(ready >= 0, "Camera disconnected or stream ended unexpectedly.");
            if (!ready) {
                const auto through = watermark();
                if (through >= 0 && events_->ingress_drained(through)) {
                    // Start only after the pre-request HAL backlog has been consumed. This is
                    // an ingress barrier, not a measurement of USB/sensor-to-screen latency.
                    state_.metadata.insert("calibrationIngressReady", true);
                    state_.metadata.insert("calibrationFreshAfterUs", through);
                    freshness_timestamp_ = through;
                    freshness_clock_.start();
                }
                return;
            }
            bool pulled = false;
            try {
                pulled = true;
                auto raw = stream_->get_latest_raw_data();
                require(raw && raw.size() > 0, "Empty RAW buffer.");
                decoder_->decode(raw.begin(), raw.end());
                if (!callback_error_.isEmpty())
                    throw std::runtime_error(callback_error_.toStdString());
                state_.bytes_consumed += raw.size();
            } catch (...) {
                if (logging_ && pulled)
                    state_.metadata.insert("rawMayContainUncommittedTail", true);
                throw;
            }
        }
    }
    void stop() override {
        if (!stream_)
            return;
        if (running_) {
            // OpenEB 5.2 stop closes the RAW logger and discards pending ingress.
            running_ = false;
            stream_->stop();
            logging_ = false;
        } else if (logging_) {
            stream_->stop_log_raw_data();
            logging_ = false;
        }
        if (trigger_file_.isOpen()) {
            const bool flushed = trigger_file_.flush();
            trigger_file_.close();
            require(flushed, "Unable to flush Trigger evidence.");
        }
        state_.connected = false;
    }
    CameraSnapshot snapshot() const override {
        auto result = statistics();
        if (events_)
            result.preview = events_->preview();
        return result;
    }
    CameraSnapshot statistics() const override {
        auto result = state_;
        if (freshness_clock_.isValid())
            result.metadata.insert("calibrationProgressLagMs",
                                   std::max<qint64>(0, freshness_clock_.elapsed() -
                                                           (watermark() - freshness_timestamp_) / 1000));
        if (events_) {
            const auto &stats = events_->statistics();
            result.event_count = stats.event_count;
            result.regressions = stats.regressions;
            result.first_timestamp_us = stats.first_timestamp_us;
            result.latest_timestamp_us = stats.latest_timestamp_us;
        }
        return result;
    }
    qint64 watermark() const override { return decoder_ ? decoder_->get_last_timestamp() : -1; }
    std::optional<CountWindow> window(qint64 start) const override {
        return events_ ? events_->window(start, watermark()) : std::nullopt;
    }
    void collect_windows(bool enabled) override {
        state_.metadata.insert("calibrationIngressReady", false);
        state_.metadata.insert("calibrationFreshAfterUs", -1);
        freshness_clock_.invalidate();
        if (events_)
            events_->request_collection(enabled, watermark());
    }
    void apply_tuning(const CameraTuning &tuning) override {
        auto set_bias_exact = [this](const char *exact_name, int offset) {
            if (!biases_)
                return;
            auto it_base = base_biases_.find(exact_name);
            if (it_base == base_biases_.end())
                return;
            int target = it_base->second + offset;
            if (auto it = bias_ranges_.find(exact_name);
                it != bias_ranges_.end() && it->second.second > it->second.first)
                target = std::clamp(target, it->second.first, it->second.second);
            try {
                biases_->set(exact_name, target);
            } catch (...) {
            }
        };
        set_bias_exact("bias_diff", tuning.diff_offset);
        if (tuning.auto_bias && bias_applier_.attached()) {
            const float pixels = (state_.width > 0 && state_.height > 0)
                                     ? float(state_.width) * float(state_.height)
                                     : 1280.0f * 720.0f;
            const float scale = pixels / (1280.0f * 720.0f);
            auto_bias_.set_rate_bounds(float(tuning.rate_min_mev) * scale,
                                       float(tuning.rate_max_mev) * scale);
            if (!auto_bias_enabled_) {
                auto_bias_.reset();
                last_auto_bias_tick_us_ = -1;
            }
            auto_bias_enabled_ = true;
        } else {
            auto_bias_enabled_ = false;
            auto_bias_.reset();
            last_auto_bias_tick_us_ = -1;
            set_bias_exact("bias_diff_on", tuning.diff_on_offset);
            set_bias_exact("bias_diff_off", tuning.diff_off_offset);
        }
        set_bias_exact("bias_fo", tuning.fo_offset);
        set_bias_exact("bias_hpf", tuning.hpf_offset);
        set_bias_exact("bias_refr", tuning.refr_offset);
        if (trail_) {
            try {
                if (tuning.stc_enabled) {
                    const auto avail = trail_->get_available_types();
                    const auto desired =
                        tuning.stc_type == 1 ? Metavision::I_EventTrailFilterModule::Type::STC_KEEP_TRAIL
                        : tuning.stc_type == 2 ? Metavision::I_EventTrailFilterModule::Type::TRAIL
                                               : Metavision::I_EventTrailFilterModule::Type::STC_CUT_TRAIL;
                    if (avail.count(desired))
                        trail_->set_type(desired);
                    else if (avail.count(Metavision::I_EventTrailFilterModule::Type::STC_KEEP_TRAIL))
                        trail_->set_type(Metavision::I_EventTrailFilterModule::Type::STC_KEEP_TRAIL);
                    else if (avail.count(Metavision::I_EventTrailFilterModule::Type::STC_CUT_TRAIL))
                        trail_->set_type(Metavision::I_EventTrailFilterModule::Type::STC_CUT_TRAIL);
                    else if (avail.count(Metavision::I_EventTrailFilterModule::Type::TRAIL))
                        trail_->set_type(Metavision::I_EventTrailFilterModule::Type::TRAIL);
                    trail_->set_threshold(static_cast<uint32_t>(std::max(100, tuning.stc_threshold_us)));
                }
                trail_->enable(tuning.stc_enabled);
            } catch (...) {
            }
        }
        if (afk_) {
            try {
                if (tuning.afk_enabled) {
                    const int lo = std::clamp(tuning.afk_low_hz, 50, 500);
                    const int hi = std::clamp(std::max(lo, tuning.afk_high_hz), 50, 500);
                    afk_->set_filtering_mode(Metavision::I_AntiFlickerModule::BAND_STOP);
                    afk_->set_frequency_band(lo, hi);
                }
                afk_->enable(tuning.afk_enabled);
            } catch (...) {
            }
        }
        if (erc_) {
            try {
                if (tuning.erc_enabled) {
                    const uint32_t target_ev =
                        static_cast<uint32_t>(std::clamp(tuning.erc_rate_mev * 1e6, 1e5, 1e9));
                    erc_->set_cd_event_rate(target_ev);
                }
                erc_->enable(tuning.erc_enabled);
            } catch (...) {
            }
        }
        refresh_bias_metadata();
    }

  private:
    void refresh_bias_metadata() {
        if (!biases_)
            return;
        try {
            QJsonObject b;
            for (const auto &[name, val] : biases_->get_all_biases())
                b.insert(QString::fromStdString(name), val);
            state_.metadata.insert("biases", b);
        } catch (...) {
        }
    }

    std::unique_ptr<Metavision::Device> device_;
    Metavision::I_EventsStream *stream_ = nullptr;
    Metavision::I_EventsStreamDecoder *decoder_ = nullptr;
    Metavision::I_LL_Biases *biases_ = nullptr;
    Metavision::I_EventTrailFilterModule *trail_ = nullptr;
    Metavision::I_AntiFlickerModule *afk_ = nullptr;
    Metavision::I_ErcModule *erc_ = nullptr;
    std::map<std::string, int> base_biases_;
    std::map<std::string, std::pair<int, int>> bias_ranges_;
    gui::BiasApplier bias_applier_;
    gui_algo::AutoBiasController auto_bias_{5.0f, 16.0f};
    bool auto_bias_enabled_ = false;
    qint64 last_auto_bias_tick_us_ = -1;
    CameraSnapshot state_;
    QFile trigger_file_;
    QString callback_error_;
    QElapsedTimer freshness_clock_;
    qint64 freshness_timestamp_ = -1;
    bool running_ = false, logging_ = false;
    std::unique_ptr<LiveEventBuffer> events_;
};
} // namespace

LiveEventBuffer::LiveEventBuffer(int width, int height) {
    require(width > 0 && height > 0, "Invalid camera geometry.");
    statistics_.width = width;
    statistics_.height = height;
    recent_.reserve(8192);
}

void LiveEventBuffer::consume(const Metavision::EventCD *begin, const Metavision::EventCD *end) {
    if (begin == end)
        return;
    const int width = statistics_.width, height = statistics_.height;
    qint64 bucket_start = -1, bucket_end = -1;
    int *on = nullptr, *off = nullptr;
    for (auto *p = begin; p != end; ++p) {
        if (statistics_.latest_timestamp_us >= 0 && p->t < statistics_.latest_timestamp_us) {
            ++statistics_.regressions;
            throw std::runtime_error("CD timestamps moved backwards.");
        }
        if (statistics_.first_timestamp_us < 0)
            statistics_.first_timestamp_us = p->t;
        statistics_.latest_timestamp_us = p->t;
        ++statistics_.event_count;
        require(p->x < width && p->y < height, "Out-of-bounds CD event.");
        if (!counting_)
            continue;
        require(p->t >= sealed_until_, "CD event arrived after its calibration window was sealed.");
        if (p->t < bucket_start || p->t >= bucket_end) {
            bucket_start = p->t / calibration_window_us * calibration_window_us;
            bucket_end = bucket_start + calibration_window_us;
            auto it = windows_.find(bucket_start);
            if (it == windows_.end()) {
                CountWindow w{bucket_start, cv::Mat::zeros(height, width, CV_32S),
                              cv::Mat::zeros(height, width, CV_32S)};
                it = windows_.emplace(bucket_start, std::move(w)).first;
                while (windows_.size() > 5)
                    windows_.erase(windows_.begin());
            }
            on = it->second.on.ptr<int>();
            off = it->second.off.ptr<int>();
        }
        auto &pixel = (p->p ? on : off)[size_t(p->y) * width + p->x];
        if (pixel < std::numeric_limits<int>::max())
            ++pixel;
    }
    const auto count = std::min<std::ptrdiff_t>(8192, end - begin);
    const auto retained = std::min<size_t>(recent_.size(), 8192 - size_t(count));
    if (retained && retained < recent_.size())
        std::move(recent_.end() - retained, recent_.end(), recent_.begin());
    recent_.resize(retained);
    recent_.insert(recent_.end(), end - count, end);
}

QImage LiveEventBuffer::preview() const {
    if (preview_revision_ == statistics_.event_count)
        return preview_;
    QImage image(640, 360, QImage::Format_RGB32);
    image.fill(QColor(27, 37, 49));
    auto *pixels = reinterpret_cast<QRgb *>(image.bits());
    for (const auto &p : recent_) {
        const int x = int(p.x) * 640 / statistics_.width;
        const int y = int(p.y) * 360 / statistics_.height;
        pixels[y * 640 + x] = p.p ? qRgb(220, 237, 239) : qRgb(67, 153, 197);
    }
    preview_revision_ = statistics_.event_count;
    preview_ = std::move(image);
    return preview_;
}

void LiveEventBuffer::collect_windows(bool enabled, qint64 watermark) {
    if (enabled != counting_) {
        windows_.clear();
        partial_window_ = watermark / calibration_window_us * calibration_window_us;
        sealed_until_ = -1;
    }
    counting_ = enabled;
}

void LiveEventBuffer::request_collection(bool enabled, qint64 watermark) {
    collect_windows(false, watermark);
    collection_pending_ = enabled;
}

bool LiveEventBuffer::ingress_drained(qint64 watermark) {
    if (!collection_pending_)
        return false;
    collect_windows(true, watermark);
    collection_pending_ = false;
    return true;
}

std::optional<CountWindow> LiveEventBuffer::window(qint64 start, qint64 watermark) const {
    if (start < 0 || start <= partial_window_ || watermark < start + calibration_window_us)
        return {};
    const auto it = windows_.find(start);
    if (it == windows_.end())
        return {};
    sealed_until_ = std::max(sealed_until_, start + calibration_window_us);
    // References stay alive after eviction; late writes fail instead of mutating a detector's input.
    return it->second;
}

std::unique_ptr<CameraStream> make_hal_camera() {
    return make_buffered_camera(std::make_unique<HalCamera>());
}
QVector<DeviceEntry> discover_cameras() {
    QVector<DeviceEntry> result;
    QSet<QString> seen;
    for (const auto &source : Metavision::DeviceDiscovery::list_available_sources_local()) {
        const auto full = QString::fromStdString(source.get_full_serial());
        if (!full.contains("silky", Qt::CaseInsensitive) || seen.contains(full))
            continue;
        seen.insert(full);
        const auto serial = QString::fromStdString(source.serial_);
        const auto label = serial.isEmpty() ? full : QStringLiteral("%1  (%2)").arg(serial, full);
        result.push_back({full, serial, label});
    }
    return result;
}

SyncEvidence match_trigger_edges(const std::array<QVector<TriggerEdge>, 3> &records, int master, double limit,
                                 quint64 epoch) {
    SyncEvidence out;
    out.epoch = epoch;
    out.verdict = Verdict::Failed;
    if (master < 0 || master >= 3 || !std::isfinite(limit) || limit < 0) {
        out.reason = "Invalid project synchronization condition.";
        return out;
    }
    std::array<QVector<TriggerEdge>, 3> selected;
    static const char *const names[3]{"Left", "Center", "Right"};
    int first_polarity = -1;
    for (int camera = 0; camera < 3; ++camera) {
        QVector<QVector<TriggerEdge>> candidates;
        const auto &edges = records[camera];
        int channel0 = 0;
        for (const auto &e : edges)
            channel0 += e.channel == 0;
        // The formal burst is 3 pulses at 0.5 Hz: six alternating edges ~1 s apart. The camera
        // Trigger In front-end may invert GP2, so the starting polarity is not assumed; it only
        // has to alternate and be identical on all three cameras.
        for (qsizetype first = 0; first + 5 < edges.size(); ++first) {
            QVector<TriggerEdge> candidate;
            bool ok = edges[first].channel == 0;
            const int start = edges[first].polarity;
            for (int j = 0; j < 6 && ok; ++j) {
                const auto e = edges[first + j];
                candidate.push_back(e);
                if (e.channel != 0 || e.polarity != (j % 2 == 0 ? start : 1 - start))
                    ok = false;
                if (j &&
                    std::abs(double(e.timestamp_us - edges[first + j - 1].timestamp_us) - 1000000) > 50000)
                    ok = false;
            }
            if (ok)
                candidates.push_back(candidate);
        }
        if (candidates.size() != 1) {
            const QString what =
                candidates.isEmpty()
                    ? (channel0 < 6 ? QString("only %1 of 6 expected Trigger edges were decoded").arg(channel0)
                                    : QString("%1 edges decoded but no 6 consecutive alternating edges ~1.000 s "
                                              "apart (extra glitch edges or wrong spacing)")
                                          .arg(channel0))
                    : QString("%1 equally valid 6-edge windows (ambiguous: more pulses than expected)")
                          .arg(candidates.size());
            out.reason = QString("%1 camera: %2 formal six-edge matches (%3).")
                             .arg(names[camera])
                             .arg(candidates.size())
                             .arg(what);
            return out;
        }
        const int polarity = candidates.front().front().polarity;
        if (first_polarity < 0)
            first_polarity = polarity;
        else if (polarity != first_polarity) {
            out.reason = QString("%1 camera: Trigger In polarity is opposite to the other cameras "
                                 "(check its trigger wiring / input interface).")
                             .arg(names[camera]);
            return out;
        }
        selected[camera] = candidates.front();
        out.matched[camera] = 6;
        out.extras[camera] = int(edges.size()) - 6;
    }
    out.polarity_inverted = first_polarity == 0;
    for (int camera = 0; camera < 3; ++camera)
        for (int edge = 0; edge < 6; ++edge) {
            out.delta_us[camera][edge] =
                selected[camera][edge].timestamp_us - selected[master][edge].timestamp_us;
            out.max_delta_us = std::max(
                out.max_delta_us,
                std::abs(double(selected[camera][edge].timestamp_us - selected[master][edge].timestamp_us)));
        }
    if (out.max_delta_us > limit) {
        out.reason = QString("Formal Trigger timing exceeds the project limit (max %1 us > %2 us).")
                         .arg(out.max_delta_us, 0, 'f', 1)
                         .arg(limit, 0, 'f', 1);
        return out;
    }
    out.verdict = Verdict::Passed;
    out.reason = "Six formal edges uniquely matched on each camera.";
    if (std::any_of(out.extras.begin(), out.extras.end(), [](int n) { return n > 0; }))
        out.reason += " Extra records are present; timing passed, record completeness is not certified.";
    return out;
}
} // namespace gui::multicam
