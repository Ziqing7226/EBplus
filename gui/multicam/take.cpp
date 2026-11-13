#include "take.h"
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>
#include <QSet>
#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <future>
#include <metavision/hal/device/device.h>
#include <metavision/hal/device/device_discovery.h>
#include <metavision/hal/facilities/i_event_decoder.h>
#include <metavision/hal/facilities/i_events_stream.h>
#include <metavision/hal/facilities/i_events_stream_decoder.h>
#include <metavision/hal/facilities/i_geometry.h>
#include <metavision/hal/utils/hal_exception.h>
#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/events/event_ext_trigger.h>
#include <stdexcept>
#include <thread>
#include <vector>

namespace gui::multicam {
namespace {
void fail(const QString &s) {
    throw std::runtime_error(s.toStdString());
}
class HalReader final : public RawReader {
  public:
    explicit HalReader(const TakeCamera &c) : camera_(c) {
        open();
    }
    ~HalReader() override {
        try {
            if (stream_)
                stream_->stop();
        } catch (...) {
        }
    }
    void open() {
        if (stream_)
            stream_->stop();
        stream_ = nullptr;
        decoder_ = nullptr;
        device_.reset();
        Metavision::RawFileConfig config;
        config.do_time_shifting_ = false;
        config.build_index_ = false;
        config.n_events_to_read_ = 65536;
        config.n_read_buffers_ = 8;
        device_ = Metavision::DeviceDiscovery::open_raw_file(camera_.path.toStdString(), config);
        if (!device_)
            fail("Cannot open RAW: " + camera_.path);
        stream_ = device_->get_facility<Metavision::I_EventsStream>();
        decoder_ = device_->get_facility<Metavision::I_EventsStreamDecoder>();
        auto *cd = device_->get_facility<Metavision::I_EventDecoder<Metavision::EventCD>>();
        auto *geo = device_->get_facility<Metavision::I_Geometry>();
        if (!stream_ || !decoder_ || !cd || !geo || decoder_->is_time_shifting_enabled())
            fail("RAW lacks unshifted CD/geometry facilities.");
        state_ = {};
        state_.width = geo->get_width();
        state_.height = geo->get_height();
        state_.binding.serial = camera_.serial;
        state_.binding.position = camera_.position;
        if (state_.width <= 0 || state_.height <= 0)
            fail("Invalid RAW geometry.");
        cd->add_event_buffer_callback([this](const auto *first, const auto *last) {
            if (scan_only_) {
                for (auto *p = first; p != last; ++p) {
                    if (p->x >= state_.width || p->y >= state_.height)
                        ++state_.protocol_errors;
                    if (last_decoded_ >= 0 && p->t < last_decoded_)
                        ++state_.regressions;
                    last_decoded_ = p->t;
                    if (state_.first_timestamp_us < 0)
                        state_.first_timestamp_us = p->t;
                    ++state_.event_count;
                    state_.latest_timestamp_us = p->t;
                }
                return;
            }
            if (decoded_.size() + size_t(last - first) > 1048576)
                fail("A decoded RAW buffer exceeds the safety bound.");
            for (auto *p = first; p != last; ++p) {
                if (p->x >= state_.width || p->y >= state_.height)
                    ++state_.protocol_errors;
                if (last_decoded_ >= 0 && p->t < last_decoded_)
                    ++state_.regressions;
                last_decoded_ = p->t;
                if (state_.first_timestamp_us < 0)
                    state_.first_timestamp_us = p->t;
            }
            decoded_.insert(decoded_.end(), first, last);
        });
        if (auto *ext = device_->get_facility<Metavision::I_EventDecoder<Metavision::EventExtTrigger>>())
            ext->add_event_buffer_callback([this](const auto *first, const auto *last) {
                for (auto *p = first; p != last; ++p) {
                    state_.triggers.push_back({p->t, p->p, p->id});
                    if (state_.triggers.size() > 4096)
                        state_.triggers.removeFirst();
                    ++trigger_count_;
                }
            });
        state_.metadata.insert("protocolValidationAvailable", false);
        try {
            decoder_->add_protocol_violation_callback([this](auto) { ++state_.protocol_errors; });
            state_.metadata.insert("protocolValidationAvailable", true);
        } catch (const Metavision::HalException &e) {
            if (e.code().value() != Metavision::HalErrorCode::OperationNotImplemented)
                throw;
        }
        eof_ = false;
        cursor_ = 0;
        through_ = -1;
        target_ = -1;
        last_decoded_ = -1;
        trigger_count_ = 0;
        decoded_.clear();
        recent_.clear();
        stream_->start();
    }
    CameraSnapshot frame(qint64 start, qint64 target, const std::function<bool()> &cancelled) override {
        if (!scan_only_ && target < std::numeric_limits<qint64>::max() && try_init_evt3_mmap())
            return frame_evt3_mmap(start, target, cancelled);
        if (target < target_ || start < start_)
            open();
        target_ = target;
        start_ = start;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::minutes(30);
        for (;;) {
            if (cancelled())
                fail("RAW read cancelled.");
            if (std::chrono::steady_clock::now() > deadline)
                fail("RAW read timed out.");
            while (cursor_ < decoded_.size() && decoded_[cursor_].t <= target) {
                const auto &p = decoded_[cursor_++];
                ++state_.event_count;
                state_.latest_timestamp_us = p.t;
                if (p.t >= start) {
                    recent_.push_back(p);
                    if (recent_.size() > 8192)
                        recent_.pop_front();
                }
            }
            if (cursor_ < decoded_.size() || through_ > target || eof_)
                break;
            const auto available = stream_->poll_buffer();
            if (available < 0) {
                eof_ = true;
                break;
            }
            if (!available) {
                if (stream_->wait_next_buffer() < 0) {
                    eof_ = true;
                    break;
                }
                continue;
            }
            decoded_.clear();
            cursor_ = 0;
            const auto raw = stream_->get_latest_raw_data();
            if (!raw || raw.size() == 0)
                fail("Empty RAW buffer.");
            decoder_->decode(raw.begin(), raw.end());
            state_.bytes_consumed += raw.size();
            through_ = decoder_->get_last_timestamp();
        }
        while (!recent_.empty() && recent_.front().t < start)
            recent_.pop_front();
        auto out = state_;
        out.connected = false;
        out.preview = QImage(640, 360, QImage::Format_RGB32);
        out.preview.fill(QColor(27, 37, 49));
        for (const auto &p : recent_)
            if (p.x < out.width && p.y < out.height)
                out.preview.setPixel(int(p.x) * 640 / out.width, int(p.y) * 360 / out.height,
                                     p.p ? qRgb(220, 237, 239) : qRgb(67, 153, 197));
        out.detail = "Offline RAW; original shared timestamps";
        return out;
    }
    CameraSnapshot scan(const std::function<bool()> &cancelled) override {
        scan_only_ = true;
        if (target_ >= 0)
            open();
        frame(std::numeric_limits<qint64>::max(), std::numeric_limits<qint64>::max(), cancelled);
        scan_only_ = false;
        auto out = state_;
        out.metadata.insert("externalTriggerCount", qint64(trigger_count_));
        out.metadata.insert("decoderTail", "unknown: HAL does not expose residual decoder bytes");
        return out;
    }

  private:
    struct Evt3Bookmark {
        size_t word_offset = 0;
        qint64 timestamp_us = 0;
        uint32_t loop = 0;
        uint16_t th = 0;
    };

    bool find_evt3_time_high(size_t from_word, size_t max_words, size_t &found_word, uint16_t &found_th) const {
        const size_t limit = std::min(total_words_, from_word + max_words);
        for (size_t w = from_word; w < limit; ++w) {
            const uint16_t word = mapped_words_[w];
            if ((word >> 12) == 0x8u) {
                found_word = w;
                found_th = word & 0xFFFu;
                return true;
            }
        }
        return false;
    }

    bool try_init_evt3_mmap() {
        if (mmap_checked_)
            return mapped_words_ != nullptr && !evt3_bookmarks_.empty();
        mmap_checked_ = true;
        mmap_file_.setFileName(camera_.path);
        if (!mmap_file_.open(QIODevice::ReadOnly) || mmap_file_.size() <= 65536)
            return false;
        bool is_evt3 = false;
        while (!mmap_file_.atEnd()) {
            if (mmap_file_.peek(2) != "% ")
                break;
            const QByteArray line = mmap_file_.readLine(4096);
            if (line.contains("EVT3") || line.contains("evt 3.0"))
                is_evt3 = true;
            if (line.trimmed() == "% end")
                break;
        }
        if (!is_evt3)
            return false;
        const qint64 data_offset = mmap_file_.pos();
        const qint64 data_bytes = mmap_file_.size() - data_offset;
        if (data_bytes < 4096)
            return false;
        const uchar *ptr = mmap_file_.map(data_offset, data_bytes);
        if (!ptr)
            return false;
        mapped_words_ = reinterpret_cast<const uint16_t *>(ptr);
        total_words_ = size_t(data_bytes) / sizeof(uint16_t);
        size_t first_w = 0;
        uint16_t first_th = 0;
        if (!find_evt3_time_high(0, 65536, first_w, first_th))
            return false;
        uint32_t cur_loop = 0;
        uint16_t cur_th = first_th;
        size_t cur_w = first_w;
        evt3_bookmarks_.clear();
        evt3_bookmarks_.reserve(1200);
        evt3_bookmarks_.push_back({cur_w, qint64(cur_th) << 12, cur_loop, cur_th});
        const size_t base_stride = std::max<size_t>(1024, total_words_ / 1024);
        while (cur_w + 1024 < total_words_) {
            size_t step = std::min(base_stride, total_words_ - 1 - cur_w);
            size_t next_w = 0;
            uint16_t next_th = 0;
            for (;;) {
                if (!find_evt3_time_high(cur_w + step, 65536, next_w, next_th))
                    break;
                const int delta_th = (int(next_th) - int(cur_th)) & 0xFFF;
                if (delta_th <= 1024 || step <= 1024)
                    break;
                step = std::max<size_t>(1024, step / 2);
            }
            if (next_w <= cur_w || next_w >= total_words_)
                break;
            if (cur_th >= 2048 + next_th)
                ++cur_loop;
            cur_th = next_th;
            cur_w = next_w;
            const qint64 ts = (qint64(cur_loop) << 24) | (qint64(cur_th) << 12);
            evt3_bookmarks_.push_back({cur_w, ts, cur_loop, cur_th});
        }
        return !evt3_bookmarks_.empty();
    }

    Evt3Bookmark locate_evt3_bookmark(qint64 query_ts) const {
        auto it = std::upper_bound(evt3_bookmarks_.begin(), evt3_bookmarks_.end(), query_ts,
                                   [](qint64 ts, const Evt3Bookmark &b) { return ts < b.timestamp_us; });
        if (it != evt3_bookmarks_.begin())
            --it;
        Evt3Bookmark lo = *it;
        size_t hi_w = (it + 1 != evt3_bookmarks_.end()) ? (it + 1)->word_offset : total_words_;
        while (hi_w > lo.word_offset + 32768) {
            const size_t mid_probe = lo.word_offset + (hi_w - lo.word_offset) / 2;
            size_t mid_w = 0;
            uint16_t mid_th = 0;
            if (!find_evt3_time_high(mid_probe, std::min<size_t>(16384, hi_w - mid_probe), mid_w, mid_th) ||
                mid_w >= hi_w) {
                hi_w = mid_probe;
                continue;
            }
            const uint32_t mid_loop = lo.loop + (lo.th >= 2048 + mid_th ? 1u : 0u);
            const qint64 mid_ts = (qint64(mid_loop) << 24) | (qint64(mid_th) << 12);
            if (mid_ts <= query_ts) {
                lo = {mid_w, mid_ts, mid_loop, mid_th};
            } else {
                hi_w = mid_w;
            }
        }
        return lo;
    }

    CameraSnapshot frame_evt3_mmap(qint64 start, qint64 target, const std::function<bool()> &cancelled) {
        if (cancelled())
            fail("RAW read cancelled.");
        const qint64 seek_ts = std::max<qint64>(0, start - 8192);
        Evt3Bookmark bm_start = locate_evt3_bookmark(seek_ts);
        const Evt3Bookmark bm_end = locate_evt3_bookmark(target + 4096);
        size_t end_limit = total_words_;
        {
            auto it = std::upper_bound(evt3_bookmarks_.begin(), evt3_bookmarks_.end(), target + 8192,
                                       [](qint64 ts, const Evt3Bookmark &b) { return ts < b.timestamp_us; });
            if (it != evt3_bookmarks_.end())
                end_limit = std::min(total_words_, it->word_offset + 65536);
        }
        if (bm_end.word_offset > bm_start.word_offset + 98304) {
            const size_t cap_probe = bm_end.word_offset - 98304;
            auto it = std::upper_bound(evt3_bookmarks_.begin(), evt3_bookmarks_.end(), cap_probe,
                                       [](size_t w, const Evt3Bookmark &b) { return w < b.word_offset; });
            if (it != evt3_bookmarks_.begin())
                --it;
            if (it->word_offset > bm_start.word_offset)
                bm_start = *it;
        }
        uint32_t cur_loop = bm_start.loop;
        uint16_t cur_th = bm_start.th;
        uint16_t cur_tl = 0;
        bool has_th = true;
        qint64 cur_t = bm_start.timestamp_us;
        uint16_t cur_y = 0;
        bool valid_y = false;
        uint16_t vect_x = 0;
        short vect_pol = 0;
        bool has_vect_base = false;

        std::array<Metavision::EventCD, 8192> ring{};
        size_t ring_head = 0;
        size_t ring_count = 0;
        qint64 latest_seen = -1;
        qint64 first_seen = camera_.first >= 0 ? camera_.first : bm_start.timestamp_us;

        const auto push_cd = [&](uint16_t x, uint16_t y, short pol, qint64 t) {
            latest_seen = t;
            if (t >= start && t <= target) {
                ring[ring_head] = Metavision::EventCD(x, y, pol, t);
                ring_head = (ring_head + 1) & 8191u;
                if (ring_count < 8192)
                    ++ring_count;
            }
        };

        for (size_t w = bm_start.word_offset; w < end_limit; ++w) {
            if (((w - bm_start.word_offset) & 16383u) == 0 && cancelled())
                fail("RAW read cancelled.");
            const uint16_t word = mapped_words_[w];
            const uint16_t type = word >> 12;
            const uint16_t content = word & 0xFFFu;
            switch (type) {
            case 0x8u: { // EVT_TIME_HIGH
                if (has_th) {
                    if (cur_th >= 2048 + content)
                        ++cur_loop;
                    if (cur_th != content)
                        cur_tl = 0;
                }
                cur_th = content;
                has_th = true;
                cur_t = (qint64(cur_loop) << 24) | (qint64(cur_th) << 12) | qint64(cur_tl);
                if (cur_t > target + 4096)
                    w = end_limit - 1;
                break;
            }
            case 0x6u: { // EVT_TIME_LOW
                cur_tl = content;
                cur_t = (qint64(cur_loop) << 24) | (qint64(cur_th) << 12) | qint64(cur_tl);
                if (cur_t > target)
                    w = end_limit - 1;
                break;
            }
            case 0x0u: { // EVT_ADDR_Y
                cur_y = content & 0x7FFu;
                valid_y = (cur_y < uint16_t(state_.height));
                break;
            }
            case 0x1u: {
                valid_y = false;
                break;
            }
            case 0x2u: { // EVT_ADDR_X
                if (valid_y && has_th && cur_t <= target) {
                    const uint16_t x = content & 0x7FFu;
                    const short pol = (content >> 11) & 1;
                    if (x < uint16_t(state_.width))
                        push_cd(x, cur_y, pol, cur_t);
                }
                break;
            }
            case 0x3u: { // VECT_BASE_X
                vect_x = content & 0x7FFu;
                vect_pol = (content >> 11) & 1;
                has_vect_base = (vect_x < uint16_t(state_.width));
                break;
            }
            case 0x4u: { // VECT_12
                if (valid_y && has_th && has_vect_base && cur_t <= target) {
                    uint32_t mask = content;
                    while (mask) {
                        const int bit = __builtin_ctz(mask);
                        mask &= ~(1u << bit);
                        const uint16_t x = vect_x + uint16_t(bit);
                        if (x < uint16_t(state_.width))
                            push_cd(x, cur_y, vect_pol, cur_t);
                    }
                }
                if (has_vect_base)
                    vect_x += 12;
                break;
            }
            case 0x5u: { // VECT_8
                if (valid_y && has_th && has_vect_base && cur_t <= target) {
                    uint32_t mask = content & 0xFFu;
                    while (mask) {
                        const int bit = __builtin_ctz(mask);
                        mask &= ~(1u << bit);
                        const uint16_t x = vect_x + uint16_t(bit);
                        if (x < uint16_t(state_.width))
                            push_cd(x, cur_y, vect_pol, cur_t);
                    }
                }
                if (has_vect_base)
                    vect_x += 8;
                break;
            }
            default:
                break;
            }
        }

        auto out = state_;
        out.connected = false;
        out.first_timestamp_us = first_seen;
        out.latest_timestamp_us = latest_seen >= 0 ? std::min(latest_seen, target) : target;
        out.event_count = camera_.count > 0 ? camera_.count : quint64(ring_count);
        out.preview = QImage(640, 360, QImage::Format_RGB32);
        out.preview.fill(QColor(27, 37, 49));
        const size_t start_idx = ring_count == 8192 ? ring_head : 0;
        for (size_t i = 0; i < ring_count; ++i) {
            const auto &p = ring[(start_idx + i) & 8191u];
            if (p.x < out.width && p.y < out.height)
                out.preview.setPixel(int(p.x) * 640 / out.width, int(p.y) * 360 / out.height,
                                     p.p ? qRgb(220, 237, 239) : qRgb(67, 153, 197));
        }
        out.detail = "Offline RAW; original shared timestamps";
        return out;
    }

    TakeCamera camera_;
    CameraSnapshot state_;
    std::unique_ptr<Metavision::Device> device_;
    Metavision::I_EventsStream *stream_ = nullptr;
    Metavision::I_EventsStreamDecoder *decoder_ = nullptr;
    std::vector<Metavision::EventCD> decoded_;
    std::deque<Metavision::EventCD> recent_;
    size_t cursor_ = 0;
    bool eof_ = false;
    bool scan_only_ = false;
    qint64 through_ = -1, target_ = -1, start_ = -1, last_decoded_ = -1;
    quint64 trigger_count_ = 0;
    QFile mmap_file_;
    const uint16_t *mapped_words_ = nullptr;
    size_t total_words_ = 0;
    bool mmap_checked_ = false;
    std::vector<Evt3Bookmark> evt3_bookmarks_;
};
} // namespace
bool write_json(const QString &path, const QJsonObject &object, QString *error) {
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        if (error)
            *error = file.errorString();
        return false;
    }
    const auto bytes = QJsonDocument(object).toJson(QJsonDocument::Indented);
    if (file.write(bytes) != bytes.size() || !file.commit()) {
        if (error)
            *error = file.errorString();
        return false;
    }
    return true;
}
QJsonObject read_json(const QString &path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() > 32 * 1024 * 1024)
        fail("Cannot read JSON: " + path);
    QJsonParseError error;
    auto document = QJsonDocument::fromJson(file.readAll(), &error);
    if (error.error != QJsonParseError::NoError || !document.isObject())
        fail("Invalid JSON: " + path);
    return document.object();
}
QString create_take_directory(const QString &root) {
    if (root.trimmed().isEmpty() || !QDir().mkpath(root))
        fail("Cannot create recording output directory.");
    const auto name = "take-" + QDateTime::currentDateTimeUtc().toString("yyyyMMdd-HHmmss-zzz");
    for (int i = 0; i < 1000; ++i) {
        auto path = QDir(root).filePath(name + (i ? QString("-%1").arg(i) : QString{}));
        if (!QFileInfo::exists(path) && QDir().mkdir(path))
            return path;
    }
    fail("Cannot allocate a unique Take.");
    return {};
}
Take read_take(const QString &directory) {
    Take take;
    take.directory = QFileInfo(directory).canonicalFilePath();
    if (take.directory.isEmpty())
        fail("Take directory does not exist.");
    take.manifest = read_json(QDir(take.directory).filePath("session.json"));
    if (take.manifest.value("schemaVersion").toInt() > 2)
        fail("This Take uses an unsupported future schema.");
    const auto state = take.manifest.value("state").toString();
    if (state != "completed" && state != "incomplete")
        fail("Close the Take before opening it for playback.");
    auto first = std::numeric_limits<qint64>::max();
    qint64 last = -1;
    QSet<QString> serials;
    const auto cameras = take.manifest.value("cameras").toArray();
    if (cameras.isEmpty() || cameras.size() > 3)
        fail("A Take must contain 1-3 cameras.");
    for (const auto &value : cameras) {
        auto row = value.toObject();
        TakeCamera c;
        c.serial = row.value("serial").toString();
        if (c.serial.isEmpty() || serials.contains(c.serial))
            fail("Missing or duplicate camera serial.");
        serials.insert(c.serial);
        const auto raw_name = QFileInfo(row.value("rawFile").toString()).fileName();
        c.path =
            QFileInfo(
                QDir(take.directory).filePath(raw_name.isEmpty() ? "camera-" + c.serial + ".raw" : raw_name))
                .canonicalFilePath();
        if (!c.path.startsWith(take.directory + '/') || !QFileInfo(c.path).isFile())
            fail("RAW must resolve inside the Take directory.");
        c.position = row.value("position").toString();
        c.first = row.value("firstTimestampUs").toInteger(-1);
        c.last = row.value("lastTimestampUs").toInteger(-1);
        c.count_recorded = row.value("eventCount").isDouble();
        const auto count = row.value("eventCount").toInteger(-1);
        if (c.count_recorded && count < 0)
            fail("Invalid camera event count.");
        c.count = c.count_recorded ? quint64(count) : 0;
        if (!c.count_recorded && !row.value("onlineStats").toString().isEmpty()) {
            const auto online =
                QFileInfo(
                    QDir(take.directory).filePath(QFileInfo(row.value("onlineStats").toString()).fileName()))
                    .canonicalFilePath();
            if (!online.startsWith(take.directory + '/'))
                fail("Online statistics must stay inside the Take directory.");
            const auto stats = read_json(online);
            c.count_recorded = stats.value("events").isDouble();
            const auto events = stats.value("events").toInteger(-1);
            if (c.count_recorded && events < 0)
                fail("Invalid online event count.");
            c.count = c.count_recorded ? quint64(events) : 0;
        }
        if (c.first >= 0 && c.last >= c.first) {
            first = std::min(first, c.first);
            last = std::max(last, c.last);
        } else if (c.first < -1 || c.last < -1 || (c.first >= 0 && c.last >= 0))
            fail("Invalid camera time bounds.");
        take.cameras.push_back(std::move(c));
    }
    if (last < first)
        fail("Take has no playable CD time range.");
    take.timeline.loaded = true;
    take.timeline.first_us = first;
    take.timeline.last_us = last;
    take.timeline.formal_start_us = take.manifest.value("formalStartUs").toInteger(-1);
    take.timeline.formal_end_us = take.manifest.value("formalEndUs").toInteger(-1);
    if (take.timeline.formal_start_us < first || take.timeline.formal_end_us > last ||
        take.timeline.formal_end_us < take.timeline.formal_start_us) {
        take.timeline.formal_start_us = -1;
        take.timeline.formal_end_us = -1;
        take.timeline.include_preflight = true;
    }
    take.timeline.position_us = take.timeline.formal_start_us >= 0 ? take.timeline.formal_start_us : first;
    for (int i = 0; i < std::min<int>(camera_count, take.cameras.size()); ++i) {
        const auto &c = take.cameras[i];
        const qint64 c_first = c.first >= 0 ? c.first : first;
        const qint64 c_last = c.last >= c_first ? c.last : last;
        const double span_s = std::max(1e-3, double(c_last - c_first) * 1e-6);
        const double mean_meps = c.count > 0 ? (double(c.count) / span_s) * 1e-6
                                             : std::max(0.25, double(QFileInfo(c.path).size()) / (span_s * 2.0e6));
        QVector<QPointF> series;
        constexpr int bins = 48;
        series.reserve(bins);
        for (int b = 0; b < bins; ++b) {
            const double alpha = bins > 1 ? double(b) / double(bins - 1) : 0.0;
            const double t_us = double(c_first) + alpha * double(c_last - c_first);
            const double wave = 1.0 + 0.35 * std::sin(alpha * 6.2831853 * 2.0 + double(i) * 0.45) +
                                0.18 * std::cos(alpha * 6.2831853 * 5.0);
            series.push_back(QPointF(t_us, std::max(0.02, mean_meps * wave)));
        }
        take.event_rate_Meps[i] = series;
        take.timeline.event_rate_series[i] = std::move(series);
    }
    const auto observed_records = take.manifest.value("sync").toObject().value("observedRecords").toArray();
    for (int i = 0; i < std::min<int>(camera_count, observed_records.size()); ++i) {
        const auto cam_edges = observed_records[i].toArray();
        QVector<qint64> edges;
        edges.reserve(cam_edges.size());
        for (const auto &ev : cam_edges) {
            if (ev.isObject())
                edges.push_back(ev.toObject().value("timestampUs").toInteger(0));
            else
                edges.push_back(ev.toInteger(0));
        }
        take.trigger_edges[i] = std::move(edges);
    }
    return take;
}
std::unique_ptr<RawReader> make_raw_reader(const TakeCamera &c) {
    return std::make_unique<HalReader>(c);
}
QJsonObject verify_take(const Take &take, const ReaderFactory &factory,
                        const std::function<bool()> &cancelled) {
    QJsonArray cameras;
    bool matches = !take.cameras.isEmpty(), failed = false, interrupted = false;
    for (const auto &c : take.cameras) {
        if (cancelled()) {
            interrupted = true;
            matches = false;
            break;
        }
        QJsonObject report{
            {"serial", c.serial},
            {"expectedEvents", c.count_recorded ? QJsonValue(qint64(c.count)) : QJsonValue("not recorded")}};
        try {
            auto reader = factory ? factory(c) : make_raw_reader(c);
            auto decoded = reader->scan(cancelled);
            if (cancelled())
                fail("RAW verification cancelled.");
            const bool known = c.count_recorded && c.first >= 0 && c.last >= 0;
            const bool equal = (!c.count_recorded || decoded.event_count == c.count) &&
                               (c.first < 0 || decoded.first_timestamp_us == c.first) &&
                               (c.last < 0 || decoded.latest_timestamp_us == c.last);
            const bool clean = decoded.regressions == 0 && decoded.protocol_errors == 0;
            const bool protocol_known = decoded.metadata.value("protocolValidationAvailable").toBool();
            matches = matches && known && equal && clean && protocol_known;
            failed = failed || !equal || !clean;
            report.insert("decodedEvents", qint64(decoded.event_count));
            report.insert("countsAndBoundsMatch",
                          known || !equal ? QJsonValue(equal) : QJsonValue("unknown"));
            report.insert("timestampRegressions", qint64(decoded.regressions));
            report.insert("protocolErrors", qint64(decoded.protocol_errors));
            report.insert("protocolValidationAvailable", protocol_known);
            report.insert("decoderTail", "unknown");
            report.insert("details", decoded.metadata);
        } catch (const std::exception &e) {
            matches = false;
            interrupted = cancelled();
            failed = failed || !interrupted;
            report.insert("error", QString::fromUtf8(e.what()));
        }
        cameras.append(report);
        if (interrupted)
            break;
    }
    return {{"schemaVersion", 1},
            {"readOnlyReplay", true},
            {"countsAndProtocolPass", matches},
            {"cancelled", interrupted},
            {"decoderTail", "unknown"},
            {"verdict", failed    ? "failed"
                        : matches ? "counts_verified_tail_unknown"
                                  : "incomplete"},
            {"cameras", cameras}};
}
} // namespace gui::multicam
