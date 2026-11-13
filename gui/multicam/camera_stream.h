#pragma once

#include "types.h"
#include <functional>
#include <map>
#include <memory>
#include <metavision/sdk/base/events/event_cd.h>
#include <optional>
#include <vector>

namespace gui::multicam {

struct CountWindow {
    qint64 start_us = 0;
    cv::Mat on, off;
};

struct DeviceEntry {
    QString selection, serial, label;
};

class LiveEventBuffer {
  public:
    LiveEventBuffer(int width, int height);
    void consume(const Metavision::EventCD *begin, const Metavision::EventCD *end);
    const CameraSnapshot &statistics() const { return statistics_; }
    QImage preview() const;
    void collect_windows(bool enabled, qint64 watermark);
    void request_collection(bool enabled, qint64 watermark);
    bool ingress_drained(qint64 watermark);
    std::optional<CountWindow> window(qint64 start, qint64 watermark) const;

  private:
    CameraSnapshot statistics_;
    bool counting_ = false;
    bool collection_pending_ = false;
    qint64 partial_window_ = -1;
    mutable qint64 sealed_until_ = -1;
    std::map<qint64, CountWindow> windows_;
    std::vector<Metavision::EventCD> recent_;
    mutable QImage preview_;
    mutable quint64 preview_revision_ = std::numeric_limits<quint64>::max();
};

class CameraStream {
  public:
    virtual ~CameraStream() = default;
    virtual void open(const CameraBinding &, bool master) = 0;
    virtual void start(const QString &raw_path = {}) = 0;
    virtual void pump() = 0;
    virtual void stop() = 0;
    virtual CameraSnapshot snapshot() const = 0;
    virtual CameraSnapshot statistics() const { return snapshot(); }
    virtual qint64 watermark() const = 0;
    virtual std::optional<CountWindow> window(qint64 start) const = 0;
    virtual void collect_windows(bool enabled) = 0;
    virtual void apply_tuning(const CameraTuning &) {}
};

using CameraFactory = std::function<std::unique_ptr<CameraStream>()>;
std::unique_ptr<CameraStream> make_hal_camera();
std::unique_ptr<CameraStream> make_buffered_camera(std::unique_ptr<CameraStream> source);
QVector<DeviceEntry> discover_cameras();
SyncEvidence match_trigger_edges(const std::array<QVector<TriggerEdge>, 3> &edges, int master,
                                 double max_delta_us, quint64 epoch);

} // namespace gui::multicam
