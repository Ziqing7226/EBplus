#pragma once
#include "types.h"
#include <functional>
#include <memory>

namespace gui::multicam {
bool write_json(const QString &path, const QJsonObject &, QString *error = nullptr);
QJsonObject read_json(const QString &path);
QString create_take_directory(const QString &root);
struct TakeCamera {
    QString path, serial, position;
    qint64 first = -1, last = -1;
    quint64 count = 0;
    bool count_recorded = false;
};
struct Take {
    QString directory;
    QJsonObject manifest;
    QVector<TakeCamera> cameras;
    PlaybackStatus timeline;
    std::array<QVector<QPointF>, camera_count> event_rate_Meps;
    std::array<QVector<qint64>, camera_count> trigger_edges;
};
Take read_take(const QString &directory);

class RawReader {
  public:
    virtual ~RawReader() = default;
    virtual CameraSnapshot frame(qint64 start, qint64 target, const std::function<bool()> &cancelled) = 0;
    virtual CameraSnapshot scan(const std::function<bool()> &cancelled) = 0;
};
using ReaderFactory = std::function<std::unique_ptr<RawReader>(const TakeCamera &)>;
std::unique_ptr<RawReader> make_raw_reader(const TakeCamera &);
QJsonObject verify_take(const Take &, const ReaderFactory &, const std::function<bool()> &cancelled);
} // namespace gui::multicam
