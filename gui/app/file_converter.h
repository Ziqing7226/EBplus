// gui/app/file_converter.h — offline file conversion tools (design §1.5.7).
//
// Converts a source event file to:
//   • HDF5   (Metavision::HDF5EventFileWriter)
//   • CSV    (plain "t,x,y,p" text — one event per line)
//   • RAW cut(time-range sub-clip via Metavision::RAWEvt2EventFileWriter)
//
// Also reports file metadata (duration, geometry, encoding, integrator).
// All operations run on a background thread; progress is reported via signals.

#ifndef GUI_APP_FILE_CONVERTER_H
#define GUI_APP_FILE_CONVERTER_H

#include <QObject>
#include <QString>
#include <QJsonObject>
#include <atomic>
#include <functional>
#include <thread>
#include <utility>

#include <metavision/sdk/base/utils/timestamp.h>

namespace gui {

struct FileInfo {
    QString path;
    int width{0};
    int height{0};
    Metavision::timestamp duration_us{0};
    QString integrator;
    QString serial;
    QString plugin;
    QString encoding;
    qint64 event_count{-1};
};

class FileConverter : public QObject {
    Q_OBJECT
public:
    explicit FileConverter(QObject* parent = nullptr);
    ~FileConverter();

    enum class Format { HDF5, CSV };
    void convert(const QString& src, const QString& dst, Format fmt);
    /// @brief Cuts @p src to [@p start_us, @p end_us] writing RAW to @p dst.
    void cut(const QString& src, const QString& dst,
             Metavision::timestamp start_us, Metavision::timestamp end_us);
    /// @brief Queries metadata for @p src (synchronous, fast).
    FileInfo info(const QString& src) const;

    /// @brief Provides a known total duration (us) for a source path, or 0
    /// when unknown. Set by MainWindow so that operations on the currently
    /// open (fully buffered) file skip the blocking OSC duration query,
    /// which would otherwise freeze progress reporting and cancellation.
    void set_duration_provider(
        std::function<Metavision::timestamp(const QString&)> provider) {
        duration_provider_ = std::move(provider);
    }

    /// @brief Returns the provider's known duration for @p src, or 0.
    /// Used by dialogs to display the source duration without a blocking
    /// OSC query.
    Metavision::timestamp duration_of(const QString& src) const {
        return duration_provider_ ? duration_provider_(src) : 0;
    }

    /// @brief Provides the timeline range {first, last} event timestamp (us)
    /// for a source path, or {0, 0} when unknown. Cut files keep the source
    /// timestamps (no re-basing to 0), so a cut dialog must prefill the real
    /// range — last alone (the old duration) is NOT enough.
    void set_time_range_provider(
        std::function<std::pair<Metavision::timestamp, Metavision::timestamp>(
            const QString&)>
            provider) {
        time_range_provider_ = std::move(provider);
    }

    /// @brief Returns the provider's known {first, last} event timestamps for
    /// @p src, or {0, 0}. last > 0 means the range is known.
    std::pair<Metavision::timestamp, Metavision::timestamp>
    time_range_of(const QString& src) const {
        return time_range_provider_ ? time_range_provider_(src)
                                    : std::make_pair(
                                          static_cast<Metavision::timestamp>(0),
                                          static_cast<Metavision::timestamp>(0));
    }

    void cancel();

signals:
    void progress(double ratio);
    void completed(const QString& output);
    void failed(const QString& message);

private:
    void run_convert(const QString& src, const QString& dst, Format fmt);
    void run_cut(const QString& src, const QString& dst,
                 Metavision::timestamp start_us, Metavision::timestamp end_us);
    /// External-format variants (AEDAT4 / ALPDATA): the reader's synchronous
    /// run() replaces the SDK camera + polling loop; everything downstream
    /// (writers, cancel semantics, progress) is shared behavior.
    void run_convert_external(const QString& src, const QString& dst, Format fmt);
    void run_cut_external(const QString& src, const QString& dst,
                          Metavision::timestamp start_us, Metavision::timestamp end_us);

    std::function<Metavision::timestamp(const QString&)> duration_provider_;
    std::function<std::pair<Metavision::timestamp, Metavision::timestamp>(
        const QString&)>
        time_range_provider_;
    std::atomic<bool> running_{false};
    std::atomic<bool> cancel_{false};
    std::thread worker_;
};

} // namespace gui

#endif // GUI_APP_FILE_CONVERTER_H
