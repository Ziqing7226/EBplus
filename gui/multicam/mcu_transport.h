#pragma once

#include <QByteArray>
#include <QJsonObject>
#include <QString>
#include <QStringList>
#include <QVector>
#include <atomic>
#include <functional>
#include <memory>

namespace gui::multicam {

QStringList distinct_serial_port_paths(const QStringList &paths);

class RawReplResponse {
  public:
    bool feed(const QByteArray &bytes);
    bool complete() const { return complete_; }
    bool failed() const { return !error_.isEmpty(); }
    QByteArray output() const { return output_; }
    QString error() const { return error_; }

  private:
    QByteArray bytes_, output_;
    QString error_;
    bool complete_ = false;
};

struct McuResult {
    bool ok = false;
    bool low_confirmed = false;
    QString reason;
    QJsonObject details;
};

bool valid_finite_burst_response(const QJsonObject &, int pulses, int original_drive_ma,
                                 int diagnostic_drive_ma = 0);

QByteArray mcu_prepare_command();
QByteArray finite_burst_command(int pulses, int diagnostic_drive_ma);
QByteArray continuous_start_command(int original_drive_ma);
QByteArray continuous_keepalive_command(int original_drive_ma);
QByteArray mcu_cleanup_command(int original_drive_ma);
QByteArray mcu_state_command();
int restore_drive_from_mcu_state(const QJsonObject &, int known_original_drive_ma = -1);
bool valid_continuous_response(const QJsonObject &, int original_drive_ma);
bool valid_mcu_cleanup_response(const QJsonObject &, int original_drive_ma);

class McuTransport {
  public:
    using Pump = std::function<void()>;
    virtual ~McuTransport() = default;
    virtual McuResult check(const QString &expected_serial, std::atomic_bool &cancelled,
                            const Pump &pump) = 0;
    virtual McuResult burst(int pulses, std::atomic_bool &cancelled, const Pump &pump) = 0;
    virtual McuResult start_continuous(std::atomic_bool &, const Pump &);
    virtual McuResult keepalive_continuous(std::atomic_bool &, const Pump &);
    virtual McuResult cleanup(const Pump &pump) = 0;
};

struct McuPort {
    QString path;
    QString serial;
    quint16 vendor_id = 0;
    quint16 product_id = 0;
};

class McuConnection {
  public:
    virtual ~McuConnection() = default;
    virtual QVector<McuPort> ports() = 0;
    virtual void open(const QString &path) = 0;
    virtual QByteArray execute(const QByteArray &, int timeout_ms, std::atomic_bool &,
                               const McuTransport::Pump &) = 0;
};

class SerialMcuTransport final : public McuTransport {
  public:
    explicit SerialMcuTransport(int diagnostic_drive_ma = 0, std::unique_ptr<McuConnection> = {});
    ~SerialMcuTransport() override;
    McuResult check(const QString &, std::atomic_bool &, const Pump &) override;
    McuResult burst(int, std::atomic_bool &, const Pump &) override;
    // Requires an explicit 12 mA constructor setting. Call keepalive at least every 2 s;
    // the MCU's soft Timer stops output after ~8 s without a valid heartbeat.
    McuResult start_continuous(std::atomic_bool &, const Pump &) override;
    McuResult keepalive_continuous(std::atomic_bool &, const Pump &) override;
    McuResult cleanup(const Pump &) override;

  private:
    std::unique_ptr<McuConnection> connection_;
    QString port_path_;
    bool verified_ = false;
    bool low_confirmed_ = false;
    bool continuous_requested_ = false;
    bool continuous_confirmed_ = false;
    int diagnostic_drive_ma_ = 0;
    int original_drive_ma_ = -1;
    QByteArray execute(const QByteArray &code, int timeout_ms, std::atomic_bool &, const Pump &);
    void reopen_bound_port(std::atomic_bool &);
    McuResult stop_after_continuous_failure(McuResult, const Pump &);
};
} // namespace gui::multicam
