#include "mcu_transport.h"

#include <QElapsedTimer>
#include <QJsonDocument>
#include <QMap>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <stdexcept>

namespace gui::multicam {
namespace {
constexpr const char *firmware_sha256 = "06b46f189635824b2ded6baeccf2794e3e52a78b004f1413fb8c00aac4a3651b";
constexpr const char *board_serial = "36f9f437460d3ba8";
bool valid_drive(int ma) { return ma == 2 || ma == 4 || ma == 8 || ma == 12; }
void require_original_drive(int ma) {
    if (!valid_drive(ma))
        throw std::invalid_argument("Invalid original MCU drive strength.");
}
bool number_is(const QJsonObject &details, const char *key, double expected) {
    const auto value = details.value(QLatin1String(key));
    return value.isDouble() && value.toDouble() == expected;
}
bool boolean_is(const QJsonObject &details, const char *key, bool expected) {
    const auto value = details.value(QLatin1String(key));
    return value.isBool() && value.toBool() == expected;
}
QByteArray firmware_verification_command() {
    return QByteArray("import json, os, hashlib, binascii\n"
                      "with open('/rp2350_burst.py','rb') as f: "
                      "h=binascii.hexlify(hashlib.sha256(f.read()).digest()).decode()\n"
                      "if h!='") +
           firmware_sha256 +
           "': raise ValueError('Unapproved finite-burst firmware')\n"
           "if 'RP2350' not in os.uname().machine: raise ValueError('Expected RP2350')\n";
}
QByteArray direct_gpio_command() {
    return "import json, os, machine, rp2\n"
           "if 'RP2350' not in os.uname().machine: raise ValueError('Expected RP2350')\n"
           "_ebplus_gp2_pad=0x4003800C\n"
           "_ebplus_drives=(2,4,8,12)\n";
}
QByteArray pio_release_command() {
    // Deactivating a state machine does not release its PIO instruction allocation.
    return R"MCU(def _ebplus_release_pio_program(_program,_r=rp2):
    if _program is None:
        return
    if any(_r.StateMachine(i).active() for i in range(4)):
        raise ValueError('Stop all PIO0 state machines before releasing a program')
    try:
        _r.PIO(0).remove_program(_program)
    except ValueError as _e:
        if str(_e)!='prog not in instruction memory':
            raise
)MCU";
}

class SerialConnection final : public McuConnection {
  public:
    QVector<McuPort> ports() override {
        QVector<McuPort> result;
        for (const auto &info : QSerialPortInfo::availablePorts())
            if (info.hasVendorIdentifier() && info.hasProductIdentifier())
                result.append({info.systemLocation(), info.serialNumber(), info.vendorIdentifier(),
                               info.productIdentifier()});
        return result;
    }
    void open(const QString &path) override {
        if (port_)
            port_->close();
        port_ = std::make_unique<QSerialPort>(path);
        port_->setBaudRate(115200);
        if (!port_->open(QIODevice::ReadWrite))
            throw std::runtime_error(port_->errorString().toStdString());
    }
    QByteArray execute(const QByteArray &, int, std::atomic_bool &, const McuTransport::Pump &) override;

  private:
    std::unique_ptr<QSerialPort> port_;
    void enter_raw(std::atomic_bool &, const McuTransport::Pump &);
};
QJsonObject response_json(const QByteArray &output) {
    for (const auto &row : output.split('\n')) {
        if (!row.startsWith("EBPLUS_MCU "))
            continue;
        QJsonParseError error;
        const auto document = QJsonDocument::fromJson(row.mid(11).trimmed(), &error);
        if (error.error == QJsonParseError::NoError && document.isObject())
            return document.object();
    }
    throw std::runtime_error("Missing or malformed MCU JSON response.");
}
McuResult failure(const std::exception &e) {
    auto reason = QString::fromUtf8(e.what());
    if (reason.contains("ENOMEM"))
        reason += QStringLiteral("\nPIO resources could not be allocated. Old allocations may remain from an "
                                 "earlier app. Stop Trigger, close EBplus, power-cycle the RP2350 and check "
                                 "the controller again. No automatic reset or pulse retry was performed.");
    return {false, false, reason, {}};
}
} // namespace

QStringList distinct_serial_port_paths(const QStringList &paths) {
    QMap<QString, QString> ports;
    for (const auto &path : paths) {
        QString key = path;
        if (path.startsWith("/dev/cu."))
            key = "/dev/" + path.mid(8);
        else if (path.startsWith("/dev/tty."))
            key = "/dev/" + path.mid(9);
        if (!ports.contains(key) || path.startsWith("/dev/cu."))
            ports.insert(key, path);
    }
    return ports.values();
}

bool RawReplResponse::feed(const QByteArray &bytes) {
    if (complete_ || failed())
        return complete_;
    bytes_ += bytes;
    if (bytes_.size() > 1024 * 1024) {
        error_ = QStringLiteral("MCU response exceeds 1 MiB.");
        return false;
    }
    if (bytes_.size() < 2)
        return false;
    if (!bytes_.startsWith("OK")) {
        error_ = QStringLiteral("MCU did not acknowledge raw REPL execution.");
        return false;
    }
    const auto first = bytes_.indexOf('\x04', 2);
    if (first < 0)
        return false;
    const auto second = bytes_.indexOf('\x04', first + 1);
    if (second < 0)
        return false;
    const auto stderr_bytes = bytes_.mid(first + 1, second - first - 1);
    if (!stderr_bytes.trimmed().isEmpty()) {
        error_ = QString::fromUtf8(stderr_bytes).left(2048);
        return false;
    }
    output_ = bytes_.mid(2, first - 2);
    complete_ = true;
    return true;
}

bool valid_finite_burst_response(const QJsonObject &details, int pulses, int original_drive_ma,
                                 int diagnostic_drive_ma) {
    if ((pulses != 1 && pulses != 3) ||
        (original_drive_ma != 2 && original_drive_ma != 4 && original_drive_ma != 8 &&
         original_drive_ma != 12) ||
        (diagnostic_drive_ma != 0 && diagnostic_drive_ma != 12))
        return false;
    const int expected_during = diagnostic_drive_ma ? diagnostic_drive_ma : original_drive_ma;
    return details.value("output").toString() == QStringLiteral("low") &&
           details.value("gpio").toInt(-1) == 2 && details.value("pulses").toInt() == pulses &&
           details.value("expected_edges").toInt() == 2 * pulses &&
           details.value("high_us").toInt() == 1000000 && details.value("low_us").toInt() == 1000000 &&
           details.value("drive_strength_mA_before").isDouble() &&
           details.value("drive_strength_mA_during").isDouble() &&
           details.value("drive_strength_mA_after").isDouble() &&
           details.value("drive_strength_mA_before").toDouble() == original_drive_ma &&
           details.value("drive_strength_mA_during").toDouble() == expected_during &&
           details.value("drive_strength_mA_after").toDouble() == original_drive_ma;
}

QByteArray mcu_prepare_command() {
    return firmware_verification_command() + "import sys, rp2\n" + pio_release_command() +
           "if any(rp2.StateMachine(i).active() for i in range(4)): "
           "raise ValueError('PIO0 is busy; stop all state machines before checking')\n"
           "if globals().get('_ebplus_continuous',{}).get('watchdog_active',False): "
           "raise ValueError('Continuous watchdog is busy')\n"
           "_cached=sys.modules.get('rp2350_burst')\n"
           "_ebplus_release_pio_program(getattr(_cached,'_finite_burst',None))\n"
           "_ebplus_release_pio_program(globals().get('_ebplus_continuous_program'))\n"
           "os.chdir('/')\nsys.path[:]=['/']+[p for p in sys.path if p!='/']\n"
           "sys.modules.pop('rp2350_burst',None)\n"
           "import rp2350_burst as b\n"
           "print('EBPLUS_MCU "
           "'+json.dumps({'machine':os.uname().machine,'gpio':b.GPIO,'level':b._pin.value(),"
           "'drive':b._drive_ma(),'sha256':h,'burst_callable':callable(b.burst)}))\n";
}

QByteArray finite_burst_command(int pulses, int diagnostic_drive_ma) {
    if ((pulses != 1 && pulses != 3) || (diagnostic_drive_ma != 0 && diagnostic_drive_ma != 12))
        throw std::invalid_argument("Invalid finite Trigger pulse count or drive strength.");
    const QByteArray drive = diagnostic_drive_ma ? QByteArray::number(diagnostic_drive_ma) : "None";
    return "import json, rp2, rp2350_burst as b\n" + pio_release_command() + "try:\n    _result=b.burst(" +
           QByteArray::number(pulses) + ",1000000," + drive +
           ")\n"
           "finally:\n    _ebplus_release_pio_program(b._finite_burst)\n"
           "print('EBPLUS_MCU '+json.dumps(_result))\n";
}

QByteArray continuous_start_command(int original_drive_ma) {
    require_original_drive(original_drive_ma);
    return firmware_verification_command() + "import machine, rp2, time, rp2350_burst as b\n" +
           pio_release_command() +
           "if b.GPIO!=2 or b._pin.value()!=0: raise ValueError('GP2 must be LOW')\n"
           "if b._drive_ma()!=" +
           QByteArray::number(original_drive_ma) +
           ": raise ValueError('Original drive changed')\n"
           "if rp2.StateMachine(0).active(): raise ValueError('PIO0 SM0 is busy')\n"
           "if any(rp2.StateMachine(i).active() for i in range(1,4)): "
           "raise ValueError('Another PIO0 state machine is busy')\n"
           "if globals().get('_ebplus_continuous',{}).get('watchdog_active',False): "
           "raise ValueError('Continuous watchdog is already armed')\n" +
           // Only this RAM-resident state and program are installed, never boot files.
           QByteArray(R"MCU(_ebplus_continuous={'running':False,'watchdog_active':False,'expired':False,
    'restore_drive_ma':)MCU") +
           QByteArray::number(original_drive_ma) + R"MCU(,'heartbeat_ms':time.ticks_ms()}
def _ebplus_continuous_stop(_s=_ebplus_continuous,_b=b,_m=machine,_r=rp2,_release=_ebplus_release_pio_program):
    try:
        _r.StateMachine(0).active(0)
    finally:
        _s['running']=False
        try:
            if _s.get('timer') is not None:
                _s['timer'].deinit()
            _s['watchdog_active']=False
        finally:
            try:
                _b._pin.init(_m.Pin.OUT,value=0)
            finally:
                try:
                    _b._set_drive_ma(_s['restore_drive_ma'])
                finally:
                    _release(_s.get('program'))
def _ebplus_continuous_watchdog(_timer,_s=_ebplus_continuous,_t=time,_stop=_ebplus_continuous_stop):
    if _s['running'] and _t.ticks_diff(_t.ticks_ms(),_s['heartbeat_ms'])>=8000:
        _s['expired']=True
        _stop()
def _ebplus_continuous_status(_s=_ebplus_continuous,_b=b,_r=rp2,_t=time,_h=h):
    return {'mode':'continuous','gpio':2,'state_machine':0,
        'active':bool(_r.StateMachine(0).active()),'running':_s['running'],
        'watchdog_active':_s['watchdog_active'],'watchdog_expired':_s['expired'],
        'watchdog_timeout_ms':8000,'watchdog_period_ms':250,
        'heartbeat_age_ms':_t.ticks_diff(_t.ticks_ms(),_s['heartbeat_ms']),
        'high_us':1000000,'low_us':1000000,'nominal_hz':0.5,
        'drive_strength_ma':_b._drive_ma(),'restore_drive_ma_on_stop':_s['restore_drive_ma'],
        'sha256':_h,'persistent_boot_change':False}
if '_ebplus_continuous_program' not in globals():
    @rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
    def _ebplus_continuous_program():
        pull(block)
        mov(isr,osr)
        wrap_target()
        set(pins,1)
        mov(x,isr)
        label('high_delay')
        jmp(x_dec,'high_delay')
        set(pins,0)
        mov(x,isr)
        label('low_delay')
        jmp(x_dec,'low_delay')
        wrap()
try:
    _ebplus_continuous['program']=_ebplus_continuous_program
    _ebplus_continuous['timer']=machine.Timer(-1)
    _ebplus_continuous['timer'].init(period=250,mode=machine.Timer.PERIODIC,callback=_ebplus_continuous_watchdog)
    _ebplus_continuous['watchdog_active']=True
    b._set_drive_ma(12)
    _ebplus_continuous['sm']=rp2.StateMachine(0,_ebplus_continuous_program,freq=1000000,set_base=b._pin)
    _ebplus_continuous['sm'].put(1000000-3)
    _ebplus_continuous['heartbeat_ms']=time.ticks_ms()
    _ebplus_continuous['running']=True
    _ebplus_continuous['sm'].active(1)
    print('EBPLUS_MCU '+json.dumps(_ebplus_continuous_status()))
except:
    _ebplus_continuous_stop()
    raise
)MCU";
}

QByteArray continuous_keepalive_command(int original_drive_ma) {
    require_original_drive(original_drive_ma);
    return firmware_verification_command() +
           "import time, rp2, rp2350_burst as b\n"
           "_s=globals().get('_ebplus_continuous')\n"
           "if not _s or not _s['running'] or not _s['watchdog_active'] or _s['expired']: "
           "raise ValueError('Continuous output is not running')\n"
           "if _s['restore_drive_ma']!=" +
           QByteArray::number(original_drive_ma) +
           ": raise ValueError('Continuous session changed')\n"
           "if time.ticks_diff(time.ticks_ms(),_s['heartbeat_ms'])>=8000:\n"
           "    _s['expired']=True\n"
           "    _ebplus_continuous_stop()\n"
           "    raise ValueError('Continuous heartbeat expired')\n"
           "if not rp2.StateMachine(0).active() or b._drive_ma()!=12:\n"
           "    _ebplus_continuous_stop()\n"
           "    raise ValueError('Continuous PIO or drive readback failed')\n"
           "_s['heartbeat_ms']=time.ticks_ms()\n"
           "print('EBPLUS_MCU '+json.dumps(_ebplus_continuous_status()))\n";
}

QByteArray mcu_state_command() {
    return direct_gpio_command() + R"MCU(_present='_ebplus_continuous' in globals()
_s=globals().get('_ebplus_continuous')
if _present:
    if type(_s) is not dict or type(_s.get('restore_drive_ma')) is not int or _s['restore_drive_ma'] not in _ebplus_drives:
        raise ValueError('Invalid residual continuous restore drive')
    if type(_s.get('running')) is not bool or type(_s.get('watchdog_active')) is not bool:
        raise ValueError('Invalid residual continuous state')
print('EBPLUS_MCU '+json.dumps({'machine':os.uname().machine,'gpio':2,
    'level':machine.Pin(2).value(),'drive':_ebplus_drives[(machine.mem32[_ebplus_gp2_pad]>>4)&3],
    'active':bool(rp2.StateMachine(0).active()),'session_present':_present,
    'session_restore_drive_ma':_s['restore_drive_ma'] if _present else None,
    'watchdog_active':_s['watchdog_active'] if _present else False}))
)MCU";
}

int restore_drive_from_mcu_state(const QJsonObject &details, int known_original_drive_ma) {
    if ((known_original_drive_ma != -1 && !valid_drive(known_original_drive_ma)) ||
        !details.value("machine").toString().contains("RP2350") || !number_is(details, "gpio", 2) ||
        (!number_is(details, "level", 0) && !number_is(details, "level", 1)) ||
        !details.value("active").isBool() || !details.value("watchdog_active").isBool() ||
        !details.value("session_present").isBool())
        throw std::runtime_error("Invalid RP2350 state readback.");
    int drive = -1;
    for (const int ma : {2, 4, 8, 12})
        if (number_is(details, "drive", ma))
            drive = ma;
    if (drive < 0)
        throw std::runtime_error("Invalid RP2350 drive readback.");
    if (details.value("session_present").toBool()) {
        int restore = -1;
        for (const int ma : {2, 4, 8, 12})
            if (number_is(details, "session_restore_drive_ma", ma))
                restore = ma;
        if (restore < 0 || (known_original_drive_ma != -1 && restore != known_original_drive_ma))
            throw std::runtime_error(
                "Residual continuous session has an invalid or conflicting original drive.");
        return restore;
    }
    if (known_original_drive_ma != -1)
        return known_original_drive_ma;
    if (details.value("active").toBool() || details.value("watchdog_active").toBool() ||
        !number_is(details, "level", 0))
        throw std::runtime_error("Unowned MCU output is not confirmed stopped and LOW.");
    return drive;
}

QByteArray mcu_cleanup_command(int original_drive_ma) {
    require_original_drive(original_drive_ma);
    // Emergency LOW must not depend on firmware files or globals surviving a USB power reset.
    return direct_gpio_command() + "import sys\n" + pio_release_command() +
           R"MCU(_s=globals().get('_ebplus_continuous')
try:
    rp2.StateMachine(0).active(0)
finally:
    try:
        if _s is not None:
            if type(_s) is not dict:
                raise ValueError('Invalid residual continuous state')
            _s['running']=False
            if _s.get('timer') is not None:
                _s['timer'].deinit()
            _s['watchdog_active']=False
    finally:
        try:
            machine.Pin(2,machine.Pin.OUT,value=0)
        finally:
            machine.mem32[_ebplus_gp2_pad]=(machine.mem32[_ebplus_gp2_pad]&~0x30)|(_ebplus_drives.index()MCU" +
           QByteArray::number(original_drive_ma) + R"MCU()<<4)
_ebplus_release_pio_program(getattr(sys.modules.get('rp2350_burst'),'_finite_burst',None))
_ebplus_release_pio_program(globals().get('_ebplus_continuous_program'))
print('EBPLUS_MCU '+json.dumps({'gpio':2,'state_machine':0,'level':machine.Pin(2).value(),
    'drive':_ebplus_drives[(machine.mem32[_ebplus_gp2_pad]>>4)&3],'active':bool(rp2.StateMachine(0).active()),
    'watchdog_active':bool(_s and _s.get('watchdog_active',False))}))
)MCU";
}

bool valid_continuous_response(const QJsonObject &details, int original_drive_ma) {
    const auto age = details.value("heartbeat_age_ms");
    return valid_drive(original_drive_ma) && details.value("mode").toString() == "continuous" &&
           number_is(details, "gpio", 2) && number_is(details, "state_machine", 0) &&
           boolean_is(details, "active", true) && boolean_is(details, "running", true) &&
           boolean_is(details, "watchdog_active", true) && boolean_is(details, "watchdog_expired", false) &&
           number_is(details, "watchdog_timeout_ms", 8000) && number_is(details, "watchdog_period_ms", 250) &&
           age.isDouble() && age.toDouble() >= 0 && age.toDouble() < 8000 &&
           number_is(details, "high_us", 1000000) && number_is(details, "low_us", 1000000) &&
           number_is(details, "nominal_hz", 0.5) && number_is(details, "drive_strength_ma", 12) &&
           number_is(details, "restore_drive_ma_on_stop", original_drive_ma) &&
           details.value("sha256").toString() == QLatin1String(firmware_sha256) &&
           boolean_is(details, "persistent_boot_change", false);
}

bool valid_mcu_cleanup_response(const QJsonObject &details, int original_drive_ma) {
    return valid_drive(original_drive_ma) && number_is(details, "gpio", 2) &&
           number_is(details, "state_machine", 0) && number_is(details, "level", 0) &&
           number_is(details, "drive", original_drive_ma) && boolean_is(details, "active", false) &&
           boolean_is(details, "watchdog_active", false);
}

McuResult McuTransport::start_continuous(std::atomic_bool &, const Pump &) {
    return {false, false, "Continuous MCU output is not supported by this transport.", {}};
}

McuResult McuTransport::keepalive_continuous(std::atomic_bool &, const Pump &) {
    return {false, false, "Continuous MCU output is not supported by this transport.", {}};
}

SerialMcuTransport::SerialMcuTransport(int diagnostic_drive_ma, std::unique_ptr<McuConnection> connection)
    : connection_(std::move(connection)), diagnostic_drive_ma_(diagnostic_drive_ma) {
    if (diagnostic_drive_ma != 0 && diagnostic_drive_ma != 12)
        throw std::invalid_argument("Only unchanged or explicitly approved 12 mA diagnostics are supported.");
    if (!connection_)
        connection_ = std::make_unique<SerialConnection>();
}
SerialMcuTransport::~SerialMcuTransport() = default;

void SerialConnection::enter_raw(std::atomic_bool &cancelled, const McuTransport::Pump &pump) {
    if (cancelled.load())
        throw std::runtime_error("MCU operation cancelled.");
    if (!port_ || !port_->isOpen())
        throw std::runtime_error("MCU serial port is not open.");
    port_->clear();
    port_->write(QByteArray("\r\x03\x03\x01", 4));
    port_->waitForBytesWritten(100);
    QByteArray reply;
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < 2000) {
        if (cancelled.load())
            throw std::runtime_error("MCU operation cancelled.");
        if (pump)
            pump();
        port_->waitForReadyRead(2);
        reply += port_->readAll();
        if (reply.contains("raw REPL") && reply.trimmed().endsWith('>'))
            return;
        if (reply.size() > 65536)
            throw std::runtime_error("Unexpected MCU REPL response.");
    }
    throw std::runtime_error("Timed out entering the MCU raw REPL.");
}

QByteArray SerialConnection::execute(const QByteArray &code, int timeout_ms, std::atomic_bool &cancelled,
                                     const McuTransport::Pump &pump) {
    enter_raw(cancelled, pump);
    if (cancelled.load())
        throw std::runtime_error("MCU operation cancelled.");
    const QByteArray command = code + '\x04';
    if (port_->write(command) != command.size())
        throw std::runtime_error("Could not write MCU command.");
    port_->waitForBytesWritten(100);
    RawReplResponse response;
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeout_ms) {
        if (cancelled.load()) {
            port_->write(QByteArray("\x03", 1));
            port_->waitForBytesWritten(50);
            throw std::runtime_error("MCU operation cancelled.");
        }
        if (pump)
            pump();
        port_->waitForReadyRead(2);
        if (response.feed(port_->readAll()))
            return response.output();
        if (response.failed())
            throw std::runtime_error(response.error().toStdString());
        if (port_->error() != QSerialPort::NoError && port_->error() != QSerialPort::TimeoutError)
            throw std::runtime_error(port_->errorString().toStdString());
    }
    port_->write(QByteArray("\x03", 1));
    throw std::runtime_error("MCU command timed out; output state is unconfirmed.");
}

QByteArray SerialMcuTransport::execute(const QByteArray &code, int timeout_ms, std::atomic_bool &cancelled,
                                       const Pump &pump) {
    return connection_->execute(code, timeout_ms, cancelled, pump);
}

void SerialMcuTransport::reopen_bound_port(std::atomic_bool &cancelled) {
    if (cancelled.load())
        throw std::runtime_error("MCU operation cancelled.");
    QStringList candidates;
    for (const auto &port : connection_->ports())
        if (port.vendor_id == 0x2e8a && port.product_id == 0x0005 &&
            port.serial == QLatin1String(board_serial) && !port.path.isEmpty())
            candidates.append(port.path);
    candidates = distinct_serial_port_paths(candidates);
    if (candidates.isEmpty())
        throw std::runtime_error("RP2350 trigger board not found over USB. Plug the RP2350 into this "
                                 "computer with a USB data cable, then click \"Check bound RP2350\" again.");
    if (candidates.size() != 1)
        throw std::runtime_error("More than one serial port reports the bound RP2350 USB identity; "
                                 "unplug duplicates and check again.");
    if (cancelled.load())
        throw std::runtime_error("MCU operation cancelled.");
    connection_->open(candidates.front());
    port_path_ = candidates.front();
}

McuResult SerialMcuTransport::check(const QString &serial, std::atomic_bool &cancelled, const Pump &pump) {
    if (continuous_requested_)
        return {false, false, "Stop continuous output before checking the MCU again.", {}};
    verified_ = false;
    low_confirmed_ = false;
    try {
        if (serial != QLatin1String(board_serial))
            throw std::runtime_error("Only the fixed approved RP2350 USB serial is allowed.");
        reopen_bound_port(cancelled);
        const auto initial = response_json(execute(mcu_state_command(), 2500, cancelled, pump));
        const bool recover = initial.value("session_present").toBool() || valid_drive(original_drive_ma_);
        original_drive_ma_ = restore_drive_from_mcu_state(initial, original_drive_ma_);
        QJsonObject recovery;
        if (recover) {
            // Preserve the saved baseline even if stop/readback fails; never adopt a stranded 12 mA.
            const auto stopped = cleanup(pump);
            if (!stopped.ok || !stopped.low_confirmed)
                return stopped;
            recovery = stopped.details;
        }
        auto details = response_json(execute(mcu_prepare_command(), 4000, cancelled, pump));
        details.insert("usbSerial", serial);
        details.insert("port", port_path_);
        details.insert("initialState", initial);
        if (recover)
            details.insert("residualRecovery", recovery);
        if (!details.value("machine").toString().contains("RP2350", Qt::CaseInsensitive) ||
            details.value("gpio").toInt(-1) != 2 || details.value("level").toInt(-1) != 0 ||
            !details.value("burst_callable").toBool() || !number_is(details, "drive", original_drive_ma_) ||
            details.value("sha256").toString() != QLatin1String(firmware_sha256))
            return {false, details.value("level").toInt(-1) == 0,
                    "RP2350 firmware, GPIO or output state does not match the approved finite-burst module.",
                    details};
        verified_ = true;
        low_confirmed_ = true;
        return {true, true,
                recover ? "Bound RP2350 verified; residual output stopped, LOW and original drive confirmed."
                        : "Bound RP2350 verified; GP2 is LOW. Drive strength was not changed.",
                details};
    } catch (const std::exception &e) {
        return failure(e);
    }
}

McuResult SerialMcuTransport::burst(int pulses, std::atomic_bool &cancelled, const Pump &pump) {
    if (!verified_ || !low_confirmed_ || continuous_requested_ || (pulses != 1 && pulses != 3))
        return {false, false, "MCU is not verified or pulse count is invalid.", {}};
    low_confirmed_ = false;
    try {
        auto details = response_json(execute(finite_burst_command(pulses, diagnostic_drive_ma_),
                                             pulses * 2000 + 3000, cancelled, pump));
        const bool low = details.value("output").toString() == QStringLiteral("low");
        const bool ok =
            valid_finite_burst_response(details, pulses, original_drive_ma_, diagnostic_drive_ma_);
        details.insert("requestedDriveMa", diagnostic_drive_ma_ ? QJsonValue(diagnostic_drive_ma_)
                                                                : QJsonValue(QJsonValue::Null));
        low_confirmed_ = ok && low;
        return {ok, low,
                ok ? QStringLiteral("Finite burst completed; GP2 is LOW.")
                   : QStringLiteral("Finite burst response did not validate."),
                details};
    } catch (const std::exception &e) {
        return failure(e);
    }
}

McuResult SerialMcuTransport::stop_after_continuous_failure(McuResult result, const Pump &pump) {
    continuous_confirmed_ = false;
    const auto stopped = cleanup(pump);
    result.low_confirmed = stopped.low_confirmed;
    result.details.insert("cleanup", stopped.details);
    result.details.insert("cleanupConfirmed", stopped.ok);
    result.reason += QStringLiteral(" Cleanup: ") + stopped.reason;
    return result;
}

McuResult SerialMcuTransport::start_continuous(std::atomic_bool &cancelled, const Pump &pump) {
    if (!verified_ || !low_confirmed_ || continuous_requested_ || diagnostic_drive_ma_ != 12 ||
        cancelled.load())
        return {false,
                low_confirmed_,
                "Continuous output requires a verified LOW MCU, explicit 12 mA setting and no cancellation.",
                {}};
    low_confirmed_ = false;
    // A lost start response may still leave PIO running. Require cleanup before any retry.
    continuous_requested_ = true;
    try {
        auto details =
            response_json(execute(continuous_start_command(original_drive_ma_), 3000, cancelled, pump));
        if (!valid_continuous_response(details, original_drive_ma_))
            return stop_after_continuous_failure(
                {false, false, "Continuous start response did not validate.", details}, pump);
        continuous_confirmed_ = true;
        return {true, false, "GP2 continuous 0.5 Hz output started at 12 mA; keepalive required.", details};
    } catch (const std::exception &e) {
        return stop_after_continuous_failure(failure(e), pump);
    }
}

McuResult SerialMcuTransport::keepalive_continuous(std::atomic_bool &cancelled, const Pump &pump) {
    if (!verified_ || !continuous_requested_ || !continuous_confirmed_)
        return {false,
                low_confirmed_,
                "Continuous MCU output is not confirmed running; cleanup may be required.",
                {}};
    try {
        auto details =
            response_json(execute(continuous_keepalive_command(original_drive_ma_), 2500, cancelled, pump));
        if (!valid_continuous_response(details, original_drive_ma_))
            return stop_after_continuous_failure(
                {false, false, "Continuous keepalive response did not validate.", details}, pump);
        return {true, false, "Continuous PIO, 12 mA drive and heartbeat verified.", details};
    } catch (const std::exception &e) {
        return stop_after_continuous_failure(failure(e), pump);
    }
}

McuResult SerialMcuTransport::cleanup(const Pump &pump) {
    continuous_confirmed_ = false;
    low_confirmed_ = false;
    verified_ = false;
    if (!valid_drive(original_drive_ma_))
        return {false, false, "MCU original drive is unknown; output state could not be read back.", {}};
    try {
        std::atomic_bool cancelled{false};
        // A stale USB handle cannot recover itself. Re-resolve the fixed identity on every stop retry.
        reopen_bound_port(cancelled);
        auto details = response_json(execute(mcu_cleanup_command(original_drive_ma_), 2500, cancelled, pump));
        const bool low = number_is(details, "level", 0);
        const bool restored = number_is(details, "drive", original_drive_ma_);
        const bool ok = valid_mcu_cleanup_response(details, original_drive_ma_);
        details.insert("originalDriveMa", original_drive_ma_);
        details.insert("driveRestored", restored);
        details.insert("usbSerial", QLatin1String(board_serial));
        details.insert("port", port_path_);
        low_confirmed_ = ok;
        if (ok)
            continuous_requested_ = false;
        return {
            ok, low,
            ok ? QStringLiteral("PIO and watchdog stopped; GP2 LOW and original drive confirmed.")
               : QStringLiteral("MCU cleanup did not confirm stopped PIO/watchdog, LOW and original drive."),
            details};
    } catch (const std::exception &e) {
        return failure(e);
    }
}
} // namespace gui::multicam
