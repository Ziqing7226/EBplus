#include "multicam/mcu_transport.h"

#include <QJsonDocument>
#include <QRegularExpression>
#include <gtest/gtest.h>
#include <stdexcept>

using namespace gui::multicam;

namespace {
constexpr const char *approved_sha = "06b46f189635824b2ded6baeccf2794e3e52a78b004f1413fb8c00aac4a3651b";

QJsonObject running_response(int original_drive = 4) {
    return {{"mode", "continuous"},
            {"gpio", 2},
            {"state_machine", 0},
            {"active", true},
            {"running", true},
            {"watchdog_active", true},
            {"watchdog_expired", false},
            {"watchdog_timeout_ms", 8000},
            {"watchdog_period_ms", 250},
            {"heartbeat_age_ms", 0},
            {"high_us", 1000000},
            {"low_us", 1000000},
            {"nominal_hz", 0.5},
            {"drive_strength_ma", 12},
            {"restore_drive_ma_on_stop", original_drive},
            {"sha256", approved_sha},
            {"persistent_boot_change", false}};
}

QJsonObject stopped_response(int original_drive = 4) {
    return {{"gpio", 2},       {"state_machine", 0},      {"level", 0}, {"drive", original_drive},
            {"active", false}, {"watchdog_active", false}};
}

// Intentionally implements only the pre-continuous interface.
class LegacyMcu final : public McuTransport {
  public:
    McuResult check(const QString &, std::atomic_bool &, const Pump &) override { return {}; }
    McuResult burst(int, std::atomic_bool &, const Pump &) override { return {}; }
    McuResult cleanup(const Pump &) override { return {}; }
};

TEST(MultiCameraMcuContinuous, ExistingFakeTransportsDefaultToUnsupportedWithoutPumping) {
    LegacyMcu fake;
    McuTransport &transport = fake;
    std::atomic_bool cancelled{false};
    int pumps = 0;
    const auto pump = [&] { ++pumps; };
    for (const auto &result :
         {transport.start_continuous(cancelled, pump), transport.keepalive_continuous(cancelled, pump)}) {
        EXPECT_FALSE(result.ok);
        EXPECT_FALSE(result.low_confirmed);
        EXPECT_TRUE(result.reason.contains("not supported"));
    }
    EXPECT_EQ(pumps, 0);
}

TEST(MultiCameraMcuContinuous, UnverifiedTransportNeverStartsOrKeepsAlive) {
    std::atomic_bool cancelled{false};
    int pumps = 0;
    for (int setting : {0, 12}) {
        SerialMcuTransport transport(setting);
        EXPECT_FALSE(transport.start_continuous(cancelled, [&] { ++pumps; }).ok);
        EXPECT_FALSE(transport.keepalive_continuous(cancelled, [&] { ++pumps; }).ok);
        EXPECT_FALSE(transport.cleanup([&] { ++pumps; }).ok);
        cancelled = true;
        EXPECT_FALSE(transport.start_continuous(cancelled, [&] { ++pumps; }).ok);
        cancelled = false;
    }
    EXPECT_EQ(pumps, 0);
}

TEST(MultiCameraMcuContinuous, DefaultBurstResponseStillRequiresUnchangedDrive) {
    const QJsonObject response{{"output", "low"},
                               {"gpio", 2},
                               {"pulses", 3},
                               {"expected_edges", 6},
                               {"high_us", 1000000},
                               {"low_us", 1000000},
                               {"drive_strength_mA_before", 4},
                               {"drive_strength_mA_during", 4},
                               {"drive_strength_mA_after", 4}};
    EXPECT_TRUE(valid_finite_burst_response(response, 3, 4));
    EXPECT_FALSE(valid_finite_burst_response(response, 3, 4, 12));
}

TEST(MultiCameraMcuContinuous, CommandsOnlyAcceptRealOriginalDriveSettings) {
    for (int ma : {2, 4, 8, 12}) {
        EXPECT_FALSE(continuous_start_command(ma).isEmpty());
        EXPECT_FALSE(continuous_keepalive_command(ma).isEmpty());
        EXPECT_TRUE(mcu_cleanup_command(ma).contains("_ebplus_drives.index(" + QByteArray::number(ma) + ")"));
    }
    for (int ma : {-1, 0, 1, 3, 16}) {
        EXPECT_THROW(continuous_start_command(ma), std::invalid_argument);
        EXPECT_THROW(continuous_keepalive_command(ma), std::invalid_argument);
        EXPECT_THROW(mcu_cleanup_command(ma), std::invalid_argument);
    }
}

TEST(MultiCameraMcuPio, PrepareReleasesKnownProgramsBeforeReplacingTheirReferences) {
    const auto code = mcu_prepare_command();
    const auto guard = code.indexOf("if any(rp2.StateMachine(i).active() for i in range(4))");
    const auto finite = code.indexOf("_ebplus_release_pio_program(getattr(_cached,'_finite_burst',None))");
    const auto continuous =
        code.indexOf("_ebplus_release_pio_program(globals().get('_ebplus_continuous_program'))");
    const auto reload = code.indexOf("sys.modules.pop('rp2350_burst',None)");
    ASSERT_GE(guard, 0);
    ASSERT_GE(finite, 0);
    ASSERT_GE(continuous, 0);
    EXPECT_LT(guard, finite);
    EXPECT_LT(guard, continuous);
    EXPECT_LT(finite, reload);
    EXPECT_LT(continuous, reload);
    EXPECT_LT(code.indexOf(approved_sha), finite);
}

TEST(MultiCameraMcuPio, FiniteBurstReleasesItsProgramEvenWhenBurstRaises) {
    for (int pulses : {1, 3}) {
        for (int drive : {0, 12}) {
            const auto code = finite_burst_command(pulses, drive);
            EXPECT_TRUE(
                code.contains("try:\n    _result=b.burst(" + QByteArray::number(pulses) + ",1000000,"));
            const auto release = code.indexOf("finally:\n    _ebplus_release_pio_program(b._finite_burst)");
            EXPECT_GE(release, 0);
            EXPECT_LT(release, code.indexOf("print('EBPLUS_MCU '"));
            EXPECT_EQ(code.count("b.burst("), 1);
        }
    }
    EXPECT_THROW(finite_burst_command(2, 12), std::invalid_argument);
    EXPECT_THROW(finite_burst_command(3, 4), std::invalid_argument);
}

TEST(MultiCameraMcuPio, ReleaseIsSpecificIdleOnlyAndDoesNotHideUnexpectedErrors) {
    for (const auto &code : {mcu_prepare_command(), finite_burst_command(3, 12), continuous_start_command(4),
                             mcu_cleanup_command(4)}) {
        EXPECT_TRUE(code.contains("_r.PIO(0).remove_program(_program)"));
        EXPECT_TRUE(code.contains("if any(_r.StateMachine(i).active() for i in range(4))"));
        EXPECT_TRUE(code.contains("if str(_e)!='prog not in instruction memory':\n            raise"));
        EXPECT_FALSE(code.contains(".remove_program()"));
        EXPECT_FALSE(code.contains("machine.reset"));
        EXPECT_FALSE(code.contains("machine.soft_reset"));
    }
    const auto cleanup = mcu_cleanup_command(4);
    EXPECT_LT(cleanup.indexOf("machine.mem32[_ebplus_gp2_pad]="),
              cleanup.indexOf("_ebplus_release_pio_program(getattr(sys.modules.get("));
}

TEST(MultiCameraMcuContinuous, StartChecksFirmwareAndIdleStateBeforeChangingDrive) {
    const auto code = continuous_start_command(4);
    const auto change = code.indexOf("b._set_drive_ma(12)");
    ASSERT_GE(change, 0);
    for (const QByteArray &guard :
         {QByteArray(approved_sha), QByteArray("Expected RP2350"), QByteArray("b.GPIO!=2"),
          QByteArray("b._pin.value()!=0"), QByteArray("b._drive_ma()!=4"), QByteArray("PIO0 SM0 is busy"),
          QByteArray("Continuous watchdog is already armed")}) {
        SCOPED_TRACE(guard.constData());
        const auto at = code.indexOf(guard);
        EXPECT_GE(at, 0);
        EXPECT_LT(at, change);
    }
}

TEST(MultiCameraMcuContinuous, StartMatchesReferencePioWaveformWithoutBusyWaiting) {
    const auto code = continuous_start_command(4);
    EXPECT_TRUE(code.contains("freq=1000000,set_base=b._pin"));
    EXPECT_TRUE(code.contains(".put(1000000-3)"));
    EXPECT_TRUE(code.contains("set(pins,1)\n        mov(x,isr)"));
    EXPECT_TRUE(code.contains("set(pins,0)\n        mov(x,isr)"));
    EXPECT_TRUE(code.contains("jmp(x_dec,'high_delay')"));
    EXPECT_TRUE(code.contains("jmp(x_dec,'low_delay')"));
    EXPECT_TRUE(code.contains("'high_us':1000000,'low_us':1000000,'nominal_hz':0.5"));
    EXPECT_TRUE(code.contains("if '_ebplus_continuous_program' not in globals():"));
    EXPECT_FALSE(code.contains("while "));
    EXPECT_FALSE(code.contains("sleep("));
}

TEST(MultiCameraMcuContinuous, WatchdogIsArmedBeforeDriveAndPioStart) {
    const auto code = continuous_start_command(4);
    const auto arm = code.indexOf("['timer'].init(period=250,mode=machine.Timer.PERIODIC");
    const auto flag = code.indexOf("['watchdog_active']=True");
    const auto drive = code.indexOf("b._set_drive_ma(12)");
    const auto start = code.indexOf("['sm'].active(1)");
    ASSERT_GE(arm, 0);
    EXPECT_LT(arm, flag);
    EXPECT_LT(flag, drive);
    EXPECT_LT(drive, start);
    EXPECT_TRUE(code.contains("machine.Timer(-1)"));
    EXPECT_TRUE(code.contains("_t.ticks_diff(_t.ticks_ms(),_s['heartbeat_ms'])>=8000"));
    EXPECT_TRUE(code.contains("_s['expired']=True\n        _stop()"));
    EXPECT_TRUE(code.contains("except:\n    _ebplus_continuous_stop()\n    raise"));
}

TEST(MultiCameraMcuContinuous, WatchdogShutdownAttemptsAllSafetyActionsWithFinally) {
    const auto code = continuous_start_command(4);
    const auto stop = code.mid(code.indexOf("def _ebplus_continuous_stop("),
                               code.indexOf("def _ebplus_continuous_watchdog(") -
                                   code.indexOf("def _ebplus_continuous_stop("));
    const auto pio = stop.indexOf("_r.StateMachine(0).active(0)");
    const auto timer = stop.indexOf("['timer'].deinit()");
    const auto low = stop.indexOf("_b._pin.init(_m.Pin.OUT,value=0)");
    const auto restore = stop.indexOf("_b._set_drive_ma(_s['restore_drive_ma'])");
    ASSERT_GE(pio, 0);
    EXPECT_LT(pio, timer);
    EXPECT_LT(timer, low);
    EXPECT_LT(low, restore);
    EXPECT_EQ(stop.count("finally:"), 4);
    EXPECT_LT(restore, stop.indexOf("_release(_s.get('program'))"));
}

TEST(MultiCameraMcuContinuous, KeepaliveChecksDeadlineAndReadbackBeforeRefreshingHeartbeat) {
    const auto code = continuous_keepalive_command(4);
    const auto refresh = code.indexOf("_s['heartbeat_ms']=time.ticks_ms()");
    ASSERT_GE(refresh, 0);
    for (const QByteArray &guard :
         {QByteArray(approved_sha), QByteArray("not _s['running']"), QByteArray("not _s['watchdog_active']"),
          QByteArray("_s['expired']"), QByteArray("_s['restore_drive_ma']!=4"),
          QByteArray("time.ticks_diff(time.ticks_ms(),_s['heartbeat_ms'])>=8000"),
          QByteArray("not rp2.StateMachine(0).active()"), QByteArray("b._drive_ma()!=12")}) {
        SCOPED_TRACE(guard.constData());
        const auto at = code.indexOf(guard);
        EXPECT_GE(at, 0);
        EXPECT_LT(at, refresh);
    }
    EXPECT_FALSE(code.contains(".active(1)"));
    EXPECT_FALSE(code.contains("._set_drive_ma(12)"));
    EXPECT_EQ(code.count("_ebplus_continuous_stop()"), 2);
}

TEST(MultiCameraMcuContinuous, CommandsNeverWriteFilesOrChangePersistentConfiguration) {
    for (const auto &code :
         {continuous_start_command(4), continuous_keepalive_command(4), mcu_cleanup_command(4)}) {
        for (const QByteArray &forbidden :
             {QByteArray("boot.py"), QByteArray("main.py"), QByteArray("'wb'"), QByteArray("'w'"),
              QByteArray(".write("), QByteArray("os.remove"), QByteArray("machine.reset"),
              QByteArray("machine.WDT"), QByteArray("mpremote"), QByteArray("subprocess")}) {
            SCOPED_TRACE(forbidden.constData());
            EXPECT_FALSE(code.contains(forbidden));
        }
        EXPECT_FALSE(code.contains('\x04'));
    }
    EXPECT_TRUE(continuous_start_command(4).contains("open('/rp2350_burst.py','rb')"));
}

TEST(MultiCameraMcuContinuous, CleanupIsStandaloneAndChecksStoppedPioAndWatchdog) {
    const auto code = mcu_cleanup_command(8);
    const auto pio = code.indexOf("rp2.StateMachine(0).active(0)");
    const auto timer = code.indexOf("['timer'].deinit()");
    const auto low = code.indexOf("machine.Pin(2,machine.Pin.OUT,value=0)");
    const auto restore = code.indexOf("machine.mem32[_ebplus_gp2_pad]=");
    const auto report = code.indexOf("print('EBPLUS_MCU '");
    ASSERT_GE(pio, 0);
    EXPECT_LT(pio, timer);
    EXPECT_LT(timer, low);
    EXPECT_LT(low, restore);
    EXPECT_LT(restore, report);
    EXPECT_EQ(code.count("finally:"), 3);
    EXPECT_FALSE(code.contains("_ebplus_continuous_stop()"));
    EXPECT_FALSE(code.contains("hashlib"));
    EXPECT_FALSE(code.contains("import rp2350_burst"));
    EXPECT_FALSE(code.contains("b."));
    EXPECT_FALSE(code.contains("open("));
    EXPECT_TRUE(code.contains("'RP2350' not in os.uname().machine"));
    EXPECT_TRUE(code.contains("_ebplus_gp2_pad=0x4003800C"));
    EXPECT_TRUE(code.contains("_ebplus_drives.index(8)"));
    EXPECT_TRUE(code.contains("'active':bool(rp2.StateMachine(0).active())"));
    EXPECT_TRUE(code.contains("'watchdog_active':bool(_s and _s.get('watchdog_active',False))"));
}

TEST(MultiCameraMcuContinuous, ValidReadbackAcceptsAllOriginalDriveSettings) {
    for (int ma : {2, 4, 8, 12}) {
        EXPECT_TRUE(valid_continuous_response(running_response(ma), ma));
        EXPECT_TRUE(valid_mcu_cleanup_response(stopped_response(ma), ma));
    }
    EXPECT_FALSE(valid_continuous_response(running_response(), 0));
    EXPECT_FALSE(valid_mcu_cleanup_response(stopped_response(), -1));
}

TEST(MultiCameraMcuContinuous, EveryContinuousReadbackFieldIsRequiredAndTyped) {
    const auto valid = running_response();
    for (auto it = valid.begin(); it != valid.end(); ++it) {
        SCOPED_TRACE(it.key().toStdString());
        auto missing = valid;
        missing.remove(it.key());
        EXPECT_FALSE(valid_continuous_response(missing, 4));
        auto null = valid;
        null.insert(it.key(), QJsonValue::Null);
        EXPECT_FALSE(valid_continuous_response(null, 4));
        auto wrong_type = valid;
        wrong_type.insert(it.key(), it.value().isString() ? QJsonValue(1) : QJsonValue("true"));
        EXPECT_FALSE(valid_continuous_response(wrong_type, 4));
    }
}

TEST(MultiCameraMcuContinuous, InvalidContinuousStateNeverPassesReadback) {
    const QJsonObject bad{{"mode", "burst"},
                          {"gpio", 3},
                          {"state_machine", 1},
                          {"active", false},
                          {"running", false},
                          {"watchdog_active", false},
                          {"watchdog_expired", true},
                          {"watchdog_timeout_ms", 8001},
                          {"watchdog_period_ms", 1000},
                          {"high_us", 50000},
                          {"low_us", 999999},
                          {"nominal_hz", 1.0},
                          {"drive_strength_ma", 4},
                          {"restore_drive_ma_on_stop", 12},
                          {"sha256", "unapproved"},
                          {"persistent_boot_change", true}};
    for (auto it = bad.begin(); it != bad.end(); ++it) {
        SCOPED_TRACE(it.key().toStdString());
        auto details = running_response();
        details.insert(it.key(), it.value());
        EXPECT_FALSE(valid_continuous_response(details, 4));
    }
}

TEST(MultiCameraMcuContinuous, ExpiredOrNegativeHeartbeatCannotBeAccepted) {
    for (int age : {-1, 8000, 8001, 60000}) {
        auto details = running_response();
        details.insert("heartbeat_age_ms", age);
        EXPECT_FALSE(valid_continuous_response(details, 4));
    }
    auto details = running_response();
    details.insert("heartbeat_age_ms", 7999);
    EXPECT_TRUE(valid_continuous_response(details, 4));
}

TEST(MultiCameraMcuContinuous, CleanupRequiresEveryReadbackFieldAndOriginalDrive) {
    const auto valid = stopped_response();
    for (auto it = valid.begin(); it != valid.end(); ++it) {
        SCOPED_TRACE(it.key().toStdString());
        auto details = valid;
        details.remove(it.key());
        EXPECT_FALSE(valid_mcu_cleanup_response(details, 4));
        details = valid;
        details.insert(it.key(), it.value().isBool() ? QJsonValue(true) : QJsonValue(12));
        EXPECT_FALSE(valid_mcu_cleanup_response(details, 4));
        details.insert(it.key(), "false");
        EXPECT_FALSE(valid_mcu_cleanup_response(details, 4));
    }
}

TEST(MultiCameraMcuContinuous, LowWithoutStoppedPioOrWatchdogIsNotSuccessfulCleanup) {
    auto details = stopped_response();
    details.insert("active", true);
    EXPECT_FALSE(valid_mcu_cleanup_response(details, 4));
    details = stopped_response();
    details.insert("watchdog_active", true);
    EXPECT_FALSE(valid_mcu_cleanup_response(details, 4));
    details = stopped_response();
    details.insert("drive", 12);
    EXPECT_FALSE(valid_mcu_cleanup_response(details, 4));
}

class EmulatedConnection final : public McuConnection {
  public:
    QVector<McuPort> listed{{"/dev/cu.old", "36f9f437460d3ba8", 0x2e8a, 0x0005}};
    QStringList opened;
    QVector<QByteArray> commands;
    int enumerations = 0, starts = 0, stops = 0, firmware_checks = 0, bursts = 0;
    QString burst_error;
    int drive = 4;
    bool active = false, watchdog = false, session = false, firmware_valid = true;
    bool lose_stop_reply = false, bad_stop_readback = false;
    QJsonValue saved_drive = 4;

    QJsonObject state() const {
        return {{"machine", "Waveshare RP2350A Board with RP2350"},
                {"gpio", 2},
                {"level", 0},
                {"drive", drive},
                {"active", active},
                {"watchdog_active", watchdog},
                {"session_present", session},
                {"session_restore_drive_ma", session ? saved_drive : QJsonValue{}}};
    }
    QVector<McuPort> ports() override {
        ++enumerations;
        return listed;
    }
    void open(const QString &path) override { opened.append(path); }
    QByteArray execute(const QByteArray &code, int, std::atomic_bool &cancelled,
                       const McuTransport::Pump &pump) override {
        if (cancelled)
            throw std::runtime_error("emulated cancellation");
        if (pump)
            pump();
        commands.append(code);
        QJsonObject result;
        if (code == mcu_state_command()) {
            result = state();
        } else if (code.contains("machine.mem32[_ebplus_gp2_pad]=")) {
            const auto match =
                QRegularExpression("_ebplus_drives\\.index\\(([248]|12)\\)").match(QString::fromUtf8(code));
            if (!match.hasMatch())
                throw std::runtime_error("emulator: missing restore drive");
            ++stops;
            active = watchdog = false;
            drive = match.captured(1).toInt();
            if (lose_stop_reply)
                throw std::runtime_error("emulated lost stop reply");
            result = stopped_response(bad_stop_readback ? 12 : drive);
        } else if (code.contains("'burst_callable':callable(b.burst)")) {
            ++firmware_checks;
            if (!firmware_valid)
                throw std::runtime_error("emulated firmware hash mismatch");
            result = {{"machine", "RP2350"},
                      {"gpio", 2},
                      {"level", 0},
                      {"drive", drive},
                      {"sha256", approved_sha},
                      {"burst_callable", true}};
        } else if (code.contains("_result=b.burst(")) {
            ++bursts;
            if (!burst_error.isEmpty())
                throw std::runtime_error(burst_error.toStdString());
            const int pulses = code.contains("b.burst(3,") ? 3 : 1;
            result = {{"output", "low"},
                      {"gpio", 2},
                      {"pulses", pulses},
                      {"expected_edges", 2 * pulses},
                      {"high_us", 1000000},
                      {"low_us", 1000000},
                      {"drive_strength_mA_before", drive},
                      {"drive_strength_mA_during", 12},
                      {"drive_strength_mA_after", drive}};
        } else if (code.contains("['sm'].active(1)")) {
            ++starts;
            saved_drive = drive;
            drive = 12;
            active = watchdog = session = true;
            result = running_response(saved_drive.toInt());
        } else {
            throw std::runtime_error("emulator: unexpected command");
        }
        return "EBPLUS_MCU " + QJsonDocument(result).toJson(QJsonDocument::Compact) + '\n';
    }
};

class MultiCameraMcuRecovery : public ::testing::Test {
  protected:
    EmulatedConnection *io = new EmulatedConnection;
    SerialMcuTransport transport{12, std::unique_ptr<McuConnection>(io)};
    std::atomic_bool cancelled{false};
    McuResult check() { return transport.check("36f9f437460d3ba8", cancelled, {}); }
};

TEST_F(MultiCameraMcuRecovery, FixedUsbIdentityIsRequiredBeforeOpening) {
    io->listed = {{"wrong-vid", "36f9f437460d3ba8", 0x1234, 0x0005},
                  {"wrong-pid", "36f9f437460d3ba8", 0x2e8a, 0x0006},
                  {"wrong-serial", "another-board", 0x2e8a, 0x0005}};
    EXPECT_FALSE(check().ok);
    EXPECT_EQ(io->enumerations, 1);
    EXPECT_TRUE(io->opened.isEmpty());
    EXPECT_TRUE(io->commands.isEmpty());
}

TEST_F(MultiCameraMcuRecovery, AliasesCollapseButTwoPhysicalPathsRemainAmbiguous) {
    io->listed = {{"/dev/tty.usbmodem1", "36f9f437460d3ba8", 0x2e8a, 0x0005},
                  {"/dev/cu.usbmodem1", "36f9f437460d3ba8", 0x2e8a, 0x0005}};
    ASSERT_TRUE(check().ok);
    EXPECT_EQ(io->opened, QStringList{"/dev/cu.usbmodem1"});
    io->listed.append({"/dev/cu.usbmodem2", "36f9f437460d3ba8", 0x2e8a, 0x0005});
    EXPECT_FALSE(transport.cleanup({}).ok);
    EXPECT_EQ(io->opened.size(), 1);
    EXPECT_EQ(io->stops, 0);
}

TEST_F(MultiCameraMcuRecovery, StopReopensChangedUsbPathWithoutResettingBaselineOrRestarting) {
    ASSERT_TRUE(check().ok);
    ASSERT_TRUE(transport.start_continuous(cancelled, {}).ok);
    io->listed.clear();
    EXPECT_FALSE(transport.cleanup({}).ok);
    EXPECT_FALSE(check().ok);
    io->listed = {{"/dev/cu.new", "36f9f437460d3ba8", 0x2e8a, 0x0005}};
    auto stopped = transport.cleanup({});
    ASSERT_TRUE(stopped.ok) << stopped.reason.toStdString();
    EXPECT_TRUE(stopped.low_confirmed);
    EXPECT_EQ(stopped.details.value("port").toString(), "/dev/cu.new");
    EXPECT_EQ(io->opened.last(), "/dev/cu.new");
    EXPECT_EQ(io->drive, 4);
    EXPECT_EQ(io->starts, 1);
    EXPECT_EQ(io->firmware_checks, 1);
    EXPECT_FALSE(transport.start_continuous(cancelled, {}).ok);
    EXPECT_FALSE(transport.keepalive_continuous(cancelled, {}).ok);
    EXPECT_FALSE(transport.burst(1, cancelled, {}).ok);
    EXPECT_FALSE(io->active);
}

TEST_F(MultiCameraMcuRecovery, StopDoesNotTouchAReplacementBoardWithDifferentIdentity) {
    ASSERT_TRUE(check().ok);
    ASSERT_TRUE(transport.start_continuous(cancelled, {}).ok);
    const auto executed = io->commands.size();
    const auto opened = io->opened.size();
    io->listed = {{"/dev/cu.old", "another-board", 0x2e8a, 0x0005}};
    EXPECT_FALSE(transport.cleanup({}).ok);
    EXPECT_EQ(io->commands.size(), executed);
    EXPECT_EQ(io->opened.size(), opened);
    EXPECT_EQ(io->stops, 0);
    io->listed[0].serial = "36f9f437460d3ba8";
    ASSERT_TRUE(transport.cleanup({}).ok);
    EXPECT_EQ(io->drive, 4);
    EXPECT_EQ(io->starts, 1);
}

TEST_F(MultiCameraMcuRecovery, UsbPowerResetNeedsNeitherSessionNorFirmwareGlobalsForStop) {
    io->drive = 8;
    ASSERT_TRUE(check().ok);
    ASSERT_TRUE(transport.start_continuous(cancelled, {}).ok);
    io->session = io->active = io->watchdog = false;
    io->drive = 4;
    io->firmware_valid = false;
    io->listed[0].path = "/dev/cu.after-reset";
    const auto stopped = transport.cleanup({});
    ASSERT_TRUE(stopped.ok);
    EXPECT_EQ(io->drive, 8);
    EXPECT_EQ(stopped.details.value("originalDriveMa").toInt(), 8);
    EXPECT_EQ(io->firmware_checks, 1);
    EXPECT_FALSE(io->commands.last().contains("b."));
    EXPECT_FALSE(io->commands.last().contains("import rp2350_burst"));
    EXPECT_TRUE(
        io->commands.last().contains("getattr(sys.modules.get('rp2350_burst'),'_finite_burst',None)"));
    EXPECT_EQ(io->starts, 1);
}

TEST_F(MultiCameraMcuRecovery, FiniteEnomemExplainsRecoveryWithoutResetOrAutomaticPulseRetry) {
    ASSERT_TRUE(check().ok);
    io->burst_error = "OSError: [Errno 12] ENOMEM";
    const auto result = transport.burst(1, cancelled, {});
    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.low_confirmed);
    EXPECT_TRUE(result.reason.contains("power-cycle the RP2350"));
    EXPECT_TRUE(result.reason.contains("No automatic reset or pulse retry"));
    EXPECT_EQ(io->bursts, 1);
    EXPECT_FALSE(transport.burst(1, cancelled, {}).ok);
    EXPECT_EQ(io->bursts, 1);
    ASSERT_TRUE(transport.cleanup({}).ok);
    EXPECT_EQ(io->bursts, 1);
    EXPECT_FALSE(transport.burst(1, cancelled, {}).ok);
    for (const auto &command : io->commands) {
        EXPECT_FALSE(command.contains("machine.reset"));
        EXPECT_FALSE(command.contains(".remove_program()"));
    }
}

TEST_F(MultiCameraMcuRecovery, FiniteCheckAndBurstUseTheTestedPioLifecycleCommands) {
    ASSERT_TRUE(check().ok);
    EXPECT_EQ(io->commands.last(), mcu_prepare_command());
    ASSERT_TRUE(transport.burst(1, cancelled, {}).ok);
    EXPECT_EQ(io->commands.last(), finite_burst_command(1, 12));
    ASSERT_TRUE(transport.burst(3, cancelled, {}).ok);
    EXPECT_EQ(io->commands.last(), finite_burst_command(3, 12));
    EXPECT_EQ(io->bursts, 2);
}

TEST_F(MultiCameraMcuRecovery, FailedStopReadbackDoesNotClearPendingOrAdoptTwelve) {
    ASSERT_TRUE(check().ok);
    ASSERT_TRUE(transport.start_continuous(cancelled, {}).ok);
    io->bad_stop_readback = true;
    EXPECT_FALSE(transport.cleanup({}).ok);
    EXPECT_FALSE(check().ok);
    EXPECT_FALSE(transport.start_continuous(cancelled, {}).ok);
    io->bad_stop_readback = false;
    EXPECT_TRUE(transport.cleanup({}).ok);
    EXPECT_EQ(io->drive, 4);
    EXPECT_EQ(io->starts, 1);
}

TEST_F(MultiCameraMcuRecovery, LostStopReplyCanBeRetriedAfterASecondPathChange) {
    ASSERT_TRUE(check().ok);
    ASSERT_TRUE(transport.start_continuous(cancelled, {}).ok);
    io->lose_stop_reply = true;
    EXPECT_FALSE(transport.cleanup({}).ok);
    EXPECT_FALSE(io->active);
    io->lose_stop_reply = false;
    io->listed[0].path = "/dev/cu.retry";
    EXPECT_TRUE(transport.cleanup({}).ok);
    EXPECT_EQ(io->opened.last(), "/dev/cu.retry");
    EXPECT_EQ(io->drive, 4);
    EXPECT_EQ(io->starts, 1);
}

TEST_F(MultiCameraMcuRecovery, StoppedResidualRamRestoresFourRatherThanAdoptingStrandedTwelve) {
    io->session = true;
    io->drive = 12;
    io->saved_drive = 4;
    const auto checked = check();
    ASSERT_TRUE(checked.ok) << checked.reason.toStdString();
    EXPECT_EQ(io->drive, 4);
    EXPECT_EQ(io->stops, 1);
    EXPECT_EQ(io->starts, 0);
    EXPECT_EQ(checked.details.value("drive").toInt(), 4);
    EXPECT_TRUE(checked.details.contains("residualRecovery"));
    ASSERT_EQ(io->commands.size(), 3);
    EXPECT_EQ(io->commands[0], mcu_state_command());
    EXPECT_EQ(io->commands[1], mcu_cleanup_command(4));
    EXPECT_TRUE(io->commands[2].contains("'burst_callable'"));
}

TEST_F(MultiCameraMcuRecovery, ActiveOrphanIsStoppedBeforeFirmwareVerification) {
    io->session = io->active = io->watchdog = true;
    io->saved_drive = 8;
    io->drive = 12;
    ASSERT_TRUE(check().ok);
    EXPECT_FALSE(io->active);
    EXPECT_FALSE(io->watchdog);
    EXPECT_EQ(io->drive, 8);
    EXPECT_EQ(io->starts, 0);
    EXPECT_EQ(io->stops, 1);
}

TEST_F(MultiCameraMcuRecovery, MalformedResidualOriginalFailsClosedBeforeImportOrStart) {
    io->session = true;
    io->drive = 12;
    for (const auto &invalid :
         {QJsonValue{}, QJsonValue(true), QJsonValue("4"), QJsonValue(4.5), QJsonValue(0), QJsonValue(16)}) {
        io->saved_drive = invalid;
        EXPECT_FALSE(check().ok);
        EXPECT_FALSE(transport.cleanup({}).ok);
        EXPECT_FALSE(transport.start_continuous(cancelled, {}).ok);
    }
    EXPECT_EQ(io->drive, 12);
    EXPECT_EQ(io->firmware_checks, 0);
    EXPECT_EQ(io->starts, 0);
    EXPECT_EQ(io->stops, 0);
}

TEST_F(MultiCameraMcuRecovery, FailedResidualRecoveryRetainsOriginalForStopRetry) {
    io->session = true;
    io->drive = 12;
    io->saved_drive = 4;
    io->bad_stop_readback = true;
    EXPECT_FALSE(check().ok);
    EXPECT_EQ(io->firmware_checks, 0);
    io->bad_stop_readback = false;
    EXPECT_TRUE(transport.cleanup({}).ok);
    EXPECT_EQ(io->drive, 4);
    EXPECT_EQ(io->starts, 0);
}

TEST_F(MultiCameraMcuRecovery, ConflictingRamOriginalCannotReplaceKnownHostBaseline) {
    ASSERT_TRUE(check().ok);
    io->session = true;
    io->drive = 12;
    io->saved_drive = 8;
    EXPECT_FALSE(check().ok);
    EXPECT_EQ(io->firmware_checks, 1);
    EXPECT_TRUE(transport.cleanup({}).ok);
    EXPECT_EQ(io->drive, 4);
    EXPECT_EQ(io->starts, 0);
}

TEST_F(MultiCameraMcuRecovery, UnknownActiveOutputCannotBecomeAFreshBaseline) {
    io->active = true;
    io->drive = 12;
    EXPECT_FALSE(check().ok);
    EXPECT_FALSE(transport.cleanup({}).ok);
    EXPECT_EQ(io->firmware_checks, 0);
    EXPECT_EQ(io->stops, 0);
    EXPECT_EQ(io->starts, 0);
}

TEST_F(MultiCameraMcuRecovery, WrongSerialAndCancellationNeverEnumerate) {
    EXPECT_FALSE(transport.check("another-board", cancelled, {}).ok);
    cancelled = true;
    EXPECT_FALSE(check().ok);
    EXPECT_EQ(io->enumerations, 0);
    EXPECT_TRUE(io->commands.isEmpty());
}

TEST(MultiCameraMcuRecoveryCommands, InspectionDoesNotImportFirmwareOrConfigurePins) {
    const auto code = mcu_state_command();
    EXPECT_FALSE(code.contains("rp2350_burst"));
    EXPECT_FALSE(code.contains("Pin.OUT"));
    EXPECT_FALSE(code.contains(".active(1)"));
    EXPECT_FALSE(code.contains("machine.mem32[_ebplus_gp2_pad]="));
    EXPECT_TRUE(code.contains("type(_s.get('restore_drive_ma')) is not int"));
    EXPECT_TRUE(code.contains("'session_restore_drive_ma':_s['restore_drive_ma']"));
}

TEST(MultiCameraMcuRecoveryCommands, StateReadbackIsStrictlyTypedAndNeverRoundsRestoreDrive) {
    EmulatedConnection io;
    io.session = true;
    io.drive = 12;
    EXPECT_EQ(restore_drive_from_mcu_state(io.state()), 4);
    for (const auto &key : {"machine", "gpio", "level", "drive", "active", "watchdog_active",
                            "session_present", "session_restore_drive_ma"}) {
        auto details = io.state();
        details.remove(key);
        EXPECT_THROW(restore_drive_from_mcu_state(details), std::runtime_error) << key;
    }
    EXPECT_THROW(restore_drive_from_mcu_state(io.state(), 8), std::runtime_error);
    io.saved_drive = 4.1;
    EXPECT_THROW(restore_drive_from_mcu_state(io.state()), std::runtime_error);
}

TEST(MultiCameraMcuRecoveryCommands, KnownBaselineWinsAfterPowerResetAndResidualBaselineWinsWhenPresent) {
    for (const int original : {2, 4, 8, 12}) {
        EmulatedConnection io;
        io.drive = 4;
        EXPECT_EQ(restore_drive_from_mcu_state(io.state(), original), original);
        io.drive = 12;
        io.session = true;
        io.saved_drive = original;
        EXPECT_EQ(restore_drive_from_mcu_state(io.state()), original);
    }
    const auto cleanup = mcu_cleanup_command(4);
    EXPECT_LT(cleanup.indexOf("Expected RP2350"), cleanup.indexOf("StateMachine(0).active(0)"));
    EXPECT_LT(cleanup.indexOf("Expected RP2350"), cleanup.indexOf("machine.mem32[_ebplus_gp2_pad]="));
}
} // namespace
