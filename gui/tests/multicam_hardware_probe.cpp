#include "multicam/camera_stream.h"
#include "multicam/mcu_transport.h"
#include "multicam/take.h"
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSerialPortInfo>
#include <QStorageInfo>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <set>
#include <stdexcept>
#include <thread>

using namespace gui::multicam;
namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) {
    interrupted = 1;
}
void require(bool ok, const QString &message) {
    if (!ok)
        throw std::runtime_error(message.toStdString());
}
QJsonArray edge_json(const QVector<TriggerEdge> &edges) {
    QJsonArray a;
    for (const auto &e : edges)
        a.append(
            QJsonObject{{"timestampUs", e.timestamp_us}, {"polarity", e.polarity}, {"channel", e.channel}});
    return a;
}
QJsonObject mcu_json(const McuResult &r) {
    return {{"ok", r.ok}, {"lowConfirmed", r.low_confirmed}, {"reason", r.reason}, {"details", r.details}};
}
void output(const QJsonObject &o) {
    std::cout << QJsonDocument(o).toJson(QJsonDocument::Indented).constData() << std::flush;
}
} // namespace
int main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    QCommandLineParser cli;
    cli.setApplicationDescription("Explicit finite three-camera Trigger measurement; no project approval.");
    cli.addHelpOption();
    cli.addOption({"list", "List identities only; do not open cameras or the serial port."});
    cli.addOption(
        {"allow-trigger", "Confirm verified wiring/current limiting and authorize finite GP2 pulses."});
    cli.addOption({"drive-ma", "Explicit diagnostic override: 12 mA; restored after each burst.", "mA"});
    cli.addOption({"serials", "Left,Center,Right full serials, comma-separated.", "serials"});
    cli.addOption({"master", "Clock Master serial.", "serial"});
    cli.addOption({"reference", "Geometry reference serial (metadata only).", "serial"});
    cli.addOption({"layout-note", "User-confirmed physical arrangement; not a measured extrinsic.", "text"});
    cli.addOption({"output", "Parent directory for a new, preserved diagnostic Take.", "directory"});
    cli.process(app);
    std::signal(SIGINT, interrupt);
    std::signal(SIGTERM, interrupt);
    try {
        require(!cli.isSet("drive-ma") || cli.value("drive-ma") == "12",
                "The only supported explicit diagnostic drive override is 12 mA.");
        const int drive_ma = cli.isSet("drive-ma") ? 12 : 0;
        const auto devices = discover_cameras();
        QJsonArray found, ports;
        for (const auto &d : devices)
            found.append(QJsonObject{{"selection", d.selection}, {"serial", d.serial}});
        for (const auto &p : QSerialPortInfo::availablePorts())
            ports.append(QJsonObject{{"port", p.systemLocation()},
                                     {"serial", p.serialNumber()},
                                     {"vid", p.vendorIdentifier()},
                                     {"pid", p.productIdentifier()}});
        if (cli.isSet("list")) {
            output({{"cameras", found}, {"serialPorts", ports}, {"pulsesSent", 0}});
            return 0;
        }
        require(cli.isSet("allow-trigger"), "Use --list, or explicitly authorize --allow-trigger.");
        const auto serials = cli.value("serials").split(',');
        require(serials.size() == 3 && std::set<QString>(serials.begin(), serials.end()).size() == 3,
                "Specify three distinct Left,Center,Right serials.");
        const int master = serials.indexOf(cli.value("master"));
        require(master >= 0 && serials.contains(cli.value("reference")),
                "Select Master and geometry reference.");
        require(!cli.value("output").isEmpty(), "An evidence output directory is required.");
        std::array<CameraBinding, 3> bindings;
        const QString positions[]{"Left", "Center", "Right"};
        for (int i = 0; i < 3; ++i) {
            QVector<DeviceEntry> matches;
            for (const auto &d : devices)
                if (d.serial == serials[i])
                    matches.append(d);
            require(matches.size() == 1, "Missing or ambiguous camera identity: " + serials[i]);
            bindings[i] = {matches[0].selection, serials[i], positions[i]};
        }
        const auto directory = create_take_directory(cli.value("output"));
        QStorageInfo disk(directory);
        require(disk.isValid() && disk.bytesAvailable() > 2LL * 1024 * 1024 * 1024,
                "Need at least 2 GiB free for diagnostic RAW evidence.");
        std::cerr << "Evidence: " << directory.toStdString() << '\n';
        QJsonObject report{{"schemaVersion", 1},
                           {"diagnosticOnly", true},
                           {"projectAcceptance", "not_evaluated_no_threshold"},
                           {"createdUtc", QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)},
                           {"masterSerial", serials[master]},
                           {"referenceSerial", cli.value("reference")},
                           {"layoutDescription", cli.value("layout-note")},
                           {"requestedDriveMa", drive_ma ? QJsonValue(drive_ma) : QJsonValue(QJsonValue::Null)},
                           {"detectedDevices", found},
                           {"preambleRequested", 1},
                           {"formalPulsesRequested", 3}};
        std::array<std::unique_ptr<CameraStream>, 3> cameras;
        std::atomic_bool cancelled{false};
        SerialMcuTransport mcu(drive_ma);
        QElapsedTimer elapsed;
        elapsed.start();
        auto pump = [&] {
            cancelled = interrupted != 0;
            require(!cancelled && elapsed.elapsed() < 60000,
                    "Diagnostic cancelled or acquisition timed out.");
            for (auto &c : cameras)
                if (c)
                    c->pump();
        };
        auto wait_ms = [&](int ms) {
            const auto until = elapsed.elapsed() + ms;
            while (elapsed.elapsed() < until) {
                pump();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        };
        QString failure;
        bool sequence_complete = false;
        try {
            const auto checked = mcu.check("36f9f437460d3ba8", cancelled, pump);
            report.insert("controllerCheck", mcu_json(checked));
            require(checked.ok && checked.low_confirmed, checked.reason);
            for (int order = 0; order < 3; ++order) {
                const int i = (master + 1 + order) % 3;
                cameras[i] = make_hal_camera();
                cameras[i]->open(bindings[i], i == master);
            }
            for (int order = 0; order < 3; ++order) {
                const int i = (master + 1 + order) % 3;
                cameras[i]->start(QDir(directory).filePath("camera-" + serials[i] + ".raw"));
            }
            std::cerr << "Roles read back; settling clocks for 5 s.\n";
            wait_ms(5000);
            const auto preamble = mcu.burst(1, cancelled, pump);
            report.insert("preamble", mcu_json(preamble));
            require(preamble.ok && preamble.low_confirmed, preamble.reason);
            wait_ms(1000);
            QJsonArray preamble_edges;
            std::array<qint64, 3> after{{-1, -1, -1}};
            bool all_observed = true;
            for (int i = 0; i < 3; ++i) {
                const auto edges = cameras[i]->snapshot().triggers;
                preamble_edges.append(edge_json(edges));
                bool main_seen = false;
                for (const auto &e : edges) {
                    after[i] = std::max(after[i], e.timestamp_us);
                    main_seen = main_seen || e.channel == 0;
                }
                all_observed = all_observed && main_seen;
            }
            report.insert("preambleDecoded", preamble_edges);
            require(all_observed,
                    "At least one camera saw no preamble Trigger In records; stop before further pulses.");
            std::cerr << "Preamble observed in all three cameras; sending 3 formal pulses.\n";
            const auto formal = mcu.burst(3, cancelled, pump);
            report.insert("formal", mcu_json(formal));
            require(formal.ok && formal.low_confirmed, formal.reason);
            wait_ms(1000);
            std::array<QVector<TriggerEdge>, 3> edges;
            QJsonArray decoded;
            for (int i = 0; i < 3; ++i) {
                for (const auto &e : cameras[i]->snapshot().triggers)
                    if (e.timestamp_us > after[i])
                        edges[i].push_back(e);
                decoded.append(edge_json(edges[i]));
            }
            report.insert("formalDecoded", decoded);
            // A generous computational bound allows measurement, not project approval.
            const auto match = match_trigger_edges(edges, master, std::numeric_limits<double>::max(), 1);
            auto timing = match.to_json();
            timing.remove("verdict");
            timing.insert("uniqueSixEdgeMatch", match.verdict == Verdict::Passed);
            timing.insert("acceptanceThresholdUs", QJsonValue::Null);
            report.insert("timingMeasurement", timing);
            require(match.verdict == Verdict::Passed, match.reason);
            sequence_complete = true;
        } catch (const std::exception &e) {
            failure = QString::fromUtf8(e.what());
        }
        const auto low = mcu.cleanup([] {});
        report.insert("cleanup", mcu_json(low));
        if (!low.ok || !low.low_confirmed)
            failure += " Controller LOW could not be confirmed.";
        QJsonArray rows;
        Take take;
        take.directory = directory;
        for (int i = 0; i < 3; ++i) {
            if (!cameras[i])
                continue;
            try {
                cameras[i]->stop();
            } catch (const std::exception &e) {
                failure += QString::fromUtf8(e.what());
            }
            const auto s = cameras[i]->snapshot();
            const auto name = "camera-" + serials[i] + ".raw";
            rows.append(QJsonObject{{"serial", serials[i]},
                                    {"selection", bindings[i].selection},
                                    {"position", positions[i]},
                                    {"role", i == master ? "master" : "slave"},
                                    {"rawFile", name},
                                    {"eventCount", qint64(s.event_count)},
                                    {"width", s.width},
                                    {"height", s.height},
                                    {"firstTimestampUs", s.first_timestamp_us},
                                    {"lastTimestampUs", s.latest_timestamp_us},
                                    {"readback", s.metadata},
                                    {"timestampRegressions", qint64(s.regressions)},
                                    {"protocolErrors", qint64(s.protocol_errors)},
                                    {"triggers", edge_json(s.triggers)}});
            const auto path = QDir(directory).filePath(name);
            if (QFileInfo::exists(path))
                take.cameras.push_back({path, serials[i], positions[i], s.first_timestamp_us,
                                        s.latest_timestamp_us, s.event_count, true});
        }
        cameras = {};
        report.insert("acquisitionWallMs", elapsed.elapsed());
        report.insert("pulseSequenceCompleted", report.value("preamble").toObject().value("ok").toBool() &&
                                                    report.value("formal").toObject().value("ok").toBool());
        auto session = report;
        session.insert("schemaVersion", 2);
        session.insert("state", sequence_complete && failure.isEmpty() ? "completed" : "incomplete");
        session.insert("cameras", rows);
        session.insert("reason", failure);
        session.insert("formalStartUs", -1);
        session.insert("formalEndUs", -1);
        require(write_json(QDir(directory).filePath("session.json"), session),
                "Could not save session evidence.");
        const auto verification = verify_take(take, {}, [&] { return interrupted != 0; });
        if (verification.value("verdict").toString() == "failed" ||
            verification.value("cancelled").toBool()) {
            failure += " RAW verification failed or was cancelled.";
            session.insert("state", "incomplete");
            session.insert("reason", failure);
            require(write_json(QDir(directory).filePath("session.json"), session),
                    "Could not preserve the failed verification state.");
        }
        report.insert("rawVerification", verification);
        report.insert("cameras", rows);
        report.insert("error", failure);
        report.insert("sequenceCompleted", sequence_complete && low.ok && low.low_confirmed);
        require(write_json(QDir(directory).filePath("report.json"), report),
                "Could not save diagnostic report.");
        QJsonArray summary_cameras;
        for (const auto &value : rows) {
            const auto row = value.toObject();
            summary_cameras.append(
                QJsonObject{{"serial", row.value("serial")},
                            {"role", row.value("role")},
                            {"eventCount", row.value("eventCount")},
                            {"retainedTriggerCount", row.value("triggers").toArray().size()}});
        }
        output({{"evidenceDirectory", directory},
                {"cameras", summary_cameras},
                {"controllerCheck", report.value("controllerCheck")},
                {"cleanup", report.value("cleanup")},
                {"timingMeasurement", report.value("timingMeasurement")},
                {"error", failure},
                {"projectAcceptance", report.value("projectAcceptance")},
                {"rawVerification", verification}});
        return failure.isEmpty() && verification.value("verdict").toString() != "failed" ? 0 : 1;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
