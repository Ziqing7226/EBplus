// bench_guiopen — reproduces the GUI's file-open path EXACTLY:
// CameraController::connect_file() (the same function the Open File dialog
// calls), including external-source routing for AEDAT4/ALPDATA and the
// SDK Camera::from_file path for RAW/HDF5.
//
// Usage: bench_guiopen <file> [<file>...]
// Run it with the SAME environment as the GUI (via start-ebplus.cmd / a
// matching env) to prove or disprove "file won't open" outside the UI.

#include <QCoreApplication>

#include "app/camera_controller.h"

#include <QTimer>

#include <cstdio>
#include <string>
#include <vector>

namespace {

gui::CameraController* g_controller = nullptr;
std::vector<std::string> g_files;
int g_index = 0;
int g_fails = 0;

void open_next();

void open_one(const std::string& f) {
    std::printf("open [%d]: %s\n", g_index, f.c_str());
    const bool ok = g_controller->connect_file(f);
    std::printf("  connect_file => %s\n", ok ? "OK" : "FAILED");
    if (ok) {
        // Mirror what the GUI does synchronously inside its connected()
        // handler: populate the facility panels and query source
        // capabilities. Every facility getter must return nullptr for an
        // offline source instead of throwing
        // CameraException(DeviceUnavailable) — a throw here propagates back
        // through the direct-connected signal into connect_file() and fails
        // the whole open (the HDF5 GUI regression this probe used to miss).
        int fac_fails = 0;
        auto try_get = [&](const char* name, auto&& fn) {
            try {
                void* p = static_cast<void*>(fn());
                std::printf("  facility %-14s => %s\n", name, p ? "non-null" : "nullptr");
            } catch (const std::exception& e) {
                std::printf("  facility %-14s => THREW: %s\n", name, e.what());
                ++fac_fails;
            }
        };
        try_get("biases", [] { return g_controller->biases_facility(); });
        try_get("roi", [] { return g_controller->roi_facility(); });
        try_get("anti_flicker", [] { return g_controller->anti_flicker_facility(); });
        try_get("trail_filter", [] { return g_controller->trail_filter_facility(); });
        try_get("erc", [] { return g_controller->erc_facility(); });
        try_get("trigger_in", [] { return g_controller->trigger_in_facility(); });
        try_get("trigger_out", [] { return g_controller->trigger_out_facility(); });
        try_get("camera_sync", [] { return g_controller->camera_sync_facility(); });
        const auto caps = g_controller->source_capabilities();
        std::printf("  capabilities: imu=%d aps=%d trigger=%d esp=%d\n",
                    caps.imu ? 1 : 0, caps.aps ? 1 : 0,
                    caps.trigger ? 1 : 0, caps.esp ? 1 : 0);
        g_fails += fac_fails;
    }
    if (!ok) ++g_fails;
    ++g_index;
    QTimer::singleShot(0, QCoreApplication::instance(), [] { open_next(); });
}

void open_next() {
    if (g_index >= static_cast<int>(g_files.size())) {
        std::printf("guiopen batch: %d file(s), %d failure(s)\n",
                    static_cast<int>(g_files.size()), g_fails);
        QCoreApplication::exit(g_fails ? 1 : 0);
        return;
    }
    open_one(g_files[static_cast<size_t>(g_index)]);
}

} // namespace

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <file> [<file>...]\n", argv[0]);
        return 2;
    }
    for (int i = 1; i < argc; ++i) g_files.emplace_back(argv[i]);

    gui::CameraController controller;
    g_controller = &controller;

    QObject::connect(&controller, &gui::CameraController::error,
                     [&](const QString& msg) {
                         std::printf("  error: %s\n", msg.toUtf8().constData());
                     });
    QObject::connect(&controller, &gui::CameraController::runtime_warning,
                     [&](const QString& msg) {
                         std::printf("  warning: %s\n", msg.toUtf8().constData());
                     });

    QTimer::singleShot(0, &app, [] { open_next(); });
    return app.exec();
}
