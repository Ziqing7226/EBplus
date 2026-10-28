// bench_loop — headless reproduction of the user's RAW loop path through
// the REAL components: CameraController (streaming file playback) with
// set_file_loop(true). PASS = after reaching the end of the stream, the
// position wraps back near 0 and playback keeps producing positions.
//
// Usage: bench_loop <file> [timeout_s]

#include <QCoreApplication>
#include <QTimer>

#include "app/camera_controller.h"

#include <metavision/sdk/base/utils/timestamp.h>

#include <cstdio>
#include <cstdlib>

namespace {

gui::CameraController* g_controller = nullptr;
Metavision::timestamp g_last_pos = -1;
Metavision::timestamp g_max_pos = 0;
bool g_wrapped = false;
int g_ticks = 0;

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: %s <file> [timeout_s]\n", argv[0]);
        return 2;
    }
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    const int timeout_s = argc > 2 ? std::atoi(argv[2]) : 45;

    QCoreApplication app(argc, argv);

    gui::CameraController controller;
    g_controller = &controller;

    QObject::connect(&controller, &gui::CameraController::error,
                     [&](const QString& m) {
                         std::printf("error: %s\n", m.toUtf8().constData());
                     });

    if (!controller.connect_file(argv[1])) {
        std::printf("connect_file FAILED\n");
        return 1;
    }
    auto* fp = controller.frame_pipeline();
    fp->set_file_loop(true);
    controller.start();
    fp->play_file();

    QObject::connect(fp, &gui::FramePipeline::file_position_changed,
                     [&](Metavision::timestamp pos, Metavision::timestamp) {
                         ++g_ticks;
                         if (g_last_pos >= 0 && pos + 100'000 < g_last_pos) {
                             if (!g_wrapped) {
                                 std::printf("WRAPPED at tick %d: %lld -> %lld\n",
                                             g_ticks, (long long)g_last_pos,
                                             (long long)pos);
                             }
                             g_wrapped = true;
                         }
                         g_last_pos = pos;
                         if (pos > g_max_pos) g_max_pos = pos;
                     });
    QObject::connect(fp, &gui::FramePipeline::file_looped,
                     [] { std::printf("EVENT looped\n"); });
    QObject::connect(fp, &gui::FramePipeline::file_eof_reached,
                     [] { std::printf("EVENT eof_reached\n"); });
    QObject::connect(fp, &gui::FramePipeline::loader_seek_requested,
                     [](Metavision::timestamp t) {
                         std::printf("EVENT loader_seek_requested(%lld)\n",
                                     (long long)t);
                     });
    QObject::connect(fp, &gui::FramePipeline::loader_pause_requested,
                     [] { std::printf("EVENT loader_pause\n"); });
    QObject::connect(fp, &gui::FramePipeline::loader_resume_requested,
                     [] { std::printf("EVENT loader_resume\n"); });
    QObject::connect(&controller, &gui::CameraController::stopped,
                     [] { std::printf("EVENT camera stopped\n"); });
    QObject::connect(&controller, &gui::CameraController::started,
                     [] { std::printf("EVENT camera started\n"); });

    QTimer report;
    int elapsed = 0;
    QObject::connect(&report, &QTimer::timeout, [&]() {
        elapsed += 2;
        std::printf("t=%2ds pos=%lld max=%lld wrapped=%d\n", elapsed,
                    (long long)g_last_pos, (long long)g_max_pos,
                    g_wrapped ? 1 : 0);
        if (g_wrapped && elapsed >= 12) {
            std::printf("RESULT: LOOP-OK (wrapped and kept playing)\n");
            app.exit(0);
        } else if (elapsed >= timeout_s) {
            std::printf("RESULT: NO-WRAP (pos=%lld max=%lld)\n",
                        (long long)g_last_pos, (long long)g_max_pos);
            app.exit(1);
        }
    });
    report.start(2000);

    return app.exec();
}
