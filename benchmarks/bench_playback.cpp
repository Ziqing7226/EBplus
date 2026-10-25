// bench_playback — offline RAW read+decode throughput (end to end).
//
// Usage: bench_playback <file.raw>
//
// Opens the file with real_time_playback disabled (the same setting the GUI
// uses) and drains the CD stream as fast as the SDK allows, reporting
// Mevents/s. This is the regression guard for the whole decode pipeline.

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/stream/file_config_hints.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <file.raw>\n", argv[0]);
        return 2;
    }
    Metavision::FileConfigHints hints;
    hints.real_time_playback(false);
    Metavision::Camera cam = Metavision::Camera::from_file(argv[1], hints);

    std::atomic<long long> n_events{0};
    std::atomic<long long> n_batches{0};
    cam.cd().add_callback([&](const Metavision::EventCD* b, const Metavision::EventCD* e) {
        n_events += static_cast<long long>(e - b);
        n_batches += 1;
    });

    const auto t0 = std::chrono::steady_clock::now();
    cam.start();
    while (cam.is_running()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double secs = std::chrono::duration<double>(t1 - t0).count();
    const long long ev = n_events.load();
    const long long bat = n_batches.load();
    if (ev <= 0 || bat <= 0) {
        std::fprintf(stderr, "bench_playback: no events decoded\n");
        return 1;
    }
    std::printf("playback: %lld events / %lld batches in %.3f s => %.2f Mevents/s (avg %.0f ev/batch)\n",
                ev, bat, secs, static_cast<double>(ev) / secs / 1e6,
                static_cast<double>(ev) / static_cast<double>(bat));
    return 0;
}
