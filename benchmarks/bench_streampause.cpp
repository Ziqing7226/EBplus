// bench_streampause — probe: does an offline (file) Camera resume reading
// where it stopped after stop() → start()? This is the API precondition for
// the bounded-buffer streaming playback design (high/low-water pause/resume
// instead of buffering the whole file).
//
// Usage: bench_streampause <file>

#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/base/events/event_cd.h>

#include <chrono>
#include <cstdio>
#include <thread>

using namespace Metavision;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: %s <file> [eof]\n", argv[0]);
        return 2;
    }
    const bool eof_mode = argc > 2 && std::string(argv[2]) == "eof";
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    FileConfigHints hints;
    hints.real_time_playback(false);
    Camera cam = Camera::from_file(argv[1], hints);

    timestamp last         = -1;
    bool eof               = false;
    int phase              = 0; // 0 = first run, 1 = after restart
    long n_phase0          = 0;
    timestamp first_restart = -1;

    cam.add_runtime_error_callback([&](const CameraException&) { eof = true; });
    // RAW files can end with a silent status change instead of a runtime
    // error — treat STOPPED as EOF as well (this is what the GUI does).
    cam.add_status_change_callback([&](const CameraStatus& s) {
        if (s == CameraStatus::STOPPED) eof = true;
    });
    cam.cd().add_callback([&](const EventCD* b, const EventCD* e) {
        if (phase == 0) {
            ++n_phase0;
            last = (e - 1)->t;
        } else if (first_restart < 0) {
            first_restart = b->t;
        }
    });

    cam.start();
    if (eof_mode) {
        // Loop-wrap scenario: stream the WHOLE file to EOF, then seek(0)
        // and restart. This is what FileFrameGenerator's loop does.
        while (!eof) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        cam.stop();
        std::printf("phase0: eof reached, last_t=%lld us\n", (long long)last);

        bool seek_ok = false;
        try {
            seek_ok = cam.offline_streaming_control().seek(0);
        } catch (const std::exception& e) {
            std::printf("seek(0) THREW: %s\n", e.what());
        }
        std::printf("osc.seek(0) => %s\n", seek_ok ? "true" : "false");

        phase = 1;
        cam.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        cam.stop();
        std::printf("phase1: first_t_after_restart=%lld us\n",
                    (long long)first_restart);
        if (first_restart < 0) {
            std::printf("RESULT: FROZEN (no events after seek(0)+start)\n");
            return 1;
        }
        std::printf("RESULT: LOOP-RESTART %s (first_t=%lld)\n",
                    first_restart == 0 ? "AT-ZERO" : "AT-WRONG-TS",
                    (long long)first_restart);
        return first_restart == 0 ? 0 : 1;
    }

    cam.start();
    while (last < 1'500'000 && !eof) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    cam.stop();
    const timestamp at_stop = last;
    std::printf("phase0: %ld batches, last_t=%lld us, eof=%d\n", n_phase0,
                (long long)at_stop, eof ? 1 : 0);

    phase = 1;
    cam.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    cam.stop();
    std::printf("phase1: first_t_after_restart=%lld us\n", (long long)first_restart);

    if (first_restart < 0) {
        std::printf("RESULT: NO-EVENTS-AFTER-RESTART\n");
        return 1;
    }
    if (first_restart >= at_stop) {
        std::printf("RESULT: RESUMES (continues from ~last_t)\n");
        return 0;
    }
    std::printf("RESULT: REWINDS/OTHER (first after restart << last before stop)\n");
    return 1;
}
