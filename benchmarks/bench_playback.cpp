// bench_playback — offline RAW read+decode throughput (end to end).
//
// Usage: bench_playback save|check|time <file.raw> [golden_dir]
//   save  — record the FNV-1a hash of the decoded event stream
//   check — verify the hash matches the saved one (any decoded byte that
//           differs from the baseline build fails loudly)
//   time  — throughput only
//
// Opens the file with real_time_playback disabled (the same setting the GUI
// uses) and drains the CD stream as fast as the SDK allows. The hash covers
// every decoded CD event (x, y, p, t) in stream order, so it is a strict
// bit-exactness guard for decoder work — the same EVT2 decode path the
// SilkyEvCam live stream takes.

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/stream/file_config_hints.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>

namespace {

std::uint64_t fnv1a(const void* data, std::size_t n,
                    std::uint64_t h = 1469598103934665603ULL) {
    const auto* p = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < n; ++i) {
        h ^= p[i];
        h *= 1099511628211ULL;
    }
    return h;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s save|check|time <file.raw> [golden_dir]\n", argv[0]);
        return 2;
    }
    const std::string mode = argv[1];
    const std::string golden_dir = argc > 3 ? argv[3] : ".";

    Metavision::FileConfigHints hints;
    hints.real_time_playback(false);
    Metavision::Camera cam = Metavision::Camera::from_file(argv[2], hints);

    std::atomic<long long> n_events{0};
    std::atomic<long long> n_batches{0};
    // Single-writer (the CD callback thread) — hashing needs no atomics
    // beyond the counters used for reporting. Only the golden modes pay the
    // per-event hashing cost; "time" runs measure the raw pipeline.
    const bool hash_events = (mode != "time");
    std::uint64_t stream_hash = 1469598103934665603ULL;
    cam.cd().add_callback([&](const Metavision::EventCD* b, const Metavision::EventCD* e) {
        n_events += static_cast<long long>(e - b);
        n_batches += 1;
        if (hash_events) {
            for (const Metavision::EventCD* p = b; p != e; ++p) {
                stream_hash = fnv1a(&p->x, sizeof(p->x), stream_hash);
                stream_hash = fnv1a(&p->y, sizeof(p->y), stream_hash);
                stream_hash = fnv1a(&p->p, sizeof(p->p), stream_hash);
                stream_hash = fnv1a(&p->t, sizeof(p->t), stream_hash);
            }
        }
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
    if (hash_events) {
        std::printf("event stream hash: %016llx\n",
                    static_cast<unsigned long long>(stream_hash));
    }

    if (mode == "save") {
        std::ofstream f(golden_dir + "/golden_playback.txt");
        if (!f) return 1;
        f << "events " << ev << "\nhash " << std::hex << stream_hash << "\n";
        std::printf("golden: saved to %s/golden_playback.txt\n", golden_dir.c_str());
        return 0;
    }
    if (mode == "check") {
        std::ifstream f(golden_dir + "/golden_playback.txt");
        if (!f) {
            std::fprintf(stderr, "golden file missing\n");
            return 1;
        }
        std::string key;
        unsigned long long ghash = 0, gev = 0;
        while (f >> key) {
            if (key == "events") f >> gev;
            else if (key == "hash") f >> std::hex >> ghash;
        }
        const bool ok = (gev == static_cast<unsigned long long>(ev) && ghash == stream_hash);
        std::printf("golden: %s (events %llu/%lld, hash %016llx/%016llx)\n",
                    ok ? "match" : "MISMATCH", gev, ev,
                    static_cast<unsigned long long>(ghash),
                    static_cast<unsigned long long>(stream_hash));
        return ok ? 0 : 1;
    }
    return 0;
}
