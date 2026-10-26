// bench_playback — offline RAW/HDF5/AEDAT4 read+decode throughput.
//
// Usage: bench_playback save|check|time <file> [<file>...]
//   save  — record the FNV-1a hash of the decoded event stream (per file,
//           golden file per input, in the current directory)
//   check — verify hashes against the recorded goldens (any decoded byte
//           that differs from the baseline build fails loudly)
//   time  — throughput only
//
// Opens files with real_time_playback disabled (the same setting the GUI
// uses) and drains the CD stream as fast as the SDK allows. The hash covers
// every decoded CD event (x, y, p, t) in stream order, so it is a strict
// bit-exactness guard for decoder work — the same EVT2 decode path the
// SilkyEvCam live stream takes. Golden files are named
// golden_playback_<stem>.txt next to the input.

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/stream/file_config_hints.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <filesystem>
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

std::string golden_name_for(const std::string& file) {
    const std::filesystem::path p(file);
    // Include the extension: RAW and HDF5 recordings may share a stem but
    // their decoded streams are hashed independently.
    return "golden_playback_" + p.filename().string() + ".txt";
}

} // namespace

int run_one(const std::string& raw, const std::string& mode) {
    Metavision::FileConfigHints hints;
    hints.real_time_playback(false);
    Metavision::Camera cam;
    try {
        cam = Metavision::Camera::from_file(raw, hints);
    } catch (const std::exception& e) {
        std::printf("playback %s: OPEN FAILED: %s\n", raw.c_str(), e.what());
        return 1;
    }

    std::atomic<long long> n_events{0};
    std::atomic<long long> n_batches{0};
    // Single-writer (the CD callback thread). Only the golden modes pay the
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
        std::printf("playback %s: NO EVENTS DECODED\n", raw.c_str());
        return 1;
    }
    std::printf("playback %s: %lld events / %lld batches in %.3f s => %.2f Mevents/s\n",
                raw.c_str(), ev, bat, secs, static_cast<double>(ev) / secs / 1e6);
    if (hash_events) {
        std::printf("  event stream hash: %016llx\n",
                    static_cast<unsigned long long>(stream_hash));
    }

    const std::string gpath = (std::filesystem::path(raw).parent_path() /
                               golden_name_for(raw)).string();
    if (mode == "save") {
        std::ofstream f(gpath);
        if (!f) {
            std::printf("  golden: cannot write %s\n", gpath.c_str());
            return 1;
        }
        f << "events " << ev << "\nhash " << std::hex << stream_hash << "\n";
        std::printf("  golden: saved to %s\n", gpath.c_str());
        return 0;
    }
    if (mode == "check") {
        std::ifstream f(gpath);
        if (!f) {
            std::printf("  golden: MISSING %s\n", gpath.c_str());
            return 1;
        }
        std::string key;
        unsigned long long ghash = 0, gev = 0;
        while (f >> key) {
            if (key == "events") f >> gev;
            else if (key == "hash") f >> std::hex >> ghash;
        }
        const bool ok = (gev == static_cast<unsigned long long>(ev) && ghash == stream_hash);
        std::printf("  golden: %s (%s)\n", ok ? "match" : "MISMATCH", gpath.c_str());
        return ok ? 0 : 1;
    }
    return 0;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s save|check|time <file> [<file>...]\n", argv[0]);
        return 2;
    }
    const std::string mode = argv[1];
    int fails = 0;
    for (int argi = 2; argi < argc; ++argi) {
        if (run_one(argv[argi], mode) != 0) ++fails;
    }
    std::printf("playback batch: %d file(s), %d failure(s)\n", argc - 2, fails);
    return fails ? 1 : 0;
}
