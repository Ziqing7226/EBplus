// bench_index — RAW index build timing + seek correctness guard.
//
// Usage:
//   bench_index time <file.raw>
//       Deletes the sidecar index, opens the file (which rebuilds it in a
//       background thread), reports the time until the index status becomes
//       usable. Pure file-side pipeline — needs no camera.
//   bench_index seektest <file.raw> <target_us> [...]
//       For each target timestamp: hash the CD events decoded in a 20 ms
//       window after the target from (a) a fresh sequential open and (b) a
//       seek through I_EventsStream::seek. The two hashes must match
//       exactly (bit-exact seek continuity); events strictly before the
//       target (the replayed prefix from a quantized bookmark offset) are
//       allowed and excluded from both hashes.

#include <metavision/hal/device/device_discovery.h>
#include <metavision/hal/device/device.h>
#include <metavision/hal/facilities/i_events_stream.h>
#include <metavision/hal/facilities/i_event_decoder.h>
#include <metavision/hal/facilities/i_events_stream_decoder.h>
#include <metavision/sdk/base/events/event_cd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

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

std::filesystem::path sidecar_of(const std::string& raw) {
    std::filesystem::path p(raw);
    p += ".tmp_index";
    return p;
}

// Decodes the stream from the current position, hashing CD events in
// [start_ts, start_ts+window_us). Returns the hash; stops decoding once the
// window closes or the stream ends.
std::uint64_t decode_and_hash(Metavision::Device& device, Metavision::timestamp start_ts,
                              Metavision::timestamp window_us) {
    auto* fes  = device.get_facility<Metavision::I_EventsStream>();
    auto* dec  = device.get_facility<Metavision::I_EventDecoder<Metavision::EventCD>>();
    auto* esd  = device.get_facility<Metavision::I_EventsStreamDecoder>();
    auto h     = std::make_shared<std::atomic<std::uint64_t>>(1469598103934665603ULL);
    auto done  = std::make_shared<std::atomic<bool>>(false);
    dec->add_event_buffer_callback([h, done, start_ts, window_us](const Metavision::EventCD* b,
                                                                  const Metavision::EventCD* e) {
        if (done->load()) return;
        for (const Metavision::EventCD* p = b; p != e; ++p) {
            if (p->t < start_ts) continue; // replayed prefix before the window
            if (p->t >= start_ts + window_us) {
                done->store(true);
                return;
            }
            std::uint64_t h2 = h->load();
            h2               = fnv1a(&p->x, sizeof(p->x), h2);
            h2               = fnv1a(&p->y, sizeof(p->y), h2);
            h2               = fnv1a(&p->p, sizeof(p->p), h2);
            h2               = fnv1a(&p->t, sizeof(p->t), h2);
            h->store(h2);
        }
    });

    fes->start();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
    while (!done->load() && std::chrono::steady_clock::now() < deadline) {
        if (fes->wait_next_buffer() <= 0) break; // stream ended
        auto buffer = fes->get_latest_raw_data();
        if (buffer.size() == 0) continue;
        esd->decode(buffer.data(), buffer.data() + buffer.size());
    }
    try {
        fes->stop();
    } catch (...) {}
    return h->load();
}

int time_mode(const std::string& raw) {
    std::error_code ec;
    std::filesystem::remove(sidecar_of(raw), ec);

    const auto t0 = std::chrono::steady_clock::now();
    auto device   = Metavision::DeviceDiscovery::open_raw_file(raw);
    if (!device) {
        std::fprintf(stderr, "bench_index: open failed\n");
        return 1;
    }
    auto* fes = device->get_facility<Metavision::I_EventsStream>();
    if (!fes) {
        std::fprintf(stderr, "bench_index: no I_EventsStream facility\n");
        return 1;
    }
    Metavision::timestamp start_ts = -1, end_ts = -1;
    auto status = Metavision::I_EventsStream::IndexStatus::NotBuilt;
    const auto deadline = t0 + std::chrono::seconds(300);
    while (std::chrono::steady_clock::now() < deadline) {
        status = fes->get_seek_range(start_ts, end_ts);
        if (status == Metavision::I_EventsStream::IndexStatus::Good ||
            status == Metavision::I_EventsStream::IndexStatus::Bad) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    const auto t1  = std::chrono::steady_clock::now();
    const double s = std::chrono::duration<double>(t1 - t0).count();
    std::printf("index build: %.3f s (status %d, range [%lld, %lld])\n", s, static_cast<int>(status),
                static_cast<long long>(start_ts), static_cast<long long>(end_ts));
    return status == Metavision::I_EventsStream::IndexStatus::Good ? 0 : 1;
}

int seektest_mode(const std::string& raw, const std::vector<Metavision::timestamp>& targets) {
    int failures = 0;
    for (Metavision::timestamp target : targets) {
        // (a) sequential reference
        const std::uint64_t ref_hash = [&]() {
            auto device = Metavision::DeviceDiscovery::open_raw_file(raw);
            return decode_and_hash(*device, target, 20000);
        }();
        // (b) seek then decode the same window
        std::uint64_t seek_hash       = 0;
        Metavision::timestamp reached = -1;
        {
            auto device = Metavision::DeviceDiscovery::open_raw_file(raw);
            auto* fes   = device->get_facility<Metavision::I_EventsStream>();
            Metavision::timestamp s = -1, e = -1;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(300);
            while (fes->get_seek_range(s, e) != Metavision::I_EventsStream::IndexStatus::Good &&
                   std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            const auto st = fes->seek(target, reached);
            if (st != Metavision::I_EventsStream::SeekStatus::Success) {
                std::printf("seek %lld: status %d\n", static_cast<long long>(target), static_cast<int>(st));
                ++failures;
                continue;
            }
            seek_hash = decode_and_hash(*device, target, 20000);
        }
        const bool ok = (ref_hash == seek_hash && ref_hash != 1469598103934665603ULL);
        std::printf("seek %lld (reached %lld): %s (ref %016llx seek %016llx)\n",
                    static_cast<long long>(target), static_cast<long long>(reached),
                    ok ? "CONTINUITY OK" : "MISMATCH", static_cast<unsigned long long>(ref_hash),
                    static_cast<unsigned long long>(seek_hash));
        if (!ok) ++failures;
    }
    std::printf("seektest: %d failures\n", failures);
    return failures == 0 ? 0 : 1;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s time <file.raw> | seektest <file.raw> <target_us>...\n", argv[0]);
        return 2;
    }
    const std::string mode = argv[1];
    if (mode == "time") return time_mode(argv[2]);
    if (mode == "seektest") {
        std::vector<Metavision::timestamp> targets;
        for (int i = 3; i < argc; ++i) targets.push_back(std::atoll(argv[i]));
        return seektest_mode(argv[2], targets);
    }
    return 2;
}
