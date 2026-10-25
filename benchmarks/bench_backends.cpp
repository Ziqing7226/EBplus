// bench_backends — AlgoBackend push/pull benches via the public factory,
// with golden bit-exactness over (frame, status, output span).
//
// Usage: bench_backends save|check|time [golden_dir]
//   save  — write golden hashes to <golden_dir>/golden_backends.txt
//   check — compare against the saved hashes (fail on any diff)
//   time  — timings only
//
// Golden mode is the optimization guard for the algo_bridge backends: the
// filtered_events field removal (P2) and copy-elimination work (P6-P8) must
// not change a single byte of frame/status/output-span output.

#include "algo_bridge/algo_backend.h"

#include <metavision/sdk/base/events/event_cd.h>

#include <opencv2/core.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <random>
#include <string>
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

std::uint64_t hash_mat(const cv::Mat& m) {
    if (m.empty()) return 0;
    const int rows = m.rows, cols = m.cols, type = m.type();
    std::uint64_t h = fnv1a(&rows, sizeof(rows));
    h = fnv1a(&cols, sizeof(cols), h);
    h = fnv1a(&type, sizeof(type), h);
    for (int y = 0; y < m.rows; ++y) {
        h = fnv1a(m.ptr(y), static_cast<std::size_t>(m.step), h);
    }
    return h;
}

std::uint64_t hash_events(const Metavision::EventCD* b, const Metavision::EventCD* e) {
    if (b == nullptr || e <= b) return 0;
    return fnv1a(b, static_cast<std::size_t>(reinterpret_cast<const char*>(e) -
                                             reinterpret_cast<const char*>(b)));
}

double median_us(std::vector<double>& v) {
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

std::vector<Metavision::EventCD> make_batch(int w, int h, int count, int base_t,
                                            std::mt19937& rng) {
    std::uniform_int_distribution<int> dx(0, w - 1), dy(0, h - 1), dp(0, 1);
    std::vector<Metavision::EventCD> ev;
    ev.reserve(count);
    for (int i = 0; i < count; ++i) {
        Metavision::EventCD e;
        e.x = static_cast<std::uint16_t>(dx(rng));
        e.y = static_cast<std::uint16_t>(dy(rng));
        e.p = static_cast<short>(dp(rng));
        e.t = base_t + i;
        ev.push_back(e);
    }
    return ev;
}

struct Golden {
    std::string path;
    std::vector<std::pair<std::string, std::uint64_t>> hashes;
    void add(const std::string& n, std::uint64_t h) { hashes.emplace_back(n, h); }
    bool run(const std::string& mode) {
        if (mode == "save") {
            std::ofstream f(path);
            if (!f) return false;
            for (const auto& [n, h] : hashes) f << n << ' ' << std::hex << h << '\n';
            std::printf("golden: %zu hashes saved to %s\n", hashes.size(), path.c_str());
            return true;
        }
        std::ifstream f(path);
        if (!f) return false;
        std::size_t i = 0;
        bool ok = true;
        std::string n;
        unsigned long long h = 0;
        while (f >> n >> std::hex >> h) {
            if (i >= hashes.size()) { ok = false; break; }
            if (hashes[i].first != n || hashes[i].second != h) {
                std::printf("golden MISMATCH: %s expected %016llx got %016llx\n",
                            hashes[i].first.c_str(),
                            static_cast<unsigned long long>(hashes[i].second),
                            static_cast<unsigned long long>(h));
                ok = false;
            }
            ++i;
        }
        if (i != hashes.size()) ok = false;
        std::printf("golden: %s\n", ok ? "all hashes match" : "FAILED");
        return ok;
    }
};

constexpr int W = 1280, H = 720;

// One backend scenario: 100 pushes of 30k events (a 33ms window at ~1M ev/s),
// pull_result after every push (the GUI pulls per display frame).
void scenario(const char* name, const char* mode, Golden& golden) {
    auto backend = gui::create_algo_backend(name, W, H);
    if (!backend) {
        std::fprintf(stderr, "bench_backends: unknown backend '%s'\n", name);
        return;
    }
    std::mt19937 rng(99 + name[0]);
    std::vector<double> t_push, t_pull;
    std::uint64_t frame_h = 0;
    std::string status;
    constexpr int kBatch = 30000;
    constexpr int kBatches = 100;
    for (int i = 0; i < kBatches; ++i) {
        const auto batch = make_batch(W, H, kBatch, i * kBatch, rng);
        auto a = std::chrono::steady_clock::now();
        backend->push_events(batch.data(), batch.data() + batch.size());
        auto b = std::chrono::steady_clock::now();
        auto r = backend->pull_result();
        auto c = std::chrono::steady_clock::now();
        t_push.push_back(std::chrono::duration<double, std::micro>(b - a).count());
        t_pull.push_back(std::chrono::duration<double, std::micro>(c - b).count());
        frame_h = hash_mat(r.frame);
        const auto span = backend->output_span();
        status = r.status;
        (void)span;
    }
    const auto span = backend->output_span();
    golden.add(std::string(name) + "/frame", frame_h);
    golden.add(std::string(name) + "/status",
               fnv1a(status.data(), status.size()));
    golden.add(std::string(name) + "/span",
               hash_events(span.first, span.first + span.second));
    std::printf("%-18s push %8.1f us/batch | pull %8.1f us/pull (30k ev)\n",
                name, median_us(t_push), median_us(t_pull));
    (void)mode;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s save|check|time [golden_dir]\n", argv[0]);
        return 2;
    }
    const std::string mode = argv[1];
    const std::string gdir = argc > 2 ? argv[2] : ".";

    Golden golden;
    golden.path = gdir + "/golden_backends.txt";

    scenario("time_surface", mode.c_str(), golden);
    scenario("hot_pixel_filter", mode.c_str(), golden);
    scenario("freq_detector", mode.c_str(), golden);
    scenario("background_mask", mode.c_str(), golden);
    scenario("sensor_self_test", mode.c_str(), golden);

    if (mode == "save" || mode == "check") {
        if (!golden.run(mode)) return 1;
    }
    return 0;
}
