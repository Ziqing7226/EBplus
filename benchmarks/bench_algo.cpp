// bench_algo — header-only algorithm micro benches + golden bit-exactness.
//
// Usage: bench_algo save|check|time [golden_dir]
//   save  — write golden hashes of every algorithm output into <golden_dir>
//   check — recompute and compare against the saved hashes (fail on diff)
//   time  — timings only, no golden I/O (golden_dir may be omitted)
//
// The golden mode is the optimization guard: run "save" on the baseline
// build, then "check" after each change — any output byte that differs from
// the baseline makes the bench exit non-zero.

#include "algo/analytics/sensor_self_test.h"
#include "algo/cv/time_surface.h"
#include "algo/cv/xyt_visualizer.h"

#include <opencv2/core.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include <vector>

namespace {

using gui_algo::TimeSurface;

std::uint64_t fnv1a(const void* data, std::size_t n, std::uint64_t h = 1469598103934665603ULL) {
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

double median_ms(std::vector<double>& v) {
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

// Deterministic synthetic event stream (same on every run/build).
std::vector<Metavision::EventCD> make_events(int w, int h, int count, std::uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> dx(0, w - 1), dy(0, h - 1), dp(0, 1);
    std::vector<Metavision::EventCD> ev;
    ev.reserve(count);
    const int dt = 300; // us between events overall stream
    for (int i = 0; i < count; ++i) {
        Metavision::EventCD e;
        e.x = static_cast<std::uint16_t>(dx(rng));
        e.y = static_cast<std::uint16_t>(dy(rng));
        e.p = static_cast<short>(dp(rng));
        e.t = static_cast<Metavision::timestamp>(i) * dt;
        ev.push_back(e);
    }
    return ev;
}

struct Golden {
    std::string path;
    std::vector<std::pair<std::string, std::uint64_t>> hashes;
    void add(const char* name, std::uint64_t h) { hashes.emplace_back(name, h); }
    bool run(const char* mode) {
        if (std::string(mode) == "save") {
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

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s save|check|time [golden_dir]\n", argv[0]);
        return 2;
    }
    const std::string mode = argv[1];
    const std::string gdir = argc > 2 ? argv[2] : ".";
    constexpr int W = 1280, H = 720;

    Golden golden;
    golden.path = gdir + "/golden_algo.txt";

    // --- TimeSurface: all decay/channel variants -----------------------------
    {
        const auto ev = make_events(W, H, 200000, 1234);
        struct Cfg { const char* name; TimeSurface::Decay d; TimeSurface::Channels c; int pal; };
        const Cfg cfgs[] = {
            {"ts_linear_merged", TimeSurface::Decay::Linear, TimeSurface::Channels::Merged, 0},
            {"ts_linear_split", TimeSurface::Decay::Linear, TimeSurface::Channels::Split, 1},
            {"ts_exp_merged", TimeSurface::Decay::Exponential, TimeSurface::Channels::Merged, 2},
            {"ts_exp_split", TimeSurface::Decay::Exponential, TimeSurface::Channels::Split, 3},
        };
        for (const auto& c : cfgs) {
            TimeSurface ts(W, H, c.c, 100000,
                           static_cast<TimeSurface::Palette>(c.pal), 30, c.d, 100000);
            ts.process(reinterpret_cast<const gui_algo::Event*>(ev.data()), ev.size());
            golden.add(c.name, hash_mat(ts.render()));
        }
        // Timing on the dominant config (linear merged).
        TimeSurface ts(W, H, TimeSurface::Channels::Merged, 100000,
                       TimeSurface::Palette::Hot, 30);
        ts.process(reinterpret_cast<const gui_algo::Event*>(ev.data()), ev.size());
        std::vector<double> t;
        const cv::Mat out = ts.render();
        (void)out;
        for (int i = 0; i < 30; ++i) {
            const auto a = std::chrono::steady_clock::now();
            const cv::Mat img = ts.render();
            const auto b = std::chrono::steady_clock::now();
            t.push_back(std::chrono::duration<double, std::milli>(b - a).count());
            (void)img;
        }
        std::printf("time_surface render: %.3f ms/frame (1280x720 linear merged)\n", median_ms(t));
    }

    // --- SensorSelfTest -------------------------------------------------------
    {
        gui_algo::SensorSelfTest sst(W, H);
        const auto ev = make_events(W, H, 200000, 42);
        sst.process(reinterpret_cast<const gui_algo::Event*>(ev.data()), ev.size());
        golden.add("sst_render", hash_mat(sst.render()));

        std::vector<double> t;
        for (int i = 0; i < 30; ++i) {
            const auto a = std::chrono::steady_clock::now();
            const cv::Mat img = sst.render();
            const auto b = std::chrono::steady_clock::now();
            t.push_back(std::chrono::duration<double, std::milli>(b - a).count());
            (void)img;
        }
        std::printf("sensor_self_test render: %.3f ms/frame\n", median_ms(t));
    }

    // --- XYT point cloud build (the work paintGL repeats 60x/s today) -------
    {
        gui_algo::XYTVisualizer viz(50.0f, gui_algo::XYTVisualizer::ColorMode::Age,
                                    2.5f, false, true);
        // Dense stream: 1us apart -> 50k events = a full 50ms window at the
        // equivalent of 1M ev/s.
        std::mt19937 rng(7);
        std::uniform_int_distribution<int> dx(0, W - 1), dy(0, H - 1), dp(0, 1);
        std::vector<Metavision::EventCD> ev(50000);
        for (std::size_t i = 0; i < ev.size(); ++i) {
            ev[i].x = static_cast<std::uint16_t>(dx(rng));
            ev[i].y = static_cast<std::uint16_t>(dy(rng));
            ev[i].p = static_cast<short>(dp(rng));
            ev[i].t = static_cast<Metavision::timestamp>(i);
        }
        viz.process(reinterpret_cast<const gui_algo::Event*>(ev.data()), ev.size());
        std::vector<gui_algo::XYTPoint> points;
        viz.render(points);
        std::printf("xyt: %zu points in window\n", points.size());
        std::vector<double> t;
        for (int i = 0; i < 50; ++i) {
            const auto a = std::chrono::steady_clock::now();
            viz.render(points);
            const auto b = std::chrono::steady_clock::now();
            t.push_back(std::chrono::duration<double, std::milli>(b - a).count());
        }
        std::printf("xyt render: %.3f ms/rebuild => %.1f ms/s wasted at 60FPS idle\n",
                    median_ms(t), median_ms(t) * 60.0);
    }

    // --- list_live pattern A/B (P9 evidence) ---------------------------------
    {
        struct Obj { int v{0}; };
        std::mutex m;
        std::vector<std::shared_ptr<Obj>> live(4);
        for (auto& p : live) p = std::make_shared<Obj>();
        std::shared_ptr<const std::vector<std::shared_ptr<Obj>>> snapshot =
            std::make_shared<const std::vector<std::shared_ptr<Obj>>>(live);

        constexpr int N = 200000;
        std::vector<double> t_copy, t_snap;
        for (int i = 0; i < N; ++i) {
            auto a = std::chrono::steady_clock::now();
            std::vector<std::shared_ptr<Obj>> v2;
            { std::lock_guard<std::mutex> lk(m); v2 = live; }
            auto b = std::chrono::steady_clock::now();
            auto s = std::atomic_load(&snapshot);
            auto c = std::chrono::steady_clock::now();
            t_copy.push_back(std::chrono::duration<double, std::nano>(b - a).count());
            t_snap.push_back(std::chrono::duration<double, std::nano>(c - b).count());
        }
        std::sort(t_copy.begin(), t_copy.end());
        std::sort(t_snap.begin(), t_snap.end());
        std::printf("list_live pattern: copy+lock %.0f ns/call | atomic snapshot %.1f ns/call\n",
                    t_copy[t_copy.size() / 2], t_snap[t_snap.size() / 2]);
    }

    if (mode == "save" || mode == "check") {
        if (!golden.run(mode.c_str())) return 1;
    }
    return 0;
}
