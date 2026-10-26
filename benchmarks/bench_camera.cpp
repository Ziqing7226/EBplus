// bench_camera — live camera enumeration + streaming probe (SDK level).
//
// Usage: bench_camera [list|open <index>]
//   list          — enumerate every camera the installed HAL plugins discover
//   open <index>  — open camera #index, stream ~2 s, report the event count
//
// This is the live-camera counterpart of bench_playback: it exercises the
// exact path the GUI's Devices panel uses (plugin discovery -> Camera open
// -> CD stream), so a failure here isolates the problem to the driver /
// plugin stack instead of the UI.

#include <metavision/sdk/stream/camera.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <exception>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// USB-index flat list over all discovered online sources.
static std::vector<std::pair<Metavision::OnlineSourceType, uint32_t>> enumerate() {
    std::vector<std::pair<Metavision::OnlineSourceType, uint32_t>> out;
    const auto sources = Metavision::Camera::list_online_sources();
    for (const auto& [type, serials] : sources) {
        for (uint32_t i = 0; i < serials.size(); ++i) {
            out.emplace_back(type, i);
            std::printf("  discovered %s source #%u: %s\n",
                        type == Metavision::OnlineSourceType::USB         ? "USB"
                        : type == Metavision::OnlineSourceType::EMBEDDED  ? "EMBEDDED"
                                                                          : "REMOTE",
                        i, serials[i].c_str());
        }
    }
    return out;
}

int list_mode() {
    const auto cams = enumerate();
    if (cams.empty()) {
        std::printf("no cameras discovered\n");
        return 1;
    }
    std::printf("%zu camera(s) discovered; run 'bench_camera open <index>' to stream\n",
                cams.size());
    return 0;
}

int open_mode(size_t index) {
    const auto cams = enumerate();
    if (index >= cams.size()) {
        std::printf("index %zu out of range\n", index);
        return 1;
    }
    Metavision::Camera cam;
    try {
        cam = Metavision::Camera::from_source(cams[index].first, cams[index].second);
    } catch (const std::exception& e) {
        std::printf("OPEN FAILED: %s\n", e.what());
        return 1;
    }
    std::printf("opened OK\n");

    std::atomic<long long> ev{0};
    cam.cd().add_callback([&ev](const Metavision::EventCD* b, const Metavision::EventCD* e) {
        ev += static_cast<long long>(e - b);
    });
    cam.start();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    cam.stop();
    std::printf("streamed %lld events in ~2 s\n", ev.load());
    return 0;
}

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "list";
    try {
        if (mode == "list") return list_mode();
        if (mode == "open" && argc > 2) return open_mode(static_cast<size_t>(std::atoi(argv[2])));
    } catch (const std::exception& e) {
        std::printf("ERROR: %s\n", e.what());
        return 1;
    }
    std::fprintf(stderr, "usage: %s list | open <index>\n", argv[0]);
    return 2;
}
