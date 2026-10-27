// bench_flip — headless verification of the FilterChain flip stages exactly
// as the GUI's preprocessing panel drives them (set_stage_enabled + set_geom
// + process). Verifies x/y mirroring and that a disabled stage passes
// events through untouched. Camera-free.

#include "algo_bridge/filter_chain.h"

#include <metavision/sdk/base/events/event_cd.h>

#include <cstdio>
#include <vector>

using namespace Metavision;

int main() {
    const int W = 640, H = 480;
    gui::FilterChain chain;
    chain.set_geometry(W, H);

    // One ON event at (100, 50), t=1000.
    const std::vector<EventCD> in = {{100, 50, 1, 1000}};

    int fails = 0;
    auto check = [&](const char* what, bool ok) {
        std::printf("%-40s %s\n", what, ok ? "OK" : "FAILED");
        if (!ok) ++fails;
    };

    // 1. All stages disabled: passthrough.
    {
        std::vector<EventCD> out;
        chain.process(in.data(), in.data() + in.size(), out);
        check("disabled: passthrough", out.size() == 1 && out[0].x == 100 && out[0].y == 50);
    }

    // 2. flip_x enabled: x -> W-1-x.
    chain.set_stage_enabled("flip_x", true);
    {
        std::vector<EventCD> out;
        chain.process(in.data(), in.data() + in.size(), out);
        check("flip_x: x 100 -> 539", out.size() == 1 && out[0].x == W - 1 - 100 && out[0].y == 50);
    }

    // 3. flip_x + flip_y: also y -> H-1-y.
    chain.set_stage_enabled("flip_y", true);
    {
        std::vector<EventCD> out;
        chain.process(in.data(), in.data() + in.size(), out);
        check("flip_x+flip_y: (100,50) -> (539,429)",
              out.size() == 1 && out[0].x == W - 1 - 100 && out[0].y == H - 1 - 50);
    }

    // 4. flip_x disabled again: only flip_y remains.
    chain.set_stage_enabled("flip_x", false);
    {
        std::vector<EventCD> out;
        chain.process(in.data(), in.data() + in.size(), out);
        check("flip_x off: y-only flip", out.size() == 1 && out[0].x == 100 && out[0].y == H - 1 - 50);
    }

    // 5. Geometry change (sensor switch 320x240): flip_y must remap within
    //    the NEW geometry — the exact property the camera-switch flow needs.
    chain.set_geometry(320, 240);
    {
        std::vector<EventCD> out;
        chain.process(in.data(), in.data() + in.size(), out);
        check("after set_geometry(320,240): y -> 239",
              out.size() == 1 && out[0].x == 100 && out[0].y == 240 - 1 - 50);
    }

    std::printf("bench_flip: %d failure(s)\n", fails);
    return fails ? 1 : 0;
}
