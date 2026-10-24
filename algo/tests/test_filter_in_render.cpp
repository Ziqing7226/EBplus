// Quick test: verify FilterChain is applied in FileFrameGenerator::render_frame()
// AND that events_window_ready emits filtered events (so algorithm output is
// also flipped when a Replace-mode algorithm is running).
#include <QCoreApplication>
#include <cstdio>
#include <cmath>
#include <filesystem>
#include <opencv2/imgproc.hpp>
// OpenCV 5 moved undistortPoints from calib3d to the geometry module; the
// calib3d.hpp shim in some 5.x packagings does not pull it in.
#if CV_MAJOR_VERSION >= 5
#include <opencv2/geometry.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif

#include <metavision/sdk/base/events/event_cd.h>

#include "algo_bridge/filter_chain.h"
#include "app/file_frame_generator.h"

using namespace gui;

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);

    const int W = 100, H = 100;
    FilterChain chain;
    chain.set_geometry(W, H);

    FileFrameGenerator gen;
    gen.set_geometry(W, H);
    gen.set_filter_chain(&chain);
    gen.set_fps(60);
    gen.set_accumulation_time_us(1000);

    // Capture the rendered frame
    QImage rendered;
    QObject::connect(&gen, &FileFrameGenerator::frame_ready,
                     [&](QImage f, Metavision::timestamp) { rendered = f; });

    // Capture events_window_ready
    std::shared_ptr<std::vector<Metavision::EventCD>> emitted_events;
    QObject::connect(&gen, &FileFrameGenerator::events_window_ready,
                     [&](std::shared_ptr<std::vector<Metavision::EventCD>> evs,
                         Metavision::timestamp) { emitted_events = evs; });

    // Add a single event at (10, 50, p=1, t=100)
    Metavision::EventCD ev{10, 50, 100, 1};
    gen.add_events(&ev, &ev + 1);
    gen.set_duration_us(2000);

    // --- Test 1: No filter ---
    gen.seek(0);
    if (rendered.isNull()) {
        std::fprintf(stderr, "FAIL: no frame rendered\n");
        return 1;
    }
    const QRgb no_filter_pixel = rendered.pixel(10, 50);
    std::fprintf(stderr, "No filter: pixel(10,50) = %08X\n", no_filter_pixel);
    if (!emitted_events || emitted_events->size() != 1) {
        std::fprintf(stderr, "FAIL: events_window_ready didn't emit 1 event\n");
        return 1;
    }
    if ((*emitted_events)[0].x != 10) {
        std::fprintf(stderr, "FAIL: emitted event x=%d (expected 10)\n",
                     (*emitted_events)[0].x);
        return 1;
    }

    // --- Test 2: Enable flip_x ---
    // With flip_x, x=10 → W-1-10 = 89
    chain.set_stage_enabled("flip_x", true);
    gen.seek(0);
    if (rendered.isNull()) {
        std::fprintf(stderr, "FAIL: no frame rendered with flip_x\n");
        return 1;
    }
    const QRgb flip_pixel_at_10 = rendered.pixel(10, 50);
    const QRgb flip_pixel_at_89 = rendered.pixel(89, 50);
    std::fprintf(stderr, "Flip X: pixel(10,50) = %08X, pixel(89,50) = %08X\n",
                 flip_pixel_at_10, flip_pixel_at_89);

    bool ok = true;
    if (flip_pixel_at_10 == no_filter_pixel) {
        std::fprintf(stderr, "FAIL: pixel(10,50) unchanged after flip_x — filter NOT applied\n");
        ok = false;
    }
    if (flip_pixel_at_89 != no_filter_pixel) {
        std::fprintf(stderr, "FAIL: pixel(89,50) should match pre-flip pixel(10,50)\n");
        ok = false;
    }
    // Verify events_window_ready also emits FLIPPED events
    if (!emitted_events || emitted_events->size() != 1) {
        std::fprintf(stderr, "FAIL: events_window_ready didn't emit 1 event with flip\n");
        ok = false;
    } else if ((*emitted_events)[0].x != 89) {
        std::fprintf(stderr, "FAIL: emitted event x=%d (expected 89 after flip_x)\n",
                     (*emitted_events)[0].x);
        ok = false;
    } else {
        std::fprintf(stderr, "PASS: events_window_ready emitted flipped event x=89\n");
    }

    if (!ok) return 1;

    // --- Test 3: display-path noise filter (Phase 2.5) ---
    // An isolated event (no spatial-temporal neighbours) must be filtered
    // from the RENDERED frame, while events_window_ready keeps the
    // un-noise-filtered stream for algorithm instances.
    {
        FileFrameGenerator gen2;
        gen2.set_geometry(W, H);
        gen2.set_fps(60);
        gen2.set_accumulation_time_us(1000);
        gen2.set_conditioner_param("preproc_filter_enabled", "true");
        gen2.set_conditioner_param("preproc_filter_mode", "1");  // STCF
        gen2.set_conditioner_param("preproc_filter_correlation_time_s", "0.001");
        gen2.set_conditioner_param("preproc_filter_min_neighbors", "1");

        QImage rendered2;
        QObject::connect(&gen2, &FileFrameGenerator::frame_ready,
                         [&](QImage f, Metavision::timestamp) { rendered2 = f; });
        std::shared_ptr<std::vector<Metavision::EventCD>> emitted2;
        QObject::connect(&gen2, &FileFrameGenerator::events_window_ready,
                         [&](std::shared_ptr<std::vector<Metavision::EventCD>> evs,
                             Metavision::timestamp) { emitted2 = evs; });

        // Isolated event at (10,50) t=100 + a supported pair at (60,60) t=200/300.
        std::vector<Metavision::EventCD> evs;
        evs.push_back(Metavision::EventCD{10, 50, 100, 1});
        evs.push_back(Metavision::EventCD{60, 60, 200, 1});
        evs.push_back(Metavision::EventCD{61, 60, 300, 1});
        gen2.add_events(evs.data(), evs.data() + evs.size());
        gen2.set_duration_us(2000);
        gen2.seek(0);

        if (rendered2.isNull()) {
            std::fprintf(stderr, "FAIL: no frame rendered with display filter\n");
            return 1;
        }
        const QRgb bg = rendered2.pixel(0, 0);
        if (rendered2.pixel(10, 50) != bg) {
            std::fprintf(stderr, "FAIL: isolated event should be display-filtered\n");
            return 1;
        }
        if (rendered2.pixel(60, 60) == bg && rendered2.pixel(61, 60) == bg) {
            std::fprintf(stderr, "FAIL: supported pair should pass the display filter\n");
            return 1;
        }
        // 2026-08-19 rework: events_window_ready carries the SAME conditioned
        // stream the display rendered — the isolated event is dropped
        // everywhere (one conditioning pass for every consumer).
        // Only the second pair event passes STCF: (60,60) fires first with
        // no prior support; (61,60) then sees (60,60) within the window.
        if (!emitted2 || emitted2->size() != 1) {
            std::fprintf(stderr, "FAIL: events_window_ready must carry the "
                                 "conditioned stream (expected 1 event, got %zu)\n",
                         emitted2 ? emitted2->size() : 0);
            return 1;
        }
        std::fprintf(stderr, "PASS: shared noise filter — render and algo stream both filtered\n");
    }

    // --- Test 4: display-path undistort (Phase 2.5 step 3) ---
    // With a distortion calibration loaded, an event must be RENDERED at the
    // undistorted coordinate (matching cv::undistortPoints ground truth).
    {
        // POSIX-only "/tmp" breaks cv::FileStorage on Windows.
        const std::string yml =
            (std::filesystem::temp_directory_path() / "gui_test_calib.yml").string();
        cv::FileStorage fs(yml, cv::FileStorage::WRITE);
        fs << "image_width" << W << "image_height" << H;
        cv::Mat K = (cv::Mat_<double>(3, 3) << 80, 0, 50, 0, 80, 50, 0, 0, 1);
        cv::Mat dist = (cv::Mat_<double>(1, 5) << -0.25, 0.08, 0.0, 0.0, 0.0);
        fs << "camera_matrix" << K << "distortion_coefficients" << dist;
        fs.release();

        // Ground truth for (10, 10).
        std::vector<cv::Point2f> in{{10, 10}}, gt;
        cv::undistortPoints(in, gt, K, dist, cv::noArray(), K);

        FileFrameGenerator gen3;
        gen3.set_geometry(W, H);
        gen3.set_fps(60);
        gen3.set_accumulation_time_us(1000);
        gen3.set_conditioner_param("preproc_undistort_path", yml);
        gen3.set_conditioner_param("preproc_undistort_enabled", "true");

        QImage rendered3;
        QObject::connect(&gen3, &FileFrameGenerator::frame_ready,
                         [&](QImage f, Metavision::timestamp) { rendered3 = f; });
        Metavision::EventCD ev3{10, 10, 100, 1};
        gen3.add_events(&ev3, &ev3 + 1);
        gen3.set_duration_us(2000);
        gen3.seek(0);

        if (rendered3.isNull()) {
            std::fprintf(stderr, "FAIL: no frame rendered with undistort\n");
            return 1;
        }
        const QRgb bg = rendered3.pixel(0, 0);
        const int gx = static_cast<int>(std::lround(gt[0].x));
        const int gy = static_cast<int>(std::lround(gt[0].y));
        if (rendered3.pixel(10, 10) != bg) {
            std::fprintf(stderr, "FAIL: original position must be empty after undistort\n");
            return 1;
        }
        if (gx < 0 || gy < 0 || gx >= W || gy >= H || rendered3.pixel(gx, gy) == bg) {
            std::fprintf(stderr, "FAIL: event not rendered at undistorted (%d,%d)\n", gx, gy);
            return 1;
        }
        std::fprintf(stderr, "PASS: display undistort renders at (%d,%d) per cv ground truth\n",
                     gx, gy);
    }

    // --- Test 5: shared downsample as a thinning stage: only even-parity
    // coordinates survive for EVERY consumer (positions unchanged, no
    // coordinate halving here — halving backends shift downstream).
    {
        FileFrameGenerator gen4;
        gen4.set_geometry(W, H);
        gen4.set_fps(60);
        gen4.set_accumulation_time_us(1000);
        gen4.set_conditioner_param("preproc_downsample", "true");

        QImage rendered4;
        QObject::connect(&gen4, &FileFrameGenerator::frame_ready,
                         [&](QImage f, Metavision::timestamp) { rendered4 = f; });
        std::shared_ptr<std::vector<Metavision::EventCD>> emitted4;
        QObject::connect(&gen4, &FileFrameGenerator::events_window_ready,
                         [&](std::shared_ptr<std::vector<Metavision::EventCD>> evs,
                             Metavision::timestamp) { emitted4 = evs; });

        std::vector<Metavision::EventCD> evs;
        evs.push_back(Metavision::EventCD{10, 50, 100, 1});   // even/even → kept
        evs.push_back(Metavision::EventCD{11, 50, 101, 1});   // odd x     → thinned
        evs.push_back(Metavision::EventCD{10, 51, 102, 1});   // odd y     → thinned
        evs.push_back(Metavision::EventCD{11, 51, 103, 1});   // odd/odd   → thinned
        evs.push_back(Metavision::EventCD{60, 60, 104, 1});   // even/even → kept
        gen4.add_events(evs.data(), evs.data() + evs.size());
        gen4.set_duration_us(2000);
        gen4.seek(0);

        if (rendered4.isNull()) {
            std::fprintf(stderr, "FAIL: no frame rendered with display downsample\n");
            return 1;
        }
        const QRgb bg = rendered4.pixel(0, 0);
        if (rendered4.pixel(10, 50) == bg || rendered4.pixel(60, 60) == bg) {
            std::fprintf(stderr, "FAIL: even-coordinate events must render (no coord halving)\n");
            return 1;
        }
        if (rendered4.pixel(11, 50) != bg || rendered4.pixel(10, 51) != bg ||
            rendered4.pixel(11, 51) != bg) {
            std::fprintf(stderr, "FAIL: odd-coordinate events must be thinned\n");
            return 1;
        }
        // 2026-08-19 rework: events_window_ready carries the THINNED stream —
        // halving backends shift coordinates themselves, non-halving
        // consumers use it as-is.
        if (!emitted4 || emitted4->size() != 2) {
            std::fprintf(stderr, "FAIL: events_window_ready must carry the thinned "
                                 "stream (expected 2 events, got %zu)\n",
                         emitted4 ? emitted4->size() : 0);
            return 1;
        }
        std::fprintf(stderr, "PASS: shared downsample — even-parity kept at "
                             "original coordinates for every consumer\n");
    }

    // --- Test 6: unified-ROI display placement (2026-08-21 review): the
    // display frame stays FULL-sensor with ROI content at its ABSOLUTE
    // position (non-ROI keeps the background — the pre-rework pass-through
    // mode), while events_window_ready carries the CANONICAL ROI-relative
    // stream for the ROI-dim backends.
    {
        FileFrameGenerator gen5;
        gen5.set_geometry(W, H);
        gen5.set_fps(60);
        gen5.set_accumulation_time_us(1000);
        gen5.set_display_roi(true, 20, 30, 40, 25, /*roni=*/false);

        QImage rendered5;
        QObject::connect(&gen5, &FileFrameGenerator::frame_ready,
                         [&](QImage f, Metavision::timestamp) { rendered5 = f; });
        std::shared_ptr<std::vector<Metavision::EventCD>> emitted5;
        QObject::connect(&gen5, &FileFrameGenerator::events_window_ready,
                         [&](std::shared_ptr<std::vector<Metavision::EventCD>> evs,
                             Metavision::timestamp) { emitted5 = evs; });

        std::vector<Metavision::EventCD> evs;
        evs.push_back(Metavision::EventCD{30, 40, 100, 1});  // in-rect → ROI-rel (10,10)
        evs.push_back(Metavision::EventCD{5, 5, 150, 1});    // outside → dropped
        gen5.add_events(evs.data(), evs.data() + evs.size());
        gen5.set_duration_us(2000);
        gen5.seek(0);

        if (rendered5.isNull()) {
            std::fprintf(stderr, "FAIL: no frame rendered with ROI\n");
            return 1;
        }
        if (rendered5.width() != W || rendered5.height() != H) {
            std::fprintf(stderr, "FAIL: ROI display frame must stay full-sensor "
                                 "(got %dx%d, want %dx%d)\n",
                         rendered5.width(), rendered5.height(), W, H);
            return 1;
        }
        const QRgb bg5 = rendered5.pixel(0, 0);
        if (rendered5.pixel(30, 40) == bg5) {
            std::fprintf(stderr, "FAIL: in-ROI event must render at its ABSOLUTE "
                                 "position (30,40)\n");
            return 1;
        }
        if (rendered5.pixel(10, 10) != bg5) {
            std::fprintf(stderr, "FAIL: nothing may render at the ROI-relative "
                                 "position (10,10)\n");
            return 1;
        }
        if (rendered5.pixel(5, 5) != bg5) {
            std::fprintf(stderr, "FAIL: outside-ROI event must be dropped\n");
            return 1;
        }
        if (!emitted5 || emitted5->size() != 1 || (*emitted5)[0].x != 10 ||
            (*emitted5)[0].y != 10) {
            std::fprintf(stderr, "FAIL: events_window_ready must carry ONE "
                                 "ROI-RELATIVE event at (10,10)\n");
            return 1;
        }
        std::fprintf(stderr, "PASS: ROI display placement — full frame, absolute "
                             "render, ROI-relative algo stream\n");
    }

    std::fprintf(stderr, "PASS: flip_x correctly applied in render_frame() and events_window_ready\n");
    return 0;
}
