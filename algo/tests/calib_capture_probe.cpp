// algo/tests/calib_capture_probe.cpp — diagnostic for the blinking-chessboard
// calibration capture.
//
// Replays a .raw file and, at sampled timestamps, accumulates the last W µs of
// CD events into per-polarity masks, builds the binary blink frame (exactly
// what CalibrationWizard + CalibrationWorker produce) and runs chessboard
// detection on it. Saves annotated PNGs and prints a per-window detection tally
// so we can see which capture windows succeed in real event data.
//
// Usage: calib_capture_probe <file.raw> [window_us] [sample_period_us] [max_samples] [count_frac]
//   window_us   default 100000 (wizard default capture window)
//   sample_period_us  default 300000
//   max_samples default 20
//   count_frac  default 0.30 (production default). Board threshold as a
//   fraction of the p99 per-pixel count — lower values erode square corners
//   less.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
// OpenCV 5.0 split calib3d: findChessboardCorners/CALIB_CB_* live in
// objdetect, undistortPoints in geometry. Include per major version
// (CV_MAJOR_VERSION comes from the core.hpp include above).
#if CV_MAJOR_VERSION >= 5
#include <opencv2/objdetect.hpp>
#include <opencv2/calib.hpp>
#include <opencv2/geometry.hpp>
#include <opencv2/2d.hpp>   // boundingRect etc. left imgproc in 5.0
#else
#include <opencv2/calib3d.hpp>
#endif
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/utils/timestamp.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/stream/file_config_hints.h>

#include "algo/calibration/blinking_detect.h"
#include "algo/calibration/intrinsic.h"

using Metavision::EventCD;
using Metavision::timestamp;
using gui_algo::accumulate_blink_masks;
using gui_algo::build_blink_frame;

namespace {
// Inner corners (wizard default).
constexpr int kCols = 9, kRows = 6;
} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <file.raw> [window_us] "
                     "[sample_period_us] [max_samples]\n", argv[0]);
        return 2;
    }
    const std::string path = argv[1];
    const timestamp window_us = (argc > 2) ? std::max(timestamp(200), std::atoll(argv[2])) : 100000;
    const timestamp sample_period = (argc > 3) ? std::atoll(argv[3]) : 300000;
    const int max_samples = (argc > 4) ? std::atoi(argv[4]) : 20;
    const double count_frac = (argc > 5) ? std::atof(argv[5]) : 0.30;

    std::fprintf(stderr, "[probe] %s\n[probe] blinking chessboard %dx%d, window=%lld us, "
                 "sample every %lld us, max %d samples\n",
                 path.c_str(), kCols, kRows,
                 static_cast<long long>(window_us),
                 static_cast<long long>(sample_period), max_samples);

    Metavision::FileConfigHints hints;
    hints.real_time_playback(false);
    Metavision::Camera cam;
    try {
        cam = Metavision::Camera::from_file(path, hints);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[probe] open failed: %s\n", e.what());
        return 1;
    }
    const int W = cam.geometry().get_width();
    const int H = cam.geometry().get_height();
    std::fprintf(stderr, "[probe] sensor %dx%d\n", W, H);

    // Keep last 260 ms in the ring (≥ the 200 ms maximum capture window).
    constexpr timestamp kKeepUs = 260000;
    std::deque<EventCD> ring;
    timestamp last_t = 0;

    gui_algo::IntrinsicCalibration calib;
    calib.set_pattern(gui_algo::CalibrationPattern::Chessboard, kCols, kRows, 20.0f);

    int hits = 0;
    int total_samples = 0;

    timestamp next_sample = sample_period;
    int sample_idx = 0;
    bool done = false;
    const std::string outdir = "/tmp/blink_chessboard_probe";
    std::error_code ec;
    std::filesystem::create_directories(outdir, ec);

    // Long-term density map: accumulate all events over the recording to see
    // the stable structure (the board) behind the per-window noise.
    cv::Mat_<int> density = cv::Mat_<int>::zeros(H, W);

    cam.cd().add_callback([&](const EventCD* b, const EventCD* e) {
        for (const EventCD* p = b; p != e; ++p) {
            ring.push_back(*p);
            last_t = p->t;
            density(static_cast<int>(p->y), static_cast<int>(p->x)) += 1;
        }
        while (!ring.empty() && last_t - ring.front().t > kKeepUs) ring.pop_front();

        if (done || sample_idx >= max_samples || last_t < next_sample) return;
        next_sample = last_t + sample_period;
        const int s = sample_idx++;
        total_samples++;

        // Count events in the capture window for diagnostics.
        const timestamp t0 = last_t - window_us;
        std::size_t n_ev = 0;
        for (auto it = ring.rbegin(); it != ring.rend(); ++it) {
            if (it->t < t0) break;
            ++n_ev;
        }

        // Blink frame from the last window (same path as the wizard).
        std::vector<EventCD> win;
        for (auto it = ring.rbegin(); it != ring.rend(); ++it) {
            if (it->t < t0) break;
            win.push_back(*it);
        }
        // Wizard path: per-pixel counts + adaptive threshold (the binary
        // both-mask is solid on a real LCD — backlight PWM gives every screen
        // pixel both polarities, so it is only kept as a diagnostic below).
        cv::Mat_<int> on_cnt = cv::Mat_<int>::zeros(H, W);
        cv::Mat_<int> off_cnt = cv::Mat_<int>::zeros(H, W);
        for (const auto& ev : win) {
            if (ev.p != 0) on_cnt(static_cast<int>(ev.y), static_cast<int>(ev.x)) += 1;
            else           off_cnt(static_cast<int>(ev.y), static_cast<int>(ev.x)) += 1;
        }
        gui_algo::BlinkParams bp;
        bp.ratio_on = 1.0;
        bp.ratio_off = 1.0;
        bp.count_frac = count_frac;
        const gui_algo::BlinkFrame bf = gui_algo::build_blink_frame_from_counts(
            on_cnt, off_cnt, bp);
        auto res = calib.detect_only(bf.frame, true);
        if (res.found) hits++;

        // Structural diagnostics on the blink frame (black = board px).
        cv::Mat bf_bin;
        cv::threshold(bf.frame, bf_bin, 128, 255, cv::THRESH_BINARY_INV);
        const int n_black = cv::countNonZero(bf_bin);
        double bfrac = 0.0, bb_area = 0.0;
        std::vector<std::vector<cv::Point>> bcontours;
        cv::findContours(bf_bin, bcontours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (!bcontours.empty()) {
            std::sort(bcontours.begin(), bcontours.end(),
                      [](const auto& a, const auto& b) { return cv::contourArea(a) > cv::contourArea(b); });
            const auto& big = bcontours[0];
            cv::Rect bb = cv::boundingRect(big);
            bb_area = static_cast<double>(bb.width) * bb.height;
            bfrac = static_cast<double>(n_black) / (W * H);
            std::fprintf(stderr, "[probe]   board=%d (%.1f%%)  largest comp bbox=%dx%d+%d+%d  "
                         "valid=%d\n",
                         n_black, 100.0 * bfrac,
                         bb.width, bb.height, bb.x, bb.y, bf.valid ? 1 : 0);
        } else {
            std::fprintf(stderr, "[probe]   board=%d (0%%) no contours  valid=%d\n",
                         n_black, bf.valid ? 1 : 0);
        }

        std::fprintf(stderr, "[probe] sample %d @ t=%lld us  events=%zu  both=%d  %s\n",
                     s, static_cast<long long>(last_t), n_ev, bf.both,
                     res.found ? "DETECTED" : "no");

        // Per-sample coarse ASCII of the actual wizard-path count frame.
        {
            std::fprintf(stderr, "[probe]   count-frame ASCII ('#'=board):\n");
            for (int by = 0; by < 20; ++by) {
                std::fprintf(stderr, "[probe]     ");
                for (int bx = 0; bx < 80; ++bx) {
                    const int x0 = bx * W / 80, x1 = (bx + 1) * W / 80;
                    const int y0 = by * H / 20, y1 = (by + 1) * H / 20;
                    int nb = 0, nc = 0;
                    for (int yy = y0; yy < y1; ++yy) {
                        const uchar* row = bf.frame.ptr<uchar>(yy);
                        for (int xx = x0; xx < x1; ++xx) {
                            ++nc;
                            if (row[xx] < 128) ++nb;
                        }
                    }
                    std::fprintf(stderr, "%c", nb > nc / 2 ? '#' : '.');
                }
                std::fprintf(stderr, "\n");
            }
        }

        // Detection-flag variants (raw OpenCV) to see what the frame supports.
        {
            std::vector<cv::Point2f> corners;
            struct Variant { const char* name; int flags; };
            const Variant variants[] = {
                {"none", 0},
                {"FQ", cv::CALIB_CB_FILTER_QUADS},
                {"AT", cv::CALIB_CB_ADAPTIVE_THRESH},
                {"FQ|AT", cv::CALIB_CB_FILTER_QUADS | cv::CALIB_CB_ADAPTIVE_THRESH},
                {"FQ|EXHAUSTIVE", cv::CALIB_CB_FILTER_QUADS | cv::CALIB_CB_EXHAUSTIVE},
            };
            for (const auto& v : variants) {
                const bool ok = cv::findChessboardCorners(bf.frame, {kCols, kRows},
                                                          corners, v.flags);
                std::fprintf(stderr, "[probe]   detect(9,6) %-14s %s\n",
                             v.name, ok ? "OK" : "no");
            }

            // Step-level: FQ found → cornerSubPix → straightness. Which step
            // rejects the (valid) FQ detection in detect_only?
            auto worst_bend = [](const std::vector<cv::Point2f>& pts,
                                 int& wr, int& wc) {
                double worst = 1.0;
                wr = -1; wc = -1;
                for (int r = 0; r < kRows; ++r) {
                    for (int c = 1; c < kCols - 1; ++c) {
                        const cv::Point2f& a = pts[r * kCols + c - 1];
                        const cv::Point2f& b = pts[r * kCols + c];
                        const cv::Point2f& cc = pts[r * kCols + c + 1];
                        const cv::Point2f v1 = b - a, v2 = cc - b;
                        const double n1 = cv::norm(v1), n2 = cv::norm(v2);
                        if (n1 > 1e-6 && n2 > 1e-6) {
                            const double cs = (v1.x * v2.x + v1.y * v2.y) / (n1 * n2);
                            if (cs < worst) { worst = cs; wr = r; wc = c; }
                        }
                    }
                }
                for (int c = 0; c < kCols; ++c) {
                    for (int r = 1; r < kRows - 1; ++r) {
                        const cv::Point2f& a = pts[(r - 1) * kCols + c];
                        const cv::Point2f& b = pts[r * kCols + c];
                        const cv::Point2f& cc = pts[(r + 1) * kCols + c];
                        const cv::Point2f v1 = b - a, v2 = cc - b;
                        const double n1 = cv::norm(v1), n2 = cv::norm(v2);
                        if (n1 > 1e-6 && n2 > 1e-6) {
                            const double cs = (v1.x * v2.x + v1.y * v2.y) / (n1 * n2);
                            if (cs < worst) { worst = cs; wr = r; wc = c; }
                        }
                    }
                }
                return worst;
            };
            if (cv::findChessboardCorners(bf.frame, {kCols, kRows}, corners,
                                          cv::CALIB_CB_FILTER_QUADS)) {
                int wr, wc;
                const double w_raw = worst_bend(corners, wr, wc);
                cv::Mat gray;
                if (bf.frame.channels() == 1) gray = bf.frame;
                else cv::cvtColor(bf.frame, gray, cv::COLOR_BGR2GRAY);
                cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                     40, 0.001));
                const double w_sub = worst_bend(corners, wr, wc);
                std::fprintf(stderr, "[probe]   FQ worst cos raw=%.4f subpix=%.4f "
                             "[cos(pi/9)=%.4f] %s\n",
                             w_raw, w_sub, std::cos(3.14159 / 9),
                             w_sub >= std::cos(3.14159 / 9) ? "PASSES" : "REJECTED");
            }
        }

        // Coarse ASCII map of the blink frame (black='#'): shows whether the
        // both-polarity region is a clean checkerboard or corrupted.
        {
            std::fprintf(stderr, "[probe]   ASCII map (%.0fx%.0f blocks, '#'=black):\n",
                         80.0, 40.0);
            const int cb = 128;
            for (int by = 0; by < 40; ++by) {
                std::fprintf(stderr, "[probe]     ");
                for (int bx = 0; bx < 80; ++bx) {
                    const int x0 = bx * W / 80, x1 = (bx + 1) * W / 80;
                    const int y0 = by * H / 40, y1 = (by + 1) * H / 40;
                    int n_black_cell = 0, n_cell = 0;
                    for (int yy = y0; yy < y1; ++yy) {
                        const uchar* row = bf.frame.ptr<uchar>(yy);
                        for (int xx = x0; xx < x1; ++xx) {
                            ++n_cell;
                            if (row[xx] < cb) ++n_black_cell;
                        }
                    }
                    std::fprintf(stderr, "%c",
                                 n_black_cell > n_cell / 2 ? '#' : '.');
                }
                std::fprintf(stderr, "\n");
            }
        }

        // First sample only: per-polarity ASCII maps + polarity event counts.
        if (s == 0) {
            // Per-pixel total-count histogram: if black squares (high count)
            // separate from white squares / noise (low count), a count
            // threshold reconstructs the checkerboard where the binary
            // both-mask fails.
            {
                cv::Mat_<int> cnt = cv::Mat_<int>::zeros(H, W);
                for (auto it = ring.rbegin(); it != ring.rend(); ++it) {
                    if (it->t < t0) break;
                    cnt(static_cast<int>(it->y), static_cast<int>(it->x)) += 1;
                }
                std::vector<long long> hist(64, 0);
                int cmax = 0;
                for (int y = 0; y < H; ++y)
                    for (int x = 0; x < W; ++x) cmax = std::max(cmax, cnt(y, x));
                for (int y = 0; y < H; ++y) {
                    for (int x = 0; x < W; ++x) {
                        const int c = cnt(y, x);
                        const int bin = (cmax > 0)
                            ? static_cast<int>(64.0 * std::log1p(c) / std::log1p(cmax))
                            : 0;
                        if (bin >= 0 && bin < 64) ++hist[bin];
                    }
                }
                std::fprintf(stderr, "[probe]   count histogram (log bins, "
                             "max per-pixel count %d):\n", cmax);
                for (int b = 0; b < 64; b += 4) {
                    long long s = 0;
                    for (int k = 0; k < 4 && b + k < 64; ++k) s += hist[b + k];
                    const int bar = static_cast<int>(
                        std::min(60LL, s / std::max(1LL, *std::max_element(
                            hist.begin(), hist.end()) / 60)));
                    std::fprintf(stderr, "[probe]     %2d-%2d: %9lld %s\n",
                                 b, b + 3, s, std::string(bar, '#').c_str());
                }
            }
            // Per-pixel event-count profile along a row through the blob:
            // does the count alternate (checkerboard) or is it flat (noise)?
            {
                const int py = 200;
                cv::Mat_<int> counts = cv::Mat_<int>::zeros(1, W);
                for (auto it = ring.rbegin(); it != ring.rend(); ++it) {
                    if (it->t < t0) break;
                    if (static_cast<int>(it->y) == py) {
                        counts(0, static_cast<int>(it->x)) += 1;
                    }
                }
                std::fprintf(stderr, "[probe]   count profile row y=%d (per-pixel "
                             "events, 80 cols of 16px):\n", py);
                for (int bx = 0; bx < 80; ++bx) {
                    const int x0 = bx * W / 80, x1 = (bx + 1) * W / 80;
                    int s = 0;
                    for (int xx = x0; xx < x1; ++xx) s += counts(0, xx);
                    const int bar = std::min(30, s / 30);
                    std::fprintf(stderr, "[probe]     %3d: %6d %s\n",
                                 bx, s, std::string(bar, '#').c_str());
                }
            }

            // Temporal histogram: events per ms over the window. A 10 ms
            // blink + 10 ms blank pattern shows 20 ms bursts; uniform refresh
            // noise is flat.
            std::fprintf(stderr, "[probe]   event rate (Kev/ms) over the window:\n");
            const int n_bins = static_cast<int>(window_us / 1000);
            std::vector<int> hist(n_bins, 0);
            for (auto it = ring.rbegin(); it != ring.rend(); ++it) {
                if (it->t < t0) break;
                const int bin = static_cast<int>((it->t - t0) / 1000);
                if (bin >= 0 && bin < n_bins) ++hist[bin];
            }
            for (int b = 0; b < n_bins; b += 5) {
                std::fprintf(stderr, "[probe]     %4d ms:", b);
                for (int k = 0; k < 5 && b + k < n_bins; ++k) {
                    std::fprintf(stderr, " %5.1f", hist[b + k] / 1000.0);
                }
                std::fprintf(stderr, "\n");
            }

            // Per-pixel timelines: how many events, and the polarity pattern,
            // at a few probe pixels across the blob.
            const cv::Point probes[] = {{640, 360}, {640, 200}, {400, 200},
                                        {900, 100}, {300, 600}, {1100, 600}};
            for (const auto& pt : probes) {
                std::vector<EventCD> seq;
                for (auto it = ring.rbegin(); it != ring.rend(); ++it) {
                    if (it->t < t0) break;
                    if (static_cast<int>(it->x) == pt.x &&
                        static_cast<int>(it->y) == pt.y) {
                        seq.push_back(*it);
                    }
                }
                std::fprintf(stderr, "[probe]   pixel (%d,%d): %zu events; ts(us)=",
                             pt.x, pt.y, seq.size());
                int shown = 0;
                for (auto it = seq.rbegin(); it != seq.rend() && shown < 12;
                     ++it, ++shown) {
                    std::fprintf(stderr, " %lld/%c",
                                 static_cast<long long>(it->t - t0),
                                 it->p != 0 ? '+' : '-');
                }
                if (seq.size() > 12) std::fprintf(stderr, " ...");
                std::fprintf(stderr, "\n");
            }

            auto dump_pol = [&](const char* label, const cv::Mat& m) {
                std::fprintf(stderr, "[probe]   %s map:", label);
                for (int by = 0; by < 20; ++by) {
                    std::fprintf(stderr, "\n[probe]     ");
                    for (int bx = 0; bx < 80; ++bx) {
                        const int x0 = bx * W / 80, x1 = (bx + 1) * W / 80;
                        const int y0 = by * H / 20, y1 = (by + 1) * H / 20;
                        int n_on_cell = 0, n_cell = 0;
                        for (int yy = y0; yy < y1; ++yy) {
                            const uchar* row = m.ptr<uchar>(yy);
                            for (int xx = x0; xx < x1; ++xx) {
                                ++n_cell;
                                n_on_cell += row[xx] != 0;
                            }
                        }
                        std::fprintf(stderr, "%c", n_on_cell > n_cell / 2 ? '#' : '.');
                    }
                }
                std::fprintf(stderr, "\n");
            };
            cv::Mat has_on(H, W, CV_8UC1, cv::Scalar(0));
            cv::Mat has_off(H, W, CV_8UC1, cv::Scalar(0));
            for (int y = 0; y < H; ++y) {
                for (int x = 0; x < W; ++x) {
                    if (on_cnt(y, x) > 0) has_on.at<uchar>(y, x) = 1;
                    if (off_cnt(y, x) > 0) has_off.at<uchar>(y, x) = 1;
                }
            }
            dump_pol("has_on ", has_on);
            dump_pol("has_off", has_off);
        }

        // Save annotated PNG (downscale to 1024 wide for inspection).
        cv::Mat vis = res.image.empty() ? bf.frame : res.image;
        if (vis.cols > 1024) {
            cv::resize(vis, vis, cv::Size(), 1024.0 / vis.cols, 1024.0 / vis.cols,
                       cv::INTER_NEAREST);
        }
        char fn[256];
        std::snprintf(fn, sizeof(fn), "%s/s%02d_%s.png",
                      outdir.c_str(), s, res.found ? "OK" : "no");
        cv::imwrite(fn, vis);

        if (sample_idx >= max_samples) done = true;
    });

    cam.start();
    while (cam.is_running()) {
        if (done) { cam.stop(); break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (cam.is_running()) cam.stop();

    std::fprintf(stderr, "[probe] === summary: %d samples, blinking chessboard %dx%d, "
                 "window=%lld us ===\n",
                 total_samples, kCols, kRows,
                 static_cast<long long>(window_us));
    std::fprintf(stderr, "[probe] detected %d / %d\n", hits, total_samples);
    std::fprintf(stderr, "[probe] PNGs in %s/ (tagged _OK / _no)\n", outdir.c_str());

    // Long-term density map (whole recording) — coarse ASCII. Shows whether a
    // stable structure (the board) exists behind the per-window noise.
    {
        int dmax = 1;
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x) dmax = std::max(dmax, density(y, x));
        std::fprintf(stderr, "[probe] long-term density map (whole recording, "
                     "max count %d):\n", dmax);
        for (int by = 0; by < 40; ++by) {
            std::fprintf(stderr, "[probe]   ");
            for (int bx = 0; bx < 80; ++bx) {
                const int x0 = bx * W / 80, x1 = (bx + 1) * W / 80;
                const int y0 = by * H / 40, y1 = (by + 1) * H / 40;
                long long sum = 0;
                int n = 0;
                for (int yy = y0; yy < y1; ++yy) {
                    const int* row = density.ptr<int>(yy);
                    for (int xx = x0; xx < x1; ++xx) { sum += row[xx]; ++n; }
                }
                const double avg = static_cast<double>(sum) / std::max(1, n);
                std::fprintf(stderr, "%c", avg > 0.35 * dmax ? '#' :
                                           avg > 0.12 * dmax ? '+' : '.');
            }
            std::fprintf(stderr, "\n");
        }
    }
    return 0;
}
