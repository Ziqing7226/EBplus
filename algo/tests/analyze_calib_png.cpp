// analyze_calib_png.cpp — load a PNG and try findCirclesGrid with many configs.
// Links against OpenCV only (no SDK), builds fast. Used to find the right board
// size / detector / flags for the calibration board visible in the event frame.
//
// Usage: analyze_calib_png <image.png>

#include <cstdio>
#include <string>
#include <vector>

#include "algo/common/opencv5_compat.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <image.png>\n", argv[0]);
        return 2;
    }
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::fprintf(stderr, "failed to load %s\n", argv[1]);
        return 1;
    }
    std::fprintf(stderr, "[analyze] %s: %dx%d, channels=%d\n",
                 argv[1], img.cols, img.rows, img.channels());

    // Dark-pixel ratio + connected components.
    cv::Mat bw = img < 128;
    int dark = cv::countNonZero(bw);
    cv::Mat labels, stats_cc, centroids;
    int ncc = cv::connectedComponentsWithStats(bw, labels, stats_cc, centroids, 8);
    int big_blobs = 0, med_blobs = 0;
    for (int k = 1; k < ncc; ++k) {
        int a = stats_cc.at<int>(k, cv::CC_STAT_AREA);
        if (a >= 20) ++big_blobs;
        if (a >= 50 && a <= 5000) ++med_blobs;
    }
    std::fprintf(stderr, "[analyze] dark=%.1f%%  CC=%d  big(>=20)=%d  med[50,5000]=%d\n",
                 100.0 * dark / (img.cols * img.rows), ncc - 1, big_blobs, med_blobs);

    // Spatial density map: divide frame into 12×8 grid, report dark % per cell.
    // Reveals whether events form a structured pattern or are uniform noise.
    {
        const int gw = 12, gh = 8;
        const int cw = img.cols / gw, ch = img.rows / gh;
        std::fprintf(stderr, "[analyze] spatial density (%% dark per 12x8 cell):\n");
        for (int gy = 0; gy < gh; ++gy) {
            std::fprintf(stderr, "  ");
            for (int gx = 0; gx < gw; ++gx) {
                cv::Rect roi(gx * cw, gy * ch, cw, ch);
                double d = 100.0 * cv::countNonZero(bw(roi)) / (cw * ch);
                std::fprintf(stderr, "%4.1f ", d);
            }
            std::fprintf(stderr, "\n");
        }
    }

    // Tuned detector: relaxed filters for noisy event-painted blobs.
    cv::SimpleBlobDetector::Params p;
    p.thresholdStep = 10;
    p.minThreshold = 10;
    p.maxThreshold = 220;
    p.minRepeatability = 1;
    p.filterByColor = true;
    p.blobColor = 0;
    p.filterByArea = true;
    p.minArea = 5;
    p.maxArea = 500000;
    p.filterByCircularity = false;
    p.filterByInertia = false;
    p.filterByConvexity = false;
    auto det = cv::SimpleBlobDetector::create(p);

    // Downscale for faster detection.
    cv::Mat det_gray = img;
    if (img.cols > 480) {
        cv::resize(img, det_gray, cv::Size(), 480.0 / img.cols,
                   480.0 / img.cols, cv::INTER_AREA);
    }

    // --- Preprocessing strategies ---
    // Event frame polarity: dark (0) = events, white (255) = background.
    // In OpenCV: erode = MIN filter (expands dark), dilate = MAX (shrinks dark).
    //
    // To remove noise then fill rings, the correct sequence is:
    //   dilate(small) → removes isolated dark noise pixels (shrinks dark)
    //   erode(large)  → expands remaining dark ring edges, merging/filling interiors
    //
    // Strategies:
    //   raw        — no preprocessing (baseline)
    //   d3e5/7/9/11/15 — dilate 3x3 (denoise) then erode NxN (fill rings)
    //   blur15/21  — Gaussian blur then re-threshold at mean
    //   blur15_e3  — blur15 then dilate3+erode3 (smooth + clean up)
    //   inv_blur   — invert + blur (smooth interior blobs)
    struct PreProc { const char* tag; };
    const std::vector<PreProc> preprocs = {
        {"raw"},
        {"d3e5"},
        {"d3e7"},
        {"d3e9"},
        {"d3e11"},
        {"d3e15"},
        {"blur15"},
        {"blur21"},
        {"blur15_e3"},
        {"inv_blur"},
    };
    auto preprocess = [&](const cv::Mat& src, const std::string& tag) -> cv::Mat {
        cv::Mat out = src.clone();
        if (tag.substr(0, 3) == "d3e") {
            const int ek = std::stoi(tag.substr(3));
            auto kd = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            auto ke = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ek, ek));
            cv::dilate(out, out, kd);   // shrink dark (remove noise)
            cv::erode(out, out, ke);    // expand dark (fill rings)
        } else if (tag == "blur15") {
            cv::GaussianBlur(out, out, cv::Size(15, 15), 0);
            cv::Scalar mean, sd; cv::meanStdDev(out, mean, sd);
            cv::threshold(out, out, mean[0], 255, cv::THRESH_BINARY);
        } else if (tag == "blur21") {
            cv::GaussianBlur(out, out, cv::Size(21, 21), 0);
            cv::Scalar mean, sd; cv::meanStdDev(out, mean, sd);
            cv::threshold(out, out, mean[0], 255, cv::THRESH_BINARY);
        } else if (tag == "blur15_e3") {
            cv::GaussianBlur(out, out, cv::Size(15, 15), 0);
            cv::Scalar mean, sd; cv::meanStdDev(out, mean, sd);
            cv::threshold(out, out, mean[0], 255, cv::THRESH_BINARY);
            auto k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            cv::dilate(out, out, k);
            cv::erode(out, out, k);
        } else if (tag == "inv_blur") {
            cv::bitwise_not(out, out);
            cv::GaussianBlur(out, out, cv::Size(15, 15), 0);
        }
        return out;
    };

    // Focus on common calibration board sizes (asymmetric only — symmetric
    // findCirclesGrid asserts with a custom detector in OpenCV 4.x).
    struct BoardCfg { int cols; int rows; bool asym; };
    const std::vector<BoardCfg> cfgs = {
        {6, 5, true}, {5, 6, true},
        {4, 11, true}, {11, 4, true},
        {4, 7, true}, {7, 4, true},
        {5, 4, true}, {4, 5, true},
        {5, 6, true}, {6, 5, true},
        {4, 3, true}, {3, 4, true},
        {5, 8, true}, {8, 5, true},
        {6, 7, true}, {7, 6, true},
        {4, 9, true}, {9, 4, true},
        {7, 8, true}, {8, 7, true},
        {6, 9, true}, {9, 6, true},
        {7, 10, true}, {10, 7, true},
        {8, 9, true}, {9, 8, true},
        {5, 10, true}, {10, 5, true},
        {6, 11, true}, {11, 6, true},
    };

    std::fprintf(stderr, "\n[analyze] === %zu preproc x %zu board configs ===\n",
                 preprocs.size(), cfgs.size());
    int total_found = 0;
    for (const auto& pp : preprocs) {
        cv::Mat pp_gray = preprocess(det_gray, pp.tag);

        // Save preprocessed frame for visual inspection.
        char fn[256];
        std::snprintf(fn, sizeof(fn), "/tmp/calib_probe/pp_%s.png", pp.tag);
        cv::imwrite(fn, pp_gray);

        // Quick blob stats on preprocessed frame.
        cv::Mat bw_pp = pp_gray < 128;
        int dark_pp = cv::countNonZero(bw_pp);
        cv::Mat lab_pp, st_pp, ce_pp;
        int ncc_pp = cv::connectedComponentsWithStats(bw_pp, lab_pp, st_pp, ce_pp, 8);
        int med_pp = 0;
        for (int k = 1; k < ncc_pp; ++k) {
            int a = st_pp.at<int>(k, cv::CC_STAT_AREA);
            if (a >= 50 && a <= 5000) ++med_pp;
        }
        std::fprintf(stderr, "\n  [pp=%-9s] dark=%5.1f%%  CC=%4d  med[50,5000]=%4d\n",
                     pp.tag, 100.0 * dark_pp / (pp_gray.cols * pp_gray.rows),
                     ncc_pp - 1, med_pp);

        for (const auto& c : cfgs) {
            std::vector<cv::Point2f> corners;
            bool found = false;
            try {
                found = cv::findCirclesGrid(pp_gray, cv::Size(c.cols, c.rows),
                                            corners, cv::CALIB_CB_ASYMMETRIC_GRID, det);
            } catch (const cv::Exception&) {
                continue;
            }
            if (found) {
                total_found++;
                std::fprintf(stderr, "    *** DETECTED: %s %2dx%2d (%zu pts) ***\n",
                             c.asym ? "asym" : "sym ", c.cols, c.rows, corners.size());
                cv::Mat vis;
                cv::cvtColor(pp_gray, vis, cv::COLOR_GRAY2BGR);
                const double sx = static_cast<double>(img.cols) / pp_gray.cols;
                for (auto& pt : corners) { pt.x *= float(sx); pt.y *= float(sx); }
                cv::drawChessboardCorners(vis, cv::Size(c.cols, c.rows), corners, true);
                char f2[256];
                std::snprintf(f2, sizeof(f2), "/tmp/calib_probe/FOUND_pp%s_%dx%d.png",
                              pp.tag, c.cols, c.rows);
                cv::imwrite(f2, vis);
                std::fprintf(stderr, "    saved -> %s\n", f2);
            }
        }
    }
    std::fprintf(stderr, "\n[analyze] total detections: %d\n", total_found);
    return 0;
}
