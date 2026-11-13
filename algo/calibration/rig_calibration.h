#pragma once

#include <array>
#include <cstdint>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace gui_algo::rig {

constexpr int camera_count = 3;
constexpr int corner_count = 54;

struct CameraModel {
    cv::Size size;
    cv::Mat K;
    cv::Mat distortion;
};

struct Observation {
    std::int64_t id = 0;
    std::int64_t start_us = 0;
    std::int64_t end_us = 0;
    std::array<std::vector<cv::Point2f>, camera_count> corners;
};

struct Result {
    bool trained = false;
    bool validated = false;
    int reference = 1;
    double square_m = 0;
    std::array<CameraModel, camera_count> models;
    // X_camera = T_camera_from_reference * X_reference; lengths are metres.
    std::array<cv::Matx44d, camera_count> T_camera_from_reference;
    std::array<double, camera_count> training_rms{};
    std::vector<std::array<double, camera_count>> training_views;
    std::array<double, camera_count> validation_rms{};
    std::vector<std::array<double, camera_count>> validation_views;
    std::vector<std::int64_t> training_ids;
    std::vector<std::int64_t> validation_ids;
    // Preserve the actual fitting inputs so relabeling cannot create a hold-out.
    std::vector<Observation> training_observations;
    std::array<std::vector<std::vector<cv::Point2f>>, camera_count> intrinsic_training_views;
    std::vector<std::array<std::vector<cv::Point2f>, camera_count>> validation_projected;
    std::vector<std::array<std::vector<cv::Point2f>, camera_count>> validation_observed;
    std::string error;
    Result();
};

std::vector<cv::Point3f> object_points(double square_m);
Result solve_joint(const std::array<CameraModel, camera_count> &models,
                   const std::vector<Observation> &observations, int reference, double square_m);
Result solve_intrinsics_and_joint(
    const std::array<cv::Size, camera_count> &sizes,
    const std::array<std::vector<std::vector<cv::Point2f>>, camera_count> &intrinsic_views,
    const std::vector<Observation> &common_views, int reference, double square_m);
Result validate(const Result &trained, const std::vector<Observation> &held_out, double max_rms_px);

} // namespace gui_algo::rig
