#pragma once

#include "algo/calibration/rig_calibration.h"
#include "camera_stream.h"

namespace gui::multicam {

using CalibrationObservationGroup = gui_algo::rig::Observation;
using RigCalibrationResult = gui_algo::rig::Result;

struct DetectedGroup {
    qint64 start_us = 0;
    std::array<cv::Size, 3> sizes;
    std::array<std::vector<cv::Point2f>, 3> corners;
};

DetectedGroup detect_group(const std::array<CountWindow, 3> &windows);
QJsonObject calibration_json(const gui_algo::rig::Result &, const RigConfig &);
gui_algo::rig::Result read_calibration(const QJsonObject &, const RigConfig &,
                                       const std::array<cv::Size, 3> &sizes);

class Guidance {
  public:
    void reset();
    void pause();
    void observe(const DetectedGroup &group, bool collect, bool manual, int supplemental_camera = -1);
    void remove_last();
    void remove_observation(int index);
    void set_result(const gui_algo::rig::Result &result);
    CalibrationStatus status(const RigConfig &) const;
    std::array<QVector<QPointF>, 3> corners() const;
    const std::vector<gui_algo::rig::Observation> &training() const { return training_; }
    const std::vector<gui_algo::rig::Observation> &validation() const { return validation_; }
    std::array<std::vector<std::vector<cv::Point2f>>, 3> intrinsic_views() const;
    const auto &sizes() const { return sizes_; }
    const auto &result() const { return result_; }
    bool stable() const { return stable_; }
    QJsonObject observations_json() const;
    void import_observations(const QJsonObject &, const RigConfig &);

  private:
    CalibrationStatus build_status(const RigConfig &) const;
    mutable bool status_dirty_ = true;
    mutable QJsonObject status_config_;
    mutable CalibrationStatus cached_status_;
    void invalidate_validation();
    bool duplicate(const DetectedGroup &, int supplemental) const;
    std::vector<gui_algo::rig::Observation> training_, validation_;
    std::array<std::vector<std::vector<cv::Point2f>>, 3> supplemental_;
    std::array<cv::Size, 3> sizes_;
    DetectedGroup last_, anchor_;
    qint64 stable_since_ = -1, last_time_ = -1;
    qint64 next_id_ = 1;
    bool stable_ = false;
    int stable_camera_ = -2;
    double progress_ = 0;
    int target_ = 4;
    QString reason_ = "Show the complete board in all three cameras.";
    gui_algo::rig::Result result_;
};
} // namespace gui::multicam
