#include "calibration.h"
#include "algo/calibration/blinking_detect.h"
#include "algo/calibration/intrinsic.h"
#include <QJsonArray>
#include <QStringList>
#include <algorithm>
#include <cmath>
#include <functional>
#include <future>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace gui::multicam {
namespace {
using Points = std::vector<cv::Point2f>;
bool valid_size(cv::Size size) {
    return size.width > 0 && size.height > 0 && size.width <= std::numeric_limits<int>::max() / size.height;
}
bool valid_points(const Points &points, cv::Size size) {
    if (!valid_size(size) || points.size() != 54)
        return false;
    for (const auto &p : points)
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 || p.x >= size.width ||
            p.y >= size.height)
            return false;
    return true;
}
double displacement(const Points &a, const Points &b) {
    if (a.size() != 54 || b.size() != 54)
        return std::numeric_limits<double>::infinity();
    double normal = 0, reversed = 0;
    for (size_t j = 0; j < a.size(); ++j) {
        normal += cv::norm(a[j] - b[j]) * cv::norm(a[j] - b[j]);
        reversed += cv::norm(a[j] - b[a.size() - 1 - j]) * cv::norm(a[j] - b[a.size() - 1 - j]);
    }
    return std::sqrt(std::min(normal, reversed) / a.size());
}
QJsonArray points_json(const Points &points) {
    QJsonArray a;
    for (const auto &p : points)
        a.append(QJsonArray{p.x, p.y});
    return a;
}
Points read_points(const QJsonArray &a) {
    if (a.size() != 54)
        throw std::runtime_error("An observation needs 54 corners.");
    Points points;
    for (const auto &v : a) {
        const auto xy = v.toArray();
        if (xy.size() != 2)
            throw std::runtime_error("Invalid corner.");
        const double x = xy[0].toDouble(NAN), y = xy[1].toDouble(NAN);
        if (!std::isfinite(x) || !std::isfinite(y))
            throw std::runtime_error("Non-finite corner.");
        points.emplace_back(x, y);
    }
    return points;
}
QJsonArray matrix_json(const cv::Mat &m) {
    QJsonArray a;
    for (int i = 0; i < m.rows; ++i)
        for (int j = 0; j < m.cols; ++j)
            a.append(m.at<double>(i, j));
    return a;
}
cv::Mat read_matrix(const QJsonArray &a, int rows, int cols) {
    if (a.size() != rows * cols)
        throw std::runtime_error("Invalid calibration matrix shape.");
    cv::Mat m(rows, cols, CV_64F);
    for (int i = 0; i < rows * cols; ++i) {
        const double x = a[i].toDouble(NAN);
        if (!std::isfinite(x))
            throw std::runtime_error("Non-finite calibration matrix.");
        m.at<double>(i / cols, i % cols) = x;
    }
    return m;
}
cv::Point2f center(const Points &p) {
    cv::Point2f c;
    for (const auto &v : p)
        c += v;
    return c / float(p.size());
}
int region(const Points &p, cv::Size size) {
    if (p.empty() || !valid_size(size))
        return 4;
    auto c = center(p);
    return std::clamp(int(c.y * 3 / size.height), 0, 2) * 3 + std::clamp(int(c.x * 3 / size.width), 0, 2);
}
} // namespace

DetectedGroup detect_group(const std::array<CountWindow, 3> &windows) {
    DetectedGroup out;
    out.start_us = windows[0].start_us;
    for (int i = 0; i < 3; ++i) {
        if (windows[i].start_us != out.start_us)
            throw std::runtime_error("Calibration windows are not simultaneous.");
        out.sizes[i] = windows[i].on.size();
    }
    const auto detect = [](const CountWindow &window) {
        gui_algo::IntrinsicCalibration detector;
        detector.set_pattern(gui_algo::CalibrationPattern::Chessboard, 9, 6, 1);
        for (const double frac : {0.30, 0.20, 0.42}) {
            gui_algo::BlinkParams params;
            params.ratio_on = 1;
            params.ratio_off = 1;
            params.count_frac = frac;
            if (frac > 0.35)
                params.median_blur_diameter = 5;
            auto frame = gui_algo::build_blink_frame_from_counts(window.on, window.off, params);
            if (frame.valid) {
                auto detection = detector.detect_only(frame.frame, false);
                if (detection.found)
                    return std::move(detection.points);
            }
        }
        // Fallback for oblique Left/Right cameras or high-threshold / STC-filtered streams where
        // the 99th-percentile per-pixel count in a 100 ms window is below build_blink_frame_from_counts's
        // hardcoded floor of 12 (e.g., 3..11 events per toggling square pixel).
        if (!window.on.empty() && !window.off.empty() && window.on.type() == CV_32S &&
            window.off.type() == CV_32S && window.on.size() == window.off.size()) {
            cv::Mat sum;
            cv::add(window.on, window.off, sum);
            const int p99 = gui_algo::detail::blink_count_threshold(sum, 0, 256, 1.0);
            if (p99 >= 2 && p99 < 45) {
                int last_t = -1;
                for (const double frac : {0.30, 0.20, 0.45}) {
                    const int T = std::clamp(static_cast<int>(std::lround(frac * p99)), 2, 11);
                    if (T == last_t)
                        continue;
                    last_t = T;
                    cv::Mat bin(window.on.rows, window.on.cols, CV_8UC1, cv::Scalar(255));
                    int both = 0;
                    for (int y = 0; y < window.on.rows; ++y) {
                        const int *on = window.on.ptr<int>(y);
                        const int *off = window.off.ptr<int>(y);
                        uchar *row = bin.ptr<uchar>(y);
                        for (int x = 0; x < window.on.cols; ++x) {
                            if (on[x] + off[x] >= T) {
                                row[x] = 0;
                                ++both;
                            }
                        }
                    }
                    if (both >= 100) {
                        cv::medianBlur(bin, bin, 3);
                        auto detection = detector.detect_only(bin, false);
                        if (detection.found)
                            return std::move(detection.points);
                    }
                }
            }
        }
        return Points{};
    };
    // One group has exactly three independent jobs, all joined before its count windows can be released.
    auto left = std::async(std::launch::async, detect, std::cref(windows[0]));
    auto center = std::async(std::launch::async, detect, std::cref(windows[1]));
    out.corners[2] = detect(windows[2]);
    out.corners[0] = left.get();
    out.corners[1] = center.get();
    return out;
}

void Guidance::reset() { *this = Guidance{}; }
void Guidance::pause() {
    status_dirty_ = true;
    last_ = {};
    stable_since_ = -1;
    stable_ = false;
    progress_ = 0;
    reason_ = "Collection paused. Resume and hold a new eligible pose still for one second.";
}
bool Guidance::duplicate(const DetectedGroup &group, int supplemental) const {
    if (result_.trained)
        for (int i = 0; i < 3; ++i)
            if (supplemental < 0 || supplemental == i)
                for (const auto &p : result_.intrinsic_training_views[i])
                    if (displacement(p, group.corners[i]) < 10.0 * group.sizes[i].width / 1280)
                        return true;
    auto same = [&](const auto &observation) {
        if (supplemental >= 0)
            return displacement(group.corners[supplemental], observation.corners[supplemental]) <
                   10.0 * group.sizes[supplemental].width / 1280;
        for (int i = 0; i < 3; ++i)
            if (displacement(group.corners[i], observation.corners[i]) >= 10.0 * group.sizes[i].width / 1280)
                return false;
        return true;
    };
    for (const auto &view : training_)
        if (same(view))
            return true;
    for (const auto &view : validation_)
        if (same(view))
            return true;
    if (supplemental >= 0)
        for (const auto &p : supplemental_[supplemental])
            if (displacement(p, group.corners[supplemental]) < 10.0 * group.sizes[supplemental].width / 1280)
                return true;
    if (supplemental < 0)
        for (int i = 0; i < 3; ++i)
            for (const auto &p : supplemental_[i])
                if (displacement(p, group.corners[i]) < 10.0 * group.sizes[i].width / 1280)
                    return true;
    return false;
}
void Guidance::observe(const DetectedGroup &g, bool collect, bool manual, int supplemental) {
    status_dirty_ = true;
    last_ = g;
    QString failure;
    if (supplemental < -1 || supplemental >= 3 || g.start_us < 0 ||
        g.start_us > std::numeric_limits<qint64>::max() - calibration_window_us)
        failure = "Invalid calibration camera or time window.";
    for (int i = 0; i < 3; ++i) {
        if (!failure.isEmpty())
            break;
        if (valid_size(sizes_[i]) && sizes_[i] != g.sizes[i]) {
            invalidate_validation();
            failure = "Sensor resolution changed; reset calibration before collecting more observations.";
            break;
        }
        if (supplemental >= 0 && supplemental != i)
            continue;
        if (!valid_points(g.corners[i], g.sizes[i])) {
            failure = QString("Camera %1: show the whole board; common capture is blocked.").arg(i + 1);
            break;
        }
        auto box = cv::boundingRect(g.corners[i]);
        if (double(box.area()) / g.sizes[i].area() < .04) {
            failure = QString("Camera %1: board is too small; move the display closer.").arg(i + 1);
            break;
        }
        for (const auto &p : g.corners[i])
            if (p.x < 2 || p.y < 2 || p.x > g.sizes[i].width - 3 || p.y > g.sizes[i].height - 3)
                failure = "Move the complete board away from the image edge.";
    }
    if (failure.isEmpty() && duplicate(g, supplemental))
        failure = "Repeated pose: change position, image scale or tilt before capturing.";
    if (!failure.isEmpty()) {
        stable_camera_ = supplemental >= 0 && supplemental < 3 ? supplemental : -1;
        stable_since_ = -1;
        stable_ = false;
        progress_ = 0;
        reason_ = failure;
        last_time_ = g.start_us;
        return;
    }
    sizes_ = g.sizes;
    bool moved = stable_since_ < 0 || stable_camera_ != supplemental || g.start_us <= last_time_ ||
                 g.start_us - last_time_ > 300000;
    for (int i = 0; i < 3; ++i)
        if (supplemental < 0 || supplemental == i)
            moved =
                moved || displacement(g.corners[i], anchor_.corners[i]) >
                             2.0 * std::hypot(g.sizes[i].width, g.sizes[i].height) / std::hypot(1280., 720.);
    if (moved) {
        anchor_ = g;
        stable_since_ = g.start_us;
        stable_camera_ = supplemental;
    }
    last_time_ = g.start_us;
    progress_ = std::clamp(double(g.start_us - stable_since_) / 1000000, 0., 1.);
    stable_ = progress_ >= 1;
    reason_ = stable_ ? "Current pose is stable and eligible. This is not whole-rig calibration approval."
                      : "Hold the complete board still for 1 second.";
    if (!stable_ || (!collect && !manual))
        return;
    if (supplemental >= 0) {
        if (result_.trained) {
            result_ = {};
            validation_.clear();
        }
        supplemental_[supplemental].push_back(g.corners[supplemental]);
        reason_ = "Intrinsic-only observation accepted; common count unchanged.";
    } else {
        gui_algo::rig::Observation observation;
        observation.id = next_id_++;
        observation.start_us = g.start_us;
        observation.end_us = g.start_us + calibration_window_us;
        observation.corners = g.corners;
        if (result_.trained) {
            invalidate_validation();
            validation_.push_back(std::move(observation));
        } else
            training_.push_back(std::move(observation));
        reason_ = result_.trained ? "Independent validation pose accepted; parameters remain frozen."
                                  : "Common training pose accepted. Move to a new pose.";
    }
    stable_ = false;
    progress_ = 0;
    stable_since_ = -1;
    std::array<int, 9> counts{};
    for (const auto &v : training_)
        ++counts[region(v.corners[1], sizes_[1])];
    target_ = int(std::min_element(counts.begin(), counts.end()) - counts.begin());
}
void Guidance::invalidate_validation() {
    status_dirty_ = true;
    result_.validated = false;
    result_.validation_views.clear();
    result_.validation_ids.clear();
    result_.validation_projected.clear();
    result_.validation_observed.clear();
    result_.validation_rms.fill(0);
    result_.error.clear();
}
void Guidance::remove_last() {
    status_dirty_ = true;
    if (!validation_.empty()) {
        validation_.pop_back();
        invalidate_validation();
    } else if (!training_.empty()) {
        training_.pop_back();
        result_ = {};
    }
    stable_since_ = -1;
    stable_ = false;
    progress_ = 0;
}
void Guidance::set_result(const gui_algo::rig::Result &r) {
    status_dirty_ = true;
    if (r.trained && !r.training_ids.empty()) {
        bool matches = r.training_ids.size() == training_.size();
        for (size_t k = 0; matches && k < training_.size(); ++k) {
            matches = r.training_ids[k] == training_[k].id;
            if (k < r.training_observations.size())
                matches = matches && r.training_observations[k].start_us == training_[k].start_us &&
                          r.training_observations[k].corners == training_[k].corners;
        }
        // Validation of an imported profile has no local training observations.
        if (training_.empty() && r.training_ids == result_.training_ids && !r.validation_ids.empty())
            matches = true;
        if (!matches) {
            training_.clear();
            validation_.clear();
            supplemental_ = {};
        }
        for (int i = 0; i < 3; ++i)
            sizes_[i] = r.models[i].size;
    }
    if (r.trained && !r.validated && r.validation_ids.empty() && r.error.empty())
        validation_.clear();
    result_ = r;
    stable_since_ = -1;
    stable_camera_ = -2;
    stable_ = false;
    progress_ = 0;
    if (!r.error.empty())
        reason_ = QString::fromStdString(r.error);
}
void Guidance::remove_observation(int index) {
    status_dirty_ = true;
    if (index < 0) {
        remove_last();
        return;
    }
    if (index < int(training_.size())) {
        training_.erase(training_.begin() + index);
        validation_.clear();
        result_ = {};
    } else {
        index -= int(training_.size());
        if (index < int(validation_.size())) {
            validation_.erase(validation_.begin() + index);
            invalidate_validation();
        } else {
            index -= int(validation_.size());
            for (int i = 0; i < 3; ++i) {
                if (index < int(supplemental_[i].size())) {
                    supplemental_[i].erase(supplemental_[i].begin() + index);
                    result_ = {};
                    validation_.clear();
                    break;
                }
                index -= int(supplemental_[i].size());
            }
        }
    }
    stable_since_ = -1;
    stable_ = false;
    progress_ = 0;
}
std::array<QVector<QPointF>, 3> Guidance::corners() const {
    std::array<QVector<QPointF>, 3> out;
    for (int i = 0; i < 3; ++i)
        for (const auto &p : last_.corners[i])
            out[i].push_back({p.x, p.y});
    return out;
}
std::array<std::vector<std::vector<cv::Point2f>>, 3> Guidance::intrinsic_views() const {
    auto out = supplemental_;
    for (const auto &view : training_)
        for (int i = 0; i < 3; ++i)
            out[i].push_back(view.corners[i]);
    return out;
}
CalibrationStatus Guidance::status(const RigConfig &c) const {
    auto config = c.to_json();
    config.insert("liveBoardLocked", c.board_locked);
    config.insert("liveOpticsConfirmed", c.optics_confirmed);
    config.insert("liveRolesConfirmed", c.roles_confirmed);
    if (status_dirty_ || config != status_config_) {
        cached_status_ = build_status(c);
        status_config_ = config;
        status_dirty_ = false;
    }
    return cached_status_;
}
CalibrationStatus Guidance::build_status(const RigConfig &c) const {
    CalibrationStatus s;
    for (const auto &v : training_)
        s.observation_labels << QString("Training #%1 | %2 s").arg(v.id).arg(v.start_us / 1e6, 0, 'f', 3);
    for (const auto &v : validation_)
        s.observation_labels << QString("Validation #%1 | %2 s").arg(v.id).arg(v.start_us / 1e6, 0, 'f', 3);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < int(supplemental_[i].size()); ++j)
            s.observation_labels << QString("Cam %1 intrinsic-only #%2").arg(i + 1).arg(j + 1);
    s.common_count = std::max(int(training_.size()), int(result_.training_ids.size()));
    s.validation_count = std::max(int(validation_.size()), int(result_.validation_views.size()));
    for (int i = 0; i < 3; ++i)
        s.intrinsic_counts[i] = s.common_count + int(supplemental_[i].size());
    s.training_complete = result_.trained;
    s.verdict =
        result_.validated ? Verdict::Passed : (!result_.error.empty() ? Verdict::Failed : Verdict::Unknown);
    s.board_locked = c.board_locked;
    s.geometry_key = c.geometry_key();
    s.target_camera = stable_camera_ >= 0 && stable_camera_ < 3 ? stable_camera_ : c.reference;
    s.reason = reason_;
    int target = target_;
    if (!training_.empty() && c.reference >= 0 && c.reference < 3) {
        std::array<int, 9> counts{};
        for (const auto &v : training_)
            ++counts[region(v.corners[c.reference], sizes_[c.reference])];
        target = int(std::min_element(counts.begin(), counts.end()) - counts.begin());
    }
    s.guide = QString("Target: move the complete board toward the %1 image region.")
                  .arg(QStringList{"upper-left", "upper-middle", "upper-right", "middle-left", "center",
                                   "middle-right", "lower-left", "lower-middle", "lower-right"}[target]);
    s.target_region = QRectF((target % 3) / 3., (target / 3) / 3., 1. / 3, 1. / 3);
    const auto count = training_.size() + validation_.size();
    if (count % 3 == 1) {
        s.guide = "Target: change the board's image size while keeping all corners visible.";
        s.target_region = {};
    }
    if (count % 3 == 2) {
        s.guide = "Target: gently tilt the display to add perspective while keeping all corners visible.";
        s.target_region = {};
    }
    for (int i = 0; i < 3; ++i) {
        for (const auto &view : training_)
            ++s.coverage[i][region(view.corners[i], sizes_[i])];
        if (last_.corners[i].size() != 54 && (stable_camera_ < 0 || stable_camera_ == i)) {
            s.guide = stable_camera_ < 0 ? "Priority: restore the complete board in all three views. Pose "
                                           "movement targets are paused."
                                         : QString("Priority: restore the complete board in Camera %1. Pose "
                                                   "movement targets are paused.")
                                               .arg(i + 1);
            s.target_region = {};
        }
    }
    s.stable_progress = progress_;
    if (result_.trained) {
        s.rms = 0;
        for (int i = 0; i < 3; ++i)
            s.rms = std::max(s.rms, result_.training_rms[i]);
        for (const auto &view : result_.training_views)
            for (double error : view)
                s.rms = std::max(s.rms, error);
        for (const auto &view : result_.validation_views)
            for (double error : view)
                s.rms = std::max(s.rms, error);
        if (result_.validated &&
            (!std::isfinite(c.max_rms_px) || c.max_rms_px <= 0 || s.rms > c.max_rms_px ||
             s.common_count < c.training_target || s.validation_count < c.validation_target)) {
            s.verdict = Verdict::Failed;
            s.reason = "Stored calibration does not meet the current project's RMS/count conditions.";
        }
        s.profile = calibration_json(result_, c);
        if (!result_.validation_projected.empty() &&
            result_.validation_projected.size() == result_.validation_observed.size())
            for (int i = 0; i < 3; ++i) {
                for (const auto &p : result_.validation_projected.back()[i])
                    s.validation_projected[i].push_back({p.x, p.y});
                for (const auto &p : result_.validation_observed.back()[i])
                    s.validation_observed[i].push_back({p.x, p.y});
            }
    }
    return s;
}

QJsonObject Guidance::observations_json() const {
    auto list = [](const auto &views) {
        QJsonArray a;
        for (const auto &v : views) {
            QJsonArray cameras;
            for (const auto &p : v.corners)
                cameras.append(points_json(p));
            a.append(QJsonObject{{"startUs", v.start_us}, {"corners", cameras}});
        }
        return a;
    };
    QJsonArray sizes, extra;
    for (int i = 0; i < 3; ++i) {
        sizes.append(QJsonArray{sizes_[i].width, sizes_[i].height});
        QJsonArray views;
        for (const auto &p : supplemental_[i])
            views.append(points_json(p));
        extra.append(views);
    }
    return {{"schemaVersion", 1},
            {"sizes", sizes},
            {"training", list(training_)},
            {"validation", list(validation_)},
            {"supplemental", extra}};
}
void Guidance::import_observations(const QJsonObject &object, const RigConfig &config) {
    if (object.value("geometryKey").toString() != config.geometry_key())
        throw std::runtime_error("Observation identity/board size does not match this rig.");
    const auto o = object.value("observations").toObject();
    if (o.value("schemaVersion").toInt() != 1 || !o.value("training").isArray() ||
        !o.value("supplemental").isArray())
        throw std::runtime_error("Invalid observation schema.");
    const auto sizes = o.value("sizes").toArray();
    if (sizes.size() != 3)
        throw std::runtime_error("Missing sensor sizes.");
    Guidance next;
    for (int i = 0; i < 3; ++i) {
        auto xy = sizes[i].toArray();
        if (xy.size() != 2 || !valid_size({xy[0].toInt(), xy[1].toInt()}))
            throw std::runtime_error("Invalid sensor size.");
        next.sizes_[i] = {xy[0].toInt(), xy[1].toInt()};
        if (valid_size(sizes_[i]) && sizes_[i] != next.sizes_[i])
            throw std::runtime_error("Observation resolution does not match the current sensor.");
    }
    const auto views = o.value("training").toArray();
    if (views.size() > 1000)
        throw std::runtime_error("Too many calibration observations.");
    for (const auto &v : views) {
        const auto row = v.toObject();
        const auto cameras = row.value("corners").toArray();
        if (cameras.size() != 3)
            throw std::runtime_error("Incomplete observation group.");
        gui_algo::rig::Observation view;
        view.start_us = row.value("startUs").toInteger(-1);
        if (view.start_us < 0 || view.start_us > std::numeric_limits<qint64>::max() - calibration_window_us)
            throw std::runtime_error("Invalid observation time window.");
        view.id = next.next_id_++;
        view.end_us = view.start_us + calibration_window_us;
        for (int i = 0; i < 3; ++i) {
            view.corners[i] = read_points(cameras[i].toArray());
            if (!valid_points(view.corners[i], next.sizes_[i]))
                throw std::runtime_error("Observation corners are outside the bound sensor resolution.");
        }
        for (const auto &previous : next.training_)
            if (view.start_us < previous.end_us && previous.start_us < view.end_us)
                throw std::runtime_error("Imported common observation windows overlap.");
        DetectedGroup group{view.start_us, next.sizes_, view.corners};
        if (next.duplicate(group, -1))
            throw std::runtime_error("Imported common observations repeat the same pose.");
        next.training_.push_back(std::move(view));
    }
    const auto extras = o.value("supplemental").toArray();
    if (extras.size() != 3)
        throw std::runtime_error("Invalid supplemental camera binding.");
    for (int i = 0; i < 3; ++i) {
        if (!extras[i].isArray())
            throw std::runtime_error("Invalid supplemental observations.");
        const auto a = extras[i].toArray();
        if (a.size() > 1000)
            throw std::runtime_error("Too many supplemental views.");
        for (const auto &p : a) {
            auto points = read_points(p.toArray());
            if (!valid_points(points, next.sizes_[i]))
                throw std::runtime_error("Supplemental corners are outside the bound sensor resolution.");
            DetectedGroup group;
            group.sizes = next.sizes_;
            group.corners[i] = points;
            if (next.duplicate(group, i))
                throw std::runtime_error("Imported intrinsic observations repeat the same pose.");
            next.supplemental_[i].push_back(std::move(points));
        }
    }
    // Imported observations are training only. A fresh hold-out is required after solving.
    *this = std::move(next);
}

QJsonObject calibration_json(const gui_algo::rig::Result &r, const RigConfig &c) {
    QJsonArray cameras, training, validation, overlays, fitting_corners;
    for (const auto &views : r.intrinsic_training_views) {
        QJsonArray a;
        for (const auto &p : views)
            a.append(points_json(p));
        fitting_corners.append(a);
    }
    for (int i = 0; i < 3; ++i)
        cameras.append(
            QJsonObject{{"serial", c.cameras[i].serial},
                        {"width", r.models[i].size.width},
                        {"height", r.models[i].size.height},
                        {"K", matrix_json(r.models[i].K)},
                        {"D", matrix_json(r.models[i].distortion.reshape(1, 1))},
                        {"T_camera_from_reference", matrix_json(cv::Mat(r.T_camera_from_reference[i]))},
                        {"trainingRmsPx", r.training_rms[i]},
                        {"validationRmsPx", r.validation_rms[i]}});
    for (const auto &v : r.validation_views)
        validation.append(QJsonArray{v[0], v[1], v[2]});
    for (const auto &v : r.training_views)
        training.append(QJsonArray{v[0], v[1], v[2]});
    for (size_t v = 0; v < std::min(r.validation_projected.size(), r.validation_observed.size()); ++v) {
        QJsonArray group;
        for (int i = 0; i < 3; ++i)
            group.append(QJsonObject{{"observed", points_json(r.validation_observed[v][i])},
                                     {"projected", points_json(r.validation_projected[v][i])}});
        overlays.append(group);
    }
    return {{"schemaVersion", 1},
            {"model", "raw-distorted-pinhole-k1-k2-p1-p2-k3"},
            {"trainingCommonCount", int(r.training_ids.size())},
            {"opticsAndRigConfirmed", c.optics_confirmed},
            {"validationErrorOverlays", overlays},
            {"lengthUnit", "metre"},
            {"geometryKey", c.geometry_key()},
            {"reference", r.reference},
            {"squareM", r.square_m},
            {"trained", r.trained},
            {"validated", r.validated},
            {"cameras", cameras},
            {"trainingViews", training},
            {"intrinsicTrainingViews", fitting_corners},
            {"validationViews", validation},
            {"maxRmsPx", c.max_rms_px},
            {"boardMeasurement", c.to_json().value("boardMeasurement")},
            {"boardDisplay", c.board_display}};
}
gui_algo::rig::Result read_calibration(const QJsonObject &o, const RigConfig &c,
                                       const std::array<cv::Size, 3> &sizes) {
    if (c.reference < 0 || c.reference >= 3 || !std::isfinite(c.square_mm) || c.square_mm <= 0 ||
        !std::isfinite(c.max_rms_px) || c.max_rms_px <= 0 || c.training_target < 3 || c.validation_target < 3)
        throw std::runtime_error("Project calibration conditions are invalid.");
    if (o.value("schemaVersion").toInt() != 1 || o.value("geometryKey").toString() != c.geometry_key() ||
        o.value("model").toString() != "raw-distorted-pinhole-k1-k2-p1-p2-k3" ||
        o.value("lengthUnit").toString() != "metre" || !o.value("trained").toBool() ||
        !o.value("validated").toBool())
        throw std::runtime_error("Profile is not a validated, matching rig calibration.");
    gui_algo::rig::Result r;
    r.reference = c.reference;
    r.square_m = c.square_mm / 1000;
    const int training_count = o.value("trainingCommonCount").toInt(-1);
    const double square_m = o.value("squareM").toDouble(NAN);
    if (o.value("reference").toInt(-1) != c.reference || !std::isfinite(square_m) ||
        std::abs(square_m - r.square_m) > 1e-9 || training_count < c.training_target || training_count > 1000)
        throw std::runtime_error(
            "Profile reference, board size or training count does not match the project.");
    for (int i = 0; i < training_count; ++i)
        r.training_ids.push_back(-i - 1);
    const auto cameras = o.value("cameras").toArray();
    if (cameras.size() != 3)
        throw std::runtime_error("Profile needs three cameras.");
    for (int i = 0; i < 3; ++i) {
        const auto a = cameras[i].toObject();
        if (!valid_size(sizes[i]) || c.cameras[i].serial.isEmpty() ||
            a.value("serial").toString() != c.cameras[i].serial ||
            cv::Size(a.value("width").toInt(), a.value("height").toInt()) != sizes[i])
            throw std::runtime_error("Profile camera identity or resolution mismatch.");
        r.models[i] = {sizes[i], read_matrix(a.value("K").toArray(), 3, 3),
                       read_matrix(a.value("D").toArray(), 1, 5)};
        if (r.models[i].K.at<double>(0, 0) <= 0 || r.models[i].K.at<double>(1, 1) <= 0 ||
            std::abs(r.models[i].K.at<double>(2, 2) - 1) > 1e-9 ||
            std::abs(r.models[i].K.at<double>(0, 1)) > 1e-9 ||
            std::abs(r.models[i].K.at<double>(1, 0)) > 1e-9 ||
            std::abs(r.models[i].K.at<double>(2, 0)) > 1e-9 ||
            std::abs(r.models[i].K.at<double>(2, 1)) > 1e-9)
            throw std::runtime_error("Invalid pinhole camera matrix.");
        cv::Mat t = read_matrix(a.value("T_camera_from_reference").toArray(), 4, 4);
        std::copy(t.ptr<double>(), t.ptr<double>() + 16, r.T_camera_from_reference[i].val);
        r.training_rms[i] = a.value("trainingRmsPx").toDouble(NAN);
        r.validation_rms[i] = a.value("validationRmsPx").toDouble(NAN);
        if (!std::isfinite(r.training_rms[i]) || !std::isfinite(r.validation_rms[i]) ||
            r.training_rms[i] < 0 || r.validation_rms[i] < 0 || r.training_rms[i] > c.max_rms_px ||
            r.validation_rms[i] > c.max_rms_px)
            throw std::runtime_error("Profile RMS does not meet this project's condition.");
        const cv::Mat rotation = t(cv::Rect(0, 0, 3, 3));
        if (cv::norm(t.row(3) - (cv::Mat_<double>(1, 4) << 0, 0, 0, 1)) > 1e-8 ||
            (i == c.reference && cv::norm(t - cv::Mat::eye(4, 4, CV_64F)) > 1e-8))
            throw std::runtime_error("Invalid reference gauge or homogeneous transform.");
        if (cv::norm(rotation.t() * rotation - cv::Mat::eye(3, 3, CV_64F)) > 1e-5 ||
            std::abs(cv::determinant(rotation) - 1) > 1e-5)
            throw std::runtime_error("Invalid rigid transform.");
    }
    const auto views = o.value("validationViews").toArray();
    if (views.size() < c.validation_target || views.size() > 1000)
        throw std::runtime_error("Profile has too few independent validation poses.");
    for (const auto &v : views) {
        auto a = v.toArray();
        if (a.size() != 3)
            throw std::runtime_error("Invalid validation report.");
        std::array<double, 3> row{};
        for (int i = 0; i < 3; ++i) {
            row[i] = a[i].toDouble(NAN);
            if (!std::isfinite(row[i]) || row[i] < 0 || row[i] > c.max_rms_px)
                throw std::runtime_error("A held-out pose exceeds this project's RMS condition.");
        }
        r.validation_views.push_back(row);
    }
    if (!o.value("trainingViews").isArray())
        throw std::runtime_error("Invalid training report.");
    const auto training = o.value("trainingViews").toArray();
    if (training.size() != training_count)
        throw std::runtime_error("Training report count does not match the profile.");
    for (const auto &v : training) {
        const auto a = v.toArray();
        if (a.size() != 3)
            throw std::runtime_error("Invalid training report.");
        std::array<double, 3> row;
        for (int i = 0; i < 3; ++i) {
            row[i] = a[i].toDouble(NAN);
            if (!std::isfinite(row[i]) || row[i] < 0 || row[i] > c.max_rms_px)
                throw std::runtime_error("A training pose exceeds this project's RMS condition.");
        }
        r.training_views.push_back(row);
    }
    for (int i = 0; i < 3; ++i) {
        auto consistent = [i](const auto &rows, double aggregate) {
            double sum = 0;
            for (const auto &row : rows)
                sum += row[i] * row[i];
            return rows.empty() || std::abs(std::sqrt(sum / rows.size()) - aggregate) < 1e-6;
        };
        if (!consistent(r.validation_views, r.validation_rms[i]) ||
            !consistent(r.training_views, r.training_rms[i]))
            throw std::runtime_error("Per-view and aggregate RMS reports disagree.");
    }
    if (o.contains("intrinsicTrainingViews")) {
        const auto cameras = o.value("intrinsicTrainingViews").toArray();
        if (cameras.size() != 3)
            throw std::runtime_error("Invalid intrinsic training observation binding.");
        for (int i = 0; i < 3; ++i) {
            if (!cameras[i].isArray() || cameras[i].toArray().size() > 2000)
                throw std::runtime_error("Invalid intrinsic training observations.");
            for (const auto &v : cameras[i].toArray()) {
                auto points = read_points(v.toArray());
                if (!valid_points(points, sizes[i]))
                    throw std::runtime_error("Training corners do not match the profile sensor resolution.");
                r.intrinsic_training_views[i].push_back(std::move(points));
            }
        }
    }
    if (o.contains("observations")) {
        Guidance observations;
        observations.import_observations(o, c);
        if (observations.sizes() != sizes ||
            (!observations.training().empty() && int(observations.training().size()) != training_count))
            throw std::runtime_error(
                "Profile and stored observations have inconsistent sensor/count bindings.");
        const auto inputs = observations.intrinsic_views();
        for (int i = 0; i < 3; ++i)
            for (const auto &p : inputs[i])
                if (std::find(r.intrinsic_training_views[i].begin(), r.intrinsic_training_views[i].end(),
                              p) == r.intrinsic_training_views[i].end())
                    r.intrinsic_training_views[i].push_back(p);
    }
    r.trained = true;
    r.validated = true;
    return r;
}
} // namespace gui::multicam
