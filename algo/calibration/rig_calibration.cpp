#include "rig_calibration.h"
#include "intrinsic.h"
#include "opencv_compat.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <cmath>
#include <limits>
#include <set>

namespace gui_algo::rig {
namespace {
using Transform = cv::Matx44d;
using Corners = std::vector<cv::Point2f>;

Transform pose(const cv::Mat &rv, const cv::Mat &tv) {
    cv::Mat rotation;
    cv::Rodrigues(rv, rotation);
    Transform t = Transform::eye();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c)
            t(r, c) = rotation.at<double>(r, c);
        t(r, 3) = tv.at<double>(r);
    }
    return t;
}
void pose_vectors(const Transform &t, cv::Mat &rv, cv::Mat &tv) {
    cv::Mat rotation(3, 3, CV_64F);
    tv = cv::Mat(3, 1, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c)
            rotation.at<double>(r, c) = t(r, c);
        tv.at<double>(r) = t(r, 3);
    }
    cv::Rodrigues(rotation, rv);
}
bool valid_model(const CameraModel &m) {
    return m.size.width > 0 && m.size.height > 0 && m.K.type() == CV_64F && m.K.rows == 3 && m.K.cols == 3 &&
           m.distortion.type() == CV_64F && m.distortion.total() == 5 &&
           (m.distortion.rows == 1 || m.distortion.cols == 1) && cv::checkRange(m.K) &&
           cv::checkRange(m.distortion) && m.K.at<double>(0, 0) > 0 && m.K.at<double>(1, 1) > 0 &&
           std::abs(m.K.at<double>(0, 1)) < 1e-9 && std::abs(m.K.at<double>(1, 0)) < 1e-9 &&
           std::abs(m.K.at<double>(2, 0)) < 1e-9 && std::abs(m.K.at<double>(2, 1)) < 1e-9 &&
           std::abs(m.K.at<double>(2, 2) - 1) < 1e-9;
}
bool valid_points(const Corners &points, const CameraModel &m) {
    if (points.size() != corner_count)
        return false;
    for (const auto &p : points)
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 || p.x >= m.size.width ||
            p.y >= m.size.height)
            return false;
    return true;
}
std::vector<cv::Point2f> project(const Transform &t, const CameraModel &m,
                                 const std::vector<cv::Point3f> &grid) {
    cv::Mat r, v;
    pose_vectors(t, r, v);
    std::vector<cv::Point2f> p;
    cv::projectPoints(grid, r, v, m.K, m.distortion, p);
    return p;
}
double rms(const Corners &a, const Corners &b) {
    if (a.size() != b.size() || a.empty())
        return std::numeric_limits<double>::infinity();
    double sum = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const auto d = a[i] - b[i];
        sum += d.dot(d);
    }
    return std::sqrt(sum / static_cast<double>(a.size()));
}
bool same_corners(const Corners &a, const Corners &b, cv::Size size) {
    if (a.size() != corner_count || b.size() != corner_count)
        return false;
    auto reversed = b;
    std::reverse(reversed.begin(), reversed.end());
    const double tolerance = 2 * std::hypot(size.width, size.height) / std::hypot(1280., 720.);
    return std::min(rms(a, b), rms(a, reversed)) <= tolerance;
}
bool valid_window(const Observation &view) {
    return view.start_us >= 0 && view.end_us > view.start_us && view.end_us - view.start_us == 100000;
}
bool overlaps(const Observation &a, const Observation &b) {
    return a.start_us < b.end_us && b.start_us < a.end_us;
}
bool valid_transform(const Transform &t) {
    if (!cv::checkRange(cv::Mat(t)))
        return false;
    const cv::Mat m(t), rotation = m(cv::Rect(0, 0, 3, 3));
    return cv::norm(m.row(3) - (cv::Mat_<double>(1, 4) << 0, 0, 0, 1)) < 1e-8 &&
           cv::norm(rotation.t() * rotation - cv::Mat::eye(3, 3, CV_64F)) < 1e-5 &&
           std::abs(cv::determinant(rotation) - 1) < 1e-5;
}
bool intrinsic_observable(const std::vector<Corners> &views, cv::Size size) {
    if (views.size() < 3 || size.width <= 0 || size.height <= 0)
        return false;
    cv::Mat constraints(int(views.size()) * 2, 6, CV_64F);
    std::vector<cv::Point2f> grid;
    for (int y = 0; y < 6; ++y)
        for (int x = 0; x < 9; ++x)
            grid.emplace_back(x / 8.f, y / 5.f);
    for (size_t k = 0; k < views.size(); ++k) {
        if (views[k].size() != 54)
            return false;
        auto normalized = views[k];
        for (auto &p : normalized) {
            p.x = (p.x - size.width * .5f) / size.width;
            p.y = (p.y - size.height * .5f) / size.width;
        }
        const cv::Mat h = cv::findHomography(grid, normalized);
        if (h.empty())
            return false;
        auto constraint = [&](int i, int j) {
            cv::Mat row = (cv::Mat_<double>(1, 6) << h.at<double>(0, i) * h.at<double>(0, j),
                           h.at<double>(0, i) * h.at<double>(1, j) + h.at<double>(1, i) * h.at<double>(0, j),
                           h.at<double>(1, i) * h.at<double>(1, j),
                           h.at<double>(2, i) * h.at<double>(0, j) + h.at<double>(0, i) * h.at<double>(2, j),
                           h.at<double>(2, i) * h.at<double>(1, j) + h.at<double>(1, i) * h.at<double>(2, j),
                           h.at<double>(2, i) * h.at<double>(2, j));
            return row;
        };
        cv::Mat a = constraint(0, 1), b = constraint(0, 0) - constraint(1, 1);
        a /= std::max(1e-15, cv::norm(a));
        b /= std::max(1e-15, cv::norm(b));
        a.copyTo(constraints.row(int(k) * 2));
        b.copyTo(constraints.row(int(k) * 2 + 1));
    }
    cv::Mat singular;
    cv::SVD::compute(constraints, singular);
    return singular.rows >= 5 && singular.at<double>(4) > singular.at<double>(0) * 1e-5;
}
double rotation_distance(const Transform &a, const Transform &b) {
    const Transform d = a * b.inv();
    return std::acos(std::clamp((d(0, 0) + d(1, 1) + d(2, 2) - 1) * .5, -1.0, 1.0));
}
double translation_distance(const Transform &a, const Transform &b) {
    double sum = 0;
    for (int i = 0; i < 3; ++i)
        sum += std::pow(a(i, 3) - b(i, 3), 2);
    return std::sqrt(sum);
}
bool positive_depth(const Transform &t, const std::vector<cv::Point3f> &grid) {
    for (const auto &p : grid)
        if (const double z = t(2, 0) * p.x + t(2, 1) * p.y + t(2, 3); !std::isfinite(z) || z <= 1e-6)
            return false;
    return true;
}
struct PnpPose {
    Transform t;
    double error;
};
std::vector<PnpPose> pnp(const Corners &points, const CameraModel &model,
                         const std::vector<cv::Point3f> &grid) {
    std::vector<cv::Mat> rotations, translations;
    std::vector<PnpPose> result;
    cv::solvePnPGeneric(grid, points, model.K, model.distortion, rotations, translations, false,
                        cv::SOLVEPNP_IPPE);
    for (std::size_t i = 0; i < rotations.size(); ++i) {
        auto t = pose(rotations[i], translations[i]);
        if (positive_depth(t, grid))
            result.push_back({t, rms(points, project(t, model, grid))});
    }
    std::sort(result.begin(), result.end(), [](const auto &a, const auto &b) { return a.error < b.error; });
    return result;
}
bool ambiguous(const std::vector<PnpPose> &candidates) {
    return candidates.size() > 1 && candidates[1].error - candidates[0].error < .05 &&
           rotation_distance(candidates[0].t, candidates[1].t) > .05;
}

struct Parameters {
    std::array<double, 4> q{{1, 0, 0, 0}};
    std::array<double, 3> t{};
    Parameters() = default;
    explicit Parameters(const Transform &m) {
        Eigen::Matrix3d r;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j)
                r(i, j) = m(i, j);
            t[i] = m(i, 3);
        }
        const Eigen::Quaterniond rotation(r);
        q = {{rotation.w(), rotation.x(), rotation.y(), rotation.z()}};
    }
    Transform matrix() const {
        const Eigen::Quaterniond rotation(q[0], q[1], q[2], q[3]);
        const auto r = rotation.normalized().toRotationMatrix();
        Transform m = Transform::eye();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j)
                m(i, j) = r(i, j);
            m(i, 3) = t[i];
        }
        return m;
    }
};

struct Reprojection {
    cv::Point3f point;
    cv::Point2f measured;
    double fx, fy, cx, cy;
    std::array<double, 5> distortion;
    Reprojection(cv::Point3f p, cv::Point2f image, const CameraModel &m)
        : point(p), measured(image), fx(m.K.at<double>(0, 0)), fy(m.K.at<double>(1, 1)),
          cx(m.K.at<double>(0, 2)), cy(m.K.at<double>(1, 2)) {
        for (int i = 0; i < 5; ++i)
            distortion[i] = m.distortion.at<double>(i);
    }
    template <class T>
    bool operator()(const T *const qb, const T *const tb, const T *const qc, const T *const tc,
                    T *residual) const {
        const T input[3] = {T(point.x), T(point.y), T(point.z)};
        T ref[3], cam[3];
        ceres::QuaternionRotatePoint(qb, input, ref);
        for (int i = 0; i < 3; ++i)
            ref[i] += tb[i];
        ceres::QuaternionRotatePoint(qc, ref, cam);
        for (int i = 0; i < 3; ++i)
            cam[i] += tc[i];
        const T x = cam[0] / cam[2], y = cam[1] / cam[2], r2 = x * x + y * y;
        const T radial =
            T(1) + T(distortion[0]) * r2 + T(distortion[1]) * r2 * r2 + T(distortion[4]) * r2 * r2 * r2;
        const T xd = x * radial + T(2 * distortion[2]) * x * y + T(distortion[3]) * (r2 + T(2) * x * x);
        const T yd = y * radial + T(distortion[2]) * (r2 + T(2) * y * y) + T(2 * distortion[3]) * x * y;
        residual[0] = T(fx) * xd + T(cx) - T(measured.x);
        residual[1] = T(fy) * yd + T(cy) - T(measured.y);
        return true;
    }
};

struct Hypothesis {
    Transform transform;
    bool reverse = false;
};
double distance(const Transform &a, const Transform &b) {
    return rotation_distance(a, b) + translation_distance(a, b) * 4;
}

bool initialize(const std::array<CameraModel, camera_count> &models, std::vector<Observation> &views,
                int reference, const std::vector<cv::Point3f> &grid, std::vector<Transform> &boards,
                std::array<Transform, camera_count> &rig, std::string &error) {
    for (const auto &view : views) {
        const auto candidates = pnp(view.corners[reference], models[reference], grid);
        if (candidates.empty() || ambiguous(candidates)) {
            error = "Reference board pose is ambiguous; add tilted, well-resolved views.";
            return false;
        }
        boards.push_back(candidates.front().t);
    }
    for (int cam = 0; cam < camera_count; ++cam) {
        if (cam == reference)
            continue;
        std::vector<std::vector<Hypothesis>> possibilities(views.size());
        for (std::size_t k = 0; k < views.size(); ++k) {
            for (bool reverse : {false, true}) {
                auto points = views[k].corners[cam];
                if (reverse)
                    std::reverse(points.begin(), points.end());
                for (const auto &candidate : pnp(points, models[cam], grid))
                    possibilities[k].push_back({candidate.t * boards[k].inv(), reverse});
            }
            if (possibilities[k].empty()) {
                error = "Could not initialize a positive-depth board pose.";
                return false;
            }
        }
        double best = std::numeric_limits<double>::infinity();
        Transform seed = Transform::eye();
        std::vector<std::pair<double, Transform>> scores;
        for (const auto &group : possibilities)
            for (const auto &candidate : group) {
                std::vector<double> distances;
                for (const auto &other : possibilities) {
                    double d = 1e10;
                    for (const auto &h : other)
                        d = std::min(d, distance(candidate.transform, h.transform));
                    distances.push_back(d);
                }
                std::sort(distances.begin(), distances.end());
                const std::size_t keep = std::max<std::size_t>(3, distances.size() * 3 / 4);
                double score = 0;
                for (std::size_t j = 0; j < std::min(keep, distances.size()); ++j)
                    score += distances[j];
                score /= static_cast<double>(std::min(keep, distances.size()));
                scores.push_back({score, candidate.transform});
                if (score < best) {
                    best = score;
                    seed = candidate.transform;
                }
            }
        if (best > .15) {
            error = "Inconsistent corner identities or rigid geometry; collect more diverse views.";
            return false;
        }
        for (const auto &scored : scores)
            if (scored.first <= best + .001 && distance(seed, scored.second) > .2) {
                error = "Multiple rig geometries fit the observations; add tilted views.";
                return false;
            }
        rig[cam] = seed;
        for (std::size_t k = 0; k < views.size(); ++k) {
            const auto &group = possibilities[k];
            const auto selected =
                std::min_element(group.begin(), group.end(), [&](const auto &a, const auto &b) {
                    return distance(seed, a.transform) < distance(seed, b.transform);
                });
            if (distance(seed, selected->transform) > .3) {
                error = "A common observation has inconsistent corner ordering or pose.";
                return false;
            }
            if (selected->reverse)
                std::reverse(views[k].corners[cam].begin(), views[k].corners[cam].end());
        }
    }
    return true;
}
} // namespace

Result::Result() {
    for (auto &t : T_camera_from_reference)
        t = Transform::eye();
}
std::vector<cv::Point3f> object_points(double square_m) {
    std::vector<cv::Point3f> grid;
    for (int y = 0; y < 6; ++y)
        for (int x = 0; x < 9; ++x)
            grid.emplace_back(static_cast<float>(x * square_m), static_cast<float>(y * square_m), 0);
    return grid;
}

Result solve_joint(const std::array<CameraModel, camera_count> &models,
                   const std::vector<Observation> &observations, int reference, double square_m) {
    Result result;
    result.reference = reference;
    result.square_m = square_m;
    if (reference < 0 || reference >= camera_count || !std::isfinite(square_m) || square_m <= 0 ||
        observations.size() < 3) {
        result.error = "Need a reference camera, measured board and at least three common views.";
        return result;
    }
    for (int i = 0; i < camera_count; ++i) {
        if (!valid_model(models[i])) {
            result.error = "Invalid camera intrinsics.";
            return result;
        }
        result.models[i] = {models[i].size, models[i].K.clone(), models[i].distortion.clone()};
    }
    for (const auto &view : observations) {
        if (!valid_window(view)) {
            result.error = "Invalid common observation time window.";
            return result;
        }
        if (std::find(result.training_ids.begin(), result.training_ids.end(), view.id) !=
            result.training_ids.end()) {
            result.error = "Duplicate training observation identity.";
            return result;
        }
        result.training_ids.push_back(view.id);
        for (int i = 0; i < camera_count; ++i)
            if (!valid_points(view.corners[i], models[i])) {
                result.error = "A common view has missing or invalid corners.";
                return result;
            }
        for (const auto &previous : result.training_observations) {
            bool repeated = true;
            for (int i = 0; i < camera_count; ++i)
                repeated = repeated && same_corners(previous.corners[i], view.corners[i], models[i].size);
            if (overlaps(previous, view) || repeated) {
                result.error = "Training observations must have distinct poses and disjoint time windows.";
                return result;
            }
        }
        result.training_observations.push_back(view);
        for (int i = 0; i < camera_count; ++i)
            result.intrinsic_training_views[i].push_back(view.corners[i]);
    }
    try {
        const auto grid = object_points(square_m);
        auto views = observations;
        std::vector<Transform> board_poses;
        if (!initialize(models, views, reference, grid, board_poses, result.T_camera_from_reference,
                        result.error))
            return result;
        std::vector<Parameters> board_parameters;
        for (const auto &p : board_poses)
            board_parameters.emplace_back(p);
        std::array<Parameters, camera_count> cameras;
        for (int i = 0; i < camera_count; ++i)
            cameras[i] = Parameters(result.T_camera_from_reference[i]);
        ceres::Problem problem;
        for (auto &p : board_parameters) {
            problem.AddParameterBlock(p.q.data(), 4, new ceres::QuaternionManifold);
            problem.AddParameterBlock(p.t.data(), 3);
        }
        result.training_views.resize(views.size());
        for (int i = 0; i < camera_count; ++i) {
            problem.AddParameterBlock(cameras[i].q.data(), 4, new ceres::QuaternionManifold);
            problem.AddParameterBlock(cameras[i].t.data(), 3);
            if (i == reference) {
                problem.SetParameterBlockConstant(cameras[i].q.data());
                problem.SetParameterBlockConstant(cameras[i].t.data());
            }
        }
        for (std::size_t k = 0; k < views.size(); ++k)
            for (int i = 0; i < camera_count; ++i)
                for (int j = 0; j < corner_count; ++j) {
                    auto *cost = new ceres::AutoDiffCostFunction<Reprojection, 2, 4, 3, 4, 3>(
                        new Reprojection(grid[j], views[k].corners[i][j], models[i]));
                    problem.AddResidualBlock(cost, new ceres::HuberLoss(1.0), board_parameters[k].q.data(),
                                             board_parameters[k].t.data(), cameras[i].q.data(),
                                             cameras[i].t.data());
                }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.max_num_iterations = 100;
        options.function_tolerance = 1e-12;
        options.gradient_tolerance = 1e-12;
        options.parameter_tolerance = 1e-12;
        options.num_threads = 2;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        if (!summary.IsSolutionUsable()) {
            result.error = "Joint optimization did not produce a usable solution.";
            return result;
        }
        for (int i = 0; i < camera_count; ++i) {
            result.T_camera_from_reference[i] = cameras[i].matrix();
            double sum = 0;
            for (std::size_t k = 0; k < views.size(); ++k) {
                const auto t = result.T_camera_from_reference[i] * board_parameters[k].matrix();
                if (!positive_depth(t, grid)) {
                    result.error = "Optimized board is behind a camera.";
                    return result;
                }
                const double e = rms(views[k].corners[i], project(t, models[i], grid));
                result.training_views[k][i] = e;
                sum += e * e;
            }
            result.training_rms[i] = std::sqrt(sum / static_cast<double>(views.size()));
            if (!std::isfinite(result.training_rms[i])) {
                result.error = "Non-finite reprojection error.";
                return result;
            }
        }
        result.trained = true;
    } catch (const std::exception &e) {
        result.error = e.what();
    }
    return result;
}

Result solve_intrinsics_and_joint(
    const std::array<cv::Size, camera_count> &sizes,
    const std::array<std::vector<std::vector<cv::Point2f>>, camera_count> &intrinsic_views,
    const std::vector<Observation> &common_views, int reference, double square_m) {
    if (reference < 0 || reference >= camera_count || !std::isfinite(square_m) || square_m <= 0) {
        Result r;
        r.error = "Invalid reference camera or measured board size.";
        return r;
    }
    std::array<CameraModel, camera_count> models;
    for (int i = 0; i < camera_count; ++i) {
        IntrinsicCalibration engine;
        engine.set_pattern(CalibrationPattern::Chessboard, 9, 6, static_cast<float>(square_m));
        for (const auto &corners : intrinsic_views[i])
            if (!engine.accept_observation(corners, sizes[i])) {
                Result r;
                r.error = "Invalid stored intrinsic observation.";
                return r;
            }
        if (!intrinsic_observable(intrinsic_views[i], sizes[i])) {
            Result r;
            r.error = "Camera " + std::to_string(i + 1) +
                      ": insufficient independent board tilts; collect diverse perspective views.";
            return r;
        }
        auto intrinsic = engine.run();
        if (!intrinsic.ok) {
            Result r;
            r.error = "Camera " + std::to_string(i) + ": " + intrinsic.error;
            return r;
        }
        std::vector<Corners> kept;
        for (size_t k = 0; k < intrinsic.selected_views.size(); ++k)
            if (intrinsic.selected_views[k])
                kept.push_back(intrinsic_views[i][k]);
        if (!intrinsic_observable(kept, sizes[i])) {
            Result r;
            r.error = "Remaining intrinsic inliers lack independent board tilts.";
            return r;
        }
        models[i] = {sizes[i], intrinsic.K, intrinsic.dist_coeffs};
    }
    auto result = solve_joint(models, common_views, reference, square_m);
    result.intrinsic_training_views = intrinsic_views;
    return result;
}

Result validate(const Result &trained, const std::vector<Observation> &held_out, double max_rms_px) {
    Result result = trained;
    result.validated = false;
    result.validation_views.clear();
    result.validation_ids.clear();
    result.validation_projected.clear();
    result.validation_observed.clear();
    result.error.clear();
    result.validation_rms.fill(0);
    if (!trained.trained || trained.reference < 0 || trained.reference >= camera_count ||
        !std::isfinite(trained.square_m) || trained.square_m <= 0 || held_out.empty() ||
        trained.training_ids.empty() || trained.training_views.size() != trained.training_ids.size() ||
        !std::isfinite(max_rms_px) || max_rms_px <= 0) {
        result.error = "Need trained geometry, fresh validation views and a positive RMS limit.";
        return result;
    }
    for (int i = 0; i < camera_count; ++i)
        if (!valid_model(trained.models[i]) || !valid_transform(trained.T_camera_from_reference[i]) ||
            !std::isfinite(trained.training_rms[i]) || trained.training_rms[i] < 0) {
            result.error = "Invalid trained camera geometry or reprojection report.";
            return result;
        }
    if (cv::norm(cv::Mat(trained.T_camera_from_reference[trained.reference]) - cv::Mat::eye(4, 4, CV_64F)) >
        1e-8) {
        result.error = "Invalid reference gauge.";
        return result;
    }
    for (int i = 0; i < camera_count; ++i)
        if (trained.training_observations.empty() && trained.intrinsic_training_views[i].empty()) {
            result.error = "Fresh validation requires the original training corners to exclude reuse.";
            return result;
        }
    try {
        const auto grid = object_points(trained.square_m);
        bool passed = true;
        std::set<std::int64_t> seen(trained.training_ids.begin(), trained.training_ids.end());
        auto prior = trained.training_observations;
        for (const auto &view : held_out) {
            if (!seen.insert(view.id).second) {
                result.error = "Held-out observations must be independent, unique and absent from training.";
                return result;
            }
            result.validation_ids.push_back(view.id);
            if (!valid_window(view)) {
                result.error = "Invalid held-out common observation time window.";
                return result;
            }
            for (const auto &previous : prior)
                if (overlaps(view, previous)) {
                    result.error = "Held-out time windows must not overlap training or one another.";
                    return result;
                }
            for (int i = 0; i < camera_count; ++i)
                if (!valid_points(view.corners[i], trained.models[i])) {
                    result.error = "Incomplete held-out observation.";
                    return result;
                }
            for (int i = 0; i < camera_count; ++i) {
                bool reused = false;
                for (const auto &previous : prior)
                    reused =
                        reused || same_corners(view.corners[i], previous.corners[i], trained.models[i].size);
                for (const auto &points : trained.intrinsic_training_views[i])
                    reused = reused || same_corners(view.corners[i], points, trained.models[i].size);
                if (reused) {
                    result.error = "Held-out corners reuse an intrinsic, joint or validation observation.";
                    return result;
                }
            }
            prior.push_back(view);
            const auto reference =
                pnp(view.corners[trained.reference], trained.models[trained.reference], grid);
            if (reference.empty() || ambiguous(reference)) {
                result.error = "Ambiguous held-out reference pose; collect a clearer tilted view.";
                return result;
            }
            std::array<double, camera_count> errors;
            std::array<std::vector<cv::Point2f>, camera_count> projected_points, observed_points;
            for (int i = 0; i < camera_count; ++i) {
                const auto t = trained.T_camera_from_reference[i] * reference.front().t;
                if (!positive_depth(t, grid)) {
                    result.error = "Validation projection is behind a camera.";
                    return result;
                }
                const auto projected = project(t, trained.models[i], grid);
                auto reversed = view.corners[i];
                std::reverse(reversed.begin(), reversed.end());
                const bool reverse = rms(reversed, projected) < rms(view.corners[i], projected);
                observed_points[i] = reverse ? reversed : view.corners[i];
                projected_points[i] = projected;
                errors[i] = rms(observed_points[i], projected);
                result.validation_rms[i] += errors[i] * errors[i];
                passed = passed && std::isfinite(errors[i]) && errors[i] <= max_rms_px;
            }
            result.validation_views.push_back(errors);
            result.validation_projected.push_back(std::move(projected_points));
            result.validation_observed.push_back(std::move(observed_points));
        }
        for (int i = 0; i < camera_count; ++i) {
            result.validation_rms[i] =
                std::sqrt(result.validation_rms[i] / static_cast<double>(held_out.size()));
            passed = passed && trained.training_rms[i] <= max_rms_px;
        }
        for (const auto &view : trained.training_views)
            for (double error : view)
                passed = passed && std::isfinite(error) && error >= 0 && error <= max_rms_px;
        result.validated = passed;
        if (!passed)
            result.error = "Training or independent transfer RMS exceeds the project limit.";
    } catch (const std::exception &e) {
        result.error = e.what();
    }
    return result;
}
} // namespace gui_algo::rig
