#include "algo/calibration/opencv_compat.h"
#include "algo/calibration/rig_calibration.h"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>

namespace {
using namespace gui_algo::rig;
cv::Matx44d transform(cv::Vec3d rotation, cv::Vec3d translation) {
    cv::Mat r;
    cv::Rodrigues(rotation, r);
    auto t = cv::Matx44d::eye();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j)
            t(i, j) = r.at<double>(i, j);
        t(i, 3) = translation[i];
    }
    return t;
}
struct Fixture {
    std::array<CameraModel, 3> models;
    std::array<cv::Matx44d, 3> truth;
    Fixture() {
        for (auto &m : models) {
            m.size = {1280, 720};
            m.K = (cv::Mat_<double>(3, 3) << 760, 0, 640, 0, 760, 360, 0, 0, 1);
            m.distortion = (cv::Mat_<double>(1, 5) << -.06, .01, .001, -.001, .001);
        }
        truth[0] = transform({.01, -.035, .005}, {-.11, .003, .005});
        truth[1] = cv::Matx44d::eye();
        truth[2] = transform({-.01, .04, -.003}, {.12, -.002, .007});
    }
    std::vector<Observation> observations(int start, int count, bool flip = false) const {
        std::vector<Observation> views;
        for (int k = start; k < start + count; ++k) {
            Observation o;
            o.id = k;
            o.start_us = k * 2000000LL;
            o.end_us = o.start_us + 100000;
            const auto board = transform(
                {.18 + std::sin(k * .9) * .28, -.22 + std::cos(k * .7) * .27, std::sin(k * .5) * .18},
                {-.10 + std::sin(k * .6) * .08, -.07 + std::cos(k * .8) * .04, .65 + (k % 5) * .08});
            for (int i = 0; i < 3; ++i) {
                const auto t = truth[i] * board;
                cv::Mat r(3, 3, CV_64F), rv, tv(3, 1, CV_64F);
                for (int y = 0; y < 3; ++y) {
                    for (int x = 0; x < 3; ++x)
                        r.at<double>(y, x) = t(y, x);
                    tv.at<double>(y) = t(y, 3);
                }
                cv::Rodrigues(r, rv);
                cv::projectPoints(object_points(.025), rv, tv, models[i].K, models[i].distortion,
                                  o.corners[i]);
                if (flip && i == 2 && k % 2)
                    std::reverse(o.corners[i].begin(), o.corners[i].end());
            }
            views.push_back(o);
        }
        return views;
    }
};
TEST(RigCalibration, RecoversKnownThreeCameraGeometry) {
    Fixture f;
    const auto result = solve_joint(f.models, f.observations(0, 12), 1, .025);
    ASSERT_TRUE(result.trained) << result.error;
    for (int i = 0; i < 3; ++i) {
        const auto d = result.T_camera_from_reference[i] * f.truth[i].inv();
        const double angle = std::acos(std::clamp((d(0, 0) + d(1, 1) + d(2, 2) - 1) * .5, -1., 1.));
        EXPECT_LT(angle, .01 * CV_PI / 180);
        EXPECT_LT(cv::norm(cv::Vec3d(d(0, 3), d(1, 3), d(2, 3))), .0001);
        EXPECT_LT(result.training_rms[i], .001);
    }
    const auto checked = validate(result, f.observations(20, 5), .1);
    EXPECT_TRUE(checked.validated) << checked.error;
}
TEST(RigCalibration, ResolvesReversedCornerOrderAcrossDiversePoses) {
    Fixture f;
    auto result = solve_joint(f.models, f.observations(0, 12, true), 1, .025);
    ASSERT_TRUE(result.trained) << result.error;
    EXPECT_TRUE(validate(result, f.observations(20, 5, true), .1).validated);
}
TEST(RigCalibration, HeldOutFailureDoesNotRefitGeometry) {
    Fixture f;
    auto result = solve_joint(f.models, f.observations(0, 12), 1, .025);
    ASSERT_TRUE(result.trained) << result.error;
    auto held = f.observations(20, 4);
    for (auto &v : held)
        for (auto &p : v.corners[2])
            p.x += 8;
    const auto checked = validate(result, held, .5);
    EXPECT_FALSE(checked.validated);
    EXPECT_TRUE(result.validation_views.empty());
    EXPECT_EQ(checked.T_camera_from_reference, result.T_camera_from_reference);
}
TEST(RigCalibration, IntrinsicsUseTheExistingPinholeSolver) {
    Fixture f;
    const auto views = f.observations(0, 20);
    std::array<std::vector<std::vector<cv::Point2f>>, 3> intrinsic;
    for (const auto &v : views)
        for (int i = 0; i < 3; ++i)
            intrinsic[i].push_back(v.corners[i]);
    const auto result = solve_intrinsics_and_joint({f.models[0].size, f.models[1].size, f.models[2].size},
                                                   intrinsic, views, 1, .025);
    ASSERT_TRUE(result.trained) << result.error;
    EXPECT_TRUE(validate(result, f.observations(30, 4), .1).validated);
}
TEST(RigCalibration, MissingCornersAndUnmeasuredBoardAreRejected) {
    Fixture f;
    auto views = f.observations(0, 10);
    views[0].corners[0].pop_back();
    EXPECT_FALSE(solve_joint(f.models, views, 1, .025).trained);
    EXPECT_FALSE(solve_joint(f.models, f.observations(0, 10), 1, 0).trained);
    EXPECT_FALSE(solve_joint(f.models, f.observations(0, 10), 3, .025).trained);
}
TEST(RigCalibration, IndependentNoiseIsBoundedAndWrongPairingIsRejected) {
    Fixture f;
    auto views = f.observations(0, 16);
    cv::RNG random(2919);
    for (auto &v : views)
        for (auto &c : v.corners)
            for (auto &p : c) {
                p.x += static_cast<float>(random.gaussian(.12));
                p.y += static_cast<float>(random.gaussian(.12));
            }
    auto result = solve_joint(f.models, views, 1, .025);
    ASSERT_TRUE(result.trained) << result.error;
    EXPECT_TRUE(validate(result, f.observations(30, 4), .7).validated);
    std::rotate(views[0].corners[2].begin(), views[0].corners[2].begin() + 7, views[0].corners[2].end());
    auto bad = solve_joint(f.models, views, 1, .025);
    EXPECT_TRUE(!bad.trained || !validate(bad, f.observations(30, 4), .1).validated);
}
TEST(RigCalibration, HeldOutTrainingReuseAndDuplicateIdentitiesAreRejected) {
    Fixture f;
    auto training = f.observations(0, 12);
    auto solved = solve_joint(f.models, training, 1, .025);
    ASSERT_TRUE(solved.trained) << solved.error;
    EXPECT_FALSE(validate(solved, training, .1).validated);
    training[1].id = training[0].id;
    EXPECT_FALSE(solve_joint(f.models, training, 1, .025).trained);
}
TEST(RigCalibration, ParallelBoardViewsCannotBeApprovedForIntrinsics) {
    Fixture f;
    for (auto &m : f.models)
        m.distortion = cv::Mat::zeros(1, 5, CV_64F);
    auto views = f.observations(0, 10);
    std::array<std::vector<std::vector<cv::Point2f>>, 3> points;
    for (size_t k = 0; k < views.size(); ++k)
        for (int i = 0; i < 3; ++i) {
            cv::Vec3d rotation(0, 0, .02 * k), translation(-.12 + .01 * k, -.08, .7 + .02 * k);
            cv::projectPoints(object_points(.025), rotation, translation, f.models[i].K,
                              f.models[i].distortion, views[k].corners[i]);
            points[i].push_back(views[k].corners[i]);
        }
    auto result = solve_intrinsics_and_joint({f.models[0].size, f.models[1].size, f.models[2].size}, points,
                                             views, 1, .025);
    EXPECT_FALSE(result.trained);
    EXPECT_NE(result.error.find("tilts"), std::string::npos);
}
TEST(RigCalibration, RelabeledOrOverlappingTrainingCannotBecomeHeldOut) {
    Fixture f;
    auto training = f.observations(0, 12);
    const auto solved = solve_joint(f.models, training, 1, .025);
    ASSERT_TRUE(solved.trained) << solved.error;
    auto copied = training;
    for (auto &v : copied) {
        v.id += 100;
        v.start_us += 100000000;
        v.end_us += 100000000;
        for (auto &points : v.corners) {
            std::reverse(points.begin(), points.end());
            for (auto &p : points)
                p.x += .1f;
        }
    }
    EXPECT_FALSE(validate(solved, copied, .1).validated);
    auto overlapping = f.observations(20, 3);
    overlapping[0].start_us = training[0].start_us + 50000;
    overlapping[0].end_us = overlapping[0].start_us + 100000;
    EXPECT_FALSE(validate(solved, overlapping, .1).validated);
    auto held = f.observations(20, 3);
    held[1] = held[0];
    held[1].id += 100;
    held[1].start_us += 1000000;
    held[1].end_us += 1000000;
    EXPECT_FALSE(validate(solved, held, .1).validated);
}
TEST(RigCalibration, IntrinsicOnlyTrainingCannotBecomeHeldOut) {
    Fixture f;
    const auto common = f.observations(0, 12);
    const auto extra = f.observations(20, 4);
    std::array<std::vector<std::vector<cv::Point2f>>, 3> intrinsic;
    for (const auto &v : common)
        for (int i = 0; i < 3; ++i)
            intrinsic[i].push_back(v.corners[i]);
    intrinsic[2].push_back(extra[0].corners[2]);
    const auto solved = solve_intrinsics_and_joint({f.models[0].size, f.models[1].size, f.models[2].size},
                                                   intrinsic, common, 1, .025);
    ASSERT_TRUE(solved.trained) << solved.error;
    EXPECT_FALSE(validate(solved, extra, .2).validated);
}
TEST(RigCalibration, ATrainingOutlierCannotHideInTheAggregateRms) {
    Fixture f;
    auto views = f.observations(0, 16);
    views[0].corners[2][10].x += 8;
    const auto solved = solve_joint(f.models, views, 1, .025);
    ASSERT_TRUE(solved.trained) << solved.error;
    ASSERT_EQ(solved.training_views.size(), views.size());
    EXPECT_LT(solved.training_rms[2], .5);
    EXPECT_GT(solved.training_views[0][2], .5);
    const auto checked = validate(solved, f.observations(30, 4), .5);
    EXPECT_FALSE(checked.validated);
    EXPECT_EQ(checked.T_camera_from_reference, solved.T_camera_from_reference);
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(cv::norm(checked.models[i].K - solved.models[i].K), 0);
        EXPECT_EQ(cv::norm(checked.models[i].distortion - solved.models[i].distortion), 0);
    }
}
TEST(RigCalibration, InvalidModelsAndCommonWindowsAreRejected) {
    Fixture f;
    auto models = f.models;
    models[0].K = models[0].K.clone();
    models[0].K.at<double>(0, 1) = 10;
    EXPECT_FALSE(solve_joint(models, f.observations(0, 12), 1, .025).trained);
    auto views = f.observations(0, 12);
    views[0].end_us += 1;
    EXPECT_FALSE(solve_joint(f.models, views, 1, .025).trained);
    views = f.observations(0, 12);
    views[1].start_us = views[0].start_us + 50000;
    views[1].end_us = views[1].start_us + 100000;
    EXPECT_FALSE(solve_joint(f.models, views, 1, .025).trained);
    const auto solved = solve_joint(f.models, f.observations(0, 12), 1, .025);
    ASSERT_TRUE(solved.trained) << solved.error;
    auto held = f.observations(20, 3);
    held[0].end_us += 1;
    EXPECT_FALSE(validate(solved, held, .1).validated);
    auto malformed = solved;
    malformed.reference = 3;
    EXPECT_FALSE(validate(malformed, f.observations(20, 3), .1).validated);
    malformed = solved;
    malformed.T_camera_from_reference[1](0, 3) = .01;
    EXPECT_FALSE(validate(malformed, f.observations(20, 3), .1).validated);
    malformed = solved;
    malformed.training_observations.clear();
    malformed.intrinsic_training_views = {};
    EXPECT_FALSE(validate(malformed, f.observations(20, 3), .1).validated);
}
TEST(RigCalibration, MalformedIntrinsicInputsFailWithoutThrowing) {
    Fixture f;
    std::array<std::vector<std::vector<cv::Point2f>>, 3> points;
    for (const auto &v : f.observations(0, 12))
        for (int i = 0; i < 3; ++i)
            points[i].push_back(v.corners[i]);
    points[0][0][0].x = std::numeric_limits<float>::quiet_NaN();
    EXPECT_NO_THROW({
        const auto result = solve_intrinsics_and_joint({f.models[0].size, f.models[1].size, f.models[2].size},
                                                       points, f.observations(0, 12), 1, .025);
        EXPECT_FALSE(result.trained);
    });
}
TEST(RigCalibration, NoisyDistortedParallelViewsStillLackIntrinsicObservability) {
    Fixture f;
    auto views = f.observations(0, 12);
    std::array<std::vector<std::vector<cv::Point2f>>, 3> points;
    cv::RNG noise(2919);
    for (size_t k = 0; k < views.size(); ++k)
        for (int i = 0; i < 3; ++i) {
            cv::Vec3d rotation(0, 0, .02 * k), translation(-.12 + .01 * k, -.08, .7 + .02 * k);
            cv::projectPoints(object_points(.025), rotation, translation, f.models[i].K,
                              f.models[i].distortion, views[k].corners[i]);
            for (auto &p : views[k].corners[i]) {
                p.x += static_cast<float>(noise.gaussian(.12));
                p.y += static_cast<float>(noise.gaussian(.12));
            }
            points[i].push_back(views[k].corners[i]);
        }
    const auto solved = solve_intrinsics_and_joint({f.models[0].size, f.models[1].size, f.models[2].size},
                                                   points, views, 1, .025);
    EXPECT_FALSE(solved.trained) << solved.error;
}
TEST(RigCalibration, ReferenceGaugeIsNotHardCodedToTheMiddleCamera) {
    Fixture f;
    for (auto &m : f.models)
        m.distortion = m.distortion.t();
    const auto solved = solve_joint(f.models, f.observations(0, 12), 0, .025);
    ASSERT_TRUE(solved.trained) << solved.error;
    EXPECT_EQ(solved.T_camera_from_reference[0], cv::Matx44d::eye());
    for (int i = 0; i < 3; ++i)
        EXPECT_LT(cv::norm(cv::Mat(solved.T_camera_from_reference[i] - f.truth[i] * f.truth[0].inv())), 1e-4);
    EXPECT_TRUE(validate(solved, f.observations(20, 5), .1).validated);
}
} // namespace
