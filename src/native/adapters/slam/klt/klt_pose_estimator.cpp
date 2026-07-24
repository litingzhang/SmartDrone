#include "adapters/slam/klt/klt_pose_estimator.h"

#include "adapters/slam/klt/klt_mode_utils.h"
#include "adapters/slam/engine/slam_env.h"
#include "adapters/slam/engine/slam_image_utils.h"
#include "adapters/slam/stereo/visual_pnp_pose_backend.h"
#include "common/environment.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Eigenvalues>
#include <opencv2/calib3d.hpp>

namespace SmartDrone::Adapters::Slam {
namespace {

constexpr float LK_PNP_DEPTH_TRIM_FRACTION = 0.20f;

void ConfigureKltPnpGeometryGate(KltPnpPoseEstimatorOptions &options)
{
    options.requireObservableGeometry =
        EnvFlagEnabled("SMART_DRONE_LK_PNP_GEOMETRY_GATE", true);
    options.minRobustDepthSpanMeters = EnvFloatValueClamped(
        "SMART_DRONE_LK_PNP_MIN_DEPTH_SPAN_M", 0.30f, 0.0f, 5.0f);
    options.minRobustDepthSpanRatio = EnvFloatValueClamped(
        "SMART_DRONE_LK_PNP_MIN_DEPTH_SPAN_RATIO", 0.10f, 0.0f, 1.0f);
    options.minNormalizedThickness = EnvFloatValueClamped(
        "SMART_DRONE_LK_PNP_MIN_NORMALIZED_THICKNESS", 0.015f, 0.0f,
        0.50f);
}

std::vector<cv::Point3f> CollectFiniteObjectPoints(
    const std::vector<cv::Point3f> &objectPoints, size_t pointCount)
{
    std::vector<cv::Point3f> finitePoints;
    finitePoints.reserve(pointCount);
    for (size_t i = 0; i < std::min(pointCount, objectPoints.size()); ++i) {
        const cv::Point3f &point = objectPoints[i];
        if (std::isfinite(point.x) && std::isfinite(point.y) &&
            std::isfinite(point.z)) {
            finitePoints.push_back(point);
        }
    }
    return finitePoints;
}

std::vector<cv::Point3f> CollectPnpInlierObjectPoints(
    const std::vector<cv::Point3f> &objectPoints,
    const VisualPnpPoseBackendResult &pnpResult)
{
    if (pnpResult.inlierIndices.empty()) {
        return CollectFiniteObjectPoints(objectPoints, objectPoints.size());
    }
    std::vector<cv::Point3f> inlierPoints;
    inlierPoints.reserve(pnpResult.inlierIndices.size());
    for (int index : pnpResult.inlierIndices) {
        if (index < 0 || static_cast<size_t>(index) >= objectPoints.size()) {
            continue;
        }
        const cv::Point3f &point = objectPoints[static_cast<size_t>(index)];
        if (std::isfinite(point.x) && std::isfinite(point.y) &&
            std::isfinite(point.z)) {
            inlierPoints.push_back(point);
        }
    }
    return inlierPoints;
}

bool HasRobustDepthSpan(const std::vector<cv::Point3f> &points,
                        const KltPnpPoseEstimatorOptions &options)
{
    std::vector<float> depths;
    depths.reserve(points.size());
    for (const cv::Point3f &point : points) {
        depths.push_back(point.z);
    }
    std::sort(depths.begin(), depths.end());
    const size_t lastIndex = depths.size() - 1;
    const size_t lowIndex = static_cast<size_t>(
        std::floor(LK_PNP_DEPTH_TRIM_FRACTION * lastIndex));
    const size_t highIndex = static_cast<size_t>(
        std::ceil((1.0f - LK_PNP_DEPTH_TRIM_FRACTION) * lastIndex));
    const float span = depths[highIndex] - depths[lowIndex];
    const float median = depths[depths.size() / 2];
    return span >= options.minRobustDepthSpanMeters &&
           span >= options.minRobustDepthSpanRatio * std::max(median, 0.01f);
}

float NormalizedGeometryThickness(const std::vector<cv::Point3f> &points)
{
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const cv::Point3f &point : points) {
        centroid += Eigen::Vector3f(point.x, point.y, point.z);
    }
    centroid /= static_cast<float>(points.size());

    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const cv::Point3f &point : points) {
        const Eigen::Vector3f centered =
            Eigen::Vector3f(point.x, point.y, point.z) - centroid;
        covariance.noalias() += centered * centered.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    if (solver.info() != Eigen::Success) {
        return 0.0f;
    }
    const Eigen::Vector3f eigenvalues = solver.eigenvalues().cwiseMax(0.0f);
    return std::sqrt(eigenvalues.x() / std::max(eigenvalues.z(), 1.0e-9f));
}

bool HasObservableKltPnpGeometry(
    const std::vector<cv::Point3f> &points,
    const KltPnpPoseEstimatorOptions &options, int minimumPointCount)
{
    if (!options.requireObservableGeometry) {
        return true;
    }
    if (points.size() < static_cast<size_t>(std::max(4, minimumPointCount))) {
        return false;
    }
    return HasRobustDepthSpan(points, options) &&
           NormalizedGeometryThickness(points) >=
               options.minNormalizedThickness;
}

} // namespace

KltPnpPoseEstimatorOptions
MakeKltContinuousPnpPoseEstimatorOptions(const KltPnpCameraIntrinsics &camera,
                                         bool horizontalLateralFlow)
{
    KltPnpPoseEstimatorOptions options;
    options.camera = camera;
    options.minPoints = LK_MIN_PNP_POINTS;
    options.minInliers = LK_MIN_PNP_INLIERS;
    options.iterations = 80;
    options.reprojectionError = 4.0;
    options.confidence = 0.995;
    options.method = cv::SOLVEPNP_EPNP;
    options.horizontalLateralFlow = horizontalLateralFlow;
    options.logTag = "lk_pnp";
    ConfigureKltPnpGeometryGate(options);
    return options;
}

KltPnpPoseEstimatorOptions
MakeKltPerFramePnpPoseEstimatorOptions(const KltPnpCameraIntrinsics &camera,
                                       bool preferVpiDefaults)
{
    KltPnpPoseEstimatorOptions options;
    options.camera = camera;
    options.minPoints = LK_MIN_PNP_POINTS;
    options.minInliers = LK_MIN_PNP_INLIERS;
    options.iterations =
        std::max(20, EnvIntValue("SMART_DRONE_LK_PER_FRAME_PNP_ITERS",
                                 LK_PER_FRAME_DEFAULT_PNP_ITERATIONS));
    options.confidence =
        std::clamp(static_cast<double>(EnvFloatValue(
                       "SMART_DRONE_LK_PER_FRAME_PNP_CONF",
                       static_cast<float>(LK_PER_FRAME_DEFAULT_PNP_CONFIDENCE))),
                   0.5, 0.9999);
    (void)preferVpiDefaults;
    const double defaultReprojection =
        LK_PER_FRAME_PNP_REPROJ_THRESHOLD_PX;
    const bool reprojectionOverride =
        !SmartDrone::Common::EnvVarIsUnsetOrEmpty(
            "SMART_DRONE_LK_PER_FRAME_PNP_REPROJ");
    options.reprojectionError =
        reprojectionOverride
            ? std::max(0.5, static_cast<double>(EnvFloatValue(
                                "SMART_DRONE_LK_PER_FRAME_PNP_REPROJ",
                                static_cast<float>(defaultReprojection))))
            : defaultReprojection;
    options.method = LkPerFramePnPMethod();
    options.refineWithInliers = true;
    options.logTag = "lk_per_frame_pnp";
    ConfigureKltPnpGeometryGate(options);
    return options;
}

KltPnpPoseEstimateResult
EstimateKltPnpPoseDelta(const std::vector<cv::Point3f> &objectPoints,
                        const std::vector<cv::Point2f> &imagePoints,
                        const KltPnpPoseEstimatorOptions &options,
                        Core::Ports::IVisualPnpPoseBackend &backend)
{
    KltPnpPoseEstimateResult result;
    const size_t pointCount = std::min(objectPoints.size(), imagePoints.size());
    const std::vector<cv::Point3f> candidatePoints =
        CollectFiniteObjectPoints(objectPoints, pointCount);
    if (!HasObservableKltPnpGeometry(candidatePoints, options,
                                     options.minPoints)) {
        return result;
    }

    VisualPnpPoseBackendOptions pnpOptions;
    pnpOptions.cameraMatrix =
        MakeCameraMatrix(options.camera.fx, options.camera.fy, options.camera.cx,
                         options.camera.cy);
    pnpOptions.minPoints = options.minPoints;
    pnpOptions.minInliers = options.minInliers;
    pnpOptions.iterations = options.iterations;
    pnpOptions.reprojectionError = options.reprojectionError;
    pnpOptions.confidence = options.confidence;
    pnpOptions.method = options.method;
    pnpOptions.refineWithInliers = options.refineWithInliers;
    pnpOptions.logTag = options.logTag;

    VisualPnpPoseBackendResult pnpResult;
    if (!backend.EstimatePoseRansac(objectPoints, imagePoints, pnpOptions,
                                    pnpResult)) {
        return result;
    }

    result.inlierCount = pnpResult.inlierCount;
    const std::vector<cv::Point3f> inlierPoints =
        CollectPnpInlierObjectPoints(objectPoints, pnpResult);
    result.inlierIndices = std::move(pnpResult.inlierIndices);
    if (!HasObservableKltPnpGeometry(inlierPoints, options,
                                     options.minInliers)) {
        return result;
    }
    if (pnpResult.T_camera_object.translation().norm() > LK_MAX_STEP_METERS) {
        return result;
    }

    result.deltaTwc = StabilizeLkCameraDelta(pnpResult.T_camera_object.inverse(),
                                             options.horizontalLateralFlow);
    result.poseUpdated = true;
    return result;
}

KltPnpPoseEstimateResult
EstimateKltPnpPoseDelta(const std::vector<cv::Point3f> &objectPoints,
                        const std::vector<cv::Point2f> &imagePoints,
                        const KltPnpPoseEstimatorOptions &options)
{
    DefaultVisualPnpPoseBackend pnpBackend;
    return EstimateKltPnpPoseDelta(objectPoints, imagePoints, options,
                                   pnpBackend);
}

} // namespace SmartDrone::Adapters::Slam
