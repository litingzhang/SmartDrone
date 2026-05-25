#include "adapters/slam/klt/klt_pose_estimator.h"

#include "adapters/slam/klt/klt_mode_utils.h"
#include "adapters/slam/engine/slam_env.h"
#include "adapters/slam/engine/slam_image_utils.h"
#include "adapters/slam/stereo/visual_pnp_pose_backend.h"
#include "common/environment.h"

#include <algorithm>
#include <utility>

#include <opencv2/calib3d.hpp>

namespace SmartDrone::Adapters::Slam {
namespace {

constexpr double LK_PER_FRAME_ACCELERATED_PNP_REPROJ_THRESHOLD_PX = 3.0;

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
    const double defaultReprojection =
        preferVpiDefaults ? LK_PER_FRAME_ACCELERATED_PNP_REPROJ_THRESHOLD_PX
                          : LK_PER_FRAME_PNP_REPROJ_THRESHOLD_PX;
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
    return options;
}

KltPnpPoseEstimateResult
EstimateKltPnpPoseDelta(const std::vector<cv::Point3f> &objectPoints,
                        const std::vector<cv::Point2f> &imagePoints,
                        const KltPnpPoseEstimatorOptions &options,
                        Core::Ports::IVisualPnpPoseBackend &backend)
{
    KltPnpPoseEstimateResult result;
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
    result.inlierIndices = std::move(pnpResult.inlierIndices);
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
