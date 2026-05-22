#include "adapters/slam/klt_pose_estimator.h"

#include "adapters/slam/klt_mode_utils.h"
#include "adapters/slam/slam_env.h"
#include "adapters/slam/slam_image_utils.h"
#include "adapters/slam/visual_pnp_pose_backend.h"

#include <algorithm>
#include <cstdlib>
#include <utility>

#include <opencv2/calib3d.hpp>

namespace SmartDrone::adapters::slam {
namespace {

constexpr double kLkPerFrameAcceleratedPnPReprojThresholdPx = 3.0;

} // namespace

KltPnpPoseEstimatorOptions
MakeKltContinuousPnpPoseEstimatorOptions(const KltPnpCameraIntrinsics &camera,
                                         bool horizontalLateralFlow)
{
    KltPnpPoseEstimatorOptions options;
    options.camera = camera;
    options.minPoints = kLkMinPnPPoints;
    options.minInliers = kLkMinPnPInliers;
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
    options.minPoints = kLkMinPnPPoints;
    options.minInliers = kLkMinPnPInliers;
    options.iterations =
        std::max(20, EnvIntValue("SMART_DRONE_LK_PER_FRAME_PNP_ITERS",
                                 kLkPerFrameDefaultPnPIterations));
    options.confidence =
        std::clamp(static_cast<double>(EnvFloatValue(
                       "SMART_DRONE_LK_PER_FRAME_PNP_CONF",
                       static_cast<float>(kLkPerFrameDefaultPnPConfidence))),
                   0.5, 0.9999);
    const double defaultReprojection =
        preferVpiDefaults ? kLkPerFrameAcceleratedPnPReprojThresholdPx
                          : kLkPerFramePnPReprojThresholdPx;
    const char *reprojectionOverride =
        std::getenv("SMART_DRONE_LK_PER_FRAME_PNP_REPROJ");
    options.reprojectionError =
        reprojectionOverride != nullptr && reprojectionOverride[0] != '\0'
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
                        core::ports::IVisualPnpPoseBackend &backend)
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
    if (pnpResult.T_camera_object.translation().norm() > kLkMaxStepMeters) {
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

} // namespace SmartDrone::adapters::slam
