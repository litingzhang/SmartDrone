#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "core/ports/visual_pose_backend.h"

namespace SmartDrone::adapters::slam {

struct KltPnpCameraIntrinsics {
    float fx{0.0f};
    float fy{0.0f};
    float cx{0.0f};
    float cy{0.0f};
};

struct KltPnpPoseEstimatorOptions {
    KltPnpCameraIntrinsics camera;
    int minPoints{4};
    int minInliers{4};
    int iterations{80};
    double reprojectionError{4.0};
    double confidence{0.995};
    int method{0};
    bool refineWithInliers{false};
    bool horizontalLateralFlow{false};
    std::string logTag;
};

struct KltPnpPoseEstimateResult {
    bool poseUpdated{false};
    int inlierCount{0};
    Sophus::SE3f deltaTwc{Sophus::SE3f()};
    std::vector<int> inlierIndices;
};

KltPnpPoseEstimatorOptions
MakeKltContinuousPnpPoseEstimatorOptions(const KltPnpCameraIntrinsics &camera,
                                         bool horizontalLateralFlow);

KltPnpPoseEstimatorOptions
MakeKltPerFramePnpPoseEstimatorOptions(const KltPnpCameraIntrinsics &camera,
                                       bool preferVpiDefaults);

KltPnpPoseEstimateResult
EstimateKltPnpPoseDelta(const std::vector<cv::Point3f> &objectPoints,
                        const std::vector<cv::Point2f> &imagePoints,
                        const KltPnpPoseEstimatorOptions &options,
                        core::ports::IVisualPnpPoseBackend &backend);

KltPnpPoseEstimateResult
EstimateKltPnpPoseDelta(const std::vector<cv::Point3f> &objectPoints,
                        const std::vector<cv::Point2f> &imagePoints,
                        const KltPnpPoseEstimatorOptions &options);

} // namespace SmartDrone::adapters::slam
