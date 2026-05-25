#include "adapters/slam/stereo/visual_pnp_pose_backend.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace SmartDrone::Adapters::Slam {
namespace {

struct PnpRansacRequest {
    const std::vector<cv::Point3f> &objectPoints;
    const std::vector<cv::Point2f> &imagePoints;
    const VisualPnpPoseBackendOptions &options;
    cv::Mat &rvec;
    cv::Mat &tvec;
    cv::Mat &inliers;
};

struct CollectPnpInliersRequest {
    const std::vector<cv::Point3f> &objectPoints;
    const std::vector<cv::Point2f> &imagePoints;
    const cv::Mat &inliers;
    size_t pointCount;
    bool collectPoints;
    VisualPnpPoseBackendResult &result;
    std::vector<cv::Point3f> &inlierObjectPoints;
    std::vector<cv::Point2f> &inlierImagePoints;
};

bool HasEnoughPnpInput(const std::vector<cv::Point3f> &objectPoints,
                       const std::vector<cv::Point2f> &imagePoints,
                       const VisualPnpPoseBackendOptions &options,
                       size_t &pointCount)
{
    pointCount = std::min(objectPoints.size(), imagePoints.size());
    return pointCount >= static_cast<size_t>(std::max(1, options.minPoints)) && !options.cameraMatrix.empty();
}

void LogPnpException(const VisualPnpPoseBackendOptions &options, const char *stage, size_t pointCount,
                     const cv::Exception &error)
{
    if (options.logTag.empty()) {
        return;
    }
    std::cerr << "[" << options.logTag << "] " << stage << " skipped points=" << pointCount
              << " error=" << error.what() << "\n";
}

bool RunPnpRansac(const PnpRansacRequest &request)
{
    const auto &options = request.options;
    try {
        return cv::solvePnPRansac(
            request.objectPoints, request.imagePoints, options.cameraMatrix,
            options.distCoeffs, request.rvec, request.tvec, false,
            options.iterations, options.reprojectionError, options.confidence,
            request.inliers, options.method);
    } catch (const cv::Exception &error) {
        LogPnpException(options, "solvePnPRansac",
                        request.objectPoints.size(), error);
        return false;
    }
}

void CollectPnpInliers(const CollectPnpInliersRequest &request)
{
    const cv::Mat &inliers = request.inliers;
    VisualPnpPoseBackendResult &result = request.result;
    result.inlierIndices.reserve(static_cast<size_t>(inliers.rows));
    if (request.collectPoints) {
        request.inlierObjectPoints.reserve(static_cast<size_t>(inliers.rows));
        request.inlierImagePoints.reserve(static_cast<size_t>(inliers.rows));
    }
    for (int row = 0; row < inliers.rows; ++row) {
        const int index = inliers.at<int>(row, 0);
        if (index < 0 || static_cast<size_t>(index) >= request.pointCount) {
            continue;
        }
        result.inlierIndices.push_back(index);
        if (request.collectPoints) {
            request.inlierObjectPoints.push_back(
                request.objectPoints[static_cast<size_t>(index)]);
            request.inlierImagePoints.push_back(
                request.imagePoints[static_cast<size_t>(index)]);
        }
    }
    result.inlierCount = static_cast<int>(result.inlierIndices.size());
}

bool HasEnoughInliers(const VisualPnpPoseBackendOptions &options, VisualPnpPoseBackendResult &result)
{
    if (result.inlierCount >= options.minInliers) {
        return true;
    }
    result.inlierIndices.clear();
    return false;
}

void RefinePnpWithInliers(const std::vector<cv::Point3f> &inlierObjectPoints,
                          const std::vector<cv::Point2f> &inlierImagePoints,
                          const VisualPnpPoseBackendOptions &options, cv::Mat &rvec, cv::Mat &tvec)
{
    if (!options.refineWithInliers || inlierObjectPoints.size() < static_cast<size_t>(options.minInliers)) {
        return;
    }
    try {
        (void)cv::solvePnP(inlierObjectPoints, inlierImagePoints, options.cameraMatrix, options.distCoeffs, rvec, tvec,
                           true, cv::SOLVEPNP_ITERATIVE);
    } catch (const cv::Exception &error) {
        LogPnpException(options, "iterative refine", inlierObjectPoints.size(), error);
    }
}

bool ConvertPnpPoseToSophus(const cv::Mat &rvec, const cv::Mat &tvec, Sophus::SE3f &pose)
{
    cv::Mat rotationCv;
    cv::Rodrigues(rvec, rotationCv);
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
    Eigen::Vector3f translation = Eigen::Vector3f::Zero();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            rotation(row, col) = static_cast<float>(rotationCv.at<double>(row, col));
        }
        translation(row) = static_cast<float>(tvec.at<double>(row, 0));
    }
    if (!std::isfinite(translation.norm())) {
        return false;
    }
    pose = Sophus::SE3f(Sophus::SO3f(rotation), translation);
    return true;
}

} // namespace

bool EstimateVisualPnpPoseRansac(const std::vector<cv::Point3f> &objectPoints,
                                 const std::vector<cv::Point2f> &imagePoints,
                                 const VisualPnpPoseBackendOptions &options,
                                 VisualPnpPoseBackendResult &result)
{
    result = {};
    size_t pointCount = 0;
    if (!HasEnoughPnpInput(objectPoints, imagePoints, options, pointCount)) {
        return false;
    }

    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat inliers;
    if (!RunPnpRansac(
            {objectPoints, imagePoints, options, rvec, tvec, inliers}) ||
        inliers.rows < options.minInliers) {
        return false;
    }

    std::vector<cv::Point3f> inlierObjectPoints;
    std::vector<cv::Point2f> inlierImagePoints;
    CollectPnpInliers({objectPoints, imagePoints, inliers, pointCount,
                       options.refineWithInliers, result, inlierObjectPoints,
                       inlierImagePoints});
    if (!HasEnoughInliers(options, result)) {
        return false;
    }

    RefinePnpWithInliers(inlierObjectPoints, inlierImagePoints, options, rvec, tvec);
    if (!ConvertPnpPoseToSophus(rvec, tvec, result.T_camera_object)) {
        result.poseValid = false;
        return false;
    }

    result.poseValid = true;
    return true;
}

bool DefaultVisualPnpPoseBackend::EstimatePoseRansac(const std::vector<cv::Point3f> &objectPoints,
                                                     const std::vector<cv::Point2f> &imagePoints,
                                                     const VisualPnpPoseBackendOptions &options,
                                                     VisualPnpPoseBackendResult &result) const
{
    return EstimateVisualPnpPoseRansac(objectPoints, imagePoints, options, result);
}

} // namespace SmartDrone::Adapters::Slam
