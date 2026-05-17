#include "adapters/slam/visual_pnp_pose_backend.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace smartdrone::adapters::slam {

bool EstimateVisualPnpPoseRansac(const std::vector<cv::Point3f> &objectPoints,
                                 const std::vector<cv::Point2f> &imagePoints,
                                 const VisualPnpPoseBackendOptions &options,
                                 VisualPnpPoseBackendResult &result)
{
    result = {};
    const size_t pointCount = std::min(objectPoints.size(), imagePoints.size());
    if (pointCount < static_cast<size_t>(std::max(1, options.minPoints)) || options.cameraMatrix.empty()) {
        return false;
    }

    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat inliers;
    bool ok = false;
    try {
        ok = cv::solvePnPRansac(objectPoints, imagePoints, options.cameraMatrix, options.distCoeffs, rvec, tvec,
                                false, options.iterations, options.reprojectionError, options.confidence, inliers,
                                options.method);
        result.inlierCount = inliers.rows;
    } catch (const cv::Exception &e) {
        if (!options.logTag.empty()) {
            std::cerr << "[" << options.logTag << "] solvePnPRansac skipped points=" << objectPoints.size()
                      << " error=" << e.what() << "\n";
        }
        return false;
    }

    if (!ok || result.inlierCount < options.minInliers) {
        return false;
    }

    result.inlierIndices.reserve(static_cast<size_t>(result.inlierCount));
    std::vector<cv::Point3f> inlierObjectPoints;
    std::vector<cv::Point2f> inlierImagePoints;
    if (options.refineWithInliers) {
        inlierObjectPoints.reserve(static_cast<size_t>(result.inlierCount));
        inlierImagePoints.reserve(static_cast<size_t>(result.inlierCount));
    }
    for (int row = 0; row < inliers.rows; ++row) {
        const int idx = inliers.at<int>(row, 0);
        if (idx < 0 || static_cast<size_t>(idx) >= pointCount) {
            continue;
        }
        result.inlierIndices.push_back(idx);
        if (options.refineWithInliers) {
            inlierObjectPoints.push_back(objectPoints[static_cast<size_t>(idx)]);
            inlierImagePoints.push_back(imagePoints[static_cast<size_t>(idx)]);
        }
    }
    result.inlierCount = static_cast<int>(result.inlierIndices.size());
    if (result.inlierCount < options.minInliers) {
        result.inlierIndices.clear();
        return false;
    }

    if (options.refineWithInliers && inlierObjectPoints.size() >= static_cast<size_t>(options.minInliers)) {
        try {
            (void)cv::solvePnP(inlierObjectPoints, inlierImagePoints, options.cameraMatrix, options.distCoeffs,
                               rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);
        } catch (const cv::Exception &e) {
            if (!options.logTag.empty()) {
                std::cerr << "[" << options.logTag << "] iterative refine skipped inliers="
                          << inlierObjectPoints.size() << " error=" << e.what() << "\n";
            }
        }
    }

    cv::Mat Rcv;
    cv::Rodrigues(rvec, Rcv);
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t = Eigen::Vector3f::Zero();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            R(r, c) = static_cast<float>(Rcv.at<double>(r, c));
        }
        t(r) = static_cast<float>(tvec.at<double>(r, 0));
    }
    if (!std::isfinite(t.norm())) {
        result.poseValid = false;
        return false;
    }

    result.T_camera_object = Sophus::SE3f(Sophus::SO3f(R), t);
    result.poseValid = true;
    return true;
}

} // namespace smartdrone::adapters::slam
