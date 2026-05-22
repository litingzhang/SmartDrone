#pragma once

#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace SmartDrone::core::ports {

struct VisualPnpPoseBackendOptions {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    int minPoints{4};
    int minInliers{4};
    int iterations{80};
    double reprojectionError{4.0};
    double confidence{0.995};
    int method{cv::SOLVEPNP_EPNP};
    bool refineWithInliers{false};
    std::string logTag;
};

struct VisualPnpPoseBackendResult {
    bool poseValid{false};
    int inlierCount{0};
    Sophus::SE3f T_camera_object{Sophus::SE3f()};
    std::vector<int> inlierIndices;
};

class IVisualPnpPoseBackend {
  public:
    virtual ~IVisualPnpPoseBackend() = default;

    virtual bool EstimatePoseRansac(const std::vector<cv::Point3f> &objectPoints,
                                    const std::vector<cv::Point2f> &imagePoints,
                                    const VisualPnpPoseBackendOptions &options,
                                    VisualPnpPoseBackendResult &result) const = 0;
};

} // namespace SmartDrone::core::ports
