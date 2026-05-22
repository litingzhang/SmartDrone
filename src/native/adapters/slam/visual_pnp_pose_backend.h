#pragma once

#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "core/ports/visual_pose_backend.h"

namespace SmartDrone::adapters::slam {

using VisualPnpPoseBackendOptions = core::ports::VisualPnpPoseBackendOptions;
using VisualPnpPoseBackendResult = core::ports::VisualPnpPoseBackendResult;

class DefaultVisualPnpPoseBackend final : public core::ports::IVisualPnpPoseBackend {
  public:
    bool EstimatePoseRansac(const std::vector<cv::Point3f> &objectPoints,
                            const std::vector<cv::Point2f> &imagePoints,
                            const VisualPnpPoseBackendOptions &options,
                            VisualPnpPoseBackendResult &result) const override;
};

bool EstimateVisualPnpPoseRansac(const std::vector<cv::Point3f> &objectPoints,
                                 const std::vector<cv::Point2f> &imagePoints,
                                 const VisualPnpPoseBackendOptions &options,
                                 VisualPnpPoseBackendResult &result);

} // namespace SmartDrone::adapters::slam
