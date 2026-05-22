#pragma once

#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "core/ports/visual_pose_backend.h"

namespace SmartDrone::Adapters::Slam {

using VisualPnpPoseBackendOptions = Core::Ports::VisualPnpPoseBackendOptions;
using VisualPnpPoseBackendResult = Core::Ports::VisualPnpPoseBackendResult;

class DefaultVisualPnpPoseBackend final : public Core::Ports::IVisualPnpPoseBackend {
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

} // namespace SmartDrone::Adapters::Slam
