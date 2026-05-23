#pragma once

#include <string>

#include <opencv2/core.hpp>

#include "core/ports/stereo_processing.h"

namespace SmartDrone::Adapters::Slam {

using StereoCameraIntrinsics = Core::Ports::StereoCameraIntrinsics;
using StereoRectification = Core::Ports::StereoRectification;
using StereoCalibration = Core::Ports::StereoCalibration;

class DefaultStereoCalibrationLoader final
    : public Core::Ports::IStereoCalibrationLoader {
  public:
    bool LoadFromSettings(const std::string &settingsPath,
                          StereoCalibration &calibration) const override;
};

class DefaultStereoRectifier final : public Core::Ports::IStereoRectifier {
  public:
    bool EnsureRectifier(StereoCalibration &calibration,
                         const cv::Size &inputSize) const override;
    bool ApplyRectification(StereoCalibration &calibration,
                            const cv::Mat &leftGray, const cv::Mat &rightGray,
                            cv::Mat &leftRect, cv::Mat &rightRect) const override;
};

bool LoadStereoCalibrationFromSettings(const std::string &settingsPath,
                                       StereoCalibration &calibration);
bool EnsureStereoRectifier(StereoCalibration &calibration,
                           const cv::Size &inputSize);
bool ApplyStereoRectification(StereoCalibration &calibration,
                              const cv::Mat &leftGray, const cv::Mat &rightGray,
                              cv::Mat &leftRect, cv::Mat &rightRect);

} // namespace SmartDrone::Adapters::Slam
