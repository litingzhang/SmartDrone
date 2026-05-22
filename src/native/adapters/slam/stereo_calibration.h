#pragma once

#include <string>

#include <opencv2/core.hpp>

#include "core/ports/stereo_processing.h"

namespace SmartDrone::adapters::slam {

using StereoCameraIntrinsics = core::ports::StereoCameraIntrinsics;
using StereoRectification = core::ports::StereoRectification;
using StereoCalibration = core::ports::StereoCalibration;

class DefaultStereoCalibrationLoader final
    : public core::ports::IStereoCalibrationLoader {
  public:
    bool LoadFromSettings(const std::string &settingsPath,
                          StereoCalibration &calibration) const override;
};

class DefaultStereoRectifier final : public core::ports::IStereoRectifier {
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

} // namespace SmartDrone::adapters::slam
