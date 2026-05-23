#pragma once

#include <opencv2/core.hpp>

#include "adapters/slam/stereo/stereo_calibration.h"
#include "core/ports/stereo_processing.h"

namespace SmartDrone::Adapters::Slam {

using PreparedStereoFrame = Core::Ports::PreparedStereoFrame;

class DefaultStereoFramePreprocessor final
    : public Core::Ports::IStereoFramePreprocessor {
  public:
    bool PrepareForFrontend(const cv::Mat &leftImage, const cv::Mat &rightImage,
                            PreparedStereoFrame &frame,
                            StereoCalibration *calibration,
                            bool rectify) const override;
};

bool PrepareStereoFrameForFrontend(const cv::Mat &leftImage,
                                   const cv::Mat &rightImage,
                                   PreparedStereoFrame &frame,
                                   StereoCalibration *calibration,
                                   bool rectify);

} // namespace SmartDrone::Adapters::Slam
