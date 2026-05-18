#pragma once

#include <opencv2/core.hpp>

#include "adapters/slam/stereo_calibration.h"
#include "core/ports/stereo_processing.h"

namespace smartdrone::adapters::slam {

using PreparedStereoFrame = core::ports::PreparedStereoFrame;

class DefaultStereoFramePreprocessor final
    : public core::ports::IStereoFramePreprocessor {
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

} // namespace smartdrone::adapters::slam
