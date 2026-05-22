#include "adapters/slam/stereo_frame_preprocessor.h"

#include "adapters/slam/slam_image_utils.h"

namespace SmartDrone::Adapters::Slam {

bool PrepareStereoFrameForFrontend(const cv::Mat &leftImage,
                                   const cv::Mat &rightImage,
                                   PreparedStereoFrame &frame,
                                   StereoCalibration *calibration,
                                   bool rectify)
{
    frame = PreparedStereoFrame{};
    frame.leftGray = EnsureGray8(leftImage);
    frame.rightGray = EnsureGray8(rightImage);
    if (frame.leftGray.empty() || frame.rightGray.empty()) {
        return false;
    }

    frame.leftRect = frame.leftGray;
    frame.rightRect = frame.rightGray;
    if (rectify && calibration != nullptr && calibration->loaded) {
        frame.rectified =
            ApplyStereoRectification(*calibration, frame.leftGray, frame.rightGray,
                                     frame.leftRect, frame.rightRect);
    }
    return true;
}

bool DefaultStereoFramePreprocessor::PrepareForFrontend(
    const cv::Mat &leftImage, const cv::Mat &rightImage,
    PreparedStereoFrame &frame, StereoCalibration *calibration,
    bool rectify) const
{
    return PrepareStereoFrameForFrontend(leftImage, rightImage, frame,
                                         calibration, rectify);
}

} // namespace SmartDrone::Adapters::Slam
