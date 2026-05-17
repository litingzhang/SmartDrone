#pragma once

#include <string>

#include <opencv2/core.hpp>

namespace smartdrone::adapters::slam {

struct StereoCameraIntrinsics {
    float fx{0.0f};
    float fy{0.0f};
    float cx{0.0f};
    float cy{0.0f};
    cv::Mat K;
    cv::Mat D;
};

struct StereoRectification {
    cv::Size imageSize{};
    cv::Mat leftMapX;
    cv::Mat leftMapY;
    cv::Mat rightMapX;
    cv::Mat rightMapY;
};

struct StereoCalibration {
    StereoCameraIntrinsics left;
    StereoCameraIntrinsics right;
    cv::Mat T_c1_c2;
    float baselineMeters{0.0f};
    bool loaded{false};
    mutable StereoRectification rectification;
};

bool LoadStereoCalibrationFromSettings(const std::string &settingsPath, StereoCalibration &calibration);
bool EnsureStereoRectifier(StereoCalibration &calibration, const cv::Size &inputSize);
bool ApplyStereoRectification(StereoCalibration &calibration, const cv::Mat &leftGray,
                              const cv::Mat &rightGray, cv::Mat &leftRect, cv::Mat &rightRect);

} // namespace smartdrone::adapters::slam
