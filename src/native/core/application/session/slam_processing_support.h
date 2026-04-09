#pragma once

#include <chrono>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "System.h"
#include "common/tlv/tlv_protocol.h"
#include "core/domain/runtime_mode.h"
#include "core/ports/camera_provider.h"
#include "core/ports/imu_provider.h"
#include "core/ports/pose_publisher.h"

namespace smartdrone::core::application {

double DurationMs(const std::chrono::steady_clock::time_point &start, const std::chrono::steady_clock::time_point &end);
void ComputeImageStats(const cv::Mat &gray, double &meanOut, double &stddevOut);
double ComputeSharpnessLaplacianVar(const cv::Mat &gray);
std::vector<cv::Point2f> ComputeOrbDebugFeatures(const cv::Mat &gray);
bool ShouldEnhanceLowLightFrame(double mean, double stddev);
cv::Mat EnhanceLowLightGrayForSlam(const cv::Mat &gray);
void PrepareStereoPairForSlam(const ports::StereoFrame &stereo, double meanL, double stdL, double meanR, double stdR,
                              double sharpL, double sharpR, bool enableLowLightEnhance, ports::StereoFrame &out);
bool IsTrackingPoseUsable(int trackingState);
uint8_t ToRuntimeSlamModeValue(smartdrone::core::domain::SlamOperationMode mode);

class AutoSlamModeController {
  public:
    using SlamOperationMode = smartdrone::core::domain::SlamOperationMode;
    using PoseQuality = smartdrone::core::ports::PoseQuality;

    void Reset();
    SlamOperationMode EffectiveMode() const;
    SlamOperationMode Observe(bool trackingUsable, PoseQuality quality, double frameGapMs, size_t leftFeatureCount,
                              size_t rightFeatureCount);

  private:
    static constexpr int kFramesToLocalization = 12;
    static constexpr int kFramesToMapping = 4;
    static constexpr double kStableFrameGapMs = 75.0;
    static constexpr size_t kStableLeftFeatures = 180;
    static constexpr size_t kStableRightFeatures = 40;
    static constexpr size_t kWeakLeftFeatures = 80;
    static constexpr size_t kWeakRightFeatures = 20;

    SlamOperationMode m_effectiveMode{SlamOperationMode::Mapping};
    int m_stableFrames{0};
    int m_weakFrames{0};
};

std::vector<ORB_SLAM3::IMU::Point> ToOrbImuPoints(const std::vector<smartdrone::core::ports::ImuReading> &readings);

} // namespace smartdrone::core::application
