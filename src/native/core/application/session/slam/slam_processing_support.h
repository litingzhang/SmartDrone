#pragma once

#include <chrono>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "core/domain/runtime_mode.h"
#include "core/ports/camera_provider.h"
#include "core/ports/pose_publisher.h"

namespace SmartDrone::core::application {

double DurationMs(const std::chrono::steady_clock::time_point &start, const std::chrono::steady_clock::time_point &end);
void ComputeImageStats(const cv::Mat &gray, double &meanOut, double &stddevOut);
double ComputeSharpnessLaplacianVar(const cv::Mat &gray);
std::vector<cv::Point2f> ComputeOrbDebugFeatures(const cv::Mat &gray);
bool ShouldEnhanceLowLightFrame(double mean, double stddev);
cv::Mat EnhanceLowLightGrayForSlam(const cv::Mat &gray);

struct StereoFrameQuality {
    double leftMean{0.0};
    double leftStddev{0.0};
    double rightMean{0.0};
    double rightStddev{0.0};
    double leftSharpness{0.0};
    double rightSharpness{0.0};
};

void PrepareStereoPairForSlam(const ports::StereoFrame &stereo, const StereoFrameQuality &quality,
                              bool enableLowLightEnhance, ports::StereoFrame &out);
uint8_t ToRuntimeSlamModeValue(SmartDrone::core::domain::SlamOperationMode mode);

class AutoSlamModeController {
  public:
    using SlamOperationMode = SmartDrone::core::domain::SlamOperationMode;
    using PoseQuality = SmartDrone::core::ports::PoseQuality;

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

} // namespace SmartDrone::core::application
