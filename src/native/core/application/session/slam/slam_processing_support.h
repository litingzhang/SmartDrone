#pragma once

#include <chrono>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "core/domain/runtime_mode.h"
#include "core/ports/camera_provider.h"
#include "core/ports/pose_publisher.h"

namespace SmartDrone::Core::Application {

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

void PrepareStereoPairForSlam(const Ports::StereoFrame &stereo, const StereoFrameQuality &quality,
                              bool enableLowLightEnhance, Ports::StereoFrame &out);
uint8_t ToRuntimeSlamModeValue(SmartDrone::Core::Domain::SlamOperationMode mode);

class AutoSlamModeController {
  public:
    using SlamOperationMode = SmartDrone::Core::Domain::SlamOperationMode;
    using PoseQuality = SmartDrone::Core::Ports::PoseQuality;

    void Reset();
    SlamOperationMode EffectiveMode() const;
    SlamOperationMode Observe(bool trackingUsable, PoseQuality quality, double frameGapMs, size_t leftFeatureCount,
                              size_t rightFeatureCount);

  private:
    static constexpr int FRAMES_TO_LOCALIZATION = 12;
    static constexpr int FRAMES_TO_MAPPING = 4;
    static constexpr double STABLE_FRAME_GAP_MS = 75.0;
    static constexpr size_t STABLE_LEFT_FEATURES = 180;
    static constexpr size_t STABLE_RIGHT_FEATURES = 40;
    static constexpr size_t WEAK_LEFT_FEATURES = 80;
    static constexpr size_t WEAK_RIGHT_FEATURES = 20;

    SlamOperationMode m_effectiveMode{SlamOperationMode::Mapping};
    int m_stableFrames{0};
    int m_weakFrames{0};
};

} // namespace SmartDrone::Core::Application
