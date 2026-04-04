#pragma once

#include <opencv2/opencv.hpp>

#include <atomic>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "core/application/state/imu_buffer.h"
#include "core/application/config/runtime_app_types.h"
#include "core/domain/runtime_mode.h"
#include <sophus/se3.hpp>

namespace smartdrone::core::application {

namespace fs = std::filesystem;
using ControllerMode = smartdrone::core::domain::RuntimeMode;

struct StereoBodyExtrinsics {
    Sophus::SE3f Tbc{Sophus::SE3f()};
    bool loaded{false};
};

struct ImuThreadState {
    ImuBuffer imuBuffer;
    std::atomic<bool> imuOk{false};
    std::atomic<uint64_t> imuCnt{0};
    std::atomic<uint64_t> imuDrop{0};
    std::atomic<float> accelLsbPerG{0.0f};
    std::atomic<float> gyroLsbPerDps{0.0f};
};

constexpr int64_t kPointCloudUpdateIntervalNs = 200000000LL;

int ClampSlamInputFps(int requestedFps, int cameraFps);

struct ImuWindowValidation {
    size_t inputCount{0};
    size_t outputCount{0};
    size_t droppedNonFinite{0};
    size_t droppedNonMonotonic{0};
    size_t droppedOutOfRange{0};
    double largestGapSec{0.0};
    double firstLeadSec{0.0};
    double tailLagSec{0.0};
    const char* failureReason{nullptr};
};

bool IsFiniteImuPoint(const ORB_SLAM3::IMU::Point& p);
bool SanitizeImuWindow(
    std::vector<ORB_SLAM3::IMU::Point>& vImu,
    double prevFrameTime,
    double frameTime,
    double expectedImuDtSec,
    ImuWindowValidation& stats);
std::optional<Sophus::SE3f> ReadSe3Node(const cv::FileNode& node);
StereoBodyExtrinsics LoadStereoBodyExtrinsics(const std::string& settingsPath);
MainRuntimeAliases BuildRuntimeAliases(const AppConfig& c);
void PrintStartupConfig(const AppConfig& app, const MainRuntimeAliases& a, ControllerMode mode);

}  // namespace smartdrone::core::application
