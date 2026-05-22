#include "core/application/session/slam/slam_processing_support.h"

#include <algorithm>
#include <cmath>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "common/tlv/tlv_protocol.h"

namespace SmartDrone::core::application {

double DurationMs(const std::chrono::steady_clock::time_point &start, const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

void ComputeImageStats(const cv::Mat &gray, double &meanOut, double &stddevOut)
{
    meanOut = 0.0;
    stddevOut = 0.0;
    if (gray.empty()) {
        return;
    }
    cv::Scalar mean{};
    cv::Scalar stddev{};
    cv::meanStdDev(gray, mean, stddev);
    meanOut = mean[0];
    stddevOut = stddev[0];
}

double ComputeSharpnessLaplacianVar(const cv::Mat &gray)
{
    if (gray.empty()) {
        return 0.0;
    }
    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar mean{};
    cv::Scalar stddev{};
    cv::meanStdDev(laplacian, mean, stddev);
    return stddev[0] * stddev[0];
}

std::vector<cv::Point2f> ComputeOrbDebugFeatures(const cv::Mat &gray)
{
    std::vector<cv::Point2f> points;
    if (gray.empty()) {
        return points;
    }

    std::vector<cv::KeyPoint> keypoints;
    auto orb = cv::ORB::create(1000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
    orb->detect(gray, keypoints);

    points.reserve(keypoints.size());
    for (const auto &keypoint : keypoints) {
        points.push_back(keypoint.pt);
    }
    return points;
}

bool ShouldEnhanceLowLightFrame(double mean, double stddev)
{
    return mean < 35.0 || (mean < 50.0 && stddev < 18.0);
}

namespace {

cv::Mat ApplyGammaU8(const cv::Mat &gray, double gamma)
{
    cv::Mat out;
    if (gray.empty()) {
        return out;
    }
    if (gamma <= 0.0 || std::abs(gamma - 1.0) < 1e-3) {
        gray.copyTo(out);
        return out;
    }

    cv::Mat lut(1, 256, CV_8UC1);
    uint8_t *lutPtr = lut.ptr<uint8_t>(0);
    const double invGamma = 1.0 / gamma;
    for (int i = 0; i < 256; ++i) {
        const double normalized = static_cast<double>(i) / 255.0;
        const double corrected = std::pow(normalized, invGamma);
        lutPtr[i] = static_cast<uint8_t>(std::clamp<int>(static_cast<int>(std::lround(corrected * 255.0)), 0, 255));
    }
    cv::LUT(gray, lut, out);
    return out;
}

bool IsLowTextureFrame(double stddev, double sharpness)
{
    return stddev < 14.0 || sharpness < 95.0;
}

} // namespace

cv::Mat EnhanceLowLightGrayForSlam(const cv::Mat &gray)
{
    if (gray.empty()) {
        return gray;
    }

    cv::Mat enhanced = gray.clone();
    auto clahe = cv::createCLAHE(2.5, cv::Size(8, 8));
    clahe->apply(enhanced, enhanced);
    return enhanced;
}

void PrepareStereoPairForSlam(const ports::StereoFrame &stereo, const StereoFrameQuality &quality,
                              bool enableLowLightEnhance, ports::StereoFrame &out)
{
    out = stereo;
    if (!enableLowLightEnhance) {
        return;
    }

    const bool lowLightL = ShouldEnhanceLowLightFrame(quality.leftMean, quality.leftStddev);
    const bool lowLightR = ShouldEnhanceLowLightFrame(quality.rightMean, quality.rightStddev);
    const bool lowTextureL = IsLowTextureFrame(quality.leftStddev, quality.leftSharpness);
    const bool lowTextureR = IsLowTextureFrame(quality.rightStddev, quality.rightSharpness);
    const bool shouldEnhance = lowLightL || lowLightR || (lowTextureL && lowTextureR);
    if (!shouldEnhance) {
        return;
    }

    // Keep left/right photometric transform consistent to avoid stereo matcher drift.
    const bool severeLowLight = (quality.leftMean < 25.0 || quality.rightMean < 25.0);
    const double gamma = severeLowLight ? 1.45 : 1.20;
    const double clipLimit = severeLowLight ? 3.0 : 2.3;

    auto processOne = [gamma, clipLimit](const cv::Mat &in) {
        cv::Mat gammaOut = ApplyGammaU8(in, gamma);
        cv::Mat claheOut;
        auto clahe = cv::createCLAHE(clipLimit, cv::Size(8, 8));
        clahe->apply(gammaOut, claheOut);
        return claheOut;
    };

    out.left.gray = processOne(stereo.left.gray);
    out.left.owner.reset();
    out.right.gray = processOne(stereo.right.gray);
    out.right.owner.reset();
}

uint8_t ToRuntimeSlamModeValue(SmartDrone::core::domain::SlamOperationMode mode)
{
    switch (mode) {
    case SmartDrone::core::domain::SlamOperationMode::Localization:
        return RUNTIME_SLAM_MODE_LOCALIZATION;
    case SmartDrone::core::domain::SlamOperationMode::Relocalization:
        return RUNTIME_SLAM_MODE_RELOCALIZATION;
    case SmartDrone::core::domain::SlamOperationMode::TrackingOnly:
        return RUNTIME_SLAM_MODE_TRACKING_ONLY;
    case SmartDrone::core::domain::SlamOperationMode::Auto:
        return RUNTIME_SLAM_MODE_AUTO;
    case SmartDrone::core::domain::SlamOperationMode::Mapping:
    default:
        return RUNTIME_SLAM_MODE_MAPPING;
    }
}

void AutoSlamModeController::Reset()
{
    m_effectiveMode = SlamOperationMode::Mapping;
    m_stableFrames = 0;
    m_weakFrames = 0;
}

AutoSlamModeController::SlamOperationMode AutoSlamModeController::EffectiveMode() const
{
    return m_effectiveMode;
}

AutoSlamModeController::SlamOperationMode AutoSlamModeController::Observe(bool trackingUsable, PoseQuality quality,
                                                                          double frameGapMs, size_t leftFeatureCount,
                                                                          size_t rightFeatureCount)
{
    const bool frameGapAcceptable = frameGapMs <= 0.0 || frameGapMs <= kStableFrameGapMs;
    const bool featuresStable = leftFeatureCount >= kStableLeftFeatures && rightFeatureCount >= kStableRightFeatures;
    const bool featuresWeak = leftFeatureCount < kWeakLeftFeatures || rightFeatureCount < kWeakRightFeatures;
    const bool stableNow = trackingUsable && quality != PoseQuality::Lost && frameGapAcceptable && featuresStable;
    const bool weakNow = !trackingUsable || quality == PoseQuality::Lost || !frameGapAcceptable || featuresWeak;

    if (stableNow) {
        ++m_stableFrames;
        m_weakFrames = 0;
    } else if (weakNow) {
        ++m_weakFrames;
        m_stableFrames = 0;
    }

    if (m_effectiveMode == SlamOperationMode::Mapping && m_stableFrames >= kFramesToLocalization) {
        m_effectiveMode = SlamOperationMode::Localization;
        m_stableFrames = 0;
    } else if (m_effectiveMode == SlamOperationMode::Localization && m_weakFrames >= kFramesToMapping) {
        m_effectiveMode = SlamOperationMode::Mapping;
        m_weakFrames = 0;
    }
    return m_effectiveMode;
}

} // namespace SmartDrone::core::application
