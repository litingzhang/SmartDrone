#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "core/ports/slam_engine.h"

namespace smartdrone::adapters::slam {

class ExternalFeatureFrontendClient;

struct LkStereoTrack {
    cv::Point2f left;
    cv::Point2f right;
    float quality{0.0f};
    uint32_t age{0};
};

struct LkLoopKeyframe {
    uint64_t frameId{0};
    Sophus::SE3f rawTwc{Sophus::SE3f()};
    Sophus::SE3f correctedTwc{Sophus::SE3f()};
    cv::Mat descriptor;
};

struct LkPerFrameVpiState;

struct SlamModeSharedState {
    SlamModeSharedState() = default;
    ~SlamModeSharedState();

    bool LoadStereoCalibration(const std::string &settingsPath);
    void EnsureStereoRectifier(const cv::Size &inputSize);
    void ResetTrackingState();
    void ResetExternalFeatureStats();
    void CopyExternalFeatureStatsToOutput(core::ports::SlamOutput &out) const;

    ExternalFeatureFrontendClient *m_externalFeatureFrontendClient{nullptr};
    int m_externalFeatureInputMaxWidth{640};
    int m_externalFeatureInputMaxHeight{400};
    mutable int m_lastSuperPointRawLeftCount{0};
    mutable int m_lastSuperPointRawRightCount{0};
    mutable int m_lastSuperPointMatchedStereoCount{0};
    mutable int m_lastSuperPointInjectedLeftCount{0};
    mutable int m_lastSuperPointInjectedRightCount{0};
    mutable double m_lastSuperPointPrepareMs{0.0};
    mutable double m_lastSuperPointInputMs{0.0};
    mutable double m_lastSuperPointForwardMs{0.0};
    mutable double m_lastSuperPointFrontendMs{0.0};
    mutable double m_lastSuperPointStereoMatchMs{0.0};
    mutable double m_lastSuperPointTotalMs{0.0};
    mutable uint32_t m_lastSuperPointImageCount{0};
    mutable uint32_t m_lastSuperPointPayloadBytes{0};
    mutable int m_superPointLightGlueOkStreak{0};
    mutable int m_superPointLightGlueLastEveryN{0};

    bool m_lkCalibrationLoaded{false};
    cv::Mat m_lkK1;
    cv::Mat m_lkD1;
    cv::Mat m_lkK2;
    cv::Mat m_lkD2;
    cv::Mat m_lkTc1c2;
    float m_lkFx{0.0f};
    float m_lkFy{0.0f};
    float m_lkCx{0.0f};
    float m_lkCy{0.0f};
    float m_lkBaseline{0.0f};
    mutable cv::Size m_lkRectifierSize{};
    mutable cv::Mat m_lkMap1x;
    mutable cv::Mat m_lkMap1y;
    mutable cv::Mat m_lkMap2x;
    mutable cv::Mat m_lkMap2y;
    mutable cv::Ptr<cv::StereoSGBM> m_lkPerFrameSgbm;
    mutable std::shared_ptr<LkPerFrameVpiState> m_lkPerFrameVpi;
    std::string m_lkPerFrameAcceleration{"cpu"};
    mutable bool m_lkPerFrameAccelLogged{false};
    mutable cv::Mat m_lkPrevLeft;
    mutable cv::Mat m_lkPrevRight;
    mutable std::vector<LkStereoTrack> m_lkTracks;
    mutable uint64_t m_lkLastSeedFrameId{0};
    mutable Sophus::SE3f m_lkTwc{Sophus::SE3f()};
    mutable Sophus::SE3f m_lkPerFrameReferenceTwc{Sophus::SE3f()};
    mutable bool m_lkHavePrev{false};
    mutable uint32_t m_lkFrameCount{0};
    bool m_lkLoopClosureEnabled{false};
    float m_lkLoopScale{1.20f};
    float m_lkLoopRelaxation{1.40f};
    mutable std::deque<LkLoopKeyframe> m_lkLoopKeyframes;
    mutable Sophus::SE3f m_lkLoopCorrection{Sophus::SE3f()};
    mutable uint64_t m_lkLastLoopClosureFrameId{0};
};

} // namespace smartdrone::adapters::slam
