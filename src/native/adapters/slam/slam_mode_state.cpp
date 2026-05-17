#include "adapters/slam/slam_mode_state.h"

#include <algorithm>
#include <iostream>
#include <utility>

#include "adapters/slam/stereo_frame_preprocessor.h"

#if defined(SMART_DRONE_ENABLE_ORB_SLAM3)
#include "adapters/slam/slam_engine_adapter.h"
#endif

namespace smartdrone::adapters::slam {

namespace {

void SyncLegacyStereoCalibrationFields(SlamModeSharedState &state)
{
    const StereoCalibration &calibration = state.m_stereoCalibration;
    state.m_lkCalibrationLoaded = calibration.loaded;
    state.m_lkK1 = calibration.left.K;
    state.m_lkK2 = calibration.right.K;
    state.m_lkD1 = calibration.left.D;
    state.m_lkD2 = calibration.right.D;
    state.m_lkTc1c2 = calibration.T_c1_c2;
    state.m_lkFx = calibration.left.fx;
    state.m_lkFy = calibration.left.fy;
    state.m_lkCx = calibration.left.cx;
    state.m_lkCy = calibration.left.cy;
    state.m_lkBaseline = calibration.baselineMeters;
    state.m_lkRectifierSize = calibration.rectification.imageSize;
    state.m_lkMap1x = calibration.rectification.leftMapX;
    state.m_lkMap1y = calibration.rectification.leftMapY;
    state.m_lkMap2x = calibration.rectification.rightMapX;
    state.m_lkMap2y = calibration.rectification.rightMapY;
}

} // namespace

SlamModeSharedState::~SlamModeSharedState() = default;

#if defined(SMART_DRONE_ENABLE_ORB_SLAM3)
void SlamEngineAdapter::SetExternalFeatureFrontendClient(ExternalFeatureFrontendClient *client)
{
    m_modeState->m_externalFeatureFrontendClient = client;
}

void SlamEngineAdapter::SetExternalFeatureInputSizeLimit(int maxWidth, int maxHeight)
{
    m_modeState->m_externalFeatureInputMaxWidth = std::max(0, maxWidth);
    m_modeState->m_externalFeatureInputMaxHeight = std::max(0, maxHeight);
}
#endif

bool SlamModeSharedState::LoadStereoCalibration(const std::string &settingsPath)
{
    if (!LoadStereoCalibrationFromSettings(settingsPath, m_stereoCalibration)) {
        m_lkCalibrationLoaded = false;
        std::cerr << "[stereo_vo] stereo calibration unavailable; stereo VO disabled\n";
        return false;
    }
    SyncLegacyStereoCalibrationFields(*this);
    std::cerr << "[stereo_vo] calibration loaded fx=" << m_lkFx << " fy=" << m_lkFy
              << " baseline=" << m_lkBaseline << "\n";
    return true;
}

void SlamModeSharedState::EnsureStereoRectifier(const cv::Size &inputSize)
{
    if (!m_lkCalibrationLoaded || inputSize.area() <= 0 || m_lkRectifierSize == inputSize) {
        return;
    }
    if (::smartdrone::adapters::slam::EnsureStereoRectifier(m_stereoCalibration, inputSize)) {
        SyncLegacyStereoCalibrationFields(*this);
    }
}

bool SlamModeSharedState::PrepareRectifiedStereoCpu(const cv::Mat &leftImage, const cv::Mat &rightImage,
                                                    cv::Mat &leftRect, cv::Mat &rightRect)
{
    PreparedStereoFrame frame;
    if (!PrepareStereoFrameForFrontend(leftImage, rightImage, frame, &m_stereoCalibration, m_lkCalibrationLoaded)) {
        leftRect.release();
        rightRect.release();
        return false;
    }
    SyncLegacyStereoCalibrationFields(*this);
    leftRect = std::move(frame.leftRect);
    rightRect = std::move(frame.rightRect);
    return true;
}

void SlamModeSharedState::ResetTrackingState()
{
    m_superPointLightGlueOkStreak = 0;
    m_superPointLightGlueLastEveryN = 0;
    m_superPointLightGlueBootstrapTrustFrames = 0;
    m_superPointLightGlueBootstrapTrustClosed = false;
    m_lastSlamMatchesInliers = 0;
    m_lastSlamTrackedMapPoints = 0;
    m_spLgPrevLeft.release();
    m_spLgPrevRight.release();
    m_spLgPrevLeftPoints.clear();
    m_spLgPrevRightPoints.clear();
    m_spLgHavePrevStereo = false;
    m_lkPrevLeft.release();
    m_lkPrevRight.release();
    m_lkPerFrameSgbm = nullptr;
    m_lkPerFrameVpi.reset();
    m_lkTracks.clear();
    m_lkLastSeedFrameId = 0;
    m_lkTwc = Sophus::SE3f();
    m_lkPerFrameReferenceTwc = Sophus::SE3f();
    m_lkHavePrev = false;
    m_lkFrameCount = 0;
    m_lkLoopKeyframes.clear();
    m_lkLoopCorrection = Sophus::SE3f();
    m_lkLastLoopClosureFrameId = 0;
}

void SlamModeSharedState::ResetExternalFeatureStats()
{
    m_lastSuperPointRawLeftCount = 0;
    m_lastSuperPointRawRightCount = 0;
    m_lastSuperPointMatchedStereoCount = 0;
    m_lastSuperPointInjectedLeftCount = 0;
    m_lastSuperPointInjectedRightCount = 0;
    m_lastSuperPointExternalHash = 0;
    m_lastSuperPointPrepareMs = 0.0;
    m_lastSuperPointInputMs = 0.0;
    m_lastSuperPointForwardMs = 0.0;
    m_lastSuperPointFrontendMs = 0.0;
    m_lastSuperPointStereoMatchMs = 0.0;
    m_lastSuperPointTotalMs = 0.0;
    m_lastSuperPointImageCount = 0;
    m_lastSuperPointPayloadBytes = 0;
}

void SlamModeSharedState::CopyExternalFeatureStatsToOutput(core::ports::SlamOutput &out) const
{
    out.superpointRawLeftCount = m_lastSuperPointRawLeftCount;
    out.superpointRawRightCount = m_lastSuperPointRawRightCount;
    out.superpointMatchedStereoCount = m_lastSuperPointMatchedStereoCount;
    out.superpointInjectedLeftCount = m_lastSuperPointInjectedLeftCount;
    out.superpointInjectedRightCount = m_lastSuperPointInjectedRightCount;
    out.superpointExternalHash = m_lastSuperPointExternalHash;
    out.superpointLightGlueEveryN = m_superPointLightGlueLastEveryN;
    out.superpointPrepareMs = m_lastSuperPointPrepareMs;
    out.superpointInputMs = m_lastSuperPointInputMs;
    out.superpointForwardMs = m_lastSuperPointForwardMs;
    out.superpointFrontendMs = m_lastSuperPointFrontendMs;
    out.superpointStereoMatchMs = m_lastSuperPointStereoMatchMs;
    out.superpointTotalMs = m_lastSuperPointTotalMs;
    out.superpointImageCount = m_lastSuperPointImageCount;
    out.superpointPayloadBytes = m_lastSuperPointPayloadBytes;
}

} // namespace smartdrone::adapters::slam
