#include "adapters/slam/slam_mode_state.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "adapters/slam/slam_engine_adapter.h"
#include "adapters/slam/slam_mode_common.h"

namespace smartdrone::adapters::slam {

SlamModeSharedState::~SlamModeSharedState() = default;

void SlamEngineAdapter::SetExternalFeatureFrontendClient(ExternalFeatureFrontendClient *client)
{
    m_modeState->m_externalFeatureFrontendClient = client;
}

void SlamEngineAdapter::SetExternalFeatureInputSizeLimit(int maxWidth, int maxHeight)
{
    m_modeState->m_externalFeatureInputMaxWidth = std::max(0, maxWidth);
    m_modeState->m_externalFeatureInputMaxHeight = std::max(0, maxHeight);
}

bool SlamModeSharedState::LoadStereoCalibration(const std::string &settingsPath)
{
    if (settingsPath.empty()) {
        std::cerr << "[stereo_vo] settings path empty; stereo VO disabled\n";
        return false;
    }
    cv::FileStorage fs(settingsPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[stereo_vo] failed to open settings: " << settingsPath << "\n";
        return false;
    }

    const float fx1 = static_cast<float>(fs["Camera1.fx"]);
    const float fy1 = static_cast<float>(fs["Camera1.fy"]);
    const float cx1 = static_cast<float>(fs["Camera1.cx"]);
    const float cy1 = static_cast<float>(fs["Camera1.cy"]);
    const float fx2 = static_cast<float>(fs["Camera2.fx"]);
    const float fy2 = static_cast<float>(fs["Camera2.fy"]);
    const float cx2 = static_cast<float>(fs["Camera2.cx"]);
    const float cy2 = static_cast<float>(fs["Camera2.cy"]);
    if (!(fx1 > 0.0f) || !(fy1 > 0.0f) || !(fx2 > 0.0f) || !(fy2 > 0.0f)) {
        std::cerr << "[stereo_vo] invalid camera intrinsics in settings\n";
        return false;
    }

    m_lkK1 = MakeCameraMatrix(fx1, fy1, cx1, cy1);
    m_lkK2 = MakeCameraMatrix(fx2, fy2, cx2, cy2);
    m_lkD1 = MakeDistCoeffs(static_cast<float>(fs["Camera1.k1"]), static_cast<float>(fs["Camera1.k2"]),
                            static_cast<float>(fs["Camera1.p1"]), static_cast<float>(fs["Camera1.p2"]));
    m_lkD2 = MakeDistCoeffs(static_cast<float>(fs["Camera2.k1"]), static_cast<float>(fs["Camera2.k2"]),
                            static_cast<float>(fs["Camera2.p1"]), static_cast<float>(fs["Camera2.p2"]));
    fs["Stereo.T_c1_c2"] >> m_lkTc1c2;
    if (m_lkTc1c2.empty() || m_lkTc1c2.rows != 4 || m_lkTc1c2.cols != 4) {
        std::cerr << "[stereo_vo] Stereo.T_c1_c2 missing; falling back to Camera.bf baseline\n";
    }
    const float bf = static_cast<float>(fs["Camera.bf"]);
    m_lkBaseline = bf > 0.0f ? bf / fx1 : 0.0f;
    if (!m_lkTc1c2.empty()) {
        cv::Mat T64;
        m_lkTc1c2.convertTo(T64, CV_64F);
        m_lkBaseline = std::abs(static_cast<float>(T64.at<double>(0, 3)));
    }
    if (!(m_lkBaseline > 0.005f)) {
        std::cerr << "[stereo_vo] invalid stereo baseline\n";
        return false;
    }
    m_lkFx = fx1;
    m_lkFy = fy1;
    m_lkCx = cx1;
    m_lkCy = cy1;
    m_lkCalibrationLoaded = true;
    std::cerr << "[stereo_vo] calibration loaded fx=" << m_lkFx << " fy=" << m_lkFy
              << " baseline=" << m_lkBaseline << "\n";
    return true;
}

void SlamModeSharedState::EnsureStereoRectifier(const cv::Size &inputSize)
{
    if (!m_lkCalibrationLoaded || inputSize.area() <= 0 || m_lkRectifierSize == inputSize) {
        return;
    }

    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t = (cv::Mat_<double>(3, 1) << -static_cast<double>(m_lkBaseline), 0.0, 0.0);
    if (!m_lkTc1c2.empty()) {
        cv::Mat T64;
        m_lkTc1c2.convertTo(T64, CV_64F);
        cv::Mat Tlr = T64.inv();
        R = Tlr(cv::Rect(0, 0, 3, 3)).clone();
        t = Tlr(cv::Rect(3, 0, 1, 3)).clone();
    }

    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(m_lkK1, m_lkD1, m_lkK2, m_lkD2, inputSize, R, t, R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, -1.0, inputSize);
    cv::initUndistortRectifyMap(m_lkK1, m_lkD1, R1, P1, inputSize, CV_32FC1, m_lkMap1x, m_lkMap1y);
    cv::initUndistortRectifyMap(m_lkK2, m_lkD2, R2, P2, inputSize, CV_32FC1, m_lkMap2x, m_lkMap2y);
    m_lkFx = static_cast<float>(P1.at<double>(0, 0));
    m_lkFy = static_cast<float>(P1.at<double>(1, 1));
    m_lkCx = static_cast<float>(P1.at<double>(0, 2));
    m_lkCy = static_cast<float>(P1.at<double>(1, 2));
    m_lkBaseline = std::abs(static_cast<float>(P2.at<double>(0, 3) / P2.at<double>(0, 0)));
    m_lkRectifierSize = inputSize;
}

void SlamModeSharedState::ResetTrackingState()
{
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
