#include "adapters/slam/slam_mode_state.h"

#include <algorithm>
#include <iostream>
#include <utility>

#include "adapters/slam/feature_point_tracking.h"
#include "adapters/slam/klt_loop_closure_backend.h"
#include "adapters/slam/klt_pnp_observation_builder.h"
#include "adapters/slam/stereo_feature_packet.h"
#include "adapters/slam/stereo_frame_preprocessor.h"
#include "adapters/slam/stereo_matching.h"
#include "adapters/slam/stereo_pair_builder.h"
#include "adapters/slam/temporal_stereo.h"
#include "adapters/slam/visual_pnp_pose_backend.h"

namespace SmartDrone::Adapters::Slam {

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

bool SlamModeSharedState::LoadStereoCalibration(
    const std::string &settingsPath)
{
    if (!StereoCalibrationLoader().LoadFromSettings(settingsPath,
                                                    m_stereoCalibration)) {
        m_lkCalibrationLoaded = false;
        std::cerr
            << "[stereo_vo] stereo calibration unavailable; stereo VO disabled\n";
        return false;
    }
    SyncLegacyStereoCalibrationFields(*this);
    std::cerr << "[stereo_vo] calibration loaded fx=" << m_lkFx
              << " fy=" << m_lkFy << " baseline=" << m_lkBaseline << "\n";
    return true;
}

void SlamModeSharedState::EnsureStereoRectifier(const cv::Size &inputSize)
{
    if (!m_lkCalibrationLoaded || inputSize.area() <= 0 ||
        m_lkRectifierSize == inputSize) {
        return;
    }
    if (StereoRectifier().EnsureRectifier(m_stereoCalibration, inputSize)) {
        SyncLegacyStereoCalibrationFields(*this);
    }
}

bool SlamModeSharedState::PrepareRectifiedStereoCpu(const cv::Mat &leftImage,
                                                    const cv::Mat &rightImage,
                                                    cv::Mat &leftRect,
                                                    cv::Mat &rightRect)
{
    PreparedStereoFrame frame;
    if (!StereoFramePreprocessor().PrepareForFrontend(leftImage, rightImage,
                                                      frame, &m_stereoCalibration,
                                                      m_lkCalibrationLoaded)) {
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
    m_visualFeatureLightGlueOkStreak = 0;
    m_visualFeatureLightGlueLastEveryN = 0;
    m_visualFeatureLightGlueBootstrapTrustFrames = 0;
    m_visualFeatureLightGlueBootstrapTrustClosed = false;
    m_lastSlamMatchesInliers = 0;
    m_lastSlamTrackedMapPoints = 0;
    m_visualFeatureTemporalPrevLeft.release();
    m_visualFeatureTemporalPrevRight.release();
    m_visualFeatureTemporalPrevLeftPoints.clear();
    m_visualFeatureTemporalPrevRightPoints.clear();
    m_visualFeatureTemporalHavePrevStereo = false;
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
    m_lkLoop.keyframes.clear();
    m_lkLoop.correction = Sophus::SE3f();
    m_lkLoop.lastClosureFrameId = 0;
}

void SlamModeSharedState::ResetVisualFeatureStats()
{
    m_lastVisualFeatureRawLeftCount = 0;
    m_lastVisualFeatureRawRightCount = 0;
    m_lastVisualFeatureMatchedStereoCount = 0;
    m_lastVisualFeatureInjectedLeftCount = 0;
    m_lastVisualFeatureInjectedRightCount = 0;
    m_lastVisualFeatureObservationHash = 0;
    m_lastVisualFeaturePrepareMs = 0.0;
    m_lastVisualFeatureInputMs = 0.0;
    m_lastVisualFeatureForwardMs = 0.0;
    m_lastVisualFeatureFrontendMs = 0.0;
    m_lastVisualFeatureStereoMatchMs = 0.0;
    m_lastVisualFeatureTotalMs = 0.0;
    m_lastVisualFeatureImageCount = 0;
    m_lastVisualFeaturePayloadBytes = 0;
}

void SlamModeSharedState::CopyVisualFeatureStatsToOutput(
    Core::Ports::SlamOutput &out) const
{
    out.visualFeatureRawLeftCount = m_lastVisualFeatureRawLeftCount;
    out.visualFeatureRawRightCount = m_lastVisualFeatureRawRightCount;
    out.visualFeatureMatchedStereoCount = m_lastVisualFeatureMatchedStereoCount;
    out.visualFeatureInjectedLeftCount = m_lastVisualFeatureInjectedLeftCount;
    out.visualFeatureInjectedRightCount = m_lastVisualFeatureInjectedRightCount;
    out.visualFeatureObservationHash = m_lastVisualFeatureObservationHash;
    out.visualFeatureMatchEveryN = m_visualFeatureLightGlueLastEveryN;
    out.visualFeaturePrepareMs = m_lastVisualFeaturePrepareMs;
    out.visualFeatureInputMs = m_lastVisualFeatureInputMs;
    out.visualFeatureForwardMs = m_lastVisualFeatureForwardMs;
    out.visualFeatureFrontendMs = m_lastVisualFeatureFrontendMs;
    out.visualFeatureStereoMatchMs = m_lastVisualFeatureStereoMatchMs;
    out.visualFeatureTotalMs = m_lastVisualFeatureTotalMs;
    out.visualFeatureImageCount = m_lastVisualFeatureImageCount;
    out.visualFeaturePayloadBytes = m_lastVisualFeaturePayloadBytes;
}

Core::Ports::IStereoCalibrationLoader &
SlamModeSharedState::StereoCalibrationLoader()
{
    if (!m_stereoCalibrationLoader) {
        m_stereoCalibrationLoader =
            std::make_unique<DefaultStereoCalibrationLoader>();
    }
    return *m_stereoCalibrationLoader;
}

const Core::Ports::IStereoCalibrationLoader &
SlamModeSharedState::StereoCalibrationLoader() const
{
    return const_cast<SlamModeSharedState *>(this)->StereoCalibrationLoader();
}

Core::Ports::IStereoRectifier &SlamModeSharedState::StereoRectifier()
{
    if (!m_stereoRectifier) {
        m_stereoRectifier = std::make_unique<DefaultStereoRectifier>();
    }
    return *m_stereoRectifier;
}

const Core::Ports::IStereoRectifier &
SlamModeSharedState::StereoRectifier() const
{
    return const_cast<SlamModeSharedState *>(this)->StereoRectifier();
}

Core::Ports::IStereoFramePreprocessor &
SlamModeSharedState::StereoFramePreprocessor()
{
    if (!m_stereoFramePreprocessor) {
        m_stereoFramePreprocessor =
            std::make_unique<DefaultStereoFramePreprocessor>();
    }
    return *m_stereoFramePreprocessor;
}

const Core::Ports::IStereoFramePreprocessor &
SlamModeSharedState::StereoFramePreprocessor() const
{
    return const_cast<SlamModeSharedState *>(this)->StereoFramePreprocessor();
}

Core::Ports::IStereoPairBuilder &SlamModeSharedState::StereoPairBuilder()
{
    if (!m_stereoPairBuilder) {
        m_stereoPairBuilder = std::make_unique<DefaultStereoPairBuilder>();
    }
    return *m_stereoPairBuilder;
}

const Core::Ports::IStereoPairBuilder &
SlamModeSharedState::StereoPairBuilder() const
{
    return const_cast<SlamModeSharedState *>(this)->StereoPairBuilder();
}

Core::Ports::IStereoMatchSelector &SlamModeSharedState::StereoMatchSelector()
{
    if (!m_stereoMatchSelector) {
        m_stereoMatchSelector = std::make_unique<DefaultStereoMatchSelector>();
    }
    return *m_stereoMatchSelector;
}

const Core::Ports::IStereoMatchSelector &
SlamModeSharedState::StereoMatchSelector() const
{
    return const_cast<SlamModeSharedState *>(this)->StereoMatchSelector();
}

Core::Ports::ITemporalStereoProcessor &
SlamModeSharedState::TemporalStereoProcessor()
{
    if (!m_temporalStereoProcessor) {
        m_temporalStereoProcessor =
            std::make_unique<DefaultTemporalStereoProcessor>();
    }
    return *m_temporalStereoProcessor;
}

const Core::Ports::ITemporalStereoProcessor &
SlamModeSharedState::TemporalStereoProcessor() const
{
    return const_cast<SlamModeSharedState *>(this)->TemporalStereoProcessor();
}

Core::Ports::IStereoFeaturePacketBuilder &
SlamModeSharedState::StereoFeaturePacketBuilder()
{
    if (!m_stereoFeaturePacketBuilder) {
        m_stereoFeaturePacketBuilder =
            std::make_unique<DefaultStereoFeaturePacketBuilder>();
    }
    return *m_stereoFeaturePacketBuilder;
}

const Core::Ports::IStereoFeaturePacketBuilder &
SlamModeSharedState::StereoFeaturePacketBuilder() const
{
    return const_cast<SlamModeSharedState *>(this)->StereoFeaturePacketBuilder();
}

void SlamModeSharedState::SetStereoMatchSelector(
    std::unique_ptr<Core::Ports::IStereoMatchSelector> selector)
{
    m_stereoMatchSelector = std::move(selector);
}

void SlamModeSharedState::SetTemporalStereoProcessor(
    std::unique_ptr<Core::Ports::ITemporalStereoProcessor> processor)
{
    m_temporalStereoProcessor = std::move(processor);
}

void SlamModeSharedState::SetStereoFeaturePacketBuilder(
    std::unique_ptr<Core::Ports::IStereoFeaturePacketBuilder> builder)
{
    m_stereoFeaturePacketBuilder = std::move(builder);
}

void SlamModeSharedState::SetStereoCalibrationLoader(
    std::unique_ptr<Core::Ports::IStereoCalibrationLoader> loader)
{
    m_stereoCalibrationLoader = std::move(loader);
}

void SlamModeSharedState::SetStereoRectifier(
    std::unique_ptr<Core::Ports::IStereoRectifier> rectifier)
{
    m_stereoRectifier = std::move(rectifier);
}

void SlamModeSharedState::SetStereoFramePreprocessor(
    std::unique_ptr<Core::Ports::IStereoFramePreprocessor> preprocessor)
{
    m_stereoFramePreprocessor = std::move(preprocessor);
}

void SlamModeSharedState::SetStereoPairBuilder(
    std::unique_ptr<Core::Ports::IStereoPairBuilder> builder)
{
    m_stereoPairBuilder = std::move(builder);
}

Core::Ports::IVisualPnpObservationBuilder &
SlamModeSharedState::VisualPnpObservationBuilder()
{
    if (!m_visualPnpObservationBuilder) {
        m_visualPnpObservationBuilder =
            std::make_unique<DefaultVisualPnpObservationBuilder>();
    }
    return *m_visualPnpObservationBuilder;
}

const Core::Ports::IVisualPnpObservationBuilder &
SlamModeSharedState::VisualPnpObservationBuilder() const
{
    return const_cast<SlamModeSharedState *>(this)->VisualPnpObservationBuilder();
}

Core::Ports::IVisualPnpPoseBackend &
SlamModeSharedState::VisualPnpPoseBackend()
{
    if (!m_visualPnpPoseBackend) {
        m_visualPnpPoseBackend = std::make_unique<DefaultVisualPnpPoseBackend>();
    }
    return *m_visualPnpPoseBackend;
}

const Core::Ports::IVisualPnpPoseBackend &
SlamModeSharedState::VisualPnpPoseBackend() const
{
    return const_cast<SlamModeSharedState *>(this)->VisualPnpPoseBackend();
}

void SlamModeSharedState::SetVisualPnpObservationBuilder(
    std::unique_ptr<Core::Ports::IVisualPnpObservationBuilder> builder)
{
    m_visualPnpObservationBuilder = std::move(builder);
}

void SlamModeSharedState::SetVisualPnpPoseBackend(
    std::unique_ptr<Core::Ports::IVisualPnpPoseBackend> backend)
{
    m_visualPnpPoseBackend = std::move(backend);
}

Core::Ports::IPointTracker2d &SlamModeSharedState::PointTracker2d()
{
    if (!m_pointTracker2d) {
        m_pointTracker2d = std::make_unique<DefaultPointTracker2d>();
    }
    return *m_pointTracker2d;
}

const Core::Ports::IPointTracker2d &
SlamModeSharedState::PointTracker2d() const
{
    return const_cast<SlamModeSharedState *>(this)->PointTracker2d();
}

Core::Ports::IVisualLoopClosureBackend &
SlamModeSharedState::VisualLoopClosureBackend()
{
    if (!m_visualLoopClosureBackend) {
        m_visualLoopClosureBackend =
            std::make_unique<DefaultVisualLoopClosureBackend>();
    }
    return *m_visualLoopClosureBackend;
}

const Core::Ports::IVisualLoopClosureBackend &
SlamModeSharedState::VisualLoopClosureBackend() const
{
    return const_cast<SlamModeSharedState *>(this)->VisualLoopClosureBackend();
}

void SlamModeSharedState::SetPointTracker2d(
    std::unique_ptr<Core::Ports::IPointTracker2d> tracker)
{
    m_pointTracker2d = std::move(tracker);
}

void SlamModeSharedState::SetVisualLoopClosureBackend(
    std::unique_ptr<Core::Ports::IVisualLoopClosureBackend> backend)
{
    m_visualLoopClosureBackend = std::move(backend);
}

} // namespace SmartDrone::Adapters::Slam
