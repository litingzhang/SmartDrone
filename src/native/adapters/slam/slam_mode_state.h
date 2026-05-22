#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "adapters/slam/stereo_calibration.h"
#include "core/ports/slam_engine.h"
#include "core/ports/stereo_processing.h"
#include "core/ports/visual_feature_frontend.h"
#include "core/ports/visual_pose_backend.h"
#include "core/ports/visual_tracking.h"

namespace SmartDrone::adapters::slam {

using LkStereoTrack = core::ports::StereoTrack;
using LkLoopKeyframe = core::ports::LoopKeyframe;
using LkLoopClosureState = core::ports::LoopClosureState;

struct LkPerFrameVpiState;

struct SlamModeSharedState {
    SlamModeSharedState() = default;
    ~SlamModeSharedState();

    bool LoadStereoCalibration(const std::string &settingsPath);
    void EnsureStereoRectifier(const cv::Size &inputSize);
    bool PrepareRectifiedStereoCpu(const cv::Mat &leftImage,
                                   const cv::Mat &rightImage, cv::Mat &leftRect,
                                   cv::Mat &rightRect);
    void ResetTrackingState();
    void ResetVisualFeatureStats();
    void CopyVisualFeatureStatsToOutput(core::ports::SlamOutput &out) const;
    core::ports::IStereoCalibrationLoader &StereoCalibrationLoader();
    const core::ports::IStereoCalibrationLoader &StereoCalibrationLoader() const;
    core::ports::IStereoRectifier &StereoRectifier();
    const core::ports::IStereoRectifier &StereoRectifier() const;
    core::ports::IStereoFramePreprocessor &StereoFramePreprocessor();
    const core::ports::IStereoFramePreprocessor &StereoFramePreprocessor() const;
    core::ports::IStereoPairBuilder &StereoPairBuilder();
    const core::ports::IStereoPairBuilder &StereoPairBuilder() const;
    core::ports::IStereoMatchSelector &StereoMatchSelector();
    const core::ports::IStereoMatchSelector &StereoMatchSelector() const;
    core::ports::ITemporalStereoProcessor &TemporalStereoProcessor();
    const core::ports::ITemporalStereoProcessor &TemporalStereoProcessor() const;
    core::ports::IStereoFeaturePacketBuilder &StereoFeaturePacketBuilder();
    const core::ports::IStereoFeaturePacketBuilder &
    StereoFeaturePacketBuilder() const;
    void SetStereoMatchSelector(
        std::unique_ptr<core::ports::IStereoMatchSelector> selector);
    void SetTemporalStereoProcessor(
        std::unique_ptr<core::ports::ITemporalStereoProcessor> processor);
    void SetStereoFeaturePacketBuilder(
        std::unique_ptr<core::ports::IStereoFeaturePacketBuilder> builder);
    void SetStereoCalibrationLoader(
        std::unique_ptr<core::ports::IStereoCalibrationLoader> loader);
    void
    SetStereoRectifier(std::unique_ptr<core::ports::IStereoRectifier> rectifier);
    void SetStereoFramePreprocessor(
        std::unique_ptr<core::ports::IStereoFramePreprocessor> preprocessor);
    void SetStereoPairBuilder(
        std::unique_ptr<core::ports::IStereoPairBuilder> builder);
    core::ports::IVisualPnpObservationBuilder &VisualPnpObservationBuilder();
    const core::ports::IVisualPnpObservationBuilder &
    VisualPnpObservationBuilder() const;
    core::ports::IVisualPnpPoseBackend &VisualPnpPoseBackend();
    const core::ports::IVisualPnpPoseBackend &VisualPnpPoseBackend() const;
    void SetVisualPnpObservationBuilder(
        std::unique_ptr<core::ports::IVisualPnpObservationBuilder> builder);
    void SetVisualPnpPoseBackend(
        std::unique_ptr<core::ports::IVisualPnpPoseBackend> backend);
    core::ports::IPointTracker2d &PointTracker2d();
    const core::ports::IPointTracker2d &PointTracker2d() const;
    core::ports::IVisualLoopClosureBackend &VisualLoopClosureBackend();
    const core::ports::IVisualLoopClosureBackend &
    VisualLoopClosureBackend() const;
    void SetPointTracker2d(std::unique_ptr<core::ports::IPointTracker2d> tracker);
    void SetVisualLoopClosureBackend(
        std::unique_ptr<core::ports::IVisualLoopClosureBackend> backend);

    core::ports::IVisualFeatureFrontend *m_visualFeatureFrontend{nullptr};
    int m_visualFeatureInputMaxWidth{640};
    int m_visualFeatureInputMaxHeight{400};
    mutable int m_lastVisualFeatureRawLeftCount{0};
    mutable int m_lastVisualFeatureRawRightCount{0};
    mutable int m_lastVisualFeatureMatchedStereoCount{0};
    mutable int m_lastVisualFeatureInjectedLeftCount{0};
    mutable int m_lastVisualFeatureInjectedRightCount{0};
    mutable uint64_t m_lastVisualFeatureObservationHash{0};
    mutable double m_lastVisualFeaturePrepareMs{0.0};
    mutable double m_lastVisualFeatureInputMs{0.0};
    mutable double m_lastVisualFeatureForwardMs{0.0};
    mutable double m_lastVisualFeatureFrontendMs{0.0};
    mutable double m_lastVisualFeatureStereoMatchMs{0.0};
    mutable double m_lastVisualFeatureTotalMs{0.0};
    mutable uint32_t m_lastVisualFeatureImageCount{0};
    mutable uint32_t m_lastVisualFeaturePayloadBytes{0};
    mutable int m_visualFeatureLightGlueOkStreak{0};
    mutable int m_visualFeatureLightGlueLastEveryN{0};
    mutable int m_visualFeatureLightGlueBootstrapTrustFrames{0};
    mutable bool m_visualFeatureLightGlueBootstrapTrustClosed{false};
    mutable int m_lastSlamMatchesInliers{0};
    mutable int m_lastSlamTrackedMapPoints{0};
    mutable cv::Mat m_visualFeatureTemporalPrevLeft;
    mutable cv::Mat m_visualFeatureTemporalPrevRight;
    mutable std::vector<cv::Point2f> m_visualFeatureTemporalPrevLeftPoints;
    mutable std::vector<cv::Point2f> m_visualFeatureTemporalPrevRightPoints;
    mutable bool m_visualFeatureTemporalHavePrevStereo{false};
    std::unique_ptr<core::ports::IStereoMatchSelector> m_stereoMatchSelector;
    std::unique_ptr<core::ports::ITemporalStereoProcessor>
        m_temporalStereoProcessor;
    std::unique_ptr<core::ports::IStereoFeaturePacketBuilder>
        m_stereoFeaturePacketBuilder;
    std::unique_ptr<core::ports::IStereoCalibrationLoader>
        m_stereoCalibrationLoader;
    std::unique_ptr<core::ports::IStereoRectifier> m_stereoRectifier;
    std::unique_ptr<core::ports::IStereoFramePreprocessor>
        m_stereoFramePreprocessor;
    std::unique_ptr<core::ports::IStereoPairBuilder> m_stereoPairBuilder;
    std::unique_ptr<core::ports::IVisualPnpObservationBuilder>
        m_visualPnpObservationBuilder;
    std::unique_ptr<core::ports::IVisualPnpPoseBackend> m_visualPnpPoseBackend;
    std::unique_ptr<core::ports::IPointTracker2d> m_pointTracker2d;
    std::unique_ptr<core::ports::IVisualLoopClosureBackend>
        m_visualLoopClosureBackend;

    bool m_lkCalibrationLoaded{false};
    StereoCalibration m_stereoCalibration;
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
    mutable LkLoopClosureState m_lkLoop;
};

} // namespace SmartDrone::adapters::slam
