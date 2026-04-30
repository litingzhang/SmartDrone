#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "System.h"
#include "adapters/slam/xfeat_frontend_client.h"
#include "core/application/config/app_args.h"
#include "core/domain/runtime_mode.h"
#include "core/ports/slam_engine.h"

namespace smartdrone::adapters::slam {

enum class OrbInputMode : uint8_t {
    Stereo,
    MonoLeft,
    MonoRight,
};

struct LkStereoTrack {
    cv::Point2f left;
    cv::Point2f right;
    float quality{0.0f};
    uint32_t age{0};
};

struct LkFrameSnapshot {
    uint64_t frameId{0};
    cv::Mat left;
    cv::Mat right;
};

struct LkLoopKeyframe {
    uint64_t frameId{0};
    Sophus::SE3f rawTwc{Sophus::SE3f()};
    Sophus::SE3f correctedTwc{Sophus::SE3f()};
    cv::Mat descriptor;
};

struct LkXFeatSeedResult {
    uint64_t frameId{0};
    std::vector<LkStereoTrack> seeds;
    XFeatFrontendClient::Stats stats{};
    double matchMs{0.0};
    double totalMs{0.0};
};

struct LkPerFrameVpiState;

class OrbSlam3Engine final : public core::ports::ISlamEngine {
  public:
    OrbSlam3Engine(std::unique_ptr<ORB_SLAM3::System> system, OrbInputMode inputMode, bool useImu,
                   std::string settingsPath = {});

    bool Start() override;
    void SetOperationMode(core::domain::SlamOperationMode mode);
    void SetFeatureFrontend(FeatureFrontend frontend);
    void SetXFeatFrontendClient(XFeatFrontendClient *client);
    void SetXFeatInputSizeLimit(int maxWidth, int maxHeight);
    void SetLkLoopClosure(bool enabled, float scale = 1.20f, float relaxation = 1.40f);
    void SetLkPerFrameAcceleration(std::string acceleration);
    void Stop() override;
    bool ShutdownAndSaveOrbTrajectoryEuRoC(const std::string &path);
    core::ports::SlamOutput Process(const core::ports::SlamInputBatch &input, bool extractFeatures,
                                    bool extractPointCloud) override;

  private:
    void StabilizeOutputPose(core::ports::PoseEstimate &pose, bool &poseValid, double timestampSec, int trackingState);
    bool LoadLkCalibration(const std::string &settingsPath);
    void EnsureLkRectifier(const cv::Size &inputSize);
    core::ports::SlamOutput ProcessLkStereoVo(const core::ports::SlamInputBatch &input, bool extractFeatures);
    core::ports::SlamOutput ProcessLkGfttPerFrameStereoVo(const core::ports::SlamInputBatch &input,
                                                          bool extractFeatures);
    Sophus::SE3f ApplyLkLoopClosure(const cv::Mat &leftRect, uint64_t frameId, const Sophus::SE3f &rawTwc);

    std::unique_ptr<ORB_SLAM3::System> m_system;
    OrbInputMode m_inputMode{OrbInputMode::Stereo};
    bool m_useImu{false};
    core::domain::SlamOperationMode m_operationMode{core::domain::SlamOperationMode::Mapping};
    FeatureFrontend m_featureFrontend{FeatureFrontend::Orb};
    XFeatFrontendClient *m_xfeatFrontendClient{nullptr};
    int m_xfeatInputMaxWidth{640};
    int m_xfeatInputMaxHeight{400};
    mutable int m_lastXFeatRawLeftCount{0};
    mutable int m_lastXFeatRawRightCount{0};
    mutable int m_lastXFeatMatchedStereoCount{0};
    mutable int m_lastXFeatInjectedLeftCount{0};
    mutable int m_lastXFeatInjectedRightCount{0};
    mutable uint64_t m_lastXFeatSeedSourceFrameId{0};
    mutable uint64_t m_lastXFeatSeedCurrentFrameId{0};
    mutable uint32_t m_lastXFeatSeedAgeFrames{0};
    mutable int m_lastXFeatSeedForwardedCount{0};
    mutable double m_lastXFeatPrepareMs{0.0};
    mutable double m_lastXFeatWorkerWriteMs{0.0};
    mutable double m_lastXFeatWorkerReadMs{0.0};
    mutable double m_lastXFeatWorkerTotalMs{0.0};
    mutable double m_lastXFeatStereoMatchMs{0.0};
    mutable double m_lastXFeatTotalMs{0.0};
    mutable uint32_t m_lastXFeatImageCount{0};
    mutable uint32_t m_lastXFeatPayloadBytes{0};
    mutable std::string m_lastXFeatStatusReason;
    core::ports::PoseEstimate m_lastStablePose{};
    bool m_haveLastStablePose{false};
    double m_lastStableTimestampSec{0.0};
    float m_stableVelX{0.0f};
    float m_stableVelY{0.0f};
    float m_stableVelZ{0.0f};

    std::string m_settingsPath;
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
    mutable std::deque<LkFrameSnapshot> m_lkFrameHistory;
    mutable uint64_t m_lkLastXFeatSeedFrameId{0};
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
