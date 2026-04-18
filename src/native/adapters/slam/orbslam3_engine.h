#pragma once

#include <cstdint>
#include <memory>

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

class OrbSlam3Engine final : public core::ports::ISlamEngine {
  public:
    OrbSlam3Engine(std::unique_ptr<ORB_SLAM3::System> system, OrbInputMode inputMode, bool useImu);

    bool Start() override;
    void SetOperationMode(core::domain::SlamOperationMode mode);
    void SetFeatureFrontend(FeatureFrontend frontend);
    void SetXFeatFrontendClient(XFeatFrontendClient *client);
    void Stop() override;
    core::ports::SlamOutput Process(const core::ports::SlamInputBatch &input, bool extractFeatures,
                                    bool extractPointCloud) override;

  private:
    static std::vector<cv::KeyPoint> ToKeyPoints(const std::vector<cv::Point2f> &points);
    bool BuildMonoExternalData(const cv::Mat &gray, ORB_SLAM3::ExternalMonoFrameData &outData) const;
    bool BuildStereoExternalData(const cv::Mat &leftGray, const cv::Mat &rightGray,
                                 ORB_SLAM3::ExternalStereoFrameData &outData) const;

    std::unique_ptr<ORB_SLAM3::System> m_system;
    OrbInputMode m_inputMode{OrbInputMode::Stereo};
    bool m_useImu{false};
    core::domain::SlamOperationMode m_operationMode{core::domain::SlamOperationMode::Mapping};
    FeatureFrontend m_featureFrontend{FeatureFrontend::Orb};
    XFeatFrontendClient *m_xfeatFrontendClient{nullptr};
    mutable int m_lastXFeatRawLeftCount{0};
    mutable int m_lastXFeatRawRightCount{0};
    mutable int m_lastXFeatMatchedStereoCount{0};
    mutable int m_lastXFeatInjectedLeftCount{0};
    mutable int m_lastXFeatInjectedRightCount{0};
};

} // namespace smartdrone::adapters::slam
