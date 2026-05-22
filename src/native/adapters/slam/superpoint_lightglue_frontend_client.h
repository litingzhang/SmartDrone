#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "adapters/slam/visual_feature_frontend_client.h"
#include "core/ports/visual_feature_frontend.h"

namespace SmartDrone::Adapters::Slam {

class SuperPointNativeExtractor;

using SuperPointFeatureSet = Core::Ports::VisualFeatureSet;

class SuperPointLightGlueFrontendClient final
    : public IManagedVisualFeatureFrontend {
  public:
    using Stats = Core::Ports::IVisualFeatureFrontend::Stats;

    SuperPointLightGlueFrontendClient();
    ~SuperPointLightGlueFrontendClient() override;

    SuperPointLightGlueFrontendClient(const SuperPointLightGlueFrontendClient &) =
        delete;
    SuperPointLightGlueFrontendClient &
    operator=(const SuperPointLightGlueFrontendClient &) = delete;

    bool Start(const VisualFeatureFrontendRuntimeConfig &config,
               std::string *err) override;
    bool Start(const std::string &repoPath, const std::string &device, int topK,
               int maxPoints, std::string *err);
    void Stop() override;
    bool Running() const override;
    bool Detect(const Core::Ports::VisualFeatureDetectRequest &request,
                Core::Ports::VisualFeatureDetectResult &result) override;
    bool DetectAndCompute(
        const Core::Ports::VisualFeatureComputeRequest &request,
        Core::Ports::VisualFeatureComputeResult &result) override;
    bool DetectAndComputeStereo(
        const Core::Ports::StereoVisualFeatureComputeRequest &request,
        Core::Ports::StereoVisualFeatureComputeResult &result) override;
    void SetLightGlueEveryNOverride(int everyN) override;
    Stats LastStats() const override;

  private:
    Stats m_lastStats{};
    std::unique_ptr<SuperPointNativeExtractor> m_superPointNativeExtractor;

    void CopyNativeStats();
};

} // namespace SmartDrone::Adapters::Slam
