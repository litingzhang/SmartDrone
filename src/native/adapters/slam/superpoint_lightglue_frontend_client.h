#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "adapters/slam/external_feature_frontend_client.h"

namespace smartdrone::adapters::slam {

class SuperPointNativeExtractor;

using SuperPointFeatureSet = ExternalFeatureSet;

class SuperPointLightGlueFrontendClient final : public ExternalFeatureFrontendClient {
  public:
    using Stats = ExternalFeatureFrontendClient::Stats;

    SuperPointLightGlueFrontendClient();
    ~SuperPointLightGlueFrontendClient() override;

    SuperPointLightGlueFrontendClient(const SuperPointLightGlueFrontendClient &) = delete;
    SuperPointLightGlueFrontendClient &operator=(const SuperPointLightGlueFrontendClient &) = delete;

    bool Start(const std::string &repoPath, const std::string &device, int topK, int maxPoints, std::string *err);
    void Stop();
    bool Running() const override;
    bool Detect(const cv::Mat &gray, std::vector<cv::Point2f> &outPoints, std::string *err) override;
    bool DetectAndCompute(const cv::Mat &gray, SuperPointFeatureSet &outFeatures, std::string *err) override;
    bool DetectAndComputeStereo(const cv::Mat &leftGray, const cv::Mat &rightGray, SuperPointFeatureSet &leftFeatures,
                                SuperPointFeatureSet &rightFeatures, std::string *err) override;
    void SetLightGlueEveryNOverride(int everyN) override;
    Stats LastStats() const override;

  private:
    Stats m_lastStats{};
    std::unique_ptr<SuperPointNativeExtractor> m_superPointNativeExtractor;

    void CopyNativeStats();
};

} // namespace smartdrone::adapters::slam
