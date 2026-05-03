#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace smartdrone::adapters::slam {

class SuperPointNativeExtractor;

struct SuperPointFeatureSet {
    std::vector<cv::Point2f> keypoints;
    cv::Mat descriptors;
};

using ExternalFeatureSet = SuperPointFeatureSet;

struct ExternalFeatureFrontendStats {
    double prepareMs{0.0};
    double inputMs{0.0};
    double forwardMs{0.0};
    double totalMs{0.0};
    uint32_t imageCount{0};
    uint32_t payloadBytes{0};
};

class ExternalFeatureFrontendClient {
  public:
    using Stats = ExternalFeatureFrontendStats;

    virtual ~ExternalFeatureFrontendClient() = default;

    virtual bool Running() const = 0;
    virtual bool Detect(const cv::Mat &gray, std::vector<cv::Point2f> &outPoints, std::string *err) = 0;
    virtual bool DetectAndCompute(const cv::Mat &gray, SuperPointFeatureSet &outFeatures, std::string *err) = 0;
    virtual bool DetectAndComputeStereo(const cv::Mat &leftGray, const cv::Mat &rightGray,
                                        SuperPointFeatureSet &leftFeatures, SuperPointFeatureSet &rightFeatures,
                                        std::string *err) = 0;
    virtual Stats LastStats() const = 0;
};

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
    Stats LastStats() const override;

  private:
    Stats m_lastStats{};
    std::unique_ptr<SuperPointNativeExtractor> m_superPointNativeExtractor;

    void CopyNativeStats();
};

} // namespace smartdrone::adapters::slam
