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

class SuperPointLightGlueFrontendClient {
  public:
    struct Stats {
        double prepareMs{0.0};
        double inputMs{0.0};
        double forwardMs{0.0};
        double totalMs{0.0};
        uint32_t imageCount{0};
        uint32_t payloadBytes{0};
    };

    SuperPointLightGlueFrontendClient();
    ~SuperPointLightGlueFrontendClient();

    SuperPointLightGlueFrontendClient(const SuperPointLightGlueFrontendClient &) = delete;
    SuperPointLightGlueFrontendClient &operator=(const SuperPointLightGlueFrontendClient &) = delete;

    bool Start(const std::string &repoPath, const std::string &device, int topK, int maxPoints, std::string *err);
    void Stop();
    bool Running() const;
    bool Detect(const cv::Mat &gray, std::vector<cv::Point2f> &outPoints, std::string *err);
    bool DetectAndCompute(const cv::Mat &gray, SuperPointFeatureSet &outFeatures, std::string *err);
    bool DetectAndComputeStereo(const cv::Mat &leftGray, const cv::Mat &rightGray, SuperPointFeatureSet &leftFeatures,
                                SuperPointFeatureSet &rightFeatures, std::string *err);
    Stats LastStats() const;

  private:
    Stats m_lastStats{};
    std::unique_ptr<SuperPointNativeExtractor> m_superPointNativeExtractor;

    void CopyNativeStats();
};

} // namespace smartdrone::adapters::slam
