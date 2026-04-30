#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace smartdrone::adapters::slam {

class SuperPointNativeExtractor;

struct XFeatFeatureSet {
    std::vector<cv::Point2f> keypoints;
    cv::Mat descriptors;
};

class XFeatFrontendClient {
  public:
    struct Stats {
        double prepareMs{0.0};
        double writeMs{0.0};
        double readMs{0.0};
        double totalMs{0.0};
        uint32_t imageCount{0};
        uint32_t payloadBytes{0};
    };

    XFeatFrontendClient();
    ~XFeatFrontendClient();

    XFeatFrontendClient(const XFeatFrontendClient &) = delete;
    XFeatFrontendClient &operator=(const XFeatFrontendClient &) = delete;

    bool Start(const std::string &repoPath, const std::string &device, int topK, int maxPoints, std::string *err);
    void Stop();
    bool Running() const;
    bool Detect(const cv::Mat &gray, std::vector<cv::Point2f> &outPoints, std::string *err);
    bool DetectAndCompute(const cv::Mat &gray, XFeatFeatureSet &outFeatures, std::string *err);
    bool DetectAndComputeStereo(const cv::Mat &leftGray, const cv::Mat &rightGray, XFeatFeatureSet &leftFeatures,
                                XFeatFeatureSet &rightFeatures, std::string *err);
    Stats LastStats() const;

  private:
    Stats m_lastStats{};
    std::unique_ptr<SuperPointNativeExtractor> m_superPointNativeExtractor;

    void CopyNativeStats();
};

} // namespace smartdrone::adapters::slam
