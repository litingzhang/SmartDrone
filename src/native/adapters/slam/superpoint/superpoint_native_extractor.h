#pragma once

#include <memory>
#include <iosfwd>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "adapters/slam/superpoint/superpoint_lightglue_frontend_client.h"

namespace SmartDrone::Adapters::Slam {

class SuperPointNativeExtractor {
  public:
    struct Stats {
        double prepareMs{0.0};
        double inputMs{0.0};
        double forwardMs{0.0};
        double postMs{0.0};
        double inferMs{0.0};
        double totalMs{0.0};
        int rawLeftCount{0};
        int rawRightCount{0};
        int stereoLeftCount{0};
        int stereoRightCount{0};
        bool lightGlueUsed{false};
        bool descriptorFallbackUsed{false};
        uint32_t imageCount{0};
        uint32_t payloadBytes{0};
    };

    SuperPointNativeExtractor();
    ~SuperPointNativeExtractor();

    bool Start(const std::string &repoPath, const std::string &device, int topK, int maxPoints, std::string *err);
    void Stop();
    bool Running() const;
    bool Detect(const cv::Mat &gray, std::vector<cv::Point2f> &outPoints, std::string *err);
    bool DetectAndCompute(const cv::Mat &gray, SuperPointFeatureSet &outFeatures, std::string *err);
    bool DetectAndComputeStereo(const cv::Mat &leftGray, const cv::Mat &rightGray, SuperPointFeatureSet &leftFeatures,
                                SuperPointFeatureSet &rightFeatures, std::string *err);
    void SetLightGlueEveryNOverride(int everyN);
    Stats LastStats() const;

  private:
    struct Impl;
    struct StereoComputeContext;
    struct StereoMatchState;

    static bool PrepareGrayImage(const cv::Mat &gray, cv::Mat &gray8, std::string *err);
    static bool PrepareStereoGrayImages(const cv::Mat &leftGray,
                                        const cv::Mat &rightGray,
                                        StereoComputeContext &context,
                                        std::string *err);

    void ConfigureStereoExtractionBudgets(StereoComputeContext &context) const;
    bool RunStereoSuperPointBatch(StereoComputeContext &context,
                                  std::string *err);
    void ResetLightGlueStats();
    void ConfigureStereoMatchState(const StereoComputeContext &context,
                                   StereoMatchState &match);
    void BuildDescriptorMatches(const StereoComputeContext &context,
                                StereoMatchState &match);
    void UseDescriptorPrimary(const StereoComputeContext &context,
                              StereoMatchState &match,
                              SuperPointFeatureSet &leftFeatures,
                              SuperPointFeatureSet &rightFeatures);
    void AppendDescriptorSupplementIfNeeded(
        const StereoComputeContext &context, StereoMatchState &match,
        SuperPointFeatureSet &leftFeatures,
        SuperPointFeatureSet &rightFeatures);
    bool TryUseLightGlue(const StereoComputeContext &context,
                         StereoMatchState &match,
                         SuperPointFeatureSet &leftFeatures,
                         SuperPointFeatureSet &rightFeatures,
                         std::string *err);
    void UpdateStereoStats(const StereoComputeContext &context,
                           const StereoMatchState &match,
                           const SuperPointFeatureSet &leftFeatures,
                           const SuperPointFeatureSet &rightFeatures);
    void LogStereoPerformance(const StereoComputeContext &context,
                              const StereoMatchState &match,
                              const SuperPointFeatureSet &leftFeatures,
                              const SuperPointFeatureSet &rightFeatures) const;
    void LogSuperPointPerformanceFields(std::ostream &out) const;
    void LogLightGluePerformanceFields(std::ostream &out,
                                       const StereoMatchState &match) const;
    void LogDescriptorPerformanceFields(
        std::ostream &out, const StereoComputeContext &context,
        const StereoMatchState &match) const;
    void LogStereoPointSummaryFields(
        std::ostream &out, const StereoComputeContext &context,
        const StereoMatchState &match,
        const SuperPointFeatureSet &leftFeatures,
        const SuperPointFeatureSet &rightFeatures) const;

    bool m_running{false};
    int m_topK{0};
    int m_maxPoints{0};
    int m_lightGlueEveryNOverride{0};
    Stats m_lastStats{};
    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Adapters::Slam
