#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "adapters/slam/orb_feature_options.h"
#include "core/ports/visual_feature_frontend.h"

namespace ORB_SLAM3 {
class ORBextractor;
}

namespace SmartDrone::Adapters::Slam {

class OrbDescriptorProvider final
    : public Core::Ports::IVisualDescriptorProvider {
  public:
    explicit OrbDescriptorProvider(ORB_SLAM3::ORBextractor *extractor = nullptr);

    void SetExtractor(ORB_SLAM3::ORBextractor *extractor);
    bool Available() const;

    bool ComputeDescriptorsAtPoints(const cv::Mat &gray,
                                    const std::vector<cv::Point2f> &points,
                                    std::vector<cv::KeyPoint> &keypoints,
                                    cv::Mat &descriptors) const override;
    bool DetectAndCompute(const cv::Mat &gray,
                          std::vector<cv::KeyPoint> &keypoints,
                          cv::Mat &descriptors) const override;
    int DescriptorDistance(const cv::Mat &leftDescriptor,
                           const cv::Mat &rightDescriptor) const override;

  private:
    ORB_SLAM3::ORBextractor *m_extractor{nullptr};
};

class DefaultOrbFeatureFrontend final
    : public Core::Ports::IVisualFeatureFrontend,
      public Core::Ports::IVisualDescriptorProvider {
  public:
    explicit DefaultOrbFeatureFrontend(
        const OrbFeatureExtractorOptions &options = {});
    ~DefaultOrbFeatureFrontend() override;

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
    Core::Ports::VisualFeatureFrontendStats LastStats() const override;

    bool ComputeDescriptorsAtPoints(const cv::Mat &gray,
                                    const std::vector<cv::Point2f> &points,
                                    std::vector<cv::KeyPoint> &keypoints,
                                    cv::Mat &descriptors) const override;
    bool DetectAndCompute(const cv::Mat &gray,
                          std::vector<cv::KeyPoint> &keypoints,
                          cv::Mat &descriptors) const override;
    int DescriptorDistance(const cv::Mat &leftDescriptor,
                           const cv::Mat &rightDescriptor) const override;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Adapters::Slam
