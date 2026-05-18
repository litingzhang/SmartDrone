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

namespace smartdrone::adapters::slam {

class OrbDescriptorProvider final
    : public core::ports::IVisualDescriptorProvider {
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
    : public core::ports::IVisualFeatureFrontend,
      public core::ports::IVisualDescriptorProvider {
public:
  explicit DefaultOrbFeatureFrontend(
      const OrbFeatureExtractorOptions &options = {});
  ~DefaultOrbFeatureFrontend() override;

  bool Running() const override;
  bool Detect(const core::ports::VisualFeatureDetectRequest &request,
              core::ports::VisualFeatureDetectResult &result) override;
  bool DetectAndCompute(
      const core::ports::VisualFeatureComputeRequest &request,
      core::ports::VisualFeatureComputeResult &result) override;
  bool DetectAndComputeStereo(
      const core::ports::StereoVisualFeatureComputeRequest &request,
      core::ports::StereoVisualFeatureComputeResult &result) override;
  void SetLightGlueEveryNOverride(int everyN) override;
  core::ports::VisualFeatureFrontendStats LastStats() const override;

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

} // namespace smartdrone::adapters::slam
