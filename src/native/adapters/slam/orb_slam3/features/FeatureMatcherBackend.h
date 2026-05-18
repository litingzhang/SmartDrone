#pragma once

#include <memory>
#include <set>
#include <vector>

#include <opencv2/core.hpp>
#include <sophus/sim3.hpp>

namespace ORB_SLAM3 {

class Frame;
class KeyFrame;
class MapPoint;

constexpr int kOrbMatcherLowDistance = 50;
constexpr int kOrbMatcherHighDistance = 100;
constexpr int kOrbMatcherOrientationHistogramLength = 30;

struct OrbFeatureMatcherOptions {
  float nnRatio{0.6f};
  bool checkOrientation{true};
};

struct OrbFrameMapProjectionRequest {
  Frame *frame{nullptr};
  const std::vector<MapPoint *> *mapPoints{nullptr};
  float threshold{3.0f};
  bool farPoints{false};
  float farPointThreshold{50.0f};
};

struct OrbFrameFrameProjectionRequest {
  Frame *currentFrame{nullptr};
  const Frame *lastFrame{nullptr};
  float threshold{0.0f};
  bool monocular{false};
};

struct OrbFrameKeyFrameProjectionRequest {
  Frame *currentFrame{nullptr};
  KeyFrame *keyFrame{nullptr};
  const std::set<MapPoint *> *alreadyFound{nullptr};
  float threshold{0.0f};
  int orbDistance{0};
};

struct OrbKeyFrameSim3ProjectionRequest {
  KeyFrame *keyFrame{nullptr};
  Sophus::Sim3f *cameraPose{nullptr};
  const std::vector<MapPoint *> *mapPoints{nullptr};
  std::vector<MapPoint *> *matchedMapPoints{nullptr};
  int threshold{0};
  float hammingRatio{1.0f};
};

struct OrbKeyFrameSim3ProjectionWithSourcesRequest {
  KeyFrame *keyFrame{nullptr};
  Sophus::Sim3f *cameraPose{nullptr};
  const std::vector<MapPoint *> *mapPoints{nullptr};
  const std::vector<KeyFrame *> *sourceKeyFrames{nullptr};
  std::vector<MapPoint *> *matchedMapPoints{nullptr};
  std::vector<KeyFrame *> *matchedKeyFrames{nullptr};
  int threshold{0};
  float hammingRatio{1.0f};
};

struct OrbFrameBoWMatchRequest {
  KeyFrame *keyFrame{nullptr};
  Frame *frame{nullptr};
  std::vector<MapPoint *> *mapPointMatches{nullptr};
};

struct OrbKeyFrameBoWMatchRequest {
  KeyFrame *firstKeyFrame{nullptr};
  KeyFrame *secondKeyFrame{nullptr};
  std::vector<MapPoint *> *matches12{nullptr};
};

struct OrbInitializationMatchRequest {
  Frame *firstFrame{nullptr};
  Frame *secondFrame{nullptr};
  std::vector<cv::Point2f> *previousMatched{nullptr};
  std::vector<int> *matches12{nullptr};
  int windowSize{10};
};

struct OrbTriangulationMatchRequest {
  KeyFrame *firstKeyFrame{nullptr};
  KeyFrame *secondKeyFrame{nullptr};
  std::vector<std::pair<size_t, size_t>> *matchedPairs{nullptr};
  bool onlyStereo{false};
  bool coarse{false};
};

struct OrbSim3MatchRequest {
  KeyFrame *firstKeyFrame{nullptr};
  KeyFrame *secondKeyFrame{nullptr};
  std::vector<MapPoint *> *matches12{nullptr};
  const Sophus::Sim3f *transform12{nullptr};
  float threshold{0.0f};
};

struct OrbKeyFrameFuseRequest {
  KeyFrame *keyFrame{nullptr};
  const std::vector<MapPoint *> *mapPoints{nullptr};
  float threshold{3.0f};
  bool rightImage{false};
};

struct OrbSim3FuseRequest {
  KeyFrame *keyFrame{nullptr};
  Sophus::Sim3f *cameraPose{nullptr};
  const std::vector<MapPoint *> *mapPoints{nullptr};
  float threshold{0.0f};
  std::vector<MapPoint *> *replacePoints{nullptr};
};

class IOrbFeatureMatcher {
public:
  virtual ~IOrbFeatureMatcher() = default;

  virtual int DescriptorDistance(const cv::Mat &a, const cv::Mat &b) const = 0;
  virtual int SearchFrameMapByProjection(
      const OrbFrameMapProjectionRequest &request) = 0;
  virtual int SearchFrameFrameByProjection(
      const OrbFrameFrameProjectionRequest &request) = 0;
  virtual int SearchFrameKeyFrameByProjection(
      const OrbFrameKeyFrameProjectionRequest &request) = 0;
  virtual int SearchKeyFrameSim3ByProjection(
      const OrbKeyFrameSim3ProjectionRequest &request) = 0;
  virtual int SearchKeyFrameSim3WithSourcesByProjection(
      const OrbKeyFrameSim3ProjectionWithSourcesRequest &request) = 0;
  virtual int SearchFrameByBoW(const OrbFrameBoWMatchRequest &request) = 0;
  virtual int SearchKeyFrameByBoW(
      const OrbKeyFrameBoWMatchRequest &request) = 0;
  virtual int SearchForInitialization(
      const OrbInitializationMatchRequest &request) = 0;
  virtual int SearchForTriangulation(
      const OrbTriangulationMatchRequest &request) = 0;
  virtual int SearchBySim3(const OrbSim3MatchRequest &request) = 0;
  virtual int FuseKeyFrame(const OrbKeyFrameFuseRequest &request) = 0;
  virtual int FuseSim3(const OrbSim3FuseRequest &request) = 0;
};

int ComputeOrbDescriptorDistance(const cv::Mat &a, const cv::Mat &b);

std::unique_ptr<IOrbFeatureMatcher>
CreateDefaultOrbFeatureMatcher(const OrbFeatureMatcherOptions &options);

} // namespace ORB_SLAM3
