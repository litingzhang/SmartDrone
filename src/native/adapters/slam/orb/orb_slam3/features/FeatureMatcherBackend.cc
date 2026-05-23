#include "FeatureMatcherBackend.h"

#include "ORBmatcher.h"

namespace ORB_SLAM3 {

namespace {

class DefaultOrbFeatureMatcher final : public IOrbFeatureMatcher {
public:
  explicit DefaultOrbFeatureMatcher(const OrbFeatureMatcherOptions &options)
      : m_matcher(options.nnRatio, options.checkOrientation) {}

  int DescriptorDistance(const cv::Mat &a, const cv::Mat &b) const override {
    return ORBmatcher::DescriptorDistance(a, b);
  }

  int SearchFrameMapByProjection(
      const OrbFrameMapProjectionRequest &request) override {
    if (request.frame == nullptr || request.mapPoints == nullptr) {
      return 0;
    }
    return m_matcher.SearchByProjection(
        *request.frame, *request.mapPoints, request.threshold,
        request.farPoints, request.farPointThreshold);
  }

  int SearchFrameFrameByProjection(
      const OrbFrameFrameProjectionRequest &request) override {
    if (request.currentFrame == nullptr || request.lastFrame == nullptr) {
      return 0;
    }
    return m_matcher.SearchByProjection(
        *request.currentFrame, *request.lastFrame, request.threshold,
        request.monocular);
  }

  int SearchFrameKeyFrameByProjection(
      const OrbFrameKeyFrameProjectionRequest &request) override {
    if (request.currentFrame == nullptr || request.keyFrame == nullptr ||
        request.alreadyFound == nullptr) {
      return 0;
    }
    return m_matcher.SearchByProjection(
        *request.currentFrame, request.keyFrame, *request.alreadyFound,
        request.threshold, request.orbDistance);
  }

  int SearchKeyFrameSim3ByProjection(
      const OrbKeyFrameSim3ProjectionRequest &request) override {
    if (request.keyFrame == nullptr || request.cameraPose == nullptr ||
        request.mapPoints == nullptr || request.matchedMapPoints == nullptr) {
      return 0;
    }
    return m_matcher.SearchByProjection(
        request.keyFrame, *request.cameraPose, *request.mapPoints,
        *request.matchedMapPoints, request.threshold, request.hammingRatio);
  }

  int SearchKeyFrameSim3WithSourcesByProjection(
      const OrbKeyFrameSim3ProjectionWithSourcesRequest &request) override {
    if (request.keyFrame == nullptr || request.cameraPose == nullptr ||
        request.mapPoints == nullptr || request.sourceKeyFrames == nullptr ||
        request.matchedMapPoints == nullptr ||
        request.matchedKeyFrames == nullptr) {
      return 0;
    }
    return m_matcher.SearchByProjection(
        request.keyFrame, *request.cameraPose, *request.mapPoints,
        *request.sourceKeyFrames, *request.matchedMapPoints,
        *request.matchedKeyFrames, request.threshold, request.hammingRatio);
  }

  int SearchFrameByBoW(const OrbFrameBoWMatchRequest &request) override {
    if (request.keyFrame == nullptr || request.frame == nullptr ||
        request.mapPointMatches == nullptr) {
      return 0;
    }
    return m_matcher.SearchByBoW(request.keyFrame, *request.frame,
                                 *request.mapPointMatches);
  }

  int SearchKeyFrameByBoW(
      const OrbKeyFrameBoWMatchRequest &request) override {
    if (request.firstKeyFrame == nullptr ||
        request.secondKeyFrame == nullptr || request.matches12 == nullptr) {
      return 0;
    }
    return m_matcher.SearchByBoW(request.firstKeyFrame, request.secondKeyFrame,
                                 *request.matches12);
  }

  int SearchForInitialization(
      const OrbInitializationMatchRequest &request) override {
    if (request.firstFrame == nullptr || request.secondFrame == nullptr ||
        request.previousMatched == nullptr || request.matches12 == nullptr) {
      return 0;
    }
    return m_matcher.SearchForInitialization(
        *request.firstFrame, *request.secondFrame, *request.previousMatched,
        *request.matches12, request.windowSize);
  }

  int SearchForTriangulation(
      const OrbTriangulationMatchRequest &request) override {
    if (request.firstKeyFrame == nullptr ||
        request.secondKeyFrame == nullptr || request.matchedPairs == nullptr) {
      return 0;
    }
    return m_matcher.SearchForTriangulation(
        request.firstKeyFrame, request.secondKeyFrame, *request.matchedPairs,
        request.onlyStereo, request.coarse);
  }

  int SearchBySim3(const OrbSim3MatchRequest &request) override {
    if (request.firstKeyFrame == nullptr ||
        request.secondKeyFrame == nullptr || request.matches12 == nullptr ||
        request.transform12 == nullptr) {
      return 0;
    }
    return m_matcher.SearchBySim3(request.firstKeyFrame,
                                  request.secondKeyFrame, *request.matches12,
                                  *request.transform12, request.threshold);
  }

  int FuseKeyFrame(const OrbKeyFrameFuseRequest &request) override {
    if (request.keyFrame == nullptr || request.mapPoints == nullptr) {
      return 0;
    }
    return m_matcher.Fuse(request.keyFrame, *request.mapPoints,
                          request.threshold, request.rightImage);
  }

  int FuseSim3(const OrbSim3FuseRequest &request) override {
    if (request.keyFrame == nullptr || request.cameraPose == nullptr ||
        request.mapPoints == nullptr || request.replacePoints == nullptr) {
      return 0;
    }
    return m_matcher.Fuse(request.keyFrame, *request.cameraPose,
                          *request.mapPoints, request.threshold,
                          *request.replacePoints);
  }

private:
  ORBmatcher m_matcher;
};

} // namespace

std::unique_ptr<IOrbFeatureMatcher>
CreateDefaultOrbFeatureMatcher(const OrbFeatureMatcherOptions &options) {
  return std::make_unique<DefaultOrbFeatureMatcher>(options);
}

int ComputeOrbDescriptorDistance(const cv::Mat &a, const cv::Mat &b) {
  return ORBmatcher::DescriptorDistance(a, b);
}

} // namespace ORB_SLAM3
