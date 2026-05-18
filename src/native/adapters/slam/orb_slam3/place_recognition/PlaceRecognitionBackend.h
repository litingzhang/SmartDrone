#pragma once

#include <memory>
#include <vector>

namespace ORB_SLAM3 {

class Frame;
class KeyFrame;
class KeyFrameDatabase;
class Map;

struct OrbPlaceRecognitionKeyFrameRequest {
  KeyFrame *keyFrame{nullptr};
};

struct OrbPlaceRecognitionRelocalizationRequest {
  Frame *frame{nullptr};
  Map *map{nullptr};
};

struct OrbPlaceRecognitionLoopMergeRequest {
  KeyFrame *keyFrame{nullptr};
  int candidateCount{3};
};

struct OrbPlaceRecognitionLoopMergeCandidates {
  std::vector<KeyFrame *> loopCandidates;
  std::vector<KeyFrame *> mergeCandidates;
};

struct OrbPlaceRecognitionResetRequest {
  Map *map{nullptr};
};

class IOrbPlaceRecognitionBackend {
public:
  virtual ~IOrbPlaceRecognitionBackend() = default;

  virtual void AddKeyFrame(
      const OrbPlaceRecognitionKeyFrameRequest &request) = 0;
  virtual std::vector<KeyFrame *> DetectRelocalizationCandidates(
      const OrbPlaceRecognitionRelocalizationRequest &request) = 0;
  virtual OrbPlaceRecognitionLoopMergeCandidates DetectLoopMergeCandidates(
      const OrbPlaceRecognitionLoopMergeRequest &request) = 0;
  virtual void Clear() = 0;
  virtual void ClearMap(const OrbPlaceRecognitionResetRequest &request) = 0;
};

std::unique_ptr<IOrbPlaceRecognitionBackend>
CreateDefaultOrbPlaceRecognitionBackend(KeyFrameDatabase *database);

} // namespace ORB_SLAM3
