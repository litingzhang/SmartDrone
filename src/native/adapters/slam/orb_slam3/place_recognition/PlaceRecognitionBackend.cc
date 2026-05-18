#include "PlaceRecognitionBackend.h"

#include "KeyFrameDatabase.h"

namespace ORB_SLAM3 {

namespace {

class DefaultOrbPlaceRecognitionBackend final
    : public IOrbPlaceRecognitionBackend {
public:
  explicit DefaultOrbPlaceRecognitionBackend(KeyFrameDatabase *database)
      : m_database(database) {}

  void AddKeyFrame(
      const OrbPlaceRecognitionKeyFrameRequest &request) override {
    if (m_database != nullptr && request.keyFrame != nullptr) {
      m_database->add(request.keyFrame);
    }
  }

  std::vector<KeyFrame *> DetectRelocalizationCandidates(
      const OrbPlaceRecognitionRelocalizationRequest &request) override {
    if (m_database == nullptr || request.frame == nullptr ||
        request.map == nullptr) {
      return {};
    }
    return m_database->DetectRelocalizationCandidates(request.frame,
                                                     request.map);
  }

  OrbPlaceRecognitionLoopMergeCandidates DetectLoopMergeCandidates(
      const OrbPlaceRecognitionLoopMergeRequest &request) override {
    OrbPlaceRecognitionLoopMergeCandidates out;
    if (m_database == nullptr || request.keyFrame == nullptr) {
      return out;
    }
    m_database->DetectNBestCandidates(request.keyFrame, out.loopCandidates,
                                      out.mergeCandidates,
                                      request.candidateCount);
    return out;
  }

  void Clear() override {
    if (m_database != nullptr) {
      m_database->clear();
    }
  }

  void ClearMap(const OrbPlaceRecognitionResetRequest &request) override {
    if (m_database != nullptr && request.map != nullptr) {
      m_database->clearMap(request.map);
    }
  }

private:
  KeyFrameDatabase *m_database{nullptr};
};

} // namespace

std::unique_ptr<IOrbPlaceRecognitionBackend>
CreateDefaultOrbPlaceRecognitionBackend(KeyFrameDatabase *database) {
  return std::make_unique<DefaultOrbPlaceRecognitionBackend>(database);
}

} // namespace ORB_SLAM3
