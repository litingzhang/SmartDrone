#include "LoopClosingBackend.h"

#include "LoopClosing.h"
#include "TrackingBackend.h"

namespace ORB_SLAM3 {

namespace {

class DefaultOrbLoopClosingBackend final : public IOrbLoopClosingBackend {
public:
  explicit DefaultOrbLoopClosingBackend(LoopClosing *loopClosing)
      : m_loopClosing(loopClosing) {}

  OrbLoopClosingStatus Status() const override {
    OrbLoopClosingStatus out;
    if (m_loopClosing == nullptr) {
      return out;
    }
    out.finished = m_loopClosing->isFinished();
    out.runningGlobalBundleAdjustment = m_loopClosing->isRunningGBA();
    return out;
  }

  OrbLoopClosingTimingStats TimingStats() const override {
    OrbLoopClosingTimingStats out;
#ifdef REGISTER_TIMES
    if (m_loopClosing == nullptr) {
      return out;
    }
    out.databaseQueryMs = m_loopClosing->vdDataQuery_ms;
    out.sim3EstimationMs = m_loopClosing->vdEstSim3_ms;
    out.placeRecognitionTotalMs = m_loopClosing->vdPRTotal_ms;
    out.mergeMapsMs = m_loopClosing->vdMergeMaps_ms;
    out.weldingBundleAdjustmentMs = m_loopClosing->vdWeldingBA_ms;
    out.mergeEssentialGraphMs = m_loopClosing->vdMergeOptEss_ms;
    out.mergeTotalMs = m_loopClosing->vdMergeTotal_ms;
    out.mergeKeyframes = m_loopClosing->vnMergeKFs;
    out.mergeMapPoints = m_loopClosing->vnMergeMPs;
    out.mergeExecutions = m_loopClosing->nMerges;
    out.loopFusionMs = m_loopClosing->vdLoopFusion_ms;
    out.loopEssentialGraphMs = m_loopClosing->vdLoopOptEss_ms;
    out.loopTotalMs = m_loopClosing->vdLoopTotal_ms;
    out.loopKeyframes = m_loopClosing->vnLoopKFs;
    out.loopExecutions = m_loopClosing->nLoop;
    out.globalBundleAdjustmentMs = m_loopClosing->vdGBA_ms;
    out.mapUpdateMs = m_loopClosing->vdUpdateMap_ms;
    out.fullGlobalBundleAdjustmentTotalMs = m_loopClosing->vdFGBATotal_ms;
    out.globalBundleAdjustmentKeyframes = m_loopClosing->vnGBAKFs;
    out.globalBundleAdjustmentMapPoints = m_loopClosing->vnGBAMPs;
    out.globalBundleAdjustmentExecutions = m_loopClosing->nFGBA_exec;
    out.globalBundleAdjustmentAborts = m_loopClosing->nFGBA_abort;
#endif
    return out;
  }

  void InsertKeyFrame(
      const OrbLoopClosingKeyFrameRequest &request) override {
    if (m_loopClosing != nullptr && request.keyframe != nullptr) {
      m_loopClosing->InsertKeyFrame(request.keyframe);
    }
  }

  void AbortGlobalBundleAdjustment() override {
    if (m_loopClosing != nullptr) {
      m_loopClosing->AbortGlobalBundleAdjustment();
    }
  }

  void RequestFinish() override {
    if (m_loopClosing != nullptr) {
      m_loopClosing->RequestFinish();
    }
  }

  void Reset(const OrbLoopClosingResetRequest &request) override {
    if (m_loopClosing == nullptr) {
      return;
    }
    if (request.activeMapOnly) {
      m_loopClosing->RequestResetActiveMap(request.map);
      return;
    }
    m_loopClosing->RequestReset();
  }

  void AttachRuntimePeers(
      const OrbLoopClosingRuntimePeers &peers) override {
    if (m_loopClosing != nullptr) {
      m_loopClosing->SetTrackingBackend(peers.tracking);
    }
  }

private:
  LoopClosing *m_loopClosing{nullptr};
};

} // namespace

std::unique_ptr<IOrbLoopClosingBackend>
CreateDefaultOrbLoopClosingBackend(LoopClosing *loopClosing) {
  return std::make_unique<DefaultOrbLoopClosingBackend>(loopClosing);
}

} // namespace ORB_SLAM3
