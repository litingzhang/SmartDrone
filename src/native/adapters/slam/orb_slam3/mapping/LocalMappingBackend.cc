#include "LocalMappingBackend.h"

#include "LocalMapping.h"
#include "TrackingBackend.h"

namespace ORB_SLAM3 {

namespace {

class DefaultOrbLocalMappingBackend final : public IOrbLocalMappingBackend {
public:
  explicit DefaultOrbLocalMappingBackend(LocalMapping *localMapping)
      : m_localMapping(localMapping) {}

  OrbLocalMappingStatus Status() const override {
    OrbLocalMappingStatus out;
    if (m_localMapping == nullptr) {
      return out;
    }
    out.stopped = m_localMapping->isStopped();
    out.stopRequested = m_localMapping->stopRequested();
    out.acceptingKeyframes = m_localMapping->AcceptKeyFrames();
    out.initializing = m_localMapping->IsInitializing();
    out.finished = m_localMapping->isFinished();
    out.badImu = m_localMapping->mbBadImu;
    out.farPoints = m_localMapping->mbFarPoints;
    out.farPointThreshold = m_localMapping->mThFarPoints;
    out.keyframesInQueue = m_localMapping->KeyframesInQueue();
    out.matchesInliers = m_localMapping->mnMatchesInliers;
    out.firstTimestamp = m_localMapping->mFirstTs;
    out.currentKeyframeTime = m_localMapping->GetCurrKFTime();
    return out;
  }

  OrbLocalMappingDebugSnapshot DebugSnapshot() const override {
    OrbLocalMappingDebugSnapshot out;
    if (m_localMapping == nullptr) {
      return out;
    }
    out.available = true;
    out.initSection = m_localMapping->mInitSect;
    out.scale = m_localMapping->mScale;
    out.gravityRotation = m_localMapping->mRwg;
    out.gyroBias = m_localMapping->mbg;
    out.accBias = m_localMapping->mba;
    out.inertialCovariance = m_localMapping->mcovInertial;
    out.costTime = m_localMapping->mCostTime;
    out.initTime = m_localMapping->mInitTime;
    return out;
  }

  OrbLocalMappingTimingStats TimingStats() const override {
    OrbLocalMappingTimingStats out;
#ifdef REGISTER_TIMES
    if (m_localMapping == nullptr) {
      return out;
    }
    out.keyframeInsertMs = m_localMapping->vdKFInsert_ms;
    out.mapPointCullingMs = m_localMapping->vdMPCulling_ms;
    out.mapPointCreationMs = m_localMapping->vdMPCreation_ms;
    out.localBundleAdjustmentMs = m_localMapping->vdLBA_ms;
    out.localBundleAdjustmentSyncMs = m_localMapping->vdLBASync_ms;
    out.keyframeCullingMs = m_localMapping->vdKFCulling_ms;
    out.keyframeCullingSyncMs = m_localMapping->vdKFCullingSync_ms;
    out.totalMs = m_localMapping->vdLMTotal_ms;
    out.localBundleAdjustmentEdges = m_localMapping->vnLBA_edges;
    out.localBundleAdjustmentOptimizedKeyframes = m_localMapping->vnLBA_KFopt;
    out.localBundleAdjustmentFixedKeyframes = m_localMapping->vnLBA_KFfixed;
    out.localBundleAdjustmentMapPoints = m_localMapping->vnLBA_MPs;
    out.localBundleAdjustmentExecutions = m_localMapping->nLBA_exec;
    out.localBundleAdjustmentAborts = m_localMapping->nLBA_abort;
#endif
    return out;
  }

  void SetMatchesInliers(int matchesInliers) override {
    if (m_localMapping != nullptr) {
      m_localMapping->mnMatchesInliers = matchesInliers;
    }
  }

  void InsertKeyFrame(
      const OrbLocalMappingKeyFrameRequest &request) override {
    if (m_localMapping != nullptr && request.keyframe != nullptr) {
      m_localMapping->InsertKeyFrame(request.keyframe);
    }
  }

  void InterruptBundleAdjustment() override {
    if (m_localMapping != nullptr) {
      m_localMapping->InterruptBA();
    }
  }

  bool SetNotStop(bool flag) override {
    return m_localMapping != nullptr && m_localMapping->SetNotStop(flag);
  }

  void SetFirstTimestamp(double timestamp) override {
    if (m_localMapping != nullptr) {
      m_localMapping->mFirstTs = timestamp;
    }
  }

  void RequestStop() override {
    if (m_localMapping != nullptr) {
      m_localMapping->RequestStop();
    }
  }

  void RequestFinish() override {
    if (m_localMapping != nullptr) {
      m_localMapping->RequestFinish();
    }
  }

  void Release() override {
    if (m_localMapping != nullptr) {
      m_localMapping->Release();
    }
  }

  void EmptyQueue() override {
    if (m_localMapping != nullptr) {
      m_localMapping->EmptyQueue();
    }
  }

  void AttachRuntimePeers(
      const OrbLocalMappingRuntimePeers &peers) override {
    if (m_localMapping == nullptr) {
      return;
    }
    if (peers.tracking != nullptr) {
      m_localMapping->SetTrackingBackend(peers.tracking);
    }
    if (peers.loopCloser != nullptr) {
      m_localMapping->SetLoopCloser(peers.loopCloser);
    }
  }

  void ApplyRuntimeConfig(
      const OrbLocalMappingRuntimeConfig &config) override {
    if (m_localMapping == nullptr) {
      return;
    }
    m_localMapping->mInitFr = config.initFrame;
    m_localMapping->mThFarPoints = config.farPointThreshold;
    m_localMapping->mbFarPoints = config.farPointThreshold != 0.0f;
  }

  void Reset(const OrbLocalMappingResetRequest &request) override {
    if (m_localMapping == nullptr) {
      return;
    }
    if (request.activeMapOnly) {
      m_localMapping->RequestResetActiveMap(request.map);
      return;
    }
    m_localMapping->RequestReset();
  }

private:
  LocalMapping *m_localMapping{nullptr};
};

} // namespace

std::unique_ptr<IOrbLocalMappingBackend>
CreateDefaultOrbLocalMappingBackend(LocalMapping *localMapping) {
  return std::make_unique<DefaultOrbLocalMappingBackend>(localMapping);
}

} // namespace ORB_SLAM3
