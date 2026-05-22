#pragma once

#include "adapters/slam/slam_engine_adapter.h"

namespace SmartDrone::adapters::slam {

class SlamEngineAccess final {
  public:
    static core::ports::ISlamTrackingBackend *TrackingBackend(SlamEngineAdapter &engine)
    {
        return engine.m_trackingBackend.get();
    }
    static core::ports::ISlamBackendLifecycle *BackendLifecycle(SlamEngineAdapter &engine)
    {
        return engine.m_trackingBackend.get();
    }
    static core::ports::ISlamTrackingStatusProvider *TrackingStatus(SlamEngineAdapter &engine)
    {
        return engine.m_trackingBackend.get();
    }
    static core::ports::ISlamDescriptorProviderSource *DescriptorProviders(SlamEngineAdapter &engine)
    {
        return engine.m_trackingBackend.get();
    }
    static core::ports::ITrackedVisualDataProvider *TrackedVisualDataProvider(SlamEngineAdapter &engine)
    {
        return engine.m_trackingBackend.get();
    }
    static SlamModeSharedState &ModeState(SlamEngineAdapter &engine)
    {
        return *engine.m_modeState;
    }
    static SlamInputMode InputMode(const SlamEngineAdapter &engine)
    {
        return engine.m_inputMode;
    }
    static bool UseImu(const SlamEngineAdapter &engine)
    {
        return engine.m_useImu;
    }

    static void StabilizeOutputPose(SlamEngineAdapter &engine, core::ports::PoseEstimate &pose, bool &poseValid,
                                    double timestampSec, int trackingState)
    {
        engine.StabilizeOutputPose(pose, poseValid, timestampSec, trackingState);
    }

    static void MaintainRealtimePoseContinuity(SlamEngineAdapter &engine, core::ports::PoseEstimate &pose,
                                               bool &poseValid, double timestampSec, int trackingState)
    {
        engine.MaintainRealtimePoseContinuity(pose, poseValid, timestampSec, trackingState);
    }

    static void GateRealtimePoseQuality(SlamEngineAdapter &engine, core::ports::SlamOutput &out, double timestampSec)
    {
        engine.GateRealtimePoseQuality(out, timestampSec);
    }
};

} // namespace SmartDrone::adapters::slam
