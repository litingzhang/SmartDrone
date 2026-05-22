#pragma once

#include "adapters/slam/slam_engine_adapter.h"

namespace SmartDrone::Adapters::Slam {

class SlamEngineAccess final {
  public:
    static Core::Ports::ISlamTrackingBackend *TrackingBackend(SlamEngineAdapter &engine)
    {
        return engine.m_trackingBackend.get();
    }
    static Core::Ports::ISlamBackendLifecycle *BackendLifecycle(SlamEngineAdapter &engine)
    {
        return engine.m_trackingBackend.get();
    }
    static Core::Ports::ISlamTrackingStatusProvider *TrackingStatus(SlamEngineAdapter &engine)
    {
        return engine.m_trackingBackend.get();
    }
    static Core::Ports::ISlamDescriptorProviderSource *DescriptorProviders(SlamEngineAdapter &engine)
    {
        return engine.m_trackingBackend.get();
    }
    static Core::Ports::ITrackedVisualDataProvider *TrackedVisualDataProvider(SlamEngineAdapter &engine)
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

    static void StabilizeOutputPose(SlamEngineAdapter &engine, Core::Ports::PoseEstimate &pose, bool &poseValid,
                                    double timestampSec, int trackingState)
    {
        engine.StabilizeOutputPose(pose, poseValid, timestampSec, trackingState);
    }

    static void MaintainRealtimePoseContinuity(SlamEngineAdapter &engine, Core::Ports::PoseEstimate &pose,
                                               bool &poseValid, double timestampSec, int trackingState)
    {
        engine.MaintainRealtimePoseContinuity(pose, poseValid, timestampSec, trackingState);
    }

    static void GateRealtimePoseQuality(SlamEngineAdapter &engine, Core::Ports::SlamOutput &out, double timestampSec)
    {
        engine.GateRealtimePoseQuality(out, timestampSec);
    }
};

} // namespace SmartDrone::Adapters::Slam
