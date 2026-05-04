#pragma once

#include "adapters/slam/slam_engine_adapter.h"

namespace smartdrone::adapters::slam {

class SlamEngineAccess final {
  public:
    static ORB_SLAM3::System *System(SlamEngineAdapter &engine) { return engine.m_system.get(); }
    static SlamModeSharedState &ModeState(SlamEngineAdapter &engine) { return *engine.m_modeState; }
    static SlamInputMode InputMode(const SlamEngineAdapter &engine) { return engine.m_inputMode; }
    static bool UseImu(const SlamEngineAdapter &engine) { return engine.m_useImu; }

    static void StabilizeOutputPose(SlamEngineAdapter &engine, core::ports::PoseEstimate &pose, bool &poseValid,
                                    double timestampSec, int trackingState)
    {
        engine.StabilizeOutputPose(pose, poseValid, timestampSec, trackingState);
    }
};

} // namespace smartdrone::adapters::slam
