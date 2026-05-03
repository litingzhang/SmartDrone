#pragma once

#include "adapters/slam/orbslam3_engine.h"

namespace smartdrone::adapters::slam {

class OrbSlam3EngineAccess final {
  public:
    static ORB_SLAM3::System *System(OrbSlam3Engine &engine) { return engine.m_system.get(); }
    static SlamModeSharedState &ModeState(OrbSlam3Engine &engine) { return *engine.m_modeState; }
    static OrbInputMode InputMode(const OrbSlam3Engine &engine) { return engine.m_inputMode; }
    static bool UseImu(const OrbSlam3Engine &engine) { return engine.m_useImu; }

    static void StabilizeOutputPose(OrbSlam3Engine &engine, core::ports::PoseEstimate &pose, bool &poseValid,
                                    double timestampSec, int trackingState)
    {
        engine.StabilizeOutputPose(pose, poseValid, timestampSec, trackingState);
    }
};

} // namespace smartdrone::adapters::slam
