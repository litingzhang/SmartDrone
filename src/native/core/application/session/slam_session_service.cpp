#include "core/application/session/slam_session_service.h"

#include "core/application/session/slam_frame_processor.h"
#include "core/application/session/slam_session_runtime.h"

namespace smartdrone::core::application {

bool RunSlamSession(const UnifiedConfig &cfg, LiveRuntimeTuning &tuning, Px4MavlinkGateway &mav,
                    std::atomic<bool> &stop, LivePoseState &livePose, std::atomic<bool> &runningFlag)
{
    SlamSessionRuntime runtime(cfg, tuning, mav, livePose, stop, runningFlag);
    if (!runtime.Start())
        return false;
    SlamFrameProcessor &frameProcessor = runtime.FrameProcessor();
    while (runningFlag.load() && !stop.load()) {
        if (!runtime.WaitForImuReady())
            continue;
        if (frameProcessor.ProcessNextFrame(runtime.sessionOk) == SlamFrameProcessor::StepResult::SessionAbort)
            break;
    }
    runtime.Stop();
    return runtime.sessionOk;
}

} // namespace smartdrone::core::application
