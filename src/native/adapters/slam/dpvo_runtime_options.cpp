#include "adapters/slam/dpvo_runtime_options.h"

#include <algorithm>

#include "adapters/slam/slam_env.h"

namespace SmartDrone::Adapters::Slam {

DpvoGraphRuntimeOptions LoadDpvoGraphRuntimeOptions()
{
    DpvoGraphRuntimeOptions options;
    options.persistentEdges =
        EnvFlagEnabled("SMART_DRONE_DPVO_PERSISTENT_EDGES", false);
    options.keyframeRemovalEnabled =
        EnvFlagEnabled("SMART_DRONE_DPVO_KEYFRAME", false);
    options.capRebuiltEdges =
        EnvFlagEnabled("SMART_DRONE_DPVO_CAP_REBUILT_EDGES", false);
    options.maxActiveEdges =
        std::clamp(EnvIntValue("SMART_DRONE_DPVO_MAX_EDGES", 1024), 128, 4096);
    return options;
}

DpvoStereoDepthOptions LoadDpvoStereoDepthOptions(int rightWidth)
{
    DpvoStereoDepthOptions options;
    options.maxDisparity =
        std::clamp(EnvIntValue("SMART_DRONE_DPVO_STEREO_MAX_DISP", 36), 2,
                   std::max(2, rightWidth / 3));
    options.minScore = std::clamp(
        EnvFloatValue("SMART_DRONE_DPVO_STEREO_NCC_MIN", 0.05f), -1.0f, 1.0f);
    options.minMargin = std::clamp(
        EnvFloatValue("SMART_DRONE_DPVO_STEREO_NCC_MARGIN", 0.01f), 0.0f, 1.0f);
    return options;
}

} // namespace SmartDrone::Adapters::Slam
