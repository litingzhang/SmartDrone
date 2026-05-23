#include "adapters/slam/superpoint/superpoint_runtime_options.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "adapters/slam/engine/slam_env.h"

namespace SmartDrone::Adapters::Slam {
namespace {

int EnvIntAtLeastOne(const char *name, int fallback)
{
    return std::max(1, EnvIntValue(name, fallback));
}

float EnvFloatFinite(const char *name, float fallback)
{
    const float value = EnvFloatValue(name, fallback);
    return std::isfinite(value) ? value : fallback;
}

} // namespace

SuperPointTensorRtRuntimeOptions LoadSuperPointTensorRtRuntimeOptions()
{
    SuperPointTensorRtRuntimeOptions options;
    options.inputMaxWidth =
        EnvIntAtLeastOne("SMART_DRONE_SUPERPOINT_INPUT_MAX_WIDTH", 640);
    options.inputMaxHeight =
        EnvIntAtLeastOne("SMART_DRONE_SUPERPOINT_INPUT_MAX_HEIGHT", 480);
    options.lightGluePoints =
        EnvIntAtLeastOne("SMART_DRONE_LIGHTGLUE_POINTS", 512);
    options.lightGlueMinScore =
        EnvFloatFinite("SMART_DRONE_LIGHTGLUE_MIN_SCORE", 0.02f);
    options.lightGlueMaxYDiffPx =
        EnvFloatFinite("SMART_DRONE_LIGHTGLUE_MAX_Y_DIFF_PX", 1.5f);
    options.lightGlueMinDisparityPx =
        EnvFloatFinite("SMART_DRONE_LIGHTGLUE_MIN_DISPARITY_PX", 0.8f);
    options.lightGlueEmptyDisableThreshold =
        EnvIntAtLeastOne("SMART_DRONE_LIGHTGLUE_EMPTY_DISABLE_THRESHOLD", 3);
    options.lightGlueLowYieldDisableThreshold =
        EnvIntAtLeastOne("SMART_DRONE_LIGHTGLUE_LOW_YIELD_DISABLE_THRESHOLD", 3);
    options.lightGlueLowYieldMinPairs =
        EnvIntAtLeastOne("SMART_DRONE_LIGHTGLUE_LOW_YIELD_MIN_PAIRS", 8);
    options.lightGlueEmptyCooldownFrames =
        EnvIntAtLeastOne("SMART_DRONE_LIGHTGLUE_EMPTY_COOLDOWN_FRAMES", 120);
    return options;
}

} // namespace SmartDrone::Adapters::Slam
