#include "adapters/slam/engine/slam_env.h"

#include <algorithm>
#include <cmath>

#include "common/environment.h"

namespace SmartDrone::Adapters::Slam {

namespace {

std::string LegacyStereoFeatureName(const char *name)
{
    const std::string envName(name ? name : "");
    constexpr const char *stereoFeaturePrefix = "SMART_DRONE_STEREO_FEATURE_";
    constexpr const char *legacyExternalStereoPrefix =
        "SMART_DRONE_EXTERNAL_STEREO_";
    if (envName.rfind(stereoFeaturePrefix, 0) != 0) {
        return {};
    }
    return std::string(legacyExternalStereoPrefix) +
        envName.substr(std::string(stereoFeaturePrefix).size());
}

std::string EnvStringWithStereoFeatureFallback(const char *name,
                                               const char *defaultValue)
{
    if (!SmartDrone::Common::EnvVarIsUnsetOrEmpty(name)) {
        return SmartDrone::Common::EnvStringValue(name, defaultValue);
    }

    const std::string legacyName = LegacyStereoFeatureName(name);
    if (legacyName.empty()) {
        return defaultValue;
    }
    return SmartDrone::Common::EnvStringValue(legacyName.c_str(),
                                              defaultValue);
}

} // namespace

bool EnvFlagEnabled(const char *name, bool defaultValue)
{
    const std::string value =
        EnvStringWithStereoFeatureFallback(name, "");
    return SmartDrone::Common::ParseFlagValue(value.c_str(), defaultValue);
}

int EnvIntValue(const char *name, int defaultValue)
{
    const std::string value =
        EnvStringWithStereoFeatureFallback(name, "");
    return SmartDrone::Common::ParseIntValue(value.c_str(), defaultValue);
}

float EnvFloatValue(const char *name, float defaultValue)
{
    const std::string value =
        EnvStringWithStereoFeatureFallback(name, "");
    return SmartDrone::Common::ParseFloatValue(value.c_str(), defaultValue);
}

std::string EnvStringValue(const char *name, const char *defaultValue)
{
    return EnvStringWithStereoFeatureFallback(name, defaultValue);
}

size_t EnvSizeValueClamped(const char *name, size_t defaultValue,
                           size_t minValue, size_t maxValue)
{
    const std::string value =
        EnvStringWithStereoFeatureFallback(name, "");
    return SmartDrone::Common::ParseSizeValueClamped(
        value.c_str(), defaultValue, minValue, maxValue);
}

int EnvIntValueClamped(const char *name, int defaultValue, int minValue,
                       int maxValue)
{
    return std::clamp(EnvIntValue(name, defaultValue), minValue, maxValue);
}

float EnvFloatValueClamped(const char *name, float defaultValue,
                           float minValue, float maxValue)
{
    const float parsed = EnvFloatValue(name, defaultValue);
    if (!std::isfinite(parsed)) {
        return std::clamp(defaultValue, minValue, maxValue);
    }
    return std::clamp(parsed, minValue, maxValue);
}

} // namespace SmartDrone::Adapters::Slam
