#pragma once

#include <cstddef>
#include <string>

namespace SmartDrone::Adapters::Slam {

bool EnvFlagEnabled(const char *name, bool defaultValue);
int EnvIntValue(const char *name, int defaultValue);
float EnvFloatValue(const char *name, float defaultValue);
std::string EnvStringValue(const char *name, const char *defaultValue);
size_t EnvSizeValueClamped(const char *name, size_t defaultValue,
                           size_t minValue, size_t maxValue);
int EnvIntValueClamped(const char *name, int defaultValue, int minValue,
                       int maxValue);
float EnvFloatValueClamped(const char *name, float defaultValue,
                           float minValue, float maxValue);

} // namespace SmartDrone::Adapters::Slam
