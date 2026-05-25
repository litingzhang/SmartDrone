#pragma once

#include <string>

#include "common/numeric_parse.h"

namespace SmartDrone::Common {

bool EnvVarIsUnsetOrEmpty(const char *name);
bool ParseFlagValue(const char *value, bool fallback);
std::string EnvStringValue(const char *name, const char *fallback);
int EnvIntValue(const char *name, int fallback);
int EnvIntValueClamped(const char *name, int fallback, int minValue,
                       int maxValue);
float EnvFloatValue(const char *name, float fallback);
float EnvFloatValueClamped(const char *name, float fallback, float minValue,
                           float maxValue);
double EnvDoubleValue(const char *name, double fallback);
bool EnvFlagEnabled(const char *name, bool fallback);
void SetEnvVar(const char *name, const char *value);
void UnsetEnvVar(const char *name);
void SetEnvVarIfUnset(const char *name, const char *value);

} // namespace SmartDrone::Common
