#include "common/environment.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <string>

namespace SmartDrone::Common {
namespace {

const char *EnvValue(const char *name)
{
    const char *value = std::getenv(name);
    return value != nullptr && value[0] != '\0' ? value : nullptr;
}

} // namespace

bool EnvVarIsUnsetOrEmpty(const char *name)
{
    return EnvValue(name) == nullptr;
}

bool ParseFlagValue(const char *value, bool fallback)
{
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    std::string text(value);
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return !(text == "0" || text == "false" || text == "off" ||
             text == "no" || text == "disabled");
}

std::string EnvStringValue(const char *name, const char *fallback)
{
    const char *value = EnvValue(name);
    return value != nullptr ? value : fallback;
}

int EnvIntValue(const char *name, int fallback)
{
    return ParseIntValue(EnvValue(name), fallback);
}

int EnvIntValueClamped(const char *name, int fallback, int minValue,
                       int maxValue)
{
    const char *value = EnvValue(name);
    if (value == nullptr) {
        return std::clamp(fallback, minValue, maxValue);
    }

    int parsed = fallback;
    if (!TryParseIntPrefix(value, 10, parsed)) {
        return std::clamp(fallback, minValue, maxValue);
    }
    return std::clamp(parsed, minValue, maxValue);
}

float EnvFloatValue(const char *name, float fallback)
{
    return ParseFloatValue(EnvValue(name), fallback);
}

float EnvFloatValueClamped(const char *name, float fallback, float minValue,
                           float maxValue)
{
    const float parsed = EnvFloatValue(name, fallback);
    if (!std::isfinite(parsed)) {
        return std::clamp(fallback, minValue, maxValue);
    }
    return std::clamp(parsed, minValue, maxValue);
}

double EnvDoubleValue(const char *name, double fallback)
{
    double parsed = fallback;
    return TryParseDoublePrefix(EnvValue(name), parsed) &&
                   std::isfinite(parsed)
               ? parsed
               : fallback;
}

bool EnvFlagEnabled(const char *name, bool fallback)
{
    return ParseFlagValue(EnvValue(name), fallback);
}

void SetEnvVar(const char *name, const char *value)
{
#if defined(_WIN32)
    _putenv_s(name, value);
#else
    setenv(name, value, 1);
#endif
}

void UnsetEnvVar(const char *name)
{
#if defined(_WIN32)
    _putenv_s(name, "");
#else
    unsetenv(name);
#endif
}

void SetEnvVarIfUnset(const char *name, const char *value)
{
    if (!EnvVarIsUnsetOrEmpty(name)) {
        return;
    }
    SetEnvVar(name, value);
}

} // namespace SmartDrone::Common
