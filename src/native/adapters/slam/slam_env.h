#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <string>

namespace smartdrone::adapters::slam {

inline const char *EnvValueWithStereoFeatureFallback(const char *name) {
  const char *value = std::getenv(name);
  if (value != nullptr && value[0] != '\0') {
    return value;
  }

  const std::string envName(name ? name : "");
  constexpr const char *stereoFeaturePrefix = "SMART_DRONE_STEREO_FEATURE_";
  constexpr const char *legacyExternalStereoPrefix =
      "SMART_DRONE_EXTERNAL_STEREO_";
  if (envName.rfind(stereoFeaturePrefix, 0) != 0) {
    return nullptr;
  }

  const std::string legacyName =
      std::string(legacyExternalStereoPrefix) +
      envName.substr(std::string(stereoFeaturePrefix).size());
  value = std::getenv(legacyName.c_str());
  return value != nullptr && value[0] != '\0' ? value : nullptr;
}

inline bool EnvFlagEnabled(const char *name, bool defaultValue) {
  const char *value = EnvValueWithStereoFeatureFallback(name);
  if (value == nullptr || value[0] == '\0') {
    return defaultValue;
  }
  const std::string text(value);
  return !(text == "0" || text == "false" || text == "FALSE" || text == "off" ||
           text == "OFF");
}

inline int EnvIntValue(const char *name, int defaultValue) {
  const char *value = EnvValueWithStereoFeatureFallback(name);
  if (value == nullptr || value[0] == '\0') {
    return defaultValue;
  }
  char *end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  return end != value ? static_cast<int>(parsed) : defaultValue;
}

inline float EnvFloatValue(const char *name, float defaultValue) {
  const char *value = EnvValueWithStereoFeatureFallback(name);
  if (value == nullptr || value[0] == '\0') {
    return defaultValue;
  }
  char *end = nullptr;
  const float parsed = std::strtof(value, &end);
  return end != value ? parsed : defaultValue;
}

inline std::string EnvStringValue(const char *name, const char *defaultValue) {
  const char *value = EnvValueWithStereoFeatureFallback(name);
  if (value == nullptr || value[0] == '\0') {
    return defaultValue;
  }
  return value;
}

inline size_t EnvSizeValueClamped(const char *name, size_t defaultValue,
                                  size_t minValue, size_t maxValue) {
  const char *value = EnvValueWithStereoFeatureFallback(name);
  if (value == nullptr || value[0] == '\0') {
    return defaultValue;
  }
  char *end = nullptr;
  const unsigned long parsed = std::strtoul(value, &end, 10);
  if (end == value) {
    return defaultValue;
  }
  return std::clamp(static_cast<size_t>(parsed), minValue, maxValue);
}

inline int EnvIntValueClamped(const char *name, int defaultValue, int minValue,
                              int maxValue) {
  const char *value = EnvValueWithStereoFeatureFallback(name);
  if (value == nullptr || value[0] == '\0') {
    return defaultValue;
  }
  char *end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value) {
    return defaultValue;
  }
  return std::clamp(static_cast<int>(parsed), minValue, maxValue);
}

inline float EnvFloatValueClamped(const char *name, float defaultValue,
                                  float minValue, float maxValue) {
  const float parsed = EnvFloatValue(name, defaultValue);
  if (!std::isfinite(parsed)) {
    return std::clamp(defaultValue, minValue, maxValue);
  }
  return std::clamp(parsed, minValue, maxValue);
}

} // namespace smartdrone::adapters::slam
