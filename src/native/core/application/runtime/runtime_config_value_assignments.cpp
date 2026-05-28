#include "core/application/runtime/runtime_config_value_assignments.h"

#include <cstdint>
#include <variant>

namespace SmartDrone::Core::Application {

CommandResult OkResult()
{
    return {true, ""};
}

CommandResult TypeMismatchResult(const char *key)
{
    return {false, std::string(key) + " type mismatch"};
}

CommandResult AssignStrictInt(const ConfigValue &value, int &out,
                              const char *key)
{
    if (const auto *typedValue = std::get_if<int64_t>(&value)) {
        out = static_cast<int>(*typedValue);
        return OkResult();
    }
    return TypeMismatchResult(key);
}

CommandResult AssignNumericInt(const ConfigValue &value, int &out,
                               const char *key)
{
    if (const auto *typedValue = std::get_if<int64_t>(&value)) {
        out = static_cast<int>(*typedValue);
        return OkResult();
    }
    if (const auto *typedValue = std::get_if<double>(&value)) {
        out = static_cast<int>(*typedValue);
        return OkResult();
    }
    return TypeMismatchResult(key);
}

CommandResult AssignFloat(const ConfigValue &value, float &out,
                          const char *key)
{
    if (const auto *typedValue = std::get_if<double>(&value)) {
        out = static_cast<float>(*typedValue);
        return OkResult();
    }
    if (const auto *typedValue = std::get_if<int64_t>(&value)) {
        out = static_cast<float>(*typedValue);
        return OkResult();
    }
    return TypeMismatchResult(key);
}

CommandResult AssignBool(const ConfigValue &value, bool &out,
                         const char *key)
{
    if (const auto *typedValue = std::get_if<bool>(&value)) {
        out = *typedValue;
        return OkResult();
    }
    return TypeMismatchResult(key);
}

CommandResult AssignString(const ConfigValue &value, std::string &out,
                           const char *key)
{
    if (const auto *typedValue = std::get_if<std::string>(&value)) {
        out = *typedValue;
        return OkResult();
    }
    return TypeMismatchResult(key);
}

} // namespace SmartDrone::Core::Application
