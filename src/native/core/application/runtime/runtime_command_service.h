#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <variant>

#include "core/domain/runtime_mode.h"

namespace smartdrone::core::application {

struct RuntimeAction {
    enum class Type : uint8_t {
        StartRuntime,
        StopRuntime,
        CleanCalibration,
        ForceRestart,
        ResetMap,
        SaveMap,
    };

    Type type{Type::StartRuntime};
    domain::RuntimeSelection selection{};
};

using ConfigValue = std::variant<int64_t, double, bool, std::string>;

struct ConfigUpdate {
    std::unordered_map<std::string, ConfigValue> values;
};

struct CommandResult {
    bool ok{false};
    std::string message;
};

class IRuntimeCommandTarget {
public:
    virtual ~IRuntimeCommandTarget() = default;

    virtual CommandResult ExecuteAction(const RuntimeAction& action) = 0;
    virtual CommandResult ApplyConfig(const ConfigUpdate& update) = 0;
};

class RuntimeCommandService {
public:
    explicit RuntimeCommandService(IRuntimeCommandTarget& target);

    CommandResult ExecuteAction(const RuntimeAction& action);
    CommandResult ApplyConfig(const ConfigUpdate& update);

private:
    IRuntimeCommandTarget& m_target;
};

}  // namespace smartdrone::core::application
