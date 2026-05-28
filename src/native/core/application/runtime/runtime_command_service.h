#pragma once

#include <cstdint>

#include "core/application/runtime/runtime_command_result.h"
#include "core/application/runtime/runtime_config_update.h"
#include "core/domain/runtime_mode.h"

namespace SmartDrone::Core::Application {

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
    Domain::RuntimeSelection selection{};
};

class IRuntimeCommandTarget {
  public:
    virtual ~IRuntimeCommandTarget() = default;

    virtual CommandResult ExecuteAction(const RuntimeAction &action) = 0;
    virtual CommandResult ApplyConfig(const ConfigUpdate &update) = 0;
};

class RuntimeCommandService {
  public:
    explicit RuntimeCommandService(IRuntimeCommandTarget &target);

    CommandResult ExecuteAction(const RuntimeAction &action);
    CommandResult ApplyConfig(const ConfigUpdate &update);

  private:
    IRuntimeCommandTarget &m_target;
};

} // namespace SmartDrone::Core::Application
