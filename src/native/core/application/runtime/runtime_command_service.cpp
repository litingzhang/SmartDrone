#include "core/application/runtime/runtime_command_service.h"

namespace SmartDrone::Core::Application {

RuntimeCommandService::RuntimeCommandService(IRuntimeCommandTarget &target)
    : m_target(target)
{
}

CommandResult RuntimeCommandService::ExecuteAction(const RuntimeAction &action)
{
    return m_target.ExecuteAction(action);
}

CommandResult RuntimeCommandService::ApplyConfig(const ConfigUpdate &update)
{
    return m_target.ApplyConfig(update);
}

} // namespace SmartDrone::Core::Application
