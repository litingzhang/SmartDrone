#include "core/application/runtime/system_runtime_messages.h"

#include <utility>

namespace SmartDrone::Core::Application {

const bool SYSTEM_RUNTIME_PULSE_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<SystemRuntimePulse>(
        "SystemRuntimePulse");

void DrainSystemRuntimePulse(Epg::TaskContext &context)
{
    if (!context.InputExists(SYSTEM_RUNTIME_PULSE_PORT)) {
        return;
    }
    (void)context.TryPopLatest<SystemRuntimePulse>(SYSTEM_RUNTIME_PULSE_PORT);
}

void PushSystemRuntimePulse(Epg::TaskContext &context,
                            std::uint64_t &sequence)
{
    if (!context.OutputExists(SYSTEM_RUNTIME_PULSE_PORT)) {
        return;
    }
    auto pulse = context.Make<SystemRuntimePulse>();
    pulse->sequence = ++sequence;
    context.Push(SYSTEM_RUNTIME_PULSE_PORT, std::move(pulse));
}

} // namespace SmartDrone::Core::Application
