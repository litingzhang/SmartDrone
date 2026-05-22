#include "app/bootstrap/signal_handlers.h"

#include <csignal>

#include "common/runtime_state.h"

namespace SmartDrone::app::bootstrap {

void InstallSignalHandlers()
{
    signal(SIGINT, SmartDrone::common::SigIntHandler);
    signal(SIGTERM, SmartDrone::common::SigIntHandler);
}

} // namespace SmartDrone::app::bootstrap
