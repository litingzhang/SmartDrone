#include "app/bootstrap/signal_handlers.h"

#include <csignal>

#include "common/runtime_state.h"

namespace SmartDrone::App::Bootstrap {

void InstallSignalHandlers()
{
    signal(SIGINT, SmartDrone::Common::SigIntHandler);
    signal(SIGTERM, SmartDrone::Common::SigIntHandler);
}

} // namespace SmartDrone::App::Bootstrap
