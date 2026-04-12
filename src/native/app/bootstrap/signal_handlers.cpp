#include "app/bootstrap/signal_handlers.h"

#include <csignal>

#include "common/runtime_state.h"

namespace smartdrone::app::bootstrap {

void InstallSignalHandlers()
{
    signal(SIGINT, smartdrone::common::SigIntHandler);
    signal(SIGTERM, smartdrone::common::SigIntHandler);
}

} // namespace smartdrone::app::bootstrap
