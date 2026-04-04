#include "app/bootstrap/signal_handlers.h"

#include <csignal>

#include "adapters/camera/libcamera_ov9281/stereo_ov9281.h"

namespace smartdrone::app::bootstrap {

void InstallSignalHandlers()
{
    signal(SIGINT, SigIntHandler);
    signal(SIGTERM, SigIntHandler);
}

} // namespace smartdrone::app::bootstrap
