#include <string>

#include "app/bootstrap/runtime_host.h"
#include "app/bootstrap/signal_handlers.h"
#include "core/application/config/app_args.h"

int main(int argc, char **argv)
{
    smartdrone::app::bootstrap::InstallSignalHandlers();
    smartdrone::core::application::UnifiedConfig cfg{};
    cfg.app = ParseAppConfig(argc, argv);
    ArgReader args(argc, argv);
    cfg.calib.root = args.GetString("--calib-root", "./calib_runs");
    cfg.calib.maxFrames = args.GetInt("--calib-max-frames", 0);
    const std::string autoModeText = args.GetString("--auto-mode", "idle");

    smartdrone::app::bootstrap::RuntimeHost runtimeHost;
    return runtimeHost.Run(cfg, autoModeText);
}
