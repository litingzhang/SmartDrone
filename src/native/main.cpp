#include <string>

#include "app/bootstrap/runtime_host.h"
#include "app/bootstrap/signal_handlers.h"
#include "core/application/config/app_args.h"
#include "common/environment.h"

int main(int argc, char **argv)
{
    SmartDrone::App::Bootstrap::InstallSignalHandlers();
    SmartDrone::Core::Application::UnifiedConfig cfg{};
    cfg.app = ParseAppConfig(argc, argv);
    ArgReader args(argc, argv);
    cfg.calib.root = args.GetString("--calib-root", "./calib_runs");
    cfg.calib.maxFrames = args.GetInt("--calib-max-frames", 0);
    const std::string autoModeDefault = SmartDrone::Common::EnvStringValue(
        "SMART_DRONE_AUTO_MODE", "idle");
    const std::string autoModeText =
        args.GetString("--auto-mode", autoModeDefault.c_str());

    SmartDrone::App::Bootstrap::RuntimeHost runtimeHost;
    return runtimeHost.Run(cfg, autoModeText);
}
