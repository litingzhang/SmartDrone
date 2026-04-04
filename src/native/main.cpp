#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <cstdio>
#include <ctime>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "System.h"
#include "core/application/app_args.h"
#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "adapters/camera/libcamera_ov9281/stereo_ov9281.h"
#include "core/application/live_pose_state.h"
#include "core/application/payload_builders.h"
#include "core/application/px4_udp_hooks.h"
#include "core/application/runtime_app_types.h"
#include "core/application/runtime_controller.h"
#include "core/application/runtime_session_common.h"
#include "core/application/slam_session_service.h"
#include "core/application/calib_session_service.h"
#include "core/application/calib_storage_helpers.h"
#include "core/application/udp_command_thread.h"

#include "common/tlv/udp_server.h"

namespace {
using ControllerMode = smartdrone::core::domain::RuntimeMode;
using LivePoseState = smartdrone::core::application::LivePoseState;
using LiveRuntimeTuning = smartdrone::core::application::LiveRuntimeTuning;
using MainRuntimeAliases = smartdrone::core::application::MainRuntimeAliases;
using Px4UdpHooks = smartdrone::core::application::Px4UdpHooks;
using UnifiedConfig = smartdrone::core::application::UnifiedConfig;
using UnifiedRuntimeController = smartdrone::core::application::UnifiedRuntimeController;

void InstallSignalHandlers()
{
    signal(SIGINT, SigIntHandler);
    signal(SIGTERM, SigIntHandler);
}

}  // namespace

int main(int argc, char** argv)
{
    InstallSignalHandlers();
    UnifiedConfig cfg{};
    cfg.app = ParseAppConfig(argc, argv);
    ArgReader args(argc, argv);
    cfg.calib.root = args.GetString("--calib-root", "./calib_runs");
    cfg.calib.maxFrames = args.GetInt("--calib-max-frames", 0);
    std::string autoModeText = args.GetString("--auto-mode", "idle");
    std::transform(autoModeText.begin(), autoModeText.end(), autoModeText.begin(), [](unsigned char c) {
        return (char)std::tolower(c);
    });
    ControllerMode autoMode = ControllerMode::Idle;
    if (autoModeText == "slam") autoMode = ControllerMode::Slam;
    else if (autoModeText == "calib") autoMode = ControllerMode::Calib;

    const MainRuntimeAliases aliases = smartdrone::core::application::BuildRuntimeAliases(cfg.app);
    smartdrone::core::application::PrintStartupConfig(cfg.app, aliases, ControllerMode::Idle);

    Px4MavlinkGateway mav("/dev/ttyAMA0", 921600);
    mav.StartRx();
    LivePoseState livePose;
    LiveRuntimeTuning tuning;
    Px4UdpHooks hooks(mav, livePose);
    UnifiedRuntimeController controller(
        cfg,
        tuning,
        mav,
        livePose,
        g_runningFlag,
        [&runningFlag = g_runningFlag](const UnifiedConfig& sessionCfg,
                                       LiveRuntimeTuning& sessionTuning,
                                       Px4MavlinkGateway& sessionMav,
                                       std::atomic<bool>& stop,
                                       LivePoseState& poseState) {
            return smartdrone::core::application::RunSlamSession(
                sessionCfg, sessionTuning, sessionMav, stop, poseState, runningFlag);
        },
        [&runningFlag = g_runningFlag](const UnifiedConfig& sessionCfg,
                                       std::atomic<bool>& stop,
                                       LivePoseState& poseState) {
            return smartdrone::core::application::RunCalibSession(
                sessionCfg, stop, poseState, runningFlag);
        },
        [](const std::string& root) {
            return smartdrone::core::application::CleanupCalibDataDirs(root);
        });
    controller.Start();
    if (autoMode != ControllerMode::Idle) controller.SetMode(autoMode, nullptr);
    std::thread udpCmdThread = smartdrone::core::application::StartUdpCommandThread(
        aliases.cmdPort,
        hooks,
        controller,
        livePose,
        g_runningFlag,
        []() { return smartdrone::core::application::BuildCapabilitiesPayload(); },
        [](const UnifiedConfig& currentConfig, ControllerMode currentMode) {
            return smartdrone::core::application::BuildConfigPayload(currentConfig, currentMode);
        },
        [](const UdpPeer& peer) { return UdpPeerToIpString(peer); });
    while (g_runningFlag.load()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    controller.Stop();
    mav.StopSetpointStream();
    mav.StopRx();
    if (udpCmdThread.joinable()) udpCmdThread.join();
    return 0;
}
