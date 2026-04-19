#include "app/bootstrap/runtime_host.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "common/discovery/udp_discovery_beacon.h"
#include "common/runtime_state.h"
#include "common/tlv/udp_server.h"
#include "core/application/runtime/payload_builders.h"
#include "core/application/runtime/px4_udp_hooks.h"
#include "core/application/runtime/runtime_controller.h"
#include "core/application/runtime/udp_command_thread.h"
#include "core/application/session/calib_session_service.h"
#include "core/application/session/calib_storage_helpers.h"
#include "core/application/session/runtime_session_common.h"
#include "core/application/session/slam_session_service.h"
#include "core/application/state/live_pose_state.h"

namespace smartdrone::app::bootstrap {

namespace {

using ControllerMode = smartdrone::core::domain::RuntimeMode;
using LivePoseState = smartdrone::core::application::LivePoseState;
using LiveRuntimeTuning = smartdrone::core::application::LiveRuntimeTuning;
using MainRuntimeAliases = smartdrone::core::application::MainRuntimeAliases;
using Px4UdpHooks = smartdrone::core::application::Px4UdpHooks;
using UnifiedConfig = smartdrone::core::application::UnifiedConfig;
using UnifiedRuntimeController = smartdrone::core::application::UnifiedRuntimeController;
constexpr int kDiscoveryPort = 15000;

ControllerMode ParseAutoMode(std::string autoModeText)
{
    std::transform(autoModeText.begin(), autoModeText.end(), autoModeText.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    ControllerMode autoMode = ControllerMode::Idle;
    if (autoModeText == "slam") {
        autoMode = ControllerMode::Slam;
    } else if (autoModeText == "calib") {
        autoMode = ControllerMode::Calib;
    }
    return autoMode;
}

} // namespace

int RuntimeHost::Run(const UnifiedConfig &cfg, const std::string &autoModeText)
{
    const ControllerMode autoMode = ParseAutoMode(autoModeText);
    const MainRuntimeAliases aliases = smartdrone::core::application::BuildRuntimeAliases(cfg.app);
    smartdrone::core::application::PrintStartupConfig(cfg.app, aliases, ControllerMode::Idle);

    Px4MavlinkGateway mav("/dev/ttyAMA0", 921600);
    mav.SetJsonDiagnostics(cfg.app.runtime.jsonDiagnostics);
    mav.StartRx();
    LivePoseState livePose;
    LiveRuntimeTuning tuning;
    Px4UdpHooks hooks(mav, livePose);
    UnifiedRuntimeController controller(
        cfg, tuning, mav, livePose, smartdrone::common::g_runningFlag,
        [&runningFlag = smartdrone::common::g_runningFlag](const UnifiedConfig &sessionCfg, LiveRuntimeTuning &sessionTuning,
                                       Px4MavlinkGateway &sessionMav, std::atomic<bool> &stop,
                                       LivePoseState &poseState) {
            return smartdrone::core::application::RunSlamSession(sessionCfg, sessionTuning, sessionMav, stop, poseState,
                                                                 runningFlag);
        },
        [&runningFlag = smartdrone::common::g_runningFlag](const UnifiedConfig &sessionCfg, std::atomic<bool> &stop,
                                       LivePoseState &poseState) {
            return smartdrone::core::application::RunCalibSession(sessionCfg, stop, poseState, runningFlag);
        },
        [](const std::string &root) { return smartdrone::core::application::CleanupCalibDataDirs(root); });

    controller.Start();
    if (autoMode != ControllerMode::Idle) {
        controller.SetMode(autoMode, nullptr);
    }

    std::thread udpCmdThread = smartdrone::core::application::StartUdpCommandThread(
        aliases.cmdPort, hooks, controller, livePose, smartdrone::common::g_runningFlag,
        []() { return smartdrone::core::application::BuildCapabilitiesPayload(); },
        [](const UnifiedConfig &currentConfig, ControllerMode currentMode) {
            return smartdrone::core::application::BuildConfigPayload(currentConfig, currentMode);
        },
        [](const UdpPeer &peer) { return UdpPeerToIpString(peer); });
    std::thread discoveryThread = smartdrone::common::discovery::StartUdpDiscoveryBeaconThread(
        kDiscoveryPort, aliases.cmdPort, aliases.udpPort, smartdrone::common::g_runningFlag);

    while (smartdrone::common::g_runningFlag.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    controller.Stop();
    mav.StopSetpointStream();
    mav.StopRx();
    if (discoveryThread.joinable()) {
        discoveryThread.join();
    }
    if (udpCmdThread.joinable()) {
        udpCmdThread.join();
    }
    return 0;
}

} // namespace smartdrone::app::bootstrap
