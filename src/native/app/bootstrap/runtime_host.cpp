#include "app/bootstrap/runtime_host.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "adapters/telemetry/mavlink_pose_publisher.h"
#include "adapters/telemetry/px4_vehicle_control_port.h"
#include "common/runtime_state.h"
#include "common/tlv/udp_server.h"
#include "core/application/runtime/payload_builders.h"
#include "core/application/runtime/px4_udp_hooks.h"
#include "core/application/runtime/runtime_aliases.h"
#include "core/application/runtime/runtime_controller.h"
#include "core/application/runtime/system_runtime_graph_service.h"
#include "core/application/session/calib/calib_storage_helpers.h"
#include "core/application/session/epg/session_graph_runtime_factory.h"
#include "core/application/state/live_pose_state.h"
#include "core/ports/vehicle_control_port.h"

namespace smartdrone::app::bootstrap {

namespace {

using ControllerMode = smartdrone::core::domain::RuntimeMode;
using EpgRedeployCoordinator =
    smartdrone::core::application::EpgRedeployCoordinator;
using EpgRedeployRequest = smartdrone::core::application::EpgRedeployRequest;
using IVehicleControlPort = smartdrone::core::ports::IVehicleControlPort;
using LivePoseState = smartdrone::core::application::LivePoseState;
using LiveRuntimeTuning = smartdrone::core::application::LiveRuntimeTuning;
using MainRuntimeAliases = smartdrone::core::application::MainRuntimeAliases;
using MavlinkPosePublisher = smartdrone::adapters::telemetry::MavlinkPosePublisher;
using Px4UdpHooks = smartdrone::core::application::Px4UdpHooks;
using Px4UdpHooksConfig = smartdrone::core::application::Px4UdpHooksConfig;
using Px4VehicleControlPort = smartdrone::adapters::telemetry::Px4VehicleControlPort;
using RuntimeSessionSupervisor = smartdrone::core::application::RuntimeSessionSupervisor;
using RuntimeGateSnapshot = smartdrone::core::application::RuntimeGateSnapshot;
using SessionGraphRuntimeFactoryConfig = smartdrone::core::application::SessionGraphRuntimeFactoryConfig;
using SystemRuntimeGraph = smartdrone::core::application::SystemRuntimeGraph;
using SystemRuntimeGraphConfig = smartdrone::core::application::SystemRuntimeGraphConfig;
using UdpCommandRuntimeConfig = smartdrone::core::application::UdpCommandRuntimeConfig;
using UdpRuntimeStateSnapshot = smartdrone::core::application::UdpRuntimeStateSnapshot;
using UnifiedConfig = smartdrone::core::application::UnifiedConfig;
using UnifiedRuntimeController = smartdrone::core::application::UnifiedRuntimeController;
using UnifiedRuntimeControllerConfig = smartdrone::core::application::UnifiedRuntimeControllerConfig;
using IPosePublisher = smartdrone::core::ports::IPosePublisher;
using ISlamSessionTelemetryPort = smartdrone::core::ports::ISlamSessionTelemetryPort;
constexpr int DISCOVERY_PORT = 15000;
constexpr auto SYSTEM_REDEPLOY_POLL_INTERVAL = std::chrono::milliseconds(100);

struct BuildSystemRuntimeGraphConfigInput {
    const MainRuntimeAliases &aliases;
    Px4MavlinkGateway &mav;
    Px4UdpHooks &hooks;
    UnifiedRuntimeController &controller;
    std::shared_ptr<EpgRedeployCoordinator> redeploy;
    UdpCommandRuntimeConfig commandRuntime;
};

std::string GetEnvOrDefault(const char *name, const char *fallback)
{
    const char *value = std::getenv(name);
    if (!value || value[0] == '\0') {
        return fallback;
    }
    return value;
}

int GetEnvIntOrDefault(const char *name, int fallback)
{
    const char *value = std::getenv(name);
    if (!value || value[0] == '\0') {
        return fallback;
    }
    try {
        return std::stoi(value);
    } catch (...) {
        return fallback;
    }
}

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

RuntimeSessionSupervisor::CreateSessionFn BuildSessionRuntimeFactory(LiveRuntimeTuning &tuning,
                                                                     ISlamSessionTelemetryPort &telemetry,
                                                                     IPosePublisher &posePublisher,
                                                                     LivePoseState &livePose)
{
    return [&tuning, &telemetry, &posePublisher, &livePose](
               const RuntimeSessionSupervisor::SessionStartRequest &request) {
        return smartdrone::core::application::CreateSessionGraphRuntime(SessionGraphRuntimeFactoryConfig{
            request.mode, request.cfg, tuning, telemetry, posePublisher, request.stop, livePose, request.runningFlag});
    };
}

UdpRuntimeStateSnapshot ToUdpRuntimeStateSnapshot(const LivePoseState::Snapshot &input)
{
    UdpRuntimeStateSnapshot output{};
    output.hasPeer = input.hasPeer;
    output.peer = input.peer;
    output.runtimeMode = input.runtimeMode;
    output.slamMode = input.slamMode;
    output.trackingState = input.trackingState;
    output.armed = input.armed;
    output.px4MainMode = input.px4MainMode;
    output.px4SubMode = input.px4SubMode;
    output.resetCounter = input.resetCounter;
    output.resetMapCount = input.resetMapCount;
    output.x = input.x;
    output.y = input.y;
    output.z = input.z;
    output.qw = input.qw;
    output.qx = input.qx;
    output.qy = input.qy;
    output.qz = input.qz;
    output.seq = input.seq;
    output.pointCloudXyz = input.pointCloudXyz;
    output.pointCloudSeq = input.pointCloudSeq;
    return output;
}

RuntimeGateSnapshot ToRuntimeGateSnapshot(const LivePoseState::Snapshot &input)
{
    RuntimeGateSnapshot output{};
    output.runtimeMode = input.runtimeMode;
    output.poseValid = input.poseValid;
    output.trackingState = input.trackingState;
    output.poseQuality = input.poseQuality;
    return output;
}

Px4UdpHooksConfig BuildPx4UdpHooksConfig(IVehicleControlPort &vehicleControl, LivePoseState &livePose)
{
    return Px4UdpHooksConfig{
        vehicleControl,
        [&livePose](RuntimeGateSnapshot &snapshot) {
            LivePoseState::Snapshot liveSnapshot{};
            if (!livePose.ReadSnapshot(liveSnapshot)) {
                return false;
            }
            snapshot = ToRuntimeGateSnapshot(liveSnapshot);
            return true;
        },
        [&livePose](bool armed, uint8_t mainMode, uint8_t subMode) {
            livePose.SetVehicleFlightState(armed, mainMode, subMode);
        }};
}

UnifiedRuntimeControllerConfig BuildRuntimeControllerConfig(const UnifiedConfig &cfg, LiveRuntimeTuning &tuning,
                                                           ISlamSessionTelemetryPort &telemetry,
                                                           IPosePublisher &posePublisher, LivePoseState &livePose)
{
    return UnifiedRuntimeControllerConfig{
        cfg, tuning, smartdrone::common::g_runningFlag,
        BuildSessionRuntimeFactory(tuning, telemetry, posePublisher, livePose),
        [&livePose](ControllerMode mode) { livePose.SetRuntimeMode(static_cast<uint8_t>(mode)); },
        [](const std::string &root) { return smartdrone::core::application::CleanupCalibDataDirs(root); }};
}

UdpCommandRuntimeConfig BuildUdpCommandRuntimeConfig(Px4UdpHooks &hooks, UnifiedRuntimeController &controller,
                                                     LivePoseState &livePose)
{
    UdpCommandRuntimeConfig config{};
    config.commandHook = &hooks;
    config.commandTarget = &controller;
    config.currentConfig = [&controller]() { return controller.CurrentConfig(); };
    config.currentRuntimeMode = [&controller]() { return controller.CurrentDesiredMode(); };
    config.updateCommandPeer = [&livePose](const UdpPeer &peer) { livePose.UpdatePeer(peer); };
    config.readRuntimeState = [&livePose](UdpRuntimeStateSnapshot &snapshot) {
        LivePoseState::Snapshot liveSnapshot{};
        if (!livePose.ReadSnapshot(liveSnapshot)) {
            return false;
        }
        snapshot = ToUdpRuntimeStateSnapshot(liveSnapshot);
        return true;
    };
    return config;
}

SystemRuntimeGraphConfig BuildSystemRuntimeGraphConfig(BuildSystemRuntimeGraphConfigInput input)
{
    return SystemRuntimeGraphConfig{
        input.aliases,
        DISCOVERY_PORT,
        [&mav = input.mav]() { (void)mav.PollRxOnce(0); },
        [&mav = input.mav]() { mav.StepSetpointStream(); },
        [&hooks = input.hooks]() { hooks.StepManualControl(); },
        [&controller = input.controller]() { controller.StepForceRestart(); },
        [&controller = input.controller]() { controller.OnSessionSupervisorGraphTick(); },
        [&controller = input.controller](EpgRedeployCoordinator &coordinator) {
            controller.StepEpgRedeploy(coordinator);
        },
        std::move(input.redeploy),
        std::move(input.commandRuntime),
        []() { return smartdrone::core::application::BuildCapabilitiesPayload(); },
        [](const UnifiedConfig &currentConfig, ControllerMode currentMode) {
            return smartdrone::core::application::BuildConfigPayload(currentConfig, currentMode);
        },
        [](const UdpPeer &peer) { return UdpPeerToIpString(peer); }};
}

void LogSystemGraphRedeployRequest(const EpgRedeployRequest &request)
{
    std::cerr << "[epg] system graph redeploy requested: "
              << smartdrone::core::application::DescribeEpgRedeployRequest(
                     request)
              << "\n";
}

bool RestartSystemGraph(SystemRuntimeGraph &systemGraph,
                        const EpgRedeployRequest &request)
{
    LogSystemGraphRedeployRequest(request);
    systemGraph.Stop();
    if (systemGraph.Start()) {
        std::cerr << "[epg] system graph redeploy applied\n";
        return true;
    }
    std::cerr << "[runtime] system EPG restart failed\n";
    smartdrone::common::RequestRuntimeStop();
    return false;
}

bool RunSystemGraphUntilStopped(SystemRuntimeGraph &systemGraph,
                                EpgRedeployCoordinator &redeploy)
{
    while (!smartdrone::common::RuntimeStopRequested()) {
        EpgRedeployRequest request;
        if (redeploy.TakeSystemRedeployRequest(request) &&
            !RestartSystemGraph(systemGraph, request)) {
            return false;
        }
        (void)redeploy.WaitForSystemRedeploy(SYSTEM_REDEPLOY_POLL_INTERVAL);
    }
    return true;
}

} // namespace

int RuntimeHost::Run(const UnifiedConfig &cfg, const std::string &autoModeText)
{
    const ControllerMode autoMode = ParseAutoMode(autoModeText);
    const MainRuntimeAliases aliases = smartdrone::core::application::BuildRuntimeAliases(cfg.app);
    smartdrone::core::application::PrintStartupConfig(cfg.app, aliases, ControllerMode::Idle);

    const std::string mavDev = GetEnvOrDefault("SMART_DRONE_MAVLINK_DEV", "/dev/ttyAMA0");
    const int mavBaud = GetEnvIntOrDefault("SMART_DRONE_MAVLINK_BAUD", 921600);
    std::cerr << "[runtime] epg=on\n";
    Px4MavlinkGateway mav(mavDev, mavBaud);
    mav.SetJsonDiagnostics(cfg.app.runtime.jsonDiagnostics);
    Px4VehicleControlPort vehicleControl(mav);
    MavlinkPosePublisher posePublisher(mav);
    LivePoseState livePose;
    LiveRuntimeTuning tuning;
    Px4UdpHooks hooks(BuildPx4UdpHooksConfig(vehicleControl, livePose));
    UnifiedRuntimeController controller(BuildRuntimeControllerConfig(cfg, tuning, vehicleControl, posePublisher, livePose));
    UdpCommandRuntimeConfig commandRuntime = BuildUdpCommandRuntimeConfig(hooks, controller, livePose);
    auto redeploy = std::make_shared<EpgRedeployCoordinator>();
    SystemRuntimeGraph systemGraph(
        BuildSystemRuntimeGraphConfig({
            aliases,
            mav,
            hooks,
            controller,
            redeploy,
            std::move(commandRuntime),
        }));
    if (!systemGraph.Start()) {
        smartdrone::common::RequestRuntimeStop();
        controller.Stop();
        mav.StopSetpointStream();
        return 1;
    }
    if (autoMode != ControllerMode::Idle) {
        controller.SetMode(autoMode, nullptr);
    }

    const bool runtimeOk = RunSystemGraphUntilStopped(systemGraph, *redeploy);

    systemGraph.Stop();
    controller.Stop();
    mav.StopSetpointStream();
    return runtimeOk ? 0 : 1;
}

} // namespace smartdrone::app::bootstrap
