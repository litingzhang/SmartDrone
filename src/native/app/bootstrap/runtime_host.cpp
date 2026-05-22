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

#include "adapters/runtime/default_application_runtime_factories.h"
#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "adapters/telemetry/mavlink_pose_publisher.h"
#include "adapters/telemetry/px4_vehicle_control_port.h"
#include "common/runtime_state.h"
#include "core/application/runtime/application_runtime_factories.h"
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

namespace SmartDrone::app::bootstrap {

namespace {

using ControllerMode = SmartDrone::core::domain::RuntimeMode;
using ApplicationRuntimeFactories =
    SmartDrone::core::application::ApplicationRuntimeFactories;
using EpgRedeployCoordinator =
    SmartDrone::core::application::EpgRedeployCoordinator;
using EpgRedeployRequest = SmartDrone::core::application::EpgRedeployRequest;
using IVehicleControlPort = SmartDrone::core::ports::IVehicleControlPort;
using LivePoseState = SmartDrone::core::application::LivePoseState;
using LiveRuntimeTuning = SmartDrone::core::application::LiveRuntimeTuning;
using MainRuntimeAliases = SmartDrone::core::application::MainRuntimeAliases;
using MavlinkPosePublisher = SmartDrone::adapters::telemetry::MavlinkPosePublisher;
using Px4UdpHooks = SmartDrone::core::application::Px4UdpHooks;
using Px4UdpHooksConfig = SmartDrone::core::application::Px4UdpHooksConfig;
using Px4VehicleControlPort = SmartDrone::adapters::telemetry::Px4VehicleControlPort;
using RuntimeSessionSupervisor = SmartDrone::core::application::RuntimeSessionSupervisor;
using RuntimeGateSnapshot = SmartDrone::core::application::RuntimeGateSnapshot;
using SessionGraphRuntimeFactoryConfig = SmartDrone::core::application::SessionGraphRuntimeFactoryConfig;
using SystemRuntimeGraph = SmartDrone::core::application::SystemRuntimeGraph;
using SystemRuntimeGraphConfig = SmartDrone::core::application::SystemRuntimeGraphConfig;
using UdpCommandRuntimeConfig = SmartDrone::core::application::UdpCommandRuntimeConfig;
using UdpRuntimeStateSnapshot = SmartDrone::core::application::UdpRuntimeStateSnapshot;
using UnifiedConfig = SmartDrone::core::application::UnifiedConfig;
using UnifiedRuntimeController = SmartDrone::core::application::UnifiedRuntimeController;
using UnifiedRuntimeControllerConfig = SmartDrone::core::application::UnifiedRuntimeControllerConfig;
using IPosePublisher = SmartDrone::core::ports::IPosePublisher;
using ISlamSessionTelemetryPort = SmartDrone::core::ports::ISlamSessionTelemetryPort;
constexpr int DISCOVERY_PORT = 15000;
constexpr auto SYSTEM_REDEPLOY_POLL_INTERVAL = std::chrono::milliseconds(100);

struct BuildSystemRuntimeGraphConfigInput {
    const MainRuntimeAliases &aliases;
    Px4MavlinkGateway &mav;
    Px4UdpHooks &hooks;
    UnifiedRuntimeController &controller;
    const ApplicationRuntimeFactories &factories;
    std::shared_ptr<EpgRedeployCoordinator> redeploy;
    UdpCommandRuntimeConfig commandRuntime;
};

struct BuildRuntimeControllerConfigInput {
    const UnifiedConfig &cfg;
    LiveRuntimeTuning &tuning;
    ISlamSessionTelemetryPort &telemetry;
    IPosePublisher &posePublisher;
    LivePoseState &livePose;
    const ApplicationRuntimeFactories &factories;
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
                                                                     LivePoseState &livePose,
                                                                     const ApplicationRuntimeFactories &factories)
{
    return [&tuning, &telemetry, &posePublisher, &livePose, &factories](
               const RuntimeSessionSupervisor::SessionStartRequest &request) {
        return SmartDrone::core::application::CreateSessionGraphRuntime(SessionGraphRuntimeFactoryConfig{
            request.mode, request.cfg, tuning, telemetry, posePublisher,
            request.stop, livePose, request.runningFlag, factories});
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

UnifiedRuntimeControllerConfig BuildRuntimeControllerConfig(
    BuildRuntimeControllerConfigInput input)
{
    return UnifiedRuntimeControllerConfig{
        input.cfg, input.tuning, SmartDrone::common::g_runningFlag,
        BuildSessionRuntimeFactory(input.tuning, input.telemetry,
                                   input.posePublisher, input.livePose,
                                   input.factories),
        [&livePose = input.livePose](ControllerMode mode) {
            livePose.SetRuntimeMode(static_cast<uint8_t>(mode));
        },
        [](const std::string &root) { return SmartDrone::core::application::CleanupCalibDataDirs(root); }};
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
        [cameraProvider = input.factories.cameraProvider]() {
            return SmartDrone::core::application::BuildCapabilitiesPayload(
                cameraProvider);
        },
        [cameraProvider = input.factories.cameraProvider](
            const UnifiedConfig &currentConfig, ControllerMode currentMode) {
            return SmartDrone::core::application::BuildConfigPayload(
                currentConfig, currentMode, cameraProvider);
        },
        [](const UdpPeer &peer) { return UdpPeerToIpString(peer); }};
}

void LogSystemGraphRedeployRequest(const EpgRedeployRequest &request)
{
    std::cerr << "[epg] system graph redeploy requested: "
              << SmartDrone::core::application::DescribeEpgRedeployRequest(
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
    SmartDrone::common::RequestRuntimeStop();
    return false;
}

bool RunSystemGraphUntilStopped(SystemRuntimeGraph &systemGraph,
                                EpgRedeployCoordinator &redeploy)
{
    while (!SmartDrone::common::RuntimeStopRequested()) {
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
    const std::string mavDev = GetEnvOrDefault("SMART_DRONE_MAVLINK_DEV", "/dev/ttyAMA0");
    const int mavBaud = GetEnvIntOrDefault("SMART_DRONE_MAVLINK_BAUD", 921600);
    const ApplicationRuntimeFactories factories =
        SmartDrone::adapters::runtime::CreateDefaultApplicationRuntimeFactories();
    if (!factories.Valid()) {
        std::cerr << "[runtime] application runtime factories invalid\n";
        return 1;
    }
    const MainRuntimeAliases aliases =
        SmartDrone::core::application::BuildRuntimeAliases(cfg.app);
    SmartDrone::core::application::PrintStartupConfig(
        cfg.app, aliases, factories.cameraProvider, ControllerMode::Idle);

    std::cerr << "[runtime] epg=on\n";
    Px4MavlinkGateway mav(mavDev, mavBaud);
    mav.SetJsonDiagnostics(cfg.app.runtime.jsonDiagnostics);
    Px4VehicleControlPort vehicleControl(mav);
    MavlinkPosePublisher posePublisher(mav);
    LivePoseState livePose;
    LiveRuntimeTuning tuning;
    Px4UdpHooks hooks(BuildPx4UdpHooksConfig(vehicleControl, livePose));
    UnifiedRuntimeController controller(BuildRuntimeControllerConfig({
        cfg,
        tuning,
        vehicleControl,
        posePublisher,
        livePose,
        factories,
    }));
    UdpCommandRuntimeConfig commandRuntime = BuildUdpCommandRuntimeConfig(hooks, controller, livePose);
    auto redeploy = std::make_shared<EpgRedeployCoordinator>();
    SystemRuntimeGraph systemGraph(
        BuildSystemRuntimeGraphConfig({
            aliases,
            mav,
            hooks,
            controller,
            factories,
            redeploy,
            std::move(commandRuntime),
        }));
    if (!systemGraph.Start()) {
        SmartDrone::common::RequestRuntimeStop();
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

} // namespace SmartDrone::app::bootstrap
