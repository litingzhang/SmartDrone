#include "app/bootstrap/runtime_host.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "app/composition/default_application_runtime_factories.h"
#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "adapters/telemetry/mavlink_pose_publisher.h"
#include "adapters/telemetry/px4_vehicle_control_port.h"
#include "common/environment.h"
#include "common/runtime_state.h"
#include "core/application/runtime/application_runtime_factories.h"
#include "core/application/runtime/live_pose_runtime_snapshots.h"
#include "core/application/runtime/obstacle_avoidance_config.h"
#include "core/application/runtime/px4_udp_hooks.h"
#include "core/application/runtime/runtime_aliases.h"
#include "core/application/runtime/runtime_controller.h"
#include "core/application/runtime/system_runtime_graph_service.h"
#include "core/application/session/calib/calib_storage_helpers.h"
#include "core/application/session/epg/session_graph_runtime_factory.h"
#include "core/application/state/live_pose_state.h"
#include "core/ports/vehicle_control_port.h"
#include "core/ports/external_pose_source.h"

namespace SmartDrone::App::Bootstrap {

namespace {

using ControllerMode = SmartDrone::Core::Domain::RuntimeMode;
using ApplicationRuntimeFactories =
    SmartDrone::Core::Application::ApplicationRuntimeFactories;
using EpgRedeployCoordinator =
    SmartDrone::Core::Application::EpgRedeployCoordinator;
using EpgRedeployRequest = SmartDrone::Core::Application::EpgRedeployRequest;
using IVehicleControlPort = SmartDrone::Core::Ports::IVehicleControlPort;
using LivePoseState = SmartDrone::Core::Application::LivePoseState;
using LiveRuntimeTuning = SmartDrone::Core::Application::LiveRuntimeTuning;
using MainRuntimeAliases = SmartDrone::Core::Application::MainRuntimeAliases;
using MavlinkPosePublisher = SmartDrone::Adapters::Telemetry::MavlinkPosePublisher;
using Px4UdpHooks = SmartDrone::Core::Application::Px4UdpHooks;
using Px4UdpHooksConfig = SmartDrone::Core::Application::Px4UdpHooksConfig;
using Px4VehicleControlPort = SmartDrone::Adapters::Telemetry::Px4VehicleControlPort;
using Px4VehicleControlPortConfig =
    SmartDrone::Adapters::Telemetry::Px4VehicleControlPortConfig;
using RuntimeSessionSupervisor = SmartDrone::Core::Application::RuntimeSessionSupervisor;
using SessionGraphRuntimeFactoryConfig = SmartDrone::Core::Application::SessionGraphRuntimeFactoryConfig;
using SystemRuntimeGraph = SmartDrone::Core::Application::SystemRuntimeGraph;
using SystemRuntimeGraphConfig = SmartDrone::Core::Application::SystemRuntimeGraphConfig;
using UdpCommandRuntimeConfig = SmartDrone::Core::Application::UdpCommandRuntimeConfig;
using UnifiedConfig = SmartDrone::Core::Application::UnifiedConfig;
using UnifiedRuntimeController = SmartDrone::Core::Application::UnifiedRuntimeController;
using UnifiedRuntimeControllerConfig = SmartDrone::Core::Application::UnifiedRuntimeControllerConfig;
using IPosePublisher = SmartDrone::Core::Ports::IPosePublisher;
using ISlamSessionTelemetryPort = SmartDrone::Core::Ports::ISlamSessionTelemetryPort;
using CameraRuntimeOverrides = SmartDrone::Core::Ports::CameraRuntimeOverrides;
constexpr int DISCOVERY_PORT = 15000;
constexpr auto SYSTEM_REDEPLOY_POLL_INTERVAL = std::chrono::milliseconds(100);
constexpr double MIN_SIM_WATCHDOG_SCALE = 1.0;
constexpr double MAX_SIM_WATCHDOG_SCALE = 20.0;
constexpr uint32_t MAX_SCALED_WATCHDOG_TIMEOUT_MS = 60000;
constexpr uint32_t PX4_FLIGHT_MODE_MAX_AGE_MS = 1500;

struct BuildSystemRuntimeGraphConfigInput {
    const MainRuntimeAliases &aliases;
    Px4MavlinkGateway &mav;
    Px4UdpHooks &hooks;
    UnifiedRuntimeController &controller;
    const ApplicationRuntimeFactories &factories;
    std::shared_ptr<EpgRedeployCoordinator> redeploy;
    UdpCommandRuntimeConfig commandRuntime;
    IPosePublisher &posePublisher;
    LivePoseState &livePose;
    bool externalPoseEnabled{false};
};

struct BuildRuntimeControllerConfigInput {
    const UnifiedConfig &cfg;
    LiveRuntimeTuning &tuning;
    ISlamSessionTelemetryPort &telemetry;
    IPosePublisher &posePublisher;
    LivePoseState &livePose;
    const ApplicationRuntimeFactories &factories;
};

struct RuntimeHostServices {
    Px4MavlinkGateway mav;
    Px4VehicleControlPort vehicleControl;
    MavlinkPosePublisher posePublisher;
    LivePoseState livePose;
    LiveRuntimeTuning tuning;

    RuntimeHostServices(const std::string &mavDev, int mavBaud,
                        bool jsonDiagnostics,
                        Px4VehicleControlPortConfig vehicleConfig)
        : mav(mavDev, mavBaud), vehicleControl(mav, vehicleConfig),
          posePublisher(mav)
    {
        mav.SetJsonDiagnostics(jsonDiagnostics);
    }
};

std::string GetEnvOrDefault(const char *name, const char *fallback)
{
    return SmartDrone::Common::EnvStringValue(name, fallback);
}

int GetEnvIntOrDefault(const char *name, int fallback)
{
    return SmartDrone::Common::EnvIntValue(name, fallback);
}

uint32_t ScaleSimulationWatchdogTimeout(uint32_t timeoutMs)
{
    const double scale = std::clamp(
        SmartDrone::Common::EnvDoubleValue(
            "SMART_DRONE_SIM_WATCHDOG_SCALE", 1.0),
        MIN_SIM_WATCHDOG_SCALE, MAX_SIM_WATCHDOG_SCALE);
    return static_cast<uint32_t>(std::min<long long>(
        MAX_SCALED_WATCHDOG_TIMEOUT_MS,
        std::llround(static_cast<double>(timeoutMs) * scale)));
}

Px4VehicleControlPortConfig BuildPx4VehicleControlPortConfig()
{
    Px4VehicleControlPortConfig config{};
    config.flightModeMaxAgeUs =
        static_cast<uint64_t>(ScaleSimulationWatchdogTimeout(
            PX4_FLIGHT_MODE_MAX_AGE_MS)) *
        1000ULL;
    return config;
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
        return SmartDrone::Core::Application::CreateSessionGraphRuntime(SessionGraphRuntimeFactoryConfig{
            request.mode, request.cfg, tuning, telemetry, posePublisher,
            request.stop, livePose, request.runningFlag, factories});
    };
}

Px4UdpHooksConfig BuildPx4UdpHooksConfig(IVehicleControlPort &vehicleControl,
                                         LiveRuntimeTuning &tuning,
                                         LivePoseState &livePose,
                                         IPosePublisher &posePublisher)
{
    Px4UdpHooksConfig config{
        vehicleControl,
        &tuning,
        SmartDrone::Core::Application::MakeRuntimeGateReader(livePose),
        SmartDrone::Core::Application::MakeAvoidanceSnapshotReader(livePose),
        SmartDrone::Core::Application::MakeVehicleFlightStatePublisher(
            livePose),
        SmartDrone::Core::Application::MakeAvoidanceTelemetryPublisher(
            livePose)};
    config.requireVisualLocalization = SmartDrone::Common::EnvFlagEnabled(
        "SMART_DRONE_OFFBOARD_REQUIRES_VISION", true);
    const uint32_t visualPoseMaxAgeMs = static_cast<uint32_t>(
        SmartDrone::Common::EnvIntValueClamped(
            "SMART_DRONE_VISUAL_POSE_MAX_AGE_MS", 100, 50, 10000));
    const uint32_t visualLossLandTimeoutMs = static_cast<uint32_t>(
        SmartDrone::Common::EnvIntValueClamped(
            "SMART_DRONE_VISUAL_LOSS_LAND_MS", 500, 100, 10000));
    config.visualPoseMaxAgeMs =
        ScaleSimulationWatchdogTimeout(visualPoseMaxAgeMs);
    config.visualLossLandTimeoutMs =
        ScaleSimulationWatchdogTimeout(visualLossLandTimeoutMs);
    config.publishVisualLoss =
        SmartDrone::Core::Application::MakeVisualLossPublisher(
            livePose, posePublisher);
    return config;
}

UnifiedRuntimeControllerConfig BuildRuntimeControllerConfig(
    BuildRuntimeControllerConfigInput input)
{
    return UnifiedRuntimeControllerConfig{
        input.cfg, input.tuning, SmartDrone::Common::g_runningFlag,
        BuildSessionRuntimeFactory(input.tuning, input.telemetry,
                                   input.posePublisher, input.livePose,
                                   input.factories),
        SmartDrone::Core::Application::MakeRuntimeModePublisher(
            input.livePose),
        [](const std::string &root) { return SmartDrone::Core::Application::CleanupCalibDataDirs(root); }};
}

UdpCommandRuntimeConfig BuildUdpCommandRuntimeConfig(Px4UdpHooks &hooks, UnifiedRuntimeController &controller,
                                                     LivePoseState &livePose)
{
    UdpCommandRuntimeConfig config{};
    config.commandHook = &hooks;
    config.commandTarget = &controller;
    config.currentConfig = [&controller]() { return controller.CurrentConfig(); };
    config.currentRuntimeMode = [&controller]() { return controller.CurrentDesiredMode(); };
    config.updateCommandPeer =
        SmartDrone::Core::Application::MakeCommandPeerUpdater(livePose);
    config.readRuntimeState =
        SmartDrone::Core::Application::MakeUdpRuntimeStateReader(livePose);
    return config;
}

SystemRuntimeGraphConfig BuildSystemRuntimeGraphConfig(BuildSystemRuntimeGraphConfigInput input)
{
    return SystemRuntimeGraphConfig{
        input.aliases,
        DISCOVERY_PORT,
        [&mav = input.mav,
         source = input.factories.externalPoseSource,
         &publisher = input.posePublisher,
         &livePose = input.livePose,
         enabled = input.externalPoseEnabled]() {
            (void)mav.PollRxOnce();
            if (!enabled || !source) {
                return;
            }
            SmartDrone::Core::Ports::PosePublishRequest request{};
            if (source->TryRead(request)) {
                SmartDrone::Core::Application::PublishExternalPose(
                    request, publisher, livePose);
            }
        },
        [&mav = input.mav]() { mav.StepSetpointStream(); },
        [&hooks = input.hooks]() { hooks.StepManualControl(); },
        [&controller = input.controller]() { controller.StepForceRestart(); },
        [&controller = input.controller]() { controller.OnSessionSupervisorGraphTick(); },
        [&controller = input.controller](EpgRedeployCoordinator &coordinator) {
            controller.StepEpgRedeploy(coordinator);
        },
        std::move(input.redeploy),
        std::move(input.commandRuntime),
        input.factories.cameraProvider};
}

void LogSystemGraphRedeployRequest(const EpgRedeployRequest &request)
{
    std::cerr << "[epg] system graph redeploy requested: "
              << SmartDrone::Core::Application::DescribeEpgRedeployRequest(
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
    SmartDrone::Common::RequestRuntimeStop();
    return false;
}

bool RunSystemGraphUntilStopped(SystemRuntimeGraph &systemGraph,
                                EpgRedeployCoordinator &redeploy)
{
    while (!SmartDrone::Common::RuntimeStopRequested()) {
        EpgRedeployRequest request;
        if (redeploy.TakeSystemRedeployRequest(request) &&
            !RestartSystemGraph(systemGraph, request)) {
            return false;
        }
        (void)SmartDrone::Common::WaitForRuntimeStop(
            SYSTEM_REDEPLOY_POLL_INTERVAL);
    }
    return true;
}

void StopRuntimeHostServices(SystemRuntimeGraph &systemGraph,
                             UnifiedRuntimeController &controller,
                             Px4MavlinkGateway &mav)
{
    systemGraph.Stop();
    controller.Stop();
    mav.StopSetpointStream();
}

int RunSystemGraphOrFail(SystemRuntimeGraph &systemGraph,
                         UnifiedRuntimeController &controller,
                         Px4MavlinkGateway &mav,
                         EpgRedeployCoordinator &redeploy)
{
    if (!systemGraph.Start()) {
        SmartDrone::Common::RequestRuntimeStop();
        controller.Stop();
        mav.StopSetpointStream();
        return 1;
    }
    const bool runtimeOk = RunSystemGraphUntilStopped(systemGraph, redeploy);
    StopRuntimeHostServices(systemGraph, controller, mav);
    return runtimeOk ? 0 : 1;
}

void PrintAvoidanceStartupConfig(const UnifiedConfig &cfg)
{
    const auto config = SmartDrone::Core::Application::
        ObstacleAvoidanceConfigFromRuntime(cfg.app.runtime);
    std::cerr << "[avoid] enabled=" << (config.enabled ? "Y" : "N")
              << " radius_m=" << config.radiusM
              << " lookahead_m=" << config.lookaheadM
              << " speed_lookahead_s=" << config.speedLookaheadS
              << " near_field_radius_m=" << config.nearFieldRadiusM
              << " max_age_ms=" << config.maxPointCloudAgeMs
              << " min_cloud_points=" << config.minCloudPoints
              << " min_points=" << config.minBlockingPoints
              << " hold_on_stale_cloud="
              << (config.holdOnStaleCloud ? "Y" : "N") << "\n";
}

bool ValidateRuntimeFactories(const ApplicationRuntimeFactories &factories,
                              bool externalPoseEnabled)
{
    if (!factories.Valid()) {
        std::cerr << "[runtime] application runtime factories invalid\n";
        return false;
    }
    if (externalPoseEnabled &&
        (!factories.externalPoseSource ||
         !factories.externalPoseSource->Healthy())) {
        std::cerr << "[runtime] truth profile requires Gazebo external pose source\n";
        return false;
    }
    return true;
}

void PrintRuntimeStartup(const UnifiedConfig &cfg,
                         const MainRuntimeAliases &aliases,
                         const ApplicationRuntimeFactories &factories)
{
    SmartDrone::Core::Application::PrintStartupConfig(
        cfg.app, aliases, factories.cameraProvider, ControllerMode::Idle);
    PrintAvoidanceStartupConfig(cfg);
    std::cerr << "[runtime] epg=on\n";
}

void ApplyCameraRuntimeOverrides(const CameraRuntimeOverrides &overrides,
                                 UnifiedConfig &config)
{
    if (!overrides.Valid()) {
        return;
    }
    config.app.camera.width = overrides.width;
    config.app.camera.height = overrides.height;
    config.app.camera.fps = overrides.fps;
    config.app.settings = overrides.settingsPath;
    std::cerr << "[runtime] camera config override=" << overrides.width << "x"
              << overrides.height << "@" << overrides.fps
              << " settings=" << overrides.settingsPath << "\n";
}

} // namespace

int RuntimeHost::Run(const UnifiedConfig &cfg, const std::string &autoModeText)
{
    UnifiedConfig effectiveConfig = cfg;
    const ControllerMode autoMode = ParseAutoMode(autoModeText);
    const ApplicationRuntimeFactories factories =
        SmartDrone::App::Composition::CreateDefaultApplicationRuntimeFactories();
    const bool externalPoseEnabled =
        GetEnvOrDefault("SMART_DRONE_SIM_PROFILE", "") == "truth";
    if (!ValidateRuntimeFactories(factories, externalPoseEnabled)) {
        return 1;
    }
    ApplyCameraRuntimeOverrides(factories.cameraOverrides, effectiveConfig);
    const MainRuntimeAliases aliases =
        SmartDrone::Core::Application::BuildRuntimeAliases(effectiveConfig.app);
    PrintRuntimeStartup(effectiveConfig, aliases, factories);
    RuntimeHostServices services(
        GetEnvOrDefault("SMART_DRONE_MAVLINK_DEV", "/dev/ttyAMA0"),
        GetEnvIntOrDefault("SMART_DRONE_MAVLINK_BAUD", 921600),
        effectiveConfig.app.runtime.jsonDiagnostics,
        BuildPx4VehicleControlPortConfig());
    services.mav.SetMeasurementClock(factories.measurementClock);
    Px4UdpHooks hooks(
        BuildPx4UdpHooksConfig(services.vehicleControl, services.tuning,
                               services.livePose, services.posePublisher));
    UnifiedRuntimeController controller(BuildRuntimeControllerConfig({
        effectiveConfig, services.tuning, services.vehicleControl,
        services.posePublisher, services.livePose, factories,
    }));
    UdpCommandRuntimeConfig commandRuntime =
        BuildUdpCommandRuntimeConfig(hooks, controller, services.livePose);
    auto redeploy = std::make_shared<EpgRedeployCoordinator>();
    SystemRuntimeGraph systemGraph(
        BuildSystemRuntimeGraphConfig({
            aliases,
            services.mav,
            hooks,
            controller,
            factories,
            redeploy,
            std::move(commandRuntime),
            services.posePublisher,
            services.livePose,
            externalPoseEnabled,
        }));
    if (autoMode != ControllerMode::Idle) {
        controller.SetMode(autoMode, nullptr);
    }
    return RunSystemGraphOrFail(systemGraph, controller, services.mav, *redeploy);
}

} // namespace SmartDrone::App::Bootstrap
