#pragma once

#include <atomic>
#include <memory>

#include "core/application/session/epg/session_graph_runtime.h"
#include "core/domain/runtime_mode.h"

namespace SmartDrone::Core::Ports {
class IPosePublisher;
class ISlamSessionTelemetryPort;
} // namespace SmartDrone::Core::Ports

namespace SmartDrone::Core::Application {

struct ApplicationRuntimeFactories;
struct LiveRuntimeTuning;
class LivePoseState;
struct UnifiedConfig;

struct SessionGraphRuntimeFactoryConfig {
    Domain::RuntimeMode mode{Domain::RuntimeMode::Idle};
    const UnifiedConfig &cfg;
    LiveRuntimeTuning &tuning;
    SmartDrone::Core::Ports::ISlamSessionTelemetryPort &telemetry;
    SmartDrone::Core::Ports::IPosePublisher &posePublisher;
    std::atomic<bool> &stop;
    LivePoseState &livePose;
    std::atomic<bool> &runningFlag;
    const ApplicationRuntimeFactories &factories;
};

std::unique_ptr<ISessionGraphRuntime> CreateSessionGraphRuntime(const SessionGraphRuntimeFactoryConfig &config);

} // namespace SmartDrone::Core::Application
