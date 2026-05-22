#pragma once

#include <atomic>
#include <memory>

#include "core/application/session/epg/session_graph_runtime.h"
#include "core/domain/runtime_mode.h"

namespace SmartDrone::core::ports {
class IPosePublisher;
class ISlamSessionTelemetryPort;
} // namespace SmartDrone::core::ports

namespace SmartDrone::core::application {

struct ApplicationRuntimeFactories;
struct LiveRuntimeTuning;
struct LivePoseState;
struct UnifiedConfig;

struct SessionGraphRuntimeFactoryConfig {
    domain::RuntimeMode mode{domain::RuntimeMode::Idle};
    const UnifiedConfig &cfg;
    LiveRuntimeTuning &tuning;
    SmartDrone::core::ports::ISlamSessionTelemetryPort &telemetry;
    SmartDrone::core::ports::IPosePublisher &posePublisher;
    std::atomic<bool> &stop;
    LivePoseState &livePose;
    std::atomic<bool> &runningFlag;
    const ApplicationRuntimeFactories &factories;
};

std::unique_ptr<ISessionGraphRuntime> CreateSessionGraphRuntime(const SessionGraphRuntimeFactoryConfig &config);

} // namespace SmartDrone::core::application
