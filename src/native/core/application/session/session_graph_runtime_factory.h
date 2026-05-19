#pragma once

#include <atomic>
#include <memory>

#include "core/application/session/session_graph_runtime.h"
#include "core/domain/runtime_mode.h"
#include "core/ports/pose_publisher.h"
#include "core/ports/slam_session_telemetry.h"

namespace smartdrone::core::application {

struct LiveRuntimeTuning;
struct LivePoseState;
struct UnifiedConfig;

struct SessionGraphRuntimeFactoryConfig {
    domain::RuntimeMode mode{domain::RuntimeMode::Idle};
    const UnifiedConfig &cfg;
    LiveRuntimeTuning &tuning;
    smartdrone::core::ports::ISlamSessionTelemetryPort &telemetry;
    smartdrone::core::ports::IPosePublisher &posePublisher;
    std::atomic<bool> &stop;
    LivePoseState &livePose;
    std::atomic<bool> &runningFlag;
};

std::unique_ptr<ISessionGraphRuntime> CreateSessionGraphRuntime(const SessionGraphRuntimeFactoryConfig &config);

} // namespace smartdrone::core::application
