#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "core/application/session/slam/slam_frame_step_result.h"
#include "core/application/state/perception_pipeline.h"
#include "core/application/state/pose_postprocessor.h"
#include "core/domain/feature_frontend.h"
#include "core/domain/runtime_mode.h"
#include "core/ports/slam_engine.h"

namespace SmartDrone::Core::Application {

struct SlamPreparedFrameData {
    std::chrono::steady_clock::time_point frameStartTp;
    std::chrono::steady_clock::time_point acquireStartTp;
    std::chrono::steady_clock::time_point acquireEndTp;
    std::chrono::steady_clock::time_point imuStartTp;
    std::chrono::steady_clock::time_point imuEndTp;
    StereoBatch stereoBatch;
    SmartDrone::Core::Ports::SlamInputBatch slamInput;
    SlamBackend slamBackend{SlamBackend::Klt};
    FeatureFrontend featureFrontend{FeatureFrontend::LkGfttPerFrame};
    int configuredSlamInputFps{0};
    int effectiveSlamInputFps{0};
    bool sendImage{false};
    bool sendFeature{false};
    bool sendMap{false};
    std::int64_t pairDtMs{0};
    double rejectDtMs{0.0};
    std::uint64_t dropUnpairedL{0};
    std::uint64_t dropUnpairedR{0};
    std::size_t pendingL{0};
    std::size_t pendingR{0};
    std::int64_t captureTimestampNs{0};
    std::int64_t logicalFrameTimestampNs{0};
    double frameTime{0.0};
    double frameGapMs{0.0};
    double monoStepMs{0.0};
    double meanL{0.0};
    double stdL{0.0};
    double meanR{0.0};
    double stdR{0.0};
    double sharpL{0.0};
    double sharpR{0.0};
    bool debugRightOnlyFeatures{false};
    bool extractFeatures{false};
    bool updatePointCloud{false};
};

struct SlamTrackedFrameData {
    std::shared_ptr<SlamPreparedFrameData> frame;
    SmartDrone::Core::Ports::SlamOutput slamOutput;
    std::chrono::steady_clock::time_point slamStartTp;
    std::chrono::steady_clock::time_point slamEndTp;
};

struct SlamPublishedFrameData {
    std::shared_ptr<SlamTrackedFrameData> frame;
    PosePostprocessor::Result poseResult;
    std::chrono::steady_clock::time_point cloudStartTp;
    std::chrono::steady_clock::time_point cloudEndTp;
    std::chrono::steady_clock::time_point udpStartTp;
    std::chrono::steady_clock::time_point udpEndTp;
    std::chrono::steady_clock::time_point postStartTp;
    std::chrono::steady_clock::time_point postEndTp;
    std::chrono::steady_clock::time_point livePoseStartTp;
    std::chrono::steady_clock::time_point livePoseEndTp;
    std::chrono::steady_clock::time_point publishStartTp;
    std::chrono::steady_clock::time_point publishEndTp;
    std::size_t pointCount{0};
    int trackingState{0};
    bool trackingUsable{false};
    std::uint8_t effectiveResetCounter{0};
    std::uint16_t effectiveResetMapCount{0};
};

} // namespace SmartDrone::Core::Application
