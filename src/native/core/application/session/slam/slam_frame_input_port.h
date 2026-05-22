#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>

#include "core/application/session/slam/slam_processing_support.h"
#include "core/application/session/slam/slam_frame_stage_context.h"
#include "core/application/session/slam/slam_frame_stage_data.h"
#include "core/application/session/slam/slam_frame_stage_state.h"
#include "core/application/session/slam/slam_frame_step_result.h"

namespace SmartDrone::Core::Application {

class SlamFrameInputPort final {
  public:
    SlamFrameInputPort(SlamFrameProcessingContext &context,
                       SlamFrameInputState &state,
                       SlamFrameSharedState &sharedState,
                       SlamFrameOutputState &outputState);

    SlamFrameStageResult AcquireAndPrepareFrame(SlamPreparedFrameData &frame);

  private:
    struct RuntimeFrameConfig {
        int configuredSlamInputFps{0};
        int effectiveSlamInputFps{0};
        uint8_t configuredFrontendValue{0};
        FeatureFrontend effectiveFrontend{FeatureFrontend::LkGfttPerFrame};
        bool dpvoEpgPacing{false};
        int visualFeatureLoadSheddingLevel{0};
        int visualFeatureBudgetWidth{0};
        int visualFeatureBudgetHeight{0};
    };

    struct StereoAcquireResult {
        SlamFrameStepResult stepResult{SlamFrameStepResult::Continue};
        bool sessionOk{true};
        bool hasFrame{false};
        std::chrono::steady_clock::time_point acquireStartTp;
        std::chrono::steady_clock::time_point acquireEndTp;
        StereoBatch stereoBatch;
    };

    struct FrameMetadata {
        bool sendImage{false};
        bool sendFeature{false};
        bool sendMap{false};
        int64_t pairDtMs{0};
        double rejectDtMs{0.0};
        uint64_t dropUnpairedL{0};
        uint64_t dropUnpairedR{0};
        size_t pendingL{0};
        size_t pendingR{0};
        int64_t captureTimestampNs{0};
        int64_t logicalFrameTimestampNs{0};
        double frameTime{0.0};
        double frameGapMs{0.0};
        double monoStepMs{0.0};
        StereoFrameQuality stereoQuality{};
        bool debugRightOnlyFeatures{false};
        bool extractFeatures{false};
        bool updatePointCloud{false};
    };

    struct SlamInputPreparation {
        SlamFrameStepResult stepResult{SlamFrameStepResult::Continue};
        bool ready{false};
        std::chrono::steady_clock::time_point imuStartTp;
        std::chrono::steady_clock::time_point imuEndTp;
        SmartDrone::Core::Ports::SlamInputBatch slamInput;
    };

    void SyncRequestedSlamMode();
    RuntimeFrameConfig ApplyRuntimeFrameConfig();
    void ApplySlamControlConfig(const RuntimeFrameConfig &config);
    void LogFrontendChange(const RuntimeFrameConfig &config);
    void LogInputFpsChange(const RuntimeFrameConfig &config);
    void LogVisualFeatureProfile(const RuntimeFrameConfig &config);
    StereoAcquireResult AcquireStereoBatch(const RuntimeFrameConfig &config);
    FrameMetadata BuildFrameMetadata(const StereoBatch &stereoBatch,
                                     const RuntimeFrameConfig &config);
    void PopulateFrameStreamFlags(FrameMetadata &metadata);
    void PopulateFrameTimingMetadata(
        FrameMetadata &metadata, const StereoBatch &stereoBatch,
        const SmartDrone::Core::Ports::ImageFrame &right,
        const SmartDrone::Core::Ports::CameraDiagnostics &cameraDiag);
    void PopulateFrameImageQuality(
        FrameMetadata &metadata,
        const SmartDrone::Core::Ports::ImageFrame &left,
        const SmartDrone::Core::Ports::ImageFrame &right);
    void PopulateFrameProcessingFlags(FrameMetadata &metadata);
    void MaybeLogFrameGap(const StereoBatch &stereoBatch,
                          const RuntimeFrameConfig &config,
                          const FrameMetadata &metadata);
    SlamInputPreparation PrepareSlamInput(const StereoBatch &stereoBatch,
                                          const FrameMetadata &metadata);
    void FillPreparedFrame(SlamPreparedFrameData &frame,
                           StereoAcquireResult &&acquire,
                           SlamInputPreparation &&input,
                           const RuntimeFrameConfig &config,
                           const FrameMetadata &metadata) const;

    SlamFrameProcessingContext &m_ctx;
    SlamFrameInputState &m_state;
    SlamFrameSharedState &m_sharedState;
    SlamFrameOutputState &m_outputState;
};

} // namespace SmartDrone::Core::Application
