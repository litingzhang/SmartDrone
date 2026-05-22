#pragma once

#include <functional>
#include <memory>

#include "core/application/runtime/runtime_provider_metadata.h"
#include "core/application/session/slam/slam_session_resource_factory.h"
#include "core/application/session/stream/preview_output_port.h"
#include "core/ports/camera_provider.h"
#include "core/ports/imu_provider.h"

namespace SmartDrone::core::application {

struct ApplicationRuntimeFactories {
    using CreateCameraProviderFn =
        std::function<std::unique_ptr<SmartDrone::core::ports::ICameraProvider>()>;
    using MakeCameraOpenConfigFn =
        std::function<SmartDrone::core::ports::CameraOpenConfig(
            const MainRuntimeAliases &)>;
    using CreateSlamEngineResourcesFn =
        std::function<SlamSessionEngineResources(
            const SlamSessionEngineResourceConfig &)>;
    using CreateImuProviderFn =
        std::function<std::unique_ptr<SmartDrone::core::ports::IImuProvider>(
            ImuThreadState &, const MainRuntimeAliases &)>;
    using StartVisualFeatureFrontendSessionFn =
        std::function<SlamVisualFeatureFrontendStartResult(
            const MainRuntimeAliases &, const UnifiedConfig &)>;
    using CreatePreviewOutputRuntimeFn =
        std::function<std::unique_ptr<IPreviewOutputRuntime>()>;

    CreateCameraProviderFn createCameraProvider;
    MakeCameraOpenConfigFn makeCameraOpenConfig;
    CreateSlamEngineResourcesFn createSlamEngineResources;
    CreateImuProviderFn createImuProvider;
    StartVisualFeatureFrontendSessionFn startVisualFeatureFrontendSession;
    CreatePreviewOutputRuntimeFn createPreviewOutputRuntime;
    CameraRuntimeProviderMetadata cameraProvider;

    bool Valid() const
    {
        return createCameraProvider && makeCameraOpenConfig &&
               createSlamEngineResources && createImuProvider &&
               startVisualFeatureFrontendSession && createPreviewOutputRuntime &&
               !cameraProvider.providerName.empty();
    }
};

} // namespace SmartDrone::core::application
