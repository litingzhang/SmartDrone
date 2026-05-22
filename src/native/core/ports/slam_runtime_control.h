#pragma once

#include <string>

#include "core/application/config/app_args.h"
#include "core/domain/runtime_mode.h"
#include "core/ports/visual_feature_frontend.h"

namespace SmartDrone::core::ports {

class ISlamRuntimeControl {
  public:
    virtual ~ISlamRuntimeControl() = default;

    virtual void SetOperationMode(core::domain::SlamOperationMode mode) = 0;
    virtual void SetFeatureFrontend(FeatureFrontend frontend) = 0;
    virtual void SetVisualFeatureFrontend(IVisualFeatureFrontend *frontend) = 0;
    virtual void SetVisualFeatureInputSizeLimit(int maxWidth,
                                                int maxHeight) = 0;
    virtual void SetStereoVoLoopClosure(bool enabled, float scale = 1.20f,
                                        float relaxation = 1.40f) = 0;
    virtual void SetStereoVoPerFrameAcceleration(std::string acceleration) = 0;
    virtual void StepBackend()
    {
    }
    virtual bool ShutdownAndSaveTrajectoryEuRoC(const std::string &path)
    {
        (void)path;
        return false;
    }
};

} // namespace SmartDrone::core::ports
