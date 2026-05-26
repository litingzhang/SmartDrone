#pragma once

#include <string>

#include "core/domain/feature_frontend.h"
#include "core/domain/runtime_mode.h"
#include "core/ports/visual_feature_frontend.h"

namespace SmartDrone::Core::Ports {

class ISlamRuntimeControl {
  public:
    virtual ~ISlamRuntimeControl() = default;

    virtual void SetOperationMode(Core::Domain::SlamOperationMode mode) = 0;
    virtual void SetFeatureFrontend(FeatureFrontend frontend) = 0;
    virtual void SetVisualFeatureFrontend(IVisualFeatureFrontend *frontend) = 0;
    virtual void SetVisualFeatureInputSizeLimit(int maxWidth,
                                                int maxHeight) = 0;
    virtual void SetStereoVoLoopClosure(bool enabled, float scale = 1.20f,
                                        float relaxation = 1.40f) = 0;
    virtual void SetStereoVoPerFrameAcceleration(std::string acceleration) = 0;
    virtual bool ShutdownAndSaveTrajectoryEuRoC(const std::string &path)
    {
        (void)path;
        return false;
    }
};

} // namespace SmartDrone::Core::Ports
