#pragma once

#include <string>

#include "adapters/slam/external_feature_frontend_client.h"
#include "core/application/config/app_args.h"
#include "core/domain/runtime_mode.h"

namespace smartdrone::adapters::slam {

class ISlamRuntimeControl {
  public:
    virtual ~ISlamRuntimeControl() = default;

    virtual void SetOperationMode(core::domain::SlamOperationMode mode) = 0;
    virtual void SetFeatureFrontend(FeatureFrontend frontend) = 0;
    virtual void SetExternalFeatureFrontendClient(ExternalFeatureFrontendClient *client) = 0;
    virtual void SetExternalFeatureInputSizeLimit(int maxWidth, int maxHeight) = 0;
    virtual void SetStereoVoLoopClosure(bool enabled, float scale = 1.20f, float relaxation = 1.40f) = 0;
    virtual void SetStereoVoPerFrameAcceleration(std::string acceleration) = 0;
    virtual bool ShutdownAndSaveTrajectoryEuRoC(const std::string &path)
    {
        (void)path;
        return false;
    }
};

} // namespace smartdrone::adapters::slam
