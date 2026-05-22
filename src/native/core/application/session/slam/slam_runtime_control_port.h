#pragma once

#include <mutex>
#include <string>

#include "core/application/config/app_args.h"
#include "core/ports/slam_runtime_control.h"

namespace SmartDrone::core::ports {
class IVisualFeatureFrontend;
} // namespace SmartDrone::core::ports

namespace SmartDrone::core::application {

class SlamRuntimeControlPort final {
  public:
    explicit SlamRuntimeControlPort(
        SmartDrone::core::ports::ISlamRuntimeControl *control);

    bool Available() const;
    void SetOperationMode(SmartDrone::core::domain::SlamOperationMode mode);
    void SetFeatureFrontend(FeatureFrontend frontend);
    void SetVisualFeatureFrontend(
        SmartDrone::core::ports::IVisualFeatureFrontend *frontend);
    void SetVisualFeatureInputSizeLimit(int maxWidth, int maxHeight);
    void SetStereoVoLoopClosure(bool enabled, float scale, float relaxation);
    void SetStereoVoPerFrameAcceleration(const std::string &acceleration);
    void StepBackend();

  private:
    SmartDrone::core::ports::ISlamRuntimeControl *m_control{nullptr};
    mutable std::mutex m_mu;
};

} // namespace SmartDrone::core::application
