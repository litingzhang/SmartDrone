#pragma once

#include <mutex>
#include <string>

#include "core/application/config/app_args.h"
#include "core/ports/slam_runtime_control.h"

namespace SmartDrone::Core::Ports {
class IVisualFeatureFrontend;
} // namespace SmartDrone::Core::Ports

namespace SmartDrone::Core::Application {

class SlamRuntimeControlPort final {
  public:
    explicit SlamRuntimeControlPort(
        SmartDrone::Core::Ports::ISlamRuntimeControl *control);

    bool Available() const;
    void SetOperationMode(SmartDrone::Core::Domain::SlamOperationMode mode);
    void SetFeatureFrontend(FeatureFrontend frontend);
    void SetVisualFeatureFrontend(
        SmartDrone::Core::Ports::IVisualFeatureFrontend *frontend);
    void SetVisualFeatureInputSizeLimit(int maxWidth, int maxHeight);
    void SetStereoVoLoopClosure(bool enabled, float scale, float relaxation);
    void SetStereoVoPerFrameAcceleration(const std::string &acceleration);
    void StepBackend();

  private:
    SmartDrone::Core::Ports::ISlamRuntimeControl *m_control{nullptr};
    mutable std::mutex m_mu;
};

} // namespace SmartDrone::Core::Application
