#pragma once

#include <mutex>
#include <string>

#include "core/application/config/app_args.h"
#include "core/ports/slam_runtime_control.h"

namespace smartdrone::core::ports {
class IVisualFeatureFrontend;
} // namespace smartdrone::core::ports

namespace smartdrone::core::application {

class SlamRuntimeControlPort final {
  public:
    explicit SlamRuntimeControlPort(
        smartdrone::core::ports::ISlamRuntimeControl *control);

    bool Available() const;
    void SetOperationMode(smartdrone::core::domain::SlamOperationMode mode);
    void SetFeatureFrontend(FeatureFrontend frontend);
    void SetVisualFeatureFrontend(
        smartdrone::core::ports::IVisualFeatureFrontend *frontend);
    void SetVisualFeatureInputSizeLimit(int maxWidth, int maxHeight);
    void SetStereoVoLoopClosure(bool enabled, float scale, float relaxation);
    void SetStereoVoPerFrameAcceleration(const std::string &acceleration);
    void StepBackend();

  private:
    smartdrone::core::ports::ISlamRuntimeControl *m_control{nullptr};
    mutable std::mutex m_mu;
};

} // namespace smartdrone::core::application
