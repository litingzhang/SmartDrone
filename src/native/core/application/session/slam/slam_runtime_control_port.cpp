#include "core/application/session/slam/slam_runtime_control_port.h"

namespace smartdrone::core::application {

SlamRuntimeControlPort::SlamRuntimeControlPort(
    smartdrone::core::ports::ISlamRuntimeControl *control)
    : m_control(control)
{
}

bool SlamRuntimeControlPort::Available() const
{
    std::lock_guard<std::mutex> lock(m_mu);
    return m_control != nullptr;
}

void SlamRuntimeControlPort::SetOperationMode(
    smartdrone::core::domain::SlamOperationMode mode)
{
    std::lock_guard<std::mutex> lock(m_mu);
    if (m_control == nullptr) {
        return;
    }
    m_control->SetOperationMode(mode);
}

void SlamRuntimeControlPort::SetFeatureFrontend(FeatureFrontend frontend)
{
    std::lock_guard<std::mutex> lock(m_mu);
    if (m_control == nullptr) {
        return;
    }
    m_control->SetFeatureFrontend(frontend);
}

void SlamRuntimeControlPort::SetVisualFeatureFrontend(
    smartdrone::core::ports::IVisualFeatureFrontend *frontend)
{
    std::lock_guard<std::mutex> lock(m_mu);
    if (m_control == nullptr) {
        return;
    }
    m_control->SetVisualFeatureFrontend(frontend);
}

void SlamRuntimeControlPort::SetVisualFeatureInputSizeLimit(int maxWidth,
                                                            int maxHeight)
{
    std::lock_guard<std::mutex> lock(m_mu);
    if (m_control == nullptr) {
        return;
    }
    m_control->SetVisualFeatureInputSizeLimit(maxWidth, maxHeight);
}

void SlamRuntimeControlPort::SetStereoVoLoopClosure(bool enabled,
                                                    float scale,
                                                    float relaxation)
{
    std::lock_guard<std::mutex> lock(m_mu);
    if (m_control == nullptr) {
        return;
    }
    m_control->SetStereoVoLoopClosure(enabled, scale, relaxation);
}

void SlamRuntimeControlPort::SetStereoVoPerFrameAcceleration(
    const std::string &acceleration)
{
    std::lock_guard<std::mutex> lock(m_mu);
    if (m_control == nullptr) {
        return;
    }
    m_control->SetStereoVoPerFrameAcceleration(acceleration);
}

void SlamRuntimeControlPort::StepBackend()
{
    std::lock_guard<std::mutex> lock(m_mu);
    if (m_control == nullptr) {
        return;
    }
    m_control->StepBackend();
}

} // namespace smartdrone::core::application
