#include "core/application/session/slam/slam_runtime_control_port.h"

namespace SmartDrone::Core::Application {

SlamRuntimeControlPort::SlamRuntimeControlPort(
    SmartDrone::Core::Ports::ISlamRuntimeControl *control)
    : m_control(control)
{
}

bool SlamRuntimeControlPort::Available() const
{
    return m_control != nullptr;
}

void SlamRuntimeControlPort::SetOperationMode(
    SmartDrone::Core::Domain::SlamOperationMode mode)
{
    if (m_control == nullptr) {
        return;
    }
    m_control->SetOperationMode(mode);
}

void SlamRuntimeControlPort::SetFeatureFrontend(FeatureFrontend frontend)
{
    if (m_control == nullptr) {
        return;
    }
    m_control->SetFeatureFrontend(frontend);
}

void SlamRuntimeControlPort::SetVisualFeatureFrontend(
    SmartDrone::Core::Ports::IVisualFeatureFrontend *frontend)
{
    if (m_control == nullptr) {
        return;
    }
    m_control->SetVisualFeatureFrontend(frontend);
}

void SlamRuntimeControlPort::SetVisualFeatureInputSizeLimit(int maxWidth,
                                                            int maxHeight)
{
    if (m_control == nullptr) {
        return;
    }
    m_control->SetVisualFeatureInputSizeLimit(maxWidth, maxHeight);
}

void SlamRuntimeControlPort::SetStereoVoLoopClosure(bool enabled,
                                                    float scale,
                                                    float relaxation)
{
    if (m_control == nullptr) {
        return;
    }
    m_control->SetStereoVoLoopClosure(enabled, scale, relaxation);
}

void SlamRuntimeControlPort::SetStereoVoPerFrameAcceleration(
    const std::string &acceleration)
{
    if (m_control == nullptr) {
        return;
    }
    m_control->SetStereoVoPerFrameAcceleration(acceleration);
}

void SlamRuntimeControlPort::StepBackend()
{
    if (m_control == nullptr) {
        return;
    }
    m_control->StepBackend();
}

} // namespace SmartDrone::Core::Application
