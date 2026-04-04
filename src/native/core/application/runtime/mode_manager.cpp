#include "core/application/runtime/mode_manager.h"

namespace smartdrone::core::application {

ModeManager::RuntimeMode ModeManager::DesiredMode() const
{
    return m_desiredMode;
}

ModeManager::RuntimeMode ModeManager::ActiveMode() const
{
    return m_activeMode;
}

void ModeManager::RequestMode(RuntimeMode mode)
{
    m_desiredMode = mode;
    m_restartRequested = true;
}

void ModeManager::RequestRestart()
{
    m_restartRequested = true;
}

bool ModeManager::RestartRequested() const
{
    return m_restartRequested;
}

bool ModeManager::ShouldStopActiveSession() const
{
    return m_activeMode != RuntimeMode::Idle && (m_restartRequested || m_desiredMode != m_activeMode);
}

void ModeManager::MarkSessionJoined()
{
    m_activeMode = RuntimeMode::Idle;
    m_restartRequested = false;
}

void ModeManager::MarkSessionLaunching(RuntimeMode mode)
{
    m_activeMode = mode;
    m_restartRequested = false;
}

}  // namespace smartdrone::core::application
