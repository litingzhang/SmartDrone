#pragma once

#include "core/domain/runtime_mode.hpp"

namespace smartdrone::core::application {

class ModeManager {
public:
    using RuntimeMode = smartdrone::core::domain::RuntimeMode;

    RuntimeMode DesiredMode() const { return m_desiredMode; }
    RuntimeMode ActiveMode() const { return m_activeMode; }

    void RequestMode(RuntimeMode mode)
    {
        m_desiredMode = mode;
        m_restartRequested = true;
    }

    void RequestRestart() { m_restartRequested = true; }

    bool RestartRequested() const { return m_restartRequested; }

    bool ShouldStopActiveSession() const
    {
        return m_activeMode != RuntimeMode::Idle &&
               (m_restartRequested || m_desiredMode != m_activeMode);
    }

    void MarkSessionJoined()
    {
        m_activeMode = RuntimeMode::Idle;
        m_restartRequested = false;
    }

    void MarkSessionLaunching(RuntimeMode mode)
    {
        m_activeMode = mode;
        m_restartRequested = false;
    }

private:
    RuntimeMode m_desiredMode{RuntimeMode::Idle};
    RuntimeMode m_activeMode{RuntimeMode::Idle};
    bool m_restartRequested{false};
};

}  // namespace smartdrone::core::application
