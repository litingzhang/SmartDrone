#pragma once

#include "core/domain/runtime_mode.h"

namespace SmartDrone::Core::Application {

class ModeManager {
  public:
    using RuntimeMode = SmartDrone::Core::Domain::RuntimeMode;

    RuntimeMode DesiredMode() const;
    RuntimeMode ActiveMode() const;
    void RequestMode(RuntimeMode mode);
    void RequestRestart();
    bool RestartRequested() const;
    bool ShouldStopActiveSession() const;
    void MarkSessionJoined();
    void MarkSessionLaunching(RuntimeMode mode);

  private:
    RuntimeMode m_desiredMode{RuntimeMode::Idle};
    RuntimeMode m_activeMode{RuntimeMode::Idle};
    bool m_restartRequested{false};
};

} // namespace SmartDrone::Core::Application
