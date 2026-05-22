#pragma once

#include <cstdint>

namespace SmartDrone::Core::Application {

enum class SlamFrameStepResult : std::uint8_t {
    Continue,
    SessionAbort,
};

struct SlamFrameStageResult {
    SlamFrameStepResult stepResult{SlamFrameStepResult::Continue};
    bool sessionOk{true};
};

} // namespace SmartDrone::Core::Application
