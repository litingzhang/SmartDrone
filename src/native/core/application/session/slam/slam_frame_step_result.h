#pragma once

#include <cstdint>

namespace smartdrone::core::application {

enum class SlamFrameStepResult : std::uint8_t {
    Continue,
    SessionAbort,
};

struct SlamFrameStageResult {
    SlamFrameStepResult stepResult{SlamFrameStepResult::Continue};
    bool sessionOk{true};
};

} // namespace smartdrone::core::application
