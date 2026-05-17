#pragma once

#include "core/ports/slam_engine.h"

namespace smartdrone::adapters::slam {

class SlamEngineAdapter;

core::ports::SlamOutput MakeOkSlamOutput(const core::ports::SlamInputBatch &input,
                                         unsigned long mapId = 1);

void MarkSlamOutputPoseLost(core::ports::SlamOutput &out, int trackingState);

core::ports::SlamOutput MakePoseLostSlamOutput(SlamEngineAdapter *engine,
                                               const core::ports::SlamInputBatch &input,
                                               int trackingState,
                                               bool copyOrbMapSummary = false,
                                               bool maintainRealtimeContinuity = false);

} // namespace smartdrone::adapters::slam
