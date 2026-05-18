#pragma once

#include "core/ports/slam_backend_state.h"
#include "core/ports/slam_engine.h"

namespace smartdrone::adapters::slam {

class SlamEngineAdapter;

core::ports::SlamOutput MakeOkSlamOutput(const core::ports::SlamInputBatch &input,
                                         unsigned long mapId = 1);

void MarkSlamOutputPoseLost(core::ports::SlamOutput &out, int trackingState);

void CopyMapSummaryToOutput(const core::ports::SlamMapSummary &summary,
                            core::ports::SlamOutput &out);

void CopyBackendStatsToOutput(const core::ports::SlamBackendStats &stats,
                              core::ports::SlamOutput &out);

core::ports::SlamOutput MakePoseLostSlamOutput(SlamEngineAdapter *engine,
                                               const core::ports::SlamInputBatch &input,
                                               int trackingState,
                                               bool copyOrbMapSummary = false,
                                               bool maintainRealtimeContinuity = false);

} // namespace smartdrone::adapters::slam
