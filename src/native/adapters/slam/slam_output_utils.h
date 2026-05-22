#pragma once

#include "core/ports/slam_backend_state.h"
#include "core/ports/slam_engine.h"

namespace SmartDrone::Adapters::Slam {

class SlamEngineAdapter;

Core::Ports::SlamOutput MakeOkSlamOutput(const Core::Ports::SlamInputBatch &input,
                                         unsigned long mapId = 1);

void MarkSlamOutputPoseLost(Core::Ports::SlamOutput &out, int trackingState);

void CopyMapSummaryToOutput(const Core::Ports::SlamMapSummary &summary,
                            Core::Ports::SlamOutput &out);

void CopyBackendStatsToOutput(const Core::Ports::SlamBackendStats &stats,
                              Core::Ports::SlamOutput &out);

void CopyOptionalMapSummary(SlamEngineAdapter *engine,
                            Core::Ports::SlamOutput &out);
void MaintainOptionalRealtimeContinuity(
    SlamEngineAdapter *engine, Core::Ports::SlamOutput &out,
    const Core::Ports::SlamInputBatch &input);

Core::Ports::SlamOutput MakePoseLostSlamOutput(SlamEngineAdapter *engine,
                                               const Core::Ports::SlamInputBatch &input,
                                               int trackingState,
                                               bool copyOrbMapSummary = false,
                                               bool maintainRealtimeContinuity = false);

} // namespace SmartDrone::Adapters::Slam
