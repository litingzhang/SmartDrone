#include "adapters/slam/engine/slam_output_utils.h"

namespace SmartDrone::Adapters::Slam {

void CopyOptionalMapSummary(SlamEngineAdapter *, Core::Ports::SlamOutput &)
{
}

void MaintainOptionalRealtimeContinuity(
    SlamEngineAdapter *, Core::Ports::SlamOutput &,
    const Core::Ports::SlamInputBatch &)
{
}

} // namespace SmartDrone::Adapters::Slam
