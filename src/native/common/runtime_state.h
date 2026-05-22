#pragma once

#include <atomic>

namespace SmartDrone::Common {

extern std::atomic<bool> g_runningFlag;
void SigIntHandler(int);
void RequestRuntimeStop();
bool RuntimeStopRequested();
void WaitUntilRuntimeStopRequested();

} // namespace SmartDrone::Common
