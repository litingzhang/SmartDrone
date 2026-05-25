#pragma once

#include <atomic>
#include <chrono>

namespace SmartDrone::Common {

extern std::atomic<bool> g_runningFlag;
void SigIntHandler(int);
void RequestRuntimeStop();
bool RuntimeStopRequested();
bool WaitForRuntimeStop(std::chrono::milliseconds timeout);
void WaitUntilRuntimeStopRequested();

} // namespace SmartDrone::Common
