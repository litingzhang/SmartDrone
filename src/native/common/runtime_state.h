#pragma once

#include <atomic>

namespace smartdrone::common {

extern std::atomic<bool> g_runningFlag;
void SigIntHandler(int);
void RequestRuntimeStop();
void WaitUntilRuntimeStopRequested();

} // namespace smartdrone::common
