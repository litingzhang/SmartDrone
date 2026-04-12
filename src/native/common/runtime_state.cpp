#include "common/runtime_state.h"

namespace smartdrone::common {

std::atomic<bool> g_runningFlag{true};

void SigIntHandler(int) { g_runningFlag.store(false); }

} // namespace smartdrone::common

