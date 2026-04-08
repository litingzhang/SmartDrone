#pragma once

#include <atomic>
#include <thread>

namespace smartdrone::common::discovery {

std::thread StartUdpDiscoveryBeaconThread(int discoveryPort, int cmdPort, int videoPort, std::atomic<bool> &runningFlag);

} // namespace smartdrone::common::discovery
