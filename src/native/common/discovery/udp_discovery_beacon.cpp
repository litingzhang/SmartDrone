#include "common/discovery/udp_discovery_beacon.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "common/thread_launch.h"

namespace smartdrone::common::discovery {
namespace {

constexpr const char *kDiscoveryMagic = "smartdrone_discovery";
constexpr auto kDiscoveryPeriod = std::chrono::seconds(1);

} // namespace

std::thread StartUdpDiscoveryBeaconThread(int discoveryPort, int cmdPort, int videoPort, std::atomic<bool> &runningFlag)
{
    return SMARTDRONE_START_THREAD(smartdrone::common::ThreadRole::DiscoveryBeacon, "UdpDiscoveryBeacon",
                                   [discoveryPort, cmdPort, videoPort, &runningFlag]() {
                                       int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
                                       if (fd < 0) {
                                           std::cerr << "[discovery] socket open failed\n";
                                           return;
                                       }

                                       int one = 1;
                                       ::setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));
                                       ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

                                       sockaddr_in dst{};
                                       dst.sin_family = AF_INET;
                                       dst.sin_port = htons(static_cast<uint16_t>(discoveryPort));
                                       dst.sin_addr.s_addr = htonl(INADDR_BROADCAST);

                                       const std::string payload = std::string(kDiscoveryMagic) +
                                                                   ";device=cm5;cmd=" + std::to_string(cmdPort) +
                                                                   ";video=" + std::to_string(videoPort);
                                       bool firstLog = true;
                                       while (runningFlag.load()) {
                                           const ssize_t sent = ::sendto(fd, payload.data(), payload.size(), 0,
                                                                         reinterpret_cast<const sockaddr *>(&dst),
                                                                         sizeof(dst));
                                           if (sent < 0) {
                                               if (firstLog) {
                                                   firstLog = false;
                                                   std::cerr << "[discovery] broadcast failed errno=" << errno << "\n";
                                               }
                                           } else if (firstLog) {
                                               firstLog = false;
                                               std::cerr << "[discovery] broadcasting on udp/" << discoveryPort
                                                         << " cmd=" << cmdPort << " video=" << videoPort << "\n";
                                           }
                                           std::this_thread::sleep_for(kDiscoveryPeriod);
                                       }

                                       ::close(fd);
                                   });
}

} // namespace smartdrone::common::discovery
