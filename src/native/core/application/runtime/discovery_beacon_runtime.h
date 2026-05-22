#pragma once

#include <chrono>
#include <string>

#include <netinet/in.h>
#include <sys/types.h>

namespace SmartDrone::Core::Application {

class DiscoveryBeaconRuntime final {
  public:
    DiscoveryBeaconRuntime(int discoveryPort, int cmdPort, int videoPort);
    ~DiscoveryBeaconRuntime();

    bool Start();
    void Stop();
    void OnGraphTick();
    void Step();

  private:
    bool CanRetryOpen(std::chrono::steady_clock::time_point now) const;
    void LogFirstSendResult(ssize_t sent);

    int m_discoveryPort{0};
    int m_cmdPort{0};
    int m_videoPort{0};
    int m_fd{-1};
    sockaddr_in m_dst{};
    std::string m_payload;
    std::chrono::steady_clock::time_point m_nextOpenAttempt{};
    bool m_firstLog{true};
};

} // namespace SmartDrone::Core::Application
