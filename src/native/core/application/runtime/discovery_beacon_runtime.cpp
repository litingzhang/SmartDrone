#include "core/application/runtime/discovery_beacon_runtime.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <iostream>

namespace SmartDrone::Core::Application {
namespace {

constexpr const char *kDiscoveryMagic = "smartdrone_discovery";
constexpr auto kDiscoveryPeriod = std::chrono::seconds(1);
constexpr auto kDiscoveryOpenRetryPeriod = std::chrono::seconds(1);

bool SetSocketNonBlocking(int fd)
{
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        return false;
    }
    return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

} // namespace

DiscoveryBeaconRuntime::DiscoveryBeaconRuntime(int discoveryPort, int cmdPort,
                                               int videoPort)
    : m_discoveryPort(discoveryPort),
      m_cmdPort(cmdPort),
      m_videoPort(videoPort)
{
}

DiscoveryBeaconRuntime::~DiscoveryBeaconRuntime()
{
    Stop();
}

bool DiscoveryBeaconRuntime::Start()
{
    if (m_fd >= 0) {
        return true;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!CanRetryOpen(now)) {
        return false;
    }

    m_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (m_fd < 0) {
        m_nextOpenAttempt = now + kDiscoveryOpenRetryPeriod;
        std::cerr << "[discovery] socket open failed\n";
        return false;
    }
    if (!SetSocketNonBlocking(m_fd)) {
        Stop();
        m_nextOpenAttempt = now + kDiscoveryOpenRetryPeriod;
        std::cerr << "[discovery] socket nonblock failed\n";
        return false;
    }

    int one = 1;
    ::setsockopt(m_fd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));
    ::setsockopt(m_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    m_dst.sin_family = AF_INET;
    m_dst.sin_port = htons(static_cast<uint16_t>(m_discoveryPort));
    m_dst.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    m_payload = std::string(kDiscoveryMagic) + ";device=cm5;cmd=" +
                std::to_string(m_cmdPort) + ";video=" +
                std::to_string(m_videoPort);
    m_nextOpenAttempt = {};
    return true;
}

void DiscoveryBeaconRuntime::Stop()
{
    if (m_fd < 0) {
        return;
    }
    ::close(m_fd);
    m_fd = -1;
}

void DiscoveryBeaconRuntime::OnGraphTick()
{
    if (!Start()) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (m_lastSent.time_since_epoch().count() != 0 &&
        now - m_lastSent < kDiscoveryPeriod) {
        return;
    }

    m_lastSent = now;
    const ssize_t sent =
        ::sendto(m_fd, m_payload.data(), m_payload.size(), 0,
                 reinterpret_cast<const sockaddr *>(&m_dst), sizeof(m_dst));
    LogFirstSendResult(sent);
}

void DiscoveryBeaconRuntime::Step()
{
    OnGraphTick();
}

bool DiscoveryBeaconRuntime::CanRetryOpen(
    std::chrono::steady_clock::time_point now) const
{
    return m_nextOpenAttempt.time_since_epoch().count() == 0 ||
           now >= m_nextOpenAttempt;
}

void DiscoveryBeaconRuntime::LogFirstSendResult(ssize_t sent)
{
    if (!m_firstLog) {
        return;
    }

    m_firstLog = false;
    if (sent < 0) {
        std::cerr << "[discovery] broadcast failed errno=" << errno << "\n";
        return;
    }
    std::cerr << "[discovery] broadcasting on udp/" << m_discoveryPort
              << " cmd=" << m_cmdPort << " video=" << m_videoPort << "\n";
}

} // namespace SmartDrone::Core::Application
