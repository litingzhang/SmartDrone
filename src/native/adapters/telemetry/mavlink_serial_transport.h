#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>

namespace SmartDrone::Adapters::Telemetry {

class MavlinkSerialTransport {
  public:
    MavlinkSerialTransport() = default;
    MavlinkSerialTransport(const std::string &dev, int baud);
    ~MavlinkSerialTransport();

    void Open(const std::string &dev, int baud);
    void Close();
    ssize_t WriteSome(const uint8_t *data, size_t len) const;
    int PollReadable() const;
    ssize_t Read(uint8_t *buffer, size_t len) const;

  private:
    enum class TransportMode {
        Closed,
        Serial,
        Udp,
    };

    struct UdpOpenConfig {
        std::string remoteHost;
        uint16_t remotePort{0};
        std::string bindHost{"0.0.0.0"};
        uint16_t bindPort{0};
        bool hasRemote{false};
    };

    static unsigned int BaudToTermios(int baud);
    static bool IsUdpEndpoint(const std::string &dev);
    static UdpOpenConfig ParseUdpEndpoint(const std::string &dev);
    static bool ParseHostPort(const std::string &text, std::string &host,
                              uint16_t &port);
    static bool ParsePort(const std::string &text, uint16_t &port);
    static std::string StripUdpPrefix(const std::string &dev);
    static std::string ExtractBindValue(const std::string &query);
    static bool ResolveSockaddr(const std::string &host, uint16_t port,
                                bool passive, sockaddr_storage &out,
                                socklen_t &outLen);

    void OpenSerial(const std::string &dev, int baud);
    void OpenUdp(const std::string &dev);
    void ResetUdpPeer();
    ssize_t WriteUdp(const uint8_t *data, size_t len) const;
    ssize_t ReadUdp(uint8_t *buffer, size_t len) const;

    int m_fd{-1};
    TransportMode m_mode{TransportMode::Closed};
    mutable sockaddr_storage m_udpRemote{};
    mutable socklen_t m_udpRemoteLen{0};
    mutable std::atomic<bool> m_udpHaveRemote{false};
};

} // namespace SmartDrone::Adapters::Telemetry
