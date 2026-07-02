#include "adapters/telemetry/mavlink_serial_transport.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

class UdpSocket {
  public:
    UdpSocket()
    {
        m_fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    }

    ~UdpSocket()
    {
        if (m_fd >= 0) {
            ::close(m_fd);
        }
    }

    bool BindLoopback()
    {
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        return ::bind(m_fd, reinterpret_cast<sockaddr *>(&addr),
                      sizeof(addr)) == 0;
    }

    uint16_t Port() const
    {
        sockaddr_in addr{};
        socklen_t len = sizeof(addr);
        if (::getsockname(m_fd, reinterpret_cast<sockaddr *>(&addr), &len) !=
            0) {
            return 0;
        }
        return ntohs(addr.sin_port);
    }

    bool Receive(std::array<uint8_t, 8> &buffer, ssize_t &length) const
    {
        length = ::recv(m_fd, buffer.data(), buffer.size(), 0);
        return length > 0;
    }

    bool SendToLoopback(uint16_t port, const std::array<uint8_t, 4> &payload)
    {
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(port);
        return ::sendto(m_fd, payload.data(), payload.size(), 0,
                        reinterpret_cast<sockaddr *>(&addr),
                        sizeof(addr)) == static_cast<ssize_t>(payload.size());
    }

  private:
    int m_fd{-1};
};

std::string BuildUdpEndpoint(uint16_t remotePort)
{
    return "udp://127.0.0.1:" + std::to_string(remotePort) +
           "?bind=127.0.0.1:0";
}

uint16_t FindAvailableUdpPort()
{
    UdpSocket socket;
    if (!socket.BindLoopback()) {
        return 0;
    }
    return socket.Port();
}

void WaitReadable(SmartDrone::Adapters::Telemetry::MavlinkSerialTransport &transport)
{
    for (int attempt = 0; attempt < 20; ++attempt) {
        if (transport.PollReadable() > 0) {
            return;
        }
        usleep(1000);
    }
}

} // namespace

TEST(MavlinkTransportTest, UdpEndpointSendsDatagrams)
{
    UdpSocket receiver;
    ASSERT_TRUE(receiver.BindLoopback());
    ASSERT_NE(receiver.Port(), 0);

    SmartDrone::Adapters::Telemetry::MavlinkSerialTransport transport(
        BuildUdpEndpoint(receiver.Port()), 921600);
    const std::array<uint8_t, 4> payload{0xFE, 0x01, 0x02, 0x03};
    ASSERT_EQ(transport.WriteSome(payload.data(), payload.size()),
              static_cast<ssize_t>(payload.size()));

    std::array<uint8_t, 8> received{};
    ssize_t receivedLength = 0;
    ASSERT_TRUE(receiver.Receive(received, receivedLength));
    ASSERT_EQ(receivedLength, static_cast<ssize_t>(payload.size()));
    EXPECT_EQ(std::memcmp(received.data(), payload.data(), payload.size()), 0);
}

TEST(MavlinkTransportTest, UdpListenLearnsReplyPeer)
{
    const uint16_t listenPort = FindAvailableUdpPort();
    ASSERT_NE(listenPort, 0);
    SmartDrone::Adapters::Telemetry::MavlinkSerialTransport transport(
        "udp-listen://" + std::to_string(listenPort), 921600);
    UdpSocket peer;
    ASSERT_TRUE(peer.BindLoopback());
    const std::array<uint8_t, 4> payload{1, 2, 3, 4};
    ASSERT_TRUE(peer.SendToLoopback(listenPort, payload));

    std::array<uint8_t, 8> received{};
    WaitReadable(transport);
    ASSERT_GT(transport.PollReadable(), 0);
    ASSERT_EQ(transport.Read(received.data(), received.size()),
              static_cast<ssize_t>(payload.size()));
    ASSERT_EQ(transport.WriteSome(payload.data(), payload.size()),
              static_cast<ssize_t>(payload.size()));

    ssize_t receivedLength = 0;
    ASSERT_TRUE(peer.Receive(received, receivedLength));
    ASSERT_EQ(receivedLength, static_cast<ssize_t>(payload.size()));
    EXPECT_EQ(std::memcmp(received.data(), payload.data(), payload.size()), 0);
}
