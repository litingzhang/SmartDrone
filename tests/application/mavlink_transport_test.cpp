#include "adapters/telemetry/mavlink_serial_transport.h"
#include "adapters/telemetry/mavlink_pose_publisher.h"
#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "common/mavlink.h"
#include "common/time_utils.h"
#include "core/application/state/frame_timing_tracker.h"

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

    template <std::size_t N>
    bool Receive(std::array<uint8_t, N> &buffer, ssize_t &length) const
    {
        length = ::recv(m_fd, buffer.data(), buffer.size(), 0);
        return length > 0;
    }

    template <std::size_t N>
    bool SendToLoopback(uint16_t port, const std::array<uint8_t, N> &payload)
    {
        return SendToLoopback(port, payload.data(), payload.size());
    }

    bool SendToLoopback(uint16_t port, const uint8_t *payload, std::size_t size)
    {
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(port);
        return ::sendto(m_fd, payload, size, 0,
                        reinterpret_cast<sockaddr *>(&addr),
                        sizeof(addr)) == static_cast<ssize_t>(size);
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

bool ReceiveMavlinkMessage(const UdpSocket &socket, mavlink_message_t &message)
{
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    ssize_t length = 0;
    for (int attempt = 0; attempt < 20; ++attempt) {
        if (!socket.Receive(buffer, length)) {
            usleep(1000);
            continue;
        }
        mavlink_status_t status{};
        for (ssize_t index = 0; index < length; ++index) {
            if (mavlink_parse_char(MAVLINK_COMM_1, buffer[index], &message, &status)) {
                return true;
            }
        }
    }
    return false;
}

bool ReceiveMavlinkMessageOfType(const UdpSocket &socket, uint32_t messageId,
                                 mavlink_message_t &message)
{
    for (int attempt = 0; attempt < 4; ++attempt) {
        if (!ReceiveMavlinkMessage(socket, message)) {
            return false;
        }
        if (message.msgid == messageId) {
            return true;
        }
    }
    return false;
}

std::array<uint8_t, MAVLINK_MAX_PACKET_LEN>
SerializeMavlinkMessage(const mavlink_message_t &message, uint16_t &length)
{
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
    length = mavlink_msg_to_send_buffer(bytes.data(), &message);
    return bytes;
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

TEST(MavlinkTransportTest, OdometryUsesDeclaredPoseFrameAndBodyVelocity)
{
    UdpSocket receiver;
    ASSERT_TRUE(receiver.BindLoopback());
    Px4MavlinkGateway gateway(BuildUdpEndpoint(receiver.Port()), 921600);
    SmartDrone::Core::Application::FrameTimingTracker timing;
    gateway.SetFrameTimingTracker(&timing);
    timing.UpsertCapture(7, MonoTimeUs() * 1000ULL, MonoTimeUs() * 1000ULL);

    SmartDrone::Adapters::Telemetry::MavlinkPosePublisher publisher(gateway);
    SmartDrone::Core::Ports::PosePublishRequest request{};
    request.frameId = 7;
    request.pose = {true, 1.0f, 2.0f, -3.0f, 0.7071068f, 0.0f, 0.0f, 0.7071068f};
    request.velocity = {1.0f, 0.0f, 0.0f, true};
    request.referenceFrame = SmartDrone::Core::Ports::PoseReferenceFrame::LocalNed;
    request.quality = SmartDrone::Core::Ports::PoseQuality::Good;
    publisher.PublishPose(request);
    gateway.StepTx();

    mavlink_message_t message{};
    ASSERT_TRUE(ReceiveMavlinkMessage(receiver, message));
    ASSERT_EQ(message.msgid, MAVLINK_MSG_ID_ODOMETRY);
    mavlink_odometry_t odometry{};
    mavlink_msg_odometry_decode(&message, &odometry);
    EXPECT_EQ(odometry.frame_id, MAV_FRAME_LOCAL_NED);
    EXPECT_EQ(odometry.child_frame_id, MAV_FRAME_BODY_FRD);
    EXPECT_NEAR(odometry.vx, 0.0f, 1.0e-5f);
    EXPECT_NEAR(odometry.vy, -1.0f, 1.0e-5f);
    EXPECT_NEAR(odometry.vz, 0.0f, 1.0e-5f);
    EXPECT_EQ(odometry.quality, 100);
}

TEST(MavlinkTransportTest, LostOdometryReportsFailure)
{
    UdpSocket receiver;
    ASSERT_TRUE(receiver.BindLoopback());
    Px4MavlinkGateway gateway(BuildUdpEndpoint(receiver.Port()), 921600);
    SmartDrone::Core::Application::FrameTimingTracker timing;
    gateway.SetFrameTimingTracker(&timing);
    timing.UpsertCapture(8, MonoTimeUs() * 1000ULL, MonoTimeUs() * 1000ULL);

    SmartDrone::Adapters::Telemetry::MavlinkPosePublisher publisher(gateway);
    SmartDrone::Core::Ports::PosePublishRequest request{};
    request.frameId = 8;
    request.pose = {true, 1.0f, 2.0f, -3.0f, 1.0f, 0.0f, 0.0f, 0.0f};
    request.referenceFrame = SmartDrone::Core::Ports::PoseReferenceFrame::LocalFrd;
    request.quality = SmartDrone::Core::Ports::PoseQuality::Lost;
    publisher.PublishPose(request);
    gateway.StepTx();

    mavlink_message_t message{};
    ASSERT_TRUE(ReceiveMavlinkMessage(receiver, message));
    mavlink_odometry_t odometry{};
    mavlink_msg_odometry_decode(&message, &odometry);
    EXPECT_EQ(odometry.frame_id, MAV_FRAME_LOCAL_FRD);
    EXPECT_EQ(odometry.quality, -1);
    EXPECT_TRUE(std::isnan(odometry.vx));
}

TEST(MavlinkTransportTest, TimesyncRequestReceivesCompanionTimestamp)
{
    const uint16_t listenPort = FindAvailableUdpPort();
    ASSERT_NE(listenPort, 0);
    Px4MavlinkGateway gateway("udp-listen://" + std::to_string(listenPort), 921600);
    UdpSocket peer;
    ASSERT_TRUE(peer.BindLoopback());

    mavlink_message_t request{};
    constexpr int64_t PX4_TIMESTAMP_NS = 123456789;
    mavlink_msg_timesync_pack(1, 1, &request, 0, PX4_TIMESTAMP_NS, 42,
                              MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY);
    uint16_t requestLength = 0;
    const auto requestBytes = SerializeMavlinkMessage(request, requestLength);
    ASSERT_TRUE(peer.SendToLoopback(listenPort, requestBytes.data(), requestLength));
    for (int attempt = 0; attempt < 20 && gateway.PollRxOnce() == 0; ++attempt) {
        usleep(1000);
    }
    gateway.StepTx();

    mavlink_message_t response{};
    ASSERT_TRUE(ReceiveMavlinkMessageOfType(peer, MAVLINK_MSG_ID_TIMESYNC,
                                            response));
    mavlink_timesync_t timesync{};
    mavlink_msg_timesync_decode(&response, &timesync);
    EXPECT_EQ(timesync.ts1, PX4_TIMESTAMP_NS);
    EXPECT_GT(timesync.tc1, 0);
    EXPECT_EQ(timesync.target_system, 1);
    EXPECT_EQ(timesync.target_component, 1);
}
