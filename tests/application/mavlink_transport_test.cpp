#include "adapters/telemetry/mavlink_serial_transport.h"
#include "adapters/telemetry/mavlink_pose_publisher.h"
#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "adapters/telemetry/px4_vehicle_control_port.h"
#include "common/mavlink.h"
#include "common/time_utils.h"
#include "core/application/state/frame_timing_tracker.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

class FixedMeasurementClock final
    : public SmartDrone::Core::Ports::IMeasurementClock {
  public:
    explicit FixedMeasurementClock(uint64_t nowNs)
        : m_nowNs(nowNs)
    {
    }

    uint64_t NowNs() const override
    {
        return m_nowNs;
    }

    uint32_t ResetCounter() const override
    {
        return 0;
    }

    bool Valid() const override
    {
        return true;
    }

  private:
    uint64_t m_nowNs{0};
};

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
    for (int attempt = 0; attempt < 16; ++attempt) {
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

bool DeliverMavlinkMessage(UdpSocket &peer, uint16_t listenPort,
                           Px4MavlinkGateway &gateway,
                           const mavlink_message_t &message)
{
    uint16_t length = 0;
    const auto bytes = SerializeMavlinkMessage(message, length);
    if (!peer.SendToLoopback(listenPort, bytes.data(), length)) {
        return false;
    }
    for (int attempt = 0; attempt < 20; ++attempt) {
        if (gateway.PollRxOnce() > 0) {
            return true;
        }
        usleep(1000);
    }
    return false;
}

mavlink_message_t MakeHeartbeat(uint8_t systemId, uint8_t componentId,
                                uint8_t autopilot)
{
    mavlink_message_t heartbeat{};
    mavlink_msg_heartbeat_pack(
        systemId, componentId, &heartbeat, MAV_TYPE_QUADROTOR, autopilot,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_STANDBY);
    return heartbeat;
}

bool ReceiveControlMessage(const UdpSocket &socket)
{
    mavlink_message_t message{};
    for (int attempt = 0; attempt < 8; ++attempt) {
        if (!ReceiveMavlinkMessage(socket, message)) {
            return false;
        }
        if (message.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED ||
            message.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL) {
            return true;
        }
    }
    return false;
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
    constexpr uint64_t MEASUREMENT_TIMESTAMP_NS = 123456789000ULL;
    const uint64_t monotonicNs = MonoTimeUs() * 1000ULL;
    timing.UpsertCapture(7, MEASUREMENT_TIMESTAMP_NS, monotonicNs,
                         monotonicNs);

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
    EXPECT_EQ(odometry.time_usec, MEASUREMENT_TIMESTAMP_NS / 1000ULL);
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
    const uint64_t nowNs = MonoTimeUs() * 1000ULL;
    timing.UpsertCapture(8, nowNs, nowNs, nowNs);

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

TEST(MavlinkTransportTest, OdometryDiagnosticsReportCompleteStageTiming)
{
    UdpSocket receiver;
    ASSERT_TRUE(receiver.BindLoopback());
    Px4MavlinkGateway gateway(BuildUdpEndpoint(receiver.Port()), 921600);
    SmartDrone::Core::Application::FrameTimingTracker timing;
    gateway.SetFrameTimingTracker(&timing);
    gateway.SetMeasurementClock(
        std::make_shared<FixedMeasurementClock>(1125000000ULL));
    gateway.SetJsonDiagnostics(true);
    const uint64_t baseNs = MonoTimeUs() * 1000ULL;
    timing.UpsertCapture(
        55, SmartDrone::Core::Ports::FrameCaptureTiming{
                1000000000ULL, baseNs - 10000000ULL,
                baseNs - 8000000ULL, baseNs - 7000000ULL});
    timing.MarkSlamIn(55, baseNs - 6000000ULL);
    timing.MarkSlamOut(55, baseNs - 2000000ULL);
    Px4MavlinkGateway::OdometryRequest request{};
    request.frameId = 55;
    request.measurementTimestampNs = 1000000000ULL;
    request.poseNed.qw = 1.0F;

    testing::internal::CaptureStderr();
    gateway.SendOdometry(request);
    const std::string diagnostics = testing::internal::GetCapturedStderr();

    EXPECT_NE(diagnostics.find("\"timing\":1"), std::string::npos);
    EXPECT_NE(diagnostics.find("\"eye_skew_ms\":1.000"),
              std::string::npos);
    EXPECT_NE(diagnostics.find("\"render_transport_ms\":3.000"),
              std::string::npos);
    EXPECT_NE(diagnostics.find("\"queue_ms\":1.000"),
              std::string::npos);
    EXPECT_NE(diagnostics.find("\"processing_ms\":4.000"),
              std::string::npos);
    EXPECT_NE(diagnostics.find("\"pair_to_tx_ms\":"),
              std::string::npos);
    EXPECT_NE(diagnostics.find("\"wall_total_ms\":"),
              std::string::npos);
    EXPECT_NE(diagnostics.find("\"total_ms\":"), std::string::npos);
    EXPECT_NE(diagnostics.find("\"sim_age_ms\":125.000"),
              std::string::npos);
}

TEST(MavlinkTransportTest, OdometryDiagnosticsRejectIncompleteStageOrder)
{
    UdpSocket receiver;
    ASSERT_TRUE(receiver.BindLoopback());
    Px4MavlinkGateway gateway(BuildUdpEndpoint(receiver.Port()), 921600);
    SmartDrone::Core::Application::FrameTimingTracker timing;
    gateway.SetFrameTimingTracker(&timing);
    gateway.SetJsonDiagnostics(true);
    const uint64_t baseNs = MonoTimeUs() * 1000ULL;
    timing.UpsertCapture(
        56, SmartDrone::Core::Ports::FrameCaptureTiming{
                1000000000ULL, baseNs - 10000000ULL,
                baseNs - 8000000ULL, baseNs - 7000000ULL});
    timing.MarkSlamIn(56, baseNs - 9000000ULL);
    timing.MarkSlamOut(56, baseNs - 2000000ULL);
    Px4MavlinkGateway::OdometryRequest request{};
    request.frameId = 56;
    request.measurementTimestampNs = 1000000000ULL;
    request.poseNed.qw = 1.0F;

    testing::internal::CaptureStderr();
    gateway.SendOdometry(request);
    const std::string diagnostics = testing::internal::GetCapturedStderr();

    EXPECT_NE(diagnostics.find("\"timing\":0"), std::string::npos);
}

TEST(MavlinkTransportTest, OdometryDiagnosticsRejectMissingFrameRecord)
{
    UdpSocket receiver;
    ASSERT_TRUE(receiver.BindLoopback());
    Px4MavlinkGateway gateway(BuildUdpEndpoint(receiver.Port()), 921600);
    gateway.SetJsonDiagnostics(true);
    Px4MavlinkGateway::OdometryRequest request{};
    request.frameId = 57;
    request.measurementTimestampNs = 1000000000ULL;
    request.captureMonotonicNs = MonoTimeUs() * 1000ULL;
    request.poseNed.qw = 1.0F;

    testing::internal::CaptureStderr();
    gateway.SendOdometry(request);
    const std::string diagnostics = testing::internal::GetCapturedStderr();

    EXPECT_NE(diagnostics.find("\"timing\":0"), std::string::npos);
}

TEST(MavlinkTransportTest, DirectPoseTimestampSupportsTruthSource)
{
    UdpSocket receiver;
    ASSERT_TRUE(receiver.BindLoopback());
    Px4MavlinkGateway gateway(BuildUdpEndpoint(receiver.Port()), 921600);
    SmartDrone::Adapters::Telemetry::MavlinkPosePublisher publisher(gateway);
    SmartDrone::Core::Ports::PosePublishRequest request{};
    request.frameId = 91;
    request.measurementTimestampNs = 456789000ULL;
    request.captureMonotonicNs = MonoTimeUs() * 1000ULL;
    request.pose = {true, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
    request.referenceFrame =
        SmartDrone::Core::Ports::PoseReferenceFrame::LocalNed;
    request.quality = SmartDrone::Core::Ports::PoseQuality::Good;

    publisher.PublishPose(request);
    gateway.StepTx();

    mavlink_message_t message{};
    ASSERT_TRUE(ReceiveMavlinkMessage(receiver, message));
    mavlink_odometry_t odometry{};
    mavlink_msg_odometry_decode(&message, &odometry);
    EXPECT_EQ(odometry.time_usec, 456789ULL);
    EXPECT_EQ(odometry.quality, 100);
}

TEST(MavlinkTransportTest, TimesyncRequestReceivesCompanionTimestamp)
{
    const uint16_t listenPort = FindAvailableUdpPort();
    ASSERT_NE(listenPort, 0);
    Px4MavlinkGateway gateway("udp-listen://" + std::to_string(listenPort), 921600);
    constexpr uint64_t COMPANION_TIMESTAMP_NS = 987654321ULL;
    gateway.SetMeasurementClock(
        std::make_shared<FixedMeasurementClock>(COMPANION_TIMESTAMP_NS));
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
    EXPECT_EQ(timesync.tc1, static_cast<int64_t>(COMPANION_TIMESTAMP_NS));
    EXPECT_EQ(timesync.target_system, 1);
    EXPECT_EQ(timesync.target_component, 1);
}

TEST(MavlinkTransportTest, LearnedTargetIsUsedForSetpointAndLand)
{
    const uint16_t listenPort = FindAvailableUdpPort();
    ASSERT_NE(listenPort, 0);
    Px4MavlinkGateway gateway("udp-listen://" + std::to_string(listenPort),
                              921600);
    UdpSocket peer;
    ASSERT_TRUE(peer.BindLoopback());

    mavlink_message_t heartbeat{};
    mavlink_msg_heartbeat_pack(7, 42, &heartbeat, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_PX4, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                               0, MAV_STATE_STANDBY);
    uint16_t heartbeatLength = 0;
    const auto heartbeatBytes =
        SerializeMavlinkMessage(heartbeat, heartbeatLength);
    ASSERT_TRUE(peer.SendToLoopback(listenPort, heartbeatBytes.data(),
                                    heartbeatLength));
    for (int attempt = 0; attempt < 20 && gateway.PollRxOnce() == 0;
         ++attempt) {
        usleep(1000);
    }
    EXPECT_EQ(gateway.GetTargetSystem(), 7);
    EXPECT_EQ(gateway.GetTargetComponent(), 42);

    gateway.UpdateStreamPosition(1.0f, 2.0f, -1.5f);
    gateway.StartSetpointStreamHz(20.0);
    gateway.StepSetpointStream();
    gateway.StepTx();
    mavlink_message_t setpointMessage{};
    ASSERT_TRUE(ReceiveMavlinkMessageOfType(
        peer, MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED, setpointMessage));
    mavlink_set_position_target_local_ned_t setpoint{};
    mavlink_msg_set_position_target_local_ned_decode(&setpointMessage,
                                                     &setpoint);
    EXPECT_EQ(setpoint.target_system, 7);
    EXPECT_EQ(setpoint.target_component, 42);

    ASSERT_TRUE(gateway.BeginLand());
    gateway.StepTx();
    mavlink_message_t landMessage{};
    ASSERT_TRUE(ReceiveMavlinkMessageOfType(peer, MAVLINK_MSG_ID_COMMAND_LONG,
                                            landMessage));
    mavlink_command_long_t command{};
    mavlink_msg_command_long_decode(&landMessage, &command);
    EXPECT_EQ(command.command, MAV_CMD_NAV_LAND);
    EXPECT_EQ(command.target_system, 7);
    EXPECT_EQ(command.target_component, 42);
}

TEST(MavlinkTransportTest, ImplicitControlWaitsForFirstPx4Heartbeat)
{
    UdpSocket peer;
    ASSERT_TRUE(peer.BindLoopback());
    Px4MavlinkGateway gateway(BuildUdpEndpoint(peer.Port()), 921600);
    EXPECT_EQ(gateway.GetTargetSystem(), 0);
    EXPECT_EQ(gateway.GetTargetComponent(), 0);
    EXPECT_FALSE(gateway.BeginLand());

    gateway.UpdateStreamPosition(1.0f, 2.0f, -1.5f);
    gateway.StartSetpointStreamHz(20.0);
    gateway.StepSetpointStream();
    gateway.SendManualControl({});
    gateway.StepTx();

    EXPECT_FALSE(ReceiveControlMessage(peer));
}

TEST(MavlinkTransportTest, ExplicitCommandWorksBeforeTargetDiscovery)
{
    UdpSocket peer;
    ASSERT_TRUE(peer.BindLoopback());
    Px4MavlinkGateway gateway(BuildUdpEndpoint(peer.Port()), 921600);

    ASSERT_TRUE(gateway.BeginLand(7, 42));
    gateway.StepTx();

    mavlink_message_t message{};
    ASSERT_TRUE(ReceiveMavlinkMessageOfType(
        peer, MAVLINK_MSG_ID_COMMAND_LONG, message));
    mavlink_command_long_t command{};
    mavlink_msg_command_long_decode(&message, &command);
    EXPECT_EQ(command.command, MAV_CMD_NAV_LAND);
    EXPECT_EQ(command.target_system, 7);
    EXPECT_EQ(command.target_component, 42);
}

TEST(MavlinkTransportTest, LocksFirstPx4AndFiltersForeignStateAndAck)
{
    const uint16_t listenPort = FindAvailableUdpPort();
    ASSERT_NE(listenPort, 0);
    Px4MavlinkGateway gateway("udp-listen://" + std::to_string(listenPort),
                              921600);
    UdpSocket peer;
    ASSERT_TRUE(peer.BindLoopback());

    ASSERT_TRUE(DeliverMavlinkMessage(
        peer, listenPort, gateway,
        MakeHeartbeat(200, 190, MAV_AUTOPILOT_INVALID)));
    EXPECT_EQ(gateway.GetTargetSystem(), 0);
    ASSERT_TRUE(DeliverMavlinkMessage(
        peer, listenPort, gateway, MakeHeartbeat(7, 42, MAV_AUTOPILOT_PX4)));
    ASSERT_TRUE(DeliverMavlinkMessage(
        peer, listenPort, gateway, MakeHeartbeat(8, 43, MAV_AUTOPILOT_PX4)));
    EXPECT_EQ(gateway.GetTargetSystem(), 7);
    EXPECT_EQ(gateway.GetTargetComponent(), 42);

    mavlink_message_t foreignState{};
    mavlink_msg_extended_sys_state_pack(
        8, 43, &foreignState, MAV_VTOL_STATE_MC, MAV_LANDED_STATE_IN_AIR);
    ASSERT_TRUE(DeliverMavlinkMessage(peer, listenPort, gateway, foreignState));
    Px4MavlinkGateway::ExtendedSystemState state{};
    EXPECT_FALSE(gateway.GetExtendedSystemState(state));

    mavlink_message_t foreignAck{};
    mavlink_msg_command_ack_pack(8, 43, &foreignAck, MAV_CMD_NAV_LAND,
                                 MAV_RESULT_ACCEPTED, 100, 0, 42,
                                 MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY);
    ASSERT_TRUE(DeliverMavlinkMessage(peer, listenPort, gateway, foreignAck));
    uint8_t result = MAV_RESULT_FAILED;
    EXPECT_FALSE(gateway.TryConsumeCommandAck(MAV_CMD_NAV_LAND, result));

    mavlink_message_t targetAck{};
    mavlink_msg_command_ack_pack(7, 42, &targetAck, MAV_CMD_NAV_LAND,
                                 MAV_RESULT_ACCEPTED, 100, 0, 42,
                                 MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY);
    ASSERT_TRUE(DeliverMavlinkMessage(peer, listenPort, gateway, targetAck));
    ASSERT_TRUE(gateway.TryConsumeCommandAck(MAV_CMD_NAV_LAND, result));
    EXPECT_EQ(result, MAV_RESULT_ACCEPTED);
}

TEST(MavlinkTransportTest, StoresExtendedAndEstimatorStatus)
{
    const uint16_t listenPort = FindAvailableUdpPort();
    ASSERT_NE(listenPort, 0);
    Px4MavlinkGateway gateway("udp-listen://" + std::to_string(listenPort),
                              921600);
    UdpSocket peer;
    ASSERT_TRUE(peer.BindLoopback());
    ASSERT_TRUE(DeliverMavlinkMessage(
        peer, listenPort, gateway, MakeHeartbeat(1, 1, MAV_AUTOPILOT_PX4)));

    mavlink_message_t extended{};
    mavlink_msg_extended_sys_state_pack(
        1, 1, &extended, MAV_VTOL_STATE_MC, MAV_LANDED_STATE_IN_AIR);
    uint16_t extendedLength = 0;
    const auto extendedBytes =
        SerializeMavlinkMessage(extended, extendedLength);
    ASSERT_TRUE(peer.SendToLoopback(listenPort, extendedBytes.data(),
                                    extendedLength));
    for (int attempt = 0; attempt < 20 && gateway.PollRxOnce() == 0;
         ++attempt) {
        usleep(1000);
    }
    Px4MavlinkGateway::ExtendedSystemState extendedState{};
    ASSERT_TRUE(gateway.GetExtendedSystemState(extendedState));
    EXPECT_EQ(extendedState.landedState, MAV_LANDED_STATE_IN_AIR);

    mavlink_message_t estimator{};
    mavlink_msg_estimator_status_pack(1, 1, &estimator, 1234,
                                      ESTIMATOR_ATTITUDE | ESTIMATOR_POS_HORIZ_ABS,
                                      0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f,
                                      0.7f, 0.8f);
    uint16_t estimatorLength = 0;
    const auto estimatorBytes =
        SerializeMavlinkMessage(estimator, estimatorLength);
    ASSERT_TRUE(peer.SendToLoopback(listenPort, estimatorBytes.data(),
                                    estimatorLength));
    for (int attempt = 0; attempt < 20 && gateway.PollRxOnce() == 0;
         ++attempt) {
        usleep(1000);
    }
    Px4MavlinkGateway::EstimatorStatus estimatorState{};
    ASSERT_TRUE(gateway.GetEstimatorStatus(estimatorState));
    EXPECT_EQ(estimatorState.flags,
              ESTIMATOR_ATTITUDE | ESTIMATOR_POS_HORIZ_ABS);
    EXPECT_FLOAT_EQ(estimatorState.horizontalPositionRatio, 0.2f);
    EXPECT_FLOAT_EQ(estimatorState.verticalPositionRatio, 0.3f);
}

TEST(MavlinkTransportTest, VehicleControlUsesConfiguredFlightModeFreshness)
{
    const uint16_t listenPort = FindAvailableUdpPort();
    ASSERT_NE(listenPort, 0);
    Px4MavlinkGateway gateway("udp-listen://" + std::to_string(listenPort),
                              921600);
    UdpSocket peer;
    ASSERT_TRUE(peer.BindLoopback());
    ASSERT_TRUE(DeliverMavlinkMessage(
        peer, listenPort, gateway, MakeHeartbeat(1, 1, MAV_AUTOPILOT_PX4)));

    using SmartDrone::Adapters::Telemetry::Px4VehicleControlPort;
    using SmartDrone::Adapters::Telemetry::Px4VehicleControlPortConfig;
    Px4VehicleControlPort defaultPort(gateway);
    Px4VehicleControlPortConfig scaledConfig{};
    scaledConfig.flightModeMaxAgeUs = 100000ULL;
    Px4VehicleControlPortConfig shortConfig{};
    shortConfig.flightModeMaxAgeUs = 20000ULL;
    Px4VehicleControlPort scaledPort(gateway, scaledConfig);
    Px4VehicleControlPort shortPort(gateway, shortConfig);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    SmartDrone::Core::Ports::VehicleFlightMode mode{};
    EXPECT_TRUE(defaultPort.GetFlightModeInfo(mode));
    EXPECT_TRUE(scaledPort.GetFlightModeInfo(mode));
    EXPECT_FALSE(shortPort.GetFlightModeInfo(mode));
}
