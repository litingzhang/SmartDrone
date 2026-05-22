#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>

#include "adapters/telemetry/mavlink_serial_transport.h"
#include "common/mavlink.h"
#include "core/application/state/frame_timing_tracker.h"

enum class OdomQualityMode { GOOD,
                             WEAK,
                             LOST };

class Px4MavlinkGateway {
  public:
    struct Pose {
        float x, y, z;
        float qw, qx, qy, qz;
    };

    struct LinearVelocityNed {
        LinearVelocityNed(float xIn = NAN, float yIn = NAN, float zIn = NAN)
            : x(xIn), y(yIn), z(zIn)
        {
        }

        float x;
        float y;
        float z;
    };

    struct LocalPositionNed {
        float x{0.0f}, y{0.0f}, z{0.0f};
        float vx{0.0f}, vy{0.0f}, vz{0.0f};
        uint32_t timeBootMs{0};
        uint64_t receivedUs{0};
    };

    struct DownwardDistanceSensor {
        float currentDistance{NAN};
        float minDistance{NAN};
        float maxDistance{NAN};
        uint8_t signalQuality{0};
        uint64_t receivedUs{0};
    };

    struct ManualControlInput {
        float throttleNorm{0.0f};
        float yawNorm{0.0f};
        float pitchNorm{0.0f};
        float rollNorm{0.0f};
    };

    struct FlightModeInfo {
        uint8_t baseMode{0};
        uint32_t customMode{0};
        uint8_t mainMode{0};
        uint8_t subMode{0};
        bool armed{false};
        uint64_t receivedUs{0};
    };

    struct SetpointLocalNED {
        float x = NAN, y = NAN, z = NAN;
        float vx = NAN, vy = NAN, vz = NAN;
        float ax = NAN, ay = NAN, az = NAN;
        float yaw = NAN;
        float yawspeed = NAN;
    };

    struct CommandLongRequest {
        uint16_t command{0};
        std::array<float, 7> params{};
        uint8_t targetSystem{0};
        uint8_t targetComponent{0};
        uint8_t confirmation{0};
    };

    struct OdometryRequest {
        uint64_t frameId{0};
        Pose poseNed{};
        LinearVelocityNed velocityNed{};
        uint8_t mavFrameId{MAV_FRAME_LOCAL_NED};
        uint8_t childFrameId{MAV_FRAME_BODY_FRD};
        uint8_t resetCounter{0};
        OdomQualityMode qualityMode{OdomQualityMode::GOOD};
    };

    static constexpr uint8_t PX4_CUSTOM_MAIN_MODE_MANUAL = 1;
    static constexpr uint8_t PX4_CUSTOM_MAIN_MODE_POSCTL = 3;
    static constexpr uint8_t PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;

    explicit Px4MavlinkGateway(const std::string &dev, int baud, uint8_t sysid = 42,
                               uint8_t compid = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY);
    ~Px4MavlinkGateway();

    void SetJsonDiagnostics(bool enabled);
    void SetFrameTimingTracker(SmartDrone::Core::Application::FrameTimingTracker *tracker);
    int PollRxOnce(int timeoutMs = 0);
    void StepTx();
    uint8_t GetTargetSystem() const;
    uint8_t GetTargetComponent() const;
    bool GetLocalPositionNed(LocalPositionNed &out, uint64_t maxAgeUs = 500000) const;
    bool GetDownwardDistanceSensor(DownwardDistanceSensor &out, uint64_t maxAgeUs = 200000) const;
    bool TryConsumeCommandAck(uint16_t command, uint8_t &outResult);
    bool GetFlightModeInfo(FlightModeInfo &out, uint64_t maxAgeUs = 1500000) const;
    bool SendCommandLong(const CommandLongRequest &request);
    bool BeginCommandLong(const CommandLongRequest &request);
    bool BeginSetModePx4Main(uint8_t mainMode, uint8_t targetSystem = 0, uint8_t targetComponent = 0);
    bool BeginSetModeOffboard(uint8_t targetSystem = 0, uint8_t targetComponent = 0);
    bool BeginSetModePosition(uint8_t targetSystem = 0, uint8_t targetComponent = 0);
    bool BeginArm(bool doArm, uint8_t targetSystem = 0, uint8_t targetComponent = 0);
    bool BeginEmergencyStop(uint8_t targetSystem = 0, uint8_t targetComponent = 0);
    void SendSetPositionTargetLocalNed(uint32_t timeBootMs, const SetpointLocalNED &sp,
                                       uint8_t coordinateFrame = MAV_FRAME_LOCAL_NED);
    void StartSetpointStreamHz(double hz = 20.0);
    void StopSetpointStream();
    void StepSetpointStream();
    void UpdateStreamSetpoint(const SetpointLocalNED &spNed);
    void UpdateStreamPosition(float xN, float yE, float zD, float yawRad = NAN);
    void SendManualControl(const ManualControlInput &input, uint16_t buttons = 0, uint16_t buttons2 = 0,
                           uint8_t targetSystem = 0);
    bool BeginLand(uint8_t targetSystem = 1, uint8_t targetComponent = 1);
    void SendOdometry(const OdometryRequest &request);
    static Pose EnuToNed(const Pose &pEnu);
    static void NormalizeQuat(float &w, float &x, float &y, float &z);

  private:
    struct AckInfo {
        uint8_t result = 255;
        uint8_t progress = 0;
        int32_t resultParam2 = 0;
        std::chrono::steady_clock::time_point t;
    };

    struct OdometryPacketFields {
        float poseCov[21]{};
        float velocityCov[21]{};
        int8_t quality{100};
        uint8_t estimatorType{0};
    };

    struct OdometryTiming {
        uint64_t tCamNs{0};
        uint64_t tCbNs{0};
        uint64_t tSlamInNs{0};
        uint64_t tSlamOutNs{0};
        uint64_t tMavTxNs{0};
    };

    struct OdometryTimingLog {
        uint64_t frameId{0};
        uint8_t resetCounter{0};
        int8_t quality{0};
        bool haveTiming{false};
        OdometryTiming timing{};
    };

    bool LookupFrameTiming(uint64_t frameId, SmartDrone::Core::Application::FrameTimingRecord &out) const;
    void MarkFrameMavTx(uint64_t frameId, uint64_t tMavTxNs);
    static const char *MavResultToStr(uint8_t r);
    OdometryPacketFields BuildOdometryPacketFields(const OdometryRequest &request) const;
    bool PrepareOdometryTiming(uint64_t frameId, OdometryTiming &out);
    void PackOdometryMessage(const OdometryRequest &request, const OdometryPacketFields &fields,
                             uint64_t captureTimeUs, mavlink_message_t &msg) const;
    void LogOdometryTiming(const OdometryTimingLog &log) const;
    CommandLongRequest ResolveCommandTargets(CommandLongRequest request) const;
    bool SendMessageIntervalRequest(uint32_t messageId, float intervalUs, uint8_t targetSystem, uint8_t targetComponent);
    bool QueueMessage(const mavlink_message_t &msg);
    void HandleMavlinkMessage(const mavlink_message_t &msg);
    void HandleHeartbeat(const mavlink_message_t &msg);
    void HandleLocalPositionNed(const mavlink_message_t &msg);
    void HandleDistanceSensor(const mavlink_message_t &msg);
    void HandleCommandAck(const mavlink_message_t &msg);
    void HandleStatusText(const mavlink_message_t &msg);
    void MaybeRequestLocalPositionNedStream(uint8_t targetSystem, uint8_t targetComponent);
    void MaybeRequestDistanceSensorStream(uint8_t targetSystem, uint8_t targetComponent);

    SmartDrone::Adapters::Telemetry::MavlinkSerialTransport m_transport;
    uint8_t m_sysid;
    uint8_t m_compid;
    std::atomic<bool> m_streaming{false};
    std::mutex m_spMtx;
    SetpointLocalNED m_spCurrent{};
    std::atomic<uint64_t> m_streamPeriodUs{50000};
    std::atomic<uint64_t> m_lastStreamTxUs{0};
    mavlink_message_t m_rxMessage{};
    mavlink_status_t m_rxStatus{};
    std::mutex m_ackMtx;
    std::unordered_map<uint16_t, AckInfo> m_ackMap;
    std::mutex m_txMtx;
    std::deque<std::vector<uint8_t>> m_txQueue;
    std::size_t m_txOffset{0};
    mutable std::mutex m_frameTimingTrackerMtx;
    mutable std::mutex m_flightModeMtx;
    mutable std::mutex m_localPosMtx;
    mutable std::mutex m_distanceSensorMtx;
    std::atomic<uint8_t> m_px4Sysid{1};
    std::atomic<uint8_t> m_px4Compid{1};
    std::atomic<bool> m_havePx4Heartbeat{false};
    std::atomic<bool> m_localPosStreamRequested{false};
    std::atomic<bool> m_distanceSensorStreamRequested{false};
    FlightModeInfo m_flightModeInfo{};
    bool m_haveFlightModeInfo{false};
    LocalPositionNed m_localPosNed{};
    bool m_haveLocalPosNed{false};
    DownwardDistanceSensor m_downwardDistanceSensor{};
    bool m_haveDownwardDistanceSensor{false};
    uint64_t m_lastSentOdomFrameId{0};
    SmartDrone::Core::Application::FrameTimingTracker *m_frameTimingTracker{nullptr};
    std::atomic<bool> m_jsonDiagnostics{false};
};
