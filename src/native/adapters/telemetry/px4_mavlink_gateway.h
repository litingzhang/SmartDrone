#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "adapters/telemetry/mavlink_serial_transport.h"
#include "common/mavlink.h"
#include "core/application/state/frame_timing_tracker.h"

enum class OdomQualityMode
{
    GOOD,
    WEAK,
    LOST
};

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

    struct OdomTimestampDebug {
        uint64_t frameId{0};
        uint64_t tCamNs{0};
        uint64_t tCbNs{0};
        uint64_t tSlamInNs{0};
        uint64_t tPx4Ns{0};
        uint64_t tSlamOutNs{0};
        uint64_t tMavTxNs{0};
        int64_t timesyncOffsetNs{0};
        uint32_t timesyncRttUs{0};
        bool timesyncValid{false};
    };

    static constexpr uint8_t PX4_CUSTOM_MAIN_MODE_MANUAL = 1;
    static constexpr uint8_t PX4_CUSTOM_MAIN_MODE_ALTCTL = 2;
    static constexpr uint8_t PX4_CUSTOM_MAIN_MODE_POSCTL = 3;
    static constexpr uint8_t PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;

    explicit Px4MavlinkGateway(
        const std::string& dev,
        int baud,
        uint8_t sysid = 42,
        uint8_t compid = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY);
    ~Px4MavlinkGateway();

    void SetFrameTimingTracker(smartdrone::core::application::FrameTimingTracker* tracker);
    void StartRx();
    void StopRx();
    uint8_t GetTargetSystem() const;
    uint8_t GetTargetComponent() const;
    bool GetLocalPositionNed(LocalPositionNed& out, uint64_t maxAgeUs = 500000) const;
    bool GetDownwardDistanceSensor(DownwardDistanceSensor& out, uint64_t maxAgeUs = 200000) const;
    bool WaitCommandAck(uint16_t command, int timeoutMs, uint8_t& outResult);
    bool GetFlightModeInfo(FlightModeInfo& out, uint64_t maxAgeUs = 1500000) const;
    void SendCommandLong(
        uint16_t command,
        float p1 = 0,
        float p2 = 0,
        float p3 = 0,
        float p4 = 0,
        float p5 = 0,
        float p6 = 0,
        float p7 = 0,
        uint8_t targetSystem = 1,
        uint8_t targetComponent = 1,
        uint8_t confirmation = 0);
    bool SendCommandLongAndWaitAck(
        uint16_t command,
        float p1 = 0,
        float p2 = 0,
        float p3 = 0,
        float p4 = 0,
        float p5 = 0,
        float p6 = 0,
        float p7 = 0,
        int timeoutMs = 800,
        uint8_t targetSystem = 0,
        uint8_t targetComponent = 0,
        uint8_t* outResult = nullptr);
    bool SetModePx4Main(uint8_t mainMode, int ackTimeoutMs = 800, uint8_t targetSystem = 0, uint8_t targetComponent = 0);
    bool SetModeOffboard(int ackTimeoutMs = 800, uint8_t targetSystem = 0, uint8_t targetComponent = 0);
    bool SetModePosition(int ackTimeoutMs = 800, uint8_t targetSystem = 0, uint8_t targetComponent = 0);
    bool SetModeAltitude(int ackTimeoutMs = 800, uint8_t targetSystem = 0, uint8_t targetComponent = 0);
    bool Arm(bool doArm, int ackTimeoutMs = 800, uint8_t targetSystem = 0, uint8_t targetComponent = 0);
    bool EmergencyStop(int ackTimeoutMs = 800, uint8_t targetSystem = 0, uint8_t targetComponent = 0);
    bool StartOffboardAndArm(
        double unused1,
        double unused2,
        int warmupMs = 800,
        int ackTimeoutMs = 1000,
        uint8_t targetSystem = 0,
        uint8_t targetComponent = 0);
    void SendSetPositionTargetLocalNed(
        uint32_t timeBootMs,
        const SetpointLocalNED& sp,
        uint8_t coordinateFrame = MAV_FRAME_LOCAL_NED);
    void StartSetpointStreamHz(double hz = 20.0);
    void StopSetpointStream();
    void UpdateStreamSetpoint(const SetpointLocalNED& spNed);
    void UpdateStreamPosition(float xN, float yE, float zD, float yawRad = NAN);
    void SendManualControl(
        const ManualControlInput& input,
        uint16_t buttons = 0,
        uint16_t buttons2 = 0,
        uint8_t targetSystem = 0);
    bool SendLand(int ackTimeoutMs = 800, uint8_t targetSystem = 1, uint8_t targetComponent = 1);
    bool GetTimesyncStatus(int64_t& offsetNs, uint32_t& rttUs, uint32_t& sampleCount) const;
    uint64_t MapMonotonicNsToPx4Ns(
        uint64_t monoNs,
        bool* outTimesyncValid = nullptr,
        int64_t* outOffsetNs = nullptr,
        uint32_t* outRttUs = nullptr,
        uint32_t* outSampleCount = nullptr) const;
    void SendOdometry(
        uint64_t odomFrameId,
        const Pose& poseNed,
        const LinearVelocityNed& velNed = LinearVelocityNed{},
        uint8_t mavFrameId = MAV_FRAME_LOCAL_NED,
        uint8_t childFrameId = MAV_FRAME_BODY_FRD,
        uint8_t resetCounter = 0,
        OdomQualityMode mode = OdomQualityMode::GOOD);
    static Pose EnuToNed(const Pose& pEnu);
    static void NormalizeQuat(float& w, float& x, float& y, float& z);

private:
    static constexpr uint32_t kTimesyncMaxAcceptableRttUs = 50000;
    static constexpr int64_t kTimesyncJumpResetThresholdNs = 50000000LL;
    static constexpr int64_t kTimesyncJumpWarnThresholdNs = 10000000LL;

    struct AckInfo {
        uint8_t result = 255;
        uint8_t progress = 0;
        int32_t resultParam2 = 0;
        std::chrono::steady_clock::time_point t;
    };

    void ResetTimesyncEstimateLocked();
    bool LookupFrameTiming(uint64_t frameId, smartdrone::core::application::FrameTimingRecord& out) const;
    void MarkFrameMavTx(uint64_t frameId, uint64_t tMavTxNs);
    static const char* MavResultToStr(uint8_t r);
    void WriteMessage(const mavlink_message_t& msg);
    void SendTimesyncMessage(int64_t tc1Ns, int64_t ts1Ns, uint8_t targetSystem, uint8_t targetComponent);
    void SendTimesyncRequest(uint8_t targetSystem, uint8_t targetComponent);
    void SendTimesyncResponse(int64_t requestTs1Ns, uint8_t targetSystem, uint8_t targetComponent);
    void SendTimesyncFollowUp(int64_t remoteStampNs, uint8_t targetSystem, uint8_t targetComponent);
    void TimesyncLoop();
    void RxLoop();
    void HandleMavlinkMessage(const mavlink_message_t& msg);
    void MaybeRequestLocalPositionNedStream(uint8_t targetSystem, uint8_t targetComponent);
    void MaybeRequestDistanceSensorStream(uint8_t targetSystem, uint8_t targetComponent);

    smartdrone::adapters::telemetry::MavlinkSerialTransport m_transport;
    uint8_t m_sysid;
    uint8_t m_compid;
    std::atomic<bool> m_streaming{false};
    std::thread m_streamThread;
    std::mutex m_spMtx;
    SetpointLocalNED m_spCurrent{};
    uint64_t m_streamPeriodUs{50000};
    std::atomic<bool> m_rxRunning{false};
    std::thread m_rxThread;
    std::atomic<bool> m_timesyncRunning{false};
    std::thread m_timesyncThread;
    std::mutex m_ackMtx;
    std::condition_variable m_ackCv;
    std::unordered_map<uint16_t, AckInfo> m_ackMap;
    std::mutex m_txMtx;
    mutable std::mutex m_timesyncMtx;
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
    bool m_havePendingTimesync{false};
    int64_t m_pendingTimesyncTs1Ns{0};
    uint64_t m_pendingTimesyncSentNs{0};
    uint8_t m_pendingTimesyncTargetSystem{0};
    uint8_t m_pendingTimesyncTargetComponent{0};
    int64_t m_timesyncEstimatedOffsetNs{0};
    uint32_t m_timesyncLastRttUs{0};
    uint32_t m_timesyncSampleCount{0};
    uint32_t m_timesyncInboundRequestCount{0};
    uint32_t m_timesyncInboundResponseCount{0};
    uint64_t m_lastTimesyncActivityUs{0};
    uint64_t m_lastTimesyncLogUs{0};
    OdomTimestampDebug m_lastOdomTimestampDebug{};
    smartdrone::core::application::FrameTimingTracker* m_frameTimingTracker{nullptr};
};
