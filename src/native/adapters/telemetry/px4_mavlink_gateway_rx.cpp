#include "adapters/telemetry/px4_mavlink_gateway.h"

#include <cstring>
#include <iostream>

#include "common/time_utils.h"

void Px4MavlinkGateway::HandleMavlinkMessage(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_TIMESYNC) {
        HandleTimesync(msg);
        return;
    }
    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        HandleHeartbeat(msg);
        return;
    }
    if (!IsMessageFromTarget(msg)) {
        return;
    }
    if (msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
        HandleLocalPositionNed(msg);
    } else if (msg.msgid == MAVLINK_MSG_ID_DISTANCE_SENSOR) {
        HandleDistanceSensor(msg);
    } else if (msg.msgid == MAVLINK_MSG_ID_EXTENDED_SYS_STATE) {
        HandleExtendedSystemState(msg);
    } else if (msg.msgid == MAVLINK_MSG_ID_ESTIMATOR_STATUS) {
        HandleEstimatorStatus(msg);
    } else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        HandleCommandAck(msg);
    } else if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
        HandleStatusText(msg);
    }
}

void Px4MavlinkGateway::HandleTimesync(const mavlink_message_t &msg)
{
    mavlink_timesync_t request{};
    mavlink_msg_timesync_decode(&msg, &request);
    if (request.tc1 != 0) {
        return;
    }
    if (request.target_system != 0 && request.target_system != m_sysid) {
        return;
    }
    if (request.target_component != 0 && request.target_component != m_compid) {
        return;
    }
    mavlink_message_t response{};
    mavlink_msg_timesync_pack(
        m_sysid, m_compid, &response, static_cast<int64_t>(MeasurementNowNs()),
        request.ts1, msg.sysid, msg.compid);
    QueueMessage(response);
}

void Px4MavlinkGateway::HandleHeartbeat(const mavlink_message_t &msg)
{
    mavlink_heartbeat_t heartbeat{};
    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    if (heartbeat.autopilot != MAV_AUTOPILOT_PX4 ||
        !TryLockPx4Target(msg)) {
        return;
    }
    FlightModeInfo modeInfo{};
    modeInfo.baseMode = heartbeat.base_mode;
    modeInfo.customMode = heartbeat.custom_mode;
    modeInfo.mainMode = static_cast<uint8_t>((heartbeat.custom_mode >> 16) & 0xFFu);
    modeInfo.subMode = static_cast<uint8_t>((heartbeat.custom_mode >> 24) & 0xFFu);
    modeInfo.armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    modeInfo.receivedUs = MonoTimeUs();
    auto snapshot = std::make_shared<const FlightModeInfo>(modeInfo);
    std::atomic_store_explicit(&m_flightModeInfo, std::move(snapshot),
                               std::memory_order_release);
    MaybeRequestLocalPositionNedStream(msg.sysid, msg.compid);
    MaybeRequestDistanceSensorStream(msg.sysid, msg.compid);
    MaybeRequestExtendedSystemStateStream(msg.sysid, msg.compid);
    MaybeRequestEstimatorStatusStream(msg.sysid, msg.compid);
}

bool Px4MavlinkGateway::IsMessageFromTarget(
    const mavlink_message_t &msg) const
{
    return HavePx4Target() && msg.sysid == GetTargetSystem() &&
           msg.compid == GetTargetComponent();
}

bool Px4MavlinkGateway::TryLockPx4Target(const mavlink_message_t &msg)
{
    if (msg.sysid == 0 || msg.compid == 0) {
        return false;
    }
    const std::uint16_t candidate =
        static_cast<std::uint16_t>(msg.sysid) << 8U | msg.compid;
    std::uint16_t expected = 0;
    if (!m_px4Target.compare_exchange_strong(
            expected, candidate, std::memory_order_acq_rel,
            std::memory_order_acquire)) {
        return expected == candidate;
    }
    m_localPosStreamRequested.store(false, std::memory_order_release);
    m_distanceSensorStreamRequested.store(false, std::memory_order_release);
    m_extendedSystemStateStreamRequested.store(false, std::memory_order_release);
    m_estimatorStatusStreamRequested.store(false, std::memory_order_release);
    return true;
}

void Px4MavlinkGateway::HandleLocalPositionNed(const mavlink_message_t &msg)
{
    mavlink_local_position_ned_t position{};
    mavlink_msg_local_position_ned_decode(&msg, &position);
    LocalPositionNed sample{};
    sample.x = position.x;
    sample.y = position.y;
    sample.z = position.z;
    sample.vx = position.vx;
    sample.vy = position.vy;
    sample.vz = position.vz;
    sample.timeBootMs = position.time_boot_ms;
    sample.receivedUs = MonoTimeUs();
    auto snapshot = std::make_shared<const LocalPositionNed>(sample);
    std::atomic_store_explicit(&m_localPosNed, std::move(snapshot),
                               std::memory_order_release);
}

void Px4MavlinkGateway::HandleDistanceSensor(const mavlink_message_t &msg)
{
    mavlink_distance_sensor_t distance{};
    mavlink_msg_distance_sensor_decode(&msg, &distance);
    if (distance.orientation != MAV_SENSOR_ROTATION_PITCH_270) {
        return;
    }
    DownwardDistanceSensor sample{};
    sample.currentDistance = 0.01f * static_cast<float>(distance.current_distance);
    sample.minDistance = 0.01f * static_cast<float>(distance.min_distance);
    sample.maxDistance = 0.01f * static_cast<float>(distance.max_distance);
    sample.signalQuality = distance.signal_quality;
    sample.receivedUs = MonoTimeUs();
    auto snapshot = std::make_shared<const DownwardDistanceSensor>(sample);
    std::atomic_store_explicit(&m_downwardDistanceSensor, std::move(snapshot),
                               std::memory_order_release);
}

void Px4MavlinkGateway::HandleExtendedSystemState(const mavlink_message_t &msg)
{
    mavlink_extended_sys_state_t state{};
    mavlink_msg_extended_sys_state_decode(&msg, &state);
    ExtendedSystemState sample{};
    sample.vtolState = state.vtol_state;
    sample.landedState = state.landed_state;
    sample.receivedUs = MonoTimeUs();
    auto snapshot = std::make_shared<const ExtendedSystemState>(sample);
    std::atomic_store_explicit(&m_extendedSystemState, std::move(snapshot),
                               std::memory_order_release);
}

void Px4MavlinkGateway::HandleEstimatorStatus(const mavlink_message_t &msg)
{
    mavlink_estimator_status_t status{};
    mavlink_msg_estimator_status_decode(&msg, &status);
    EstimatorStatus sample{};
    sample.velocityRatio = status.vel_ratio;
    sample.horizontalPositionRatio = status.pos_horiz_ratio;
    sample.verticalPositionRatio = status.pos_vert_ratio;
    sample.magneticRatio = status.mag_ratio;
    sample.heightAboveGroundRatio = status.hagl_ratio;
    sample.trueAirspeedRatio = status.tas_ratio;
    sample.flags = status.flags;
    sample.receivedUs = MonoTimeUs();
    auto snapshot = std::make_shared<const EstimatorStatus>(sample);
    std::atomic_store_explicit(&m_estimatorStatus, std::move(snapshot),
                               std::memory_order_release);
}

void Px4MavlinkGateway::HandleCommandAck(const mavlink_message_t &msg)
{
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&msg, &ack);
    AckInfo info{};
    info.command = ack.command;
    info.result = ack.result;
    info.progress = ack.progress;
    info.resultParam2 = ack.result_param2;
    info.t = std::chrono::steady_clock::now();
    StoreCommandAck(info);
    std::cerr << "[ACK] sys=" << int(msg.sysid) << ", comp=" << int(msg.compid)
              << " cmd=" << ack.command << " result=" << int(ack.result)
              << "(" << MavResultToStr(ack.result) << ") progress="
              << int(ack.progress) << " param2=" << ack.result_param2 << "\n";
}

void Px4MavlinkGateway::HandleStatusText(const mavlink_message_t &msg)
{
    mavlink_statustext_t statusText{};
    mavlink_msg_statustext_decode(&msg, &statusText);
    char text[sizeof(statusText.text) + 1];
    std::memcpy(text, statusText.text, sizeof(statusText.text));
    text[sizeof(statusText.text)] = '\0';
    std::cerr << "[STATUSTEXT] sev=" << int(statusText.severity) << " " << text
              << "\n";
}

void Px4MavlinkGateway::MaybeRequestLocalPositionNedStream(
    uint8_t targetSystem, uint8_t targetComponent)
{
    if (m_localPosStreamRequested.load(std::memory_order_acquire) ||
        !SendMessageIntervalRequest(MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                                    50000.0f, targetSystem,
                                    targetComponent)) {
        return;
    }
    m_localPosStreamRequested.store(true, std::memory_order_release);
    std::cerr << "[mav] requested LOCAL_POSITION_NED @20Hz from sys="
              << int(targetSystem) << " comp=" << int(targetComponent) << "\n";
}

void Px4MavlinkGateway::MaybeRequestDistanceSensorStream(
    uint8_t targetSystem, uint8_t targetComponent)
{
    if (m_distanceSensorStreamRequested.load(std::memory_order_acquire) ||
        !SendMessageIntervalRequest(MAVLINK_MSG_ID_DISTANCE_SENSOR, 50000.0f,
                                    targetSystem, targetComponent)) {
        return;
    }
    m_distanceSensorStreamRequested.store(true, std::memory_order_release);
    std::cerr << "[mav] requested DISTANCE_SENSOR @20Hz from sys="
              << int(targetSystem) << " comp=" << int(targetComponent) << "\n";
}

void Px4MavlinkGateway::MaybeRequestExtendedSystemStateStream(
    uint8_t targetSystem, uint8_t targetComponent)
{
    if (m_extendedSystemStateStreamRequested.load(std::memory_order_acquire) ||
        !SendMessageIntervalRequest(MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
                                    200000.0f, targetSystem,
                                    targetComponent)) {
        return;
    }
    m_extendedSystemStateStreamRequested.store(true,
                                               std::memory_order_release);
    std::cerr << "[mav] requested EXTENDED_SYS_STATE @5Hz from sys="
              << int(targetSystem) << " comp=" << int(targetComponent) << "\n";
}

void Px4MavlinkGateway::MaybeRequestEstimatorStatusStream(
    uint8_t targetSystem, uint8_t targetComponent)
{
    if (m_estimatorStatusStreamRequested.load(std::memory_order_acquire) ||
        !SendMessageIntervalRequest(MAVLINK_MSG_ID_ESTIMATOR_STATUS,
                                    200000.0f, targetSystem,
                                    targetComponent)) {
        return;
    }
    m_estimatorStatusStreamRequested.store(true, std::memory_order_release);
    std::cerr << "[mav] requested ESTIMATOR_STATUS @5Hz from sys="
              << int(targetSystem) << " comp=" << int(targetComponent) << "\n";
}
