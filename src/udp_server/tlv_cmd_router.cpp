#include "tlv_cmd_router.hpp"

#include <cstring>

namespace {

bool IsMonotonicSeq(uint32_t value, uint32_t& lastValue)
{
    if (value == 0 || value <= lastValue) {
        return false;
    }
    lastValue = value;
    return true;
}

}  // namespace

TlvCmdRouter::TlvCmdRouter(MavlinkHooks& hooks) : m_hooks(hooks) {}

float TlvCmdRouter::ReadF32Le(const uint8_t* p)
{
    const uint32_t raw = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
    float out = 0.0f;
    std::memcpy(&out, &raw, sizeof(out));
    return out;
}

uint32_t TlvCmdRouter::ReadU32Le(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

void TlvCmdRouter::RegisterDefaults()
{
    m_handlers[CMD_PING] = [](const TlvFrame&) -> RouteResult { return {ACK_OK, "pong"}; };

    m_handlers[CMD_ARM] = [this](const TlvFrame&) { return HandleSimple(CMD_ARM); };
    m_handlers[CMD_DISARM] = [this](const TlvFrame&) { return HandleSimple(CMD_DISARM); };
    m_handlers[CMD_OFFBOARD] = [this](const TlvFrame&) { return HandleSimple(CMD_OFFBOARD); };
    m_handlers[CMD_HOLD] = [this](const TlvFrame&) { return HandleSimple(CMD_HOLD); };
    m_handlers[CMD_LAND] = [this](const TlvFrame&) { return HandleSimple(CMD_LAND); };
    m_handlers[CMD_MOVE] = [this](const TlvFrame& frame) { return HandleMove(frame); };
}

RouteResult TlvCmdRouter::Handle(const TlvFrame& frame)
{
    if (frame.ver != TLV_VER) {
        return {ACK_E_BAD_ARGS, "bad version"};
    }
    if (!IsMonotonicSeq(frame.seq, m_lastSeq)) {
        return {ACK_E_BAD_STATE, "seq not monotonic"};
    }

    const auto it = m_handlers.find(frame.cmd);
    if (it == m_handlers.end()) {
        return {ACK_E_UNKNOWN, "unknown cmd"};
    }
    return it->second(frame);
}

RouteResult TlvCmdRouter::HandleSimple(uint8_t cmd)
{
    const VehicleGate gate = m_hooks.GetGate();
    std::string err;

    switch (cmd) {
        case CMD_ARM:
            if (!gate.vioOk) {
                return {ACK_E_BAD_STATE, "vio not ok"};
            }
            if (!m_hooks.Arm(&err)) {
                return {ACK_E_INTERNAL, err.empty() ? "arm failed" : err};
            }
            return {ACK_OK, ""};

        case CMD_DISARM:
            if (!m_hooks.Disarm(&err)) {
                return {ACK_E_INTERNAL, err.empty() ? "disarm failed" : err};
            }
            return {ACK_OK, ""};

        case CMD_OFFBOARD:
            if (!gate.vioOk) {
                return {ACK_E_BAD_STATE, "vio not ok"};
            }
            if (!gate.offboardReady) {
                return {ACK_E_BAD_STATE, "offboard not ready"};
            }
            if (!m_hooks.SetOffboard(&err)) {
                return {ACK_E_INTERNAL, err.empty() ? "offboard failed" : err};
            }
            return {ACK_OK, ""};

        case CMD_HOLD:
            if (!m_hooks.Hold(&err)) {
                return {ACK_E_INTERNAL, err.empty() ? "hold failed" : err};
            }
            return {ACK_OK, ""};

        case CMD_LAND:
            if (!m_hooks.Land(&err)) {
                return {ACK_E_INTERNAL, err.empty() ? "land failed" : err};
            }
            return {ACK_OK, ""};

        default:
            return {ACK_E_UNKNOWN, "unknown simple cmd"};
    }
}

RouteResult TlvCmdRouter::HandleMove(const TlvFrame& frame)
{
    if (frame.len != MOVE_PAYLOAD_LEN) {
        return {ACK_E_BAD_LEN, "bad move len"};
    }

    const VehicleGate gate = m_hooks.GetGate();
    if (!gate.vioOk) {
        return {ACK_E_BAD_STATE, "vio not ok"};
    }
    if (!gate.offboardReady) {
        return {ACK_E_BAD_STATE, "offboard not ready"};
    }
    if (!IsMonotonicSeq(frame.seq, m_lastMoveSeq)) {
        return {ACK_E_BAD_STATE, "old move dropped"};
    }

    const uint8_t* payload = frame.payload.data();
    MoveGoal goal;
    goal.frame = payload[0];
    const float valueA = ReadF32Le(&payload[1]);
    const float valueB = ReadF32Le(&payload[5]);
    const float valueC = ReadF32Le(&payload[9]);
    const float valueD = ReadF32Le(&payload[13]);
    goal.maxV = ReadF32Le(&payload[17]);
    goal.seq = frame.seq;
    goal.isVelocity = (frame.flags & MOVE_FLAG_VELOCITY) != 0;

    if (goal.isVelocity) {
        goal.vx = valueA;
        goal.vy = valueB;
        goal.vz = valueC;
        goal.yawRate = valueD;
    } else {
        goal.x = valueA;
        goal.y = valueB;
        goal.z = valueC;
        goal.yaw = valueD;
    }

    if (!(goal.maxV > 0.0f)) {
        return {ACK_E_BAD_ARGS, "bad maxV"};
    }
    // Clamp speed parameter from mobile joystick to a sane range.
    if (goal.maxV > 5.0f) {
        goal.maxV = 5.0f;
    }

    std::string err;
    if (!m_hooks.SetMoveGoal(goal, &err)) {
        return {ACK_E_INTERNAL, err.empty() ? "move failed" : err};
    }
    if (goal.isVelocity) {
        return {ACK_OK,
                "move(vel) accepted vx=" + std::to_string(goal.vx) +
                    " vy=" + std::to_string(goal.vy) +
                    " vz=" + std::to_string(goal.vz) +
                    " yawRate=" + std::to_string(goal.yawRate) +
                    " maxV=" + std::to_string(goal.maxV)};
    }
    return {ACK_OK,
            "move(pos) accepted x=" + std::to_string(goal.x) +
                " y=" + std::to_string(goal.y) +
                " z=" + std::to_string(goal.z) +
                " yaw=" + std::to_string(goal.yaw) +
                " maxV=" + std::to_string(goal.maxV)};
}
