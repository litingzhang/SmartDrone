#include "tlv_cmd_router.h"

#include <cmath>

bool IsFinite(float value)
{
    return std::isfinite(value);
}

TlvCmdRouter::TlvCmdRouter(RuntimeCommandHook &commandHook)
    : m_commandHook(commandHook)
{
}

void TlvCmdRouter::RegisterDefaults()
{
    m_handlers[CMD_PING] = [](const TlvFrame &) -> RouteResult { return {ACK_OK, "pong"}; };

    m_handlers[CMD_ARM] = [this](const TlvFrame &) { return HandleSimple(CMD_ARM); };
    m_handlers[CMD_DISARM] = [this](const TlvFrame &) { return HandleSimple(CMD_DISARM); };
    m_handlers[CMD_EMERGENCY_STOP] = [this](const TlvFrame &) { return HandleSimple(CMD_EMERGENCY_STOP); };
    m_handlers[CMD_OFFBOARD] = [this](const TlvFrame &) { return HandleSimple(CMD_OFFBOARD); };
    m_handlers[CMD_HOLD] = [this](const TlvFrame &) { return HandleSimple(CMD_HOLD); };
    m_handlers[CMD_LAND] = [this](const TlvFrame &) { return HandleSimple(CMD_LAND); };
    m_handlers[CMD_POSITION] = [this](const TlvFrame &) { return HandleSimple(CMD_POSITION); };
    m_handlers[CMD_MOVE] = [this](const TlvFrame &frame) { return HandleMove(frame); };
}

RouteResult TlvCmdRouter::Handle(const TlvFrame &frame)
{
    if (frame.ver != TLV_VER) {
        return {ACK_E_BAD_ARGS, "bad version"};
    }

    const auto it = m_handlers.find(frame.cmd);
    if (it == m_handlers.end()) {
        return {ACK_E_UNKNOWN, "unknown cmd"};
    }
    return it->second(frame);
}

RouteResult TlvCmdRouter::ExecuteSimple(SimpleCommandCall call,
                                        const char *fallbackError)
{
    std::string errorMessage;
    if (!(m_commandHook.*call)(&errorMessage)) {
        return {ACK_E_INTERNAL, errorMessage.empty() ? fallbackError : errorMessage};
    }
    return {ACK_OK, ""};
}

RouteResult TlvCmdRouter::HandleSimple(uint8_t cmd)
{
    switch (cmd) {
    case CMD_ARM:
        return ExecuteSimple(&RuntimeCommandHook::ArmVehicle, "arm failed");
    case CMD_DISARM:
        return ExecuteSimple(&RuntimeCommandHook::DisarmVehicle, "disarm failed");
    case CMD_EMERGENCY_STOP:
        return ExecuteSimple(&RuntimeCommandHook::StopVehicleImmediately, "emergency stop failed");
    case CMD_OFFBOARD:
        return ExecuteSimple(&RuntimeCommandHook::EnterGuidedControl, "remote mode failed");
    case CMD_HOLD:
        return ExecuteSimple(&RuntimeCommandHook::HoldVehicle, "hold failed");
    case CMD_LAND:
        return ExecuteSimple(&RuntimeCommandHook::LandVehicle, "land failed");
    case CMD_POSITION:
        return ExecuteSimple(&RuntimeCommandHook::EnterPositionControl, "position failed");
    default:
        return {ACK_E_UNKNOWN, "unknown simple cmd"};
    }
}

RouteResult TlvCmdRouter::HandleMove(const TlvFrame &frame)
{
    MoveGoal goal;
    RouteResult result = DecodeMoveGoal(frame, goal);
    if (result.status != ACK_OK) {
        return result;
    }
    result = ValidateMoveGate(goal);
    if (result.status != ACK_OK) {
        return result;
    }
    return ApplyMoveGoal(goal);
}

RouteResult TlvCmdRouter::DecodeMoveGoal(const TlvFrame &frame, MoveGoal &goal)
{
    if (frame.len != MOVE_PAYLOAD_LEN && frame.len != MOVE_RC_PAYLOAD_LEN) {
        return {ACK_E_BAD_LEN, "bad move len"};
    }
    const uint8_t *payload = frame.payload.data();
    goal.frame = payload[0];
    goal.seq = frame.seq;
    goal.isVelocity = (frame.flags & MOVE_FLAG_VELOCITY) != 0;
    goal.isRcJoystick = (frame.flags & MOVE_FLAG_RC_JOYSTICK) != 0;
    if (goal.isRcJoystick) {
        return DecodeRcMoveGoal(frame, goal);
    }
    return DecodeSetpointMoveGoal(frame, goal);
}

RouteResult TlvCmdRouter::DecodeRcMoveGoal(const TlvFrame &frame, MoveGoal &goal)
{
    if (frame.len != MOVE_RC_PAYLOAD_LEN) {
        return {ACK_E_BAD_LEN, "bad rc move len"};
    }
    const uint8_t *payload = frame.payload.data();
    goal.throttleNorm = ReadF32Le(&payload[1]);
    goal.yawNorm = ReadF32Le(&payload[5]);
    goal.pitchNorm = ReadF32Le(&payload[9]);
    goal.rollNorm = ReadF32Le(&payload[13]);
    goal.maxV = ReadF32Le(&payload[17]);
    if (!IsFinite(goal.throttleNorm) || !IsFinite(goal.yawNorm) ||
        !IsFinite(goal.pitchNorm) || !IsFinite(goal.rollNorm) ||
        !IsFinite(goal.maxV)) {
        return {ACK_E_BAD_ARGS, "non-finite rc move"};
    }
    return {ACK_OK, ""};
}

RouteResult TlvCmdRouter::DecodeSetpointMoveGoal(const TlvFrame &frame, MoveGoal &goal)
{
    const uint8_t *payload = frame.payload.data();
    const float valueA = ReadF32Le(&payload[1]);
    const float valueB = ReadF32Le(&payload[5]);
    const float valueC = ReadF32Le(&payload[9]);
    const float valueD = ReadF32Le(&payload[13]);
    goal.maxV = ReadF32Le(&payload[17]);
    if (!IsFinite(valueA) || !IsFinite(valueB) || !IsFinite(valueC) ||
        !IsFinite(valueD) || !IsFinite(goal.maxV)) {
        return {ACK_E_BAD_ARGS, "non-finite move"};
    }
    if (goal.isVelocity) {
        goal.vx = valueA;
        goal.vy = valueB;
        goal.vz = valueC;
        goal.yawRate = valueD;
        return {ACK_OK, ""};
    }
    goal.x = valueA;
    goal.y = valueB;
    goal.z = valueC;
    goal.yaw = valueD;
    return {ACK_OK, ""};
}

RouteResult TlvCmdRouter::ValidateMoveGate(const MoveGoal &goal)
{
    if (!goal.isRcJoystick) {
        const RuntimeCommandGate gate = m_commandHook.ReadCommandGate();
        if (!gate.localizationReady) {
            return {ACK_E_BAD_STATE, "vio not ok"};
        }
        if (!gate.guidedControlReady) {
            return {ACK_E_BAD_STATE, "offboard not ready"};
        }
    }
    if (!(goal.maxV > 0.0f)) {
        return {ACK_E_BAD_ARGS, "bad maxV"};
    }
    return {ACK_OK, ""};
}

RouteResult TlvCmdRouter::ApplyMoveGoal(const MoveGoal &goal)
{
    MoveGoal clampedGoal = goal;
    if (clampedGoal.maxV > 5.0f) {
        clampedGoal.maxV = 5.0f;
    }
    std::string errorMessage;
    if (!m_commandHook.ApplyMoveGoal(clampedGoal, &errorMessage)) {
        return {ACK_E_INTERNAL, errorMessage.empty() ? "move failed" : errorMessage};
    }
    return {ACK_OK, BuildMoveAcceptedMessage(clampedGoal)};
}

std::string TlvCmdRouter::BuildMoveAcceptedMessage(const MoveGoal &goal) const
{
    if (goal.isRcJoystick) {
        return "move(rc) accepted thr=" + std::to_string(goal.throttleNorm) +
               " yaw=" + std::to_string(goal.yawNorm) +
               " pitch=" + std::to_string(goal.pitchNorm) +
               " roll=" + std::to_string(goal.rollNorm) +
               " maxV=" + std::to_string(goal.maxV);
    }
    if (goal.isVelocity) {
        return "move(vel) accepted vx=" + std::to_string(goal.vx) +
               " vy=" + std::to_string(goal.vy) +
               " vz=" + std::to_string(goal.vz) +
               " yawRate=" + std::to_string(goal.yawRate) +
               " maxV=" + std::to_string(goal.maxV);
    }
    return "move(pos) accepted x=" + std::to_string(goal.x) +
           " y=" + std::to_string(goal.y) + " z=" + std::to_string(goal.z) +
           " yaw=" + std::to_string(goal.yaw) +
           " maxV=" + std::to_string(goal.maxV);
}
