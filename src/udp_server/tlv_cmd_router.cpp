#include "tlv_cmd_router.hpp"
#include <cstring>
#include <iostream>

TlvCmdRouter::TlvCmdRouter(MavlinkHooks& hooks) : m_hooks(hooks) {}

static inline bool monotonic(uint32_t x, uint32_t& last)
{
    if (x == 0) return false;
    if (x <= last) return false;
    last = x;
    return true;
}

float TlvCmdRouter::rdF32LE(const uint8_t* p)
{
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    float f;
    std::memcpy(&f, &u, sizeof(float));
    return f;
}

uint32_t TlvCmdRouter::rdU32LE(const uint8_t* p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

void TlvCmdRouter::registerDefaults()
{
    m_map[CMD_PING] = [this](const TlvFrame&) -> RouteResult {
        return {ACK_OK, "pong"};
    };

    m_map[CMD_ARM] = [this](const TlvFrame&) { return handleSimple(CMD_ARM); };
    m_map[CMD_DISARM] = [this](const TlvFrame&) { return handleSimple(CMD_DISARM); };
    m_map[CMD_OFFBOARD] = [this](const TlvFrame&) { return handleSimple(CMD_OFFBOARD); };
    m_map[CMD_HOLD] = [this](const TlvFrame&) { return handleSimple(CMD_HOLD); };
    m_map[CMD_LAND] = [this](const TlvFrame&) { return handleSimple(CMD_LAND); };

    m_map[CMD_MOVE] = [this](const TlvFrame& f) { return handleMove(f); };
}

RouteResult TlvCmdRouter::handle(const TlvFrame& f)
{
    // basic version check
    if (f.ver != TLV_VER) return {ACK_E_BAD_ARGS, "bad version"};

    // seq monotonic for all cmds (optional but recommended)
    if (!monotonic(f.seq, m_lastSeq)) return {ACK_E_BAD_STATE, "seq not monotonic"};

    auto it = m_map.find(f.cmd);
    if (it == m_map.end()) return {ACK_E_UNKNOWN, "unknown cmd"};
    return it->second(f);
}

RouteResult TlvCmdRouter::handleSimple(uint8_t cmd)
{
    const auto gate = m_hooks.gate();
    std::string err;

    switch (cmd)
    {
    case CMD_ARM:
        if (!gate.vioOk) return {ACK_E_BAD_STATE, "vio not ok"};
        if (!m_hooks.arm(&err)) return {ACK_E_INTERNAL, err.empty() ? "arm failed" : err};
        return {ACK_OK, ""};

    case CMD_DISARM:
        if (!m_hooks.disarm(&err)) return {ACK_E_INTERNAL, err.empty() ? "disarm failed" : err};
        return {ACK_OK, ""};

    case CMD_OFFBOARD:
        if (!gate.vioOk) return {ACK_E_BAD_STATE, "vio not ok"};
        if (!gate.offboardReady) return {ACK_E_BAD_STATE, "offboard not ready"};
        if (!m_hooks.setOffboard(&err)) return {ACK_E_INTERNAL, err.empty() ? "offboard failed" : err};
        return {ACK_OK, ""};

    case CMD_HOLD:
        if (!m_hooks.hold(&err)) return {ACK_E_INTERNAL, err.empty() ? "hold failed" : err};
        return {ACK_OK, ""};

    case CMD_LAND:
        if (!m_hooks.land(&err)) return {ACK_E_INTERNAL, err.empty() ? "land failed" : err};
        return {ACK_OK, ""};

    default:
        return {ACK_E_UNKNOWN, "unknown simple cmd"};
    }
}

RouteResult TlvCmdRouter::handleMove(const TlvFrame& f)
{
    if (f.len != MOVE_PAYLOAD_LEN) return {ACK_E_BAD_LEN, "bad move len"};

    const auto gate = m_hooks.gate();
    if (!gate.vioOk) return {ACK_E_BAD_STATE, "vio not ok"};
    if (!gate.offboardReady) return {ACK_E_BAD_STATE, "offboard not ready"};

    // accept only latest move seq (avoid out-of-order jitter)
    if (!monotonic(f.seq, m_lastMoveSeq)) return {ACK_E_BAD_STATE, "old move dropped"};

    const uint8_t* p = f.payload.data();
    MoveGoal g;
    g.frame = p[0];
    g.x = rdF32LE(&p[1]);
    g.y = rdF32LE(&p[5]);
    g.z = rdF32LE(&p[9]);
    g.yaw = rdF32LE(&p[13]);
    g.maxV = rdF32LE(&p[17]);
    g.seq = f.seq;

    // you can clamp bounds here if needed

    std::string err;
    if (!m_hooks.setMoveGoal(g, &err)) return {ACK_E_INTERNAL, err.empty() ? "move failed" : err};
    return {ACK_OK, ""};
}