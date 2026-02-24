#pragma once
#include "tlv_protocol.hpp"
#include "tlv_parser.hpp"
#include "tlv_pack.hpp"
#include "mavlink_hooks.hpp"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <functional>

struct RouteResult
{
    int16_t status{ACK_OK};
    std::string msg; // optional log
};

class TlvCmdRouter
{
public:
    using Handler = std::function<RouteResult(const TlvFrame&)>;

    explicit TlvCmdRouter(MavlinkHooks& hooks);

    void registerDefaults();
    RouteResult handle(const TlvFrame& f);

private:
    MavlinkHooks& m_hooks;
    std::unordered_map<uint8_t, Handler> m_map;

    // seq gating: prevent old/dup move
    uint32_t m_lastSeq{0};
    uint32_t m_lastMoveSeq{0};

    static float rdF32LE(const uint8_t* p);
    static uint32_t rdU32LE(const uint8_t* p);

    RouteResult handleMove(const TlvFrame& f);
    RouteResult handleSimple(uint8_t cmd);
};