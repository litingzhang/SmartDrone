#pragma once

#include "mavlink_hooks.hpp"
#include "tlv_pack.hpp"
#include "tlv_parser.hpp"
#include "tlv_protocol.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>

struct RouteResult {
    int16_t status{ACK_OK};
    std::string msg;
};

class TlvCmdRouter {
public:
    using Handler = std::function<RouteResult(const TlvFrame&)>;

    explicit TlvCmdRouter(MavlinkHooks& hooks);

    void RegisterDefaults();
    RouteResult Handle(const TlvFrame& frame);

private:
    struct SeqTracker {
        uint32_t lastSeq{0};
        uint32_t lastSenderMs{0};
        std::chrono::steady_clock::time_point lastRxTime{};
        bool seen{false};
    };

    static float ReadF32Le(const uint8_t* p);
    static uint32_t ReadU32Le(const uint8_t* p);

    bool AcceptSeq(uint32_t seq, uint32_t senderMs, SeqTracker& tracker);
    RouteResult HandleMove(const TlvFrame& frame);
    RouteResult HandleSimple(uint8_t cmd);

    MavlinkHooks& m_hooks;
    std::unordered_map<uint8_t, Handler> m_handlers;
    SeqTracker m_cmdSeqTracker;
    SeqTracker m_moveSeqTracker;
};
