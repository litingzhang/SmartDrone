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
#include <vector>

struct RouteResult {
    int16_t status{ACK_OK};
    std::string msg;
    uint8_t responseCmd{0};
    std::vector<uint8_t> responsePayload;
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

    bool AcceptSeq(uint32_t seq, uint32_t senderMs, SeqTracker& tracker);
    RouteResult HandleMove(const TlvFrame& frame);
    RouteResult HandleSimple(uint8_t cmd);

    MavlinkHooks& m_hooks;
    std::unordered_map<uint8_t, Handler> m_handlers;
    SeqTracker m_cmdSeqTracker;
    SeqTracker m_moveSeqTracker;
};
