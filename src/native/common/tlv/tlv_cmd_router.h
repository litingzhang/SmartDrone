#pragma once

#include "runtime_command_hooks.h"
#include "tlv_pack.h"
#include "tlv_parser.h"
#include "tlv_protocol.h"

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
    using Handler = std::function<RouteResult(const TlvFrame &)>;

    explicit TlvCmdRouter(RuntimeCommandHook &commandHook);

    void RegisterDefaults();
    RouteResult Handle(const TlvFrame &frame);

  private:
    using SimpleCommandCall = bool (RuntimeCommandHook::*)(std::string *);

    RouteResult ExecuteSimple(SimpleCommandCall call, const char *fallbackError);
    RouteResult HandleMove(const TlvFrame &frame);
    RouteResult HandleSimple(uint8_t cmd);
    RouteResult DecodeMoveGoal(const TlvFrame &frame, MoveGoal &goal);
    RouteResult DecodeRcMoveGoal(const TlvFrame &frame, MoveGoal &goal);
    RouteResult DecodeSetpointMoveGoal(const TlvFrame &frame, MoveGoal &goal);
    RouteResult ValidateMoveGate(const MoveGoal &goal);
    RouteResult ApplyMoveGoal(const MoveGoal &goal);
    std::string BuildMoveAcceptedMessage(const MoveGoal &goal) const;

    RuntimeCommandHook &m_commandHook;
    std::unordered_map<uint8_t, Handler> m_handlers;
};
