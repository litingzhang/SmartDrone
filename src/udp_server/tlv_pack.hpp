#pragma once

#include "crc16_ccitt_false.hpp"
#include "tlv_protocol.hpp"

#include <cstdint>
#include <cstring>
#include <vector>

inline void WriteU16Le(std::vector<uint8_t>& buffer, uint16_t value)
{
    buffer.push_back(static_cast<uint8_t>(value & 0xFF));
    buffer.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

inline void WriteU32Le(std::vector<uint8_t>& buffer, uint32_t value)
{
    buffer.push_back(static_cast<uint8_t>(value & 0xFF));
    buffer.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    buffer.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    buffer.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

inline void WriteF32Le(std::vector<uint8_t>& buffer, float value)
{
    uint32_t raw = 0;
    static_assert(sizeof(raw) == sizeof(value), "float size mismatch");
    std::memcpy(&raw, &value, sizeof(raw));
    WriteU32Le(buffer, raw);
}

inline std::vector<uint8_t> MakeFrame(
    uint8_t ver,
    uint8_t cmd,
    uint8_t flags,
    uint32_t seq,
    uint32_t tMs,
    const uint8_t* payload,
    uint16_t len)
{
    std::vector<uint8_t> out;
    out.reserve(static_cast<size_t>(2 + 1 + 1 + 1 + 2 + 4 + 4 + len + 2));

    out.push_back(TLV_SYNC0);
    out.push_back(TLV_SYNC1);
    out.push_back(ver);
    out.push_back(cmd);
    out.push_back(flags);

    WriteU16Le(out, len);
    WriteU32Le(out, seq);
    WriteU32Le(out, tMs);

    if (len > 0 && payload != nullptr) {
        out.insert(out.end(), payload, payload + len);
    }

    const uint8_t* crcBase = out.data() + 2;
    const size_t crcLen = out.size() - 2;
    const uint16_t crc = Crc16CcittFalse(crcBase, crcLen);
    WriteU16Le(out, crc);
    return out;
}

inline void WriteI16Le(uint8_t* p, int16_t value)
{
    p[0] = static_cast<uint8_t>(value & 0xFF);
    p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

inline void WriteU16LeToPtr(uint8_t* p, uint16_t value)
{
    p[0] = static_cast<uint8_t>(value & 0xFF);
    p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

inline void WriteU32LeToPtr(uint8_t* p, uint32_t value)
{
    p[0] = static_cast<uint8_t>(value & 0xFF);
    p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

inline std::vector<uint8_t> MakeAckFrame(
    uint32_t reqSeq,
    uint32_t tMs,
    uint8_t ackCmd,
    uint32_t ackSeq,
    int16_t status)
{
    uint8_t payload[ACK_PAYLOAD_LEN]{};
    payload[0] = ackCmd;
    WriteU32LeToPtr(&payload[1], ackSeq);
    WriteI16Le(&payload[5], status);
    WriteU16LeToPtr(&payload[7], 0);

    return MakeFrame(TLV_VER, CMD_ACK, 0, reqSeq, tMs, payload, ACK_PAYLOAD_LEN);
}
