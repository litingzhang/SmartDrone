#pragma once

#include "crc16_ccitt_false.hpp"

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

    out.push_back(0xAA);
    out.push_back(0x55);
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

inline std::vector<uint8_t> MakeMovePayload(
    uint8_t frame,
    float x,
    float y,
    float z,
    float yaw,
    float maxV)
{
    std::vector<uint8_t> payload;
    payload.reserve(21);
    payload.push_back(frame);
    WriteF32Le(payload, x);
    WriteF32Le(payload, y);
    WriteF32Le(payload, z);
    WriteF32Le(payload, yaw);
    WriteF32Le(payload, maxV);
    return payload;
}
