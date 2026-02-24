#pragma once
#include "crc16_ccitt_false.hpp"
#include <cstdint>
#include <vector>
#include <cstring>

static inline void wrU16LE(std::vector<uint8_t>& b, uint16_t v)
{
    b.push_back((uint8_t)(v & 0xFF));
    b.push_back((uint8_t)((v >> 8) & 0xFF));
}
static inline void wrU32LE(std::vector<uint8_t>& b, uint32_t v)
{
    b.push_back((uint8_t)(v & 0xFF));
    b.push_back((uint8_t)((v >> 8) & 0xFF));
    b.push_back((uint8_t)((v >> 16) & 0xFF));
    b.push_back((uint8_t)((v >> 24) & 0xFF));
}
static inline void wrF32LE(std::vector<uint8_t>& b, float f)
{
    uint32_t u;
    std::memcpy(&u, &f, sizeof(u));
    wrU32LE(b, u);
}

static inline std::vector<uint8_t> makeFrame(
    uint8_t ver, uint8_t cmd, uint8_t flags,
    uint32_t seq, uint32_t t_ms,
    const uint8_t* payload, uint16_t len)
{
    std::vector<uint8_t> out;
    out.reserve((size_t)2 + 1+1+1+2+4+4 + len + 2);

    out.push_back(0xAA);
    out.push_back(0x55);

    out.push_back(ver);
    out.push_back(cmd);
    out.push_back(flags);
    wrU16LE(out, len);
    wrU32LE(out, seq);
    wrU32LE(out, t_ms);

    if (len && payload) out.insert(out.end(), payload, payload + len);

    // CRC over VER..PAYLOAD (exclude sync + exclude crc)
    const uint8_t* crcBase = out.data() + 2;
    const size_t crcLen = out.size() - 2;
    uint16_t crc = crc16_ccitt_false(crcBase, crcLen);
    wrU16LE(out, crc);
    return out;
}

static inline std::vector<uint8_t> makeMovePayload(
    uint8_t frame, float x, float y, float z, float yaw, float max_v)
{
    std::vector<uint8_t> p;
    p.reserve(21);
    p.push_back(frame);
    wrF32LE(p, x);
    wrF32LE(p, y);
    wrF32LE(p, z);
    wrF32LE(p, yaw);
    wrF32LE(p, max_v);
    return p;
}