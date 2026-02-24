#pragma once
#include "crc16_ccitt_false.hpp"
#include "tlv_protocol.hpp"
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

static inline std::vector<uint8_t> makeFrame(
    uint8_t ver, uint8_t cmd, uint8_t flags,
    uint32_t seq, uint32_t t_ms,
    const uint8_t* payload, uint16_t len)
{
    std::vector<uint8_t> out;
    out.reserve((size_t)2 + 1+1+1+2+4+4 + len + 2);

    out.push_back(TLV_SYNC0);
    out.push_back(TLV_SYNC1);

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

static inline void wrI16LE(uint8_t* p, int16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void wrU16LEp(uint8_t* p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void wrU32LEp(uint8_t* p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline std::vector<uint8_t> makeAckFrame(
    uint32_t req_seq, uint32_t t_ms,
    uint8_t ack_cmd, uint32_t ack_seq, int16_t status)
{
    uint8_t payload[ACK_PAYLOAD_LEN]{};
    payload[0] = ack_cmd;
    wrU32LEp(&payload[1], ack_seq);
    wrI16LE(&payload[5], status);
    wrU16LEp(&payload[7], 0);

    return makeFrame(TLV_VER, CMD_ACK, 0, req_seq, t_ms, payload, ACK_PAYLOAD_LEN);
}