#include "common/tlv/tlv_pack.h"

#include "common/tlv/crc16_ccitt_false.h"
#include "common/tlv/tlv_protocol.h"

#include <cstring>

void WriteU16Le(std::vector<uint8_t> &buffer, uint16_t value)
{
    buffer.push_back(static_cast<uint8_t>(value & 0xFF));
    buffer.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void WriteU32Le(std::vector<uint8_t> &buffer, uint32_t value)
{
    buffer.push_back(static_cast<uint8_t>(value & 0xFF));
    buffer.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    buffer.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    buffer.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

void WriteF32Le(std::vector<uint8_t> &buffer, float value)
{
    uint32_t raw = 0;
    static_assert(sizeof(raw) == sizeof(value), "float size mismatch");
    std::memcpy(&raw, &value, sizeof(raw));
    WriteU32Le(buffer, raw);
}

uint16_t ReadU16Le(const uint8_t *p)
{
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t ReadU32Le(const uint8_t *p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

float ReadF32Le(const uint8_t *p)
{
    const uint32_t raw = ReadU32Le(p);
    float out = 0.0f;
    std::memcpy(&out, &raw, sizeof(out));
    return out;
}

std::vector<uint8_t> MakeFrame(const TlvFrameBuildRequest &request)
{
    std::vector<uint8_t> out;
    constexpr size_t frameOverheadBytes = 2 + 1 + 1 + 1 + 2 + 4 + 4 + 2;
    out.reserve(frameOverheadBytes + static_cast<size_t>(request.len));

    out.push_back(TLV_SYNC0);
    out.push_back(TLV_SYNC1);
    out.push_back(request.ver);
    out.push_back(request.cmd);
    out.push_back(request.flags);

    WriteU16Le(out, request.len);
    WriteU32Le(out, request.seq);
    WriteU32Le(out, request.tMs);

    if (request.len > 0 && request.payload != nullptr) {
        out.insert(out.end(), request.payload, request.payload + request.len);
    }

    const uint8_t *crcBase = out.data() + 2;
    const size_t crcLen = out.size() - 2;
    const uint16_t crc = Crc16CcittFalse(crcBase, crcLen);
    WriteU16Le(out, crc);
    return out;
}

void WriteI16Le(uint8_t *p, int16_t value)
{
    p[0] = static_cast<uint8_t>(value & 0xFF);
    p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void WriteU16LeToPtr(uint8_t *p, uint16_t value)
{
    p[0] = static_cast<uint8_t>(value & 0xFF);
    p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void WriteU32LeToPtr(uint8_t *p, uint32_t value)
{
    p[0] = static_cast<uint8_t>(value & 0xFF);
    p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

std::vector<uint8_t> MakeAckFrame(uint32_t reqSeq, uint32_t tMs, uint8_t ackCmd, uint32_t ackSeq, int16_t status)
{
    uint8_t payload[ACK_PAYLOAD_LEN]{};
    payload[0] = ackCmd;
    WriteU32LeToPtr(&payload[1], ackSeq);
    WriteI16Le(&payload[5], status);
    WriteU16LeToPtr(&payload[7], 0);

    const TlvFrameBuildRequest request{
        TLV_VER, CMD_ACK, 0, reqSeq, tMs, payload, ACK_PAYLOAD_LEN};
    return MakeFrame(request);
}
