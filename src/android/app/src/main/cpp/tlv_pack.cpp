#include "tlv_pack.h"

#include "crc16_ccitt_false.h"

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
    std::memcpy(&raw, &value, sizeof(raw));
    WriteU32Le(buffer, raw);
}

std::vector<uint8_t> MakeFrame(const TlvFrameRequest &request)
{
    std::vector<uint8_t> out;
    out.reserve(static_cast<size_t>(2 + 1 + 1 + 1 + 2 + 4 + 4 + request.len + 2));

    out.push_back(0xAA);
    out.push_back(0x55);
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

std::vector<uint8_t> MakeMovePayload(const MovePayloadValues &values)
{
    std::vector<uint8_t> payload;
    payload.reserve(21);
    payload.push_back(values.frame);
    WriteF32Le(payload, values.valueA);
    WriteF32Le(payload, values.valueB);
    WriteF32Le(payload, values.valueC);
    WriteF32Le(payload, values.valueD);
    WriteF32Le(payload, values.maxV);
    return payload;
}

std::vector<uint8_t> MakeMoveRcPayload(const MovePayloadValues &values)
{
    std::vector<uint8_t> payload;
    payload.reserve(21);
    payload.push_back(values.frame);
    WriteF32Le(payload, values.valueA);
    WriteF32Le(payload, values.valueB);
    WriteF32Le(payload, values.valueC);
    WriteF32Le(payload, values.valueD);
    WriteF32Le(payload, values.maxV);
    return payload;
}
