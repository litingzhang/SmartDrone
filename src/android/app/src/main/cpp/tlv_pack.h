#pragma once

#include <cstdint>
#include <vector>

void WriteU16Le(std::vector<uint8_t> &buffer, uint16_t value);
void WriteU32Le(std::vector<uint8_t> &buffer, uint32_t value);
void WriteF32Le(std::vector<uint8_t> &buffer, float value);

struct TlvFrameRequest {
    uint8_t ver{1};
    uint8_t cmd{0};
    uint8_t flags{0};
    uint32_t seq{0};
    uint32_t tMs{0};
    const uint8_t *payload{nullptr};
    uint16_t len{0};
};

struct MovePayloadValues {
    uint8_t frame{0};
    float valueA{0.0f};
    float valueB{0.0f};
    float valueC{0.0f};
    float valueD{0.0f};
    float maxV{0.0f};
};

std::vector<uint8_t> MakeFrame(const TlvFrameRequest &request);
std::vector<uint8_t> MakeMovePayload(const MovePayloadValues &values);
std::vector<uint8_t> MakeMoveRcPayload(const MovePayloadValues &values);
