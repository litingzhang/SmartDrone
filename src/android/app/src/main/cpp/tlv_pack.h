#pragma once

#include <cstdint>
#include <vector>

void WriteU16Le(std::vector<uint8_t>& buffer, uint16_t value);
void WriteU32Le(std::vector<uint8_t>& buffer, uint32_t value);
void WriteF32Le(std::vector<uint8_t>& buffer, float value);
std::vector<uint8_t> MakeFrame(
    uint8_t ver,
    uint8_t cmd,
    uint8_t flags,
    uint32_t seq,
    uint32_t tMs,
    const uint8_t* payload,
    uint16_t len);
std::vector<uint8_t> MakeMovePayload(
    uint8_t frame,
    float valueA,
    float valueB,
    float valueC,
    float valueD,
    float maxV);
std::vector<uint8_t> MakeMoveRcPayload(
    uint8_t frame,
    float throttle,
    float yaw,
    float pitch,
    float roll,
    float maxV);
