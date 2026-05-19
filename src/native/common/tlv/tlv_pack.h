#pragma once

#include <cstdint>
#include <vector>

void WriteU16Le(std::vector<uint8_t> &buffer, uint16_t value);
void WriteU32Le(std::vector<uint8_t> &buffer, uint32_t value);
void WriteF32Le(std::vector<uint8_t> &buffer, float value);
uint16_t ReadU16Le(const uint8_t *p);
uint32_t ReadU32Le(const uint8_t *p);
float ReadF32Le(const uint8_t *p);

struct TlvFrameBuildRequest {
    uint8_t ver{0};
    uint8_t cmd{0};
    uint8_t flags{0};
    uint32_t seq{0};
    uint32_t tMs{0};
    const uint8_t *payload{nullptr};
    uint16_t len{0};
};

std::vector<uint8_t> MakeFrame(const TlvFrameBuildRequest &request);
void WriteI16Le(uint8_t *p, int16_t value);
void WriteU16LeToPtr(uint8_t *p, uint16_t value);
void WriteU32LeToPtr(uint8_t *p, uint32_t value);
std::vector<uint8_t> MakeAckFrame(uint32_t reqSeq, uint32_t tMs, uint8_t ackCmd, uint32_t ackSeq, int16_t status);
