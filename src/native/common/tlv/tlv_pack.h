#pragma once

#include <cstdint>
#include <vector>

void WriteU16Le(std::vector<uint8_t>& buffer, uint16_t value);
void WriteU32Le(std::vector<uint8_t>& buffer, uint32_t value);
void WriteF32Le(std::vector<uint8_t>& buffer, float value);
uint16_t ReadU16Le(const uint8_t* p);
uint32_t ReadU32Le(const uint8_t* p);
float ReadF32Le(const uint8_t* p);
std::vector<uint8_t> MakeFrame(
    uint8_t ver,
    uint8_t cmd,
    uint8_t flags,
    uint32_t seq,
    uint32_t tMs,
    const uint8_t* payload,
    uint16_t len);
void WriteI16Le(uint8_t* p, int16_t value);
void WriteU16LeToPtr(uint8_t* p, uint16_t value);
void WriteU32LeToPtr(uint8_t* p, uint32_t value);
std::vector<uint8_t> MakeAckFrame(
    uint32_t reqSeq,
    uint32_t tMs,
    uint8_t ackCmd,
    uint32_t ackSeq,
    int16_t status);
