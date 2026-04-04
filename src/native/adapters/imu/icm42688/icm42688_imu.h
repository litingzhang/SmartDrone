#pragma once

#include <cstdint>

#include "core/application/imu_buffer.h"
#include "platform/linux/spi/spi_dev.h"

constexpr uint8_t REG_DEVICE_CONFIG = 0x11;
constexpr uint8_t REG_INT_CONFIG = 0x14;
constexpr uint8_t REG_INT_STATUS = 0x2D;
constexpr uint8_t REG_PWR_MGMT0 = 0x4E;
constexpr uint8_t REG_GYRO_CONFIG0 = 0x4F;
constexpr uint8_t REG_ACCEL_CONFIG0 = 0x50;
constexpr uint8_t REG_INT_CONFIG1 = 0x64;
constexpr uint8_t REG_INT_SOURCE0 = 0x65;

int16_t Be16ToI16(uint8_t hi, uint8_t lo);
bool SetThreadRealtime(int priority);
uint8_t OdrCodeFromHz(int hz);
bool BuildFsBitsAndScale(
    int accelFsG,
    int gyroFsDps,
    uint8_t& accelFsBits,
    uint8_t& gyroFsBits,
    ImuScale& scale);
bool IcmResetAndConfig(
    SpiDev& spi,
    int imuHz,
    int accelFsG,
    int gyroFsDps,
    ImuScale& scaleOut);
void ConvertRaw12AccelGyroToSi(const uint8_t raw12[12], const ImuScale& scale, ImuSample& sample);
