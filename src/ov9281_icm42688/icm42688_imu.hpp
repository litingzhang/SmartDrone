#pragma once

#include <pthread.h>
#include <sched.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>

#include "imu_buffer.hpp"
#include "spi_dev.hpp"

constexpr uint8_t REG_DEVICE_CONFIG = 0x11;
constexpr uint8_t REG_INT_CONFIG = 0x14;
constexpr uint8_t REG_INT_STATUS = 0x2D;
constexpr uint8_t REG_PWR_MGMT0 = 0x4E;
constexpr uint8_t REG_GYRO_CONFIG0 = 0x4F;
constexpr uint8_t REG_ACCEL_CONFIG0 = 0x50;
constexpr uint8_t REG_INT_CONFIG1 = 0x64;
constexpr uint8_t REG_INT_SOURCE0 = 0x65;

inline int16_t Be16ToI16(uint8_t hi, uint8_t lo)
{
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo));
}

inline bool SetThreadRealtime(int priority)
{
    sched_param schedParam{};
    schedParam.sched_priority = priority;
    return pthread_setschedparam(pthread_self(), SCHED_FIFO, &schedParam) == 0;
}

inline uint8_t OdrCodeFromHz(int hz)
{
    switch (hz) {
        case 8000:
            return 0x03;
        case 4000:
            return 0x04;
        case 2000:
            return 0x05;
        case 1000:
            return 0x06;
        case 200:
            return 0x07;
        case 100:
            return 0x08;
        case 50:
            return 0x09;
        case 25:
            return 0x0A;
        default:
            return 0x0F;
    }
}

inline bool BuildFsBitsAndScale(
    int accelFsG, int gyroFsDps, uint8_t& accelFsBits, uint8_t& gyroFsBits, ImuScale& scale)
{
    switch (accelFsG) {
        case 2:
            scale.accelLsbPerG = 16384.0f;
            accelFsBits = 0x03;
            break;
        case 4:
            scale.accelLsbPerG = 8192.0f;
            accelFsBits = 0x02;
            break;
        case 8:
            scale.accelLsbPerG = 4096.0f;
            accelFsBits = 0x01;
            break;
        case 16:
            scale.accelLsbPerG = 2048.0f;
            accelFsBits = 0x00;
            break;
        default:
            std::cerr << "Unsupported accel-fs " << accelFsG << " (use 2/4/8/16)\n";
            return false;
    }

    switch (gyroFsDps) {
        case 125:
            scale.gyroLsbPerDps = 262.4f;
            gyroFsBits = 0x04;
            break;
        case 250:
            scale.gyroLsbPerDps = 131.0f;
            gyroFsBits = 0x03;
            break;
        case 500:
            scale.gyroLsbPerDps = 65.5f;
            gyroFsBits = 0x02;
            break;
        case 1000:
            scale.gyroLsbPerDps = 32.8f;
            gyroFsBits = 0x01;
            break;
        case 2000:
            scale.gyroLsbPerDps = 16.4f;
            gyroFsBits = 0x00;
            break;
        default:
            std::cerr << "Unsupported gyro-fs " << gyroFsDps
                      << " (use 125/250/500/1000/2000)\n";
            return false;
    }

    return true;
}

inline bool IcmResetAndConfig(
    SpiDev& spi, int imuHz, int accelFsG, int gyroFsDps, ImuScale& scaleOut)
{
    uint8_t accelFsBits = 0;
    uint8_t gyroFsBits = 0;
    ImuScale scale;
    if (!BuildFsBitsAndScale(accelFsG, gyroFsDps, accelFsBits, gyroFsBits, scale)) {
        return false;
    }

    if (!spi.WriteReg(REG_DEVICE_CONFIG, 0x01)) {
        return false;
    }
    usleep(100000);

    spi.WriteReg(REG_INT_CONFIG, 0x30);
    spi.WriteReg(REG_INT_SOURCE0, 0x08);
    spi.WriteReg(REG_INT_CONFIG1, 0x00);

    spi.WriteReg(REG_PWR_MGMT0, 0x0F);
    usleep(20000);

    uint8_t odrCode = OdrCodeFromHz(imuHz);
    uint8_t gyroConfig0 = static_cast<uint8_t>((gyroFsBits << 5) | (odrCode & 0x0F));
    uint8_t accelConfig0 = static_cast<uint8_t>((accelFsBits << 5) | (odrCode & 0x0F));

    if (!spi.WriteReg(REG_GYRO_CONFIG0, gyroConfig0)) {
        return false;
    }
    if (!spi.WriteReg(REG_ACCEL_CONFIG0, accelConfig0)) {
        return false;
    }
    usleep(20000);

    scaleOut = scale;
    return true;
}

inline void ConvertRaw12AccelGyroToSi(const uint8_t raw12[12], const ImuScale& scale, ImuSample& sample)
{
    int16_t ax = Be16ToI16(raw12[0], raw12[1]);
    int16_t ay = Be16ToI16(raw12[2], raw12[3]);
    int16_t az = Be16ToI16(raw12[4], raw12[5]);
    int16_t gx = Be16ToI16(raw12[6], raw12[7]);
    int16_t gy = Be16ToI16(raw12[8], raw12[9]);
    int16_t gz = Be16ToI16(raw12[10], raw12[11]);

    constexpr float K_GRAVITY = 9.80665f;
    sample.ax = (static_cast<float>(ax) / scale.accelLsbPerG) * K_GRAVITY;
    sample.ay = (static_cast<float>(ay) / scale.accelLsbPerG) * K_GRAVITY;
    sample.az = (static_cast<float>(az) / scale.accelLsbPerG) * K_GRAVITY;

    constexpr float K_DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
    sample.gx = (static_cast<float>(gx) / scale.gyroLsbPerDps) * K_DEG_TO_RAD;
    sample.gy = (static_cast<float>(gy) / scale.gyroLsbPerDps) * K_DEG_TO_RAD;
    sample.gz = (static_cast<float>(gz) / scale.gyroLsbPerDps) * K_DEG_TO_RAD;
}
