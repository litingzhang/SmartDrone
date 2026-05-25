#pragma once

#include <chrono>
#include <cstdint>

#include "core/domain/imu_sample.h"
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
uint8_t OdrCodeFromHz(int hz);
bool BuildFsBitsAndScale(int accelFsG, int gyroFsDps, uint8_t &accelFsBits, uint8_t &gyroFsBits, ImuScale &scale);
void ConvertRaw12AccelGyroToSi(const uint8_t raw12[12], const ImuScale &scale, ImuSample &sample);

class Icm42688ConfigSequencer {
  public:
    enum class Status {
        Pending,
        Done,
        Failed,
    };

    void Reset(int imuHz, int accelFsG, int gyroFsDps);
    Status Step(SpiDev &spi, ImuScale &scaleOut);

  private:
    enum class Stage {
        Start,
        DelayAfterReset,
        PowerOn,
        DelayAfterPower,
        ConfigureRate,
        DelayAfterRate,
        Done,
        Failed,
    };

    bool DelayElapsed(std::chrono::steady_clock::time_point now) const;
    void AdvanceWaitStage();
    void ScheduleDelay(std::chrono::steady_clock::time_point now, std::chrono::milliseconds delay, Stage nextStage);
    Status StepStart(SpiDev &spi, std::chrono::steady_clock::time_point now);
    Status StepPowerOn(SpiDev &spi, std::chrono::steady_clock::time_point now);
    Status StepConfigureRate(SpiDev &spi, std::chrono::steady_clock::time_point now);
    Status Fail();

    Stage m_stage{Stage::Start};
    Stage m_nextStage{Stage::Done};
    std::chrono::steady_clock::time_point m_resumeAt{};
    ImuScale m_scale{};
    uint8_t m_accelFsBits{0};
    uint8_t m_gyroFsBits{0};
    int m_imuHz{0};
};
