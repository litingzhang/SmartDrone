#include "adapters/imu/icm42688/icm42688_imu.h"

#include <iostream>

int16_t Be16ToI16(uint8_t hi, uint8_t lo)
{
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo));
}

uint8_t OdrCodeFromHz(int hz)
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

bool BuildAccelFsBitsAndScale(int accelFsG, uint8_t &accelFsBits, ImuScale &scale)
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
    return true;
}

bool BuildGyroFsBitsAndScale(int gyroFsDps, uint8_t &gyroFsBits, ImuScale &scale)
{
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
        std::cerr << "Unsupported gyro-fs " << gyroFsDps << " (use 125/250/500/1000/2000)\n";
        return false;
    }

    return true;
}

bool BuildFsBitsAndScale(int accelFsG, int gyroFsDps, uint8_t &accelFsBits, uint8_t &gyroFsBits, ImuScale &scale)
{
    return BuildAccelFsBitsAndScale(accelFsG, accelFsBits, scale) &&
           BuildGyroFsBitsAndScale(gyroFsDps, gyroFsBits, scale);
}

void ConvertRaw12AccelGyroToSi(const uint8_t raw12[12], const ImuScale &scale, ImuSample &sample)
{
    const int16_t ax = Be16ToI16(raw12[0], raw12[1]);
    const int16_t ay = Be16ToI16(raw12[2], raw12[3]);
    const int16_t az = Be16ToI16(raw12[4], raw12[5]);
    const int16_t gx = Be16ToI16(raw12[6], raw12[7]);
    const int16_t gy = Be16ToI16(raw12[8], raw12[9]);
    const int16_t gz = Be16ToI16(raw12[10], raw12[11]);

    constexpr float K_GRAVITY = 9.80665f;
    sample.ax = (static_cast<float>(ax) / scale.accelLsbPerG) * K_GRAVITY;
    sample.ay = (static_cast<float>(ay) / scale.accelLsbPerG) * K_GRAVITY;
    sample.az = (static_cast<float>(az) / scale.accelLsbPerG) * K_GRAVITY;

    constexpr float K_DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
    sample.gx = (static_cast<float>(gx) / scale.gyroLsbPerDps) * K_DEG_TO_RAD;
    sample.gy = (static_cast<float>(gy) / scale.gyroLsbPerDps) * K_DEG_TO_RAD;
    sample.gz = (static_cast<float>(gz) / scale.gyroLsbPerDps) * K_DEG_TO_RAD;
}

void Icm42688ConfigSequencer::Reset(int imuHz, int accelFsG, int gyroFsDps)
{
    m_stage = Stage::Start;
    m_nextStage = Stage::Done;
    m_resumeAt = {};
    m_scale = {};
    m_accelFsBits = 0;
    m_gyroFsBits = 0;
    m_imuHz = imuHz;
    if (!BuildFsBitsAndScale(accelFsG, gyroFsDps, m_accelFsBits, m_gyroFsBits, m_scale)) {
        m_stage = Stage::Failed;
    }
}

Icm42688ConfigSequencer::Status Icm42688ConfigSequencer::Step(SpiDev &spi, ImuScale &scaleOut)
{
    const auto now = std::chrono::steady_clock::now();
    if (m_stage == Stage::Done) {
        scaleOut = m_scale;
        return Status::Done;
    }
    if (m_stage == Stage::Failed) {
        return Status::Failed;
    }
    if (!DelayElapsed(now)) {
        return Status::Pending;
    }
    AdvanceWaitStage();
    if (m_stage == Stage::Done) {
        scaleOut = m_scale;
        return Status::Done;
    }

    if (m_stage == Stage::Start) {
        return StepStart(spi, now);
    }
    if (m_stage == Stage::PowerOn) {
        return StepPowerOn(spi, now);
    }
    if (m_stage == Stage::ConfigureRate) {
        return StepConfigureRate(spi, now);
    }
    return Status::Pending;
}

bool Icm42688ConfigSequencer::DelayElapsed(std::chrono::steady_clock::time_point now) const
{
    return m_resumeAt.time_since_epoch().count() == 0 || now >= m_resumeAt;
}

void Icm42688ConfigSequencer::AdvanceWaitStage()
{
    if (m_stage == Stage::DelayAfterReset || m_stage == Stage::DelayAfterPower || m_stage == Stage::DelayAfterRate) {
        m_stage = m_nextStage;
    }
}

void Icm42688ConfigSequencer::ScheduleDelay(
    std::chrono::steady_clock::time_point now,
    std::chrono::milliseconds delay,
    Stage nextStage)
{
    m_nextStage = nextStage;
    m_resumeAt = now + delay;
    if (nextStage == Stage::PowerOn) {
        m_stage = Stage::DelayAfterReset;
    } else if (nextStage == Stage::ConfigureRate) {
        m_stage = Stage::DelayAfterPower;
    } else {
        m_stage = Stage::DelayAfterRate;
    }
}

Icm42688ConfigSequencer::Status Icm42688ConfigSequencer::StepStart(
    SpiDev &spi, std::chrono::steady_clock::time_point now)
{
    if (!spi.WriteReg(REG_DEVICE_CONFIG, 0x01)) {
        return Fail();
    }
    ScheduleDelay(now, std::chrono::milliseconds(100), Stage::PowerOn);
    return Status::Pending;
}

Icm42688ConfigSequencer::Status Icm42688ConfigSequencer::StepPowerOn(
    SpiDev &spi, std::chrono::steady_clock::time_point now)
{
    spi.WriteReg(REG_INT_CONFIG, 0x30);
    spi.WriteReg(REG_INT_SOURCE0, 0x08);
    spi.WriteReg(REG_INT_CONFIG1, 0x00);
    spi.WriteReg(REG_PWR_MGMT0, 0x0F);
    ScheduleDelay(now, std::chrono::milliseconds(20), Stage::ConfigureRate);
    return Status::Pending;
}

Icm42688ConfigSequencer::Status Icm42688ConfigSequencer::StepConfigureRate(
    SpiDev &spi, std::chrono::steady_clock::time_point now)
{
    const uint8_t odrCode = OdrCodeFromHz(m_imuHz);
    const uint8_t gyroConfig0 = static_cast<uint8_t>((m_gyroFsBits << 5) | (odrCode & 0x0F));
    const uint8_t accelConfig0 = static_cast<uint8_t>((m_accelFsBits << 5) | (odrCode & 0x0F));
    if (!spi.WriteReg(REG_GYRO_CONFIG0, gyroConfig0) || !spi.WriteReg(REG_ACCEL_CONFIG0, accelConfig0)) {
        return Fail();
    }
    ScheduleDelay(now, std::chrono::milliseconds(20), Stage::Done);
    return Status::Pending;
}

Icm42688ConfigSequencer::Status Icm42688ConfigSequencer::Fail()
{
    m_stage = Stage::Failed;
    return Status::Failed;
}
