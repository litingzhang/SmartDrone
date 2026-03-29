#pragma once

#include <algorithm>
#include <vector>

#include "ImuTypes.h"

#include <vector>

#include "core/application/imu_buffer.hpp"
#include "core/ports/imu_provider.hpp"

namespace smartdrone::adapters::imu {

struct Icm42688ImuProviderConfig {
    int64_t slackBeforeNs{5000000};
    int64_t slackAfterNs{5000000};
};

class Icm42688ImuProvider final : public core::ports::IImuProvider {
public:
    Icm42688ImuProvider(ImuBuffer& buffer, Icm42688ImuProviderConfig cfg)
        : m_buffer(buffer), m_cfg(cfg)
    {
    }

    bool Start() override { return true; }
    void Stop() override {}
    bool Ready() const override { return m_buffer.Size() > 0; }

    std::vector<core::ports::ImuReading> PopWindow(int64_t fromNs, int64_t toNs) override
    {
        std::vector<core::ports::ImuReading> out;
        std::vector<ORB_SLAM3::IMU::Point> window = m_buffer.PopBetweenNs(
            fromNs,
            toNs,
            m_cfg.slackBeforeNs,
            m_cfg.slackAfterNs);
        out.reserve(window.size());
        for (const auto& imuPoint : window) {
            core::ports::ImuReading reading{};
            reading.timestampNs = static_cast<int64_t>(imuPoint.t * 1e9);
            reading.ax = imuPoint.a.x();
            reading.ay = imuPoint.a.y();
            reading.az = imuPoint.a.z();
            reading.gx = imuPoint.w.x();
            reading.gy = imuPoint.w.y();
            reading.gz = imuPoint.w.z();
            out.push_back(reading);
        }
        return out;
    }

private:
    ImuBuffer& m_buffer;
    Icm42688ImuProviderConfig m_cfg;
};

}  // namespace smartdrone::adapters::imu
