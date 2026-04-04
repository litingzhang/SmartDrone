#include "adapters/imu/icm42688_imu_provider.h"

namespace smartdrone::adapters::imu {

Icm42688ImuProvider::Icm42688ImuProvider(ImuBuffer &buffer, Icm42688ImuProviderConfig cfg)
    : m_buffer(buffer), m_cfg(cfg)
{
}

bool Icm42688ImuProvider::Start() { return true; }

void Icm42688ImuProvider::Stop() {}

bool Icm42688ImuProvider::Ready() const { return m_buffer.Size() > 0; }

std::vector<core::ports::ImuReading> Icm42688ImuProvider::PopWindow(int64_t fromNs, int64_t toNs)
{
    std::vector<core::ports::ImuReading> out;
    std::vector<ORB_SLAM3::IMU::Point> window =
        m_buffer.PopBetweenNs(fromNs, toNs, m_cfg.slackBeforeNs, m_cfg.slackAfterNs);
    out.reserve(window.size());
    for (const auto &imuPoint : window) {
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

} // namespace smartdrone::adapters::imu
