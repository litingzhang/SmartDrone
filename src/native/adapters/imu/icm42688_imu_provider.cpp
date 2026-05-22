#include "adapters/imu/icm42688_imu_provider.h"

namespace SmartDrone::Adapters::Imu {

Icm42688ImuProvider::Icm42688ImuProvider(ImuBuffer &buffer, Icm42688ImuProviderConfig cfg)
    : m_buffer(buffer), m_cfg(cfg)
{
}

bool Icm42688ImuProvider::Start()
{
    return true;
}

void Icm42688ImuProvider::Stop()
{
}

bool Icm42688ImuProvider::Ready() const
{
    return m_buffer.Size() > 0;
}

std::vector<Core::Ports::ImuReading> Icm42688ImuProvider::PopWindow(int64_t fromNs, int64_t toNs)
{
    return m_buffer.PopBetweenNs(fromNs, toNs, m_cfg.slackBeforeNs, m_cfg.slackAfterNs);
}

} // namespace SmartDrone::Adapters::Imu
