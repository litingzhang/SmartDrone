#pragma once

#include <vector>

#include "ImuTypes.h"
#include "core/application/state/imu_buffer.h"
#include "core/ports/imu_provider.h"

namespace smartdrone::adapters::imu {

struct Icm42688ImuProviderConfig {
    int64_t slackBeforeNs{5000000};
    int64_t slackAfterNs{5000000};
};

class Icm42688ImuProvider final : public core::ports::IImuProvider {
  public:
    Icm42688ImuProvider(ImuBuffer &buffer, Icm42688ImuProviderConfig cfg);

    bool Start() override;
    void Stop() override;
    bool Ready() const override;
    std::vector<core::ports::ImuReading> PopWindow(int64_t fromNs, int64_t toNs) override;

  private:
    ImuBuffer &m_buffer;
    Icm42688ImuProviderConfig m_cfg;
};

} // namespace smartdrone::adapters::imu
