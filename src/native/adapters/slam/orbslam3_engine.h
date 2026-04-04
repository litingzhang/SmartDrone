#pragma once

#include <cstdint>
#include <memory>

#include "System.h"
#include "core/domain/runtime_mode.h"
#include "core/ports/slam_engine.h"

namespace smartdrone::adapters::slam {

enum class OrbInputMode : uint8_t {
    Stereo,
    MonoLeft,
    MonoRight,
};

class OrbSlam3Engine final : public core::ports::ISlamEngine {
  public:
    OrbSlam3Engine(std::unique_ptr<ORB_SLAM3::System> system, OrbInputMode inputMode, bool useImu);

    bool Start() override;
    void SetOperationMode(core::domain::SlamOperationMode mode);
    void Stop() override;
    core::ports::SlamOutput Process(const core::ports::SlamInputBatch &input, bool extractFeatures,
                                    bool extractPointCloud) override;

  private:
    std::unique_ptr<ORB_SLAM3::System> m_system;
    OrbInputMode m_inputMode{OrbInputMode::Stereo};
    bool m_useImu{false};
    core::domain::SlamOperationMode m_operationMode{core::domain::SlamOperationMode::Mapping};
};

} // namespace smartdrone::adapters::slam
