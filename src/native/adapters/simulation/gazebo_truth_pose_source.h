#pragma once

#include <memory>

#include "core/ports/external_pose_source.h"

namespace SmartDrone::Adapters::Simulation {

class GazeboMeasurementClock;

class GazeboTruthPoseSource final
    : public SmartDrone::Core::Ports::IExternalPoseSource {
  public:
    explicit GazeboTruthPoseSource(
        std::shared_ptr<GazeboMeasurementClock> measurementClock);
    ~GazeboTruthPoseSource() override;

    bool Open();
    void Close();
    bool TryRead(SmartDrone::Core::Ports::PosePublishRequest &out) override;
    bool Healthy() const override;

  private:
    struct Impl;

    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Adapters::Simulation
