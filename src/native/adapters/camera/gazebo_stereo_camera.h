#pragma once

#include <memory>

#include "core/ports/camera_provider.h"

namespace SmartDrone::Adapters::Simulation {
class GazeboMeasurementClock;
}

namespace SmartDrone::Adapters::Camera {

class GazeboStereoCamera final
    : public SmartDrone::Core::Ports::ICameraProvider {
  public:
    explicit GazeboStereoCamera(
        std::shared_ptr<SmartDrone::Adapters::Simulation::GazeboMeasurementClock>
            measurementClock);
    ~GazeboStereoCamera() override;

    bool Open(const SmartDrone::Core::Ports::CameraOpenConfig &config) override;
    void Close() override;
    bool Start() override;
    void Stop() override;
    bool GrabStereo(SmartDrone::Core::Ports::StereoFrame &out,
                    bool preferLatest,
                    std::uint64_t minTimestampNs = 0) override;
    SmartDrone::Core::Ports::CameraHealth GetHealth() const override;
    SmartDrone::Core::Ports::CameraDiagnostics GetDiagnostics() const override;
    SmartDrone::Core::Ports::CameraProviderSemantics Semantics() const override;
    bool SetFrameReadyCallback(
        SmartDrone::Core::Ports::CameraFrameReadyCallback callback) override;

  private:
    struct Impl;

    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Adapters::Camera
