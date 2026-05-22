#pragma once

#include "core/ports/camera_provider.h"

#include "adapters/camera/libcamera_ov9281/stereo_ov9281.h"

namespace SmartDrone::Adapters::Camera {

class LibcameraStereoCamera final : public Core::Ports::ICameraProvider {
  public:
    LibcameraStereoCamera() = default;

    bool Open(const Core::Ports::CameraOpenConfig &config) override;
    void Close() override;
    bool Start() override;
    void Stop() override;
    bool GrabStereo(Core::Ports::StereoFrame &out, int timeoutMs, bool preferLatest,
                    uint64_t minTimestampNs = 0) override;
    Core::Ports::CameraHealth GetHealth() const override;
    Core::Ports::CameraDiagnostics GetDiagnostics() const override;
    Core::Ports::CameraProviderSemantics Semantics() const override;

  private:
    LibcameraStereoOV9281_TsPair m_impl;
};

} // namespace SmartDrone::Adapters::Camera
