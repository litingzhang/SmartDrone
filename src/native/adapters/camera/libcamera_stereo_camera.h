#pragma once

#include "core/ports/camera_provider.h"

#include "adapters/camera/libcamera_ov9281/stereo_ov9281.h"

namespace SmartDrone::adapters::camera {

class LibcameraStereoCamera final : public core::ports::ICameraProvider {
  public:
    LibcameraStereoCamera() = default;

    bool Open(const core::ports::CameraOpenConfig &config) override;
    void Close() override;
    bool Start() override;
    void Stop() override;
    bool GrabStereo(core::ports::StereoFrame &out, int timeoutMs, bool preferLatest,
                    uint64_t minTimestampNs = 0) override;
    core::ports::CameraHealth GetHealth() const override;
    core::ports::CameraDiagnostics GetDiagnostics() const override;
    core::ports::CameraProviderSemantics Semantics() const override;

  private:
    LibcameraStereoOV9281_TsPair m_impl;
};

} // namespace SmartDrone::adapters::camera
