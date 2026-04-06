#pragma once

#include "adapters/camera/libcamera_ov9281/stereo_ov9281.h"
#include "core/ports/camera_provider.h"

namespace smartdrone::adapters::camera {

class LibcameraStereoCamera final : public core::ports::ICameraProvider {
  public:
    explicit LibcameraStereoCamera(LibcameraStereoOV9281_TsPair &impl);

    bool Start() override;
    void Stop() override;
    bool GrabStereo(core::ports::StereoFrame &out, int timeoutMs, bool preferLatest,
                    uint64_t minTimestampNs = 0) override;
    core::ports::CameraHealth GetHealth() const override;
    core::ports::CameraDiagnostics GetDiagnostics() const override;

    int64_t LastPairDtMs() const;
    int64_t LastRejectDtUs() const;
    uint32_t LastRawSeqL() const;
    uint32_t LastRawSeqR() const;
    uint64_t RawCountL() const;
    uint64_t RawCountR() const;
    uint64_t DroppedUnpairedL() const;
    uint64_t DroppedUnpairedR() const;
    size_t PendingL() const;
    size_t PendingR() const;
    int64_t PairTolNs() const;

  private:
    LibcameraStereoOV9281_TsPair &m_impl;
};

} // namespace smartdrone::adapters::camera
