#pragma once

#include "core/ports/camera_provider.hpp"
#include "adapters/camera/libcamera_ov9281/stereo_ov9281.hpp"

namespace smartdrone::adapters::camera {

class LibcameraStereoCamera final : public core::ports::ICameraProvider {
public:
    explicit LibcameraStereoCamera(LibcameraStereoOV9281_TsPair& impl) : m_impl(impl) {}

    bool Start() override { return true; }
    void Stop() override {}

    bool GrabStereo(core::ports::StereoFrame& out, int timeoutMs, bool preferLatest) override
    {
        FrameItem left;
        FrameItem right;
        if (!m_impl.GrabPair(left, right, timeoutMs, preferLatest)) {
            return false;
        }
        out.left.cameraId = left.camIndex;
        out.left.timestampNs = left.tsNs;
        out.left.sequence = left.seq;
        out.left.gray = left.gray;
        out.left.owner = left.owner;
        out.right.cameraId = right.camIndex;
        out.right.timestampNs = right.tsNs;
        out.right.sequence = right.seq;
        out.right.gray = right.gray;
        out.right.owner = right.owner;
        return true;
    }

    core::ports::CameraHealth GetHealth() const override
    {
        core::ports::CameraHealth health{};
        health.healthy = m_impl.Healthy();
        health.droppedPairs = m_impl.DroppedPaired();
        return health;
    }

private:
    LibcameraStereoOV9281_TsPair& m_impl;
};

}  // namespace smartdrone::adapters::camera
