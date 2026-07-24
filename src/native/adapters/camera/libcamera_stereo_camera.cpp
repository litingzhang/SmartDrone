#include "adapters/camera/libcamera_stereo_camera.h"

namespace SmartDrone::Adapters::Camera {

bool LibcameraStereoCamera::Open(const Core::Ports::CameraOpenConfig &config)
{
    const StereoCameraOpenParams params{
        config.width,
        config.height,
        config.fps,
        config.autoExposureDisabled,
        config.exposureUs,
        config.gain,
        config.requestY8,
        static_cast<int64_t>(config.pairWindowMs) * 1000000LL,
        static_cast<int64_t>(config.keepWindowMs) * 1000000LL,
        config.pairQueue,
        config.r16Normalize,
        config.leftCameraIndex,
        config.rightCameraIndex,
    };
    return m_impl.Open(params);
}

void LibcameraStereoCamera::Close()
{
    m_impl.Close();
}

bool LibcameraStereoCamera::Start()
{
    return true;
}

void LibcameraStereoCamera::Stop()
{
    Close();
}

bool LibcameraStereoCamera::GrabStereo(Core::Ports::StereoFrame &out, bool preferLatest,
                                       uint64_t minTimestampNs)
{
    FrameItem left;
    FrameItem right;
    if (!m_impl.GrabPair(left, right, preferLatest, minTimestampNs)) {
        return false;
    }
    out.left.cameraId = left.camIndex;
    out.left.timestampNs = left.tsNs;
    out.left.captureMonotonicNs = static_cast<int64_t>(left.tsNs);
    out.left.arriveNs = left.arriveNs;
    out.left.sequence = left.seq;
    out.left.gray = left.gray;
    out.left.owner = left.owner;
    out.right.cameraId = right.camIndex;
    out.right.timestampNs = right.tsNs;
    out.right.captureMonotonicNs = static_cast<int64_t>(right.tsNs);
    out.right.arriveNs = right.arriveNs;
    out.right.sequence = right.seq;
    out.right.gray = right.gray;
    out.right.owner = right.owner;
    return true;
}

Core::Ports::CameraHealth LibcameraStereoCamera::GetHealth() const
{
    Core::Ports::CameraHealth health{};
    health.healthy = m_impl.Healthy();
    health.droppedPairs = m_impl.DroppedPaired();
    return health;
}

Core::Ports::CameraDiagnostics LibcameraStereoCamera::GetDiagnostics() const
{
    const auto diag = m_impl.GetDiagnostics();
    Core::Ports::CameraDiagnostics out{};
    out.healthy = diag.healthy;
    out.acceptFrames = diag.acceptFrames;
    out.lastRawSeqL = diag.lastRawSeqL;
    out.lastRawSeqR = diag.lastRawSeqR;
    out.rawCountL = diag.rawCountL;
    out.rawCountR = diag.rawCountR;
    out.droppedPairs = diag.droppedPaired;
    out.droppedUnpairedL = diag.droppedUnpairedL;
    out.droppedUnpairedR = diag.droppedUnpairedR;
    out.pendingL = diag.pendingL;
    out.pendingR = diag.pendingR;
    out.pairedQueue = diag.pairedQueue;
    out.pairTolNs = diag.pairTolNs;
    out.lastPairDtMs = diag.lastPairDtMs;
    out.lastRejectDtUs = diag.lastRejectDtUs;
    out.lastFrameAgeMsL = diag.lastFrameAgeMsL;
    out.lastFrameAgeMsR = diag.lastFrameAgeMsR;
    out.lastPairAgeMs = diag.lastPairAgeMs;
    return out;
}

Core::Ports::CameraProviderSemantics LibcameraStereoCamera::Semantics() const
{
    return Core::Ports::CameraProviderSemantics::DualStreamPaired;
}

} // namespace SmartDrone::Adapters::Camera
