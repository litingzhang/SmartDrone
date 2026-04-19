#include "adapters/camera/libcamera_stereo_camera.h"

namespace smartdrone::adapters::camera {

bool LibcameraStereoCamera::Open(const smartdrone::core::application::MainRuntimeAliases &aliases)
{
    return m_impl.Open(aliases.width, aliases.height, aliases.fps, aliases.aeDisable, aliases.exposureUs, aliases.gain,
                       aliases.requestY8, static_cast<int64_t>(aliases.pairMs) * 1000000LL,
                       static_cast<int64_t>(aliases.keepMs) * 1000000LL, aliases.pairQueue, aliases.r16Norm,
                       aliases.leftCamIndex, aliases.rightCamIndex);
}

void LibcameraStereoCamera::Close() { m_impl.Close(); }

bool LibcameraStereoCamera::Start() { return true; }

void LibcameraStereoCamera::Stop() { Close(); }

bool LibcameraStereoCamera::GrabStereo(core::ports::StereoFrame &out, int timeoutMs, bool preferLatest,
                                       uint64_t minTimestampNs)
{
    FrameItem left;
    FrameItem right;
    if (!m_impl.GrabPair(left, right, timeoutMs, preferLatest, minTimestampNs)) {
        return false;
    }
    out.left.cameraId = left.camIndex;
    out.left.timestampNs = left.tsNs;
    out.left.arriveNs = left.arriveNs;
    out.left.sequence = left.seq;
    out.left.gray = left.gray;
    out.left.owner = left.owner;
    out.right.cameraId = right.camIndex;
    out.right.timestampNs = right.tsNs;
    out.right.arriveNs = right.arriveNs;
    out.right.sequence = right.seq;
    out.right.gray = right.gray;
    out.right.owner = right.owner;
    return true;
}

core::ports::CameraHealth LibcameraStereoCamera::GetHealth() const
{
    core::ports::CameraHealth health{};
    health.healthy = m_impl.Healthy();
    health.droppedPairs = m_impl.DroppedPaired();
    return health;
}

core::ports::CameraDiagnostics LibcameraStereoCamera::GetDiagnostics() const
{
    const auto diag = m_impl.GetDiagnostics();
    core::ports::CameraDiagnostics out{};
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

core::ports::CameraProviderSemantics LibcameraStereoCamera::Semantics() const
{
    return core::ports::CameraProviderSemantics::DualStreamPaired;
}

} // namespace smartdrone::adapters::camera
