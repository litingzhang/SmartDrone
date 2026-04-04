#include "adapters/camera/libcamera_stereo_camera.h"

namespace smartdrone::adapters::camera {

LibcameraStereoCamera::LibcameraStereoCamera(LibcameraStereoOV9281_TsPair& impl) : m_impl(impl)
{
}

bool LibcameraStereoCamera::Start()
{
    return true;
}

void LibcameraStereoCamera::Stop()
{
}

bool LibcameraStereoCamera::GrabStereo(
    core::ports::StereoFrame& out,
    int timeoutMs,
    bool preferLatest,
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

int64_t LibcameraStereoCamera::LastPairDtMs() const
{
    return m_impl.LastDtMs();
}

int64_t LibcameraStereoCamera::LastRejectDtUs() const
{
    return m_impl.LastRejectDtUs();
}

uint32_t LibcameraStereoCamera::LastRawSeqL() const
{
    return m_impl.LastRawSeqL();
}

uint32_t LibcameraStereoCamera::LastRawSeqR() const
{
    return m_impl.LastRawSeqR();
}

uint64_t LibcameraStereoCamera::RawCountL() const
{
    return m_impl.RawCountL();
}

uint64_t LibcameraStereoCamera::RawCountR() const
{
    return m_impl.RawCountR();
}

uint64_t LibcameraStereoCamera::DroppedUnpairedL() const
{
    return m_impl.DroppedUnpairedL();
}

uint64_t LibcameraStereoCamera::DroppedUnpairedR() const
{
    return m_impl.DroppedUnpairedR();
}

size_t LibcameraStereoCamera::PendingL() const
{
    return m_impl.PendL();
}

size_t LibcameraStereoCamera::PendingR() const
{
    return m_impl.PendR();
}

int64_t LibcameraStereoCamera::PairTolNs() const
{
    return m_impl.PairTolNs();
}

}  // namespace smartdrone::adapters::camera
