#include "core/application/session/calib/calib_camera_input_port.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <utility>

#include "core/application/config/runtime_app_types.h"
#include "core/application/sensors/camera_runtime_provider.h"
#include "core/ports/camera_provider.h"

namespace smartdrone::core::application {
namespace {

constexpr std::int64_t NANOSECONDS_PER_MILLISECOND = 1000000LL;

std::int64_t MaxPairDeltaNs(const MainRuntimeAliases &aliases)
{
    return static_cast<std::int64_t>(std::max(aliases.pairMs, 1)) *
           NANOSECONDS_PER_MILLISECOND;
}

} // namespace

class CalibCameraInputPort::Impl final {
  public:
    explicit Impl(MainRuntimeAliases aliases)
        : m_aliases(std::move(aliases)),
          m_maxPairDeltaNs(MaxPairDeltaNs(m_aliases))
    {
    }

    ~Impl()
    {
        Stop();
    }

    bool Open()
    {
        if (m_opened) {
            return true;
        }
        m_cameraProvider = CreateCameraProvider();
        m_opened = m_cameraProvider && m_cameraProvider->Open(m_aliases);
        return m_opened;
    }

    CalibFrameCaptureResult TryCaptureFrame(CalibStereoFrame &frame)
    {
        if (!m_cameraProvider || !m_opened) {
            return {CalibFrameCaptureStatus::SessionAbort};
        }
        if (!m_cameraProvider->GrabStereo(frame.stereo, 0, true)) {
            return HandleCameraGrabFailure();
        }
        if (!AcceptFrameTiming(frame)) {
            return {CalibFrameCaptureStatus::Pending};
        }
        return {CalibFrameCaptureStatus::Captured};
    }

    void Stop()
    {
        if (!m_cameraProvider) {
            m_opened = false;
            return;
        }
        m_cameraProvider->Stop();
        if (m_opened) {
            std::cerr << "[session] calib camera closed\n";
        }
        m_cameraProvider.reset();
        m_opened = false;
    }

    bool Opened() const
    {
        return m_opened;
    }

  private:
    CalibFrameCaptureResult HandleCameraGrabFailure() const
    {
        if (m_cameraProvider && m_cameraProvider->GetHealth().healthy) {
            return {CalibFrameCaptureStatus::Pending};
        }
        std::cerr << "[calib] camera pipeline unhealthy, aborting session\n";
        return {CalibFrameCaptureStatus::SessionAbort};
    }

    bool AcceptFrameTiming(const CalibStereoFrame &frame)
    {
        if (m_cameraProvider->Semantics() !=
            smartdrone::core::ports::CameraProviderSemantics::DualStreamPaired) {
            return true;
        }
        const std::int64_t absDtLr = FrameTimestampDeltaNs(frame);
        if (absDtLr <= m_maxPairDeltaNs) {
            return true;
        }
        LogWidePairDrop(absDtLr);
        return false;
    }

    static std::int64_t FrameTimestampDeltaNs(const CalibStereoFrame &frame)
    {
        const auto &left = frame.stereo.left;
        const auto &right = frame.stereo.right;
        return std::llabs(static_cast<std::int64_t>(left.timestampNs) -
                          static_cast<std::int64_t>(right.timestampNs));
    }

    void LogWidePairDrop(std::int64_t absDtLr)
    {
        ++m_droppedWide;
        if ((m_droppedWide % 10) != 1) {
            return;
        }
        std::cerr << "[calib-drop] dt_lr_us=" << (absDtLr / 1000.0)
                  << " exceeds max_save_dt_us="
                  << (m_maxPairDeltaNs / 1000.0)
                  << " dropped=" << m_droppedWide << "\n";
    }

    MainRuntimeAliases m_aliases{};
    std::unique_ptr<smartdrone::core::ports::ICameraProvider>
        m_cameraProvider;
    std::int64_t m_maxPairDeltaNs{0};
    int m_droppedWide{0};
    bool m_opened{false};
};

CalibCameraInputPort::CalibCameraInputPort(
    const MainRuntimeAliases &aliases)
    : m_impl(new Impl(aliases))
{
}

CalibCameraInputPort::~CalibCameraInputPort() = default;

bool CalibCameraInputPort::Open()
{
    return m_impl->Open();
}

CalibFrameCaptureResult CalibCameraInputPort::TryCaptureFrame(
    CalibStereoFrame &frame)
{
    return m_impl->TryCaptureFrame(frame);
}

void CalibCameraInputPort::Stop()
{
    m_impl->Stop();
}

bool CalibCameraInputPort::Opened() const
{
    return m_impl->Opened();
}

} // namespace smartdrone::core::application
