#include "core/application/session/slam/slam_preview_output_port.h"

#include <cstdint>

#include "adapters/stream/udp_image_sender.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/session/calib/calib_image_utils.h"
#include "core/application/session/epg/messages/calib_epg_messages.h"

namespace smartdrone::core::application {

namespace {

class UdpSlamPreviewOutputPort final : public ISlamPreviewOutputPort {
  public:
    explicit UdpSlamPreviewOutputPort(UdpImageSender &udpSender)
        : m_udpSender(udpSender)
    {
    }

    void Enqueue(const SlamPreviewOutputFrame &frame) override
    {
        m_udpSender.Enqueue(frame.camIndex, frame.frameId, frame.sequence,
                            frame.frameTime, frame.gray, frame.trackedPoints,
                            frame.sendImage, frame.sendFeature);
    }

    void StepAll() override
    {
        m_udpSender.StepAll();
    }

  private:
    UdpImageSender &m_udpSender;
};

} // namespace

class SlamPreviewOutputRuntime::Impl final {
  public:
    bool Open(const MainRuntimeAliases &aliases,
              DestinationResolver destinationResolver)
    {
        return m_udp.Open(aliases.udpIp, aliases.udpPort, aliases.udpJpegQ,
                          aliases.udpPayload, aliases.udpQueue,
                          std::move(destinationResolver));
    }

    bool OpenStaticPeer(const MainRuntimeAliases &aliases)
    {
        return m_udp.Open(aliases.udpIp, aliases.udpPort, aliases.udpJpegQ,
                          aliases.udpPayload, aliases.udpQueue);
    }

    void Close()
    {
        m_udp.Close();
    }

    void EnqueueCalibStereoFrame(const CalibStereoFrame &frame)
    {
        const auto &left = frame.stereo.left;
        const auto &right = frame.stereo.right;
        const std::int64_t pairNs =
            static_cast<std::int64_t>((left.timestampNs + right.timestampNs) /
                                      2);
        bool convertedLeft = false;
        bool convertedRight = false;
        const cv::Mat leftGray = EnsureCalibGray8(left.gray, convertedLeft);
        const cv::Mat rightGray = EnsureCalibGray8(right.gray, convertedRight);
        m_udp.Enqueue(0, static_cast<std::uint64_t>(left.sequence),
                      left.sequence, pairNs * 1e-9, leftGray, {}, true,
                      false);
        m_udp.Enqueue(1, static_cast<std::uint64_t>(right.sequence),
                      right.sequence, pairNs * 1e-9, rightGray, {}, true,
                      false);
        m_udp.StepAll();
    }

    ISlamPreviewOutputPort &OutputPort()
    {
        return m_outputPort;
    }

  private:
    UdpImageSender m_udp;
    UdpSlamPreviewOutputPort m_outputPort{m_udp};
};

SlamPreviewOutputRuntime::SlamPreviewOutputRuntime()
    : m_impl(new Impl())
{
}

SlamPreviewOutputRuntime::~SlamPreviewOutputRuntime() = default;

bool SlamPreviewOutputRuntime::Open(
    const MainRuntimeAliases &aliases, DestinationResolver destinationResolver)
{
    return m_impl->Open(aliases, std::move(destinationResolver));
}

bool SlamPreviewOutputRuntime::OpenStaticPeer(const MainRuntimeAliases &aliases)
{
    return m_impl->OpenStaticPeer(aliases);
}

void SlamPreviewOutputRuntime::Close()
{
    m_impl->Close();
}

void SlamPreviewOutputRuntime::EnqueueCalibStereoFrame(
    const CalibStereoFrame &frame)
{
    m_impl->EnqueueCalibStereoFrame(frame);
}

ISlamPreviewOutputPort &SlamPreviewOutputRuntime::OutputPort()
{
    return m_impl->OutputPort();
}

} // namespace smartdrone::core::application
