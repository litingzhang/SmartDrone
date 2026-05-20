#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include <netinet/in.h>

#include <opencv2/core.hpp>

namespace smartdrone::core::application {

struct CalibStereoFrame;
struct MainRuntimeAliases;

struct SlamPreviewOutputFrame {
    int camIndex;
    std::uint64_t frameId;
    std::uint32_t sequence;
    double frameTime;
    const cv::Mat &gray;
    const std::vector<cv::Point2f> &trackedPoints;
    bool sendImage;
    bool sendFeature;
};

class ISlamPreviewOutputPort {
  public:
    virtual ~ISlamPreviewOutputPort() = default;

    virtual void Enqueue(const SlamPreviewOutputFrame &frame) = 0;
    virtual void StepAll() = 0;
};

class SlamPreviewOutputRuntime final {
  public:
    using DestinationResolver = std::function<bool(sockaddr_in &)>;

    SlamPreviewOutputRuntime();
    ~SlamPreviewOutputRuntime();

    bool Open(const MainRuntimeAliases &aliases,
              DestinationResolver destinationResolver);
    bool OpenStaticPeer(const MainRuntimeAliases &aliases);
    void Close();
    void EnqueueCalibStereoFrame(const CalibStereoFrame &frame);
    ISlamPreviewOutputPort &OutputPort();

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace smartdrone::core::application
