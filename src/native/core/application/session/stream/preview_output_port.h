#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <netinet/in.h>

#include <opencv2/core.hpp>

namespace SmartDrone::Core::Application {

struct CalibStereoFrame;
struct MainRuntimeAliases;

struct PreviewOutputFrame {
    int camIndex;
    std::uint64_t frameId;
    std::uint32_t sequence;
    double frameTime;
    const cv::Mat &gray;
    const std::vector<cv::Point2f> &trackedPoints;
    bool sendImage;
    bool sendFeature;
};

struct PreviewOutputOpenConfig {
    std::string ip;
    int port{0};
    int jpegQuality{80};
    int maxPayload{1200};
    int maxQueue{4};
};

class IPreviewOutputPort {
  public:
    virtual ~IPreviewOutputPort() = default;

    virtual void Enqueue(const PreviewOutputFrame &frame) = 0;
    virtual void StepOnce() = 0;
    virtual void StepAll() = 0;
};

class IPreviewOutputRuntime {
  public:
    using DestinationResolver = std::function<bool(sockaddr_in &)>;

    virtual ~IPreviewOutputRuntime() = default;

    virtual bool Open(const PreviewOutputOpenConfig &config,
                      DestinationResolver destinationResolver) = 0;
    virtual bool OpenStaticPeer(const PreviewOutputOpenConfig &config) = 0;
    virtual void Close() = 0;
    virtual void EnqueueCalibStereoFrame(const CalibStereoFrame &frame) = 0;
    virtual IPreviewOutputPort &OutputPort() = 0;
};

PreviewOutputOpenConfig MakePreviewOutputOpenConfig(
    const MainRuntimeAliases &aliases);

} // namespace SmartDrone::Core::Application
