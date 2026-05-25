#pragma once

#include <cstdint>
#include <deque>
#include <vector>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "core/ports/slam_engine.h"

namespace SmartDrone::Core::Ports {

struct ForwardBackwardTrackingOptions {
    int windowSizePx{21};
    int maxLevel{3};
    float maxForwardBackwardErrorPx{1.5f};
};

struct ForwardBackwardTrackingRequest {
    const cv::Mat &prevGray;
    const cv::Mat &currGray;
    const std::vector<cv::Point2f> &prevPoints;
    std::vector<cv::Point2f> &currPoints;
    std::vector<uchar> &status;
    ForwardBackwardTrackingOptions options{};
};

class IPointTracker2d {
  public:
    virtual ~IPointTracker2d() = default;

    virtual bool TrackForwardBackward(
        const ForwardBackwardTrackingRequest &request) const = 0;
};

struct StereoTrack {
    cv::Point2f left;
    cv::Point2f right;
    float quality{0.0f};
    uint32_t age{0};
};

struct LoopKeyframe {
    uint64_t frameId{0};
    Sophus::SE3f rawTwc{Sophus::SE3f()};
    Sophus::SE3f correctedTwc{Sophus::SE3f()};
    cv::Mat descriptor;
};

struct LoopClosureState {
    bool enabled{false};
    float scale{1.20f};
    float relaxation{1.40f};
    std::deque<LoopKeyframe> keyframes;
    Sophus::SE3f correction{Sophus::SE3f()};
    uint64_t lastClosureFrameId{0};
};

class IVisualLoopClosureBackend {
  public:
    virtual ~IVisualLoopClosureBackend() = default;

    virtual void Reset(LoopClosureState &state) const = 0;
    virtual Sophus::SE3f Apply(LoopClosureState &state, const cv::Mat &leftRect,
                               uint64_t frameId,
                               const Sophus::SE3f &rawTwc) const = 0;
};

struct PnpObservationSet {
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    int rawCandidateCount{0};
};

struct PerFramePnpObservationBuilderOptions {
    const cv::Mat *disparity{nullptr};
    cv::Size previousImageSize{};
    cv::Size currentImageSize{};
    const std::vector<cv::Point2f> *previousPoints{nullptr};
    const std::vector<cv::Point2f> *currentPoints{nullptr};
    const std::vector<uint8_t> *status{nullptr};
    const std::vector<cv::Point2f> *backwardPoints{nullptr};
    const std::vector<uint8_t> *backwardStatus{nullptr};
    bool useForwardBackwardCheck{false};
    bool useDepthBalancedSelection{true};
    float fx{0.0f};
    float fy{0.0f};
    float cx{0.0f};
    float cy{0.0f};
    float baseline{0.0f};
};

struct TrackedStereoPnpObservationBuilderOptions {
    const std::vector<StereoTrack> *previousTracks{nullptr};
    const std::vector<cv::Point2f> *currentLeftPoints{nullptr};
    const std::vector<cv::Point2f> *currentRightPoints{nullptr};
    const std::vector<uint8_t> *leftStatus{nullptr};
    const std::vector<uint8_t> *rightStatus{nullptr};
    cv::Size previousImageSize{};
    const cv::Mat *currentLeftImage{nullptr};
    const cv::Mat *currentRightImage{nullptr};
    float fx{0.0f};
    float fy{0.0f};
    float cx{0.0f};
    float cy{0.0f};
    float baseline{0.0f};
};

struct TrackedStereoPnpObservationSet {
    PnpObservationSet pnp;
    std::vector<StereoTrack> trackedTracks;
};

class IVisualPnpObservationBuilder {
  public:
    virtual ~IVisualPnpObservationBuilder() = default;

    virtual PnpObservationSet BuildPerFrameObservations(
        const PerFramePnpObservationBuilderOptions &options) const = 0;
    virtual TrackedStereoPnpObservationSet BuildTrackedStereoObservations(
        const TrackedStereoPnpObservationBuilderOptions &options) const = 0;
};

void CopyStereoTracksToOutput(const std::vector<StereoTrack> &tracks,
                              SlamOutput &out);

} // namespace SmartDrone::Core::Ports
