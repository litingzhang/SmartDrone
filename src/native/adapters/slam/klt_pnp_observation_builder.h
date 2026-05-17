#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "adapters/slam/slam_mode_state.h"

namespace smartdrone::adapters::slam {

struct KltPerFramePnpObservationBuilderOptions {
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

struct KltPnpObservationSet {
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    int rawCandidateCount{0};
};

struct KltTrackedStereoPnpObservationBuilderOptions {
    const std::vector<LkStereoTrack> *previousTracks{nullptr};
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

struct KltTrackedStereoPnpObservationSet {
    KltPnpObservationSet pnp;
    std::vector<LkStereoTrack> trackedTracks;
};

KltPnpObservationSet BuildKltPerFramePnpObservations(
    const KltPerFramePnpObservationBuilderOptions &options);

KltTrackedStereoPnpObservationSet BuildKltTrackedStereoPnpObservations(
    const KltTrackedStereoPnpObservationBuilderOptions &options);

} // namespace smartdrone::adapters::slam
