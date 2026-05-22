#pragma once

#include <limits>
#include <vector>

#include <opencv2/core.hpp>

namespace SmartDrone::core::ports {

struct VisualFeatureSet {
    std::vector<cv::Point2f> keypoints;
    cv::Mat descriptors;
};

struct VisualKeypointFeatureSet {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
};

struct StereoFeatureObservationPacket {
    std::vector<cv::KeyPoint> leftKeypoints;
    std::vector<cv::KeyPoint> rightKeypoints;
    cv::Mat leftDescriptors;
    cv::Mat rightDescriptors;
    std::vector<int> leftToRightMatch;
    bool matchedStereoPairs{false};
};

struct StereoMatchPair {
    int leftIndex{-1};
    int rightIndex{-1};
    float descriptorScore{-std::numeric_limits<float>::infinity()};
    float zncc{-1.0f};
    float disparity{0.0f};
    float quality{0.0f};
};

} // namespace SmartDrone::core::ports
