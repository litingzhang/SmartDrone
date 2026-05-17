#pragma once

#include <limits>
#include <vector>

#include <opencv2/core.hpp>

namespace smartdrone::adapters::slam {

struct ExternalFeatureSet {
    std::vector<cv::Point2f> keypoints;
    cv::Mat descriptors;
};

class ExternalDescriptorProvider {
  public:
    virtual ~ExternalDescriptorProvider() = default;

    virtual bool ComputeDescriptorsAtPoints(const cv::Mat &gray,
                                            const std::vector<cv::Point2f> &points,
                                            std::vector<cv::KeyPoint> &keypoints,
                                            cv::Mat &descriptors) const = 0;
    virtual bool DetectAndCompute(const cv::Mat &gray,
                                  std::vector<cv::KeyPoint> &keypoints,
                                  cv::Mat &descriptors) const = 0;
    virtual int DescriptorDistance(const cv::Mat &leftDescriptor,
                                   const cv::Mat &rightDescriptor) const = 0;
};

struct ExternalStereoObservationPacket {
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

} // namespace smartdrone::adapters::slam
