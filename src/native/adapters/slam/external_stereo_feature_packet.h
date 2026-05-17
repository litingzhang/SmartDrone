#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "adapters/slam/external_feature_types.h"

namespace smartdrone::adapters::slam {

struct ExternalStereoFeaturePacketBuildInput {
    const cv::Mat *leftPrepared{nullptr};
    const cv::Mat *rightPrepared{nullptr};
    const std::vector<cv::Point2f> *matchedLeftPoints{nullptr};
    const std::vector<cv::Point2f> *matchedRightPoints{nullptr};
    const std::vector<StereoMatchPair> *filteredMatches{nullptr};
    const std::vector<StereoMatchPair> *rawMatches{nullptr};
    const ExternalFeatureSet *leftFeatures{nullptr};
    const ExternalFeatureSet *rightFeatures{nullptr};
    const ExternalDescriptorProvider *leftDescriptorProvider{nullptr};
    const ExternalDescriptorProvider *rightDescriptorProvider{nullptr};
    bool initializedForMonoAugmentation{false};
    bool allowNativeDescriptorInject{true};
    bool allowAllLeftGeometricDepth{true};
    int stableOkStreak{0};
};

struct ExternalStereoFeaturePacket {
    ExternalStereoObservationPacket observations;
    std::vector<cv::Point2f> leftFeaturePoints;
    std::vector<cv::Point2f> rightFeaturePoints;
    uint64_t hash{0};
    bool packed{false};
    size_t orbStereoAugmentPairs{0};
    double monoAugmentMs{0.0};
};

bool BuildExternalStereoFeaturePacket(const ExternalStereoFeaturePacketBuildInput &input,
                                      ExternalStereoFeaturePacket &packet);
uint64_t HashExternalStereoData(const ExternalStereoObservationPacket &data);

} // namespace smartdrone::adapters::slam
