#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

#include "adapters/slam/external_feature_types.h"

namespace smartdrone::adapters::slam {

struct ExternalStereoMatchSelectionInput {
    const ExternalFeatureSet *leftFeatures{nullptr};
    const ExternalFeatureSet *rightFeatures{nullptr};
    const cv::Mat *leftPrepared{nullptr};
    const cv::Mat *rightPrepared{nullptr};
    bool initializing{false};
    bool recovering{false};
    bool trustFrontendInitPairs{false};
    bool trustFrontendRecoveryPairs{false};
    bool trustFrontendBootstrapPairs{false};
    bool previousFrameWeak{false};
};

struct ExternalStereoMatchSelection {
    size_t pairedFeatureCount{0};
    bool trustFrontendPairs{false};
    bool initializationTrustedPairSelection{false};
    bool initializationStereoBias{false};
    std::vector<StereoMatchPair> initializationTrustedMatches;
    std::vector<StereoMatchPair> rawMatches;
    std::vector<StereoMatchPair> filteredMatches;
    std::vector<cv::Point2f> matchedLeftPoints;
    std::vector<cv::Point2f> matchedRightPoints;
};

void CopyMatchedStereoPointsFromPairs(const ExternalFeatureSet &leftFeatures,
                                      const ExternalFeatureSet &rightFeatures,
                                      const std::vector<StereoMatchPair> &matches,
                                      std::vector<cv::Point2f> &leftPoints,
                                      std::vector<cv::Point2f> &rightPoints);

bool SelectExternalStereoMatches(const ExternalStereoMatchSelectionInput &input,
                                 ExternalStereoMatchSelection &selection);

} // namespace smartdrone::adapters::slam
