#include "adapters/slam/external_stereo_matching.h"

#include <algorithm>
#include <cmath>

#include "adapters/slam/external_stereo_pair_builder.h"
#include "adapters/slam/slam_env.h"
#include "adapters/slam/stereo_geometry.h"

namespace smartdrone::adapters::slam {

namespace {

bool HasStereoFeatureInput(const ExternalStereoMatchSelectionInput &input)
{
    return input.leftFeatures != nullptr && input.rightFeatures != nullptr &&
           input.leftPrepared != nullptr && input.rightPrepared != nullptr &&
           !input.leftPrepared->empty() && !input.rightPrepared->empty();
}

void AppendAlignedFrontendPairs(const ExternalFeatureSet &leftFeatures,
                                const ExternalFeatureSet &rightFeatures,
                                size_t pairedFeatureCount,
                                std::vector<cv::Point2f> &leftPoints,
                                std::vector<cv::Point2f> &rightPoints)
{
    leftPoints.clear();
    rightPoints.clear();
    leftPoints.reserve(pairedFeatureCount);
    rightPoints.reserve(pairedFeatureCount);
    for (size_t i = 0; i < pairedFeatureCount; ++i) {
        leftPoints.push_back(leftFeatures.keypoints[i]);
        rightPoints.push_back(rightFeatures.keypoints[i]);
    }
}

std::vector<StereoMatchPair> SelectInitializationTrustedPairs(const ExternalFeatureSet &leftFeatures,
                                                              const ExternalFeatureSet &rightFeatures,
                                                              const cv::Mat &leftPrepared,
                                                              const cv::Mat &rightPrepared)
{
    std::vector<StereoMatchPair> matches =
        BuildAlignedStereoPairs(leftFeatures, rightFeatures, leftPrepared, rightPrepared);
    if (matches.empty()) {
        return matches;
    }

    const float closeDisparity =
        EnvFloatValue("SMART_DRONE_SP_LG_INIT_CLOSE_DISPARITY", 4.0f);
    std::stable_sort(matches.begin(), matches.end(),
                     [closeDisparity](const StereoMatchPair &lhs, const StereoMatchPair &rhs) {
                         const bool lhsClose = lhs.disparity >= closeDisparity;
                         const bool rhsClose = rhs.disparity >= closeDisparity;
                         if (lhsClose != rhsClose) {
                             return lhsClose;
                         }
                         if (std::abs(lhs.quality - rhs.quality) > 1.0e-6f) {
                             return lhs.quality > rhs.quality;
                         }
                         if (std::abs(lhs.zncc - rhs.zncc) > 1.0e-6f) {
                             return lhs.zncc > rhs.zncc;
                         }
                         return lhs.disparity > rhs.disparity;
                     });

    const size_t maxPairs = EnvSizeValueClamped("SMART_DRONE_SP_LG_INIT_TRUST_MAX_PAIRS", 256, 48, 1200);
    if (matches.size() > maxPairs) {
        matches.resize(maxPairs);
    }
    return matches;
}

void BiasInitializationMatches(std::vector<StereoMatchPair> &matches)
{
    if (matches.empty()) {
        return;
    }
    const float closeDisparity =
        EnvFloatValue("SMART_DRONE_SP_LG_INIT_CLOSE_DISPARITY", 4.0f);
    std::stable_sort(matches.begin(), matches.end(),
                     [closeDisparity](const StereoMatchPair &lhs, const StereoMatchPair &rhs) {
                         const bool lhsClose = lhs.disparity > closeDisparity;
                         const bool rhsClose = rhs.disparity > closeDisparity;
                         if (lhsClose != rhsClose) {
                             return lhsClose;
                         }
                         if (std::abs(lhs.quality - rhs.quality) > 1.0e-6f) {
                             return lhs.quality > rhs.quality;
                         }
                         if (std::abs(lhs.zncc - rhs.zncc) > 1.0e-6f) {
                             return lhs.zncc > rhs.zncc;
                         }
                         return lhs.disparity > rhs.disparity;
                     });
}

void LimitWeakFramePairs(std::vector<cv::Point2f> &leftPoints,
                         std::vector<cv::Point2f> &rightPoints,
                         const ExternalStereoMatchSelectionInput &input)
{
    if (input.initializing || input.recovering ||
        !EnvFlagEnabled("SMART_DRONE_SP_LG_WEAK_FRAME_PAIR_LIMIT", false) ||
        !input.previousFrameWeak) {
        return;
    }

    const size_t maxPairs = EnvSizeValueClamped("SMART_DRONE_SP_LG_WEAK_FRAME_MAX_PAIRS", 192, 48, 1200);
    LimitStereoPairsInPlace(leftPoints, rightPoints, maxPairs);
}

} // namespace

void CopyMatchedStereoPointsFromPairs(const ExternalFeatureSet &leftFeatures,
                                      const ExternalFeatureSet &rightFeatures,
                                      const std::vector<StereoMatchPair> &matches,
                                      std::vector<cv::Point2f> &leftPoints,
                                      std::vector<cv::Point2f> &rightPoints)
{
    leftPoints.clear();
    rightPoints.clear();
    leftPoints.reserve(matches.size());
    rightPoints.reserve(matches.size());
    for (const StereoMatchPair &match : matches) {
        if (match.leftIndex < 0 || match.rightIndex < 0 ||
            static_cast<size_t>(match.leftIndex) >= leftFeatures.keypoints.size() ||
            static_cast<size_t>(match.rightIndex) >= rightFeatures.keypoints.size()) {
            continue;
        }
        leftPoints.push_back(leftFeatures.keypoints[static_cast<size_t>(match.leftIndex)]);
        rightPoints.push_back(rightFeatures.keypoints[static_cast<size_t>(match.rightIndex)]);
    }
}

bool SelectExternalStereoMatches(const ExternalStereoMatchSelectionInput &input,
                                 ExternalStereoMatchSelection &selection)
{
    selection = ExternalStereoMatchSelection{};
    if (!HasStereoFeatureInput(input)) {
        return false;
    }

    const ExternalFeatureSet &leftFeatures = *input.leftFeatures;
    const ExternalFeatureSet &rightFeatures = *input.rightFeatures;
    const cv::Mat &leftPrepared = *input.leftPrepared;
    const cv::Mat &rightPrepared = *input.rightPrepared;
    selection.pairedFeatureCount = std::min(leftFeatures.keypoints.size(), rightFeatures.keypoints.size());
    selection.trustFrontendPairs =
        selection.pairedFeatureCount > 0 &&
        (input.trustFrontendInitPairs || input.trustFrontendRecoveryPairs || input.trustFrontendBootstrapPairs);
    selection.initializationTrustedPairSelection =
        input.initializing && selection.trustFrontendPairs &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_INIT_TRUST_SELECT_CLOSE_PAIRS", false);

    if (selection.initializationTrustedPairSelection) {
        selection.initializationTrustedMatches =
            SelectInitializationTrustedPairs(leftFeatures, rightFeatures, leftPrepared, rightPrepared);
        if (selection.initializationTrustedMatches.empty()) {
            selection.rawMatches = BuildAlignedStereoPairs(leftFeatures, rightFeatures, leftPrepared, rightPrepared);
        }
    } else if (!selection.trustFrontendPairs) {
        selection.rawMatches = BuildAlignedStereoPairs(leftFeatures, rightFeatures, leftPrepared, rightPrepared);
    }

    selection.initializationStereoBias =
        EnvFlagEnabled("SMART_DRONE_SP_LG_INIT_STEREO_BIAS", false) && input.initializing;
    if (selection.initializationStereoBias) {
        BiasInitializationMatches(selection.rawMatches);
    }

    selection.filteredMatches = FilterStereoPairsByDisparityConsistency(selection.rawMatches);
    if (selection.initializationTrustedPairSelection && !selection.initializationTrustedMatches.empty()) {
        CopyMatchedStereoPointsFromPairs(leftFeatures, rightFeatures, selection.initializationTrustedMatches,
                                         selection.matchedLeftPoints, selection.matchedRightPoints);
    } else if (selection.trustFrontendPairs) {
        AppendAlignedFrontendPairs(leftFeatures, rightFeatures, selection.pairedFeatureCount,
                                   selection.matchedLeftPoints, selection.matchedRightPoints);
    } else if (selection.initializationStereoBias && !selection.rawMatches.empty()) {
        CopyMatchedStereoPointsFromPairs(leftFeatures, rightFeatures, selection.rawMatches,
                                         selection.matchedLeftPoints, selection.matchedRightPoints);
    } else if (EnvFlagEnabled("SMART_DRONE_SP_LG_FILTERED_STEREO_INJECT", true) &&
               !selection.filteredMatches.empty()) {
        CopyMatchedStereoPointsFromPairs(leftFeatures, rightFeatures, selection.filteredMatches,
                                         selection.matchedLeftPoints, selection.matchedRightPoints);
    } else {
        AppendAlignedFrontendPairs(leftFeatures, rightFeatures, selection.pairedFeatureCount,
                                   selection.matchedLeftPoints, selection.matchedRightPoints);
    }

    LimitWeakFramePairs(selection.matchedLeftPoints, selection.matchedRightPoints, input);
    return !selection.matchedLeftPoints.empty() && selection.matchedLeftPoints.size() == selection.matchedRightPoints.size();
}

} // namespace smartdrone::adapters::slam
