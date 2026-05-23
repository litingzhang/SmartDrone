#include "adapters/slam/stereo/stereo_matching.h"

#include <algorithm>
#include <cmath>

#include "adapters/slam/engine/slam_env.h"
#include "adapters/slam/stereo/stereo_geometry.h"
#include "adapters/slam/stereo/stereo_pair_builder.h"

namespace SmartDrone::Adapters::Slam {

namespace {

bool HasStereoFeatureInput(
    const Core::Ports::StereoMatchSelectionInput &input)
{
    return input.leftFeatures != nullptr && input.rightFeatures != nullptr &&
           input.leftPrepared != nullptr && input.rightPrepared != nullptr &&
           !input.leftPrepared->empty() && !input.rightPrepared->empty();
}

void AppendAlignedFrontendPairs(
    const Core::Ports::VisualFeatureSet &leftFeatures,
    const Core::Ports::VisualFeatureSet &rightFeatures,
    size_t pairedFeatureCount, std::vector<cv::Point2f> &leftPoints,
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

std::vector<Core::Ports::StereoMatchPair> SelectInitializationTrustedPairs(
    const Core::Ports::VisualFeatureSet &leftFeatures,
    const Core::Ports::VisualFeatureSet &rightFeatures,
    const cv::Mat &leftPrepared, const cv::Mat &rightPrepared,
    const Core::Ports::IStereoPairBuilder &pairBuilder)
{
    Core::Ports::StereoPairBuildInput pairInput;
    pairInput.leftFeatures = &leftFeatures;
    pairInput.rightFeatures = &rightFeatures;
    pairInput.leftPrepared = &leftPrepared;
    pairInput.rightPrepared = &rightPrepared;
    pairInput.mode = Core::Ports::StereoPairBuildMode::AlignedFrontendPairs;
    Core::Ports::StereoPairBuildResult pairResult;
    if (!pairBuilder.BuildPairs(pairInput, pairResult)) {
        return {};
    }
    std::vector<Core::Ports::StereoMatchPair> matches =
        std::move(pairResult.matches);
    if (matches.empty()) {
        return matches;
    }

    const float closeDisparity =
        EnvFloatValue("SMART_DRONE_SP_LG_INIT_CLOSE_DISPARITY", 4.0f);
    std::stable_sort(matches.begin(), matches.end(),
                     [closeDisparity](const Core::Ports::StereoMatchPair &lhs,
                                      const Core::Ports::StereoMatchPair &rhs) {
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

    const size_t maxPairs = EnvSizeValueClamped(
        "SMART_DRONE_SP_LG_INIT_TRUST_MAX_PAIRS", 256, 48, 1200);
    if (matches.size() > maxPairs) {
        matches.resize(maxPairs);
    }
    return matches;
}

void BiasInitializationMatches(
    std::vector<Core::Ports::StereoMatchPair> &matches)
{
    if (matches.empty()) {
        return;
    }
    const float closeDisparity =
        EnvFloatValue("SMART_DRONE_SP_LG_INIT_CLOSE_DISPARITY", 4.0f);
    std::stable_sort(matches.begin(), matches.end(),
                     [closeDisparity](const Core::Ports::StereoMatchPair &lhs,
                                      const Core::Ports::StereoMatchPair &rhs) {
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
                         const Core::Ports::StereoMatchSelectionInput &input)
{
    if (input.initializing || input.recovering ||
        !EnvFlagEnabled("SMART_DRONE_SP_LG_WEAK_FRAME_PAIR_LIMIT", false) ||
        !input.previousFrameWeak) {
        return;
    }

    const size_t maxPairs = EnvSizeValueClamped(
        "SMART_DRONE_SP_LG_WEAK_FRAME_MAX_PAIRS", 192, 48, 1200);
    LimitStereoPairsInPlace(leftPoints, rightPoints, maxPairs);
}

std::vector<Core::Ports::StereoMatchPair> BuildStereoPairsForSelection(
    const Core::Ports::VisualFeatureSet &leftFeatures,
    const Core::Ports::VisualFeatureSet &rightFeatures,
    const cv::Mat &leftPrepared, const cv::Mat &rightPrepared,
    const Core::Ports::IStereoPairBuilder &pairBuilder)
{
    Core::Ports::StereoPairBuildInput pairInput;
    pairInput.leftFeatures = &leftFeatures;
    pairInput.rightFeatures = &rightFeatures;
    pairInput.leftPrepared = &leftPrepared;
    pairInput.rightPrepared = &rightPrepared;
    pairInput.mode =
        EnvFlagEnabled("SMART_DRONE_VISUAL_FEATURE_STEREO_DESCRIPTOR_MATCH",
                       false)
            ? Core::Ports::StereoPairBuildMode::DescriptorSearch
            : Core::Ports::StereoPairBuildMode::AlignedFrontendPairs;

    Core::Ports::StereoPairBuildResult pairResult;
    if (!pairBuilder.BuildPairs(pairInput, pairResult)) {
        return {};
    }
    return std::move(pairResult.matches);
}

void InitializeMatchSelectionState(
    const Core::Ports::StereoMatchSelectionInput &input,
    const Core::Ports::VisualFeatureSet &leftFeatures,
    const Core::Ports::VisualFeatureSet &rightFeatures,
    Core::Ports::StereoMatchSelection &selection)
{
    selection.pairedFeatureCount =
        std::min(leftFeatures.keypoints.size(), rightFeatures.keypoints.size());
    selection.trustFrontendPairs =
        selection.pairedFeatureCount > 0 &&
        (input.trustFrontendInitPairs || input.trustFrontendRecoveryPairs ||
         input.trustFrontendBootstrapPairs);
    selection.initializationTrustedPairSelection =
        input.initializing && selection.trustFrontendPairs &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_INIT_TRUST_SELECT_CLOSE_PAIRS", false);
}

void PopulateRawMatches(
    const Core::Ports::StereoMatchSelectionInput &input,
    const Core::Ports::VisualFeatureSet &leftFeatures,
    const Core::Ports::VisualFeatureSet &rightFeatures,
    const cv::Mat &leftPrepared, const cv::Mat &rightPrepared,
    const Core::Ports::IStereoPairBuilder &pairBuilder,
    Core::Ports::StereoMatchSelection &selection)
{
    if (selection.initializationTrustedPairSelection) {
        selection.initializationTrustedMatches = SelectInitializationTrustedPairs(
            leftFeatures, rightFeatures, leftPrepared, rightPrepared, pairBuilder);
        if (!selection.initializationTrustedMatches.empty()) {
            return;
        }
    } else if (selection.trustFrontendPairs) {
        return;
    }

    selection.rawMatches = BuildStereoPairsForSelection(
        leftFeatures, rightFeatures, leftPrepared, rightPrepared, pairBuilder);
}

void ApplyInitializationStereoBias(
    const Core::Ports::StereoMatchSelectionInput &input,
    Core::Ports::StereoMatchSelection &selection)
{
    selection.initializationStereoBias =
        EnvFlagEnabled("SMART_DRONE_SP_LG_INIT_STEREO_BIAS", false) &&
        input.initializing;
    if (selection.initializationStereoBias) {
        BiasInitializationMatches(selection.rawMatches);
    }
}

bool UseTrustedInitializationMatches(
    const Core::Ports::StereoMatchSelection &selection)
{
    return selection.initializationTrustedPairSelection &&
           !selection.initializationTrustedMatches.empty();
}

void CopySelectedStereoPoints(
    const Core::Ports::VisualFeatureSet &leftFeatures,
    const Core::Ports::VisualFeatureSet &rightFeatures,
    Core::Ports::StereoMatchSelection &selection)
{
    if (UseTrustedInitializationMatches(selection)) {
        CopyMatchedStereoPointsFromPairs(
            leftFeatures, rightFeatures, selection.initializationTrustedMatches,
            selection.matchedLeftPoints, selection.matchedRightPoints);
        return;
    }
    if (selection.trustFrontendPairs) {
        AppendAlignedFrontendPairs(
            leftFeatures, rightFeatures, selection.pairedFeatureCount,
            selection.matchedLeftPoints, selection.matchedRightPoints);
        return;
    }
    if (selection.initializationStereoBias && !selection.rawMatches.empty()) {
        CopyMatchedStereoPointsFromPairs(
            leftFeatures, rightFeatures, selection.rawMatches,
            selection.matchedLeftPoints, selection.matchedRightPoints);
        return;
    }
    if (EnvFlagEnabled("SMART_DRONE_SP_LG_FILTERED_STEREO_INJECT", true) &&
        !selection.filteredMatches.empty()) {
        CopyMatchedStereoPointsFromPairs(
            leftFeatures, rightFeatures, selection.filteredMatches,
            selection.matchedLeftPoints, selection.matchedRightPoints);
        return;
    }
    AppendAlignedFrontendPairs(
        leftFeatures, rightFeatures, selection.pairedFeatureCount,
        selection.matchedLeftPoints, selection.matchedRightPoints);
}

bool HasConsistentMatchedPoints(
    const Core::Ports::StereoMatchSelection &selection)
{
    return !selection.matchedLeftPoints.empty() &&
           selection.matchedLeftPoints.size() == selection.matchedRightPoints.size();
}

} // namespace

void CopyMatchedStereoPointsFromPairs(
    const Core::Ports::VisualFeatureSet &leftFeatures,
    const Core::Ports::VisualFeatureSet &rightFeatures,
    const std::vector<Core::Ports::StereoMatchPair> &matches,
    std::vector<cv::Point2f> &leftPoints,
    std::vector<cv::Point2f> &rightPoints)
{
    leftPoints.clear();
    rightPoints.clear();
    leftPoints.reserve(matches.size());
    rightPoints.reserve(matches.size());
    for (const Core::Ports::StereoMatchPair &match : matches) {
        if (match.leftIndex < 0 || match.rightIndex < 0 ||
            static_cast<size_t>(match.leftIndex) >= leftFeatures.keypoints.size() ||
            static_cast<size_t>(match.rightIndex) >=
                rightFeatures.keypoints.size()) {
            continue;
        }
        leftPoints.push_back(
            leftFeatures.keypoints[static_cast<size_t>(match.leftIndex)]);
        rightPoints.push_back(
            rightFeatures.keypoints[static_cast<size_t>(match.rightIndex)]);
    }
}

bool SelectStereoFeatureMatches(
    const Core::Ports::StereoMatchSelectionInput &input,
    Core::Ports::StereoMatchSelection &selection)
{
    selection = Core::Ports::StereoMatchSelection{};
    if (!HasStereoFeatureInput(input)) {
        return false;
    }

    const Core::Ports::VisualFeatureSet &leftFeatures = *input.leftFeatures;
    const Core::Ports::VisualFeatureSet &rightFeatures = *input.rightFeatures;
    const cv::Mat &leftPrepared = *input.leftPrepared;
    const cv::Mat &rightPrepared = *input.rightPrepared;
    const DefaultStereoPairBuilder defaultPairBuilder;
    const Core::Ports::IStereoPairBuilder &pairBuilder =
        input.pairBuilder != nullptr ? *input.pairBuilder : defaultPairBuilder;

    InitializeMatchSelectionState(input, leftFeatures, rightFeatures, selection);
    PopulateRawMatches(input, leftFeatures, rightFeatures, leftPrepared,
                       rightPrepared, pairBuilder, selection);
    ApplyInitializationStereoBias(input, selection);

    selection.filteredMatches =
        FilterStereoPairsByDisparityConsistency(selection.rawMatches);
    CopySelectedStereoPoints(leftFeatures, rightFeatures, selection);
    LimitWeakFramePairs(selection.matchedLeftPoints, selection.matchedRightPoints,
                        input);
    return HasConsistentMatchedPoints(selection);
}

bool DefaultStereoMatchSelector::SelectMatches(
    const Core::Ports::StereoMatchSelectionInput &input,
    Core::Ports::StereoMatchSelection &selection) const
{
    return SelectStereoFeatureMatches(input, selection);
}

} // namespace SmartDrone::Adapters::Slam
