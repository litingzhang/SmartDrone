#include "adapters/slam/external_temporal_stereo.h"

#include <algorithm>
#include <cstddef>
#include <utility>

#include <opencv2/calib3d.hpp>

#include "adapters/slam/feature_point_tracking.h"
#include "adapters/slam/external_descriptor_geometry.h"
#include "adapters/slam/slam_env.h"
#include "adapters/slam/stereo_geometry.h"

namespace smartdrone::adapters::slam {

namespace {

constexpr int kExternalTemporalFlowWindowPx = 21;
constexpr int kExternalTemporalFlowMaxLevel = 3;
constexpr float kExternalTemporalForwardBackwardMaxErrorPx = 1.5f;
constexpr float kExternalTemporalMergeMinDistancePx = 4.0f;
constexpr float kExternalTemporalStereoMinZnccScore = 0.05f;
constexpr size_t kExternalTemporalMaxCarryPairs = 192;
constexpr size_t kExternalTemporalMaxInjectedPairs = 320;
constexpr size_t kExternalTemporalRansacMinPairs = 10;
constexpr double kExternalTemporalRansacReprojThresholdPx = 3.5;
constexpr size_t kExternalTemporalCarryMinBudget = 24;
constexpr size_t kExternalTemporalCarryMaxBudget = 64;
constexpr size_t kExternalWeakTrackingTemporalCarryBudget = 8;

struct TemporalStereoPair {
    cv::Point2f leftPt;
    cv::Point2f rightPt;
    float zncc{-1.0f};
    int sourceIndex{-1};
};

bool HasTemporalCarryInput(const ExternalTemporalStereoCarryInput &input)
{
    if (input.state == nullptr || input.leftPrepared == nullptr || input.rightPrepared == nullptr) {
        return false;
    }
    const ExternalTemporalStereoStateView &state = *input.state;
    return state.havePrevStereo && state.prevLeft != nullptr && state.prevRight != nullptr &&
           state.prevLeftPoints != nullptr && state.prevRightPoints != nullptr &&
           !state.prevLeft->empty() && !state.prevRight->empty() &&
           !state.prevLeftPoints->empty() && state.prevLeftPoints->size() == state.prevRightPoints->size() &&
           !input.leftPrepared->empty() && !input.rightPrepared->empty();
}

size_t ExternalTemporalStereoCarryBudget(bool previousFrameWeak)
{
    const size_t stableBudget = EnvSizeValueClamped("SMART_DRONE_SP_LG_TEMPORAL_CARRY_BUDGET",
                                                    kExternalTemporalCarryMinBudget, 0,
                                                    kExternalTemporalCarryMaxBudget);
    if (!previousFrameWeak) {
        return stableBudget;
    }
    return EnvSizeValueClamped("SMART_DRONE_SP_LG_WEAK_TEMPORAL_CARRY_BUDGET",
                               kExternalWeakTrackingTemporalCarryBudget, 0,
                               kExternalTemporalCarryMaxBudget);
}

std::vector<TemporalStereoPair> TrackStereoPairsTemporally(const cv::Mat &prevLeftGray, const cv::Mat &prevRightGray,
                                                           const std::vector<cv::Point2f> &prevLeftPoints,
                                                           const std::vector<cv::Point2f> &prevRightPoints,
                                                           const cv::Mat &currLeftGray, const cv::Mat &currRightGray)
{
    std::vector<TemporalStereoPair> trackedPairs;
    if (prevLeftGray.empty() || prevRightGray.empty() || currLeftGray.empty() || currRightGray.empty() ||
        prevLeftPoints.empty() || prevLeftPoints.size() != prevRightPoints.size()) {
        return trackedPairs;
    }

    std::vector<cv::Point2f> trackedLeft;
    std::vector<cv::Point2f> trackedRight;
    std::vector<uchar> leftStatus;
    std::vector<uchar> rightStatus;
    const ForwardBackwardTrackingOptions trackingOptions{kExternalTemporalFlowWindowPx,
                                                         kExternalTemporalFlowMaxLevel,
                                                         kExternalTemporalForwardBackwardMaxErrorPx};
    if (!TrackPointsForwardBackward(prevLeftGray, currLeftGray, prevLeftPoints, trackedLeft, leftStatus,
                                    trackingOptions) ||
        !TrackPointsForwardBackward(prevRightGray, currRightGray, prevRightPoints, trackedRight, rightStatus,
                                    trackingOptions)) {
        return trackedPairs;
    }

    cv::Mat currLeftGray32f;
    cv::Mat currRightGray32f;
    currLeftGray.convertTo(currLeftGray32f, CV_32F);
    currRightGray.convertTo(currRightGray32f, CV_32F);

    trackedPairs.reserve(prevLeftPoints.size());
    for (size_t i = 0; i < prevLeftPoints.size(); ++i) {
        if (i >= trackedLeft.size() || i >= trackedRight.size() || i >= leftStatus.size() || i >= rightStatus.size() ||
            !leftStatus[i] || !rightStatus[i]) {
            continue;
        }

        const cv::Point2f &leftPt = trackedLeft[i];
        const cv::Point2f &rightPt = trackedRight[i];
        if (!IsPointSafeForDescriptor(leftPt, currLeftGray) || !IsPointSafeForDescriptor(rightPt, currRightGray) ||
            !IsStereoPairGeometricallyValid(leftPt, rightPt)) {
            continue;
        }

        float zncc = -1.0f;
        if (!ComputePatchZncc(currLeftGray32f, leftPt, currRightGray32f, rightPt, zncc) ||
            zncc < kExternalTemporalStereoMinZnccScore) {
            continue;
        }

        trackedPairs.push_back(TemporalStereoPair{leftPt, rightPt, zncc, static_cast<int>(i)});
    }

    std::sort(trackedPairs.begin(), trackedPairs.end(),
              [](const TemporalStereoPair &lhs, const TemporalStereoPair &rhs) { return lhs.zncc > rhs.zncc; });
    if (trackedPairs.size() > kExternalTemporalMaxCarryPairs) {
        trackedPairs.resize(kExternalTemporalMaxCarryPairs);
    }
    return trackedPairs;
}

std::vector<TemporalStereoPair> FilterTemporalPairsWithMotionRansac(
    const std::vector<TemporalStereoPair> &trackedPairs, const std::vector<cv::Point2f> &previousLeftPoints)
{
    if (trackedPairs.size() < kExternalTemporalRansacMinPairs || previousLeftPoints.empty()) {
        return trackedPairs;
    }

    std::vector<cv::Point2f> prevPts;
    std::vector<cv::Point2f> currPts;
    std::vector<int> pairIndices;
    prevPts.reserve(trackedPairs.size());
    currPts.reserve(trackedPairs.size());
    pairIndices.reserve(trackedPairs.size());
    for (size_t i = 0; i < trackedPairs.size(); ++i) {
        const int sourceIndex = trackedPairs[i].sourceIndex;
        if (sourceIndex < 0 || static_cast<size_t>(sourceIndex) >= previousLeftPoints.size()) {
            continue;
        }
        prevPts.push_back(previousLeftPoints[static_cast<size_t>(sourceIndex)]);
        currPts.push_back(trackedPairs[i].leftPt);
        pairIndices.push_back(static_cast<int>(i));
    }

    if (prevPts.size() < kExternalTemporalRansacMinPairs) {
        return trackedPairs;
    }

    cv::Mat inlierMask;
    const cv::Mat affine = cv::estimateAffinePartial2D(prevPts, currPts, inlierMask, cv::RANSAC,
                                                       kExternalTemporalRansacReprojThresholdPx);
    if (affine.empty() || inlierMask.empty()) {
        return trackedPairs;
    }

    std::vector<TemporalStereoPair> filtered;
    filtered.reserve(trackedPairs.size());
    for (int row = 0; row < inlierMask.rows; ++row) {
        if (inlierMask.at<uchar>(row, 0) == 0) {
            continue;
        }
        const int pairIndex = pairIndices[static_cast<size_t>(row)];
        filtered.push_back(trackedPairs[static_cast<size_t>(pairIndex)]);
    }

    if (filtered.size() < (trackedPairs.size() / 3)) {
        return trackedPairs;
    }
    return filtered;
}

std::vector<TemporalStereoPair> LimitTemporalPairs(const std::vector<TemporalStereoPair> &trackedPairs, size_t maxCount)
{
    if (trackedPairs.size() <= maxCount) {
        return trackedPairs;
    }
    return std::vector<TemporalStereoPair>(trackedPairs.begin(),
                                           trackedPairs.begin() + static_cast<std::ptrdiff_t>(maxCount));
}

bool IsStereoPairNearExisting(const cv::Point2f &leftPt, const cv::Point2f &rightPt,
                              const std::vector<cv::Point2f> &existingLeft,
                              const std::vector<cv::Point2f> &existingRight)
{
    const float minDistSq = kExternalTemporalMergeMinDistancePx * kExternalTemporalMergeMinDistancePx;
    for (size_t i = 0; i < existingLeft.size() && i < existingRight.size(); ++i) {
        const cv::Point2f leftDelta = existingLeft[i] - leftPt;
        const cv::Point2f rightDelta = existingRight[i] - rightPt;
        if ((leftDelta.x * leftDelta.x + leftDelta.y * leftDelta.y) <= minDistSq &&
            (rightDelta.x * rightDelta.x + rightDelta.y * rightDelta.y) <= minDistSq) {
            return true;
        }
    }
    return false;
}

} // namespace

size_t AppendExternalTemporalStereoCarry(const ExternalTemporalStereoCarryInput &input,
                                         std::vector<cv::Point2f> &matchedLeftPoints,
                                         std::vector<cv::Point2f> &matchedRightPoints)
{
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_TEMPORAL_CARRY", false) || input.initializing ||
        input.recovering || !HasTemporalCarryInput(input)) {
        return 0;
    }

    const ExternalTemporalStereoStateView &state = *input.state;
    const size_t budget = ExternalTemporalStereoCarryBudget(state.previousFrameWeak);
    if (budget == 0) {
        return 0;
    }

    std::vector<TemporalStereoPair> trackedPairs = TrackStereoPairsTemporally(
        *state.prevLeft, *state.prevRight, *state.prevLeftPoints, *state.prevRightPoints,
        *input.leftPrepared, *input.rightPrepared);
    trackedPairs = FilterTemporalPairsWithMotionRansac(trackedPairs, *state.prevLeftPoints);
    trackedPairs = LimitTemporalPairs(trackedPairs, budget);

    std::vector<cv::Point2f> carryLeftPoints;
    std::vector<cv::Point2f> carryRightPoints;
    carryLeftPoints.reserve(trackedPairs.size());
    carryRightPoints.reserve(trackedPairs.size());
    for (const TemporalStereoPair &pair : trackedPairs) {
        if (IsStereoPairNearExisting(pair.leftPt, pair.rightPt, matchedLeftPoints, matchedRightPoints)) {
            continue;
        }
        carryLeftPoints.push_back(pair.leftPt);
        carryRightPoints.push_back(pair.rightPt);
    }
    if (carryLeftPoints.empty()) {
        return 0;
    }

    const size_t before = matchedLeftPoints.size();
    size_t inserted = 0;
    std::vector<cv::Point2f> mergedLeftPoints;
    std::vector<cv::Point2f> mergedRightPoints;
    const size_t maxMergedPairs = EnvSizeValueClamped("SMART_DRONE_SP_LG_TEMPORAL_CARRY_MAX_PAIRS",
                                                      std::max(before, kExternalTemporalMaxInjectedPairs),
                                                      kExternalTemporalMaxInjectedPairs, 1200);
    mergedLeftPoints.reserve(std::min(maxMergedPairs, before + carryLeftPoints.size()));
    mergedRightPoints.reserve(std::min(maxMergedPairs, before + carryRightPoints.size()));
    auto appendUnique = [&](const std::vector<cv::Point2f> &sourceLeft, const std::vector<cv::Point2f> &sourceRight,
                            bool countInserted) {
        for (size_t i = 0; i < sourceLeft.size() && i < sourceRight.size(); ++i) {
            if (mergedLeftPoints.size() >= maxMergedPairs) {
                return;
            }
            if (IsStereoPairNearExisting(sourceLeft[i], sourceRight[i], mergedLeftPoints, mergedRightPoints)) {
                continue;
            }
            mergedLeftPoints.push_back(sourceLeft[i]);
            mergedRightPoints.push_back(sourceRight[i]);
            if (countInserted) {
                ++inserted;
            }
        }
    };
    appendUnique(carryLeftPoints, carryRightPoints, true);
    appendUnique(matchedLeftPoints, matchedRightPoints, false);
    matchedLeftPoints = std::move(mergedLeftPoints);
    matchedRightPoints = std::move(mergedRightPoints);
    return inserted;
}

bool ExtractExternalTemporalStereoSource(const cv::Mat &leftPrepared, const cv::Mat &rightPrepared,
                                         const ExternalStereoObservationPacket &externalData,
                                         cv::Mat &prevLeft, cv::Mat &prevRight,
                                         std::vector<cv::Point2f> &prevLeftPoints,
                                         std::vector<cv::Point2f> &prevRightPoints)
{
    prevLeftPoints.clear();
    prevRightPoints.clear();
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_TEMPORAL_CARRY", false)) {
        return false;
    }

    const size_t pairCount = std::min(externalData.leftKeypoints.size(), externalData.leftToRightMatch.size());
    prevLeftPoints.reserve(pairCount);
    prevRightPoints.reserve(pairCount);
    for (size_t leftIndex = 0; leftIndex < pairCount; ++leftIndex) {
        const int rightIndex = externalData.leftToRightMatch[leftIndex];
        if (rightIndex < 0 || static_cast<size_t>(rightIndex) >= externalData.rightKeypoints.size()) {
            continue;
        }
        prevLeftPoints.push_back(externalData.leftKeypoints[leftIndex].pt);
        prevRightPoints.push_back(externalData.rightKeypoints[static_cast<size_t>(rightIndex)].pt);
    }

    if (prevLeftPoints.empty() || prevLeftPoints.size() != prevRightPoints.size()) {
        return false;
    }

    prevLeft = leftPrepared.clone();
    prevRight = rightPrepared.clone();
    return true;
}

} // namespace smartdrone::adapters::slam
