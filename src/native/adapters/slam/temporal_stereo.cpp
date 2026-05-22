#include "adapters/slam/temporal_stereo.h"

#include <algorithm>
#include <cstddef>
#include <utility>

#include <opencv2/calib3d.hpp>

#include "adapters/slam/descriptor_geometry.h"
#include "adapters/slam/feature_point_tracking.h"
#include "adapters/slam/slam_env.h"
#include "adapters/slam/stereo_geometry.h"

namespace SmartDrone::adapters::slam {

namespace {

constexpr int kTemporalStereoFlowWindowPx = 21;
constexpr int kTemporalStereoFlowMaxLevel = 3;
constexpr float kTemporalStereoForwardBackwardMaxErrorPx = 1.5f;
constexpr float kTemporalStereoMergeMinDistancePx = 4.0f;
constexpr float kTemporalStereoMinZnccScore = 0.05f;
constexpr size_t kTemporalStereoMaxCarryPairs = 192;
constexpr size_t kTemporalStereoMaxInjectedPairs = 320;
constexpr size_t kTemporalStereoRansacMinPairs = 10;
constexpr double kTemporalStereoRansacReprojThresholdPx = 3.5;
constexpr size_t kTemporalStereoCarryMinBudget = 24;
constexpr size_t kTemporalStereoCarryMaxBudget = 64;
constexpr size_t kWeakTrackingTemporalStereoCarryBudget = 8;

struct TemporalStereoPair {
    cv::Point2f leftPt;
    cv::Point2f rightPt;
    float zncc{-1.0f};
    int sourceIndex{-1};
};

struct TemporalStereoTrackingResult {
    std::vector<cv::Point2f> trackedLeft;
    std::vector<cv::Point2f> trackedRight;
    std::vector<uchar> leftStatus;
    std::vector<uchar> rightStatus;
};

struct TemporalStereoMergeBuffers {
    std::vector<cv::Point2f> leftPoints;
    std::vector<cv::Point2f> rightPoints;
    size_t inserted{0};
};

struct TemporalStereoTrackRequest {
    const cv::Mat *prevLeftGray{nullptr};
    const cv::Mat *prevRightGray{nullptr};
    const std::vector<cv::Point2f> *prevLeftPoints{nullptr};
    const std::vector<cv::Point2f> *prevRightPoints{nullptr};
    const cv::Mat *currLeftGray{nullptr};
    const cv::Mat *currRightGray{nullptr};
    const core::ports::IPointTracker2d *pointTracker{nullptr};
};

struct TemporalStereoPairBuildRequest {
    const TemporalStereoTrackingResult *tracking{nullptr};
    const cv::Mat *currLeftGray{nullptr};
    const cv::Mat *currRightGray{nullptr};
    const cv::Mat *currLeftGray32f{nullptr};
    const cv::Mat *currRightGray32f{nullptr};
    size_t index{0};
};

bool HasTemporalCarryInput(const TemporalStereoCarryInput &input)
{
    if (input.state == nullptr || input.leftPrepared == nullptr ||
        input.rightPrepared == nullptr) {
        return false;
    }
    const TemporalStereoStateView &state = *input.state;
    return state.havePrevStereo && state.prevLeft != nullptr &&
           state.prevRight != nullptr && state.prevLeftPoints != nullptr &&
           state.prevRightPoints != nullptr && !state.prevLeft->empty() &&
           !state.prevRight->empty() && !state.prevLeftPoints->empty() &&
           state.prevLeftPoints->size() == state.prevRightPoints->size() &&
           !input.leftPrepared->empty() && !input.rightPrepared->empty();
}

size_t TemporalStereoCarryBudget(bool previousFrameWeak)
{
    const size_t stableBudget = EnvSizeValueClamped(
        "SMART_DRONE_SP_LG_TEMPORAL_CARRY_BUDGET", kTemporalStereoCarryMinBudget,
        0, kTemporalStereoCarryMaxBudget);
    if (!previousFrameWeak) {
        return stableBudget;
    }
    return EnvSizeValueClamped("SMART_DRONE_SP_LG_WEAK_TEMPORAL_CARRY_BUDGET",
                               kWeakTrackingTemporalStereoCarryBudget, 0,
                               kTemporalStereoCarryMaxBudget);
}

bool TrackTemporalStereoPoints(const TemporalStereoTrackRequest &request,
                               TemporalStereoTrackingResult &result)
{
    const ForwardBackwardTrackingOptions trackingOptions{
        kTemporalStereoFlowWindowPx, kTemporalStereoFlowMaxLevel,
        kTemporalStereoForwardBackwardMaxErrorPx};
    DefaultPointTracker2d defaultPointTracker;
    const core::ports::IPointTracker2d &tracker =
        request.pointTracker != nullptr ? *request.pointTracker
                                        : defaultPointTracker;
    return tracker.TrackForwardBackward(*request.prevLeftGray,
                                        *request.currLeftGray,
                                        *request.prevLeftPoints,
                                        result.trackedLeft,
                                        result.leftStatus, trackingOptions) &&
           tracker.TrackForwardBackward(*request.prevRightGray,
                                        *request.currRightGray,
                                        *request.prevRightPoints,
                                        result.trackedRight,
                                        result.rightStatus, trackingOptions);
}

bool IsTrackedTemporalStereoPointValid(
    const TemporalStereoTrackingResult &tracking, size_t index)
{
    return index < tracking.trackedLeft.size() &&
           index < tracking.trackedRight.size() &&
           index < tracking.leftStatus.size() &&
           index < tracking.rightStatus.size() && tracking.leftStatus[index] &&
           tracking.rightStatus[index];
}

bool BuildTemporalStereoPair(const TemporalStereoPairBuildRequest &request,
                             TemporalStereoPair &pair)
{
    const TemporalStereoTrackingResult &tracking = *request.tracking;
    if (!IsTrackedTemporalStereoPointValid(tracking, request.index)) {
        return false;
    }

    const cv::Point2f &leftPt = tracking.trackedLeft[request.index];
    const cv::Point2f &rightPt = tracking.trackedRight[request.index];
    if (!IsPointSafeForDescriptor(leftPt, *request.currLeftGray) ||
        !IsPointSafeForDescriptor(rightPt, *request.currRightGray) ||
        !IsStereoPairGeometricallyValid(leftPt, rightPt)) {
        return false;
    }

    float zncc = -1.0f;
    if (!ComputePatchZncc(*request.currLeftGray32f, leftPt,
                          *request.currRightGray32f, rightPt, zncc) ||
        zncc < kTemporalStereoMinZnccScore) {
        return false;
    }
    pair = TemporalStereoPair{leftPt, rightPt, zncc,
                              static_cast<int>(request.index)};
    return true;
}

void SortAndLimitTemporalPairs(std::vector<TemporalStereoPair> &trackedPairs)
{
    std::sort(trackedPairs.begin(), trackedPairs.end(),
              [](const TemporalStereoPair &lhs, const TemporalStereoPair &rhs) {
                  return lhs.zncc > rhs.zncc;
              });
    if (trackedPairs.size() > kTemporalStereoMaxCarryPairs) {
        trackedPairs.resize(kTemporalStereoMaxCarryPairs);
    }
}

std::vector<TemporalStereoPair> TrackStereoPairsTemporally(
    const cv::Mat &prevLeftGray, const cv::Mat &prevRightGray,
    const std::vector<cv::Point2f> &prevLeftPoints,
    const std::vector<cv::Point2f> &prevRightPoints,
    const cv::Mat &currLeftGray, const cv::Mat &currRightGray,
    const core::ports::IPointTracker2d *pointTracker)
{
    std::vector<TemporalStereoPair> trackedPairs;
    if (prevLeftGray.empty() || prevRightGray.empty() || currLeftGray.empty() ||
        currRightGray.empty() || prevLeftPoints.empty() ||
        prevLeftPoints.size() != prevRightPoints.size()) {
        return trackedPairs;
    }

    TemporalStereoTrackingResult tracking;
    const TemporalStereoTrackRequest trackRequest{
        &prevLeftGray, &prevRightGray, &prevLeftPoints, &prevRightPoints,
        &currLeftGray, &currRightGray, pointTracker};
    if (!TrackTemporalStereoPoints(trackRequest, tracking)) {
        return trackedPairs;
    }

    cv::Mat currLeftGray32f;
    cv::Mat currRightGray32f;
    currLeftGray.convertTo(currLeftGray32f, CV_32F);
    currRightGray.convertTo(currRightGray32f, CV_32F);

    trackedPairs.reserve(prevLeftPoints.size());
    for (size_t i = 0; i < prevLeftPoints.size(); ++i) {
        TemporalStereoPair pair;
        const TemporalStereoPairBuildRequest pairRequest{
            &tracking, &currLeftGray, &currRightGray, &currLeftGray32f,
            &currRightGray32f, i};
        if (BuildTemporalStereoPair(pairRequest, pair)) {
            trackedPairs.push_back(pair);
        }
    }

    SortAndLimitTemporalPairs(trackedPairs);
    return trackedPairs;
}

std::vector<TemporalStereoPair> FilterTemporalPairsWithMotionRansac(
    const std::vector<TemporalStereoPair> &trackedPairs,
    const std::vector<cv::Point2f> &previousLeftPoints)
{
    if (trackedPairs.size() < kTemporalStereoRansacMinPairs ||
        previousLeftPoints.empty()) {
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
        if (sourceIndex < 0 ||
            static_cast<size_t>(sourceIndex) >= previousLeftPoints.size()) {
            continue;
        }
        prevPts.push_back(previousLeftPoints[static_cast<size_t>(sourceIndex)]);
        currPts.push_back(trackedPairs[i].leftPt);
        pairIndices.push_back(static_cast<int>(i));
    }

    if (prevPts.size() < kTemporalStereoRansacMinPairs) {
        return trackedPairs;
    }

    cv::Mat inlierMask;
    const cv::Mat affine =
        cv::estimateAffinePartial2D(prevPts, currPts, inlierMask, cv::RANSAC,
                                    kTemporalStereoRansacReprojThresholdPx);
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

std::vector<TemporalStereoPair>
LimitTemporalPairs(const std::vector<TemporalStereoPair> &trackedPairs,
                   size_t maxCount)
{
    if (trackedPairs.size() <= maxCount) {
        return trackedPairs;
    }
    return std::vector<TemporalStereoPair>(
        trackedPairs.begin(),
        trackedPairs.begin() + static_cast<std::ptrdiff_t>(maxCount));
}

bool IsStereoPairNearExisting(const cv::Point2f &leftPt,
                              const cv::Point2f &rightPt,
                              const std::vector<cv::Point2f> &existingLeft,
                              const std::vector<cv::Point2f> &existingRight)
{
    const float minDistSq =
        kTemporalStereoMergeMinDistancePx * kTemporalStereoMergeMinDistancePx;
    for (size_t i = 0; i < existingLeft.size() && i < existingRight.size(); ++i) {
        const cv::Point2f leftDelta = existingLeft[i] - leftPt;
        const cv::Point2f rightDelta = existingRight[i] - rightPt;
        if ((leftDelta.x * leftDelta.x + leftDelta.y * leftDelta.y) <= minDistSq &&
            (rightDelta.x * rightDelta.x + rightDelta.y * rightDelta.y) <=
                minDistSq) {
            return true;
        }
    }
    return false;
}

void AppendUniqueTemporalPairs(const std::vector<cv::Point2f> &sourceLeft,
                               const std::vector<cv::Point2f> &sourceRight,
                               size_t maxMergedPairs, bool countInserted,
                               TemporalStereoMergeBuffers &buffers)
{
    for (size_t i = 0; i < sourceLeft.size() && i < sourceRight.size(); ++i) {
        if (buffers.leftPoints.size() >= maxMergedPairs) {
            return;
        }
        if (IsStereoPairNearExisting(sourceLeft[i], sourceRight[i],
                                     buffers.leftPoints, buffers.rightPoints)) {
            continue;
        }
        buffers.leftPoints.push_back(sourceLeft[i]);
        buffers.rightPoints.push_back(sourceRight[i]);
        if (countInserted) {
            ++buffers.inserted;
        }
    }
}

void CollectTemporalCarryPairs(
    const std::vector<TemporalStereoPair> &trackedPairs,
    const std::vector<cv::Point2f> &matchedLeftPoints,
    const std::vector<cv::Point2f> &matchedRightPoints,
    std::vector<cv::Point2f> &carryLeftPoints,
    std::vector<cv::Point2f> &carryRightPoints)
{
    carryLeftPoints.reserve(trackedPairs.size());
    carryRightPoints.reserve(trackedPairs.size());
    for (const TemporalStereoPair &pair : trackedPairs) {
        if (!IsStereoPairNearExisting(pair.leftPt, pair.rightPt, matchedLeftPoints,
                                      matchedRightPoints)) {
            carryLeftPoints.push_back(pair.leftPt);
            carryRightPoints.push_back(pair.rightPt);
        }
    }
}

} // namespace

size_t AppendTemporalStereoCarry(const TemporalStereoCarryInput &input,
                                 std::vector<cv::Point2f> &matchedLeftPoints,
                                 std::vector<cv::Point2f> &matchedRightPoints)
{
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_TEMPORAL_CARRY", false) ||
        input.initializing || input.recovering || !HasTemporalCarryInput(input)) {
        return 0;
    }

    const TemporalStereoStateView &state = *input.state;
    const size_t budget = TemporalStereoCarryBudget(state.previousFrameWeak);
    if (budget == 0) {
        return 0;
    }

    std::vector<TemporalStereoPair> trackedPairs = TrackStereoPairsTemporally(
        *state.prevLeft, *state.prevRight, *state.prevLeftPoints,
        *state.prevRightPoints, *input.leftPrepared, *input.rightPrepared,
        input.pointTracker);
    trackedPairs =
        FilterTemporalPairsWithMotionRansac(trackedPairs, *state.prevLeftPoints);
    trackedPairs = LimitTemporalPairs(trackedPairs, budget);

    std::vector<cv::Point2f> carryLeftPoints;
    std::vector<cv::Point2f> carryRightPoints;
    CollectTemporalCarryPairs(trackedPairs, matchedLeftPoints, matchedRightPoints,
                              carryLeftPoints, carryRightPoints);
    if (carryLeftPoints.empty()) {
        return 0;
    }

    const size_t before = matchedLeftPoints.size();
    TemporalStereoMergeBuffers merged;
    const size_t maxMergedPairs =
        EnvSizeValueClamped("SMART_DRONE_SP_LG_TEMPORAL_CARRY_MAX_PAIRS",
                            std::max(before, kTemporalStereoMaxInjectedPairs),
                            kTemporalStereoMaxInjectedPairs, 1200);
    merged.leftPoints.reserve(
        std::min(maxMergedPairs, before + carryLeftPoints.size()));
    merged.rightPoints.reserve(
        std::min(maxMergedPairs, before + carryRightPoints.size()));
    AppendUniqueTemporalPairs(carryLeftPoints, carryRightPoints, maxMergedPairs,
                              true, merged);
    AppendUniqueTemporalPairs(matchedLeftPoints, matchedRightPoints,
                              maxMergedPairs, false, merged);
    matchedLeftPoints = std::move(merged.leftPoints);
    matchedRightPoints = std::move(merged.rightPoints);
    return merged.inserted;
}

bool DefaultTemporalStereoProcessor::AppendCarry(
    const TemporalStereoCarryInput &input,
    std::vector<cv::Point2f> &matchedLeftPoints,
    std::vector<cv::Point2f> &matchedRightPoints,
    TemporalStereoCarryResult &result) const
{
    result.insertedPairCount =
        AppendTemporalStereoCarry(input, matchedLeftPoints, matchedRightPoints);
    return true;
}

bool ExtractTemporalStereoSource(
    const cv::Mat &leftPrepared, const cv::Mat &rightPrepared,
    const core::ports::StereoFeatureObservationPacket &stereoData,
    cv::Mat &prevLeft, cv::Mat &prevRight,
    std::vector<cv::Point2f> &prevLeftPoints,
    std::vector<cv::Point2f> &prevRightPoints)
{
    prevLeftPoints.clear();
    prevRightPoints.clear();
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_TEMPORAL_CARRY", false)) {
        return false;
    }

    const size_t pairCount = std::min(stereoData.leftKeypoints.size(),
                                      stereoData.leftToRightMatch.size());
    prevLeftPoints.reserve(pairCount);
    prevRightPoints.reserve(pairCount);
    for (size_t leftIndex = 0; leftIndex < pairCount; ++leftIndex) {
        const int rightIndex = stereoData.leftToRightMatch[leftIndex];
        if (rightIndex < 0 ||
            static_cast<size_t>(rightIndex) >= stereoData.rightKeypoints.size()) {
            continue;
        }
        prevLeftPoints.push_back(stereoData.leftKeypoints[leftIndex].pt);
        prevRightPoints.push_back(
            stereoData.rightKeypoints[static_cast<size_t>(rightIndex)].pt);
    }

    if (prevLeftPoints.empty() ||
        prevLeftPoints.size() != prevRightPoints.size()) {
        return false;
    }

    prevLeft = leftPrepared.clone();
    prevRight = rightPrepared.clone();
    return true;
}

bool DefaultTemporalStereoProcessor::ExtractSource(
    const TemporalStereoSourceInput &input,
    TemporalStereoSource &source) const
{
    source = TemporalStereoSource{};
    if (input.leftPrepared == nullptr || input.rightPrepared == nullptr ||
        input.observations == nullptr) {
        return false;
    }
    return ExtractTemporalStereoSource(*input.leftPrepared, *input.rightPrepared,
                                       *input.observations, source.prevLeft,
                                       source.prevRight, source.prevLeftPoints,
                                       source.prevRightPoints);
}

} // namespace SmartDrone::adapters::slam
