#include "adapters/slam/stereo/temporal_stereo.h"

#include <algorithm>
#include <cstddef>
#include <utility>

#include <opencv2/calib3d.hpp>

#include "adapters/slam/stereo/descriptor_geometry.h"
#include "adapters/slam/stereo/feature_point_tracking.h"
#include "adapters/slam/engine/slam_env.h"
#include "adapters/slam/stereo/stereo_geometry.h"

namespace SmartDrone::Adapters::Slam {

namespace {

constexpr int TEMPORAL_STEREO_FLOW_WINDOW_PX = 21;
constexpr int TEMPORAL_STEREO_FLOW_MAX_LEVEL = 3;
constexpr float TEMPORAL_STEREO_FORWARD_BACKWARD_MAX_ERROR_PX = 1.5f;
constexpr float TEMPORAL_STEREO_MERGE_MIN_DISTANCE_PX = 4.0f;
constexpr size_t TEMPORAL_STEREO_MAX_CARRY_PAIRS = 192;
constexpr size_t TEMPORAL_STEREO_MAX_INJECTED_PAIRS = 320;
constexpr size_t TEMPORAL_STEREO_RANSAC_MIN_PAIRS = 10;
constexpr double TEMPORAL_STEREO_RANSAC_REPROJ_THRESHOLD_PX = 3.5;
constexpr size_t TEMPORAL_STEREO_CARRY_MIN_BUDGET = 24;
constexpr size_t TEMPORAL_STEREO_CARRY_MAX_BUDGET = 64;
constexpr size_t WEAK_TRACKING_TEMPORAL_STEREO_CARRY_BUDGET = 8;

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
    const Core::Ports::IPointTracker2d *pointTracker{nullptr};
};

struct TemporalStereoPairBuildRequest {
    const TemporalStereoTrackingResult *tracking{nullptr};
    const cv::Mat *currLeftGray{nullptr};
    const cv::Mat *currRightGray{nullptr};
    const cv::Mat *currLeftGray32f{nullptr};
    const cv::Mat *currRightGray32f{nullptr};
    size_t index{0};
};
struct TemporalStereoRansacInput {
    std::vector<cv::Point2f> prevPts;
    std::vector<cv::Point2f> currPts;
    std::vector<int> pairIndices;
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
        "SMART_DRONE_SP_LG_TEMPORAL_CARRY_BUDGET", TEMPORAL_STEREO_CARRY_MIN_BUDGET,
        0, TEMPORAL_STEREO_CARRY_MAX_BUDGET);
    if (!previousFrameWeak) {
        return stableBudget;
    }
    return EnvSizeValueClamped("SMART_DRONE_SP_LG_WEAK_TEMPORAL_CARRY_BUDGET",
                               WEAK_TRACKING_TEMPORAL_STEREO_CARRY_BUDGET, 0,
                               TEMPORAL_STEREO_CARRY_MAX_BUDGET);
}

bool TrackTemporalStereoPoints(const TemporalStereoTrackRequest &request,
                               TemporalStereoTrackingResult &result)
{
    const ForwardBackwardTrackingOptions trackingOptions{
        TEMPORAL_STEREO_FLOW_WINDOW_PX, TEMPORAL_STEREO_FLOW_MAX_LEVEL,
        TEMPORAL_STEREO_FORWARD_BACKWARD_MAX_ERROR_PX};
    DefaultPointTracker2d defaultPointTracker;
    const Core::Ports::IPointTracker2d &tracker =
        request.pointTracker != nullptr ? *request.pointTracker
                                        : defaultPointTracker;
    return tracker.TrackForwardBackward(
               ForwardBackwardTrackingRequest{
                   *request.prevLeftGray, *request.currLeftGray,
                   *request.prevLeftPoints, result.trackedLeft,
                   result.leftStatus, trackingOptions}) &&
           tracker.TrackForwardBackward(
               ForwardBackwardTrackingRequest{
                   *request.prevRightGray, *request.currRightGray,
                   *request.prevRightPoints, result.trackedRight,
                   result.rightStatus, trackingOptions});
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
        zncc < TEMPORAL_STEREO_MIN_ZNCC_SCORE) {
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
    if (trackedPairs.size() > TEMPORAL_STEREO_MAX_CARRY_PAIRS) {
        trackedPairs.resize(TEMPORAL_STEREO_MAX_CARRY_PAIRS);
    }
}

std::vector<TemporalStereoPair> TrackStereoPairsTemporally(
    const TemporalStereoTrackRequest &request)
{
    std::vector<TemporalStereoPair> trackedPairs;
    const cv::Mat &prevLeftGray = *request.prevLeftGray;
    const cv::Mat &prevRightGray = *request.prevRightGray;
    const cv::Mat &currLeftGray = *request.currLeftGray;
    const cv::Mat &currRightGray = *request.currRightGray;
    const auto &prevLeftPoints = *request.prevLeftPoints;
    const auto &prevRightPoints = *request.prevRightPoints;
    if (prevLeftGray.empty() || prevRightGray.empty() || currLeftGray.empty() ||
        currRightGray.empty() || prevLeftPoints.empty() ||
        prevLeftPoints.size() != prevRightPoints.size()) {
        return trackedPairs;
    }

    TemporalStereoTrackingResult tracking;
    if (!TrackTemporalStereoPoints(request, tracking)) {
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

TemporalStereoRansacInput BuildTemporalStereoRansacInput(
    const std::vector<TemporalStereoPair> &trackedPairs,
    const std::vector<cv::Point2f> &previousLeftPoints)
{
    TemporalStereoRansacInput input;
    input.prevPts.reserve(trackedPairs.size());
    input.currPts.reserve(trackedPairs.size());
    input.pairIndices.reserve(trackedPairs.size());
    for (size_t i = 0; i < trackedPairs.size(); ++i) {
        const int sourceIndex = trackedPairs[i].sourceIndex;
        if (sourceIndex < 0 ||
            static_cast<size_t>(sourceIndex) >= previousLeftPoints.size()) {
            continue;
        }
        input.prevPts.push_back(
            previousLeftPoints[static_cast<size_t>(sourceIndex)]);
        input.currPts.push_back(trackedPairs[i].leftPt);
        input.pairIndices.push_back(static_cast<int>(i));
    }
    return input;
}

cv::Mat EstimateTemporalStereoInlierMask(
    const TemporalStereoRansacInput &input)
{
    cv::Mat inlierMask;
    const cv::Mat affine =
        cv::estimateAffinePartial2D(input.prevPts, input.currPts, inlierMask,
                                    cv::RANSAC,
                                    TEMPORAL_STEREO_RANSAC_REPROJ_THRESHOLD_PX);
    if (affine.empty() || inlierMask.empty()) {
        return {};
    }
    return inlierMask;
}

std::vector<TemporalStereoPair> CollectTemporalStereoInliers(
    const std::vector<TemporalStereoPair> &trackedPairs,
    const TemporalStereoRansacInput &input,
    const cv::Mat &inlierMask)
{
    std::vector<TemporalStereoPair> filtered;
    filtered.reserve(trackedPairs.size());
    for (int row = 0; row < inlierMask.rows; ++row) {
        if (inlierMask.at<uchar>(row, 0) == 0) {
            continue;
        }
        const int pairIndex = input.pairIndices[static_cast<size_t>(row)];
        filtered.push_back(trackedPairs[static_cast<size_t>(pairIndex)]);
    }
    return filtered;
}

std::vector<TemporalStereoPair> FilterTemporalPairsWithMotionRansac(
    const std::vector<TemporalStereoPair> &trackedPairs,
    const std::vector<cv::Point2f> &previousLeftPoints)
{
    if (trackedPairs.size() < TEMPORAL_STEREO_RANSAC_MIN_PAIRS ||
        previousLeftPoints.empty()) {
        return trackedPairs;
    }

    const TemporalStereoRansacInput input =
        BuildTemporalStereoRansacInput(trackedPairs, previousLeftPoints);
    if (input.prevPts.size() < TEMPORAL_STEREO_RANSAC_MIN_PAIRS) {
        return trackedPairs;
    }
    const cv::Mat inlierMask = EstimateTemporalStereoInlierMask(input);
    if (inlierMask.empty()) {
        return trackedPairs;
    }
    const std::vector<TemporalStereoPair> filtered =
        CollectTemporalStereoInliers(trackedPairs, input, inlierMask);
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
        TEMPORAL_STEREO_MERGE_MIN_DISTANCE_PX * TEMPORAL_STEREO_MERGE_MIN_DISTANCE_PX;
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
        {state.prevLeft, state.prevRight, state.prevLeftPoints,
         state.prevRightPoints, input.leftPrepared, input.rightPrepared,
         input.pointTracker});
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
                            std::max(before, TEMPORAL_STEREO_MAX_INJECTED_PAIRS),
                            TEMPORAL_STEREO_MAX_INJECTED_PAIRS, 1200);
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
    const TemporalStereoSourceExtractionRequest &request)
{
    request.prevLeftPoints.clear();
    request.prevRightPoints.clear();
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_TEMPORAL_CARRY", false)) {
        return false;
    }

    const auto &stereoData = request.stereoData;
    const size_t pairCount = std::min(stereoData.leftKeypoints.size(),
                                      stereoData.leftToRightMatch.size());
    request.prevLeftPoints.reserve(pairCount);
    request.prevRightPoints.reserve(pairCount);
    for (size_t leftIndex = 0; leftIndex < pairCount; ++leftIndex) {
        const int rightIndex = stereoData.leftToRightMatch[leftIndex];
        if (rightIndex < 0 ||
            static_cast<size_t>(rightIndex) >= stereoData.rightKeypoints.size()) {
            continue;
        }
        request.prevLeftPoints.push_back(stereoData.leftKeypoints[leftIndex].pt);
        request.prevRightPoints.push_back(
            stereoData.rightKeypoints[static_cast<size_t>(rightIndex)].pt);
    }

    if (request.prevLeftPoints.empty() ||
        request.prevLeftPoints.size() != request.prevRightPoints.size()) {
        return false;
    }

    request.prevLeft = request.leftPrepared.clone();
    request.prevRight = request.rightPrepared.clone();
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
    return ExtractTemporalStereoSource(
        {*input.leftPrepared, *input.rightPrepared, *input.observations,
         source.prevLeft, source.prevRight, source.prevLeftPoints,
         source.prevRightPoints});
}

} // namespace SmartDrone::Adapters::Slam
