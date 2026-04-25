#include "adapters/slam/orbslam3_engine.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <sophus/se3.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "ImuTypes.h"
#include "TrackedVisualData.h"

namespace smartdrone::adapters::slam {

namespace {

constexpr int kOrbDescriptorBorder = 19;
constexpr float kStereoMaxEpipolarDeltaPx = 1.5f;
constexpr float kStereoMinDisparityPx = 0.75f;
constexpr float kStereoMaxDisparityPx = 240.0f;
constexpr float kStereoSimilarityRatioTest = 0.98f;
constexpr float kStereoMinDescriptorSimilarity = 0.20f;
constexpr float kStereoMinZnccScore = 0.10f;
constexpr int kStereoPatchRadiusPx = 3;
constexpr int kStereoGridCols = 8;
constexpr int kStereoGridRows = 6;
constexpr int kStereoMaxPairsPerCell = 10;
constexpr int kTemporalFlowWindowPx = 21;
constexpr int kTemporalFlowMaxLevel = 3;
constexpr float kTemporalForwardBackwardMaxErrorPx = 1.5f;
constexpr float kTemporalMergeMinDistancePx = 4.0f;
constexpr float kTemporalStereoMinZnccScore = 0.05f;
constexpr size_t kTemporalMaxCarryPairs = 192;
constexpr size_t kTemporalMaxInjectedPairs = 320;
constexpr size_t kTemporalRansacMinPairs = 10;
constexpr double kTemporalRansacReprojThresholdPx = 3.5;
constexpr float kStereoDisparityMadScale = 2.5f;
constexpr float kStereoDisparityMinTolerancePx = 6.0f;
constexpr size_t kTemporalCarryMinBudget = 24;
constexpr size_t kTemporalCarryMaxBudget = 64;
constexpr int kWeakTrackingMinInliers = 24;
constexpr int kWeakTrackingMinTrackedMapPoints = 24;
constexpr size_t kWeakTrackingTemporalCarryBudget = 8;
constexpr size_t kWeakTrackingInjectedPairBudget = 56;
constexpr int kStableOrbTrackMinInliers = 50;
constexpr size_t kStableOrbTrackMinTrackedMapPoints = 80;
constexpr double kPoseStabilizerDefaultDtSec = 1.0 / 20.0;
constexpr double kPoseStabilizerMinDtSec = 1.0 / 120.0;
constexpr double kPoseStabilizerMaxDtSec = 0.25;
constexpr float kPoseStabilizerMaxSpeedMps = 3.0f;
constexpr float kPoseStabilizerMaxStepMeters = 0.055f;
constexpr float kPoseStabilizerMaxRotStepDeg = 3.0f;
constexpr float kPoseStabilizerVelocityAlpha = 0.35f;
constexpr float kPoseStabilizerPredictedVelocityDecay = 0.985f;
constexpr float kLkMaxDepthMeters = 12.0f;
constexpr float kLkMaxFlowPx = 96.0f;
constexpr float kLkMaxStepMeters = 0.35f;
constexpr int kLkMinPnPPoints = 12;
constexpr int kLkMinPnPInliers = 10;
constexpr int kLkGridCols = 8;
constexpr int kLkGridRows = 6;
constexpr int kLkGridCellCount = kLkGridCols * kLkGridRows;
constexpr int kLkTargetTracksPerCell = 8;
constexpr int kLkMinTracksPerCell = 4;
constexpr size_t kLkMaxTracks = 384;
constexpr uint64_t kLkAsyncMaxSeedTrackFrames = 8;
constexpr size_t kLkFrameHistoryMaxSize = 64;
constexpr float kLkMinSeedDistancePx = 5.0f;
constexpr float kLkMinCandidateQuality = 0.18f;

struct StereoMatchPair {
    int leftIndex{-1};
    int rightIndex{-1};
    float descriptorScore{-std::numeric_limits<float>::infinity()};
    float zncc{-1.0f};
    float disparity{0.0f};
    float quality{0.0f};
};

struct TemporalStereoPair {
    cv::Point2f leftPt;
    cv::Point2f rightPt;
    float zncc{-1.0f};
    int sourceIndex{-1};
};

cv::KeyPoint MakeKeyPoint(const cv::Point2f &pt)
{
    cv::KeyPoint kp;
    kp.pt = pt;
    kp.size = 31.0f;
    kp.angle = -1.0f;
    kp.octave = 0;
    kp.response = 1.0f;
    return kp;
}

bool IsPointSafeForOrbDescriptor(const cv::Point2f &pt, const cv::Mat &gray)
{
    return pt.x >= static_cast<float>(kOrbDescriptorBorder) &&
           pt.x < static_cast<float>(gray.cols - kOrbDescriptorBorder) &&
           pt.y >= static_cast<float>(kOrbDescriptorBorder) &&
           pt.y < static_cast<float>(gray.rows - kOrbDescriptorBorder);
}

ORB_SLAM3::ORBextractor *SelectMonoExtractor(ORB_SLAM3::Tracking *tracker)
{
    if (tracker == nullptr) {
        return nullptr;
    }
    const bool needInitExtractor =
        tracker->mState == ORB_SLAM3::Tracking::NOT_INITIALIZED ||
        tracker->mState == ORB_SLAM3::Tracking::NO_IMAGES_YET;
    return needInitExtractor ? tracker->GetInitORBExtractor() : tracker->GetLeftORBExtractor();
}

bool ComputeOrbDescriptorsAtPoints(ORB_SLAM3::ORBextractor *extractor, const cv::Mat &gray,
                                   const std::vector<cv::Point2f> &points, std::vector<cv::KeyPoint> &keypoints,
                                   cv::Mat &descriptors)
{
    keypoints.clear();
    descriptors.release();
    if (extractor == nullptr || gray.empty() || points.empty()) {
        return false;
    }

    keypoints.reserve(points.size());
    for (const cv::Point2f &pt : points) {
        keypoints.push_back(MakeKeyPoint(pt));
    }

    if (!extractor->ComputeDescriptorsAtKeypoints(gray, keypoints, descriptors)) {
        return false;
    }
    return !keypoints.empty() && !descriptors.empty() &&
           descriptors.rows == static_cast<int>(keypoints.size()) && descriptors.type() == CV_8U;
}

bool ComputePatchZncc(const cv::Mat &leftGray32f, const cv::Point2f &leftPt, const cv::Mat &rightGray32f,
                      const cv::Point2f &rightPt, float &score)
{
    score = -1.0f;
    if (leftGray32f.empty() || rightGray32f.empty()) {
        return false;
    }

    const int patchSize = 2 * kStereoPatchRadiusPx + 1;
    if (leftPt.x < static_cast<float>(kStereoPatchRadiusPx) ||
        leftPt.y < static_cast<float>(kStereoPatchRadiusPx) ||
        rightPt.x < static_cast<float>(kStereoPatchRadiusPx) ||
        rightPt.y < static_cast<float>(kStereoPatchRadiusPx) ||
        leftPt.x >= static_cast<float>(leftGray32f.cols - kStereoPatchRadiusPx) ||
        leftPt.y >= static_cast<float>(leftGray32f.rows - kStereoPatchRadiusPx) ||
        rightPt.x >= static_cast<float>(rightGray32f.cols - kStereoPatchRadiusPx) ||
        rightPt.y >= static_cast<float>(rightGray32f.rows - kStereoPatchRadiusPx)) {
        return false;
    }

    cv::Mat leftPatch;
    cv::Mat rightPatch;
    cv::getRectSubPix(leftGray32f, cv::Size(patchSize, patchSize), leftPt, leftPatch);
    cv::getRectSubPix(rightGray32f, cv::Size(patchSize, patchSize), rightPt, rightPatch);
    if (leftPatch.empty() || rightPatch.empty()) {
        return false;
    }

    cv::Scalar leftMean;
    cv::Scalar leftStd;
    cv::Scalar rightMean;
    cv::Scalar rightStd;
    cv::meanStdDev(leftPatch, leftMean, leftStd);
    cv::meanStdDev(rightPatch, rightMean, rightStd);
    if (leftStd[0] < 1e-3 || rightStd[0] < 1e-3) {
        return false;
    }

    cv::Mat leftNorm = leftPatch - leftMean[0];
    cv::Mat rightNorm = rightPatch - rightMean[0];
    const double denom = static_cast<double>(leftNorm.total()) * leftStd[0] * rightStd[0];
    if (denom <= 1e-9) {
        return false;
    }
    score = static_cast<float>(leftNorm.dot(rightNorm) / denom);
    return std::isfinite(score);
}

bool IsBetterRightCandidate(float candidateScore, float candidateZncc, float currentScore, float currentZncc)
{
    if (!std::isfinite(currentScore)) {
        return true;
    }
    if (candidateScore != currentScore) {
        return candidateScore > currentScore;
    }
    return candidateZncc > currentZncc;
}

bool HasValidXFeatDescriptors(const XFeatFeatureSet &features)
{
    return !features.descriptors.empty() && features.descriptors.type() == CV_32F &&
           features.descriptors.rows == static_cast<int>(features.keypoints.size()) && features.descriptors.cols > 0;
}

cv::Mat MakeCameraMatrix(float fx, float fy, float cx, float cy)
{
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = fx;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    return K;
}

cv::Mat MakeDistCoeffs(float k1, float k2, float p1, float p2)
{
    cv::Mat D = cv::Mat::zeros(1, 4, CV_64F);
    D.at<double>(0, 0) = k1;
    D.at<double>(0, 1) = k2;
    D.at<double>(0, 2) = p1;
    D.at<double>(0, 3) = p2;
    return D;
}

cv::Mat EnsureGray8(const cv::Mat &image)
{
    if (image.empty()) {
        return {};
    }
    if (image.type() == CV_8UC1) {
        return image;
    }
    cv::Mat gray;
    if (image.channels() == 1) {
        image.convertTo(gray, CV_8U);
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    return gray;
}

bool DescriptorSimilarity(const cv::Mat &lhs, const cv::Mat &rhs, float &score)
{
    score = -std::numeric_limits<float>::infinity();
    if (lhs.empty() || rhs.empty() || lhs.type() != CV_32F || rhs.type() != CV_32F || lhs.cols != rhs.cols ||
        lhs.rows != 1 || rhs.rows != 1) {
        return false;
    }
    score = lhs.dot(rhs);
    return std::isfinite(score);
}

std::vector<StereoMatchPair> SelectGridBalancedPairs(const std::vector<StereoMatchPair> &matches,
                                                     const std::vector<cv::Point2f> &leftKeypoints, int imageWidth,
                                                     int imageHeight)
{
    if (matches.empty() || imageWidth <= 0 || imageHeight <= 0) {
        return matches;
    }

    const int cellWidth = std::max(1, (imageWidth + kStereoGridCols - 1) / kStereoGridCols);
    const int cellHeight = std::max(1, (imageHeight + kStereoGridRows - 1) / kStereoGridRows);
    std::vector<int> cellCounts(static_cast<size_t>(kStereoGridCols * kStereoGridRows), 0);
    std::vector<StereoMatchPair> selected;
    selected.reserve(matches.size());

    for (const StereoMatchPair &match : matches) {
        const cv::Point2f &pt = leftKeypoints[static_cast<size_t>(match.leftIndex)];
        const int col = std::clamp(static_cast<int>(pt.x) / cellWidth, 0, kStereoGridCols - 1);
        const int row = std::clamp(static_cast<int>(pt.y) / cellHeight, 0, kStereoGridRows - 1);
        const size_t cellIndex = static_cast<size_t>(row * kStereoGridCols + col);
        if (cellCounts[cellIndex] >= kStereoMaxPairsPerCell) {
            continue;
        }
        ++cellCounts[cellIndex];
        selected.push_back(match);
    }
    return selected;
}

bool IsStereoPairGeometricallyValid(const cv::Point2f &leftPt, const cv::Point2f &rightPt)
{
    const float yDelta = std::fabs(leftPt.y - rightPt.y);
    const float disparity = leftPt.x - rightPt.x;
    return yDelta <= kStereoMaxEpipolarDeltaPx && disparity >= kStereoMinDisparityPx &&
           disparity <= kStereoMaxDisparityPx;
}

float ComputeStereoCandidateQuality(float descriptorScore, float zncc, float epipolarErrorPx, float disparity)
{
    const float descriptorTerm = std::clamp((descriptorScore - kStereoMinDescriptorSimilarity) /
                                                std::max(1e-3f, 1.0f - kStereoMinDescriptorSimilarity),
                                            0.0f, 1.0f);
    const float znccTerm = std::clamp((zncc + 1.0f) * 0.5f, 0.0f, 1.0f);
    const float epipolarPenalty = std::clamp(epipolarErrorPx / std::max(1e-3f, kStereoMaxEpipolarDeltaPx), 0.0f, 1.0f);
    const float disparityPenalty =
        (disparity < 2.0f || disparity > kStereoMaxDisparityPx * 0.85f) ? 0.25f : 0.0f;
    return std::clamp(0.50f * descriptorTerm + 0.35f * znccTerm + 0.15f * (1.0f - epipolarPenalty) -
                          disparityPenalty,
                      0.0f, 1.0f);
}

bool TrackPointsWithForwardBackward(const cv::Mat &prevGray, const cv::Mat &currGray,
                                   const std::vector<cv::Point2f> &prevPts, std::vector<cv::Point2f> &currPts,
                                   std::vector<uchar> &status)
{
    currPts.clear();
    status.clear();
    if (prevGray.empty() || currGray.empty() || prevPts.empty()) {
        return false;
    }

    std::vector<float> errors;
    cv::calcOpticalFlowPyrLK(prevGray, currGray, prevPts, currPts, status, errors,
                             cv::Size(kTemporalFlowWindowPx, kTemporalFlowWindowPx), kTemporalFlowMaxLevel);
    if (currPts.empty() || status.empty()) {
        return false;
    }

    std::vector<cv::Point2f> backwardPts;
    std::vector<uchar> backwardStatus;
    std::vector<float> backwardErrors;
    cv::calcOpticalFlowPyrLK(currGray, prevGray, currPts, backwardPts, backwardStatus, backwardErrors,
                             cv::Size(kTemporalFlowWindowPx, kTemporalFlowWindowPx), kTemporalFlowMaxLevel);

    for (size_t i = 0; i < status.size(); ++i) {
        if (!status[i] || i >= backwardStatus.size() || !backwardStatus[i] || i >= backwardPts.size()) {
            status[i] = 0;
            continue;
        }

        const cv::Point2f &backPt = backwardPts[i];
        const cv::Point2f delta = backPt - prevPts[i];
        if ((delta.x * delta.x + delta.y * delta.y) >
            (kTemporalForwardBackwardMaxErrorPx * kTemporalForwardBackwardMaxErrorPx)) {
            status[i] = 0;
        }
    }

    return true;
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
    if (!TrackPointsWithForwardBackward(prevLeftGray, currLeftGray, prevLeftPoints, trackedLeft, leftStatus) ||
        !TrackPointsWithForwardBackward(prevRightGray, currRightGray, prevRightPoints, trackedRight, rightStatus)) {
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
        if (!IsPointSafeForOrbDescriptor(leftPt, currLeftGray) || !IsPointSafeForOrbDescriptor(rightPt, currRightGray) ||
            !IsStereoPairGeometricallyValid(leftPt, rightPt)) {
            continue;
        }

        float zncc = -1.0f;
        if (!ComputePatchZncc(currLeftGray32f, leftPt, currRightGray32f, rightPt, zncc) ||
            zncc < kTemporalStereoMinZnccScore) {
            continue;
        }

        trackedPairs.push_back(TemporalStereoPair{leftPt, rightPt, zncc, static_cast<int>(i)});
    }

    std::sort(trackedPairs.begin(), trackedPairs.end(),
              [](const TemporalStereoPair &lhs, const TemporalStereoPair &rhs) { return lhs.zncc > rhs.zncc; });
    if (trackedPairs.size() > kTemporalMaxCarryPairs) {
        trackedPairs.resize(kTemporalMaxCarryPairs);
    }
    return trackedPairs;
}

std::vector<TemporalStereoPair> FilterTemporalPairsWithMotionRansac(
    const std::vector<TemporalStereoPair> &trackedPairs, const std::vector<cv::Point2f> &previousLeftPoints)
{
    if (trackedPairs.size() < kTemporalRansacMinPairs || previousLeftPoints.empty()) {
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

    if (prevPts.size() < kTemporalRansacMinPairs) {
        return trackedPairs;
    }

    cv::Mat inlierMask;
    const cv::Mat affine = cv::estimateAffinePartial2D(prevPts, currPts, inlierMask, cv::RANSAC,
                                                       kTemporalRansacReprojThresholdPx);
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
    return std::vector<TemporalStereoPair>(trackedPairs.begin(), trackedPairs.begin() + static_cast<std::ptrdiff_t>(maxCount));
}

std::vector<StereoMatchPair> FilterStereoPairsByDisparityConsistency(const std::vector<StereoMatchPair> &matches)
{
    if (matches.size() < 8) {
        return matches;
    }

    std::vector<float> disparities;
    disparities.reserve(matches.size());
    for (const StereoMatchPair &match : matches) {
        disparities.push_back(match.disparity);
    }

    std::vector<float> sortedDisparities = disparities;
    const auto medianIt = sortedDisparities.begin() + static_cast<std::ptrdiff_t>(sortedDisparities.size() / 2);
    std::nth_element(sortedDisparities.begin(), medianIt, sortedDisparities.end());
    const float medianDisparity = *medianIt;

    std::vector<float> absDeviation;
    absDeviation.reserve(disparities.size());
    for (const float disparity : disparities) {
        absDeviation.push_back(std::fabs(disparity - medianDisparity));
    }
    auto madIt = absDeviation.begin() + static_cast<std::ptrdiff_t>(absDeviation.size() / 2);
    std::nth_element(absDeviation.begin(), madIt, absDeviation.end());
    const float mad = *madIt;
    const float tolerance = std::max(kStereoDisparityMinTolerancePx, kStereoDisparityMadScale * std::max(mad, 1.0f));

    std::vector<StereoMatchPair> filtered;
    filtered.reserve(matches.size());
    for (const StereoMatchPair &match : matches) {
        if (std::fabs(match.disparity - medianDisparity) <= tolerance) {
            filtered.push_back(match);
        }
    }
    if (filtered.size() < (matches.size() / 2)) {
        return matches;
    }
    return filtered;
}

bool IsStereoPairNearExisting(const cv::Point2f &leftPt, const cv::Point2f &rightPt,
                              const std::vector<cv::Point2f> &existingLeft,
                              const std::vector<cv::Point2f> &existingRight)
{
    const float minDistSq = kTemporalMergeMinDistancePx * kTemporalMergeMinDistancePx;
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

void AppendStereoPairs(const std::vector<cv::Point2f> &sourceLeft, const std::vector<cv::Point2f> &sourceRight,
                       std::vector<cv::Point2f> &mergedLeft, std::vector<cv::Point2f> &mergedRight)
{
    for (size_t i = 0; i < sourceLeft.size() && i < sourceRight.size(); ++i) {
        if (mergedLeft.size() >= kTemporalMaxInjectedPairs) {
            return;
        }
        if (IsStereoPairNearExisting(sourceLeft[i], sourceRight[i], mergedLeft, mergedRight)) {
            continue;
        }
        mergedLeft.push_back(sourceLeft[i]);
        mergedRight.push_back(sourceRight[i]);
    }
}

void LimitStereoPairsInPlace(std::vector<cv::Point2f> &leftPoints, std::vector<cv::Point2f> &rightPoints, size_t maxCount)
{
    const size_t limitedCount = std::min({maxCount, leftPoints.size(), rightPoints.size()});
    leftPoints.resize(limitedCount);
    rightPoints.resize(limitedCount);
}

bool FinalizeStereoExternalFromPairs(ORB_SLAM3::ORBextractor *leftExtractor,
                                     ORB_SLAM3::ORBextractor *rightExtractor, const cv::Mat &leftGray,
                                     const cv::Mat &rightGray, const std::vector<cv::Point2f> &leftPoints,
                                     const std::vector<cv::Point2f> &rightPoints,
                                     ORB_SLAM3::ExternalStereoFrameData &outData)
{
    if (leftExtractor == nullptr || rightExtractor == nullptr || leftGray.empty() || rightGray.empty() ||
        leftPoints.empty() || leftPoints.size() != rightPoints.size()) {
        return false;
    }

    std::vector<cv::Point2f> filteredLeft;
    std::vector<cv::Point2f> filteredRight;
    filteredLeft.reserve(leftPoints.size());
    filteredRight.reserve(rightPoints.size());
    for (size_t i = 0; i < leftPoints.size(); ++i) {
        if (!IsPointSafeForOrbDescriptor(leftPoints[i], leftGray) ||
            !IsPointSafeForOrbDescriptor(rightPoints[i], rightGray)) {
            continue;
        }
        filteredLeft.push_back(leftPoints[i]);
        filteredRight.push_back(rightPoints[i]);
    }

    if (filteredLeft.empty() || filteredLeft.size() != filteredRight.size()) {
        return false;
    }

    std::vector<cv::KeyPoint> leftKeypoints;
    std::vector<cv::KeyPoint> rightKeypoints;
    cv::Mat leftDescriptors;
    cv::Mat rightDescriptors;
    if (!ComputeOrbDescriptorsAtPoints(leftExtractor, leftGray, filteredLeft, leftKeypoints, leftDescriptors) ||
        !ComputeOrbDescriptorsAtPoints(rightExtractor, rightGray, filteredRight, rightKeypoints, rightDescriptors)) {
        return false;
    }
    if (leftKeypoints.size() != rightKeypoints.size() || leftDescriptors.rows != rightDescriptors.rows ||
        leftDescriptors.rows != static_cast<int>(leftKeypoints.size())) {
        return false;
    }

    outData.leftKeypoints = std::move(leftKeypoints);
    outData.rightKeypoints = std::move(rightKeypoints);
    outData.leftDescriptors = std::move(leftDescriptors);
    outData.rightDescriptors = std::move(rightDescriptors);
    outData.matchedStereoPairs = true;
    return true;
}

bool FinalizeStereoExternalFromTemporalCarry(const std::vector<TemporalStereoPair> &trackedPairs,
                                             const ORB_SLAM3::ExternalStereoFrameData &previousExternal,
                                             ORB_SLAM3::ExternalStereoFrameData &outData)
{
    const size_t previousCount =
        std::min({previousExternal.leftKeypoints.size(), previousExternal.rightKeypoints.size(),
                  static_cast<size_t>(std::max(0, previousExternal.leftDescriptors.rows)),
                  static_cast<size_t>(std::max(0, previousExternal.rightDescriptors.rows))});
    if (trackedPairs.empty() || previousCount == 0 || previousExternal.leftDescriptors.empty() ||
        previousExternal.rightDescriptors.empty() ||
        previousExternal.leftDescriptors.type() != previousExternal.rightDescriptors.type() ||
        previousExternal.leftDescriptors.cols != previousExternal.rightDescriptors.cols) {
        return false;
    }

    std::vector<cv::KeyPoint> leftKeypoints;
    std::vector<cv::KeyPoint> rightKeypoints;
    std::vector<int> descriptorRows;
    leftKeypoints.reserve(trackedPairs.size());
    rightKeypoints.reserve(trackedPairs.size());
    descriptorRows.reserve(trackedPairs.size());
    for (const TemporalStereoPair &pair : trackedPairs) {
        if (pair.sourceIndex < 0 || static_cast<size_t>(pair.sourceIndex) >= previousCount) {
            continue;
        }
        leftKeypoints.push_back(MakeKeyPoint(pair.leftPt));
        rightKeypoints.push_back(MakeKeyPoint(pair.rightPt));
        descriptorRows.push_back(pair.sourceIndex);
    }

    if (descriptorRows.empty()) {
        return false;
    }

    cv::Mat leftDescriptors(static_cast<int>(descriptorRows.size()), previousExternal.leftDescriptors.cols,
                            previousExternal.leftDescriptors.type());
    cv::Mat rightDescriptors(static_cast<int>(descriptorRows.size()), previousExternal.rightDescriptors.cols,
                             previousExternal.rightDescriptors.type());
    for (size_t i = 0; i < descriptorRows.size(); ++i) {
        previousExternal.leftDescriptors.row(descriptorRows[i]).copyTo(leftDescriptors.row(static_cast<int>(i)));
        previousExternal.rightDescriptors.row(descriptorRows[i]).copyTo(rightDescriptors.row(static_cast<int>(i)));
    }

    outData.leftKeypoints = std::move(leftKeypoints);
    outData.rightKeypoints = std::move(rightKeypoints);
    outData.leftDescriptors = std::move(leftDescriptors);
    outData.rightDescriptors = std::move(rightDescriptors);
    outData.matchedStereoPairs = true;
    return true;
}

std::vector<StereoMatchPair> MatchStereoPairs(const XFeatFeatureSet &left, const XFeatFeatureSet &right,
                                              const cv::Mat &leftGray, const cv::Mat &rightGray)
{
    std::vector<StereoMatchPair> matches;
    if (!HasValidXFeatDescriptors(left) || !HasValidXFeatDescriptors(right) || left.keypoints.empty() ||
        right.keypoints.empty()) {
        return matches;
    }

    cv::Mat leftGray32f;
    cv::Mat rightGray32f;
    leftGray.convertTo(leftGray32f, CV_32F);
    rightGray.convertTo(rightGray32f, CV_32F);

    std::vector<StereoMatchPair> bestForLeft(static_cast<size_t>(left.descriptors.rows));
    std::vector<int> bestLeftForRight(static_cast<size_t>(right.descriptors.rows), -1);
    std::vector<float> bestLeftScore(static_cast<size_t>(right.descriptors.rows), -std::numeric_limits<float>::infinity());
    std::vector<float> bestLeftZncc(static_cast<size_t>(right.descriptors.rows), -1.0f);

    for (int li = 0; li < left.descriptors.rows; ++li) {
        const cv::Point2f &leftPt = left.keypoints[static_cast<size_t>(li)];
        int bestRi = -1;
        float bestScore = -std::numeric_limits<float>::infinity();
        float secondScore = -std::numeric_limits<float>::infinity();
        float bestZncc = -1.0f;
        float bestDisparity = 0.0f;

        for (int ri = 0; ri < right.descriptors.rows; ++ri) {
            const cv::Point2f &rightPt = right.keypoints[static_cast<size_t>(ri)];
            const float yDelta = std::fabs(leftPt.y - rightPt.y);
            const float disparity = leftPt.x - rightPt.x;
            if (yDelta > kStereoMaxEpipolarDeltaPx || disparity < kStereoMinDisparityPx ||
                disparity > kStereoMaxDisparityPx) {
                continue;
            }

            float score = -std::numeric_limits<float>::infinity();
            if (!DescriptorSimilarity(left.descriptors.row(li), right.descriptors.row(ri), score)) {
                continue;
            }
            if (score > bestScore) {
                secondScore = bestScore;
                bestScore = score;
                bestRi = ri;
                bestDisparity = disparity;
            } else if (score > secondScore) {
                secondScore = score;
            }
        }

        if (bestRi < 0 || !std::isfinite(bestScore) || bestScore < kStereoMinDescriptorSimilarity) {
            continue;
        }
        if (std::isfinite(secondScore) &&
            bestScore < secondScore / kStereoSimilarityRatioTest) {
            continue;
        }

        float zncc = -1.0f;
        if (!ComputePatchZncc(leftGray32f, leftPt, rightGray32f, right.keypoints[static_cast<size_t>(bestRi)], zncc) ||
            zncc < kStereoMinZnccScore) {
            continue;
        }

        bestZncc = zncc;
        const float epipolarError = std::fabs(leftPt.y - right.keypoints[static_cast<size_t>(bestRi)].y);
        const float quality = ComputeStereoCandidateQuality(bestScore, bestZncc, epipolarError, bestDisparity);
        if (quality < kLkMinCandidateQuality) {
            continue;
        }
        bestForLeft[static_cast<size_t>(li)] = StereoMatchPair{li, bestRi, bestScore, bestZncc, bestDisparity, quality};

        if (IsBetterRightCandidate(bestScore, bestZncc, bestLeftScore[static_cast<size_t>(bestRi)],
                                   bestLeftZncc[static_cast<size_t>(bestRi)])) {
            bestLeftScore[static_cast<size_t>(bestRi)] = bestScore;
            bestLeftZncc[static_cast<size_t>(bestRi)] = bestZncc;
            bestLeftForRight[static_cast<size_t>(bestRi)] = li;
        }
    }

    matches.reserve(static_cast<size_t>(std::min(left.descriptors.rows, right.descriptors.rows)));
    for (size_t li = 0; li < bestForLeft.size(); ++li) {
        const StereoMatchPair &pair = bestForLeft[li];
        if (pair.rightIndex < 0) {
            continue;
        }
        if (bestLeftForRight[static_cast<size_t>(pair.rightIndex)] != pair.leftIndex) {
            continue;
        }
        matches.push_back(pair);
    }

    std::sort(matches.begin(), matches.end(), [](const StereoMatchPair &a, const StereoMatchPair &b) {
        if (a.descriptorScore != b.descriptorScore) {
            return a.descriptorScore > b.descriptorScore;
        }
        if (a.zncc != b.zncc) {
            return a.zncc > b.zncc;
        }
        if (a.quality != b.quality) {
            return a.quality > b.quality;
        }
        return a.disparity < b.disparity;
    });
    return SelectGridBalancedPairs(matches, left.keypoints, leftGray.cols, leftGray.rows);
}

std::vector<cv::Point2f> ToPointList(const std::vector<cv::KeyPoint> &keypoints)
{
    std::vector<cv::Point2f> points;
    points.reserve(keypoints.size());
    for (const cv::KeyPoint &kp : keypoints) {
        points.push_back(kp.pt);
    }
    return points;
}

cv::Mat BuildXFeatInputImage(const cv::Mat &gray, int maxWidth, int maxHeight, float &scaleX, float &scaleY)
{
    scaleX = 1.0f;
    scaleY = 1.0f;
    if (gray.empty()) {
        return gray;
    }

    const int srcWidth = gray.cols;
    const int srcHeight = gray.rows;
    const float widthScale = maxWidth > 0 ? static_cast<float>(maxWidth) / static_cast<float>(std::max(1, srcWidth))
                                          : std::numeric_limits<float>::infinity();
    const float heightScale =
        maxHeight > 0 ? static_cast<float>(maxHeight) / static_cast<float>(std::max(1, srcHeight))
                      : std::numeric_limits<float>::infinity();
    const float resizeScale = std::min(1.0f, std::min(widthScale, heightScale));
    if (resizeScale >= 0.999f) {
        return gray;
    }

    const int targetWidth = std::max(32, static_cast<int>(std::lround(static_cast<float>(srcWidth) * resizeScale)));
    const int targetHeight = std::max(32, static_cast<int>(std::lround(static_cast<float>(srcHeight) * resizeScale)));
    cv::Mat resized;
    cv::resize(gray, resized, cv::Size(targetWidth, targetHeight), 0.0, 0.0, cv::INTER_AREA);
    scaleX = static_cast<float>(srcWidth) / static_cast<float>(targetWidth);
    scaleY = static_cast<float>(srcHeight) / static_cast<float>(targetHeight);
    return resized;
}

void RemapKeypointsToSource(std::vector<cv::Point2f> &keypoints, float scaleX, float scaleY)
{
    if (scaleX == 1.0f && scaleY == 1.0f) {
        return;
    }
    for (cv::Point2f &pt : keypoints) {
        pt.x *= scaleX;
        pt.y *= scaleY;
    }
}

int LkGridCellForPoint(const cv::Point2f &pt, const cv::Size &size)
{
    if (size.width <= 0 || size.height <= 0 || pt.x < 0.0f || pt.y < 0.0f ||
        pt.x >= static_cast<float>(size.width) || pt.y >= static_cast<float>(size.height)) {
        return -1;
    }
    const int cellWidth = std::max(1, (size.width + kLkGridCols - 1) / kLkGridCols);
    const int cellHeight = std::max(1, (size.height + kLkGridRows - 1) / kLkGridRows);
    const int col = std::clamp(static_cast<int>(pt.x) / cellWidth, 0, kLkGridCols - 1);
    const int row = std::clamp(static_cast<int>(pt.y) / cellHeight, 0, kLkGridRows - 1);
    return row * kLkGridCols + col;
}

std::array<int, kLkGridCellCount> CountLkTracksByCell(const std::vector<LkStereoTrack> &tracks, const cv::Size &size)
{
    std::array<int, kLkGridCellCount> counts{};
    for (const LkStereoTrack &track : tracks) {
        const int cell = LkGridCellForPoint(track.left, size);
        if (cell >= 0) {
            ++counts[static_cast<size_t>(cell)];
        }
    }
    return counts;
}

std::vector<LkStereoTrack> SelectLkTracksGridBalanced(const std::vector<LkStereoTrack> &tracks, const cv::Size &size)
{
    if (tracks.empty() || size.area() <= 0) {
        return tracks;
    }

    std::vector<LkStereoTrack> ranked = tracks;
    std::sort(ranked.begin(), ranked.end(), [](const LkStereoTrack &lhs, const LkStereoTrack &rhs) {
        if (lhs.quality != rhs.quality) {
            return lhs.quality > rhs.quality;
        }
        return lhs.age < rhs.age;
    });

    std::array<int, kLkGridCellCount> counts{};
    std::vector<LkStereoTrack> selected;
    selected.reserve(std::min(kLkMaxTracks, ranked.size()));
    for (const LkStereoTrack &track : ranked) {
        if (selected.size() >= kLkMaxTracks) {
            break;
        }
        const int cell = LkGridCellForPoint(track.left, size);
        if (cell < 0) {
            continue;
        }
        int &cellCount = counts[static_cast<size_t>(cell)];
        if (cellCount >= kLkTargetTracksPerCell) {
            continue;
        }
        selected.push_back(track);
        ++cellCount;
    }
    return selected;
}

bool LkTrackNearExisting(const cv::Point2f &left, const cv::Point2f &right, const std::vector<LkStereoTrack> &tracks)
{
    const float minDistSq = kLkMinSeedDistancePx * kLkMinSeedDistancePx;
    for (const LkStereoTrack &track : tracks) {
        const cv::Point2f leftDelta = track.left - left;
        const cv::Point2f rightDelta = track.right - right;
        if ((leftDelta.x * leftDelta.x + leftDelta.y * leftDelta.y) <= minDistSq &&
            (rightDelta.x * rightDelta.x + rightDelta.y * rightDelta.y) <= minDistSq) {
            return true;
        }
    }
    return false;
}

std::vector<LkStereoTrack> BuildLkXFeatStereoSeeds(XFeatFrontendClient *client, const cv::Mat &leftGray,
                                                   const cv::Mat &rightGray, int maxWidth, int maxHeight,
                                                   XFeatFrontendClient::Stats *stats, double *matchMs)
{
    std::vector<LkStereoTrack> seeds;
    if (client == nullptr || !client->Running() || leftGray.empty() || rightGray.empty()) {
        return seeds;
    }

    float leftScaleX = 1.0f;
    float leftScaleY = 1.0f;
    float rightScaleX = 1.0f;
    float rightScaleY = 1.0f;
    const cv::Mat leftInput = BuildXFeatInputImage(leftGray, maxWidth, maxHeight, leftScaleX, leftScaleY);
    const cv::Mat rightInput = BuildXFeatInputImage(rightGray, maxWidth, maxHeight, rightScaleX, rightScaleY);
    XFeatFeatureSet leftFeatures;
    XFeatFeatureSet rightFeatures;
    std::string err;
    if (!client->DetectAndComputeStereo(leftInput, rightInput, leftFeatures, rightFeatures, &err)) {
        return seeds;
    }
    if (stats != nullptr) {
        *stats = client->LastStats();
    }
    RemapKeypointsToSource(leftFeatures.keypoints, leftScaleX, leftScaleY);
    RemapKeypointsToSource(rightFeatures.keypoints, rightScaleX, rightScaleY);

    const auto matchStartTp = std::chrono::steady_clock::now();
    const std::vector<StereoMatchPair> rawMatches = MatchStereoPairs(leftFeatures, rightFeatures, leftGray, rightGray);
    const std::vector<StereoMatchPair> matches = FilterStereoPairsByDisparityConsistency(rawMatches);
    if (matchMs != nullptr) {
        *matchMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - matchStartTp).count();
    }
    seeds.reserve(matches.size());
    for (const StereoMatchPair &match : matches) {
        if (match.leftIndex < 0 || match.rightIndex < 0 ||
            static_cast<size_t>(match.leftIndex) >= leftFeatures.keypoints.size() ||
            static_cast<size_t>(match.rightIndex) >= rightFeatures.keypoints.size()) {
            continue;
        }
        seeds.push_back(LkStereoTrack{leftFeatures.keypoints[static_cast<size_t>(match.leftIndex)],
                                      rightFeatures.keypoints[static_cast<size_t>(match.rightIndex)], match.quality,
                                      0});
    }
    std::sort(seeds.begin(), seeds.end(), [](const LkStereoTrack &lhs, const LkStereoTrack &rhs) {
        return lhs.quality > rhs.quality;
    });
    return seeds;
}

LkXFeatSeedResult BuildLkXFeatStereoSeedResult(uint64_t frameId, XFeatFrontendClient *client, const cv::Mat &leftGray,
                                               const cv::Mat &rightGray, int maxWidth, int maxHeight)
{
    LkXFeatSeedResult result{};
    result.frameId = frameId;
    const auto startTp = std::chrono::steady_clock::now();
    result.seeds = BuildLkXFeatStereoSeeds(client, leftGray, rightGray, maxWidth, maxHeight, &result.stats,
                                           &result.matchMs);
    result.totalMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - startTp).count();
    return result;
}

void PushLkFrameSnapshot(std::deque<LkFrameSnapshot> &history, uint64_t frameId, const cv::Mat &left,
                         const cv::Mat &right)
{
    if (left.empty() || right.empty()) {
        return;
    }
    if (!history.empty() && history.back().frameId == frameId) {
        history.back().left = left.clone();
        history.back().right = right.clone();
        return;
    }
    history.push_back(LkFrameSnapshot{frameId, left.clone(), right.clone()});
    while (history.size() > kLkFrameHistoryMaxSize) {
        history.pop_front();
    }
}

std::vector<LkStereoTrack> TrackLkSeedsToFrame(const LkXFeatSeedResult &result,
                                               const std::deque<LkFrameSnapshot> &history, uint64_t currentFrameId)
{
    if (result.seeds.empty() || history.empty()) {
        return {};
    }

    size_t startIndex = history.size();
    size_t endIndex = history.size();
    for (size_t i = 0; i < history.size(); ++i) {
        if (history[i].frameId == result.frameId) {
            startIndex = i;
        }
        if (history[i].frameId == currentFrameId) {
            endIndex = i;
        }
    }
    if (startIndex >= history.size() || endIndex >= history.size() || startIndex > endIndex) {
        return {};
    }
    if (startIndex == endIndex) {
        return result.seeds;
    }

    std::vector<LkStereoTrack> tracks = result.seeds;
    for (size_t frameIndex = startIndex + 1; frameIndex <= endIndex && !tracks.empty(); ++frameIndex) {
        const LkFrameSnapshot &prev = history[frameIndex - 1];
        const LkFrameSnapshot &curr = history[frameIndex];
        std::vector<cv::Point2f> prevLeft;
        std::vector<cv::Point2f> prevRight;
        prevLeft.reserve(tracks.size());
        prevRight.reserve(tracks.size());
        for (const LkStereoTrack &track : tracks) {
            prevLeft.push_back(track.left);
            prevRight.push_back(track.right);
        }

        std::vector<cv::Point2f> currLeft;
        std::vector<cv::Point2f> currRight;
        std::vector<uchar> leftStatus;
        std::vector<uchar> rightStatus;
        if (!TrackPointsWithForwardBackward(prev.left, curr.left, prevLeft, currLeft, leftStatus) ||
            !TrackPointsWithForwardBackward(prev.right, curr.right, prevRight, currRight, rightStatus)) {
            return {};
        }

        cv::Mat currLeft32f;
        cv::Mat currRight32f;
        curr.left.convertTo(currLeft32f, CV_32F);
        curr.right.convertTo(currRight32f, CV_32F);

        std::vector<LkStereoTrack> nextTracks;
        nextTracks.reserve(tracks.size());
        for (size_t i = 0; i < tracks.size() && i < currLeft.size() && i < currRight.size(); ++i) {
            if (i >= leftStatus.size() || i >= rightStatus.size() || !leftStatus[i] || !rightStatus[i]) {
                continue;
            }
            const cv::Point2f &left = currLeft[i];
            const cv::Point2f &right = currRight[i];
            if (!IsStereoPairGeometricallyValid(left, right)) {
                continue;
            }
            float zncc = -1.0f;
            if (!ComputePatchZncc(currLeft32f, left, currRight32f, right, zncc) ||
                zncc < kTemporalStereoMinZnccScore) {
                continue;
            }
            const float quality =
                std::clamp(tracks[i].quality * 0.90f + std::clamp((zncc + 1.0f) * 0.5f, 0.0f, 1.0f) * 0.10f,
                           0.0f, 1.0f);
            nextTracks.push_back(LkStereoTrack{left, right, quality, tracks[i].age + 1});
        }
        tracks = SelectLkTracksGridBalanced(nextTracks, curr.left.size());
    }
    return tracks;
}

void AppendLkSeedsForDegradedCells(const std::vector<LkStereoTrack> &seeds, const cv::Size &size,
                                   std::vector<LkStereoTrack> &tracks)
{
    if (seeds.empty() || size.area() <= 0) {
        return;
    }
    auto counts = CountLkTracksByCell(tracks, size);
    for (const LkStereoTrack &seed : seeds) {
        if (tracks.size() >= kLkMaxTracks) {
            return;
        }
        const int cell = LkGridCellForPoint(seed.left, size);
        if (cell < 0) {
            continue;
        }
        int &cellCount = counts[static_cast<size_t>(cell)];
        if (cellCount >= kLkTargetTracksPerCell) {
            continue;
        }
        if (cellCount >= kLkMinTracksPerCell && tracks.size() > kLkGridCellCount * kLkMinTracksPerCell) {
            continue;
        }
        if (LkTrackNearExisting(seed.left, seed.right, tracks)) {
            continue;
        }
        tracks.push_back(seed);
        ++cellCount;
    }
}

bool IsXFeatTrackingStateSafe(int trackingState)
{
    switch (trackingState) {
    case ORB_SLAM3::Tracking::OK:
    case ORB_SLAM3::Tracking::RECENTLY_LOST:
    case ORB_SLAM3::Tracking::LOST:
    case ORB_SLAM3::Tracking::OK_KLT:
        return true;
    default:
        return false;
    }
}

bool IsOrbBootstrapState(int trackingState)
{
    return trackingState == ORB_SLAM3::Tracking::NO_IMAGES_YET ||
           trackingState == ORB_SLAM3::Tracking::NOT_INITIALIZED;
}

bool IsIdentityPose(const smartdrone::core::ports::PoseEstimate &pose)
{
    return pose.valid && pose.x == 0.0f && pose.y == 0.0f && pose.z == 0.0f && pose.qw == 1.0f &&
           pose.qx == 0.0f && pose.qy == 0.0f && pose.qz == 0.0f;
}

bool IsFinitePose(const smartdrone::core::ports::PoseEstimate &pose)
{
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z) && std::isfinite(pose.qw) &&
           std::isfinite(pose.qx) && std::isfinite(pose.qy) && std::isfinite(pose.qz);
}

void NormalizePoseQuaternion(smartdrone::core::ports::PoseEstimate &pose)
{
    const float qNorm =
        std::sqrt(pose.qw * pose.qw + pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz);
    if (qNorm > 1.0e-6f && std::isfinite(qNorm)) {
        pose.qw /= qNorm;
        pose.qx /= qNorm;
        pose.qy /= qNorm;
        pose.qz /= qNorm;
    }
}

float PoseTranslationDistance(const smartdrone::core::ports::PoseEstimate &a,
                              const smartdrone::core::ports::PoseEstimate &b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

float ClampVelocityComponent(float value, float scale) { return value * scale; }

void ClampVelocityVector(float &vx, float &vy, float &vz)
{
    const float speed = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (speed > kPoseStabilizerMaxSpeedMps && speed > 1.0e-6f) {
        const float scale = kPoseStabilizerMaxSpeedMps / speed;
        vx = ClampVelocityComponent(vx, scale);
        vy = ClampVelocityComponent(vy, scale);
        vz = ClampVelocityComponent(vz, scale);
    }
}

Eigen::Quaternionf PoseQuaternion(const smartdrone::core::ports::PoseEstimate &pose)
{
    Eigen::Quaternionf q(pose.qw, pose.qx, pose.qy, pose.qz);
    q.normalize();
    return q;
}

float QuaternionAngleDeg(const Eigen::Quaternionf &a, const Eigen::Quaternionf &b)
{
    const float dot = std::min(1.0f, std::max(-1.0f, std::abs(a.dot(b))));
    return 2.0f * std::acos(dot) * 180.0f / static_cast<float>(M_PI);
}

void LimitPoseRotationStep(const smartdrone::core::ports::PoseEstimate &reference,
                           smartdrone::core::ports::PoseEstimate &pose)
{
    const Eigen::Quaternionf qa = PoseQuaternion(reference);
    Eigen::Quaternionf qb = PoseQuaternion(pose);
    if (qa.dot(qb) < 0.0f) {
        qb.coeffs() *= -1.0f;
    }

    const float angleDeg = QuaternionAngleDeg(qa, qb);
    if (angleDeg > kPoseStabilizerMaxRotStepDeg && angleDeg > 1.0e-6f) {
        const float t = kPoseStabilizerMaxRotStepDeg / angleDeg;
        qb = qa.slerp(t, qb);
    }

    qb.normalize();
    pose.qw = qb.w();
    pose.qx = qb.x();
    pose.qy = qb.y();
    pose.qz = qb.z();
}

std::string DescribeTrackingState(int trackingState)
{
    switch (trackingState) {
    case ORB_SLAM3::Tracking::SYSTEM_NOT_READY:
        return "system_not_ready";
    case ORB_SLAM3::Tracking::NO_IMAGES_YET:
        return "no_images_yet";
    case ORB_SLAM3::Tracking::NOT_INITIALIZED:
        return "not_initialized";
    case ORB_SLAM3::Tracking::OK:
        return "ok";
    case ORB_SLAM3::Tracking::RECENTLY_LOST:
        return "recently_lost";
    case ORB_SLAM3::Tracking::LOST:
        return "lost";
    case ORB_SLAM3::Tracking::OK_KLT:
        return "ok_klt";
    default:
        return "unknown";
    }
}

} // namespace

OrbSlam3Engine::OrbSlam3Engine(std::unique_ptr<ORB_SLAM3::System> system, OrbInputMode inputMode, bool useImu,
                               std::string settingsPath)
    : m_system(std::move(system)), m_inputMode(inputMode), m_useImu(useImu), m_settingsPath(std::move(settingsPath))
{
    m_lkCalibrationLoaded = LoadLkCalibration(m_settingsPath);
}

bool OrbSlam3Engine::Start()
{
    if (m_lkXFeatFuture.valid()) {
        m_lkXFeatFuture.wait();
        m_lkXFeatFuture = std::future<LkXFeatSeedResult>{};
    }
    m_prevStereoLeftGray.release();
    m_prevStereoRightGray.release();
    m_prevInjectedStereoLeftPoints.clear();
    m_prevInjectedStereoRightPoints.clear();
    m_prevInjectedStereoExternal = ORB_SLAM3::ExternalStereoFrameData{};
    m_havePrevInjectedStereoExternal = false;
    m_lastStablePose = core::ports::PoseEstimate{};
    m_haveLastStablePose = false;
    m_lastStableTimestampSec = 0.0;
    m_lkTracks.clear();
    m_lkFrameHistory.clear();
    m_lkXFeatPendingFrameId = 0;
    m_stableVelX = 0.0f;
    m_stableVelY = 0.0f;
    m_stableVelZ = 0.0f;
    m_lkPrevLeft.release();
    m_lkPrevRight.release();
    m_lkTwc = Sophus::SE3f();
    m_lkHavePrev = false;
    m_lkFrameCount = 0;
    return static_cast<bool>(m_system);
}

void OrbSlam3Engine::SetOperationMode(core::domain::SlamOperationMode mode)
{
    if (!m_system || m_operationMode == mode) {
        return;
    }

    const bool localizationOnly = mode == core::domain::SlamOperationMode::Localization ||
                                  mode == core::domain::SlamOperationMode::Relocalization ||
                                  mode == core::domain::SlamOperationMode::TrackingOnly;

    if (localizationOnly) {
        m_system->ActivateLocalizationMode();
    } else {
        m_system->DeactivateLocalizationMode();
    }
    m_operationMode = mode;
}

void OrbSlam3Engine::SetFeatureFrontend(FeatureFrontend frontend)
{
    if (m_featureFrontend != frontend) {
        if (m_lkXFeatFuture.valid()) {
            m_lkXFeatFuture.wait();
            m_lkXFeatFuture = std::future<LkXFeatSeedResult>{};
        }
        m_prevStereoLeftGray.release();
        m_prevStereoRightGray.release();
        m_prevInjectedStereoLeftPoints.clear();
        m_prevInjectedStereoRightPoints.clear();
        m_prevInjectedStereoExternal = ORB_SLAM3::ExternalStereoFrameData{};
        m_havePrevInjectedStereoExternal = false;
        m_lkPrevLeft.release();
        m_lkPrevRight.release();
        m_lkTracks.clear();
        m_lkFrameHistory.clear();
        m_lkXFeatPendingFrameId = 0;
        m_lkTwc = Sophus::SE3f();
        m_lkHavePrev = false;
        m_lkFrameCount = 0;
    }
    m_featureFrontend = frontend;
}

void OrbSlam3Engine::SetXFeatFrontendClient(XFeatFrontendClient *client) { m_xfeatFrontendClient = client; }

void OrbSlam3Engine::SetXFeatInputSizeLimit(int maxWidth, int maxHeight)
{
    m_xfeatInputMaxWidth = std::max(0, maxWidth);
    m_xfeatInputMaxHeight = std::max(0, maxHeight);
}

bool OrbSlam3Engine::LoadLkCalibration(const std::string &settingsPath)
{
    if (settingsPath.empty()) {
        std::cerr << "[lk_vo] settings path empty; stereo VO disabled\n";
        return false;
    }
    cv::FileStorage fs(settingsPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[lk_vo] failed to open settings: " << settingsPath << "\n";
        return false;
    }

    const float fx1 = static_cast<float>(fs["Camera1.fx"]);
    const float fy1 = static_cast<float>(fs["Camera1.fy"]);
    const float cx1 = static_cast<float>(fs["Camera1.cx"]);
    const float cy1 = static_cast<float>(fs["Camera1.cy"]);
    const float fx2 = static_cast<float>(fs["Camera2.fx"]);
    const float fy2 = static_cast<float>(fs["Camera2.fy"]);
    const float cx2 = static_cast<float>(fs["Camera2.cx"]);
    const float cy2 = static_cast<float>(fs["Camera2.cy"]);
    if (!(fx1 > 0.0f) || !(fy1 > 0.0f) || !(fx2 > 0.0f) || !(fy2 > 0.0f)) {
        std::cerr << "[lk_vo] invalid camera intrinsics in settings\n";
        return false;
    }

    m_lkK1 = MakeCameraMatrix(fx1, fy1, cx1, cy1);
    m_lkK2 = MakeCameraMatrix(fx2, fy2, cx2, cy2);
    m_lkD1 = MakeDistCoeffs(static_cast<float>(fs["Camera1.k1"]), static_cast<float>(fs["Camera1.k2"]),
                               static_cast<float>(fs["Camera1.p1"]), static_cast<float>(fs["Camera1.p2"]));
    m_lkD2 = MakeDistCoeffs(static_cast<float>(fs["Camera2.k1"]), static_cast<float>(fs["Camera2.k2"]),
                               static_cast<float>(fs["Camera2.p1"]), static_cast<float>(fs["Camera2.p2"]));
    fs["Stereo.T_c1_c2"] >> m_lkTc1c2;
    if (m_lkTc1c2.empty() || m_lkTc1c2.rows != 4 || m_lkTc1c2.cols != 4) {
        std::cerr << "[lk_vo] Stereo.T_c1_c2 missing; falling back to Camera.bf baseline\n";
    }
    const float bf = static_cast<float>(fs["Camera.bf"]);
    m_lkBaseline = bf > 0.0f ? bf / fx1 : 0.0f;
    if (!m_lkTc1c2.empty()) {
        cv::Mat T64;
        m_lkTc1c2.convertTo(T64, CV_64F);
        m_lkBaseline = std::abs(static_cast<float>(T64.at<double>(0, 3)));
    }
    if (!(m_lkBaseline > 0.005f)) {
        std::cerr << "[lk_vo] invalid stereo baseline\n";
        return false;
    }
    m_lkFx = fx1;
    m_lkFy = fy1;
    m_lkCx = cx1;
    m_lkCy = cy1;
    std::cerr << "[lk_vo] calibration loaded fx=" << m_lkFx << " fy=" << m_lkFy
              << " baseline=" << m_lkBaseline << "\n";
    return true;
}

void OrbSlam3Engine::EnsureLkRectifier(const cv::Size &inputSize)
{
    if (!m_lkCalibrationLoaded || inputSize.area() <= 0 || m_lkRectifierSize == inputSize) {
        return;
    }

    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t = (cv::Mat_<double>(3, 1) << -static_cast<double>(m_lkBaseline), 0.0, 0.0);
    if (!m_lkTc1c2.empty()) {
        cv::Mat T64;
        m_lkTc1c2.convertTo(T64, CV_64F);
        R = T64(cv::Rect(0, 0, 3, 3)).clone();
        t = T64(cv::Rect(3, 0, 1, 3)).clone();
    }

    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(m_lkK1, m_lkD1, m_lkK2, m_lkD2, inputSize, R, t, R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, -1.0, inputSize);
    cv::initUndistortRectifyMap(m_lkK1, m_lkD1, R1, P1, inputSize, CV_32FC1, m_lkMap1x, m_lkMap1y);
    cv::initUndistortRectifyMap(m_lkK2, m_lkD2, R2, P2, inputSize, CV_32FC1, m_lkMap2x, m_lkMap2y);
    m_lkFx = static_cast<float>(P1.at<double>(0, 0));
    m_lkFy = static_cast<float>(P1.at<double>(1, 1));
    m_lkCx = static_cast<float>(P1.at<double>(0, 2));
    m_lkCy = static_cast<float>(P1.at<double>(1, 2));
    m_lkBaseline = std::abs(static_cast<float>(P2.at<double>(0, 3) / P2.at<double>(0, 0)));
    m_lkRectifierSize = inputSize;
}

core::ports::SlamOutput OrbSlam3Engine::ProcessLkStereoVo(const core::ports::SlamInputBatch &input,
                                                          bool extractFeatures)
{
    core::ports::SlamOutput out{};
    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    out.mapId = 1;
    out.trackingState = ORB_SLAM3::Tracking::OK;
    out.poseValid = true;
    out.pose.valid = true;
    out.usedXFeatFrontend = false;
    m_lastXFeatRawLeftCount = 0;
    m_lastXFeatRawRightCount = 0;
    m_lastXFeatMatchedStereoCount = 0;
    m_lastXFeatInjectedLeftCount = 0;
    m_lastXFeatInjectedRightCount = 0;
    m_lastXFeatPrepareMs = 0.0;
    m_lastXFeatWorkerWriteMs = 0.0;
    m_lastXFeatWorkerReadMs = 0.0;
    m_lastXFeatWorkerTotalMs = 0.0;
    m_lastXFeatStereoMatchMs = 0.0;
    m_lastXFeatTotalMs = 0.0;
    m_lastXFeatImageCount = 0;
    m_lastXFeatPayloadBytes = 0;

    cv::Mat leftGray = EnsureGray8(input.stereo.left.gray);
    cv::Mat rightGray = EnsureGray8(input.stereo.right.gray);
    if (leftGray.empty() || rightGray.empty() || !m_lkCalibrationLoaded) {
        out.trackingState = ORB_SLAM3::Tracking::LOST;
        out.poseValid = false;
        out.pose.valid = false;
        return out;
    }

    EnsureLkRectifier(leftGray.size());
    cv::Mat leftRect = leftGray;
    cv::Mat rightRect = rightGray;
    if (!m_lkMap1x.empty() && !m_lkMap2x.empty()) {
        cv::remap(leftGray, leftRect, m_lkMap1x, m_lkMap1y, cv::INTER_LINEAR);
        cv::remap(rightGray, rightRect, m_lkMap2x, m_lkMap2y, cv::INTER_LINEAR);
    }
    PushLkFrameSnapshot(m_lkFrameHistory, input.frameId, leftRect, rightRect);

    auto consumeReadyXFeatSeeds = [&]() {
        if (!m_lkXFeatFuture.valid() ||
            m_lkXFeatFuture.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
            return false;
        }
        LkXFeatSeedResult result = m_lkXFeatFuture.get();
        m_lkXFeatPendingFrameId = 0;
        m_lastXFeatPrepareMs = result.stats.prepareMs;
        m_lastXFeatWorkerWriteMs = result.stats.writeMs;
        m_lastXFeatWorkerReadMs = result.stats.readMs;
        m_lastXFeatWorkerTotalMs = result.stats.totalMs;
        m_lastXFeatStereoMatchMs = result.matchMs;
        m_lastXFeatImageCount = result.stats.imageCount;
        m_lastXFeatPayloadBytes = result.stats.payloadBytes;
        m_lastXFeatRawLeftCount = static_cast<int>(result.seeds.size());
        m_lastXFeatRawRightCount = static_cast<int>(result.seeds.size());
        m_lastXFeatMatchedStereoCount = static_cast<int>(result.seeds.size());
        if (result.seeds.empty()) {
            return false;
        }
        if (m_lkTracks.empty() && input.frameId > result.frameId + kLkAsyncMaxSeedTrackFrames) {
            std::cerr << "[lk_vo] discard stale async xfeat seeds source_frame=" << result.frameId
                      << " current_frame=" << input.frameId << " seeds=" << result.seeds.size() << "\n";
            return false;
        }

        std::vector<LkStereoTrack> seeds = TrackLkSeedsToFrame(result, m_lkFrameHistory, input.frameId);
        if (seeds.empty()) {
            return false;
        }
        const size_t before = m_lkTracks.size();
        AppendLkSeedsForDegradedCells(seeds, leftRect.size(), m_lkTracks);
        m_lastXFeatInjectedLeftCount = static_cast<int>(m_lkTracks.size() - before);
        m_lastXFeatInjectedRightCount = m_lastXFeatInjectedLeftCount;
        m_lastXFeatTotalMs = result.totalMs;
        out.usedXFeatFrontend = m_lastXFeatInjectedLeftCount > 0;
        return m_lastXFeatInjectedLeftCount > 0;
    };

    auto maybeLaunchXFeat = [&]() {
        if (m_lkXFeatFuture.valid() || m_xfeatFrontendClient == nullptr || !m_xfeatFrontendClient->Running()) {
            return false;
        }
        XFeatFrontendClient *client = m_xfeatFrontendClient;
        const cv::Mat asyncLeft = leftRect.clone();
        const cv::Mat asyncRight = rightRect.clone();
        const int maxWidth = m_xfeatInputMaxWidth;
        const int maxHeight = m_xfeatInputMaxHeight;
        const uint64_t frameId = input.frameId;
        m_lkXFeatPendingFrameId = frameId;
        m_lkXFeatFuture = std::async(std::launch::async, [client, frameId, asyncLeft, asyncRight, maxWidth, maxHeight]() {
            return BuildLkXFeatStereoSeedResult(frameId, client, asyncLeft, asyncRight, maxWidth, maxHeight);
        });
        return true;
    };
    consumeReadyXFeatSeeds();

    if (!m_lkHavePrev) {
        maybeLaunchXFeat();
        m_lkTracks = SelectLkTracksGridBalanced(m_lkTracks, leftRect.size());
        m_lkPrevLeft = leftRect.clone();
        m_lkPrevRight = rightRect.clone();
        m_lkTwc = Sophus::SE3f();
        m_lkHavePrev = true;
        m_lkFrameCount = 1;
        if (extractFeatures) {
            out.leftFeatures.reserve(m_lkTracks.size());
            out.rightFeatures.reserve(m_lkTracks.size());
            for (const LkStereoTrack &track : m_lkTracks) {
                out.leftFeatures.push_back(track.left);
                out.rightFeatures.push_back(track.right);
            }
        }
    } else {
        m_lkTracks = SelectLkTracksGridBalanced(m_lkTracks, m_lkPrevLeft.size());
        std::vector<cv::Point2f> pts0;
        pts0.reserve(m_lkTracks.size());
        for (const LkStereoTrack &track : m_lkTracks) {
            pts0.push_back(track.left);
        }
        std::vector<cv::Point2f> leftPts1;
        std::vector<cv::Point2f> rightPts1;
        std::vector<uchar> status;
        std::vector<uchar> rightStatus;
        if (!pts0.empty()) {
            std::vector<cv::Point2f> rightPts0;
            rightPts0.reserve(m_lkTracks.size());
            for (const LkStereoTrack &track : m_lkTracks) {
                rightPts0.push_back(track.right);
            }
            (void)TrackPointsWithForwardBackward(m_lkPrevLeft, leftRect, pts0, leftPts1, status);
            (void)TrackPointsWithForwardBackward(m_lkPrevRight, rightRect, rightPts0, rightPts1, rightStatus);
        }

        std::vector<cv::Point3f> objectPoints;
        std::vector<cv::Point2f> imagePoints;
        objectPoints.reserve(pts0.size());
        imagePoints.reserve(pts0.size());
        std::vector<LkStereoTrack> trackedTracks;
        trackedTracks.reserve(m_lkTracks.size());
        cv::Mat leftRect32f;
        cv::Mat rightRect32f;
        leftRect.convertTo(leftRect32f, CV_32F);
        rightRect.convertTo(rightRect32f, CV_32F);
        for (size_t i = 0; i < m_lkTracks.size() && i < leftPts1.size() && i < rightPts1.size(); ++i) {
            if (i >= status.size() || i >= rightStatus.size() || !status[i] || !rightStatus[i]) {
                continue;
            }
            const LkStereoTrack &prevTrack = m_lkTracks[i];
            const cv::Point2f &p0 = prevTrack.left;
            const cv::Point2f &prevRight = prevTrack.right;
            const cv::Point2f &p1 = leftPts1[i];
            const cv::Point2f &right1 = rightPts1[i];
            if (p0.x < 1.0f || p0.y < 1.0f || p0.x >= m_lkPrevLeft.cols - 1 || p0.y >= m_lkPrevLeft.rows - 1 ||
                p1.x < 1.0f || p1.y < 1.0f || p1.x >= leftRect.cols - 1 || p1.y >= leftRect.rows - 1 ||
                right1.x < 1.0f || right1.y < 1.0f || right1.x >= rightRect.cols - 1 ||
                right1.y >= rightRect.rows - 1) {
                continue;
            }
            if (cv::norm(p1 - p0) > kLkMaxFlowPx) {
                continue;
            }
            if (!IsStereoPairGeometricallyValid(p1, right1)) {
                continue;
            }
            float zncc = -1.0f;
            if (!ComputePatchZncc(leftRect32f, p1, rightRect32f, right1, zncc) || zncc < kTemporalStereoMinZnccScore) {
                continue;
            }
            const float d = p0.x - prevRight.x;
            const float z = m_lkFx * m_lkBaseline / d;
            if (!(z > 0.05f) || z > kLkMaxDepthMeters || !std::isfinite(z)) {
                continue;
            }
            objectPoints.emplace_back((p0.x - m_lkCx) * z / m_lkFx, (p0.y - m_lkCy) * z / m_lkFy, z);
            imagePoints.push_back(p1);
            const float trackedQuality =
                std::clamp(prevTrack.quality * 0.92f + std::clamp((zncc + 1.0f) * 0.5f, 0.0f, 1.0f) * 0.08f,
                           0.0f, 1.0f);
            trackedTracks.push_back(LkStereoTrack{p1, right1, trackedQuality, prevTrack.age + 1});
        }

        int inlierCount = 0;
        if (objectPoints.size() >= kLkMinPnPPoints) {
            cv::Mat rvec, tvec, inliers;
            const cv::Mat K = MakeCameraMatrix(m_lkFx, m_lkFy, m_lkCx, m_lkCy);
            const bool ok = cv::solvePnPRansac(objectPoints, imagePoints, K, cv::Mat(), rvec, tvec, false, 80, 4.0,
                                               0.995, inliers, cv::SOLVEPNP_EPNP);
            inlierCount = inliers.rows;
            if (ok && inlierCount >= kLkMinPnPInliers) {
                std::vector<LkStereoTrack> inlierTracks;
                inlierTracks.reserve(static_cast<size_t>(inlierCount));
                for (int row = 0; row < inliers.rows; ++row) {
                    const int idx = inliers.at<int>(row, 0);
                    if (idx >= 0 && static_cast<size_t>(idx) < trackedTracks.size()) {
                        inlierTracks.push_back(trackedTracks[static_cast<size_t>(idx)]);
                    }
                }
                if (!inlierTracks.empty()) {
                    trackedTracks = std::move(inlierTracks);
                }
                cv::Mat Rcv;
                cv::Rodrigues(rvec, Rcv);
                Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
                Eigen::Vector3f t = Eigen::Vector3f::Zero();
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        R(r, c) = static_cast<float>(Rcv.at<double>(r, c));
                    }
                    t(r) = static_cast<float>(tvec.at<double>(r, 0));
                }
                if (std::isfinite(t.norm()) && t.norm() <= kLkMaxStepMeters) {
                    const Sophus::SE3f TcurrPrev(Sophus::SO3f(R), t);
                    m_lkTwc = m_lkTwc * TcurrPrev.inverse();
                }
            }
        }

        out.matchesInliers = inlierCount;
        out.trackedMapPointCount = static_cast<uint32_t>(inlierCount);
        out.localMapPointCount = static_cast<uint32_t>(objectPoints.size());
        m_lkTracks = SelectLkTracksGridBalanced(trackedTracks, leftRect.size());
        consumeReadyXFeatSeeds();
        maybeLaunchXFeat();
        m_lkTracks = SelectLkTracksGridBalanced(m_lkTracks, leftRect.size());
        if (extractFeatures) {
            out.leftFeatures.reserve(m_lkTracks.size());
            out.rightFeatures.reserve(m_lkTracks.size());
            for (const LkStereoTrack &track : m_lkTracks) {
                out.leftFeatures.push_back(track.left);
                out.rightFeatures.push_back(track.right);
            }
        }
        m_lkPrevLeft = leftRect.clone();
        m_lkPrevRight = rightRect.clone();
        ++m_lkFrameCount;
    }

    const Eigen::Vector3f t = m_lkTwc.translation();
    out.usedXFeatFrontend = out.usedXFeatFrontend || m_lastXFeatInjectedLeftCount > 0;
    out.xfeatRawLeftCount = m_lastXFeatRawLeftCount;
    out.xfeatRawRightCount = m_lastXFeatRawRightCount;
    out.xfeatMatchedStereoCount = m_lastXFeatMatchedStereoCount;
    out.xfeatInjectedLeftCount = m_lastXFeatInjectedLeftCount;
    out.xfeatInjectedRightCount = m_lastXFeatInjectedRightCount;
    out.xfeatPrepareMs = m_lastXFeatPrepareMs;
    out.xfeatWorkerWriteMs = m_lastXFeatWorkerWriteMs;
    out.xfeatWorkerReadMs = m_lastXFeatWorkerReadMs;
    out.xfeatWorkerTotalMs = m_lastXFeatWorkerTotalMs;
    out.xfeatStereoMatchMs = m_lastXFeatStereoMatchMs;
    out.xfeatTotalMs = m_lastXFeatTotalMs;
    out.xfeatImageCount = m_lastXFeatImageCount;
    out.xfeatPayloadBytes = m_lastXFeatPayloadBytes;
    const Eigen::Quaternionf q(m_lkTwc.so3().unit_quaternion());
    out.pose.x = t.x();
    out.pose.y = t.y();
    out.pose.z = t.z();
    out.pose.qw = q.w();
    out.pose.qx = q.x();
    out.pose.qy = q.y();
    out.pose.qz = q.z();
    return out;
}

void OrbSlam3Engine::Stop()
{
    if (m_lkXFeatFuture.valid()) {
        m_lkXFeatFuture.wait();
        m_lkXFeatFuture = std::future<LkXFeatSeedResult>{};
    }
    m_prevStereoLeftGray.release();
    m_prevStereoRightGray.release();
    m_prevInjectedStereoLeftPoints.clear();
    m_prevInjectedStereoRightPoints.clear();
    m_prevInjectedStereoExternal = ORB_SLAM3::ExternalStereoFrameData{};
    m_havePrevInjectedStereoExternal = false;
    m_lastStablePose = core::ports::PoseEstimate{};
    m_haveLastStablePose = false;
    m_lastStableTimestampSec = 0.0;
    m_lkTracks.clear();
    m_lkFrameHistory.clear();
    m_lkXFeatPendingFrameId = 0;
    m_stableVelX = 0.0f;
    m_stableVelY = 0.0f;
    m_stableVelZ = 0.0f;
    if (m_system) {
        m_system->Shutdown();
    }
}

void OrbSlam3Engine::StabilizeOutputPose(core::ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
                                         int trackingState)
{
    pose.valid = poseValid && IsFinitePose(pose);
    if (pose.valid) {
        NormalizePoseQuaternion(pose);
    }

    double dt = kPoseStabilizerDefaultDtSec;
    if (m_haveLastStablePose && timestampSec > m_lastStableTimestampSec) {
        dt = std::clamp(timestampSec - m_lastStableTimestampSec, kPoseStabilizerMinDtSec, kPoseStabilizerMaxDtSec);
    }

    core::ports::PoseEstimate predicted = m_lastStablePose;
    if (m_haveLastStablePose) {
        ClampVelocityVector(m_stableVelX, m_stableVelY, m_stableVelZ);
        predicted.x += m_stableVelX * static_cast<float>(dt);
        predicted.y += m_stableVelY * static_cast<float>(dt);
        predicted.z += m_stableVelZ * static_cast<float>(dt);
        predicted.valid = true;
    }

    const bool rawIdentity = pose.valid && IsIdentityPose(pose);
    bool usePrediction = !pose.valid || rawIdentity;
    if (!usePrediction && m_haveLastStablePose) {
        const float rawStep = PoseTranslationDistance(pose, m_lastStablePose);
        const float maxStep =
            std::min(kPoseStabilizerMaxSpeedMps * static_cast<float>(dt), kPoseStabilizerMaxStepMeters);
        const bool rawPoseStuck = trackingState != ORB_SLAM3::Tracking::OK && rawStep < 1.0e-5f;
        usePrediction = rawPoseStuck || rawStep > maxStep;
        if (usePrediction && rawStep > maxStep && rawStep > 1.0e-6f) {
            const float scale = maxStep / rawStep;
            predicted.x = m_lastStablePose.x + (pose.x - m_lastStablePose.x) * scale;
            predicted.y = m_lastStablePose.y + (pose.y - m_lastStablePose.y) * scale;
            predicted.z = m_lastStablePose.z + (pose.z - m_lastStablePose.z) * scale;
            predicted.qw = m_lastStablePose.qw;
            predicted.qx = m_lastStablePose.qx;
            predicted.qy = m_lastStablePose.qy;
            predicted.qz = m_lastStablePose.qz;
            predicted.valid = true;
        }
    }

    if (usePrediction && m_haveLastStablePose) {
        pose = predicted;
        poseValid = true;
        m_stableVelX *= kPoseStabilizerPredictedVelocityDecay;
        m_stableVelY *= kPoseStabilizerPredictedVelocityDecay;
        m_stableVelZ *= kPoseStabilizerPredictedVelocityDecay;
    } else if (pose.valid) {
        if (m_haveLastStablePose) {
            float measuredVelX = (pose.x - m_lastStablePose.x) / static_cast<float>(dt);
            float measuredVelY = (pose.y - m_lastStablePose.y) / static_cast<float>(dt);
            float measuredVelZ = (pose.z - m_lastStablePose.z) / static_cast<float>(dt);
            ClampVelocityVector(measuredVelX, measuredVelY, measuredVelZ);
            LimitPoseRotationStep(m_lastStablePose, pose);
            m_stableVelX = (1.0f - kPoseStabilizerVelocityAlpha) * m_stableVelX +
                           kPoseStabilizerVelocityAlpha * measuredVelX;
            m_stableVelY = (1.0f - kPoseStabilizerVelocityAlpha) * m_stableVelY +
                           kPoseStabilizerVelocityAlpha * measuredVelY;
            m_stableVelZ = (1.0f - kPoseStabilizerVelocityAlpha) * m_stableVelZ +
                           kPoseStabilizerVelocityAlpha * measuredVelZ;
        }
        poseValid = true;
    } else {
        poseValid = false;
    }

    if (poseValid && pose.valid) {
        m_lastStablePose = pose;
        m_haveLastStablePose = true;
        m_lastStableTimestampSec = timestampSec;
    }
}

std::vector<cv::KeyPoint> OrbSlam3Engine::ToKeyPoints(const std::vector<cv::Point2f> &points)
{
    std::vector<cv::KeyPoint> out;
    out.reserve(points.size());
    for (const cv::Point2f &pt : points) {
        cv::KeyPoint kp;
        kp.pt = pt;
        kp.size = 8.0f;
        kp.angle = -1.0f;
        kp.octave = 0;
        kp.response = 1.0f;
        out.push_back(kp);
    }
    return out;
}

bool OrbSlam3Engine::BuildMonoExternalData(const cv::Mat &gray, ORB_SLAM3::ExternalMonoFrameData &outData) const
{
    outData = ORB_SLAM3::ExternalMonoFrameData{};
    const auto totalStartTp = std::chrono::steady_clock::now();
    m_lastXFeatRawLeftCount = 0;
    m_lastXFeatRawRightCount = 0;
    m_lastXFeatMatchedStereoCount = 0;
    m_lastXFeatInjectedLeftCount = 0;
    m_lastXFeatInjectedRightCount = 0;
    m_lastXFeatPrepareMs = 0.0;
    m_lastXFeatWorkerWriteMs = 0.0;
    m_lastXFeatWorkerReadMs = 0.0;
    m_lastXFeatWorkerTotalMs = 0.0;
    m_lastXFeatStereoMatchMs = 0.0;
    m_lastXFeatTotalMs = 0.0;
    m_lastXFeatImageCount = 0;
    m_lastXFeatPayloadBytes = 0;
    if (m_featureFrontend != FeatureFrontend::XFeat || m_xfeatFrontendClient == nullptr ||
        !m_xfeatFrontendClient->Running()) {
        return false;
    }

    XFeatFeatureSet features;
    std::string err;
    float scaleX = 1.0f;
    float scaleY = 1.0f;
    const cv::Mat xfeatInput = BuildXFeatInputImage(gray, m_xfeatInputMaxWidth, m_xfeatInputMaxHeight, scaleX, scaleY);
    if (!m_xfeatFrontendClient->DetectAndCompute(xfeatInput, features, &err) || features.descriptors.empty()) {
        return false;
    }
    const XFeatFrontendClient::Stats stats = m_xfeatFrontendClient->LastStats();
    m_lastXFeatPrepareMs = stats.prepareMs;
    m_lastXFeatWorkerWriteMs = stats.writeMs;
    m_lastXFeatWorkerReadMs = stats.readMs;
    m_lastXFeatWorkerTotalMs = stats.totalMs;
    m_lastXFeatImageCount = stats.imageCount;
    m_lastXFeatPayloadBytes = stats.payloadBytes;
    RemapKeypointsToSource(features.keypoints, scaleX, scaleY);
    m_lastXFeatRawLeftCount = static_cast<int>(features.keypoints.size());
    ORB_SLAM3::Tracking *const tracker = m_system != nullptr ? m_system->GetTracker() : nullptr;
    ORB_SLAM3::ORBextractor *const extractor = SelectMonoExtractor(tracker);
    std::vector<cv::Point2f> orbPoints;
    orbPoints.reserve(features.keypoints.size());
    for (const cv::Point2f &pt : features.keypoints) {
        if (IsPointSafeForOrbDescriptor(pt, gray)) {
            orbPoints.push_back(pt);
        }
    }
    if (!ComputeOrbDescriptorsAtPoints(extractor, gray, orbPoints, outData.keypoints, outData.descriptors)) {
        return false;
    }
    m_lastXFeatInjectedLeftCount = static_cast<int>(outData.keypoints.size());
    m_lastXFeatTotalMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - totalStartTp).count();
    return !outData.keypoints.empty() && !outData.descriptors.empty();
}

bool OrbSlam3Engine::BuildStereoExternalData(const cv::Mat &leftGray, const cv::Mat &rightGray,
                                             ORB_SLAM3::ExternalStereoFrameData &outData,
                                             std::vector<cv::Point2f> *leftRawPoints,
                                             std::vector<cv::Point2f> *rightRawPoints) const
{
    outData = ORB_SLAM3::ExternalStereoFrameData{};
    const auto totalStartTp = std::chrono::steady_clock::now();
    m_lastXFeatRawLeftCount = 0;
    m_lastXFeatRawRightCount = 0;
    m_lastXFeatMatchedStereoCount = 0;
    m_lastXFeatInjectedLeftCount = 0;
    m_lastXFeatInjectedRightCount = 0;
    m_lastXFeatPrepareMs = 0.0;
    m_lastXFeatWorkerWriteMs = 0.0;
    m_lastXFeatWorkerReadMs = 0.0;
    m_lastXFeatWorkerTotalMs = 0.0;
    m_lastXFeatStereoMatchMs = 0.0;
    m_lastXFeatTotalMs = 0.0;
    m_lastXFeatImageCount = 0;
    m_lastXFeatPayloadBytes = 0;
    if (m_featureFrontend != FeatureFrontend::XFeat || m_xfeatFrontendClient == nullptr ||
        !m_xfeatFrontendClient->Running()) {
        return false;
    }
    const int trackingState = m_system != nullptr ? m_system->GetTrackingState() : ORB_SLAM3::Tracking::SYSTEM_NOT_READY;
    const bool trackingHealthy = trackingState == ORB_SLAM3::Tracking::OK;
    const int previousMatchesInliers = m_system != nullptr ? m_system->GetMatchesInliers() : 0;
    const size_t previousTrackedMapPoints = m_system != nullptr ? m_system->GetTrackedMapPointCount() : 0;
    const bool weakTracking =
        trackingHealthy &&
        (previousMatchesInliers < kWeakTrackingMinInliers ||
         previousTrackedMapPoints < static_cast<size_t>(kWeakTrackingMinTrackedMapPoints));

    std::vector<cv::Point2f> temporalLeftMatchedPoints;
    std::vector<cv::Point2f> temporalRightMatchedPoints;
    const std::vector<TemporalStereoPair> rawTemporalPairs = TrackStereoPairsTemporally(
        m_prevStereoLeftGray, m_prevStereoRightGray, m_prevInjectedStereoLeftPoints, m_prevInjectedStereoRightPoints,
        leftGray, rightGray);
    const std::vector<TemporalStereoPair> filteredTemporalPairs =
        FilterTemporalPairsWithMotionRansac(rawTemporalPairs, m_prevInjectedStereoLeftPoints);
    std::vector<TemporalStereoPair> temporalPairs;
    temporalLeftMatchedPoints.reserve(temporalPairs.size());
    temporalRightMatchedPoints.reserve(temporalPairs.size());

    XFeatFeatureSet leftFeatures;
    XFeatFeatureSet rightFeatures;
    std::string err;
    float leftScaleX = 1.0f;
    float leftScaleY = 1.0f;
    float rightScaleX = 1.0f;
    float rightScaleY = 1.0f;
    const cv::Mat leftXFeatInput =
        BuildXFeatInputImage(leftGray, m_xfeatInputMaxWidth, m_xfeatInputMaxHeight, leftScaleX, leftScaleY);
    const cv::Mat rightXFeatInput =
        BuildXFeatInputImage(rightGray, m_xfeatInputMaxWidth, m_xfeatInputMaxHeight, rightScaleX, rightScaleY);
    const bool detectedStereo = m_xfeatFrontendClient->DetectAndComputeStereo(leftXFeatInput, rightXFeatInput,
                                                                              leftFeatures, rightFeatures, &err) &&
                                !leftFeatures.descriptors.empty() && !rightFeatures.descriptors.empty();
    std::vector<cv::Point2f> freshLeftMatchedPoints;
    std::vector<cv::Point2f> freshRightMatchedPoints;
    if (detectedStereo) {
        const XFeatFrontendClient::Stats stats = m_xfeatFrontendClient->LastStats();
        m_lastXFeatPrepareMs = stats.prepareMs;
        m_lastXFeatWorkerWriteMs = stats.writeMs;
        m_lastXFeatWorkerReadMs = stats.readMs;
        m_lastXFeatWorkerTotalMs = stats.totalMs;
        m_lastXFeatImageCount = stats.imageCount;
        m_lastXFeatPayloadBytes = stats.payloadBytes;
        RemapKeypointsToSource(leftFeatures.keypoints, leftScaleX, leftScaleY);
        RemapKeypointsToSource(rightFeatures.keypoints, rightScaleX, rightScaleY);
        if (leftRawPoints != nullptr) {
            *leftRawPoints = leftFeatures.keypoints;
        }
        if (rightRawPoints != nullptr) {
            *rightRawPoints = rightFeatures.keypoints;
        }
        m_lastXFeatRawLeftCount = static_cast<int>(leftFeatures.keypoints.size());
        m_lastXFeatRawRightCount = static_cast<int>(rightFeatures.keypoints.size());

        const auto matchStartTp = std::chrono::steady_clock::now();
        const std::vector<StereoMatchPair> rawMatches = MatchStereoPairs(leftFeatures, rightFeatures, leftGray, rightGray);
        const std::vector<StereoMatchPair> matches = FilterStereoPairsByDisparityConsistency(rawMatches);
        m_lastXFeatStereoMatchMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - matchStartTp).count();
        m_lastXFeatMatchedStereoCount = static_cast<int>(matches.size());
        freshLeftMatchedPoints.reserve(matches.size());
        freshRightMatchedPoints.reserve(matches.size());
        for (const StereoMatchPair &match : matches) {
            const cv::Point2f &leftPt = leftFeatures.keypoints[static_cast<size_t>(match.leftIndex)];
            const cv::Point2f &rightPt = rightFeatures.keypoints[static_cast<size_t>(match.rightIndex)];
            if (!IsPointSafeForOrbDescriptor(leftPt, leftGray) || !IsPointSafeForOrbDescriptor(rightPt, rightGray)) {
                continue;
            }
            freshLeftMatchedPoints.push_back(leftPt);
            freshRightMatchedPoints.push_back(rightPt);
        }
    }

    if (trackingHealthy && !filteredTemporalPairs.empty()) {
        const size_t freshBudget = weakTracking
                                       ? kWeakTrackingTemporalCarryBudget
                                       : (freshLeftMatchedPoints.empty()
                                              ? kTemporalCarryMinBudget
                                              : std::clamp(freshLeftMatchedPoints.size() / 2, kTemporalCarryMinBudget,
                                                           kTemporalCarryMaxBudget));
        temporalPairs = LimitTemporalPairs(filteredTemporalPairs, freshBudget);
        temporalLeftMatchedPoints.reserve(temporalPairs.size());
        temporalRightMatchedPoints.reserve(temporalPairs.size());
        for (const TemporalStereoPair &pair : temporalPairs) {
            temporalLeftMatchedPoints.push_back(pair.leftPt);
            temporalRightMatchedPoints.push_back(pair.rightPt);
        }
    }

    if (freshLeftMatchedPoints.empty() && temporalLeftMatchedPoints.empty()) {
        return false;
    }
    ORB_SLAM3::Tracking *const tracker = m_system != nullptr ? m_system->GetTracker() : nullptr;
    ORB_SLAM3::ORBextractor *const leftExtractor = tracker != nullptr ? tracker->GetLeftORBExtractor() : nullptr;
    ORB_SLAM3::ORBextractor *const rightExtractor = tracker != nullptr ? tracker->GetRightORBExtractor() : nullptr;
    std::vector<cv::Point2f> mergedLeftMatchedPoints;
    std::vector<cv::Point2f> mergedRightMatchedPoints;
    mergedLeftMatchedPoints.reserve(temporalLeftMatchedPoints.size() + freshLeftMatchedPoints.size());
    mergedRightMatchedPoints.reserve(temporalRightMatchedPoints.size() + freshRightMatchedPoints.size());
    AppendStereoPairs(freshLeftMatchedPoints, freshRightMatchedPoints, mergedLeftMatchedPoints,
                      mergedRightMatchedPoints);
    AppendStereoPairs(temporalLeftMatchedPoints, temporalRightMatchedPoints, mergedLeftMatchedPoints,
                      mergedRightMatchedPoints);
    if (weakTracking) {
        LimitStereoPairsInPlace(mergedLeftMatchedPoints, mergedRightMatchedPoints, kWeakTrackingInjectedPairBudget);
    }

    bool finalized = false;
    if (!mergedLeftMatchedPoints.empty() && mergedLeftMatchedPoints.size() == mergedRightMatchedPoints.size()) {
        finalized = FinalizeStereoExternalFromPairs(leftExtractor, rightExtractor, leftGray, rightGray,
                                                    mergedLeftMatchedPoints, mergedRightMatchedPoints, outData);
    }
    if (!finalized && !temporalLeftMatchedPoints.empty() &&
        temporalLeftMatchedPoints.size() == temporalRightMatchedPoints.size()) {
        finalized = FinalizeStereoExternalFromPairs(leftExtractor, rightExtractor, leftGray, rightGray,
                                                    temporalLeftMatchedPoints, temporalRightMatchedPoints, outData);
    }
    if (!finalized && !freshLeftMatchedPoints.empty() && freshLeftMatchedPoints.size() == freshRightMatchedPoints.size()) {
        finalized = FinalizeStereoExternalFromPairs(leftExtractor, rightExtractor, leftGray, rightGray,
                                                    freshLeftMatchedPoints, freshRightMatchedPoints, outData);
    }
    if (!finalized && trackingHealthy && m_havePrevInjectedStereoExternal) {
        finalized = FinalizeStereoExternalFromTemporalCarry(temporalPairs, m_prevInjectedStereoExternal, outData);
        if (finalized) {
            std::cerr << "[slam] xfeat_temporal_descriptor_reuse count=" << outData.leftKeypoints.size()
                      << " temporal_pairs=" << temporalPairs.size()
                      << " fresh_pairs=" << freshLeftMatchedPoints.size() << "\n";
        }
    }
    if (!finalized) {
        return false;
    }

    m_lastXFeatInjectedLeftCount = static_cast<int>(outData.leftKeypoints.size());
    m_lastXFeatInjectedRightCount = static_cast<int>(outData.rightKeypoints.size());
    m_prevStereoLeftGray = leftGray.clone();
    m_prevStereoRightGray = rightGray.clone();
    m_prevInjectedStereoLeftPoints = ToPointList(outData.leftKeypoints);
    m_prevInjectedStereoRightPoints = ToPointList(outData.rightKeypoints);
    m_prevInjectedStereoExternal.leftKeypoints = outData.leftKeypoints;
    m_prevInjectedStereoExternal.rightKeypoints = outData.rightKeypoints;
    m_prevInjectedStereoExternal.leftDescriptors = outData.leftDescriptors.clone();
    m_prevInjectedStereoExternal.rightDescriptors = outData.rightDescriptors.clone();
    m_prevInjectedStereoExternal.matchedStereoPairs = outData.matchedStereoPairs;
    m_havePrevInjectedStereoExternal = true;
    m_lastXFeatTotalMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - totalStartTp).count();
    return true;
}

core::ports::SlamOutput OrbSlam3Engine::Process(const core::ports::SlamInputBatch &input, bool extractFeatures,
                                                bool extractPointCloud)
{
    if (m_featureFrontend == FeatureFrontend::LK) {
        (void)extractPointCloud;
        return ProcessLkStereoVo(input, extractFeatures);
    }

    core::ports::SlamOutput out{};
    if (!m_system) {
        return out;
    }
    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    m_lastXFeatRawLeftCount = 0;
    m_lastXFeatRawRightCount = 0;
    m_lastXFeatMatchedStereoCount = 0;
    m_lastXFeatInjectedLeftCount = 0;
    m_lastXFeatInjectedRightCount = 0;
    m_lastXFeatPrepareMs = 0.0;
    m_lastXFeatWorkerWriteMs = 0.0;
    m_lastXFeatWorkerReadMs = 0.0;
    m_lastXFeatWorkerTotalMs = 0.0;
    m_lastXFeatStereoMatchMs = 0.0;
    m_lastXFeatTotalMs = 0.0;
    m_lastXFeatImageCount = 0;
    m_lastXFeatPayloadBytes = 0;

    Sophus::SE3f tcw;
    const bool monoMode = (m_inputMode != OrbInputMode::Stereo);
    const cv::Mat &monoImage =
        (m_inputMode == OrbInputMode::MonoRight) ? input.stereo.right.gray : input.stereo.left.gray;
    cv::Mat preparedLeftImage;
    cv::Mat preparedRightImage;
    const int previousTrackingState = m_system->GetTrackingState();
    std::string xfeatStatusReason;
    bool tryXFeat = false;
    if (m_featureFrontend != FeatureFrontend::XFeat) {
        xfeatStatusReason = "frontend_not_xfeat";
    } else if (m_xfeatFrontendClient == nullptr) {
        xfeatStatusReason = "worker_not_configured";
    } else if (!m_xfeatFrontendClient->Running()) {
        xfeatStatusReason = "worker_not_running";
    } else if (IsOrbBootstrapState(previousTrackingState)) {
        xfeatStatusReason = "orb_bootstrap";
    } else if (previousTrackingState == ORB_SLAM3::Tracking::OK ||
               previousTrackingState == ORB_SLAM3::Tracking::OK_KLT) {
        xfeatStatusReason = "orb_tracking_active";
    } else if (!IsXFeatTrackingStateSafe(previousTrackingState)) {
        xfeatStatusReason = "tracking_not_ok:" + DescribeTrackingState(previousTrackingState);
    } else if (monoMode && !m_system->CanUseExternalFeatureInjection()) {
        xfeatStatusReason = "resize_enabled";
    } else if (!monoMode &&
               !m_system->PrepareStereoImagesForTracking(input.stereo.left.gray, input.stereo.right.gray,
                                                        preparedLeftImage, preparedRightImage)) {
        xfeatStatusReason = "prepare_tracking_images_failed";
    } else {
        tryXFeat = true;
        xfeatStatusReason = "enabled";
    }
    if (m_featureFrontend == FeatureFrontend::XFeat && xfeatStatusReason != m_lastXFeatStatusReason) {
        std::cerr << "[slam] xfeat_runtime_status=" << xfeatStatusReason << "\n";
        m_lastXFeatStatusReason = xfeatStatusReason;
    }

    ORB_SLAM3::ExternalMonoFrameData monoExternal;
    ORB_SLAM3::ExternalStereoFrameData stereoExternal;
    std::vector<cv::Point2f> stereoLeftRawPoints;
    std::vector<cv::Point2f> stereoRightRawPoints;
    std::vector<cv::Point2f> stereoStreamLeftPoints;
    std::vector<cv::Point2f> stereoStreamRightPoints;
    const bool haveMonoExternal = monoMode && tryXFeat && BuildMonoExternalData(monoImage, monoExternal);
    const bool haveStereoExternal = !monoMode && tryXFeat &&
                                    BuildStereoExternalData(preparedLeftImage, preparedRightImage,
                                                            stereoExternal, &stereoLeftRawPoints, &stereoRightRawPoints);
    bool haveStereoStreamOnlyXFeat = false;
    if (!monoMode && extractFeatures && m_featureFrontend == FeatureFrontend::XFeat && !haveStereoExternal &&
        m_xfeatFrontendClient != nullptr && m_xfeatFrontendClient->Running()) {
        ORB_SLAM3::ExternalStereoFrameData streamOnlyExternal;
        (void)BuildStereoExternalData(input.stereo.left.gray, input.stereo.right.gray, streamOnlyExternal,
                                      &stereoStreamLeftPoints, &stereoStreamRightPoints);
        haveStereoStreamOnlyXFeat = !stereoStreamLeftPoints.empty() || !stereoStreamRightPoints.empty();
    }
    const bool haveXFeatFeaturesForOutput = haveMonoExternal || haveStereoExternal || haveStereoStreamOnlyXFeat;
    out.usedXFeatFrontend = haveMonoExternal || haveStereoExternal;
    out.xfeatRawLeftCount = m_lastXFeatRawLeftCount;
    out.xfeatRawRightCount = m_lastXFeatRawRightCount;
    out.xfeatMatchedStereoCount = m_lastXFeatMatchedStereoCount;
    out.xfeatInjectedLeftCount = m_lastXFeatInjectedLeftCount;
    out.xfeatInjectedRightCount = m_lastXFeatInjectedRightCount;
    out.xfeatPrepareMs = m_lastXFeatPrepareMs;
    out.xfeatWorkerWriteMs = m_lastXFeatWorkerWriteMs;
    out.xfeatWorkerReadMs = m_lastXFeatWorkerReadMs;
    out.xfeatWorkerTotalMs = m_lastXFeatWorkerTotalMs;
    out.xfeatStereoMatchMs = m_lastXFeatStereoMatchMs;
    out.xfeatTotalMs = m_lastXFeatTotalMs;
    out.xfeatImageCount = m_lastXFeatImageCount;
    out.xfeatPayloadBytes = m_lastXFeatPayloadBytes;

    if (m_useImu) {
        if (monoMode) {
            tcw = haveMonoExternal ? m_system->TrackMonocularWithFeatures(monoImage, monoExternal, input.frameTimeSec, input.imu)
                                   : m_system->TrackMonocular(monoImage, input.frameTimeSec, input.imu);
        } else {
            tcw = haveStereoExternal
                      ? m_system->TrackStereoPreparedWithFeatures(preparedLeftImage, preparedRightImage, stereoExternal,
                                                                  input.frameTimeSec, input.imu)
                      : m_system->TrackStereo(input.stereo.left.gray, input.stereo.right.gray, input.frameTimeSec,
                                              input.imu);
        }
    } else {
        if (monoMode) {
            tcw = haveMonoExternal ? m_system->TrackMonocularWithFeatures(monoImage, monoExternal, input.frameTimeSec)
                                   : m_system->TrackMonocular(monoImage, input.frameTimeSec);
        } else {
            tcw = haveStereoExternal
                      ? m_system->TrackStereoPreparedWithFeatures(preparedLeftImage, preparedRightImage, stereoExternal,
                                                                  input.frameTimeSec)
                      : m_system->TrackStereo(input.stereo.left.gray, input.stereo.right.gray, input.frameTimeSec);
        }
    }

    out.trackingState = m_system->GetTrackingState();
    out.mapId = m_system->GetCurrentMapId();
    out.matchesInliers = m_system->GetMatchesInliers();
    out.trackedMapPointCount = static_cast<uint32_t>(m_system->GetTrackedMapPointCount());
    out.localMapPointCount = static_cast<uint32_t>(m_system->GetLocalMapPointCount());

    const Sophus::SE3f twc = tcw.inverse();
    const Eigen::Vector3f t = twc.translation();
    const Eigen::Quaternionf q(twc.so3().unit_quaternion());
    out.poseValid = std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z()) && std::isfinite(q.w()) &&
                    std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z());
    out.pose.valid = out.poseValid;
    out.pose.x = t.x();
    out.pose.y = t.y();
    out.pose.z = t.z();
    out.pose.qw = q.w();
    out.pose.qx = q.x();
    out.pose.qy = q.y();
    out.pose.qz = q.z();

    StabilizeOutputPose(out.pose, out.poseValid, input.frameTimeSec, out.trackingState);

    if (extractFeatures && haveXFeatFeaturesForOutput) {
        if (monoMode) {
            if (m_inputMode == OrbInputMode::MonoRight) {
                out.rightFeatures = ToPointList(monoExternal.keypoints);
            } else {
                out.leftFeatures = ToPointList(monoExternal.keypoints);
            }
        } else if (haveStereoExternal) {
            out.leftFeatures = std::move(stereoLeftRawPoints);
            out.rightFeatures = std::move(stereoRightRawPoints);
        } else {
            out.leftFeatures = std::move(stereoStreamLeftPoints);
            out.rightFeatures = std::move(stereoStreamRightPoints);
        }
    }

    const bool needVisualExtraction = extractPointCloud || (extractFeatures && !haveXFeatFeaturesForOutput);
    if (!needVisualExtraction) {
        return out;
    }

    const int leftWidth = monoMode ? monoImage.cols : input.stereo.left.gray.cols;
    const int leftHeight = monoMode ? monoImage.rows : input.stereo.left.gray.rows;
    ORB_SLAM3::TrackedVisualData visual =
        m_system->ExtractTrackedVisualData(leftWidth, leftHeight, monoMode ? 0 : input.stereo.right.gray.cols,
                                           monoMode ? 0 : input.stereo.right.gray.rows, extractPointCloud, 120);
    out.matchesInliers = visual.matchesInliers;
    out.trackedMapPointCount = static_cast<uint32_t>(visual.trackedMapPointCount);
    out.localMapPointCount = static_cast<uint32_t>(visual.localMapPointCount);
    if (extractFeatures && !out.usedXFeatFrontend) {
        if (m_inputMode == OrbInputMode::MonoRight) {
            out.rightFeatures = std::move(visual.leftFeatures);
        } else {
            out.leftFeatures = std::move(visual.leftFeatures);
            out.rightFeatures = std::move(visual.rightFeatures);
        }
    }
    if (extractPointCloud) {
        out.pointCloudXyz = std::move(visual.pointCloudXyz);
    }
    return out;
}

} // namespace smartdrone::adapters::slam
