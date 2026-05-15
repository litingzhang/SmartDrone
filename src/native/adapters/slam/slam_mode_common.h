#pragma once

#include "adapters/slam/slam_engine_adapter.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <climits>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <sophus/se3.hpp>

#if SMART_DRONE_HAS_VPI
#include <vpi/Array.h>
#include <vpi/Image.h>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/Pyramid.h>
#include <vpi/Stream.h>
#include <vpi/WarpMap.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/GaussianPyramid.h>
#include <vpi/algo/OpticalFlowPyrLK.h>
#include <vpi/algo/Remap.h>
#include <vpi/algo/StereoDisparity.h>
#endif

#include "ImuTypes.h"
#include "ORBmatcher.h"
#include "TrackedVisualData.h"
#include "adapters/slam/superpoint_lightglue_frontend_client.h"

namespace smartdrone::adapters::slam {

#if SMART_DRONE_HAS_VPI
struct LkPerFrameVpiState {
    ~LkPerFrameVpiState()
    {
        if (prevPts != nullptr) {
            vpiArrayDestroy(prevPts);
        }
        if (curPts != nullptr) {
            vpiArrayDestroy(curPts);
        }
        if (trackStatus != nullptr) {
            vpiArrayDestroy(trackStatus);
        }
        if (prevPyr != nullptr) {
            vpiPyramidDestroy(prevPyr);
        }
        if (curPyr != nullptr) {
            vpiPyramidDestroy(curPyr);
        }
        if (lkPayload != nullptr) {
            vpiPayloadDestroy(lkPayload);
        }
        if (leftRect != nullptr) {
            vpiImageDestroy(leftRect);
        }
        if (rightRect != nullptr) {
            vpiImageDestroy(rightRect);
        }
        if (prevLeftRect != nullptr) {
            vpiImageDestroy(prevLeftRect);
        }
        if (prevRightRect != nullptr) {
            vpiImageDestroy(prevRightRect);
        }
        if (leftRemapPayload != nullptr) {
            vpiPayloadDestroy(leftRemapPayload);
        }
        if (rightRemapPayload != nullptr) {
            vpiPayloadDestroy(rightRemapPayload);
        }
        if (leftWrapper != nullptr) {
            vpiImageDestroy(leftWrapper);
        }
        if (rightWrapper != nullptr) {
            vpiImageDestroy(rightWrapper);
        }
        if (disparity != nullptr) {
            vpiImageDestroy(disparity);
        }
        if (stereoPayload != nullptr) {
            vpiPayloadDestroy(stereoPayload);
        }
        if (stream != nullptr) {
            vpiStreamDestroy(stream);
        }
        vpiWarpMapFreeData(&leftWarp);
        vpiWarpMapFreeData(&rightWarp);
    }

    int width{0};
    int height{0};
    int maxDisparity{0};
    VPIStream stream{nullptr};
    VPIPayload stereoPayload{nullptr};
    VPIPayload leftRemapPayload{nullptr};
    VPIPayload rightRemapPayload{nullptr};
    VPIPayload lkPayload{nullptr};
    VPIImage leftWrapper{nullptr};
    VPIImage rightWrapper{nullptr};
    VPIImage leftRect{nullptr};
    VPIImage rightRect{nullptr};
    VPIImage prevLeftRect{nullptr};
    VPIImage prevRightRect{nullptr};
    VPIImage disparity{nullptr};
    VPIPyramid prevPyr{nullptr};
    VPIPyramid curPyr{nullptr};
    VPIArray prevPts{nullptr};
    VPIArray curPts{nullptr};
    VPIArray trackStatus{nullptr};
    VPIWarpMap leftWarp{};
    VPIWarpMap rightWarp{};
    bool hasPrevRect{false};
};

inline const char *VpiStatusName(VPIStatus status)
{
    const char *name = vpiStatusGetName(status);
    return name != nullptr ? name : "VPI_ERROR_UNKNOWN";
}

inline bool FillVpiWarpMapFromOpenCvMaps(const cv::Mat &mapX, const cv::Mat &mapY, VPIWarpMap &warp)
{
    if (mapX.empty() || mapY.empty() || mapX.size() != mapY.size() || mapX.type() != CV_32FC1 ||
        mapY.type() != CV_32FC1) {
        return false;
    }
    vpiWarpMapFreeData(&warp);
    warp = {};
    warp.grid.numHorizRegions = 1;
    warp.grid.numVertRegions = 1;
    warp.grid.regionWidth[0] = static_cast<int16_t>(mapX.cols);
    warp.grid.regionHeight[0] = static_cast<int16_t>(mapX.rows);
    warp.grid.horizInterval[0] = 1;
    warp.grid.vertInterval[0] = 1;
    VPIStatus status = vpiWarpMapAllocData(&warp);
    if (status != VPI_SUCCESS || warp.keypoints == nullptr) {
        std::cerr << "[lk_per_frame_accel] VPI warp map allocation failed: " << VpiStatusName(status) << "\n";
        return false;
    }
    for (int y = 0; y < mapX.rows; ++y) {
        auto *row = reinterpret_cast<VPIKeypointF32 *>(reinterpret_cast<uint8_t *>(warp.keypoints) +
                                                      static_cast<size_t>(y) * warp.pitchBytes);
        for (int x = 0; x < mapX.cols; ++x) {
            row[x].x = mapX.at<float>(y, x);
            row[x].y = mapY.at<float>(y, x);
        }
    }
    return true;
}

inline cv::Mat DownloadVpiU8Image(VPIImage image)
{
    VPIImageData data{};
    VPIStatus status = vpiImageLockData(image, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &data);
    if (status != VPI_SUCCESS || data.bufferType != VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR ||
        data.buffer.pitch.numPlanes < 1 || data.buffer.pitch.planes[0].data == nullptr) {
        std::cerr << "[lk_per_frame_accel] VPI image lock failed: " << VpiStatusName(status) << "\n";
        return {};
    }
    const auto &plane = data.buffer.pitch.planes[0];
    cv::Mat view(plane.height, plane.width, CV_8UC1, plane.data, static_cast<size_t>(plane.pitchBytes));
    cv::Mat out = view.clone();
    vpiImageUnlock(image);
    return out;
}
#endif

inline bool EnvFlagEnabled(const char *name, bool defaultValue)
{
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return defaultValue;
    }
    const std::string text(value);
    return !(text == "0" || text == "false" || text == "FALSE" || text == "off" || text == "OFF");
}

inline int EnvIntValue(const char *name, int defaultValue)
{
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return defaultValue;
    }
    char *end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    return end != value ? static_cast<int>(parsed) : defaultValue;
}

inline float EnvFloatValue(const char *name, float defaultValue)
{
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return defaultValue;
    }
    char *end = nullptr;
    const float parsed = std::strtof(value, &end);
    return end != value ? parsed : defaultValue;
}

inline std::string EnvStringValue(const char *name, const char *defaultValue)
{
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return defaultValue;
    }
    return value;
}

namespace {

size_t EnvSizeValueClamped(const char *name, size_t defaultValue, size_t minValue, size_t maxValue)
{
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return defaultValue;
    }
    char *end = nullptr;
    const unsigned long parsed = std::strtoul(value, &end, 10);
    if (end == value) {
        return defaultValue;
    }
    return std::clamp(static_cast<size_t>(parsed), minValue, maxValue);
}

inline int EnvIntValueClamped(const char *name, int defaultValue, int minValue, int maxValue)
{
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return defaultValue;
    }
    char *end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (end == value) {
        return defaultValue;
    }
    return std::clamp(static_cast<int>(parsed), minValue, maxValue);
}

inline float EnvFloatValueClamped(const char *name, float defaultValue, float minValue, float maxValue)
{
    const float parsed = EnvFloatValue(name, defaultValue);
    if (!std::isfinite(parsed)) {
        return std::clamp(defaultValue, minValue, maxValue);
    }
    return std::clamp(parsed, minValue, maxValue);
}

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
constexpr size_t kExternalStereoMaxLeftFeatures = 1200;
constexpr size_t kExternalStereoMaxLeftFeaturesLimit = 2500;
constexpr int kStereoMaxPairsPerCellLimit = 32;
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

float ExternalStereoMinDisparityPx()
{
    return EnvFloatValueClamped("SMART_DRONE_EXTERNAL_STEREO_MIN_DISPARITY_PX",
                                kStereoMinDisparityPx, 0.05f, kStereoMaxDisparityPx);
}
constexpr size_t kStableOrbTrackMinTrackedMapPoints = 80;
constexpr double kPoseStabilizerDefaultDtSec = 1.0 / 20.0;
constexpr double kPoseStabilizerMinDtSec = 1.0 / 120.0;
constexpr double kPoseStabilizerMaxDtSec = 0.25;
constexpr float kPoseStabilizerMaxSpeedMps = 3.0f;
constexpr float kPoseStabilizerMaxStepMeters = 0.055f;
constexpr float kPoseStabilizerMaxRotStepDeg = 3.0f;
constexpr float kPoseStabilizerVelocityAlpha = 0.35f;
constexpr float kPoseStabilizerPredictedVelocityDecay = 0.985f;
constexpr float kLkMinDepthMeters = 0.35f;
constexpr float kLkMaxDepthMeters = 12.0f;
constexpr float kLkMaxFlowPx = 96.0f;
constexpr float kLkMaxStepMeters = 0.35f;
constexpr float kLkForwardAxisGain = 1.0f;
constexpr float kLkMaxForwardStepMeters = 0.35f;
constexpr float kLkMaxLateralStepMeters = 0.35f;
constexpr float kLkMaxVerticalStepMeters = 0.35f;
constexpr int kLkMinPnPPoints = 12;
constexpr int kLkMinPnPInliers = 10;
constexpr float kLkStereoRefineSearchRadiusPx = 10.0f;
constexpr int kLkRecoveryMinTracks = 24;
constexpr int kLkRecoveryMinInliers = 16;
constexpr int kLkHardRecoveryMinTracks = 10;
constexpr int kLkHardRecoveryMinInliers = 10;
constexpr uint64_t kLkGridRefillIntervalFrames = 30;
constexpr int kLkGridCols = 8;
constexpr int kLkGridRows = 6;
constexpr int kLkGridCellCount = kLkGridCols * kLkGridRows;
constexpr int kLkTargetTracksPerCell = 12;
constexpr int kLkMinTracksPerCell = 6;
constexpr size_t kLkMaxTracks = 576;
constexpr int kLkGfttMaxCorners = 900;
constexpr int kLkGfttPerFrameMaxCorners = 960;
constexpr int kLkGfttPerFrameMaxCornersPerCell = 20;
constexpr float kLkPerFrameForwardBackwardMaxErrPx = 1.25f;
constexpr int kLkPerFramePnPSelectGridCols = 8;
constexpr int kLkPerFramePnPSelectGridRows = 6;
constexpr int kLkPerFramePnPDepthBins = 4;
constexpr int kLkPerFramePnPMaxPerGridDepthBin = 4;
constexpr int kLkPerFrameDefaultPnPIterations = 80;
constexpr double kLkPerFrameDefaultPnPConfidence = 0.995;
constexpr double kLkPerFrameVpiPnPReprojThresholdPx = 3.0;
constexpr double kLkGfttQualityLevel = 0.01;
constexpr double kLkGfttMinDistancePx = 8.0;
constexpr int kLkGfttBlockSize = 7;
constexpr double kLkGfttHarrisK = 0.04;
constexpr float kLkGfttStereoSearchRadiusPx = 96.0f;
constexpr float kLkMinConsistentDisparityPx = 1.0f;
constexpr float kLkDisparityNeighborhoodTolerancePx = 1.5f;
constexpr int kLkDisparityNeighborhoodRadius = 1;
constexpr double kLkPerFramePnPReprojThresholdPx = 3.0;
constexpr int kVpiStereoConfidenceThreshold = 32767;
constexpr int kVpiStereoP1 = 20;
constexpr int kVpiStereoP2 = 176;
constexpr float kVpiStereoUniqueness = 0.38f;
constexpr int kVpiStereoIncludeDiagonals = 1;
constexpr float kLkMinSeedDistancePx = 3.5f;
constexpr float kLkMinCandidateQuality = 0.18f;
constexpr uint64_t kLkLoopKeyframeIntervalFrames = 30;
constexpr uint64_t kLkLoopMinAgeFrames = 300;
constexpr uint64_t kLkLoopCooldownFrames = 200;
constexpr size_t kLkLoopMaxKeyframes = 160;
constexpr double kLkLoopMinSimilarity = 0.60;

struct StereoMatchPair {
    int leftIndex{-1};
    int rightIndex{-1};
    float descriptorScore{-std::numeric_limits<float>::infinity()};
    float zncc{-1.0f};
    float disparity{0.0f};
    float quality{0.0f};
};

cv::Mat BuildLkLoopImageDescriptor(const cv::Mat &gray)
{
    if (gray.empty()) {
        return {};
    }
    cv::Mat small;
    cv::resize(gray, small, cv::Size(48, 32), 0.0, 0.0, cv::INTER_AREA);
    small.convertTo(small, CV_32F);
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(small, mean, stddev);
    const double sigma = std::max(1.0e-6, stddev[0]);
    small = (small - mean[0]) / sigma;
    return small.reshape(1, 1).clone();
}

double LkLoopDescriptorSimilarity(const cv::Mat &lhs, const cv::Mat &rhs)
{
    if (lhs.empty() || rhs.empty() || lhs.type() != CV_32F || rhs.type() != CV_32F ||
        lhs.total() != rhs.total()) {
        return -1.0;
    }
    return lhs.dot(rhs) / static_cast<double>(lhs.total());
}

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

bool HasValidSuperPointDescriptors(const SuperPointFeatureSet &features)
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
    const int maxPairsPerCell = EnvIntValueClamped("SMART_DRONE_EXTERNAL_STEREO_MAX_PAIRS_PER_CELL",
                                                   kStereoMaxPairsPerCell, 1, kStereoMaxPairsPerCellLimit);
    std::vector<int> cellCounts(static_cast<size_t>(kStereoGridCols * kStereoGridRows), 0);
    std::vector<StereoMatchPair> selected;
    selected.reserve(matches.size());

    for (const StereoMatchPair &match : matches) {
        const cv::Point2f &pt = leftKeypoints[static_cast<size_t>(match.leftIndex)];
        const int col = std::clamp(static_cast<int>(pt.x) / cellWidth, 0, kStereoGridCols - 1);
        const int row = std::clamp(static_cast<int>(pt.y) / cellHeight, 0, kStereoGridRows - 1);
        const size_t cellIndex = static_cast<size_t>(row * kStereoGridCols + col);
        if (cellCounts[cellIndex] >= maxPairsPerCell) {
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
    return yDelta <= kStereoMaxEpipolarDeltaPx && disparity >= ExternalStereoMinDisparityPx() &&
           disparity <= kStereoMaxDisparityPx;
}

bool RefineRightPointByStereoZncc(const cv::Mat &leftGray32f, const cv::Point2f &leftPt,
                                  const cv::Mat &rightGray32f, const cv::Point2f &predictedRightPt,
                                  cv::Point2f &refinedRightPt, float &bestScore)
{
    bestScore = -1.0f;
    refinedRightPt = predictedRightPt;
    if (leftGray32f.empty() || rightGray32f.empty()) {
        return false;
    }

    const float minRightX = std::max(static_cast<float>(kStereoPatchRadiusPx), leftPt.x - kStereoMaxDisparityPx);
    const float maxRightX = std::min(leftPt.x - ExternalStereoMinDisparityPx(),
                                     static_cast<float>(rightGray32f.cols - kStereoPatchRadiusPx - 1));
    if (minRightX > maxRightX) {
        return false;
    }

    float predictedScore = -1.0f;
    const bool havePredictedScore = IsStereoPairGeometricallyValid(leftPt, predictedRightPt) &&
                                    ComputePatchZncc(leftGray32f, leftPt, rightGray32f, predictedRightPt,
                                                     predictedScore) &&
                                    predictedScore >= kTemporalStereoMinZnccScore;
    float bestRank = havePredictedScore ? predictedScore : -1.0f;

    const int searchStart =
        static_cast<int>(std::floor(std::max(minRightX, predictedRightPt.x - kLkStereoRefineSearchRadiusPx)));
    const int searchEnd =
        static_cast<int>(std::ceil(std::min(maxRightX, predictedRightPt.x + kLkStereoRefineSearchRadiusPx)));
    const float rightY = leftPt.y;
    for (int x = searchStart; x <= searchEnd; ++x) {
        const cv::Point2f candidate(static_cast<float>(x), rightY);
        float score = -1.0f;
        if (!ComputePatchZncc(leftGray32f, leftPt, rightGray32f, candidate, score)) {
            continue;
        }
        const float rank = score - 0.015f * std::fabs(candidate.x - predictedRightPt.x);
        if (rank > bestRank + 0.03f) {
            bestRank = rank;
            bestScore = score;
            refinedRightPt = candidate;
        }
    }
    if (bestScore < 0.0f && havePredictedScore) {
        bestScore = predictedScore;
    }
    return bestScore >= kTemporalStereoMinZnccScore && IsStereoPairGeometricallyValid(leftPt, refinedRightPt);
}

bool FindRightPointByStereoZncc(const cv::Mat &leftGray32f, const cv::Point2f &leftPt, const cv::Mat &rightGray32f,
                                cv::Point2f &rightPt, float &bestScore)
{
    bestScore = -1.0f;
    if (leftGray32f.empty() || rightGray32f.empty()) {
        return false;
    }

    const int minRightX =
        static_cast<int>(std::ceil(std::max(static_cast<float>(kStereoPatchRadiusPx), leftPt.x - kStereoMaxDisparityPx)));
    const int maxRightX = static_cast<int>(std::floor(std::min(leftPt.x - ExternalStereoMinDisparityPx(),
                                                               static_cast<float>(rightGray32f.cols -
                                                                                  kStereoPatchRadiusPx - 1))));
    if (minRightX > maxRightX) {
        return false;
    }

    const float rightY = leftPt.y;
    for (int x = minRightX; x <= maxRightX; ++x) {
        const cv::Point2f candidate(static_cast<float>(x), rightY);
        float score = -1.0f;
        if (!ComputePatchZncc(leftGray32f, leftPt, rightGray32f, candidate, score)) {
            continue;
        }
        const float disparity = leftPt.x - candidate.x;
        const float rank = score - 0.0005f * disparity;
        const float bestRank = bestScore < -0.5f ? -1.0f : bestScore - 0.0005f * (leftPt.x - rightPt.x);
        if (rank > bestRank) {
            bestScore = score;
            rightPt = candidate;
        }
    }
    return bestScore >= kTemporalStereoMinZnccScore && IsStereoPairGeometricallyValid(leftPt, rightPt);
}

bool FindRightPointByStereoZnccAroundDisparity(const cv::Mat &leftGray32f, const cv::Point2f &leftPt,
                                               const cv::Mat &rightGray32f, float expectedDisparity,
                                               cv::Point2f &rightPt, float &bestScore)
{
    bestScore = -1.0f;
    if (leftGray32f.empty() || rightGray32f.empty() || !(expectedDisparity > 0.0f) ||
        !std::isfinite(expectedDisparity)) {
        return FindRightPointByStereoZncc(leftGray32f, leftPt, rightGray32f, rightPt, bestScore);
    }

    const float centerRightX = leftPt.x - expectedDisparity;
    const int minRightX = static_cast<int>(std::ceil(std::max(
        {static_cast<float>(kStereoPatchRadiusPx), leftPt.x - kStereoMaxDisparityPx,
         centerRightX - kLkGfttStereoSearchRadiusPx})));
    const int maxRightX = static_cast<int>(std::floor(std::min(
        {leftPt.x - ExternalStereoMinDisparityPx(), static_cast<float>(rightGray32f.cols - kStereoPatchRadiusPx - 1),
         centerRightX + kLkGfttStereoSearchRadiusPx})));
    if (minRightX > maxRightX) {
        return FindRightPointByStereoZncc(leftGray32f, leftPt, rightGray32f, rightPt, bestScore);
    }

    const float rightY = leftPt.y;
    for (int x = minRightX; x <= maxRightX; ++x) {
        const cv::Point2f candidate(static_cast<float>(x), rightY);
        float score = -1.0f;
        if (!ComputePatchZncc(leftGray32f, leftPt, rightGray32f, candidate, score)) {
            continue;
        }
        const float disparity = leftPt.x - candidate.x;
        const float rank = score - 0.0015f * std::fabs(disparity - expectedDisparity);
        const float bestRank =
            bestScore < -0.5f ? -1.0f : bestScore - 0.0015f * std::fabs((leftPt.x - rightPt.x) - expectedDisparity);
        if (rank > bestRank) {
            bestScore = score;
            rightPt = candidate;
        }
    }
    return bestScore >= kTemporalStereoMinZnccScore && IsStereoPairGeometricallyValid(leftPt, rightPt);
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

bool IsPointNearExisting(const cv::Point2f &pt, const std::vector<cv::KeyPoint> &existing)
{
    const float minDistSq = kTemporalMergeMinDistancePx * kTemporalMergeMinDistancePx;
    for (const cv::KeyPoint &keypoint : existing) {
        const cv::Point2f delta = keypoint.pt - pt;
        if ((delta.x * delta.x + delta.y * delta.y) <= minDistSq) {
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
    outData.leftToRightMatch.resize(outData.leftKeypoints.size());
    for (size_t i = 0; i < outData.leftToRightMatch.size(); ++i) {
        outData.leftToRightMatch[i] = static_cast<int>(i);
    }
    return true;
}

bool FinalizeStereoExternalFromPairsWithAllLeft(ORB_SLAM3::ORBextractor *leftExtractor,
                                                ORB_SLAM3::ORBextractor *rightExtractor, const cv::Mat &leftGray,
                                                const cv::Mat &rightGray,
                                                const std::vector<cv::Point2f> &allLeftPoints,
                                                const std::vector<StereoMatchPair> &stereoMatches,
                                                const SuperPointFeatureSet &leftFeatures,
                                                const SuperPointFeatureSet &rightFeatures,
                                                ORB_SLAM3::ExternalStereoFrameData &outData)
{
    if (leftExtractor == nullptr || rightExtractor == nullptr || leftGray.empty() || rightGray.empty() ||
        allLeftPoints.empty()) {
        return false;
    }

    std::vector<cv::Point2f> safeLeftPoints;
    safeLeftPoints.reserve(allLeftPoints.size());
    for (const cv::Point2f &point : allLeftPoints) {
        if (IsPointSafeForOrbDescriptor(point, leftGray)) {
            safeLeftPoints.push_back(point);
        }
    }
    if (safeLeftPoints.empty()) {
        return false;
    }

    std::vector<cv::KeyPoint> leftKeypoints;
    cv::Mat leftDescriptors;
    if (!ComputeOrbDescriptorsAtPoints(leftExtractor, leftGray, safeLeftPoints, leftKeypoints, leftDescriptors)) {
        return false;
    }

    std::vector<cv::Point2f> stereoLeftPoints;
    std::vector<cv::Point2f> stereoRightPoints;
    stereoLeftPoints.reserve(stereoMatches.size());
    stereoRightPoints.reserve(stereoMatches.size());
    for (const StereoMatchPair &match : stereoMatches) {
        if (match.leftIndex < 0 || match.rightIndex < 0 ||
            static_cast<size_t>(match.leftIndex) >= leftFeatures.keypoints.size() ||
            static_cast<size_t>(match.rightIndex) >= rightFeatures.keypoints.size()) {
            continue;
        }
        const cv::Point2f &leftPoint = leftFeatures.keypoints[static_cast<size_t>(match.leftIndex)];
        const cv::Point2f &rightPoint = rightFeatures.keypoints[static_cast<size_t>(match.rightIndex)];
        if (!IsPointSafeForOrbDescriptor(leftPoint, leftGray) ||
            !IsPointSafeForOrbDescriptor(rightPoint, rightGray) ||
            !IsStereoPairGeometricallyValid(leftPoint, rightPoint)) {
            continue;
        }
        stereoLeftPoints.push_back(leftPoint);
        stereoRightPoints.push_back(rightPoint);
    }
    if (stereoLeftPoints.empty()) {
        return false;
    }

    std::vector<cv::KeyPoint> stereoLeftKeypoints;
    std::vector<cv::KeyPoint> rightKeypoints;
    cv::Mat stereoLeftDescriptors;
    cv::Mat rightDescriptors;
    if (!ComputeOrbDescriptorsAtPoints(leftExtractor, leftGray, stereoLeftPoints, stereoLeftKeypoints,
                                       stereoLeftDescriptors) ||
        !ComputeOrbDescriptorsAtPoints(rightExtractor, rightGray, stereoRightPoints, rightKeypoints, rightDescriptors)) {
        return false;
    }
    if (stereoLeftKeypoints.size() != rightKeypoints.size() ||
        stereoLeftDescriptors.rows != rightDescriptors.rows ||
        stereoLeftDescriptors.rows != static_cast<int>(stereoLeftKeypoints.size())) {
        return false;
    }

    std::vector<int> leftToRight(leftKeypoints.size(), -1);
    cv::Mat mergedRightDescriptors;
    mergedRightDescriptors.create(0, rightDescriptors.cols, rightDescriptors.type());
    std::vector<cv::KeyPoint> mergedRightKeypoints;
    mergedRightKeypoints.reserve(rightKeypoints.size());
    for (size_t li = 0; li < leftKeypoints.size(); ++li) {
        for (size_t si = 0; si < stereoLeftKeypoints.size(); ++si) {
            const cv::Point2f delta = leftKeypoints[li].pt - stereoLeftKeypoints[si].pt;
            if (delta.x * delta.x + delta.y * delta.y > 1.0f) {
                continue;
            }
            leftToRight[li] = static_cast<int>(mergedRightKeypoints.size());
            mergedRightKeypoints.push_back(rightKeypoints[si]);
            mergedRightDescriptors.push_back(rightDescriptors.row(static_cast<int>(si)));
            break;
        }
    }

    if (mergedRightKeypoints.empty()) {
        return false;
    }

    outData.leftKeypoints = std::move(leftKeypoints);
    outData.rightKeypoints = std::move(mergedRightKeypoints);
    outData.leftDescriptors = std::move(leftDescriptors);
    outData.rightDescriptors = std::move(mergedRightDescriptors);
    outData.matchedStereoPairs = true;
    outData.leftToRightMatch = std::move(leftToRight);
    return true;
}

void AppendOrbLeftOnlyFeatures(ORB_SLAM3::ORBextractor *leftExtractor, const cv::Mat &leftGray,
                               ORB_SLAM3::ExternalStereoFrameData &externalData, size_t maxLeftFeatures)
{
    if (leftExtractor == nullptr || leftGray.empty() || externalData.leftKeypoints.size() >= maxLeftFeatures) {
        return;
    }

    std::vector<cv::KeyPoint> orbKeypoints;
    cv::Mat orbDescriptors;
    std::vector<int> lapping = {0, 0};
    (*leftExtractor)(leftGray, cv::Mat(), orbKeypoints, orbDescriptors, lapping);
    if (orbKeypoints.empty() || orbDescriptors.empty() || orbDescriptors.type() != CV_8U ||
        orbDescriptors.rows != static_cast<int>(orbKeypoints.size())) {
        return;
    }

    const size_t initialLeftCount = externalData.leftKeypoints.size();
    std::vector<int> selectedRows;
    selectedRows.reserve(std::min(orbKeypoints.size(), maxLeftFeatures - initialLeftCount));
    for (size_t i = 0; i < orbKeypoints.size() && initialLeftCount + selectedRows.size() < maxLeftFeatures; ++i) {
        if (!IsPointSafeForOrbDescriptor(orbKeypoints[i].pt, leftGray) ||
            IsPointNearExisting(orbKeypoints[i].pt, externalData.leftKeypoints)) {
            continue;
        }
        externalData.leftKeypoints.push_back(orbKeypoints[i]);
        selectedRows.push_back(static_cast<int>(i));
    }
    if (selectedRows.empty()) {
        return;
    }

    cv::Mat mergedDescriptors;
    if (!externalData.leftDescriptors.empty()) {
        externalData.leftDescriptors.copyTo(mergedDescriptors);
    } else {
        mergedDescriptors.create(0, orbDescriptors.cols, orbDescriptors.type());
    }
    for (int row : selectedRows) {
        mergedDescriptors.push_back(orbDescriptors.row(row));
    }
    externalData.leftDescriptors = std::move(mergedDescriptors);
    if (!externalData.leftToRightMatch.empty()) {
        externalData.leftToRightMatch.resize(externalData.leftKeypoints.size(), -1);
    }
}

size_t AppendOrbStereoAugmentFeatures(ORB_SLAM3::ORBextractor *leftExtractor,
                                      ORB_SLAM3::ORBextractor *rightExtractor, const cv::Mat &leftGray,
                                      const cv::Mat &rightGray,
                                      ORB_SLAM3::ExternalStereoFrameData &externalData, size_t maxExtraPairs)
{
    if (leftExtractor == nullptr || rightExtractor == nullptr || leftGray.empty() || rightGray.empty() ||
        maxExtraPairs == 0) {
        return 0;
    }

    std::vector<cv::KeyPoint> leftKeypoints;
    std::vector<cv::KeyPoint> rightKeypoints;
    cv::Mat leftDescriptors;
    cv::Mat rightDescriptors;
    std::vector<int> lapping = {0, 0};
    (*leftExtractor)(leftGray, cv::Mat(), leftKeypoints, leftDescriptors, lapping);
    (*rightExtractor)(rightGray, cv::Mat(), rightKeypoints, rightDescriptors, lapping);
    if (leftKeypoints.empty() || rightKeypoints.empty() || leftDescriptors.empty() || rightDescriptors.empty() ||
        leftDescriptors.type() != CV_8U || rightDescriptors.type() != CV_8U ||
        leftDescriptors.rows != static_cast<int>(leftKeypoints.size()) ||
        rightDescriptors.rows != static_cast<int>(rightKeypoints.size()) ||
        leftDescriptors.cols != rightDescriptors.cols) {
        return 0;
    }
    if ((!externalData.leftDescriptors.empty() &&
         (externalData.leftDescriptors.type() != CV_8U || externalData.leftDescriptors.cols != leftDescriptors.cols)) ||
        (!externalData.rightDescriptors.empty() &&
         (externalData.rightDescriptors.type() != CV_8U ||
          externalData.rightDescriptors.cols != rightDescriptors.cols))) {
        return 0;
    }

    struct OrbStereoAugmentCandidate {
        int leftIndex{-1};
        int rightIndex{-1};
        int distance{INT_MAX};
        float zncc{-1.0f};
        float disparity{0.0f};
    };

    cv::Mat leftGray32f;
    cv::Mat rightGray32f;
    leftGray.convertTo(leftGray32f, CV_32F);
    rightGray.convertTo(rightGray32f, CV_32F);

    const int maxHamming = EnvIntValueClamped("SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT_MAX_HAMMING", 60, 1, 256);
    const int maxSecondBest = EnvIntValueClamped("SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT_SECOND_BEST", 80, 1, 256);
    const float ratio = EnvFloatValueClamped("SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT_RATIO", 0.85f, 0.1f, 1.0f);
    const float minZncc =
        EnvFloatValueClamped("SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT_MIN_ZNCC", kStereoMinZnccScore, -1.0f, 1.0f);

    std::vector<int> bestLeftForRight(rightKeypoints.size(), -1);
    std::vector<int> bestRightDistance(rightKeypoints.size(), INT_MAX);
    std::vector<OrbStereoAugmentCandidate> candidates;
    candidates.reserve(std::min(leftKeypoints.size(), rightKeypoints.size()));

    for (size_t li = 0; li < leftKeypoints.size(); ++li) {
        const cv::Point2f &leftPt = leftKeypoints[li].pt;
        if (!IsPointSafeForOrbDescriptor(leftPt, leftGray) ||
            IsPointNearExisting(leftPt, externalData.leftKeypoints)) {
            continue;
        }

        int bestRight = -1;
        int bestDistance = INT_MAX;
        int secondDistance = INT_MAX;
        for (size_t ri = 0; ri < rightKeypoints.size(); ++ri) {
            const cv::Point2f &rightPt = rightKeypoints[ri].pt;
            if (!IsPointSafeForOrbDescriptor(rightPt, rightGray) ||
                !IsStereoPairGeometricallyValid(leftPt, rightPt)) {
                continue;
            }
            const int distance = ORB_SLAM3::ORBmatcher::DescriptorDistance(
                leftDescriptors.row(static_cast<int>(li)), rightDescriptors.row(static_cast<int>(ri)));
            if (distance < bestDistance) {
                secondDistance = bestDistance;
                bestDistance = distance;
                bestRight = static_cast<int>(ri);
            } else if (distance < secondDistance) {
                secondDistance = distance;
            }
        }
        if (bestRight < 0 || bestDistance > maxHamming) {
            continue;
        }
        if (secondDistance < maxSecondBest && bestDistance > static_cast<int>(ratio * secondDistance)) {
            continue;
        }

        const cv::Point2f &rightPt = rightKeypoints[static_cast<size_t>(bestRight)].pt;
        float zncc = -1.0f;
        if (!ComputePatchZncc(leftGray32f, leftPt, rightGray32f, rightPt, zncc) || zncc < minZncc) {
            continue;
        }
        if (bestDistance >= bestRightDistance[static_cast<size_t>(bestRight)]) {
            continue;
        }
        bestRightDistance[static_cast<size_t>(bestRight)] = bestDistance;
        bestLeftForRight[static_cast<size_t>(bestRight)] = static_cast<int>(li);
        candidates.push_back(
            OrbStereoAugmentCandidate{static_cast<int>(li), bestRight, bestDistance, zncc, leftPt.x - rightPt.x});
    }

    if (candidates.empty()) {
        return 0;
    }

    std::sort(candidates.begin(), candidates.end(), [](const OrbStereoAugmentCandidate &a,
                                                       const OrbStereoAugmentCandidate &b) {
        if (a.distance != b.distance) {
            return a.distance < b.distance;
        }
        if (a.zncc != b.zncc) {
            return a.zncc > b.zncc;
        }
        return a.disparity > b.disparity;
    });

    const bool haveLeftMatch = !externalData.leftToRightMatch.empty();
    if (!haveLeftMatch) {
        externalData.leftToRightMatch.resize(externalData.leftKeypoints.size(), -1);
        const size_t alignedCount = std::min(externalData.leftKeypoints.size(), externalData.rightKeypoints.size());
        for (size_t i = 0; i < alignedCount; ++i) {
            externalData.leftToRightMatch[i] = static_cast<int>(i);
        }
    }

    size_t appended = 0;
    for (const OrbStereoAugmentCandidate &candidate : candidates) {
        if (appended >= maxExtraPairs) {
            break;
        }
        if (candidate.leftIndex < 0 || candidate.rightIndex < 0 ||
            static_cast<size_t>(candidate.leftIndex) >= leftKeypoints.size() ||
            static_cast<size_t>(candidate.rightIndex) >= rightKeypoints.size() ||
            bestLeftForRight[static_cast<size_t>(candidate.rightIndex)] != candidate.leftIndex) {
            continue;
        }

        const cv::Point2f &leftPt = leftKeypoints[static_cast<size_t>(candidate.leftIndex)].pt;
        const cv::Point2f &rightPt = rightKeypoints[static_cast<size_t>(candidate.rightIndex)].pt;
        if (IsPointNearExisting(leftPt, externalData.leftKeypoints)) {
            continue;
        }

        const int rightOutputIndex = static_cast<int>(externalData.rightKeypoints.size());
        externalData.leftKeypoints.push_back(leftKeypoints[static_cast<size_t>(candidate.leftIndex)]);
        externalData.rightKeypoints.push_back(rightKeypoints[static_cast<size_t>(candidate.rightIndex)]);
        externalData.leftDescriptors.push_back(leftDescriptors.row(candidate.leftIndex));
        externalData.rightDescriptors.push_back(rightDescriptors.row(candidate.rightIndex));
        externalData.leftToRightMatch.push_back(rightOutputIndex);
        ++appended;
    }

    if (appended > 0) {
        externalData.matchedStereoPairs = true;
    }
    return appended;
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

std::vector<StereoMatchPair> MatchStereoPairs(const SuperPointFeatureSet &left, const SuperPointFeatureSet &right,
                                              const cv::Mat &leftGray, const cv::Mat &rightGray)
{
    std::vector<StereoMatchPair> matches;
    if (!HasValidSuperPointDescriptors(left) || !HasValidSuperPointDescriptors(right) || left.keypoints.empty() ||
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
            if (yDelta > kStereoMaxEpipolarDeltaPx || disparity < ExternalStereoMinDisparityPx() ||
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

std::vector<StereoMatchPair> BuildAlignedStereoPairs(const SuperPointFeatureSet &left, const SuperPointFeatureSet &right,
                                                     const cv::Mat &leftGray, const cv::Mat &rightGray)
{
    std::vector<StereoMatchPair> matches;
    const size_t pairCount = std::min(left.keypoints.size(), right.keypoints.size());
    if (pairCount == 0 || leftGray.empty() || rightGray.empty()) {
        return matches;
    }

    cv::Mat leftGray32f;
    cv::Mat rightGray32f;
    leftGray.convertTo(leftGray32f, CV_32F);
    rightGray.convertTo(rightGray32f, CV_32F);

    matches.reserve(pairCount);
    for (size_t i = 0; i < pairCount; ++i) {
        const cv::Point2f &leftPt = left.keypoints[i];
        const cv::Point2f &rightPt = right.keypoints[i];
        const float yDelta = std::fabs(leftPt.y - rightPt.y);
        const float disparity = leftPt.x - rightPt.x;
        if (yDelta > kStereoMaxEpipolarDeltaPx || disparity < ExternalStereoMinDisparityPx() ||
            disparity > kStereoMaxDisparityPx) {
            continue;
        }

        float zncc = -1.0f;
        if (!ComputePatchZncc(leftGray32f, leftPt, rightGray32f, rightPt, zncc) || zncc < kStereoMinZnccScore) {
            continue;
        }

        const float quality = ComputeStereoCandidateQuality(1.0f, zncc, yDelta, disparity);
        if (quality < kLkMinCandidateQuality) {
            continue;
        }
        matches.push_back(
            StereoMatchPair{static_cast<int>(i), static_cast<int>(i), 1.0f, zncc, disparity, quality});
    }

    std::sort(matches.begin(), matches.end(), [](const StereoMatchPair &a, const StereoMatchPair &b) {
        if (a.quality != b.quality) {
            return a.quality > b.quality;
        }
        if (a.zncc != b.zncc) {
            return a.zncc > b.zncc;
        }
        return a.disparity < b.disparity;
    });
    return SelectGridBalancedPairs(matches, left.keypoints, leftGray.cols, leftGray.rows);
}

bool BuildExternalStereoFromFeatureMatches(const SuperPointFeatureSet &left, const SuperPointFeatureSet &right,
                                           const std::vector<StereoMatchPair> &matches,
                                           ORB_SLAM3::ExternalStereoFrameData &outData)
{
    if (!HasValidSuperPointDescriptors(left) || !HasValidSuperPointDescriptors(right) || matches.empty() ||
        left.descriptors.cols != right.descriptors.cols) {
        return false;
    }

    const int descriptorDim = left.descriptors.cols;
    cv::Mat leftDescriptors(static_cast<int>(matches.size()), descriptorDim, CV_32F);
    cv::Mat rightDescriptors(static_cast<int>(matches.size()), descriptorDim, CV_32F);
    std::vector<cv::KeyPoint> leftKeypoints;
    std::vector<cv::KeyPoint> rightKeypoints;
    leftKeypoints.reserve(matches.size());
    rightKeypoints.reserve(matches.size());

    for (const StereoMatchPair &match : matches) {
        if (match.leftIndex < 0 || match.rightIndex < 0 ||
            static_cast<size_t>(match.leftIndex) >= left.keypoints.size() ||
            static_cast<size_t>(match.rightIndex) >= right.keypoints.size()) {
            continue;
        }
        const int row = static_cast<int>(leftKeypoints.size());
        leftKeypoints.push_back(MakeKeyPoint(left.keypoints[static_cast<size_t>(match.leftIndex)]));
        rightKeypoints.push_back(MakeKeyPoint(right.keypoints[static_cast<size_t>(match.rightIndex)]));
        left.descriptors.row(match.leftIndex).copyTo(leftDescriptors.row(row));
        right.descriptors.row(match.rightIndex).copyTo(rightDescriptors.row(row));
    }

    if (leftKeypoints.empty()) {
        return false;
    }

    const int validRows = static_cast<int>(leftKeypoints.size());
    outData.leftKeypoints = std::move(leftKeypoints);
    outData.rightKeypoints = std::move(rightKeypoints);
    outData.leftDescriptors = leftDescriptors.rowRange(0, validRows).clone();
    outData.rightDescriptors = rightDescriptors.rowRange(0, validRows).clone();
    outData.matchedStereoPairs = true;
    return true;
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

cv::Mat BuildSuperPointInputImage(const cv::Mat &gray, int maxWidth, int maxHeight, float &scaleX, float &scaleY)
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

std::vector<cv::Point2f> SelectGfttPointsGridBalanced(const std::vector<cv::Point2f> &points, const cv::Size &size,
                                                      int maxTotal, int maxPerCell)
{
    if (points.empty() || size.area() <= 0 || maxTotal <= 0 || maxPerCell <= 0) {
        return {};
    }

    std::array<int, kLkGridCellCount> counts{};
    std::vector<cv::Point2f> selected;
    selected.reserve(std::min(static_cast<size_t>(maxTotal), points.size()));
    for (const cv::Point2f &point : points) {
        if (static_cast<int>(selected.size()) >= maxTotal) {
            break;
        }
        const int cell = LkGridCellForPoint(point, size);
        if (cell < 0) {
            continue;
        }
        int &cellCount = counts[static_cast<size_t>(cell)];
        if (cellCount >= maxPerCell) {
            continue;
        }
        selected.push_back(point);
        ++cellCount;
    }
    return selected;
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

bool LkHasDegradedGridCell(const std::vector<LkStereoTrack> &tracks, const cv::Size &size)
{
    if (tracks.size() < static_cast<size_t>(kLkRecoveryMinTracks)) {
        return true;
    }
    const auto counts = CountLkTracksByCell(tracks, size);
    return std::any_of(counts.begin(), counts.end(), [](int count) { return count < kLkMinTracksPerCell; });
}

bool IsHorizontalLateralFlow(const std::vector<cv::Point2f> &prevPts, const std::vector<cv::Point2f> &currPts,
                             const std::vector<uint8_t> &status, const cv::Size &size)
{
    const size_t countLimit = std::min(prevPts.size(), currPts.size());
    if (countLimit == 0 || size.width <= 0 || size.height <= 0) {
        return false;
    }

    const cv::Point2f center(0.5f * static_cast<float>(size.width), 0.5f * static_cast<float>(size.height));
    float sumDx = 0.0f;
    float sumDy = 0.0f;
    float sumRadial = 0.0f;
    int validCount = 0;
    for (size_t i = 0; i < countLimit; ++i) {
        if (i >= status.size() || !status[i]) {
            continue;
        }
        const cv::Point2f &p0 = prevPts[i];
        const cv::Point2f &p1 = currPts[i];
        if (p0.x < 1.0f || p0.y < 1.0f || p0.x >= size.width - 1 || p0.y >= size.height - 1 ||
            p1.x < 1.0f || p1.y < 1.0f || p1.x >= size.width - 1 || p1.y >= size.height - 1) {
            continue;
        }
        const cv::Point2f flow = p1 - p0;
        if (cv::norm(flow) > kLkMaxFlowPx) {
            continue;
        }
        const cv::Point2f radial = p0 - center;
        const float radius = std::sqrt(radial.x * radial.x + radial.y * radial.y);
        if (radius > 1.0f) {
            sumRadial += (flow.x * radial.x + flow.y * radial.y) / radius;
        }
        sumDx += flow.x;
        sumDy += flow.y;
        ++validCount;
    }

    if (validCount < 30) {
        return false;
    }
    const float invCount = 1.0f / static_cast<float>(validCount);
    const float meanDx = sumDx * invCount;
    const float meanDy = sumDy * invCount;
    const float meanRadial = sumRadial * invCount;
    const float absDx = std::abs(meanDx);
    return absDx >= 1.5f && absDx >= 1.8f * std::max(1.0f, std::abs(meanDy)) &&
           std::abs(meanRadial) <= 0.35f * absDx;
}

Sophus::SE3f StabilizeLkCameraDelta(const Sophus::SE3f &delta, bool horizontalLateralFlow = false)
{
    Eigen::Vector3f t = delta.translation();
    if (!std::isfinite(t.x()) || !std::isfinite(t.y()) || !std::isfinite(t.z())) {
        return Sophus::SE3f(delta.so3(), Eigen::Vector3f::Zero());
    }

    t.x() = std::clamp(t.x(), -kLkMaxLateralStepMeters, kLkMaxLateralStepMeters);
    t.y() = std::clamp(t.y(), -kLkMaxVerticalStepMeters, kLkMaxVerticalStepMeters);
    if (horizontalLateralFlow) {
        const float maxForwardFromLateral = std::max(0.015f, 0.25f * std::abs(t.x()));
        t.z() = std::clamp(t.z(), -maxForwardFromLateral, maxForwardFromLateral);
    }
    t.z() = std::clamp(t.z() * kLkForwardAxisGain, -kLkMaxForwardStepMeters, kLkMaxForwardStepMeters);
    return Sophus::SE3f(delta.so3(), t);
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

bool ReadConsistentDisparity(const cv::Mat &disp, const cv::Point2f &pt, float &disparity)
{
    disparity = 0.0f;
    if (disp.empty()) {
        return false;
    }

    const int cx = static_cast<int>(std::lround(pt.x));
    const int cy = static_cast<int>(std::lround(pt.y));
    if (cx < kLkDisparityNeighborhoodRadius || cy < kLkDisparityNeighborhoodRadius ||
        cx >= disp.cols - kLkDisparityNeighborhoodRadius || cy >= disp.rows - kLkDisparityNeighborhoodRadius) {
        return false;
    }

    const float center = disp.at<float>(cy, cx);
    if (!(center >= kLkMinConsistentDisparityPx) || center > kStereoMaxDisparityPx || !std::isfinite(center)) {
        return false;
    }

    std::vector<float> neighborhood;
    neighborhood.reserve(9);
    for (int dy = -kLkDisparityNeighborhoodRadius; dy <= kLkDisparityNeighborhoodRadius; ++dy) {
        for (int dx = -kLkDisparityNeighborhoodRadius; dx <= kLkDisparityNeighborhoodRadius; ++dx) {
            const float value = disp.at<float>(cy + dy, cx + dx);
            if (value >= kLkMinConsistentDisparityPx && value <= kStereoMaxDisparityPx && std::isfinite(value)) {
                neighborhood.push_back(value);
            }
        }
    }
    if (neighborhood.size() < 5) {
        return false;
    }

    const size_t mid = neighborhood.size() / 2;
    std::nth_element(neighborhood.begin(), neighborhood.begin() + static_cast<std::ptrdiff_t>(mid),
                     neighborhood.end());
    const float median = neighborhood[mid];
    if (std::fabs(center - median) > kLkDisparityNeighborhoodTolerancePx) {
        return false;
    }
    disparity = median;
    return true;
}

int LkPerFrameDepthBin(float z)
{
    if (z < 1.5f) {
        return 0;
    }
    if (z < 3.0f) {
        return 1;
    }
    if (z < 6.0f) {
        return 2;
    }
    return 3;
}

int LkPerFramePnPMethod()
{
    const std::string method = EnvStringValue("SMART_DRONE_LK_PER_FRAME_PNP", "epnp");
    if (method == "p3p") {
        return cv::SOLVEPNP_P3P;
    }
    if (method == "ap3p") {
        return cv::SOLVEPNP_AP3P;
    }
    return cv::SOLVEPNP_EPNP;
}

#if SMART_DRONE_HAS_VPI
bool EnsureVpiPerFrameState(std::shared_ptr<LkPerFrameVpiState> &state, const cv::Size &size,
                            const cv::Mat &map1x, const cv::Mat &map1y, const cv::Mat &map2x, const cv::Mat &map2y,
                            int maxDisparity, bool &logged)
{
    const bool recreate = !state || state->width != size.width || state->height != size.height ||
                          state->maxDisparity != maxDisparity;
    if (!recreate) {
        return true;
    }

    state = std::make_shared<LkPerFrameVpiState>();
    state->width = size.width;
    state->height = size.height;
    state->maxDisparity = maxDisparity;

    VPIStatus status = vpiStreamCreate(VPI_BACKEND_CUDA, &state->stream);
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI CUDA stream create failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        state.reset();
        logged = true;
        return false;
    }

    if (!FillVpiWarpMapFromOpenCvMaps(map1x, map1y, state->leftWarp) ||
        !FillVpiWarpMapFromOpenCvMaps(map2x, map2y, state->rightWarp)) {
        std::cerr << "[lk_per_frame_accel] VPI remap warp map build failed; fallback=cpu_sgbm\n";
        state.reset();
        logged = true;
        return false;
    }

    status = vpiCreateRemap(VPI_BACKEND_CUDA, &state->leftWarp, &state->leftRemapPayload);
    if (status == VPI_SUCCESS) {
        status = vpiCreateRemap(VPI_BACKEND_CUDA, &state->rightWarp, &state->rightRemapPayload);
    }
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI remap payload create failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        state.reset();
        logged = true;
        return false;
    }

    const uint64_t imageBackends = VPI_BACKEND_CUDA | VPI_BACKEND_CPU;
    status = vpiImageCreate(size.width, size.height, VPI_IMAGE_FORMAT_U8, imageBackends, &state->leftRect);
    if (status == VPI_SUCCESS) {
        status = vpiImageCreate(size.width, size.height, VPI_IMAGE_FORMAT_U8, imageBackends, &state->rightRect);
    }
    if (status == VPI_SUCCESS) {
        status = vpiImageCreate(size.width, size.height, VPI_IMAGE_FORMAT_U8, imageBackends, &state->prevLeftRect);
    }
    if (status == VPI_SUCCESS) {
        status = vpiImageCreate(size.width, size.height, VPI_IMAGE_FORMAT_U8, imageBackends, &state->prevRightRect);
    }
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI rectified image create failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        state.reset();
        logged = true;
        return false;
    }

    VPIStereoDisparityEstimatorCreationParams createParams{};
    status = vpiInitStereoDisparityEstimatorCreationParams(&createParams);
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI stereo creation params init failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        state.reset();
        logged = true;
        return false;
    }
    createParams.maxDisparity = maxDisparity;
    createParams.downscaleFactor = 1;
    createParams.includeDiagonals = EnvIntValue("SMART_DRONE_VPI_STEREO_DIAG", kVpiStereoIncludeDiagonals);
    status = vpiCreateStereoDisparityEstimator(VPI_BACKEND_CUDA, size.width, size.height, VPI_IMAGE_FORMAT_U8,
                                               &createParams, &state->stereoPayload);
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI CUDA stereo payload create failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        state.reset();
        logged = true;
        return false;
    }

    status = vpiImageCreate(size.width, size.height, VPI_IMAGE_FORMAT_S16, VPI_BACKEND_CUDA | VPI_BACKEND_CPU,
                            &state->disparity);
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI disparity image create failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        state.reset();
        logged = true;
        return false;
    }

    status = vpiPyramidCreate(size.width, size.height, VPI_IMAGE_FORMAT_U8, 4, 0.5f,
                              VPI_BACKEND_CUDA | VPI_BACKEND_CPU, &state->prevPyr);
    if (status == VPI_SUCCESS) {
        status = vpiPyramidCreate(size.width, size.height, VPI_IMAGE_FORMAT_U8, 4, 0.5f,
                                  VPI_BACKEND_CUDA | VPI_BACKEND_CPU, &state->curPyr);
    }
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI pyramid create failed: " << VpiStatusName(status)
                  << "; fallback=cpu_lk\n";
        state.reset();
        logged = true;
        return false;
    }

    status = vpiCreateOpticalFlowPyrLK(VPI_BACKEND_CUDA, size.width, size.height, VPI_IMAGE_FORMAT_U8, 4, 0.5f,
                                       &state->lkPayload);
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI PyrLK payload create failed: " << VpiStatusName(status)
                  << "; fallback=cpu_lk\n";
        state.reset();
        logged = true;
        return false;
    }

    status = vpiArrayCreate(kLkGfttPerFrameMaxCorners, VPI_ARRAY_TYPE_KEYPOINT_F32,
                            VPI_BACKEND_CUDA | VPI_BACKEND_CPU, &state->prevPts);
    if (status == VPI_SUCCESS) {
        status = vpiArrayCreate(kLkGfttPerFrameMaxCorners, VPI_ARRAY_TYPE_KEYPOINT_F32,
                                VPI_BACKEND_CUDA | VPI_BACKEND_CPU, &state->curPts);
    }
    if (status == VPI_SUCCESS) {
        status = vpiArrayCreate(kLkGfttPerFrameMaxCorners, VPI_ARRAY_TYPE_U8, VPI_BACKEND_CUDA | VPI_BACKEND_CPU,
                                &state->trackStatus);
    }
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI LK array create failed: " << VpiStatusName(status)
                  << "; fallback=cpu_lk\n";
        state.reset();
        logged = true;
        return false;
    }

    std::cerr << "[lk_per_frame_accel] backend=vpi_cuda stages=remap,stereo_disparity,pyr_lk"
              << " pyr_lk=available size=" << size.width
              << "x" << size.height << " max_disparity=" << maxDisparity
              << " conf=" << EnvIntValue("SMART_DRONE_VPI_STEREO_CONF", kVpiStereoConfidenceThreshold)
              << " p1=" << EnvIntValue("SMART_DRONE_VPI_STEREO_P1", kVpiStereoP1)
              << " p2=" << EnvIntValue("SMART_DRONE_VPI_STEREO_P2", kVpiStereoP2)
              << " uniqueness=" << EnvFloatValue("SMART_DRONE_VPI_STEREO_UNIQUENESS", kVpiStereoUniqueness)
              << " diag=" << EnvIntValue("SMART_DRONE_VPI_STEREO_DIAG", kVpiStereoIncludeDiagonals) << "\n";
    logged = true;
    return true;
}

bool StoreVpiPreviousRectified(std::shared_ptr<LkPerFrameVpiState> &state)
{
    if (!state || state->stream == nullptr || state->leftRect == nullptr || state->rightRect == nullptr ||
        state->prevLeftRect == nullptr || state->prevRightRect == nullptr) {
        return false;
    }
    VPIStatus status = vpiSubmitConvertImageFormat(state->stream, VPI_BACKEND_CUDA, state->leftRect,
                                                   state->prevLeftRect, nullptr);
    if (status == VPI_SUCCESS) {
        status = vpiSubmitConvertImageFormat(state->stream, VPI_BACKEND_CUDA, state->rightRect, state->prevRightRect,
                                             nullptr);
    }
    if (status == VPI_SUCCESS) {
        status = vpiStreamSync(state->stream);
    }
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI previous rect copy failed: " << VpiStatusName(status)
                  << "; fallback=cpu_cache\n";
        state->hasPrevRect = false;
        return false;
    }
    state->hasPrevRect = true;
    return true;
}

bool VpiRemapCurrentStereo(const cv::Mat &leftRaw, const cv::Mat &rightRaw, cv::Mat &leftRect, cv::Mat &rightRect,
                           std::shared_ptr<LkPerFrameVpiState> &state, const cv::Mat &map1x, const cv::Mat &map1y,
                           const cv::Mat &map2x, const cv::Mat &map2y, bool &logged)
{
    if (leftRaw.empty() || rightRaw.empty() || leftRaw.size() != rightRaw.size() || leftRaw.type() != CV_8UC1 ||
        rightRaw.type() != CV_8UC1 || map1x.empty() || map2x.empty()) {
        return false;
    }
    const int maxDisparity = std::clamp(((leftRaw.cols / 8 + 15) / 16) * 16, 16, 256);
    if (!EnsureVpiPerFrameState(state, leftRaw.size(), map1x, map1y, map2x, map2y, maxDisparity, logged)) {
        return false;
    }

    VPIImage leftWrapper = nullptr;
    VPIImage rightWrapper = nullptr;
    VPIStatus status = vpiImageCreateWrapperOpenCVMat(leftRaw, VPI_IMAGE_FORMAT_U8, VPI_BACKEND_CUDA, &leftWrapper);
    if (status == VPI_SUCCESS) {
        status = vpiImageCreateWrapperOpenCVMat(rightRaw, VPI_IMAGE_FORMAT_U8, VPI_BACKEND_CUDA, &rightWrapper);
    }
    if (status == VPI_SUCCESS) {
        status = vpiSubmitRemap(state->stream, VPI_BACKEND_CUDA, state->leftRemapPayload, leftWrapper, state->leftRect,
                                VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0);
    }
    if (status == VPI_SUCCESS) {
        status = vpiSubmitRemap(state->stream, VPI_BACKEND_CUDA, state->rightRemapPayload, rightWrapper,
                                state->rightRect, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0);
    }
    if (status == VPI_SUCCESS) {
        status = vpiStreamSync(state->stream);
    }
    if (leftWrapper != nullptr) {
        vpiImageDestroy(leftWrapper);
    }
    if (rightWrapper != nullptr) {
        vpiImageDestroy(rightWrapper);
    }
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI remap submit failed: " << VpiStatusName(status)
                  << "; fallback=cpu_remap\n";
        state.reset();
        return false;
    }
    leftRect = DownloadVpiU8Image(state->leftRect);
    rightRect = DownloadVpiU8Image(state->rightRect);
    return !leftRect.empty() && !rightRect.empty();
}

void ConfigureVpiStereoParams(VPIStereoDisparityEstimatorParams &params, int maxDisparity)
{
    params.maxDisparity = maxDisparity;
    params.confidenceThreshold = EnvIntValue("SMART_DRONE_VPI_STEREO_CONF", kVpiStereoConfidenceThreshold);
    params.p1 = EnvIntValue("SMART_DRONE_VPI_STEREO_P1", kVpiStereoP1);
    params.p2 = EnvIntValue("SMART_DRONE_VPI_STEREO_P2", kVpiStereoP2);
    params.uniqueness = EnvFloatValue("SMART_DRONE_VPI_STEREO_UNIQUENESS", kVpiStereoUniqueness);
}

bool DownloadVpiDisparity(const cv::Size &size, VPIImage disparityImage, cv::Mat &disp)
{
    VPIImageData data{};
    VPIStatus status = vpiImageLockData(disparityImage, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &data);
    if (status != VPI_SUCCESS || data.bufferType != VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR ||
        data.buffer.pitch.numPlanes < 1 || data.buffer.pitch.planes[0].data == nullptr) {
        std::cerr << "[lk_per_frame_accel] VPI disparity lock failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        return false;
    }

    const auto &plane = data.buffer.pitch.planes[0];
    cv::Mat disp16(size.height, size.width, CV_16S, plane.data, static_cast<size_t>(plane.pitchBytes));
    disp16.convertTo(disp, CV_32F, 1.0 / 32.0);
    vpiImageUnlock(disparityImage);
    return true;
}

bool ComputeVpiCudaDisparityImages(const cv::Size &size, VPIImage leftImage, VPIImage rightImage, cv::Mat &disp,
                                   std::shared_ptr<LkPerFrameVpiState> &state)
{
    if (!state || state->stream == nullptr || state->stereoPayload == nullptr || state->disparity == nullptr ||
        leftImage == nullptr || rightImage == nullptr) {
        return false;
    }

    VPIStereoDisparityEstimatorParams params{};
    VPIStatus status = vpiInitStereoDisparityEstimatorParams(&params);
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI stereo params init failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        return false;
    }
    ConfigureVpiStereoParams(params, state->maxDisparity);

    status = vpiSubmitStereoDisparityEstimator(state->stream, VPI_BACKEND_CUDA, state->stereoPayload, leftImage,
                                               rightImage, state->disparity, nullptr, &params);
    if (status == VPI_SUCCESS) {
        status = vpiStreamSync(state->stream);
    }
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI stereo submit failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        return false;
    }
    return DownloadVpiDisparity(size, state->disparity, disp);
}

bool ComputeVpiCudaDisparity(const cv::Mat &left, const cv::Mat &right, cv::Mat &disp,
                             std::shared_ptr<LkPerFrameVpiState> &state, bool &logged)
{
    if (left.empty() || right.empty() || left.size() != right.size() || left.type() != CV_8UC1 || right.type() != CV_8UC1) {
        return false;
    }

    const int maxDisparity = std::clamp(((left.cols / 8 + 15) / 16) * 16, 16, 256);
    const bool recreate = !state || state->width != left.cols || state->height != left.rows ||
                          state->maxDisparity != maxDisparity;
    if (recreate) {
        state = std::make_shared<LkPerFrameVpiState>();
        state->width = left.cols;
        state->height = left.rows;
        state->maxDisparity = maxDisparity;

        VPIStatus status = vpiStreamCreate(VPI_BACKEND_CUDA, &state->stream);
        if (status != VPI_SUCCESS) {
            if (!logged) {
                std::cerr << "[lk_per_frame_accel] VPI CUDA stream create failed: " << VpiStatusName(status)
                          << "; fallback=cpu_sgbm\n";
                logged = true;
            }
            state.reset();
            return false;
        }

        VPIStereoDisparityEstimatorCreationParams createParams{};
        status = vpiInitStereoDisparityEstimatorCreationParams(&createParams);
        if (status != VPI_SUCCESS) {
            std::cerr << "[lk_per_frame_accel] VPI stereo creation params init failed: " << VpiStatusName(status)
                      << "; fallback=cpu_sgbm\n";
            state.reset();
            logged = true;
            return false;
        }
        createParams.maxDisparity = maxDisparity;
        createParams.downscaleFactor = 1;
        createParams.includeDiagonals = EnvIntValue("SMART_DRONE_VPI_STEREO_DIAG", kVpiStereoIncludeDiagonals);

        status = vpiCreateStereoDisparityEstimator(VPI_BACKEND_CUDA, left.cols, left.rows, VPI_IMAGE_FORMAT_U8,
                                                   &createParams, &state->stereoPayload);
        if (status != VPI_SUCCESS) {
            std::cerr << "[lk_per_frame_accel] VPI CUDA stereo payload create failed: " << VpiStatusName(status)
                      << "; fallback=cpu_sgbm\n";
            state.reset();
            logged = true;
            return false;
        }

        status = vpiImageCreate(left.cols, left.rows, VPI_IMAGE_FORMAT_S16, VPI_BACKEND_CUDA | VPI_BACKEND_CPU,
                                &state->disparity);
        if (status != VPI_SUCCESS) {
            std::cerr << "[lk_per_frame_accel] VPI disparity image create failed: " << VpiStatusName(status)
                      << "; fallback=cpu_sgbm\n";
            state.reset();
            logged = true;
            return false;
        }

        status = vpiImageCreateWrapperOpenCVMat(left, VPI_IMAGE_FORMAT_U8, VPI_BACKEND_CUDA, &state->leftWrapper);
        if (status == VPI_SUCCESS) {
            status = vpiImageCreateWrapperOpenCVMat(right, VPI_IMAGE_FORMAT_U8, VPI_BACKEND_CUDA, &state->rightWrapper);
        }
        if (status != VPI_SUCCESS) {
            std::cerr << "[lk_per_frame_accel] VPI OpenCV wrapper create failed: " << VpiStatusName(status)
                      << "; fallback=cpu_sgbm\n";
            state.reset();
            logged = true;
            return false;
        }

        std::cerr << "[lk_per_frame_accel] backend=vpi_cuda stage=stereo_disparity size=" << left.cols << "x"
                  << left.rows << " max_disparity=" << maxDisparity << "\n";
        logged = true;
    } else {
        VPIStatus status = VPI_SUCCESS;
        if (state->leftWrapper == nullptr) {
            status = vpiImageCreateWrapperOpenCVMat(left, VPI_IMAGE_FORMAT_U8, VPI_BACKEND_CUDA, &state->leftWrapper);
        } else {
            status = vpiImageSetWrappedOpenCVMat(state->leftWrapper, left);
        }
        if (status == VPI_SUCCESS) {
            if (state->rightWrapper == nullptr) {
                status =
                    vpiImageCreateWrapperOpenCVMat(right, VPI_IMAGE_FORMAT_U8, VPI_BACKEND_CUDA, &state->rightWrapper);
            } else {
                status = vpiImageSetWrappedOpenCVMat(state->rightWrapper, right);
            }
        }
        if (status != VPI_SUCCESS) {
            std::cerr << "[lk_per_frame_accel] VPI wrapper update failed: " << VpiStatusName(status)
                      << "; fallback=cpu_sgbm\n";
            state.reset();
            return false;
        }
    }

    VPIStereoDisparityEstimatorParams params{};
    VPIStatus status = vpiInitStereoDisparityEstimatorParams(&params);
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI stereo params init failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        return false;
    }
    ConfigureVpiStereoParams(params, maxDisparity);

    status = vpiSubmitStereoDisparityEstimator(state->stream, VPI_BACKEND_CUDA, state->stereoPayload,
                                               state->leftWrapper, state->rightWrapper, state->disparity, nullptr,
                                               &params);
    if (status == VPI_SUCCESS) {
        status = vpiStreamSync(state->stream);
    }
    if (status != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI stereo submit failed: " << VpiStatusName(status)
                  << "; fallback=cpu_sgbm\n";
        return false;
    }

    return DownloadVpiDisparity(left.size(), state->disparity, disp);
}

bool ComputeVpiCudaPyrLk(const cv::Mat &prevLeft, VPIImage prevLeftImage, VPIImage curLeftImage,
                         const std::vector<cv::Point2f> &pts0, std::vector<cv::Point2f> &pts1,
                         std::vector<uint8_t> &statusOut, std::shared_ptr<LkPerFrameVpiState> &state)
{
    pts1.clear();
    statusOut.clear();
    if (!state || state->stream == nullptr || state->lkPayload == nullptr || state->prevPyr == nullptr ||
        state->curPyr == nullptr || state->prevPts == nullptr || state->curPts == nullptr ||
        state->trackStatus == nullptr || curLeftImage == nullptr || pts0.empty()) {
        return false;
    }
    if (prevLeftImage == nullptr && prevLeft.empty()) {
        return false;
    }

    const int count = std::min<int>(static_cast<int>(pts0.size()), kLkGfttPerFrameMaxCorners);
    VPIArrayData prevData{};
    VPIArrayData curInitData{};
    VPIStatus vstatus =
        vpiArrayLockData(state->prevPts, VPI_LOCK_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &prevData);
    if (vstatus != VPI_SUCCESS || prevData.bufferType != VPI_ARRAY_BUFFER_HOST_AOS ||
        prevData.buffer.aos.data == nullptr || prevData.buffer.aos.sizePointer == nullptr) {
        std::cerr << "[lk_per_frame_accel] VPI prev point upload lock failed: " << VpiStatusName(vstatus)
                  << "; fallback=cpu_lk\n";
        return false;
    }
    vstatus = vpiArrayLockData(state->curPts, VPI_LOCK_WRITE, VPI_ARRAY_BUFFER_HOST_AOS, &curInitData);
    if (vstatus != VPI_SUCCESS || curInitData.bufferType != VPI_ARRAY_BUFFER_HOST_AOS ||
        curInitData.buffer.aos.data == nullptr || curInitData.buffer.aos.sizePointer == nullptr) {
        std::cerr << "[lk_per_frame_accel] VPI current point init lock failed: " << VpiStatusName(vstatus)
                  << "; fallback=cpu_lk\n";
        vpiArrayUnlock(state->prevPts);
        return false;
    }
    auto *prevKeypoints = static_cast<VPIKeypointF32 *>(prevData.buffer.aos.data);
    auto *curInitKeypoints = static_cast<VPIKeypointF32 *>(curInitData.buffer.aos.data);
    for (int i = 0; i < count; ++i) {
        prevKeypoints[i].x = pts0[static_cast<size_t>(i)].x;
        prevKeypoints[i].y = pts0[static_cast<size_t>(i)].y;
        curInitKeypoints[i].x = pts0[static_cast<size_t>(i)].x;
        curInitKeypoints[i].y = pts0[static_cast<size_t>(i)].y;
    }
    *prevData.buffer.aos.sizePointer = count;
    *curInitData.buffer.aos.sizePointer = count;
    vpiArrayUnlock(state->curPts);
    vpiArrayUnlock(state->prevPts);
    vpiArraySetSize(state->trackStatus, count);

    VPIImage prevWrapper = nullptr;
    VPIImage prevImage = prevLeftImage;
    if (prevImage == nullptr) {
        vstatus = vpiImageCreateWrapperOpenCVMat(prevLeft, VPI_IMAGE_FORMAT_U8, VPI_BACKEND_CUDA, &prevWrapper);
        prevImage = prevWrapper;
    }
    if (vstatus == VPI_SUCCESS) {
        vstatus = vpiSubmitGaussianPyramidGenerator(state->stream, VPI_BACKEND_CUDA, prevImage, state->prevPyr,
                                                    VPI_BORDER_CLAMP);
    }
    if (vstatus == VPI_SUCCESS) {
        vstatus = vpiSubmitGaussianPyramidGenerator(state->stream, VPI_BACKEND_CUDA, curLeftImage, state->curPyr,
                                                    VPI_BORDER_CLAMP);
    }
    VPIOpticalFlowPyrLKParams lkParams{};
    if (vstatus == VPI_SUCCESS) {
        vstatus = vpiInitOpticalFlowPyrLKParams(&lkParams);
        lkParams.windowDimension = 21;
        lkParams.numIterations = 24;
        lkParams.useInitialFlow = 1;
    }
    if (vstatus == VPI_SUCCESS) {
        vstatus = vpiSubmitOpticalFlowPyrLK(state->stream, VPI_BACKEND_CUDA, state->lkPayload, state->prevPyr,
                                            state->curPyr, state->prevPts, state->curPts, state->trackStatus,
                                            &lkParams);
    }
    if (vstatus == VPI_SUCCESS) {
        vstatus = vpiStreamSync(state->stream);
    }
    if (prevWrapper != nullptr) {
        vpiImageDestroy(prevWrapper);
    }
    if (vstatus != VPI_SUCCESS) {
        std::cerr << "[lk_per_frame_accel] VPI PyrLK submit failed: " << VpiStatusName(vstatus)
                  << "; fallback=cpu_lk\n";
        return false;
    }

    VPIArrayData curData{};
    VPIArrayData statusData{};
    vstatus = vpiArrayLockData(state->curPts, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &curData);
    if (vstatus == VPI_SUCCESS) {
        vstatus = vpiArrayLockData(state->trackStatus, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &statusData);
    }
    if (vstatus != VPI_SUCCESS || curData.buffer.aos.data == nullptr || statusData.buffer.aos.data == nullptr) {
        std::cerr << "[lk_per_frame_accel] VPI LK result lock failed: " << VpiStatusName(vstatus)
                  << "; fallback=cpu_lk\n";
        if (curData.buffer.aos.data != nullptr) {
            vpiArrayUnlock(state->curPts);
        }
        return false;
    }

    auto *curKeypoints = static_cast<VPIKeypointF32 *>(curData.buffer.aos.data);
    auto *trackStatus = static_cast<uint8_t *>(statusData.buffer.aos.data);
    pts1.resize(static_cast<size_t>(count));
    statusOut.resize(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        pts1[static_cast<size_t>(i)] = cv::Point2f(curKeypoints[i].x, curKeypoints[i].y);
        statusOut[static_cast<size_t>(i)] = trackStatus[i] == 0 ? 1 : 0;
    }
    vpiArrayUnlock(state->trackStatus);
    vpiArrayUnlock(state->curPts);
    return true;
}
#else
bool VpiRemapCurrentStereo(const cv::Mat &, const cv::Mat &, cv::Mat &, cv::Mat &,
                           std::shared_ptr<LkPerFrameVpiState> &, const cv::Mat &, const cv::Mat &, const cv::Mat &,
                           const cv::Mat &, bool &)
{
    return false;
}

bool ComputeVpiCudaDisparity(const cv::Mat &, const cv::Mat &, cv::Mat &, std::shared_ptr<LkPerFrameVpiState> &,
                             bool &logged)
{
    if (!logged) {
        std::cerr << "[lk_per_frame_accel] VPI support not compiled; fallback=cpu_sgbm\n";
        logged = true;
    }
    return false;
}

bool ComputeVpiCudaPyrLk(const cv::Mat &, void *, void *, const std::vector<cv::Point2f> &,
                         std::vector<cv::Point2f> &, std::vector<uint8_t> &,
                         std::shared_ptr<LkPerFrameVpiState> &)
{
    return false;
}
#endif

std::vector<LkStereoTrack> BuildLkGfttStereoSeeds(const cv::Mat &leftGray, const cv::Mat &rightGray)
{
    std::vector<LkStereoTrack> seeds;
    if (leftGray.empty() || rightGray.empty()) {
        return seeds;
    }

    cv::Mat disp32f;
    {
        const int numDisparities = std::max(16, ((leftGray.cols / 8 + 15) / 16) * 16);
        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
            0, numDisparities, 5, 8 * 5 * 5, 32 * 5 * 5, 1, 31, 8, 60, 2, cv::StereoSGBM::MODE_SGBM_3WAY);
        cv::Mat disp16;
        sgbm->compute(leftGray, rightGray, disp16);
        disp16.convertTo(disp32f, CV_32F, 1.0 / 16.0);
    }

    std::vector<cv::Point2f> leftPoints;
    cv::goodFeaturesToTrack(leftGray, leftPoints, kLkGfttMaxCorners, kLkGfttQualityLevel, kLkGfttMinDistancePx,
                            cv::Mat(), kLkGfttBlockSize, false, kLkGfttHarrisK);
    if (leftPoints.empty()) {
        return seeds;
    }

    cv::Mat left32f;
    cv::Mat right32f;
    leftGray.convertTo(left32f, CV_32F);
    rightGray.convertTo(right32f, CV_32F);
    seeds.reserve(leftPoints.size());
    for (const cv::Point2f &leftPt : leftPoints) {
        const int ix = static_cast<int>(std::lround(leftPt.x));
        const int iy = static_cast<int>(std::lround(leftPt.y));
        if (ix < 0 || iy < 0 || ix >= disp32f.cols || iy >= disp32f.rows) {
            continue;
        }
        const float expectedDisparity = disp32f.at<float>(iy, ix);
        if (!(expectedDisparity >= ExternalStereoMinDisparityPx()) || expectedDisparity > kStereoMaxDisparityPx ||
            !std::isfinite(expectedDisparity)) {
            continue;
        }
        cv::Point2f rightPt;
        float zncc = -1.0f;
        if (!FindRightPointByStereoZnccAroundDisparity(left32f, leftPt, right32f, expectedDisparity, rightPt, zncc)) {
            continue;
        }
        seeds.push_back(LkStereoTrack{leftPt, rightPt, std::clamp((zncc + 1.0f) * 0.5f, 0.0f, 1.0f), 0});
    }
    return seeds;
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
        if (LkTrackNearExisting(seed.left, seed.right, tracks)) {
            continue;
        }
        tracks.push_back(seed);
        ++cellCount;
    }
}

bool IsSuperPointTrackingStateSafe(int trackingState)
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


} // namespace smartdrone::adapters::slam
