#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "adapters/slam/slam_mode_state.h"

namespace SmartDrone::Adapters::Slam {

inline constexpr float kLkMinDepthMeters = 0.35f;
inline constexpr float kLkMaxDepthMeters = 12.0f;
inline constexpr float kLkMaxFlowPx = 96.0f;
inline constexpr float kLkMaxStepMeters = 0.35f;
inline constexpr int kLkMinPnPPoints = 12;
inline constexpr int kLkMinPnPInliers = 10;
inline constexpr int kLkHardRecoveryMinTracks = 10;
inline constexpr int kLkHardRecoveryMinInliers = 10;
inline constexpr uint64_t kLkGridRefillIntervalFrames = 30;
inline constexpr int kLkGfttPerFrameMaxCorners = 960;
inline constexpr int kLkGfttPerFrameMaxCornersPerCell = 20;
inline constexpr float kLkPerFrameForwardBackwardMaxErrPx = 1.25f;
inline constexpr int kLkPerFramePnPSelectGridCols = 8;
inline constexpr int kLkPerFramePnPSelectGridRows = 6;
inline constexpr int kLkPerFramePnPDepthBins = 4;
inline constexpr int kLkPerFramePnPMaxPerGridDepthBin = 4;
inline constexpr int kLkPerFrameDefaultPnPIterations = 80;
inline constexpr double kLkPerFrameDefaultPnPConfidence = 0.995;
inline constexpr double kLkGfttQualityLevel = 0.01;
inline constexpr double kLkGfttMinDistancePx = 8.0;
inline constexpr int kLkGfttBlockSize = 7;
inline constexpr double kLkGfttHarrisK = 0.04;
inline constexpr double kLkPerFramePnPReprojThresholdPx = 3.0;
inline constexpr uint64_t kLkLoopKeyframeIntervalFrames = 30;
inline constexpr uint64_t kLkLoopMinAgeFrames = 300;
inline constexpr uint64_t kLkLoopCooldownFrames = 200;
inline constexpr size_t kLkLoopMaxKeyframes = 160;
inline constexpr double kLkLoopMinSimilarity = 0.60;

cv::Mat BuildLkLoopImageDescriptor(const cv::Mat &gray);
double LkLoopDescriptorSimilarity(const cv::Mat &lhs, const cv::Mat &rhs);
std::vector<cv::Point2f> SelectGfttPointsGridBalanced(const std::vector<cv::Point2f> &points,
                                                      const cv::Size &size, int maxTotal, int maxPerCell);
std::vector<LkStereoTrack> SelectLkTracksGridBalanced(const std::vector<LkStereoTrack> &tracks,
                                                      const cv::Size &size);
bool LkHasDegradedGridCell(const std::vector<LkStereoTrack> &tracks, const cv::Size &size);
bool IsHorizontalLateralFlow(const std::vector<cv::Point2f> &prevPts, const std::vector<cv::Point2f> &currPts,
                             const std::vector<uint8_t> &status, const cv::Size &size);
Sophus::SE3f StabilizeLkCameraDelta(const Sophus::SE3f &delta, bool horizontalLateralFlow = false);
bool ReadConsistentDisparity(const cv::Mat &disp, const cv::Point2f &pt, float &disparity);
int LkPerFrameDepthBin(float z);
int LkPerFramePnPMethod();

std::vector<LkStereoTrack> BuildLkGfttStereoSeeds(const cv::Mat &leftGray, const cv::Mat &rightGray);
void AppendLkSeedsForDegradedCells(const std::vector<LkStereoTrack> &seeds, const cv::Size &size,
                                   std::vector<LkStereoTrack> &tracks);

} // namespace SmartDrone::Adapters::Slam
