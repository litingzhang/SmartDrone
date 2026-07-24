#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "adapters/slam/engine/slam_mode_state.h"

namespace SmartDrone::Adapters::Slam {

inline constexpr float LK_MIN_DEPTH_METERS = 0.35f;
inline constexpr float LK_MAX_DEPTH_METERS = 12.0f;
inline constexpr float LK_MAX_FLOW_PX = 96.0f;
inline constexpr float LK_MAX_STEP_METERS = 0.35f;
inline constexpr int LK_MIN_PNP_POINTS = 12;
inline constexpr int LK_MIN_PNP_INLIERS = 10;
inline constexpr int LK_HARD_RECOVERY_MIN_TRACKS = 10;
inline constexpr int LK_HARD_RECOVERY_MIN_INLIERS = 10;
inline constexpr uint64_t LK_GRID_REFILL_INTERVAL_FRAMES = 30;
inline constexpr int LK_GFTT_PER_FRAME_MAX_CORNERS = 960;
inline constexpr int LK_GFTT_PER_FRAME_MAX_CORNERS_PER_CELL = 20;
inline constexpr float LK_PER_FRAME_FORWARD_BACKWARD_MAX_ERR_PX = 1.25f;
inline constexpr int LK_PER_FRAME_PNP_SELECT_GRID_COLS = 8;
inline constexpr int LK_PER_FRAME_PNP_SELECT_GRID_ROWS = 6;
inline constexpr int LK_PER_FRAME_PNP_DEPTH_BINS = 4;
inline constexpr int LK_PER_FRAME_PNP_MAX_PER_GRID_DEPTH_BIN = 4;
inline constexpr int LK_PER_FRAME_DEFAULT_PNP_ITERATIONS = 80;
inline constexpr double LK_PER_FRAME_DEFAULT_PNP_CONFIDENCE = 0.995;
inline constexpr double LK_GFTT_QUALITY_LEVEL = 0.01;
inline constexpr double LK_GFTT_MIN_DISTANCE_PX = 8.0;
inline constexpr int LK_GFTT_BLOCK_SIZE = 7;
inline constexpr double LK_GFTT_HARRIS_K = 0.04;
inline constexpr double LK_PER_FRAME_PNP_REPROJ_THRESHOLD_PX = 0.5;
inline constexpr uint64_t LK_LOOP_KEYFRAME_INTERVAL_FRAMES = 30;
inline constexpr uint64_t LK_LOOP_MIN_AGE_FRAMES = 300;
inline constexpr uint64_t LK_LOOP_COOLDOWN_FRAMES = 200;
inline constexpr size_t LK_LOOP_MAX_KEYFRAMES = 160;
inline constexpr double LK_LOOP_MIN_SIMILARITY = 0.60;

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
