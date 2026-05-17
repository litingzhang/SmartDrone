#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "adapters/slam/slam_mode_state.h"
#include "core/ports/slam_engine.h"

namespace smartdrone::adapters::slam {

std::vector<cv::Point2f> ExtractLkTrackLeftPoints(const std::vector<LkStereoTrack> &tracks);
std::vector<cv::Point2f> ExtractLkTrackRightPoints(const std::vector<LkStereoTrack> &tracks);

std::vector<LkStereoTrack> KeepLkTracksByIndices(const std::vector<LkStereoTrack> &tracks,
                                                 const std::vector<int> &indices);

bool RefreshLkStereoSeedsIfNeeded(SlamModeSharedState &state, const cv::Mat &leftRect,
                                  const cv::Mat &rightRect, uint64_t frameId, bool force);

void UpdateLkTracksAfterPoseEstimate(SlamModeSharedState &state, const cv::Mat &leftRect,
                                     const cv::Mat &rightRect, uint64_t frameId,
                                     std::vector<LkStereoTrack> trackedTracks, int inlierCount);

void CopyLkTrackFeaturesToOutput(const std::vector<LkStereoTrack> &tracks,
                                 core::ports::SlamOutput &out);

} // namespace smartdrone::adapters::slam
