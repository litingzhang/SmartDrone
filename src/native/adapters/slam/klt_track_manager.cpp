#include "adapters/slam/klt_track_manager.h"

#include "adapters/slam/klt_mode_utils.h"

#include <algorithm>

namespace SmartDrone::adapters::slam {

std::vector<cv::Point2f> ExtractLkTrackLeftPoints(const std::vector<LkStereoTrack> &tracks)
{
    std::vector<cv::Point2f> points;
    points.reserve(tracks.size());
    for (const LkStereoTrack &track : tracks) {
        points.push_back(track.left);
    }
    return points;
}

std::vector<cv::Point2f> ExtractLkTrackRightPoints(const std::vector<LkStereoTrack> &tracks)
{
    std::vector<cv::Point2f> points;
    points.reserve(tracks.size());
    for (const LkStereoTrack &track : tracks) {
        points.push_back(track.right);
    }
    return points;
}

std::vector<LkStereoTrack> KeepLkTracksByIndices(const std::vector<LkStereoTrack> &tracks,
                                                 const std::vector<int> &indices)
{
    std::vector<LkStereoTrack> selected;
    selected.reserve(indices.size());
    for (int index : indices) {
        if (index >= 0 && static_cast<size_t>(index) < tracks.size()) {
            selected.push_back(tracks[static_cast<size_t>(index)]);
        }
    }
    return selected;
}

bool RefreshLkStereoSeedsIfNeeded(SlamModeSharedState &state, const cv::Mat &leftRect,
                                  const cv::Mat &rightRect, uint64_t frameId, bool force)
{
    const bool cadenceDue =
        state.m_lkLastSeedFrameId == 0 || frameId >= state.m_lkLastSeedFrameId + kLkGridRefillIntervalFrames;
    if (!force && (!cadenceDue || !LkHasDegradedGridCell(state.m_lkTracks, leftRect.size()))) {
        return false;
    }

    std::vector<LkStereoTrack> seeds = BuildLkGfttStereoSeeds(leftRect, rightRect);
    state.m_lkLastSeedFrameId = frameId;
    if (seeds.empty()) {
        return false;
    }

    const size_t before = state.m_lkTracks.size();
    AppendLkSeedsForDegradedCells(seeds, leftRect.size(), state.m_lkTracks);
    return state.m_lkTracks.size() > before;
}

void UpdateLkTracksAfterPoseEstimate(SlamModeSharedState &state, const cv::Mat &leftRect,
                                     const cv::Mat &rightRect, uint64_t frameId,
                                     std::vector<LkStereoTrack> trackedTracks, int inlierCount)
{
    const bool hardRecoveryRefill =
        trackedTracks.size() < static_cast<size_t>(kLkHardRecoveryMinTracks) ||
        inlierCount < kLkHardRecoveryMinInliers;
    state.m_lkTracks = hardRecoveryRefill ? std::vector<LkStereoTrack>{}
                                          : SelectLkTracksGridBalanced(trackedTracks, leftRect.size());
    if (hardRecoveryRefill) {
        RefreshLkStereoSeedsIfNeeded(state, leftRect, rightRect, frameId, true);
    }
    RefreshLkStereoSeedsIfNeeded(state, leftRect, rightRect, frameId, false);
    state.m_lkTracks = SelectLkTracksGridBalanced(state.m_lkTracks, leftRect.size());
}

void CopyLkTrackFeaturesToOutput(const std::vector<LkStereoTrack> &tracks,
                                 core::ports::SlamOutput &out)
{
    core::ports::CopyStereoTracksToOutput(tracks, out);
}

} // namespace SmartDrone::adapters::slam
