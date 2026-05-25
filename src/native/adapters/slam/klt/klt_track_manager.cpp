#include "adapters/slam/klt/klt_track_manager.h"

#include "adapters/slam/klt/klt_mode_utils.h"

#include <algorithm>

namespace SmartDrone::Adapters::Slam {

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
        state.m_lkLastSeedFrameId == 0 || frameId >= state.m_lkLastSeedFrameId + LK_GRID_REFILL_INTERVAL_FRAMES;
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

void UpdateLkTracksAfterPoseEstimate(UpdateLkTracksAfterPoseEstimateRequest request)
{
    const bool hardRecoveryRefill =
        request.trackedTracks.size() <
            static_cast<size_t>(LK_HARD_RECOVERY_MIN_TRACKS) ||
        request.inlierCount < LK_HARD_RECOVERY_MIN_INLIERS;
    request.state.m_lkTracks =
        hardRecoveryRefill
            ? std::vector<LkStereoTrack>{}
            : SelectLkTracksGridBalanced(request.trackedTracks,
                                         request.leftRect.size());
    if (hardRecoveryRefill) {
        RefreshLkStereoSeedsIfNeeded(request.state, request.leftRect,
                                     request.rightRect, request.frameId, true);
    }
    RefreshLkStereoSeedsIfNeeded(request.state, request.leftRect,
                                 request.rightRect, request.frameId, false);
    request.state.m_lkTracks = SelectLkTracksGridBalanced(
        request.state.m_lkTracks, request.leftRect.size());
}

void CopyLkTrackFeaturesToOutput(const std::vector<LkStereoTrack> &tracks,
                                 Core::Ports::SlamOutput &out)
{
    Core::Ports::CopyStereoTracksToOutput(tracks, out);
}

} // namespace SmartDrone::Adapters::Slam
