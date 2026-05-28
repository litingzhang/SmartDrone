#include "adapters/slam/dpvo/dpvo_tensorrt_engine_graph_state.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "adapters/slam/dpvo/dpvo_runtime_options.h"

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

void DpvoGraphState::Reset(int patchesPerFrame, int optimizationWindow)
{
    m_patchesPerFrame = std::clamp(patchesPerFrame, 16, 256);
    m_optimizationWindow = std::clamp(optimizationWindow, 4, 32);
    m_patchLifetime = std::max(8, m_optimizationWindow + 4);
    m_removalWindow = std::max(12, m_optimizationWindow + 9);
    m_frames.clear();
    m_edges.clear();
    m_initialized = false;
    m_counter = 0;
    m_keyframeRemovals = 0;
    const DpvoGraphRuntimeOptions options = LoadDpvoGraphRuntimeOptions();
    m_persistentEdges = options.persistentEdges;
    m_keyframeRemovalEnabled = options.keyframeRemovalEnabled;
    m_capRebuiltEdges = options.capRebuiltEdges;
    m_maxActiveEdges = options.maxActiveEdges;
}

void DpvoGraphState::PushFrame(uint64_t frameId, int64_t timestampNs, const cv::Mat &gray,
               const Sophus::SE3f &initialPose,
               const DpvoPatchifierRun &patchRun)
{
    CommitFrame(CreateFrameState(frameId, timestampNs, gray, initialPose,
                                 patchRun));
}

DpvoFrameState DpvoGraphState::CreateFrameState(
    uint64_t frameId, int64_t timestampNs, const cv::Mat &gray,
    const Sophus::SE3f &initialPose, const DpvoPatchifierRun &patchRun) const
{
    DpvoFrameState frame{};
    frame.frameId = frameId;
    frame.timestampNs = timestampNs;
    frame.tcw = initialPose;
    frame.imageWidth = gray.cols;
    frame.imageHeight = gray.rows;
    LoadFrameFmap(&frame, patchRun);
    LoadFrameImapMetadata(&frame, patchRun);
    SelectFramePatches(&frame, gray);
    SampleFrameImap(&frame, patchRun);
    SampleFrameGmap(&frame);
    return frame;
}

void DpvoGraphState::LoadFrameFmap(DpvoFrameState *frame,
                                   const DpvoPatchifierRun &patchRun)
{
    if (frame == nullptr) {
        return;
    }
    if (patchRun.ok && patchRun.fmapHost != nullptr &&
        patchRun.fmapDims.nbDims == 4) {
        frame->fmapChannels = static_cast<int>(patchRun.fmapDims.d[1]);
        frame->fmapHeight = static_cast<int>(patchRun.fmapDims.d[2]);
        frame->fmapWidth = static_cast<int>(patchRun.fmapDims.d[3]);
        const size_t expected = static_cast<size_t>(frame->fmapChannels) *
                                static_cast<size_t>(frame->fmapHeight) *
                                static_cast<size_t>(frame->fmapWidth);
        if (expected > 0 && expected <= patchRun.fmapValueCount) {
            frame->fmap.assign(patchRun.fmapHost,
                               patchRun.fmapHost + expected);
            frame->fmapLevel4 = BuildPooledFmap(
                frame->fmap, frame->fmapChannels, frame->fmapHeight,
                frame->fmapWidth, 4);
        }
    }
}

void DpvoGraphState::LoadFrameImapMetadata(
    DpvoFrameState *frame, const DpvoPatchifierRun &patchRun)
{
    if (frame == nullptr) {
        return;
    }
    if (patchRun.ok && patchRun.imapHost != nullptr &&
        patchRun.imapDims.nbDims == 4) {
        frame->imapChannels = static_cast<int>(patchRun.imapDims.d[1]);
        frame->imapHeight = static_cast<int>(patchRun.imapDims.d[2]);
        frame->imapWidth = static_cast<int>(patchRun.imapDims.d[3]);
    }
}

void DpvoGraphState::SelectFramePatches(DpvoFrameState *frame,
                                        const cv::Mat &gray) const
{
    if (frame == nullptr) {
        return;
    }
    frame->patches = SelectPatches(*frame, gray);
    if (frame->patches.size() < static_cast<size_t>(m_patchesPerFrame)) {
        frame->patches.resize(static_cast<size_t>(m_patchesPerFrame));
    }
}

void DpvoGraphState::SampleFrameImap(
    DpvoFrameState *frame, const DpvoPatchifierRun &patchRun) const
{
    if (frame == nullptr) {
        return;
    }
    if (patchRun.ok && patchRun.imapHost != nullptr &&
        frame->imapChannels == DIM && frame->imapHeight > 0 &&
        frame->imapWidth > 0 &&
        patchRun.imapValueCount >= static_cast<size_t>(frame->imapChannels) *
                                       static_cast<size_t>(frame->imapHeight) *
                                       static_cast<size_t>(frame->imapWidth)) {
        frame->patchImap.assign(static_cast<size_t>(m_patchesPerFrame) * DIM,
                                0.0f);
        const size_t imapValueCount = static_cast<size_t>(frame->imapChannels) *
                                      static_cast<size_t>(frame->imapHeight) *
                                      static_cast<size_t>(frame->imapWidth);
        const DpvoFeatureMapView imapView{patchRun.imapHost, frame->imapChannels,
                                          frame->imapHeight, frame->imapWidth,
                                          imapValueCount};
        for (int p = 0; p < m_patchesPerFrame; ++p) {
            SampleFeatureVector(imapView,
                                frame->patches[static_cast<size_t>(p)].x,
                                frame->patches[static_cast<size_t>(p)].y,
                                &frame->patchImap[static_cast<size_t>(p) *
                                                  DIM]);
        }
    }
}

void DpvoGraphState::SampleFrameGmap(DpvoFrameState *frame) const
{
    if (frame == nullptr) {
        return;
    }
    if (!frame->fmap.empty() && frame->fmapChannels == FMAP_CHANNELS) {
        frame->patchGmap.assign(static_cast<size_t>(m_patchesPerFrame) *
                                    FMAP_CHANNELS * PATCH_AREA,
                                0.0f);
        const DpvoFeatureMapView fmapView{frame->fmap.data(),
                                          frame->fmapChannels,
                                          frame->fmapHeight, frame->fmapWidth,
                                          frame->fmap.size()};
        for (int p = 0; p < m_patchesPerFrame; ++p) {
            SampleFeaturePatch3(fmapView,
                                frame->patches[static_cast<size_t>(p)].x,
                                frame->patches[static_cast<size_t>(p)].y,
                                &frame->patchGmap[static_cast<size_t>(p) *
                                                  FMAP_CHANNELS * PATCH_AREA]);
        }
    }
}

void DpvoGraphState::CommitFrame(DpvoFrameState frame)
{
    m_frames.push_back(std::move(frame));
    ++m_counter;
    if (m_frames.size() >= INITIALIZATION_FRAMES) {
        m_initialized = true;
    }
    if (!m_persistentEdges) {
        m_edges.clear();
    }
    AppendEdgesForNewest();
    PruneOldEdges();
    if (m_persistentEdges || m_capRebuiltEdges) {
        CapActiveEdges();
    }
    PruneFrames();
}

bool DpvoGraphState::Initialized() const
{
    return m_initialized;
}
int DpvoGraphState::FrameCount() const
{
    return static_cast<int>(m_frames.size());
}
int DpvoGraphState::EdgeCount() const
{
    return static_cast<int>(m_edges.size());
}
int DpvoGraphState::PatchCount() const
{
    return FrameCount() * m_patchesPerFrame;
}
int DpvoGraphState::PatchesPerFrame() const
{
    return m_patchesPerFrame;
}
int DpvoGraphState::OptimizationWindow() const
{
    return m_optimizationWindow;
}
int DpvoGraphState::KeyframeRemovals() const
{
    return m_keyframeRemovals;
}
const std::vector<DpvoFrameState> &DpvoGraphState::Frames() const
{
    return m_frames;
}
std::vector<DpvoFrameState> &DpvoGraphState::MutableFrames()
{
    return m_frames;
}
const std::vector<DpvoEdgeState> &DpvoGraphState::Edges() const
{
    return m_edges;
}
int DpvoGraphState::LastStereoDepthUpdates() const
{
    return m_lastStereoDepthUpdates;
}
void DpvoGraphState::ApplyStereoDepthFromRightFmap(const DpvoPatchifierRun &rightRun,
                                   float fx, float baseline)
{
    m_lastStereoDepthUpdates = 0;
    if (m_frames.empty() || !rightRun.ok || rightRun.fmapHost == nullptr ||
        rightRun.fmapDims.nbDims != 4 || !(fx > 0.0f) || !(baseline > 0.0f)) {
        return;
    }
    DpvoFrameState &frame = m_frames.back();
    DpvoFeatureMapView rightMap{};
    if (!BuildRightFmapView(rightRun, frame, &rightMap)) {
        return;
    }
    const DpvoStereoDepthOptions stereoOptions =
        LoadDpvoStereoDepthOptions(rightMap.width);
    const StereoDepthRequest request{
        frame, rightMap, fx, baseline, stereoOptions.maxDisparity,
        stereoOptions.minScore, stereoOptions.minMargin};
    int updated = 0;
    for (int p = 0; p < static_cast<int>(frame.patches.size()); ++p) {
        if (UpdateStereoPatchDepth(request, p)) {
            ++updated;
        }
    }
    if (updated == 0) {
        return;
    }
    m_lastStereoDepthUpdates = updated;
}

bool DpvoGraphState::BuildRightFmapView(const DpvoPatchifierRun &rightRun,
                                        const DpvoFrameState &frame,
                                        DpvoFeatureMapView *rightMap)
{
    if (rightMap == nullptr || rightRun.fmapDims.nbDims != 4) {
        return false;
    }
    const int rightChannels = static_cast<int>(rightRun.fmapDims.d[1]);
    const int rightHeight = static_cast<int>(rightRun.fmapDims.d[2]);
    const int rightWidth = static_cast<int>(rightRun.fmapDims.d[3]);
    const size_t expected = static_cast<size_t>(rightChannels) *
                            static_cast<size_t>(rightHeight) *
                            static_cast<size_t>(rightWidth);
    if (rightChannels != FMAP_CHANNELS || rightHeight != frame.fmapHeight ||
        rightWidth != frame.fmapWidth || rightRun.fmapValueCount < expected ||
        frame.patchGmap.size() < static_cast<size_t>(frame.patches.size()) *
                                     FMAP_CHANNELS * PATCH_AREA) {
        return false;
    }
    *rightMap = {rightRun.fmapHost, rightChannels, rightHeight, rightWidth,
                 expected};
    return true;
}

DpvoGraphState::StereoDepthSearchResult DpvoGraphState::SearchStereoDisparity(
    const StereoDepthRequest &request, int patchIndex)
{
    StereoDepthSearchResult result{};
    const DpvoPatchState &patch =
        request.frame.patches[static_cast<size_t>(patchIndex)];
    result.scores.assign(static_cast<size_t>(request.maxDisp + 1),
                         -std::numeric_limits<float>::infinity());
    result.maxPatchDisp = std::min(
        request.maxDisp, std::max(1, static_cast<int>(std::floor(patch.x)) - 1));
    for (int disp = 1; disp <= result.maxPatchDisp; ++disp) {
        const float score = ComputeStereoPatchScore(request, patchIndex, disp);
        result.scores[static_cast<size_t>(disp)] = score;
        if (score > result.bestScore) {
            result.secondScore = result.bestScore;
            result.bestScore = score;
            result.bestDisp = disp;
        } else if (score > result.secondScore) {
            result.secondScore = score;
        }
    }
    return result;
}

float DpvoGraphState::ComputeStereoPatchScore(
    const StereoDepthRequest &request, int patchIndex, int disparity)
{
    const DpvoPatchState &patch =
        request.frame.patches[static_cast<size_t>(patchIndex)];
    double ab = 0.0;
    double aa = 0.0;
    double bb = 0.0;
    const float x = patch.x - static_cast<float>(disparity);
    const float y = patch.y;
    const size_t gmapOffset =
        static_cast<size_t>(patchIndex) * FMAP_CHANNELS * PATCH_AREA;
    for (int c = 0; c < FMAP_CHANNELS; ++c) {
        for (int py = 0; py < PATCH_SIZE; ++py) {
            for (int px = 0; px < PATCH_SIZE; ++px) {
                const size_t k = static_cast<size_t>(py * PATCH_SIZE + px);
                const float a = request.frame.patchGmap
                    [gmapOffset + static_cast<size_t>(c) * PATCH_AREA + k];
                const float b = SampleFeatureBilinear(
                    request.rightMap, c,
                    x + static_cast<float>(px - PATCH_RADIUS),
                    y + static_cast<float>(py - PATCH_RADIUS));
                ab += static_cast<double>(a) * static_cast<double>(b);
                aa += static_cast<double>(a) * static_cast<double>(a);
                bb += static_cast<double>(b) * static_cast<double>(b);
            }
        }
    }
    return aa > 1e-9 && bb > 1e-9
               ? static_cast<float>(ab / std::sqrt(aa * bb))
               : -1.0f;
}

float DpvoGraphState::RefineStereoDisparity(
    const StereoDepthSearchResult &searchResult)
{
    float refinedDisp = static_cast<float>(searchResult.bestDisp);
    if (searchResult.bestDisp <= 1 ||
        searchResult.bestDisp >= searchResult.maxPatchDisp) {
        return refinedDisp;
    }
    const float left =
        searchResult.scores[static_cast<size_t>(searchResult.bestDisp - 1)];
    const float center =
        searchResult.scores[static_cast<size_t>(searchResult.bestDisp)];
    const float right =
        searchResult.scores[static_cast<size_t>(searchResult.bestDisp + 1)];
    const float denom = left - 2.0f * center + right;
    if (std::isfinite(denom) && std::fabs(denom) > 1e-6f) {
        refinedDisp +=
            std::clamp(0.5f * (left - right) / denom, -0.5f, 0.5f);
    }
    return refinedDisp;
}

bool DpvoGraphState::StereoSearchAccepted(
    const StereoDepthSearchResult &searchResult, float minScore,
    float minMargin)
{
    return searchResult.bestDisp > 0 && std::isfinite(searchResult.bestScore) &&
           (!std::isfinite(searchResult.secondScore) ||
            searchResult.bestScore > searchResult.secondScore + minMargin) &&
           searchResult.bestScore > minScore;
}

bool DpvoGraphState::UpdateStereoPatchDepth(
    const StereoDepthRequest &request, int patchIndex)
{
    DpvoPatchState &patch =
        request.frame.patches[static_cast<size_t>(patchIndex)];
    if (patch.x < 2.0f || patch.y < 2.0f ||
        patch.x >= static_cast<float>(request.rightMap.width - 2) ||
        patch.y >= static_cast<float>(request.rightMap.height - 2)) {
        return false;
    }
    const StereoDepthSearchResult searchResult =
        SearchStereoDisparity(request, patchIndex);
    if (!StereoSearchAccepted(searchResult, request.minScore,
                              request.minMargin)) {
        return false;
    }
    const float refinedDisp = RefineStereoDisparity(searchResult);
    patch.invDepth =
        std::clamp(refinedDisp / (request.fx * request.baseline), 1.0e-3f,
                   10.0f);
    patch.stereoPriorInvDepth = patch.invDepth;
    patch.hasStereoPrior = true;
    return true;
}
bool DpvoGraphState::FeatureMapsReady() const
{
    return !m_frames.empty() &&
           std::all_of(m_frames.begin(), m_frames.end(),
                       [](const DpvoFrameState &frame) {
                           return frame.patchImap.size() >=
                                      static_cast<size_t>(frame.patches.size()) *
                                          DIM &&
                                  frame.patchGmap.size() >=
                                      static_cast<size_t>(frame.patches.size()) *
                                          FMAP_CHANNELS * PATCH_AREA &&
                                  !frame.fmap.empty() &&
                                  !frame.fmapLevel4.empty();
                       });
}
const DpvoFrameState *DpvoGraphState::PreviousFrame() const
{
    return m_frames.size() >= 2 ? &m_frames[m_frames.size() - 2] : nullptr;
}
const DpvoFrameState *DpvoGraphState::NewestFrame() const
{
    return m_frames.empty() ? nullptr : &m_frames.back();
}
bool DpvoGraphState::MaybeRemoveKeyframe(const DpvoIntrinsics &intrinsics)
{
    if (!m_keyframeRemovalEnabled || !m_initialized ||
        FrameCount() <= KEYFRAME_INDEX + 1) {
        PruneOldEdges();
        CapActiveEdges();
        return false;
    }
    const int n = FrameCount();
    const int i = n - KEYFRAME_INDEX - 1;
    const int j = n - KEYFRAME_INDEX + 1;
    if (i < 0 || j < 0 || i >= n || j >= n) {
        return false;
    }
    const float m =
        MotionMagnitude(i, j, intrinsics) + MotionMagnitude(j, i, intrinsics);
    if (!(std::isfinite(m) && 0.5f * m < KEYFRAME_THRESHOLD)) {
        PruneOldEdges();
        CapActiveEdges();
        return false;
    }
    const int k = n - KEYFRAME_INDEX;
    RemoveFrameAt(k);
    ++m_keyframeRemovals;
    PruneOldEdges();
    CapActiveEdges();
    return true;
}

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
