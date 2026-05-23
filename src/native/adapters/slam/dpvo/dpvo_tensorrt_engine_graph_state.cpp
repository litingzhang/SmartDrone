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
    DpvoFrameState frame{};
    frame.frameId = frameId;
    frame.timestampNs = timestampNs;
    frame.Tcw = initialPose;
    frame.imageWidth = gray.cols;
    frame.imageHeight = gray.rows;
    if (patchRun.ok && patchRun.fmapHost != nullptr &&
        patchRun.fmapDims.nbDims == 4) {
        frame.fmapChannels = static_cast<int>(patchRun.fmapDims.d[1]);
        frame.fmapHeight = static_cast<int>(patchRun.fmapDims.d[2]);
        frame.fmapWidth = static_cast<int>(patchRun.fmapDims.d[3]);
        const size_t expected = static_cast<size_t>(frame.fmapChannels) *
                                static_cast<size_t>(frame.fmapHeight) *
                                static_cast<size_t>(frame.fmapWidth);
        if (expected > 0 && expected <= patchRun.fmapValueCount) {
            frame.fmap.assign(patchRun.fmapHost, patchRun.fmapHost + expected);
            frame.fmapLevel4 =
                BuildPooledFmap(frame.fmap, frame.fmapChannels, frame.fmapHeight,
                                frame.fmapWidth, 4);
        }
    }
    if (patchRun.ok && patchRun.imapHost != nullptr &&
        patchRun.imapDims.nbDims == 4) {
        frame.imapChannels = static_cast<int>(patchRun.imapDims.d[1]);
        frame.imapHeight = static_cast<int>(patchRun.imapDims.d[2]);
        frame.imapWidth = static_cast<int>(patchRun.imapDims.d[3]);
    }
    frame.patches = SelectPatches(frame, gray);
    if (frame.patches.size() < static_cast<size_t>(m_patchesPerFrame)) {
        frame.patches.resize(static_cast<size_t>(m_patchesPerFrame));
    }
    if (patchRun.ok && patchRun.imapHost != nullptr &&
        frame.imapChannels == kDim && frame.imapHeight > 0 &&
        frame.imapWidth > 0 &&
        patchRun.imapValueCount >= static_cast<size_t>(frame.imapChannels) *
                                       static_cast<size_t>(frame.imapHeight) *
                                       static_cast<size_t>(frame.imapWidth)) {
        frame.patchImap.assign(static_cast<size_t>(m_patchesPerFrame) * kDim,
                               0.0f);
        for (int p = 0; p < m_patchesPerFrame; ++p) {
            SampleFeatureVector(patchRun.imapHost, frame.imapChannels,
                                frame.imapHeight, frame.imapWidth,
                                frame.patches[static_cast<size_t>(p)].x,
                                frame.patches[static_cast<size_t>(p)].y,
                                &frame.patchImap[static_cast<size_t>(p) * kDim]);
        }
    }
    if (!frame.fmap.empty() && frame.fmapChannels == kFmapChannels) {
        frame.patchGmap.assign(static_cast<size_t>(m_patchesPerFrame) *
                                   kFmapChannels * kPatchArea,
                               0.0f);
        for (int p = 0; p < m_patchesPerFrame; ++p) {
            SampleFeaturePatch3(frame.fmap.data(), frame.fmapChannels,
                                frame.fmapHeight, frame.fmapWidth,
                                frame.patches[static_cast<size_t>(p)].x,
                                frame.patches[static_cast<size_t>(p)].y,
                                &frame.patchGmap[static_cast<size_t>(p) *
                                                 kFmapChannels * kPatchArea]);
        }
    }
    m_frames.push_back(std::move(frame));
    ++m_counter;
    if (m_frames.size() >= kInitializationFrames) {
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
    const int rightChannels = static_cast<int>(rightRun.fmapDims.d[1]);
    const int rightHeight = static_cast<int>(rightRun.fmapDims.d[2]);
    const int rightWidth = static_cast<int>(rightRun.fmapDims.d[3]);
    const size_t expected = static_cast<size_t>(rightChannels) *
                            static_cast<size_t>(rightHeight) *
                            static_cast<size_t>(rightWidth);
    if (rightChannels != kFmapChannels || rightHeight != frame.fmapHeight ||
        rightWidth != frame.fmapWidth || rightRun.fmapValueCount < expected ||
        frame.patchGmap.size() < static_cast<size_t>(frame.patches.size()) *
                                     kFmapChannels * kPatchArea) {
        return;
    }

    int updated = 0;
    const DpvoStereoDepthOptions stereoOptions =
        LoadDpvoStereoDepthOptions(rightWidth);
    const int maxDisp = stereoOptions.maxDisparity;
    const float minScore = stereoOptions.minScore;
    const float minMargin = stereoOptions.minMargin;
    for (int p = 0; p < static_cast<int>(frame.patches.size()); ++p) {
        DpvoPatchState &patch = frame.patches[static_cast<size_t>(p)];
        if (patch.x < 2.0f || patch.y < 2.0f ||
            patch.x >= static_cast<float>(rightWidth - 2) ||
            patch.y >= static_cast<float>(rightHeight - 2)) {
            continue;
        }
        float bestScore = -std::numeric_limits<float>::infinity();
        float secondScore = -std::numeric_limits<float>::infinity();
        int bestDisp = 0;
        std::vector<float> scores(static_cast<size_t>(maxDisp + 1),
                                  -std::numeric_limits<float>::infinity());
        const int maxPatchDisp = std::min(
            maxDisp, std::max(1, static_cast<int>(std::floor(patch.x)) - 1));
        for (int disp = 1; disp <= maxPatchDisp; ++disp) {
            double ab = 0.0;
            double aa = 0.0;
            double bb = 0.0;
            const float x = patch.x - static_cast<float>(disp);
            const float y = patch.y;
            const size_t gmapOffset =
                static_cast<size_t>(p) * kFmapChannels * kPatchArea;
            for (int c = 0; c < kFmapChannels; ++c) {
                for (int py = 0; py < kPatchSize; ++py) {
                    for (int px = 0; px < kPatchSize; ++px) {
                        const size_t k = static_cast<size_t>(py * kPatchSize + px);
                        const float a =
                            frame.patchGmap[gmapOffset +
                                            static_cast<size_t>(c) * kPatchArea + k];
                        const float b = SampleFeatureBilinear(
                            rightRun.fmapHost, rightChannels, rightHeight, rightWidth, c,
                            x + static_cast<float>(px - kPatchRadius),
                            y + static_cast<float>(py - kPatchRadius));
                        ab += static_cast<double>(a) * static_cast<double>(b);
                        aa += static_cast<double>(a) * static_cast<double>(a);
                        bb += static_cast<double>(b) * static_cast<double>(b);
                    }
                }
            }
            const float score = aa > 1e-9 && bb > 1e-9
                                    ? static_cast<float>(ab / std::sqrt(aa * bb))
                                    : -1.0f;
            scores[static_cast<size_t>(disp)] = score;
            if (score > bestScore) {
                secondScore = bestScore;
                bestScore = score;
                bestDisp = disp;
            } else if (score > secondScore) {
                secondScore = score;
            }
        }
        if (bestDisp > 0 && std::isfinite(bestScore) &&
            (!std::isfinite(secondScore) ||
             bestScore > secondScore + minMargin) &&
            bestScore > minScore) {
            float refinedDisp = static_cast<float>(bestDisp);
            if (bestDisp > 1 && bestDisp < maxPatchDisp) {
                const float left = scores[static_cast<size_t>(bestDisp - 1)];
                const float center = scores[static_cast<size_t>(bestDisp)];
                const float right = scores[static_cast<size_t>(bestDisp + 1)];
                const float denom = left - 2.0f * center + right;
                if (std::isfinite(denom) && std::fabs(denom) > 1e-6f) {
                    refinedDisp +=
                        std::clamp(0.5f * (left - right) / denom, -0.5f, 0.5f);
                }
            }
            patch.invDepth =
                std::clamp(refinedDisp / (fx * baseline), 1.0e-3f, 10.0f);
            patch.stereoPriorInvDepth = patch.invDepth;
            patch.hasStereoPrior = true;
            ++updated;
        }
    }
    if (updated == 0) {
        return;
    }
    m_lastStereoDepthUpdates = updated;
}
bool DpvoGraphState::FeatureMapsReady() const
{
    return !m_frames.empty() &&
           std::all_of(m_frames.begin(), m_frames.end(),
                       [](const DpvoFrameState &frame) {
                           return frame.patchImap.size() >=
                                      static_cast<size_t>(frame.patches.size()) *
                                          kDim &&
                                  frame.patchGmap.size() >=
                                      static_cast<size_t>(frame.patches.size()) *
                                          kFmapChannels * kPatchArea &&
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
        FrameCount() <= kKeyframeIndex + 1) {
        PruneOldEdges();
        CapActiveEdges();
        return false;
    }
    const int n = FrameCount();
    const int i = n - kKeyframeIndex - 1;
    const int j = n - kKeyframeIndex + 1;
    if (i < 0 || j < 0 || i >= n || j >= n) {
        return false;
    }
    const float m =
        MotionMagnitude(i, j, intrinsics) + MotionMagnitude(j, i, intrinsics);
    if (!(std::isfinite(m) && 0.5f * m < kKeyframeThreshold)) {
        PruneOldEdges();
        CapActiveEdges();
        return false;
    }
    const int k = n - kKeyframeIndex;
    RemoveFrameAt(k);
    ++m_keyframeRemovals;
    PruneOldEdges();
    CapActiveEdges();
    return true;
}

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
