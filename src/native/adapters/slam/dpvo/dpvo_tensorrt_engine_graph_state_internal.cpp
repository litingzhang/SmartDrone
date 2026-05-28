#include "adapters/slam/dpvo/dpvo_tensorrt_engine_graph_state.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

float DpvoGraphState::FeatureAt(const DpvoFeatureMapView &featureMap, int c,
                                int y, int x)
{
    if (featureMap.data == nullptr || c < 0 || c >= featureMap.channels ||
        y < 0 || y >= featureMap.height || x < 0 || x >= featureMap.width) {
        return 0.0f;
    }
    const size_t idx = (static_cast<size_t>(c) *
                            static_cast<size_t>(featureMap.height) +
                        static_cast<size_t>(y)) *
                           static_cast<size_t>(featureMap.width) +
                       static_cast<size_t>(x);
    if (idx >= featureMap.valueCount) {
        return 0.0f;
    }
    return featureMap.data[idx];
}

float DpvoGraphState::SampleFeatureBilinear(
    const DpvoFeatureMapView &featureMap, int c, float x, float y)
{
    if (featureMap.data == nullptr || featureMap.channels <= 0 ||
        featureMap.height <= 0 || featureMap.width <= 0) {
        return 0.0f;
    }
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;
    const float dx = x - static_cast<float>(x0);
    const float dy = y - static_cast<float>(y0);
    const float v00 = FeatureAt(featureMap, c, y0, x0);
    const float v01 = FeatureAt(featureMap, c, y0, x1);
    const float v10 = FeatureAt(featureMap, c, y1, x0);
    const float v11 = FeatureAt(featureMap, c, y1, x1);
    return (1.0f - dy) * ((1.0f - dx) * v00 + dx * v01) +
           dy * ((1.0f - dx) * v10 + dx * v11);
}

void DpvoGraphState::SampleFeatureVector(
    const DpvoFeatureMapView &featureMap, float x, float y, float *out)
{
    if (out == nullptr) {
        return;
    }
    for (int c = 0; c < featureMap.channels; ++c) {
        out[c] = SampleFeatureBilinear(featureMap, c, x, y);
    }
}

void DpvoGraphState::SampleFeaturePatch3(
    const DpvoFeatureMapView &featureMap, float x, float y, float *out)
{
    if (out == nullptr) {
        return;
    }
    for (int c = 0; c < featureMap.channels; ++c) {
        for (int py = 0; py < PATCH_SIZE; ++py) {
            for (int px = 0; px < PATCH_SIZE; ++px) {
                const float sx = x + static_cast<float>(px - PATCH_RADIUS);
                const float sy = y + static_cast<float>(py - PATCH_RADIUS);
                const size_t idx = (static_cast<size_t>(c) * PATCH_AREA) +
                                   static_cast<size_t>(py * PATCH_SIZE + px);
                out[idx] =
                    SampleFeatureBilinear(featureMap, c, sx, sy);
            }
        }
    }
}

std::vector<float> DpvoGraphState::BuildPooledFmap(const std::vector<float> &src,
                                          int channels, int height, int width,
                                          int level)
{
    if (src.empty() || channels <= 0 || height <= 0 || width <= 0 ||
        level <= 1) {
        return src;
    }
    const int outHeight = height / level;
    const int outWidth = width / level;
    std::vector<float> dst(static_cast<size_t>(channels) *
                               static_cast<size_t>(outHeight) *
                               static_cast<size_t>(outWidth),
                           0.0f);
    if (outHeight <= 0 || outWidth <= 0) {
        return {};
    }
    const float denom = 1.0f / static_cast<float>(level * level);
    for (int c = 0; c < channels; ++c) {
        for (int y = 0; y < outHeight; ++y) {
            for (int x = 0; x < outWidth; ++x) {
                float sum = 0.0f;
                for (int dy = 0; dy < level; ++dy) {
                    for (int dx = 0; dx < level; ++dx) {
                        sum += src[(static_cast<size_t>(c) * static_cast<size_t>(height) +
                                    static_cast<size_t>(y * level + dy)) *
                                       static_cast<size_t>(width) +
                                   static_cast<size_t>(x * level + dx)];
                    }
                }
                dst[(static_cast<size_t>(c) * static_cast<size_t>(outHeight) +
                     static_cast<size_t>(y)) *
                        static_cast<size_t>(outWidth) +
                    static_cast<size_t>(x)] = sum * denom;
            }
        }
    }
    return dst;
}

std::vector<DpvoPatchState> DpvoGraphState::SelectPatches(const DpvoFrameState &frame,
                                          const cv::Mat &gray) const
{
    std::vector<DpvoPatchState> patches;
    patches.reserve(static_cast<size_t>(m_patchesPerFrame));
    const int width =
        frame.fmapWidth > 2 ? frame.fmapWidth : std::max(3, gray.cols / 4);
    const int height =
        frame.fmapHeight > 2 ? frame.fmapHeight : std::max(3, gray.rows / 4);
    std::mt19937 rng(
        static_cast<uint32_t>(0x9e3779b9U ^ (m_counter * 0x85ebca6bU)));
    std::uniform_int_distribution<int> xDist(1, std::max(1, width - 2));
    std::uniform_int_distribution<int> yDist(1, std::max(1, height - 2));
    std::uniform_real_distribution<float> depthDist(0.2f, 1.0f);
    for (int i = 0; i < m_patchesPerFrame; ++i) {
        float invDepth = depthDist(rng);
        if (m_initialized && !m_frames.empty()) {
            invDepth = MedianRecentDepth();
        }
        patches.push_back({static_cast<float>(xDist(rng)),
                           static_cast<float>(yDist(rng)), invDepth, invDepth,
                           false});
    }
    return patches;
}

float DpvoGraphState::MedianRecentDepth() const
{
    std::vector<float> depths;
    const int start = std::max(0, FrameCount() - 3);
    for (int i = start; i < FrameCount(); ++i) {
        for (const DpvoPatchState &patch :
             m_frames[static_cast<size_t>(i)].patches) {
            if (std::isfinite(patch.invDepth) && patch.invDepth > 1e-3f &&
                patch.invDepth < 10.0f) {
                depths.push_back(patch.invDepth);
            }
        }
    }
    if (depths.empty()) {
        return 1.0f;
    }
    const size_t mid = depths.size() / 2;
    std::nth_element(depths.begin(),
                     depths.begin() + static_cast<std::ptrdiff_t>(mid),
                     depths.end());
    return std::clamp(depths[mid], 1e-3f, 10.0f);
}

std::array<float, 2> DpvoGraphState::ProjectPatchCenter(
    const DpvoPatchProjectionRequest &request)
{
    const DpvoPatchState &patch = request.patch;
    const DpvoIntrinsics &intrinsics = request.intrinsics;
    if (request.valid != nullptr) {
        *request.valid = false;
    }
    if (!(intrinsics.fx > 0.0f) || !(intrinsics.fy > 0.0f)) {
        return {patch.x, patch.y};
    }
    const Sophus::SE3f Tji =
        request.target.tcw * request.source.tcw.inverse();
    const Eigen::Matrix3f R =
        request.useOverrideR ? request.overrideR : Tji.so3().matrix();
    const Eigen::Vector3f t = Tji.translation();
    const Eigen::Vector3f Xi((patch.x - intrinsics.cx) / intrinsics.fx,
                             (patch.y - intrinsics.cy) / intrinsics.fy, 1.0f);
    const Eigen::Vector3f Xj = R * Xi + patch.invDepth * t;
    if (!(Xj.z() > 0.2f) || !Xj.allFinite()) {
        return {patch.x, patch.y};
    }
    if (request.valid != nullptr) {
        *request.valid = true;
    }
    return {intrinsics.fx * (Xj.x() / Xj.z()) + intrinsics.cx,
            intrinsics.fy * (Xj.y() / Xj.z()) + intrinsics.cy};
}

float DpvoGraphState::MotionMagnitude(int sourceFrame, int targetFrame,
                      const DpvoIntrinsics &intrinsics) const
{
    if (sourceFrame < 0 || targetFrame < 0 || sourceFrame >= FrameCount() ||
        targetFrame >= FrameCount()) {
        return std::numeric_limits<float>::infinity();
    }
    const DpvoFrameState &source = m_frames[static_cast<size_t>(sourceFrame)];
    const DpvoFrameState &target = m_frames[static_cast<size_t>(targetFrame)];
    if (!(intrinsics.fx > 0.0f) || !(intrinsics.fy > 0.0f)) {
        return std::numeric_limits<float>::infinity();
    }
    double sum = 0.0;
    int count = 0;
    const Eigen::Matrix3f identityR = Eigen::Matrix3f::Identity();
    for (const DpvoPatchState &patch : source.patches) {
        bool validFull = false;
        bool validTonly = false;
        const std::array<float, 2> full = ProjectPatchCenter(
            {source, target, patch, identityR, false, intrinsics, &validFull});
        const std::array<float, 2> tonly = ProjectPatchCenter(
            {source, target, patch, identityR, true, intrinsics, &validTonly});
        if (!validFull || !validTonly) {
            continue;
        }
        const float dxFull = full[0] - patch.x;
        const float dyFull = full[1] - patch.y;
        const float dxT = tonly[0] - patch.x;
        const float dyT = tonly[1] - patch.y;
        sum += 0.5 * std::sqrt(
                         static_cast<double>(dxFull * dxFull + dyFull * dyFull)) +
               0.5 * std::sqrt(static_cast<double>(dxT * dxT + dyT * dyT));
        ++count;
    }
    return count > 0 ? static_cast<float>(sum / static_cast<double>(count))
                     : std::numeric_limits<float>::infinity();
}

void DpvoGraphState::AppendEdgesForNewest()
{
    const int n = FrameCount();
    if (n <= 1) {
        return;
    }
    const int newest = n - 1;
    const int forwardPatchStart =
        m_patchesPerFrame * std::max(newest - m_patchLifetime, 0);
    const int forwardPatchEnd = m_patchesPerFrame * std::max(newest, 0);
    for (int k = forwardPatchStart; k < forwardPatchEnd; ++k) {
        m_edges.push_back({k, k / m_patchesPerFrame, newest});
    }
    const int backPatchStart = m_patchesPerFrame * newest;
    const int backPatchEnd = m_patchesPerFrame * n;
    const int backTargetStart = std::max(n - m_patchLifetime, 0);
    for (int k = backPatchStart; k < backPatchEnd; ++k) {
        for (int j = backTargetStart; j < n; ++j) {
            m_edges.push_back({k, newest, j});
        }
    }
}

void DpvoGraphState::PruneOldEdges()
{
    const int n = FrameCount();
    const int oldestActivePatchFrame = std::max(n - m_removalWindow, 0);
    m_edges.erase(std::remove_if(m_edges.begin(), m_edges.end(),
                                 [&](const DpvoEdgeState &edge) {
                                     const int patchFrame =
                                         edge.patchGlobal / m_patchesPerFrame;
                                     return edge.sourceFrame < 0 ||
                                            edge.targetFrame < 0 ||
                                            edge.sourceFrame >= n ||
                                            edge.targetFrame >= n ||
                                            patchFrame < oldestActivePatchFrame ||
                                            patchFrame >= n;
                                 }),
                  m_edges.end());
}

void DpvoGraphState::CapActiveEdges()
{
    if ((!m_persistentEdges && !m_capRebuiltEdges) || m_maxActiveEdges <= 0 ||
        static_cast<int>(m_edges.size()) <= m_maxActiveEdges) {
        return;
    }
    const int removeCount = static_cast<int>(m_edges.size()) - m_maxActiveEdges;
    m_edges.erase(m_edges.begin(), m_edges.begin() + removeCount);
}

void DpvoGraphState::RemoveFrameAt(int frameIndex)
{
    if (frameIndex < 0 || frameIndex >= FrameCount()) {
        return;
    }
    m_frames.erase(m_frames.begin() + frameIndex);
    m_edges.erase(std::remove_if(m_edges.begin(), m_edges.end(),
                                 [&](DpvoEdgeState &edge) {
                                     const int patchFrame =
                                         edge.patchGlobal / m_patchesPerFrame;
                                     if (edge.sourceFrame == frameIndex ||
                                         edge.targetFrame == frameIndex ||
                                         patchFrame == frameIndex) {
                                         return true;
                                     }
                                     if (edge.sourceFrame > frameIndex) {
                                         --edge.sourceFrame;
                                     }
                                     if (edge.targetFrame > frameIndex) {
                                         --edge.targetFrame;
                                     }
                                     if (patchFrame > frameIndex) {
                                         edge.patchGlobal -= m_patchesPerFrame;
                                     }
                                     return false;
                                 }),
                  m_edges.end());
}

void DpvoGraphState::PruneFrames()
{
    const int maxFrames =
        std::max(36, m_removalWindow + m_optimizationWindow + 4);
    if (FrameCount() <= maxFrames) {
        return;
    }
    const int drop = FrameCount() - maxFrames;
    m_frames.erase(m_frames.begin(), m_frames.begin() + drop);
    m_edges.erase(std::remove_if(m_edges.begin(), m_edges.end(),
                                 [&](DpvoEdgeState &edge) {
                                     const int patchFrame =
                                         edge.patchGlobal / m_patchesPerFrame;
                                     if (edge.sourceFrame < drop ||
                                         edge.targetFrame < drop ||
                                         patchFrame < drop) {
                                         return true;
                                     }
                                     edge.sourceFrame -= drop;
                                     edge.targetFrame -= drop;
                                     edge.patchGlobal -= drop * m_patchesPerFrame;
                                     return false;
                                 }),
                  m_edges.end());
    PruneOldEdges();
    CapActiveEdges();
}


bool DpvoEdgeKey::operator==(const DpvoEdgeKey &other) const
{
    return sourceFrameId == other.sourceFrameId &&
           targetFrameId == other.targetFrameId && patchLocal == other.patchLocal;
}

size_t DpvoEdgeKeyHash::operator()(const DpvoEdgeKey &key) const
{
    uint64_t x = key.sourceFrameId + 0x9e3779b97f4a7c15ULL;
    x ^= key.targetFrameId + 0x9e3779b97f4a7c15ULL + (x << 6U) + (x >> 2U);
    x ^= static_cast<uint64_t>(key.patchLocal + 0x9e3779b9) + (x << 6U) +
         (x >> 2U);
    x ^= x >> 30U;
    x *= 0xbf58476d1ce4e5b9ULL;
    x ^= x >> 27U;
    x *= 0x94d049bb133111ebULL;
    x ^= x >> 31U;
    return static_cast<size_t>(x);
}

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
