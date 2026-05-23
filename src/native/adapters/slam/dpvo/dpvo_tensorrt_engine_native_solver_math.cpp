#include "adapters/slam/dpvo/dpvo_tensorrt_engine_native_solver.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "adapters/slam/engine/slam_env.h"

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

float DpvoNativeSolver::FeatureAt(const std::vector<float> &data, int channels,
                       int height, int width, int c, int y, int x)
{
    if (data.empty() || c < 0 || c >= channels || y < 0 || y >= height ||
        x < 0 || x >= width) {
        return 0.0f;
    }
    const size_t idx = (static_cast<size_t>(c) * static_cast<size_t>(height) +
                        static_cast<size_t>(y)) *
                           static_cast<size_t>(width) +
                       static_cast<size_t>(x);
    return idx < data.size() ? data[idx] : 0.0f;
}

float DpvoNativeSolver::SampleFeatureBilinear(const std::vector<float> &data,
                                   int channels, int height, int width, int c,
                                   float x, float y)
{
    if (data.empty() || channels <= 0 || height <= 0 || width <= 0) {
        return 0.0f;
    }
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;
    const float dx = x - static_cast<float>(x0);
    const float dy = y - static_cast<float>(y0);
    const float v00 = FeatureAt(data, channels, height, width, c, y0, x0);
    const float v01 = FeatureAt(data, channels, height, width, c, y0, x1);
    const float v10 = FeatureAt(data, channels, height, width, c, y1, x0);
    const float v11 = FeatureAt(data, channels, height, width, c, y1, x1);
    return (1.0f - dy) * ((1.0f - dx) * v00 + dx * v01) +
           dy * ((1.0f - dx) * v10 + dx * v11);
}

void DpvoNativeSolver::PredictNewestPose(std::vector<DpvoFrameState> &frames)
{
    if (frames.size() <= 1U) {
        frames.back().Tcw = Sophus::SE3f();
        return;
    }
    if (frames.size() == 2U) {
        frames.back().Tcw = frames[frames.size() - 2U].Tcw;
        return;
    }
    const Sophus::SE3f &p1 = frames[frames.size() - 2U].Tcw;
    const Sophus::SE3f &p2 = frames[frames.size() - 3U].Tcw;
    const Sophus::SE3f delta = p1 * p2.inverse();
    Eigen::Matrix<float, 6, 1> xi = delta.log();
    xi *= 0.5f;
    frames.back().Tcw = Sophus::SE3f::exp(xi) * p1;
}

bool DpvoNativeSolver::AcceptPoseStep(const Sophus::SE3f &reference,
                           const Sophus::SE3f &candidate)
{
    if (!EnvFlagEnabled("SMART_DRONE_DPVO_ACCEPT_GUARD", true)) {
        return true;
    }
    const Sophus::SE3f delta = candidate * reference.inverse();
    const Eigen::Matrix<float, 6, 1> xi = delta.log();
    const float maxTrans = std::max(
        0.0f, EnvFloatValue("SMART_DRONE_DPVO_ACCEPT_MAX_TRANS", 0.08f));
    const float maxRot =
        std::max(0.0f, EnvFloatValue("SMART_DRONE_DPVO_ACCEPT_MAX_ROT", 0.16f));
    return (maxTrans <= 0.0f || xi.template head<3>().norm() <= maxTrans) &&
           (maxRot <= 0.0f || xi.template tail<3>().norm() <= maxRot) &&
           std::isfinite(xi.norm());
}

void DpvoNativeSolver::BuildTemporalNeighbors(const std::vector<DpvoEdgeState> &edges,
                                   std::vector<int> *prevEdge,
                                   std::vector<int> *nextEdge)
{
    if (prevEdge == nullptr || nextEdge == nullptr) {
        return;
    }
    prevEdge->assign(edges.size(), -1);
    nextEdge->assign(edges.size(), -1);
    std::unordered_map<int, std::vector<int>> byPatch;
    byPatch.reserve(edges.size());
    for (int e = 0; e < static_cast<int>(edges.size()); ++e) {
        byPatch[edges[static_cast<size_t>(e)].patchGlobal].push_back(e);
    }
    for (auto &entry : byPatch) {
        std::vector<int> &idx = entry.second;
        std::stable_sort(idx.begin(), idx.end(), [&](int a, int b) {
            return edges[static_cast<size_t>(a)].targetFrame <
                   edges[static_cast<size_t>(b)].targetFrame;
        });
        for (int i = 0; i < static_cast<int>(idx.size()); ++i) {
            (*prevEdge)[static_cast<size_t>(idx[static_cast<size_t>(i)])] =
                i > 0 ? idx[static_cast<size_t>(i - 1)] : -1;
            (*nextEdge)[static_cast<size_t>(idx[static_cast<size_t>(i)])] =
                i + 1 < static_cast<int>(idx.size())
                    ? idx[static_cast<size_t>(i + 1)]
                    : -1;
        }
    }
}

void DpvoNativeSolver::ReprojectPatch(const std::vector<DpvoFrameState> &frames,
                           const DpvoEdgeState &edge, int patchesPerFrame,
                           const DpvoIntrinsics &intrinsics,
                           std::array<float, DpvoNativeSolver::kPatchArea * 2> &coords)
{
    coords.fill(0.0f);
    if (edge.sourceFrame < 0 || edge.targetFrame < 0 ||
        static_cast<size_t>(edge.sourceFrame) >= frames.size() ||
        static_cast<size_t>(edge.targetFrame) >= frames.size()) {
        return;
    }
    const DpvoFrameState &source =
        frames[static_cast<size_t>(edge.sourceFrame)];
    const DpvoFrameState &target =
        frames[static_cast<size_t>(edge.targetFrame)];
    const int patchLocal = edge.patchGlobal % patchesPerFrame;
    if (patchLocal < 0 ||
        static_cast<size_t>(patchLocal) >= source.patches.size() ||
        !(intrinsics.fx > 0.0f) || !(intrinsics.fy > 0.0f)) {
        return;
    }
    const DpvoPatchState &patch =
        source.patches[static_cast<size_t>(patchLocal)];
    const Sophus::SE3f Tji = target.Tcw * source.Tcw.inverse();
    const Eigen::Matrix3f R = Tji.so3().matrix();
    const Eigen::Vector3f t = Tji.translation();
    for (int py = 0; py < kPatchSize; ++py) {
        for (int px = 0; px < kPatchSize; ++px) {
            const float x = patch.x + static_cast<float>(px - 1);
            const float y = patch.y + static_cast<float>(py - 1);
            const Eigen::Vector3f Xi((x - intrinsics.cx) / intrinsics.fx,
                                     (y - intrinsics.cy) / intrinsics.fy, 1.0f);
            Eigen::Vector3f Xj = R * Xi + patch.invDepth * t;
            const float z = std::fabs(Xj.z()) > 1e-4f ? Xj.z() : 1e-4f;
            const size_t idx = static_cast<size_t>(py * kPatchSize + px) * 2U;
            coords[idx] = intrinsics.fx * (Xj.x() / z) + intrinsics.cx;
            coords[idx + 1U] = intrinsics.fy * (Xj.y() / z) + intrinsics.cy;
        }
    }
}

void
DpvoNativeSolver::ComputeCorrelation(const std::vector<DpvoFrameState> &frames,
                   const DpvoEdgeState &edge, int patchesPerFrame,
                   const std::array<float, DpvoNativeSolver::kPatchArea * 2> &coords,
                   float *outCorr)
{
    if (outCorr == nullptr) {
        return;
    }
    std::fill(outCorr, outCorr + kCorrDim, 0.0f);
    if (edge.sourceFrame < 0 || edge.targetFrame < 0 ||
        static_cast<size_t>(edge.sourceFrame) >= frames.size() ||
        static_cast<size_t>(edge.targetFrame) >= frames.size()) {
        return;
    }
    const DpvoFrameState &source =
        frames[static_cast<size_t>(edge.sourceFrame)];
    const DpvoFrameState &target =
        frames[static_cast<size_t>(edge.targetFrame)];
    const int patchLocal = edge.patchGlobal % patchesPerFrame;
    const size_t gmapOffset =
        static_cast<size_t>(patchLocal) * kFmapChannels * kPatchArea;
    if (source.patchGmap.size() < gmapOffset + kFmapChannels * kPatchArea ||
        target.fmap.empty() || target.fmapLevel4.empty()) {
        return;
    }

    for (int ox = 0; ox < kCorrSide; ++ox) {
        for (int oy = 0; oy < kCorrSide; ++oy) {
            const int dx = ox - kCorrRadius;
            const int dy = oy - kCorrRadius;
            for (int py = 0; py < kPatchSize; ++py) {
                for (int px = 0; px < kPatchSize; ++px) {
                    const size_t coordIdx =
                        static_cast<size_t>(py * kPatchSize + px) * 2U;
                    for (int levelIndex = 0; levelIndex < 2; ++levelIndex) {
                        const int level = levelIndex == 0 ? 1 : 4;
                        const std::vector<float> &targetMap =
                            levelIndex == 0 ? target.fmap : target.fmapLevel4;
                        const int targetHeight =
                            levelIndex == 0 ? target.fmapHeight : target.fmapHeight / 4;
                        const int targetWidth =
                            levelIndex == 0 ? target.fmapWidth : target.fmapWidth / 4;
                        float dot = 0.0f;
                        const float sx = coords[coordIdx] / static_cast<float>(level) +
                                         static_cast<float>(dx);
                        const float sy =
                            coords[coordIdx + 1U] / static_cast<float>(level) +
                            static_cast<float>(dy);
                        for (int c = 0; c < kFmapChannels; ++c) {
                            const size_t gidx = gmapOffset +
                                                static_cast<size_t>(c) * kPatchArea +
                                                static_cast<size_t>(py * kPatchSize + px);
                            const float a = source.patchGmap[gidx];
                            const float b =
                                SampleFeatureBilinear(targetMap, kFmapChannels,
                                                      targetHeight, targetWidth, c, sx, sy);
                            dot += a * b;
                        }
                        const size_t outIdx = (((static_cast<size_t>(ox) * kCorrSide +
                                                 static_cast<size_t>(oy)) *
                                                    kPatchSize +
                                                static_cast<size_t>(py)) *
                                                   kPatchSize +
                                               static_cast<size_t>(px)) *
                                                  2U +
                                              static_cast<size_t>(levelIndex);
                        outCorr[outIdx] = dot;
                    }
                }
            }
        }
    }
}

bool DpvoNativeSolver::PackCorrelationCudaInputs(
    const std::vector<DpvoFrameState> &frames,
    const std::vector<DpvoEdgeState> &edges, int patchesPerFrame,
    const std::vector<std::array<float, DpvoNativeSolver::kPatchArea * 2>> &coords,
    std::vector<float> *edgePatchGmap, std::vector<float> *edgeCoords,
    std::vector<int> *edgeTargetFrame, std::vector<float> *fmapStorage,
    std::vector<float> *fmapLevel4Storage, std::vector<int> *fmapOffsets,
    std::vector<int> *fmapHeights, std::vector<int> *fmapWidths,
    std::vector<int> *level4Offsets, std::vector<int> *level4Heights,
    std::vector<int> *level4Widths)
{
    if (edgePatchGmap == nullptr || edgeCoords == nullptr ||
        edgeTargetFrame == nullptr || fmapStorage == nullptr ||
        fmapLevel4Storage == nullptr || fmapOffsets == nullptr ||
        fmapHeights == nullptr || fmapWidths == nullptr ||
        level4Offsets == nullptr || level4Heights == nullptr ||
        level4Widths == nullptr || patchesPerFrame <= 0 ||
        coords.size() < edges.size()) {
        return false;
    }
    const size_t edgeCount = edges.size();
    edgePatchGmap->assign(edgeCount * kFmapChannels * kPatchArea, 0.0f);
    edgeCoords->assign(edgeCount * kPatchArea * 2U, 0.0f);
    edgeTargetFrame->assign(edgeCount, 0);
    fmapStorage->clear();
    fmapLevel4Storage->clear();
    fmapOffsets->assign(frames.size(), 0);
    fmapHeights->assign(frames.size(), 0);
    fmapWidths->assign(frames.size(), 0);
    level4Offsets->assign(frames.size(), 0);
    level4Heights->assign(frames.size(), 0);
    level4Widths->assign(frames.size(), 0);
    for (size_t i = 0; i < frames.size(); ++i) {
        const DpvoFrameState &frame = frames[i];
        if (frame.fmapChannels != kFmapChannels || frame.fmap.empty() ||
            frame.fmapLevel4.empty() || frame.fmapHeight <= 0 ||
            frame.fmapWidth <= 0) {
            return false;
        }
        (*fmapOffsets)[i] = static_cast<int>(fmapStorage->size());
        (*fmapHeights)[i] = frame.fmapHeight;
        (*fmapWidths)[i] = frame.fmapWidth;
        fmapStorage->insert(fmapStorage->end(), frame.fmap.begin(),
                            frame.fmap.end());
        (*level4Offsets)[i] = static_cast<int>(fmapLevel4Storage->size());
        (*level4Heights)[i] = frame.fmapHeight / 4;
        (*level4Widths)[i] = frame.fmapWidth / 4;
        fmapLevel4Storage->insert(fmapLevel4Storage->end(),
                                  frame.fmapLevel4.begin(),
                                  frame.fmapLevel4.end());
    }
    for (size_t e = 0; e < edgeCount; ++e) {
        const DpvoEdgeState &edge = edges[e];
        if (edge.sourceFrame < 0 || edge.targetFrame < 0 ||
            static_cast<size_t>(edge.sourceFrame) >= frames.size() ||
            static_cast<size_t>(edge.targetFrame) >= frames.size()) {
            return false;
        }
        const DpvoFrameState &source =
            frames[static_cast<size_t>(edge.sourceFrame)];
        const int patchLocal = edge.patchGlobal % patchesPerFrame;
        if (patchLocal < 0) {
            return false;
        }
        const size_t gmapOffset =
            static_cast<size_t>(patchLocal) * kFmapChannels * kPatchArea;
        if (source.patchGmap.size() < gmapOffset + kFmapChannels * kPatchArea) {
            return false;
        }
        std::copy(
            source.patchGmap.begin() + static_cast<std::ptrdiff_t>(gmapOffset),
            source.patchGmap.begin() +
                static_cast<std::ptrdiff_t>(gmapOffset +
                                            kFmapChannels * kPatchArea),
            edgePatchGmap->begin() +
                static_cast<std::ptrdiff_t>(e * kFmapChannels * kPatchArea));
        std::copy(coords[e].begin(), coords[e].end(),
                  edgeCoords->begin() +
                      static_cast<std::ptrdiff_t>(e * kPatchArea * 2U));
        (*edgeTargetFrame)[e] = edge.targetFrame;
    }
    return !fmapStorage->empty() && !fmapLevel4Storage->empty();
}

void DpvoNativeSolver::SoftAggExpand(const std::vector<float> &f,
                          const std::vector<float> &g,
                          const std::vector<int> &groupIds, int edgeCount,
                          int dim, std::vector<float> *out)
{
    if (out == nullptr) {
        return;
    }
    out->assign(static_cast<size_t>(std::max(0, edgeCount)) *
                    static_cast<size_t>(dim),
                0.0f);
    if (edgeCount <= 0 || dim <= 0 ||
        f.size() < static_cast<size_t>(edgeCount) * dim ||
        g.size() < static_cast<size_t>(edgeCount) * dim ||
        groupIds.size() < static_cast<size_t>(edgeCount)) {
        return;
    }
    std::unordered_map<int, std::vector<int>> groups;
    groups.reserve(static_cast<size_t>(edgeCount));
    for (int e = 0; e < edgeCount; ++e) {
        groups[groupIds[static_cast<size_t>(e)]].push_back(e);
    }
    for (const auto &entry : groups) {
        const std::vector<int> &idx = entry.second;
        for (int c = 0; c < dim; ++c) {
            float maxLogit = -std::numeric_limits<float>::infinity();
            for (int e : idx) {
                maxLogit = std::max(
                    maxLogit,
                    g[static_cast<size_t>(e) * dim + static_cast<size_t>(c)]);
            }
            double denom = 0.0;
            double accum = 0.0;
            for (int e : idx) {
                const float logit =
                    g[static_cast<size_t>(e) * dim + static_cast<size_t>(c)];
                const double w = std::exp(static_cast<double>(logit - maxLogit));
                denom += w;
                accum +=
                    static_cast<double>(
                        f[static_cast<size_t>(e) * dim + static_cast<size_t>(c)]) *
                    w;
            }
            const float value =
                denom > 0.0 ? static_cast<float>(accum / denom) : 0.0f;
            for (int e : idx) {
                (*out)[static_cast<size_t>(e) * dim + static_cast<size_t>(c)] = value;
            }
        }
    }
}

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
