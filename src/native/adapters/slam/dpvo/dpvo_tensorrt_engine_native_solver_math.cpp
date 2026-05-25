#include "adapters/slam/dpvo/dpvo_tensorrt_engine_native_solver.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "adapters/slam/engine/slam_env.h"

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

namespace {

using SoftAggGroups = std::unordered_map<int, std::vector<int>>;

struct SoftAggInputs {
    const std::vector<float> &features;
    const std::vector<float> &logits;
    const std::vector<int> &groupIds;
    int edgeCount;
    int dim;
};

struct SoftAggChannelRequest {
    const std::vector<float> &features;
    const std::vector<float> &logits;
    const std::vector<int> &edgeIndices;
    int dim;
    int channel;
};

struct SoftAggGroupRequest {
    const std::vector<float> &features;
    const std::vector<float> &logits;
    const std::vector<int> &edgeIndices;
    int dim;
    std::vector<float> &out;
};

bool SoftAggInputsValid(const SoftAggInputs &inputs)
{
    if (inputs.edgeCount <= 0 || inputs.dim <= 0) {
        return false;
    }
    const size_t valueCount = static_cast<size_t>(inputs.edgeCount) *
                              static_cast<size_t>(inputs.dim);
    return inputs.features.size() >= valueCount &&
           inputs.logits.size() >= valueCount &&
           inputs.groupIds.size() >= static_cast<size_t>(inputs.edgeCount);
}

size_t SoftAggValueIndex(int edge, int dim, int channel)
{
    return static_cast<size_t>(edge) * static_cast<size_t>(dim) +
           static_cast<size_t>(channel);
}

SoftAggGroups BuildSoftAggGroups(const std::vector<int> &groupIds,
                                 int edgeCount)
{
    SoftAggGroups groups;
    groups.reserve(static_cast<size_t>(edgeCount));
    for (int edge = 0; edge < edgeCount; ++edge) {
        groups[groupIds[static_cast<size_t>(edge)]].push_back(edge);
    }
    return groups;
}

float MaxSoftAggLogit(const SoftAggChannelRequest &request)
{
    float maxLogit = -std::numeric_limits<float>::infinity();
    for (int edge : request.edgeIndices) {
        maxLogit = std::max(
            maxLogit,
            request.logits[SoftAggValueIndex(edge, request.dim,
                                             request.channel)]);
    }
    return maxLogit;
}

float ComputeSoftAggGroupChannel(const SoftAggChannelRequest &request)
{
    const float maxLogit = MaxSoftAggLogit(request);
    double denom = 0.0;
    double accum = 0.0;
    for (int edge : request.edgeIndices) {
        const size_t valueIndex =
            SoftAggValueIndex(edge, request.dim, request.channel);
        const float logit = request.logits[valueIndex];
        const double weight =
            std::exp(static_cast<double>(logit - maxLogit));
        denom += weight;
        accum += static_cast<double>(request.features[valueIndex]) * weight;
    }
    return denom > 0.0 ? static_cast<float>(accum / denom) : 0.0f;
}

void WriteSoftAggGroupChannel(const std::vector<int> &edgeIndices, int dim,
                              int channel, float value,
                              std::vector<float> &out)
{
    for (int edge : edgeIndices) {
        out[SoftAggValueIndex(edge, dim, channel)] = value;
    }
}

void FillSoftAggGroup(const SoftAggGroupRequest &request)
{
    for (int channel = 0; channel < request.dim; ++channel) {
        const SoftAggChannelRequest channelRequest{
            request.features, request.logits, request.edgeIndices,
            request.dim, channel};
        const float value = ComputeSoftAggGroupChannel(channelRequest);
        WriteSoftAggGroupChannel(request.edgeIndices, request.dim, channel,
                                 value, request.out);
    }
}

} // namespace

float DpvoNativeSolver::FeatureAt(const DpvoFeatureMapView &featureMap, int c,
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

float DpvoNativeSolver::SampleFeatureBilinear(
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
                           std::array<float, DpvoNativeSolver::PATCH_AREA * 2> &coords)
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
    for (int py = 0; py < PATCH_SIZE; ++py) {
        for (int px = 0; px < PATCH_SIZE; ++px) {
            const float x = patch.x + static_cast<float>(px - 1);
            const float y = patch.y + static_cast<float>(py - 1);
            const Eigen::Vector3f Xi((x - intrinsics.cx) / intrinsics.fx,
                                     (y - intrinsics.cy) / intrinsics.fy, 1.0f);
            Eigen::Vector3f Xj = R * Xi + patch.invDepth * t;
            const float z = std::fabs(Xj.z()) > 1e-4f ? Xj.z() : 1e-4f;
            const size_t idx = static_cast<size_t>(py * PATCH_SIZE + px) * 2U;
            coords[idx] = intrinsics.fx * (Xj.x() / z) + intrinsics.cx;
            coords[idx + 1U] = intrinsics.fy * (Xj.y() / z) + intrinsics.cy;
        }
    }
}

void
DpvoNativeSolver::ComputeCorrelation(const std::vector<DpvoFrameState> &frames,
                   const DpvoEdgeState &edge, int patchesPerFrame,
                   const std::array<float, DpvoNativeSolver::PATCH_AREA * 2> &coords,
                   float *outCorr)
{
    if (outCorr == nullptr) {
        return;
    }
    std::fill(outCorr, outCorr + CORR_DIM, 0.0f);
    CorrelationEdgeView view;
    if (!PrepareCorrelationEdgeView(frames, edge, patchesPerFrame, &view)) {
        return;
    }
    const CorrelationOffsetRequest request{*view.source, *view.target, coords,
                                           view.gmapOffset, outCorr};
    for (int offsetXIndex = 0; offsetXIndex < CORR_SIDE; ++offsetXIndex) {
        for (int offsetYIndex = 0; offsetYIndex < CORR_SIDE; ++offsetYIndex) {
            WriteCorrelationOffset(request, offsetXIndex, offsetYIndex);
        }
    }
}

bool DpvoNativeSolver::PrepareCorrelationEdgeView(
    const std::vector<DpvoFrameState> &frames, const DpvoEdgeState &edge,
    int patchesPerFrame, CorrelationEdgeView *view)
{
    if (view == nullptr || patchesPerFrame <= 0 || edge.sourceFrame < 0 ||
        edge.targetFrame < 0 ||
        static_cast<size_t>(edge.sourceFrame) >= frames.size() ||
        static_cast<size_t>(edge.targetFrame) >= frames.size()) {
        return false;
    }
    const DpvoFrameState &source =
        frames[static_cast<size_t>(edge.sourceFrame)];
    const DpvoFrameState &target =
        frames[static_cast<size_t>(edge.targetFrame)];
    const int patchLocal = edge.patchGlobal % patchesPerFrame;
    if (patchLocal < 0) {
        return false;
    }
    const size_t gmapOffset =
        static_cast<size_t>(patchLocal) * FMAP_CHANNELS * PATCH_AREA;
    if (source.patchGmap.size() < gmapOffset + FMAP_CHANNELS * PATCH_AREA ||
        target.fmap.empty() || target.fmapLevel4.empty()) {
        return false;
    }
    view->source = &source;
    view->target = &target;
    view->gmapOffset = gmapOffset;
    return true;
}

DpvoFeatureMapView DpvoNativeSolver::BuildCorrelationTargetMapView(
    const DpvoFrameState &target, int levelIndex)
{
    const std::vector<float> &targetMap =
        levelIndex == 0 ? target.fmap : target.fmapLevel4;
    const int targetHeight =
        levelIndex == 0 ? target.fmapHeight : target.fmapHeight / 4;
    const int targetWidth =
        levelIndex == 0 ? target.fmapWidth : target.fmapWidth / 4;
    return {targetMap.data(), FMAP_CHANNELS, targetHeight, targetWidth,
            targetMap.size()};
}

int DpvoNativeSolver::CorrelationLevelScale(int levelIndex)
{
    return levelIndex == 0 ? 1 : 4;
}

size_t DpvoNativeSolver::CorrelationPatchIndex(int patchY, int patchX)
{
    return static_cast<size_t>(patchY * PATCH_SIZE + patchX);
}

size_t DpvoNativeSolver::CorrelationCoordIndex(int patchY, int patchX)
{
    return CorrelationPatchIndex(patchY, patchX) * 2U;
}

size_t DpvoNativeSolver::CorrelationOutputIndex(int offsetXIndex,
                                                int offsetYIndex, int patchY,
                                                int patchX, int levelIndex)
{
    return (((static_cast<size_t>(offsetXIndex) * CORR_SIDE +
              static_cast<size_t>(offsetYIndex)) *
                 PATCH_SIZE +
             static_cast<size_t>(patchY)) *
                PATCH_SIZE +
            static_cast<size_t>(patchX)) *
               2U +
           static_cast<size_t>(levelIndex);
}

void DpvoNativeSolver::WriteCorrelationOffset(
    const CorrelationOffsetRequest &request, int offsetXIndex,
    int offsetYIndex)
{
    for (int patchY = 0; patchY < PATCH_SIZE; ++patchY) {
        for (int patchX = 0; patchX < PATCH_SIZE; ++patchX) {
            for (int levelIndex = 0; levelIndex < 2; ++levelIndex) {
                const CorrelationLevelRequest levelRequest{
                    request, offsetXIndex, offsetYIndex, patchY, patchX,
                    levelIndex};
                WriteCorrelationLevel(levelRequest);
            }
        }
    }
}

void DpvoNativeSolver::WriteCorrelationLevel(
    const CorrelationLevelRequest &request)
{
    const int level = CorrelationLevelScale(request.levelIndex);
    const size_t coordIdx =
        CorrelationCoordIndex(request.patchY, request.patchX);
    const int dx = request.offsetXIndex - CORR_RADIUS;
    const int dy = request.offsetYIndex - CORR_RADIUS;
    const DpvoFeatureMapView targetMapView = BuildCorrelationTargetMapView(
        request.offsetRequest.target, request.levelIndex);
    const CorrelationDotRequest dotRequest{
        request.offsetRequest.source, targetMapView,
        request.offsetRequest.gmapOffset,
        request.offsetRequest.coords[coordIdx] / static_cast<float>(level) +
            static_cast<float>(dx),
        request.offsetRequest.coords[coordIdx + 1U] /
                static_cast<float>(level) +
            static_cast<float>(dy),
        request.patchY, request.patchX};
    request.offsetRequest.outCorr[CorrelationOutputIndex(
        request.offsetXIndex, request.offsetYIndex, request.patchY,
        request.patchX, request.levelIndex)] = ComputeCorrelationDot(dotRequest);
}

float DpvoNativeSolver::ComputeCorrelationDot(
    const CorrelationDotRequest &request)
{
    float dot = 0.0f;
    const size_t patchIndex =
        CorrelationPatchIndex(request.patchY, request.patchX);
    for (int channel = 0; channel < FMAP_CHANNELS; ++channel) {
        const size_t gmapIndex = request.gmapOffset +
                                 static_cast<size_t>(channel) * PATCH_AREA +
                                 patchIndex;
        dot += request.source.patchGmap[gmapIndex] *
               SampleFeatureBilinear(request.targetMapView, channel,
                                     request.sampleX, request.sampleY);
    }
    return dot;
}

bool DpvoNativeSolver::PackCorrelationCudaInputs(
    const PackCorrelationCudaInputsRequest &request)
{
    if (!PackCorrelationOutputsValid(request)) {
        return false;
    }
    ResetPackCorrelationOutputs(request, request.edges.size());
    return PackCorrelationFrameMaps(request) &&
           PackCorrelationEdgeInputs(request) &&
           !request.fmapStorage->empty() &&
           !request.fmapLevel4Storage->empty();
}

bool DpvoNativeSolver::PackCorrelationOutputsValid(
    const PackCorrelationCudaInputsRequest &request)
{
    return request.edgePatchGmap != nullptr &&
           request.edgeCoords != nullptr &&
           request.edgeTargetFrame != nullptr &&
           request.fmapStorage != nullptr &&
           request.fmapLevel4Storage != nullptr &&
           request.fmapOffsets != nullptr &&
           request.fmapHeights != nullptr &&
           request.fmapWidths != nullptr &&
           request.level4Offsets != nullptr &&
           request.level4Heights != nullptr &&
           request.level4Widths != nullptr &&
           request.patchesPerFrame > 0 &&
           request.coords.size() >= request.edges.size();
}

void DpvoNativeSolver::ResetPackCorrelationOutputs(
    const PackCorrelationCudaInputsRequest &request, size_t edgeCount)
{
    request.edgePatchGmap->assign(edgeCount * FMAP_CHANNELS * PATCH_AREA,
                                  0.0f);
    request.edgeCoords->assign(edgeCount * PATCH_AREA * 2U, 0.0f);
    request.edgeTargetFrame->assign(edgeCount, 0);
    request.fmapStorage->clear();
    request.fmapLevel4Storage->clear();
    request.fmapOffsets->assign(request.frames.size(), 0);
    request.fmapHeights->assign(request.frames.size(), 0);
    request.fmapWidths->assign(request.frames.size(), 0);
    request.level4Offsets->assign(request.frames.size(), 0);
    request.level4Heights->assign(request.frames.size(), 0);
    request.level4Widths->assign(request.frames.size(), 0);
}

bool DpvoNativeSolver::PackCorrelationFrameMaps(
    const PackCorrelationCudaInputsRequest &request)
{
    for (size_t i = 0; i < request.frames.size(); ++i) {
        const DpvoFrameState &frame = request.frames[i];
        if (frame.fmapChannels != FMAP_CHANNELS || frame.fmap.empty() ||
            frame.fmapLevel4.empty() || frame.fmapHeight <= 0 ||
            frame.fmapWidth <= 0) {
            return false;
        }
        (*request.fmapOffsets)[i] =
            static_cast<int>(request.fmapStorage->size());
        (*request.fmapHeights)[i] = frame.fmapHeight;
        (*request.fmapWidths)[i] = frame.fmapWidth;
        request.fmapStorage->insert(request.fmapStorage->end(),
                                    frame.fmap.begin(), frame.fmap.end());
        (*request.level4Offsets)[i] =
            static_cast<int>(request.fmapLevel4Storage->size());
        (*request.level4Heights)[i] = frame.fmapHeight / 4;
        (*request.level4Widths)[i] = frame.fmapWidth / 4;
        request.fmapLevel4Storage->insert(request.fmapLevel4Storage->end(),
                                          frame.fmapLevel4.begin(),
                                          frame.fmapLevel4.end());
    }
    return true;
}

bool DpvoNativeSolver::PackCorrelationEdgeInputs(
    const PackCorrelationCudaInputsRequest &request)
{
    for (size_t e = 0; e < request.edges.size(); ++e) {
        const DpvoEdgeState &edge = request.edges[e];
        if (edge.sourceFrame < 0 || edge.targetFrame < 0 ||
            static_cast<size_t>(edge.sourceFrame) >= request.frames.size() ||
            static_cast<size_t>(edge.targetFrame) >= request.frames.size()) {
            return false;
        }
        const DpvoFrameState &source =
            request.frames[static_cast<size_t>(edge.sourceFrame)];
        const int patchLocal = edge.patchGlobal % request.patchesPerFrame;
        if (patchLocal < 0) {
            return false;
        }
        const size_t gmapOffset =
            static_cast<size_t>(patchLocal) * FMAP_CHANNELS * PATCH_AREA;
        if (source.patchGmap.size() < gmapOffset + FMAP_CHANNELS * PATCH_AREA) {
            return false;
        }
        std::copy(
            source.patchGmap.begin() + static_cast<std::ptrdiff_t>(gmapOffset),
            source.patchGmap.begin() +
                static_cast<std::ptrdiff_t>(gmapOffset +
                                            FMAP_CHANNELS * PATCH_AREA),
            request.edgePatchGmap->begin() +
                static_cast<std::ptrdiff_t>(e * FMAP_CHANNELS * PATCH_AREA));
        std::copy(request.coords[e].begin(), request.coords[e].end(),
                  request.edgeCoords->begin() +
                      static_cast<std::ptrdiff_t>(e * PATCH_AREA * 2U));
        (*request.edgeTargetFrame)[e] = edge.targetFrame;
    }
    return true;
}

void DpvoNativeSolver::SoftAggExpand(const SoftAggExpandRequest &request)
{
    std::vector<float> *out = request.out;
    if (out == nullptr) {
        return;
    }
    out->assign(static_cast<size_t>(std::max(0, request.edgeCount)) *
                    static_cast<size_t>(std::max(0, request.dim)),
                0.0f);
    const SoftAggInputs inputs{request.f, request.g, request.groupIds,
                               request.edgeCount, request.dim};
    if (!SoftAggInputsValid(inputs)) {
        return;
    }
    const SoftAggGroups groups =
        BuildSoftAggGroups(request.groupIds, request.edgeCount);
    for (const auto &entry : groups) {
        const SoftAggGroupRequest groupRequest{
            request.f, request.g, entry.second, request.dim, *out};
        FillSoftAggGroup(groupRequest);
    }
}

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
