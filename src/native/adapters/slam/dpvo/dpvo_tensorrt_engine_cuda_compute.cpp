#include "adapters/slam/dpvo/dpvo_tensorrt_engine_cuda_runtime.h"

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {
namespace {

const char *CudaBufferName(const char *name)
{
    return name != nullptr ? name : "unnamed";
}

} // namespace

template <typename T>
bool DpvoCudaKernelRuntime::CopyVectorToDevice(
    const std::vector<T> &src, CudaDeviceBuffer &dst, cudaStream_t stream,
    const char *name, std::string *err)
{
    if (src.empty()) {
        if (err != nullptr) {
            *err = std::string("empty DPVO CUDA buffer: ") +
                   CudaBufferName(name);
        }
        return false;
    }
    const size_t bytes = src.size() * sizeof(T);
    if (!dst.Ensure(bytes, err)) {
        return false;
    }
    const cudaError_t rc = cudaMemcpyAsync(dst.Data(), src.data(), bytes,
                                           cudaMemcpyHostToDevice, stream);
    if (rc == cudaSuccess) {
        return true;
    }
    if (err != nullptr) {
        *err = std::string("cudaMemcpyAsync failed for DPVO CUDA buffer ") +
               CudaBufferName(name) + ": " + cudaGetErrorString(rc);
    }
    return false;
}

template bool DpvoCudaKernelRuntime::CopyVectorToDevice<float>(
    const std::vector<float> &src, CudaDeviceBuffer &dst, cudaStream_t stream,
    const char *name, std::string *err);
template bool DpvoCudaKernelRuntime::CopyVectorToDevice<int>(
    const std::vector<int> &src, CudaDeviceBuffer &dst, cudaStream_t stream,
    const char *name, std::string *err);

DpvoCudaKernelRuntime::CorrelationBatchShape
DpvoCudaKernelRuntime::CorrelationShape(int edgeCount)
{
    static constexpr int CHANNELS = 128;
    static constexpr int PATCH_AREA = 9;
    static constexpr int CORR_RADIUS = 3;
    static constexpr int CORR_SIDE = 2 * CORR_RADIUS + 1;
    static constexpr int CORR_DIM = 2 * CORR_SIDE * CORR_SIDE * PATCH_AREA;
    const size_t edgeCountSize = static_cast<size_t>(std::max(0, edgeCount));
    return {edgeCountSize, edgeCountSize * CHANNELS * PATCH_AREA,
            edgeCountSize * PATCH_AREA * 2U, edgeCountSize * CORR_DIM};
}

bool DpvoCudaKernelRuntime::CorrelationBatchInputValid(
    const CorrelationBatchRequest &request,
    const CorrelationBatchShape &shape) const
{
    if (request.edgeCount <= 0 || request.stream == nullptr ||
        request.outCorr == nullptr) {
        return false;
    }
    return request.edgePatchGmap.size() == shape.patchValues &&
           request.edgeCoords.size() == shape.coordValues &&
           request.edgeTargetFrame.size() == shape.edgeCount &&
           request.fmapOffsets.size() == request.fmapHeights.size() &&
           request.fmapOffsets.size() == request.fmapWidths.size() &&
           request.level4Offsets.size() == request.level4Heights.size() &&
           request.level4Offsets.size() == request.level4Widths.size() &&
           !request.fmapStorage.empty() && !request.fmapLevel4Storage.empty();
}

bool DpvoCudaKernelRuntime::CopyCorrelationBatchInputs(
    const CorrelationBatchRequest &request, const CorrelationBatchShape &shape)
{
    cudaStream_t stream = request.stream;
    std::string *err = request.err;
    return CopyVectorToDevice(request.edgePatchGmap, m_edgePatchBuffer, stream,
                              "edge patch gmap", err) &&
           CopyVectorToDevice(request.edgeCoords, m_edgeCoordsBuffer, stream,
                              "edge coords", err) &&
           CopyVectorToDevice(request.edgeTargetFrame, m_edgeTargetFrameBuffer,
                              stream, "edge target frame", err) &&
           CopyVectorToDevice(request.fmapStorage, m_fmapBuffer, stream, "fmap",
                              err) &&
           CopyVectorToDevice(request.fmapLevel4Storage, m_fmapLevel4Buffer,
                              stream, "fmap level4", err) &&
           CopyVectorToDevice(request.fmapOffsets, m_fmapOffsetsBuffer, stream,
                              "fmap offsets", err) &&
           CopyVectorToDevice(request.fmapHeights, m_fmapHeightsBuffer, stream,
                              "fmap heights", err) &&
           CopyVectorToDevice(request.fmapWidths, m_fmapWidthsBuffer, stream,
                              "fmap widths", err) &&
           CopyVectorToDevice(request.level4Offsets, m_level4OffsetsBuffer,
                              stream, "level4 offsets", err) &&
           CopyVectorToDevice(request.level4Heights, m_level4HeightsBuffer,
                              stream, "level4 heights", err) &&
           CopyVectorToDevice(request.level4Widths, m_level4WidthsBuffer,
                              stream, "level4 widths", err) &&
           m_corrBuffer.Ensure(shape.corrValues * sizeof(float), err);
}

bool DpvoCudaKernelRuntime::LaunchCorrelationBatchKernel(
    const CorrelationBatchRequest &request, size_t corrValues)
{
    CUdeviceptr edgePatchArg =
        reinterpret_cast<CUdeviceptr>(m_edgePatchBuffer.Data());
    CUdeviceptr edgeCoordsArg =
        reinterpret_cast<CUdeviceptr>(m_edgeCoordsBuffer.Data());
    CUdeviceptr edgeTargetArg =
        reinterpret_cast<CUdeviceptr>(m_edgeTargetFrameBuffer.Data());
    CUdeviceptr fmapArg = reinterpret_cast<CUdeviceptr>(m_fmapBuffer.Data());
    CUdeviceptr fmapLevel4Arg =
        reinterpret_cast<CUdeviceptr>(m_fmapLevel4Buffer.Data());
    CUdeviceptr fmapOffsetsArg =
        reinterpret_cast<CUdeviceptr>(m_fmapOffsetsBuffer.Data());
    CUdeviceptr fmapHeightsArg =
        reinterpret_cast<CUdeviceptr>(m_fmapHeightsBuffer.Data());
    CUdeviceptr fmapWidthsArg =
        reinterpret_cast<CUdeviceptr>(m_fmapWidthsBuffer.Data());
    CUdeviceptr level4OffsetsArg =
        reinterpret_cast<CUdeviceptr>(m_level4OffsetsBuffer.Data());
    CUdeviceptr level4HeightsArg =
        reinterpret_cast<CUdeviceptr>(m_level4HeightsBuffer.Data());
    CUdeviceptr level4WidthsArg =
        reinterpret_cast<CUdeviceptr>(m_level4WidthsBuffer.Data());
    CUdeviceptr outArg = reinterpret_cast<CUdeviceptr>(m_corrBuffer.Data());
    int edgeCountArg = request.edgeCount;
    void *args[] = {&edgePatchArg, &edgeCoordsArg, &edgeTargetArg,
                    &fmapArg, &fmapLevel4Arg, &fmapOffsetsArg,
                    &fmapHeightsArg, &fmapWidthsArg, &level4OffsetsArg,
                    &level4HeightsArg, &level4WidthsArg, &outArg,
                    &edgeCountArg};
    const unsigned int threads = 128;
    const unsigned int blocks = static_cast<unsigned int>(
        (corrValues + static_cast<size_t>(threads) - 1U) /
        static_cast<size_t>(threads));
    return CheckDriver(
        m_cuLaunchKernel(m_corrBatchKernel, blocks, 1, 1, threads, 1, 1, 0,
                         reinterpret_cast<CUstream>(request.stream), args,
                         nullptr),
        "cuLaunchKernel(dpvo_corr_batch)", request.err);
}

bool DpvoCudaKernelRuntime::CopyCorrelationBatchOutput(
    const CorrelationBatchRequest &request, size_t corrValues)
{
    request.outCorr->assign(corrValues, 0.0f);
    cudaError_t rc =
        cudaMemcpyAsync(request.outCorr->data(), m_corrBuffer.Data(),
                        corrValues * sizeof(float), cudaMemcpyDeviceToHost,
                        request.stream);
    if (rc == cudaSuccess) {
        rc = cudaStreamSynchronize(request.stream);
    }
    if (rc == cudaSuccess) {
        return true;
    }
    if (request.err != nullptr) {
        *request.err = std::string("DPVO CUDA correlation copy/sync failed: ") +
                       cudaGetErrorString(rc);
    }
    return false;
}

bool DpvoCudaKernelRuntime::ComputeCorrelationBatch(
    const CorrelationBatchRequest &request)
{
    const CorrelationBatchShape shape = CorrelationShape(request.edgeCount);
    if (!m_ready || m_corrBatchKernel == nullptr) {
        if (request.err != nullptr) {
            *request.err = "DPVO CUDA correlation kernel is not ready";
        }
        return false;
    }
    if (!CorrelationBatchInputValid(request, shape)) {
        if (request.err != nullptr) {
            *request.err = "invalid DPVO CUDA correlation batch input";
        }
        return false;
    }
    return CopyCorrelationBatchInputs(request, shape) &&
           LaunchCorrelationBatchKernel(request, shape.corrValues) &&
           CopyCorrelationBatchOutput(request, shape.corrValues);
}

bool DpvoCudaKernelRuntime::SoftAggHostInputValid(
    const SoftAggExpandRequest &request, size_t values) const
{
    if (request.edgeCount <= 0 || request.dim <= 0 ||
        request.stream == nullptr || request.out == nullptr) {
        return false;
    }
    return request.f.size() == values && request.g.size() == values &&
           request.groupIds.size() == static_cast<size_t>(request.edgeCount);
}

bool DpvoCudaKernelRuntime::SoftAggDeviceInputValid(
    const SoftAggExpandDeviceRequest &request, size_t values) const
{
    if (request.edgeCount <= 0 || request.dim <= 0 ||
        request.stream == nullptr) {
        return false;
    }
    return request.fDevice.Bytes() >= values * sizeof(float) &&
           request.gDevice.Bytes() >= values * sizeof(float) &&
           request.groupIds.size() == static_cast<size_t>(request.edgeCount);
}

DpvoCudaKernelRuntime::SoftAggGroups
DpvoCudaKernelRuntime::BuildSoftAggGroups(
    const std::vector<int> &groupIds, int edgeCount)
{
    std::unordered_map<int, int> groupToDense;
    groupToDense.reserve(static_cast<size_t>(edgeCount));
    std::vector<std::vector<int>> groupedEdges;
    groupedEdges.reserve(static_cast<size_t>(edgeCount));
    for (int e = 0; e < edgeCount; ++e) {
        const int id = groupIds[static_cast<size_t>(e)];
        auto it = groupToDense.find(id);
        if (it == groupToDense.end()) {
            const int dense = static_cast<int>(groupedEdges.size());
            it = groupToDense.emplace(id, dense).first;
            groupedEdges.emplace_back();
        }
        groupedEdges[static_cast<size_t>(it->second)].push_back(e);
    }
    SoftAggGroups groups;
    groups.count = static_cast<int>(groupedEdges.size());
    groups.starts.assign(groupedEdges.size() + 1U, 0);
    groups.indices.reserve(static_cast<size_t>(edgeCount));
    for (size_t i = 0; i < groupedEdges.size(); ++i) {
        groups.starts[i] = static_cast<int>(groups.indices.size());
        groups.indices.insert(groups.indices.end(), groupedEdges[i].begin(),
                              groupedEdges[i].end());
    }
    groups.starts[groupedEdges.size()] =
        static_cast<int>(groups.indices.size());
    return groups;
}

bool DpvoCudaKernelRuntime::CopySoftAggGroupTables(
    const SoftAggGroups &groups, cudaStream_t stream, std::string *err)
{
    return CopyVectorToDevice(groups.indices, m_softAggGroupIndexBuffer, stream,
                              "softagg group indices", err) &&
           CopyVectorToDevice(groups.starts, m_softAggGroupStartBuffer, stream,
                              "softagg group starts", err);
}

bool DpvoCudaKernelRuntime::LaunchSoftAggKernel(
    const SoftAggLaunchRequest &request)
{
    CUdeviceptr fArg = reinterpret_cast<CUdeviceptr>(request.fDevice.Data());
    CUdeviceptr gArg = reinterpret_cast<CUdeviceptr>(request.gDevice.Data());
    CUdeviceptr groupIndexArg =
        reinterpret_cast<CUdeviceptr>(m_softAggGroupIndexBuffer.Data());
    CUdeviceptr groupStartArg =
        reinterpret_cast<CUdeviceptr>(m_softAggGroupStartBuffer.Data());
    CUdeviceptr outArg = reinterpret_cast<CUdeviceptr>(request.outDevice.Data());
    int groupCountArg = request.groupCount;
    int dimArg = request.dim;
    void *args[] = {&fArg, &gArg, &groupIndexArg, &groupStartArg,
                    &outArg, &groupCountArg, &dimArg};
    const unsigned int threads = 128;
    const size_t workItems =
        static_cast<size_t>(request.groupCount) * static_cast<size_t>(request.dim);
    const unsigned int blocks = static_cast<unsigned int>(
        (workItems + static_cast<size_t>(threads) - 1U) /
        static_cast<size_t>(threads));
    return CheckDriver(
        m_cuLaunchKernel(m_softAggKernel, blocks, 1, 1, threads, 1, 1, 0,
                         reinterpret_cast<CUstream>(request.stream), args,
                         nullptr),
        request.kernelName, request.err);
}

bool DpvoCudaKernelRuntime::CopySoftAggOutput(
    const SoftAggExpandRequest &request, size_t values)
{
    request.out->assign(values, 0.0f);
    cudaError_t rc =
        cudaMemcpyAsync(request.out->data(), m_softAggOutBuffer.Data(),
                        values * sizeof(float), cudaMemcpyDeviceToHost,
                        request.stream);
    if (rc == cudaSuccess) {
        rc = cudaStreamSynchronize(request.stream);
    }
    if (rc == cudaSuccess) {
        return true;
    }
    if (request.err != nullptr) {
        *request.err = std::string("DPVO CUDA SoftAgg copy/sync failed: ") +
                       cudaGetErrorString(rc);
    }
    return false;
}

bool DpvoCudaKernelRuntime::ComputeSoftAggExpand(
    const SoftAggExpandRequest &request)
{
    if (!m_ready || m_softAggKernel == nullptr) {
        if (request.err != nullptr) {
            *request.err = "DPVO CUDA SoftAgg kernel is not ready";
        }
        return false;
    }
    const size_t values =
        static_cast<size_t>(request.edgeCount) * static_cast<size_t>(request.dim);
    if (!SoftAggHostInputValid(request, values)) {
        if (request.err != nullptr) {
            *request.err = "invalid DPVO CUDA SoftAgg input";
        }
        return false;
    }
    const SoftAggGroups groups =
        BuildSoftAggGroups(request.groupIds, request.edgeCount);
    if (groups.count <= 0) {
        if (request.err != nullptr) {
            *request.err = "DPVO CUDA SoftAgg has no groups";
        }
        return false;
    }
    if (!CopyVectorToDevice(request.f, m_softAggFBuffer, request.stream,
                            "softagg f", request.err) ||
        !CopyVectorToDevice(request.g, m_softAggGBuffer, request.stream,
                            "softagg g", request.err) ||
        !CopySoftAggGroupTables(groups, request.stream, request.err) ||
        !m_softAggOutBuffer.Ensure(values * sizeof(float), request.err)) {
        return false;
    }
    const SoftAggLaunchRequest launchRequest{
        m_softAggFBuffer, m_softAggGBuffer, m_softAggOutBuffer,
        groups.count, request.dim, request.stream,
        "cuLaunchKernel(dpvo_softagg_expand)", request.err};
    return LaunchSoftAggKernel(launchRequest) &&
           CopySoftAggOutput(request, values);
}

bool DpvoCudaKernelRuntime::ComputeSoftAggExpandDevice(
    const SoftAggExpandDeviceRequest &request)
{
    if (!m_ready || m_softAggKernel == nullptr) {
        if (request.err != nullptr) {
            *request.err = "DPVO CUDA SoftAgg kernel is not ready";
        }
        return false;
    }
    const size_t values = static_cast<size_t>(std::max(0, request.edgeCount)) *
                          static_cast<size_t>(std::max(0, request.dim));
    if (!SoftAggDeviceInputValid(request, values)) {
        if (request.err != nullptr) {
            *request.err = "invalid DPVO CUDA SoftAgg device input";
        }
        return false;
    }
    const SoftAggGroups groups =
        BuildSoftAggGroups(request.groupIds, request.edgeCount);
    if (groups.count <= 0) {
        if (request.err != nullptr) {
            *request.err = "DPVO CUDA SoftAgg device path has no groups";
        }
        return false;
    }
    if (!CopySoftAggGroupTables(groups, request.stream, request.err) ||
        !request.outDevice.Ensure(values * sizeof(float), request.err)) {
        return false;
    }
    const SoftAggLaunchRequest launchRequest{
        request.fDevice, request.gDevice, request.outDevice,
        groups.count, request.dim, request.stream,
        "cuLaunchKernel(dpvo_softagg_expand device)", request.err};
    return LaunchSoftAggKernel(launchRequest);
}

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
