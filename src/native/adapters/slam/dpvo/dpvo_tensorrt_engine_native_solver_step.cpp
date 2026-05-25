#include "adapters/slam/dpvo/dpvo_tensorrt_engine_native_solver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "adapters/slam/dpvo/dpvo_tensorrt_engine_cuda_runtime.h"
#include "adapters/slam/engine/slam_env.h"

#include <NvInferPlugin.h>
#include <cuda_fp16.h>

namespace SmartDrone::Adapters::Slam {

namespace {

double ElapsedMs(const std::chrono::steady_clock::time_point &start,
                 const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

} // namespace

namespace DpvoTensorRtInternal {

#include "dpvo_tensorrt_engine_trt_handles.h"
#include "dpvo_tensorrt_engine_bindings.h"
#include "dpvo_tensorrt_engine_update_runtime.h"
#include "dpvo_tensorrt_engine_update_postagg_runtime.h"
#include "dpvo_tensorrt_engine_update_preagg_runtime.h"

void DpvoNativeSolver::Reset()
{
    m_netByEdge.clear();
    m_lastTcw = Sophus::SE3f();
    m_hasPose = false;
    m_bootstrapComplete = false;
    m_loggedCudaCorr = false;
    m_loggedCudaCorrFailure = false;
    m_loggedCudaSoftAgg = false;
    m_loggedCudaSoftAggFailure = false;
    m_loggedCudaDeviceUpdateChain = false;
    m_loggedCudaDeviceUpdateChainFailure = false;
}

bool DpvoNativeSolver::HasPose() const
{
    return m_hasPose;
}
Sophus::SE3f DpvoNativeSolver::LastTcw() const
{
    return m_lastTcw;
}

bool DpvoNativeSolver::PrepareStepInputs(
    const DpvoNativeSolverStepRequest &request,
    std::vector<DpvoFrameState> *&frames,
    const std::vector<DpvoEdgeState> *&edges)
{
    if (request.updateMs != nullptr) {
        *request.updateMs = 0.0;
    }
    frames = &request.graph.MutableFrames();
    edges = &request.graph.Edges();
    if (frames->empty() || edges->empty()) {
        return false;
    }
    PredictNewestPose(*frames);
    if (!request.graph.Initialized()) {
        m_lastTcw = frames->back().Tcw;
        m_hasPose = true;
        return false;
    }
    if (!request.graph.FeatureMapsReady()) {
        if (request.err != nullptr) {
            *request.err = "native DPVO feature maps are not ready";
        }
        return false;
    }
    if (request.preAggEngine.Engine() == nullptr ||
        request.postAggEngine.Engine() == nullptr) {
        if (request.err != nullptr) {
            *request.err = "native DPVO requires split preagg/postagg TensorRT engines";
        }
        return false;
    }
    return true;
}

int DpvoNativeSolver::UpdateIterationCount() const
{
    const int bootstrapIterations = std::clamp(
        EnvIntValue("SMART_DRONE_DPVO_BOOTSTRAP_UPDATE_ITERS", 12), 1, 12);
    const int steadyIterations =
        std::clamp(EnvIntValue("SMART_DRONE_DPVO_UPDATE_ITERS", 1), 1, 4);
    return m_bootstrapComplete ? steadyIterations : bootstrapIterations;
}

void DpvoNativeSolver::InitializeStepIteration(
    const StepIterationRequest &request, StepIterationBuffers &buffers)
{
    BuildTemporalNeighbors(request.edges, &buffers.prevEdge, &buffers.nextEdge);
    const size_t edgeCount = static_cast<size_t>(request.edgeCount);
    buffers.net.assign(edgeCount * DIM, 0.0f);
    buffers.inp.assign(edgeCount * DIM, 0.0f);
    buffers.corr.assign(edgeCount * CORR_DIM, 0.0f);
    buffers.prevNet.assign(edgeCount * DIM, 0.0f);
    buffers.nextNet.assign(edgeCount * DIM, 0.0f);
    buffers.prevMask.assign(edgeCount, 0.0f);
    buffers.nextMask.assign(edgeCount, 0.0f);
    buffers.coords.assign(edgeCount, {});
    buffers.groupKk.assign(edgeCount, 0);
    buffers.groupIj.assign(edgeCount, 0);
    FillEdgeInputs(request, buffers);
    FillTemporalInputs(request.edgeCount, buffers);
    FillGroupIds(request.edges, buffers);
}

void DpvoNativeSolver::FillEdgeInputs(const StepIterationRequest &request,
                                      StepIterationBuffers &buffers)
{
    const int patchesPerFrame = request.step.graph.PatchesPerFrame();
    for (int e = 0; e < request.edgeCount; ++e) {
        const DpvoEdgeState &edge = request.edges[static_cast<size_t>(e)];
        const DpvoFrameState &source =
            request.frames[static_cast<size_t>(edge.sourceFrame)];
        const int patchLocal = edge.patchGlobal % patchesPerFrame;
        const DpvoEdgeKey key{
            source.frameId,
            request.frames[static_cast<size_t>(edge.targetFrame)].frameId,
            patchLocal};
        auto it = m_netByEdge.find(key);
        if (it != m_netByEdge.end() &&
            it->second.size() == static_cast<size_t>(DIM)) {
            std::copy(it->second.begin(), it->second.end(),
                      buffers.net.begin() + static_cast<size_t>(e) * DIM);
        }
        const size_t imapOffset = static_cast<size_t>(patchLocal) * DIM;
        if (source.patchImap.size() >= imapOffset + DIM) {
            std::copy(source.patchImap.begin() +
                          static_cast<std::ptrdiff_t>(imapOffset),
                      source.patchImap.begin() +
                          static_cast<std::ptrdiff_t>(imapOffset + DIM),
                      buffers.inp.begin() + static_cast<std::ptrdiff_t>(
                                                static_cast<size_t>(e) * DIM));
        }
        ReprojectPatch(request.frames, edge, patchesPerFrame,
                       request.step.intrinsics, buffers.coords[static_cast<size_t>(e)]);
    }
}

void DpvoNativeSolver::FillTemporalInputs(int edgeCount,
                                          StepIterationBuffers &buffers)
{
    for (int e = 0; e < edgeCount; ++e) {
        if (buffers.prevEdge[static_cast<size_t>(e)] >= 0) {
            buffers.prevMask[static_cast<size_t>(e)] = 1.0f;
            const size_t src =
                static_cast<size_t>(buffers.prevEdge[static_cast<size_t>(e)]) * DIM;
            std::copy(buffers.net.begin() + static_cast<std::ptrdiff_t>(src),
                      buffers.net.begin() + static_cast<std::ptrdiff_t>(src + DIM),
                      buffers.prevNet.begin() + static_cast<std::ptrdiff_t>(
                                                    static_cast<size_t>(e) * DIM));
        }
        if (buffers.nextEdge[static_cast<size_t>(e)] >= 0) {
            buffers.nextMask[static_cast<size_t>(e)] = 1.0f;
            const size_t src =
                static_cast<size_t>(buffers.nextEdge[static_cast<size_t>(e)]) * DIM;
            std::copy(buffers.net.begin() + static_cast<std::ptrdiff_t>(src),
                      buffers.net.begin() + static_cast<std::ptrdiff_t>(src + DIM),
                      buffers.nextNet.begin() + static_cast<std::ptrdiff_t>(
                                                    static_cast<size_t>(e) * DIM));
        }
    }
}

void DpvoNativeSolver::FillGroupIds(const std::vector<DpvoEdgeState> &edges,
                                    StepIterationBuffers &buffers)
{
    for (int e = 0; e < static_cast<int>(edges.size()); ++e) {
        const DpvoEdgeState &edge = edges[static_cast<size_t>(e)];
        buffers.groupKk[static_cast<size_t>(e)] = edge.patchGlobal;
        buffers.groupIj[static_cast<size_t>(e)] =
            edge.sourceFrame * 12345 + edge.targetFrame;
    }
}

void DpvoNativeSolver::ComputeStepCorrelation(
    const StepIterationRequest &request, StepIterationBuffers &buffers)
{
    if (!TryComputeCudaCorrelation(request, buffers)) {
        ComputeCpuCorrelationBatch(request, buffers);
    }
}

bool DpvoNativeSolver::TryComputeCudaCorrelation(
    const StepIterationRequest &request, StepIterationBuffers &buffers)
{
    DpvoCudaKernelRuntime *cudaKernelRuntime = request.step.cudaKernelRuntime;
    if (cudaKernelRuntime == nullptr || !cudaKernelRuntime->Ready() ||
        !EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_CORR", true)) {
        return false;
    }
    CudaCorrelationInputs inputs;
    if (!PrepareCudaCorrelationInputs(request, buffers, inputs)) {
        return false;
    }
    std::vector<float> cudaCorr;
    std::string cudaCorrErr;
    if (!cudaKernelRuntime->ComputeCorrelationBatch(
            {request.edgeCount, inputs.edgePatchGmap, inputs.edgeCoords,
             inputs.edgeTargetFrame, inputs.fmapStorage, inputs.fmapLevel4Storage,
             inputs.fmapOffsets, inputs.fmapHeights, inputs.fmapWidths,
             inputs.level4Offsets, inputs.level4Heights, inputs.level4Widths,
             request.step.stream, &cudaCorr, &cudaCorrErr})) {
        if (!m_loggedCudaCorrFailure) {
            std::cerr << "[dpvo_cuda] correlation batch unavailable: "
                      << cudaCorrErr << "; falling back to CPU correlation\n";
            m_loggedCudaCorrFailure = true;
        }
        return false;
    }
    ValidateCudaCorrelation(request, buffers, cudaCorr);
    if (cudaCorr.size() != buffers.corr.size()) {
        return false;
    }
    buffers.corr.swap(cudaCorr);
    return true;
}

bool DpvoNativeSolver::PrepareCudaCorrelationInputs(
    const StepIterationRequest &request, StepIterationBuffers &buffers,
    CudaCorrelationInputs &inputs)
{
    if (PackCorrelationCudaInputs(
            {request.frames, request.edges, request.step.graph.PatchesPerFrame(),
             buffers.coords, &inputs.edgePatchGmap, &inputs.edgeCoords,
             &inputs.edgeTargetFrame, &inputs.fmapStorage,
             &inputs.fmapLevel4Storage, &inputs.fmapOffsets, &inputs.fmapHeights,
             &inputs.fmapWidths, &inputs.level4Offsets, &inputs.level4Heights,
             &inputs.level4Widths})) {
        return true;
    }
    if (!m_loggedCudaCorrFailure) {
        std::cerr << "[dpvo_cuda] correlation batch input packing failed; "
                     "falling back to CPU correlation\n";
        m_loggedCudaCorrFailure = true;
    }
    return false;
}

void DpvoNativeSolver::ValidateCudaCorrelation(
    const StepIterationRequest &request, StepIterationBuffers &buffers,
    std::vector<float> &cudaCorr)
{
    if (m_loggedCudaCorr &&
        !EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_CORR_VALIDATE_EVERY_STEP", false)) {
        return;
    }
    StepIterationBuffers cpuBuffers;
    cpuBuffers.coords = buffers.coords;
    ComputeCpuCorrelationBatch(request, cpuBuffers);
    std::vector<float> cpuCorr = std::move(cpuBuffers.corr);
    const ErrorStats stats = CompareVectors(cpuCorr, cudaCorr);
    const size_t valueCount = std::min(cpuCorr.size(), cudaCorr.size());
    std::cerr << "[dpvo_cuda] correlation batch ready edges="
              << request.edgeCount << " values=" << valueCount
              << " max_abs=" << stats.maxAbs << " rmse=" << stats.rmse
              << "\n";
    m_loggedCudaCorr = true;
    if (stats.maxAbs > 1e-3f &&
        EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_CORR_STRICT_VALIDATE", true)) {
        cudaCorr = std::move(cpuCorr);
        std::cerr << "[dpvo_cuda] correlation validation exceeded "
                     "threshold; using CPU corr for this step\n";
    }
}

void DpvoNativeSolver::ComputeCpuCorrelationBatch(
    const StepIterationRequest &request, StepIterationBuffers &buffers)
{
    buffers.corr.assign(static_cast<size_t>(request.edgeCount) * CORR_DIM, 0.0f);
    for (int e = 0; e < request.edgeCount; ++e) {
        ComputeCorrelation(request.frames, request.edges[static_cast<size_t>(e)],
                           request.step.graph.PatchesPerFrame(),
                           buffers.coords[static_cast<size_t>(e)],
                           buffers.corr.data() + static_cast<size_t>(e) * CORR_DIM);
    }
}

bool DpvoNativeSolver::TryRunDeviceUpdateChain(
    const DeviceUpdateChainRequest &request)
{
    const StepIterationRequest &iteration = request.iteration;
    DpvoCudaKernelRuntime *cudaKernelRuntime =
        iteration.step.cudaKernelRuntime;
    if (cudaKernelRuntime == nullptr || !cudaKernelRuntime->Ready() ||
        !EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_DEVICE_UPDATE_CHAIN", true) ||
        !EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_SOFTAGG", true)) {
        return false;
    }
    std::string deviceErr;
    DpvoUpdatePreAggDeviceRun preAggDevice =
        iteration.step.preAggRuntime.RunDevice(
            {iteration.step.preAggEngine, iteration.step.stream,
             iteration.edgeCount, request.buffers.net, request.buffers.inp,
             request.buffers.corr, request.buffers.prevNet,
             request.buffers.nextNet, request.buffers.prevMask,
             request.buffers.nextMask, &deviceErr});
    CudaDeviceBuffer aggKkYDevice;
    CudaDeviceBuffer aggIjYDevice;
    bool usedDeviceUpdateChain = false;
    if (preAggDevice.ok &&
        cudaKernelRuntime->ComputeSoftAggExpandDevice(
            {iteration.step.preAggRuntime.AggKkFDevice(),
             iteration.step.preAggRuntime.AggKkGDevice(), request.buffers.groupKk,
             iteration.edgeCount, DIM, iteration.step.stream, aggKkYDevice,
             &deviceErr}) &&
        cudaKernelRuntime->ComputeSoftAggExpandDevice(
            {iteration.step.preAggRuntime.AggIjFDevice(),
             iteration.step.preAggRuntime.AggIjGDevice(), request.buffers.groupIj,
             iteration.edgeCount, DIM, iteration.step.stream, aggIjYDevice,
             &deviceErr}) &&
        ValidateDeviceUpdateChain(request, aggKkYDevice, aggIjYDevice,
                                  deviceErr) &&
        deviceErr.empty()) {
        request.postAgg = iteration.step.postAggRuntime.RunDevice(
            {iteration.step.postAggEngine, iteration.step.stream,
             iteration.edgeCount, iteration.step.preAggRuntime.BaseNetDevice(),
             aggKkYDevice, aggIjYDevice, iteration.step.err});
        usedDeviceUpdateChain = request.postAgg.ok;
    }
    if (!usedDeviceUpdateChain && !m_loggedCudaDeviceUpdateChainFailure) {
        std::cerr << "[dpvo_cuda] device update chain unavailable: "
                  << deviceErr << "; falling back to host update chain\n";
        m_loggedCudaDeviceUpdateChainFailure = true;
    }
    return usedDeviceUpdateChain;
}

bool DpvoNativeSolver::ValidateDeviceUpdateChain(
    const DeviceUpdateChainRequest &request, CudaDeviceBuffer &aggKkYDevice,
    CudaDeviceBuffer &aggIjYDevice, std::string &deviceErr)
{
    if (m_loggedCudaDeviceUpdateChain ||
        !EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_DEVICE_UPDATE_CHAIN_VALIDATE",
                        true)) {
        return true;
    }
    DeviceUpdateValidationBuffers buffers;
    if (!CopyDeviceUpdateValidationBuffers(request, aggKkYDevice, aggIjYDevice,
                                           buffers, deviceErr)) {
        return false;
    }
    return ValidateDeviceUpdateSoftAgg(request, buffers, deviceErr);
}

bool DpvoNativeSolver::CopyDeviceUpdateValidationBuffers(
    const DeviceUpdateChainRequest &request, CudaDeviceBuffer &aggKkYDevice,
    CudaDeviceBuffer &aggIjYDevice, DeviceUpdateValidationBuffers &buffers,
    std::string &deviceErr)
{
    const StepIterationRequest &iteration = request.iteration;
    const size_t dimValues = static_cast<size_t>(iteration.edgeCount) * DIM;
    return
        CopyFloatDeviceBufferToHost(iteration.step.preAggRuntime.AggKkFDevice(),
                                    dimValues, iteration.step.stream, buffers.kkF,
                                    &deviceErr) &&
        CopyFloatDeviceBufferToHost(iteration.step.preAggRuntime.AggKkGDevice(),
                                    dimValues, iteration.step.stream, buffers.kkG,
                                    &deviceErr) &&
        CopyFloatDeviceBufferToHost(iteration.step.preAggRuntime.AggIjFDevice(),
                                    dimValues, iteration.step.stream, buffers.ijF,
                                    &deviceErr) &&
        CopyFloatDeviceBufferToHost(iteration.step.preAggRuntime.AggIjGDevice(),
                                    dimValues, iteration.step.stream, buffers.ijG,
                                    &deviceErr) &&
        CopyFloatDeviceBufferToHost(aggKkYDevice, dimValues,
                                    iteration.step.stream, buffers.cudaAggKkY,
                                    &deviceErr) &&
        CopyFloatDeviceBufferToHost(aggIjYDevice, dimValues,
                                    iteration.step.stream, buffers.cudaAggIjY,
                                    &deviceErr);
}

bool DpvoNativeSolver::ValidateDeviceUpdateSoftAgg(
    const DeviceUpdateChainRequest &request,
    const DeviceUpdateValidationBuffers &buffers, std::string &deviceErr)
{
    const StepIterationRequest &iteration = request.iteration;
    std::vector<float> cpuAggKkY;
    std::vector<float> cpuAggIjY;
    SoftAggExpand({buffers.kkF, buffers.kkG, request.buffers.groupKk,
                   iteration.edgeCount, DIM, &cpuAggKkY});
    SoftAggExpand({buffers.ijF, buffers.ijG, request.buffers.groupIj,
                   iteration.edgeCount, DIM, &cpuAggIjY});
    const ErrorStats kkStats = CompareVectors(cpuAggKkY, buffers.cudaAggKkY);
    const ErrorStats ijStats = CompareVectors(cpuAggIjY, buffers.cudaAggIjY);
    std::cerr << "[dpvo_cuda] device update chain softagg ready edges="
              << iteration.edgeCount << " dim=" << DIM
              << " kk_max_abs=" << kkStats.maxAbs
              << " kk_rmse=" << kkStats.rmse
              << " ij_max_abs=" << ijStats.maxAbs
              << " ij_rmse=" << ijStats.rmse << "\n";
    m_loggedCudaDeviceUpdateChain = true;
    const bool validationOk =
        (kkStats.maxAbs <= 1e-3f && ijStats.maxAbs <= 1e-3f) ||
        !EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_DEVICE_UPDATE_CHAIN_"
                        "STRICT_VALIDATE",
                        true);
    if (!validationOk) {
        deviceErr = "device update chain validation failed";
    }
    return validationOk;
}

bool DpvoNativeSolver::RunHostUpdateChain(
    const HostUpdateChainRequest &request)
{
    const StepIterationRequest &iteration = request.iteration;
    DpvoUpdatePreAggRun preAgg = iteration.step.preAggRuntime.Run(
        {iteration.step.preAggEngine, iteration.step.stream, iteration.edgeCount,
         request.buffers.net, request.buffers.inp, request.buffers.corr,
         request.buffers.prevNet, request.buffers.nextNet,
         request.buffers.prevMask, request.buffers.nextMask, iteration.step.err});
    if (!preAgg.ok) {
        return false;
    }
    std::vector<float> aggKkY;
    std::vector<float> aggIjY;
    if (!TryComputeCudaSoftAgg(request, preAgg, aggKkY, aggIjY)) {
        SoftAggExpand({preAgg.aggKkF, preAgg.aggKkG, request.buffers.groupKk,
                       iteration.edgeCount, DIM, &aggKkY});
        SoftAggExpand({preAgg.aggIjF, preAgg.aggIjG, request.buffers.groupIj,
                       iteration.edgeCount, DIM, &aggIjY});
    }
    request.postAgg = iteration.step.postAggRuntime.Run(
        {iteration.step.postAggEngine, iteration.step.stream, iteration.edgeCount,
         preAgg.baseNet, aggKkY, aggIjY, iteration.step.err});
    return request.postAgg.ok;
}

bool DpvoNativeSolver::TryComputeCudaSoftAgg(
    const HostUpdateChainRequest &request, const DpvoUpdatePreAggRun &preAgg,
    std::vector<float> &aggKkY, std::vector<float> &aggIjY)
{
    const StepIterationRequest &iteration = request.iteration;
    DpvoCudaKernelRuntime *cudaKernelRuntime =
        iteration.step.cudaKernelRuntime;
    if (cudaKernelRuntime == nullptr || !cudaKernelRuntime->Ready() ||
        !EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_SOFTAGG", true)) {
        return false;
    }
    std::vector<float> cudaAggKkY;
    std::vector<float> cudaAggIjY;
    std::string cudaSoftAggErr;
    if (!cudaKernelRuntime->ComputeSoftAggExpand(
            {preAgg.aggKkF, preAgg.aggKkG, request.buffers.groupKk,
             iteration.edgeCount, DIM, iteration.step.stream, &cudaAggKkY,
             &cudaSoftAggErr}) ||
        !cudaKernelRuntime->ComputeSoftAggExpand(
            {preAgg.aggIjF, preAgg.aggIjG, request.buffers.groupIj,
             iteration.edgeCount, DIM, iteration.step.stream, &cudaAggIjY,
             &cudaSoftAggErr})) {
        if (!m_loggedCudaSoftAggFailure) {
            std::cerr << "[dpvo_cuda] softagg unavailable: " << cudaSoftAggErr
                      << "; falling back to CPU SoftAgg\n";
            m_loggedCudaSoftAggFailure = true;
        }
        return false;
    }
    ValidateCudaSoftAgg(request, preAgg, cudaAggKkY, cudaAggIjY);
    const size_t dimValues = static_cast<size_t>(iteration.edgeCount) * DIM;
    if (cudaAggKkY.size() != dimValues || cudaAggIjY.size() != dimValues) {
        return false;
    }
    aggKkY.swap(cudaAggKkY);
    aggIjY.swap(cudaAggIjY);
    return true;
}

void DpvoNativeSolver::ValidateCudaSoftAgg(
    const HostUpdateChainRequest &request, const DpvoUpdatePreAggRun &preAgg,
    std::vector<float> &cudaAggKkY, std::vector<float> &cudaAggIjY)
{
    if (m_loggedCudaSoftAgg &&
        !EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_SOFTAGG_VALIDATE_EVERY_STEP",
                        false)) {
        return;
    }
    std::vector<float> cpuAggKkY;
    std::vector<float> cpuAggIjY;
    const StepIterationRequest &iteration = request.iteration;
    SoftAggExpand({preAgg.aggKkF, preAgg.aggKkG, request.buffers.groupKk,
                   iteration.edgeCount, DIM, &cpuAggKkY});
    SoftAggExpand({preAgg.aggIjF, preAgg.aggIjG, request.buffers.groupIj,
                   iteration.edgeCount, DIM, &cpuAggIjY});
    const ErrorStats kkStats = CompareVectors(cpuAggKkY, cudaAggKkY);
    const ErrorStats ijStats = CompareVectors(cpuAggIjY, cudaAggIjY);
    std::cerr << "[dpvo_cuda] softagg ready edges=" << iteration.edgeCount
              << " dim=" << DIM << " kk_max_abs=" << kkStats.maxAbs
              << " kk_rmse=" << kkStats.rmse
              << " ij_max_abs=" << ijStats.maxAbs
              << " ij_rmse=" << ijStats.rmse << "\n";
    m_loggedCudaSoftAgg = true;
    if ((kkStats.maxAbs > 1e-3f || ijStats.maxAbs > 1e-3f) &&
        EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_SOFTAGG_STRICT_VALIDATE", true)) {
        cudaAggKkY = std::move(cpuAggKkY);
        cudaAggIjY = std::move(cpuAggIjY);
        std::cerr << "[dpvo_cuda] softagg validation exceeded "
                     "threshold; using CPU SoftAgg for this step\n";
    }
}

void DpvoNativeSolver::UpdateRecurrentNet(
    const StepIterationRequest &request, const DpvoUpdatePostAggRun &postAgg)
{
    const int patchesPerFrame = request.step.graph.PatchesPerFrame();
    for (int e = 0; e < request.edgeCount; ++e) {
        const DpvoEdgeState &edge = request.edges[static_cast<size_t>(e)];
        const DpvoFrameState &source =
            request.frames[static_cast<size_t>(edge.sourceFrame)];
        const int patchLocal = edge.patchGlobal % patchesPerFrame;
        const DpvoEdgeKey key{
            source.frameId,
            request.frames[static_cast<size_t>(edge.targetFrame)].frameId,
            patchLocal};
        std::vector<float> &slot = m_netByEdge[key];
        slot.assign(
            postAgg.updatedNet.begin() +
                static_cast<std::ptrdiff_t>(static_cast<size_t>(e) * DIM),
            postAgg.updatedNet.begin() +
                static_cast<std::ptrdiff_t>((static_cast<size_t>(e) + 1U) *
                                            DIM));
    }
    PruneEdgeNet(request.frames, request.edges, patchesPerFrame);
}

void DpvoNativeSolver::ApplyPoseUpdate(
    const StepIterationRequest &request, const StepIterationBuffers &buffers,
    const DpvoUpdatePostAggRun &postAgg)
{
    std::vector<std::array<float, 2>> target(
        static_cast<size_t>(request.edgeCount));
    for (int e = 0; e < request.edgeCount; ++e) {
        const std::array<float, PATCH_AREA * 2> &edgeCoords =
            buffers.coords[static_cast<size_t>(e)];
        target[static_cast<size_t>(e)] = {
            edgeCoords[static_cast<size_t>(PATCH_CENTER) * 2U] +
                postAgg.delta[static_cast<size_t>(e) * 2U],
            edgeCoords[static_cast<size_t>(PATCH_CENTER) * 2U + 1U] +
                postAgg.delta[static_cast<size_t>(e) * 2U + 1U]};
    }
    const Sophus::SE3f beforeBaNewest = request.frames.back().Tcw;
    RunBundleAdjustment({request.frames, request.edges,
                         request.step.graph.PatchesPerFrame(),
                         request.step.graph.OptimizationWindow(),
                         request.step.intrinsics, target, postAgg.weight});
    if (!AcceptPoseStep(beforeBaNewest, request.frames.back().Tcw)) {
        request.frames.back().Tcw = beforeBaNewest;
    }
    if (m_bootstrapComplete) {
        request.step.graph.MaybeRemoveKeyframe(request.step.intrinsics);
    }
}

bool DpvoNativeSolver::Step(const DpvoNativeSolverStepRequest &request)
{
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<DpvoFrameState> *frames = nullptr;
    const std::vector<DpvoEdgeState> *edges = nullptr;
    if (!PrepareStepInputs(request, frames, edges)) {
        return false;
    }
    const int updateIterations = UpdateIterationCount();
    for (int updateIteration = 0; updateIteration < updateIterations;
         ++updateIteration) {
        const StepIterationRequest iteration{
            request, *frames, *edges, static_cast<int>(edges->size())};
        StepIterationBuffers buffers;
        InitializeStepIteration(iteration, buffers);
        ComputeStepCorrelation(iteration, buffers);
        DpvoUpdatePostAggRun postAgg{};
        const DeviceUpdateChainRequest deviceRequest{iteration, buffers, postAgg};
        const HostUpdateChainRequest hostRequest{iteration, buffers, postAgg};
        if (!TryRunDeviceUpdateChain(deviceRequest) &&
            !RunHostUpdateChain(hostRequest)) {
            return false;
        }
        UpdateRecurrentNet(iteration, postAgg);
        ApplyPoseUpdate(iteration, buffers, postAgg);
    }
    m_bootstrapComplete = true;
    m_lastTcw = frames->back().Tcw;
    m_hasPose = true;
    if (request.updateMs != nullptr) {
        *request.updateMs = ElapsedMs(t0, std::chrono::steady_clock::now());
    }
    return true;
}

} // namespace DpvoTensorRtInternal

} // namespace SmartDrone::Adapters::Slam
