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

bool DpvoNativeSolver::Step(DpvoGraphState &graph, DpvoUpdatePreAggRuntime &preAggRuntime,
          TensorRtEngineHandle &preAggEngine,
          DpvoUpdatePostAggRuntime &postAggRuntime,
          TensorRtEngineHandle &postAggEngine,
          DpvoCudaKernelRuntime *cudaKernelRuntime, cudaStream_t stream,
          const DpvoIntrinsics &intrinsics, double *updateMs,
          std::string *err)
{
    const auto t0 = std::chrono::steady_clock::now();
    if (updateMs != nullptr) {
        *updateMs = 0.0;
    }
    std::vector<DpvoFrameState> &frames = graph.MutableFrames();
    const std::vector<DpvoEdgeState> &edges = graph.Edges();
    if (frames.empty() || edges.empty()) {
        return false;
    }
    PredictNewestPose(frames);
    if (!graph.Initialized()) {
        m_lastTcw = frames.back().Tcw;
        m_hasPose = true;
        return false;
    }
    if (!graph.FeatureMapsReady()) {
        if (err != nullptr) {
            *err = "native DPVO feature maps are not ready";
        }
        return false;
    }
    if (preAggEngine.Engine() == nullptr || postAggEngine.Engine() == nullptr) {
        if (err != nullptr) {
            *err = "native DPVO requires split preagg/postagg TensorRT engines";
        }
        return false;
    }

    const int bootstrapIterations = std::clamp(
        EnvIntValue("SMART_DRONE_DPVO_BOOTSTRAP_UPDATE_ITERS", 12), 1, 12);
    const int steadyIterations =
        std::clamp(EnvIntValue("SMART_DRONE_DPVO_UPDATE_ITERS", 1), 1, 4);
    const int updateIterations =
        m_bootstrapComplete ? steadyIterations : bootstrapIterations;
    for (int updateIteration = 0; updateIteration < updateIterations;
         ++updateIteration) {
        const int edgeCount = static_cast<int>(edges.size());
        std::vector<int> prevEdge;
        std::vector<int> nextEdge;
        BuildTemporalNeighbors(edges, &prevEdge, &nextEdge);

        std::vector<float> net(static_cast<size_t>(edgeCount) * kDim, 0.0f);
        std::vector<float> inp(static_cast<size_t>(edgeCount) * kDim, 0.0f);
        std::vector<float> corr(static_cast<size_t>(edgeCount) * kCorrDim, 0.0f);
        std::vector<float> prevNet(static_cast<size_t>(edgeCount) * kDim, 0.0f);
        std::vector<float> nextNet(static_cast<size_t>(edgeCount) * kDim, 0.0f);
        std::vector<float> prevMask(static_cast<size_t>(edgeCount), 0.0f);
        std::vector<float> nextMask(static_cast<size_t>(edgeCount), 0.0f);
        std::vector<std::array<float, kPatchArea * 2>> coords(
            static_cast<size_t>(edgeCount));

        for (int e = 0; e < edgeCount; ++e) {
            const DpvoEdgeState &edge = edges[static_cast<size_t>(e)];
            const DpvoFrameState &source =
                frames[static_cast<size_t>(edge.sourceFrame)];
            const int patchLocal = edge.patchGlobal % graph.PatchesPerFrame();
            const DpvoEdgeKey key{
                source.frameId,
                frames[static_cast<size_t>(edge.targetFrame)].frameId, patchLocal};
            auto it = m_netByEdge.find(key);
            if (it != m_netByEdge.end() &&
                it->second.size() == static_cast<size_t>(kDim)) {
                std::copy(it->second.begin(), it->second.end(),
                          net.begin() + static_cast<size_t>(e) * kDim);
            }
            const size_t imapOffset = static_cast<size_t>(patchLocal) * kDim;
            if (source.patchImap.size() >= imapOffset + kDim) {
                std::copy(source.patchImap.begin() +
                              static_cast<std::ptrdiff_t>(imapOffset),
                          source.patchImap.begin() +
                              static_cast<std::ptrdiff_t>(imapOffset + kDim),
                          inp.begin() + static_cast<std::ptrdiff_t>(
                                            static_cast<size_t>(e) * kDim));
            }
            ReprojectPatch(frames, edge, graph.PatchesPerFrame(), intrinsics,
                           coords[static_cast<size_t>(e)]);
        }

        bool usedCudaCorrelation = false;
        if (cudaKernelRuntime != nullptr && cudaKernelRuntime->Ready() &&
            EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_CORR", true)) {
            std::vector<float> edgePatchGmap;
            std::vector<float> edgeCoords;
            std::vector<int> edgeTargetFrame;
            std::vector<float> fmapStorage;
            std::vector<float> fmapLevel4Storage;
            std::vector<int> fmapOffsets;
            std::vector<int> fmapHeights;
            std::vector<int> fmapWidths;
            std::vector<int> level4Offsets;
            std::vector<int> level4Heights;
            std::vector<int> level4Widths;
            if (PackCorrelationCudaInputs(
                    frames, edges, graph.PatchesPerFrame(), coords, &edgePatchGmap,
                    &edgeCoords, &edgeTargetFrame, &fmapStorage, &fmapLevel4Storage,
                    &fmapOffsets, &fmapHeights, &fmapWidths, &level4Offsets,
                    &level4Heights, &level4Widths)) {
                std::vector<float> cudaCorr;
                std::string cudaCorrErr;
                if (cudaKernelRuntime->ComputeCorrelationBatch(
                        edgeCount, edgePatchGmap, edgeCoords, edgeTargetFrame,
                        fmapStorage, fmapLevel4Storage, fmapOffsets, fmapHeights,
                        fmapWidths, level4Offsets, level4Heights, level4Widths,
                        stream, &cudaCorr, &cudaCorrErr)) {
                    if (!m_loggedCudaCorr ||
                        EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_CORR_VALIDATE_EVERY_STEP",
                                       false)) {
                        std::vector<float> cpuCorr(
                            static_cast<size_t>(edgeCount) * kCorrDim, 0.0f);
                        for (int e = 0; e < edgeCount; ++e) {
                            ComputeCorrelation(
                                frames, edges[static_cast<size_t>(e)],
                                graph.PatchesPerFrame(), coords[static_cast<size_t>(e)],
                                cpuCorr.data() + static_cast<size_t>(e) * kCorrDim);
                        }
                        double sq = 0.0;
                        float maxAbs = 0.0f;
                        const size_t n = std::min(cpuCorr.size(), cudaCorr.size());
                        for (size_t i = 0; i < n; ++i) {
                            const float d = std::fabs(cpuCorr[i] - cudaCorr[i]);
                            maxAbs = std::max(maxAbs, d);
                            sq += static_cast<double>(d) * static_cast<double>(d);
                        }
                        const double rmse =
                            n > 0U ? std::sqrt(sq / static_cast<double>(n)) : 0.0;
                        std::cerr << "[dpvo_cuda] correlation batch ready edges="
                                  << edgeCount << " values=" << n << " max_abs=" << maxAbs
                                  << " rmse=" << rmse << "\n";
                        m_loggedCudaCorr = true;
                        if (maxAbs > 1e-3f &&
                            EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_CORR_STRICT_VALIDATE",
                                           true)) {
                            cudaCorr = std::move(cpuCorr);
                            std::cerr << "[dpvo_cuda] correlation validation exceeded "
                                         "threshold; using CPU corr for this step\n";
                        }
                    }
                    if (cudaCorr.size() == corr.size()) {
                        corr.swap(cudaCorr);
                        usedCudaCorrelation = true;
                    }
                } else if (!m_loggedCudaCorrFailure) {
                    std::cerr << "[dpvo_cuda] correlation batch unavailable: "
                              << cudaCorrErr << "; falling back to CPU correlation\n";
                    m_loggedCudaCorrFailure = true;
                }
            } else if (!m_loggedCudaCorrFailure) {
                std::cerr << "[dpvo_cuda] correlation batch input packing failed; "
                             "falling back to CPU correlation\n";
                m_loggedCudaCorrFailure = true;
            }
        }
        if (!usedCudaCorrelation) {
            for (int e = 0; e < edgeCount; ++e) {
                ComputeCorrelation(frames, edges[static_cast<size_t>(e)],
                                   graph.PatchesPerFrame(),
                                   coords[static_cast<size_t>(e)],
                                   corr.data() + static_cast<size_t>(e) * kCorrDim);
            }
        }

        for (int e = 0; e < edgeCount; ++e) {
            if (prevEdge[static_cast<size_t>(e)] >= 0) {
                prevMask[static_cast<size_t>(e)] = 1.0f;
                const size_t src =
                    static_cast<size_t>(prevEdge[static_cast<size_t>(e)]) * kDim;
                std::copy(net.begin() + static_cast<std::ptrdiff_t>(src),
                          net.begin() + static_cast<std::ptrdiff_t>(src + kDim),
                          prevNet.begin() + static_cast<std::ptrdiff_t>(
                                                static_cast<size_t>(e) * kDim));
            }
            if (nextEdge[static_cast<size_t>(e)] >= 0) {
                nextMask[static_cast<size_t>(e)] = 1.0f;
                const size_t src =
                    static_cast<size_t>(nextEdge[static_cast<size_t>(e)]) * kDim;
                std::copy(net.begin() + static_cast<std::ptrdiff_t>(src),
                          net.begin() + static_cast<std::ptrdiff_t>(src + kDim),
                          nextNet.begin() + static_cast<std::ptrdiff_t>(
                                                static_cast<size_t>(e) * kDim));
            }
        }

        std::vector<int> groupKk(static_cast<size_t>(edgeCount), 0);
        std::vector<int> groupIj(static_cast<size_t>(edgeCount), 0);
        for (int e = 0; e < edgeCount; ++e) {
            const DpvoEdgeState &edge = edges[static_cast<size_t>(e)];
            groupKk[static_cast<size_t>(e)] = edge.patchGlobal;
            groupIj[static_cast<size_t>(e)] =
                edge.sourceFrame * 12345 + edge.targetFrame;
        }

        DpvoUpdatePostAggRun postAgg{};
        bool usedDeviceUpdateChain = false;
        if (cudaKernelRuntime != nullptr && cudaKernelRuntime->Ready() &&
            EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_DEVICE_UPDATE_CHAIN", true) &&
            EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_SOFTAGG", true)) {
            std::string deviceErr;
            DpvoUpdatePreAggDeviceRun preAggDevice = preAggRuntime.RunDevice(
                preAggEngine, stream, edgeCount, net, inp, corr, prevNet, nextNet,
                prevMask, nextMask, &deviceErr);
            CudaDeviceBuffer aggKkYDevice;
            CudaDeviceBuffer aggIjYDevice;
            if (preAggDevice.ok &&
                cudaKernelRuntime->ComputeSoftAggExpandDevice(
                    preAggRuntime.AggKkFDevice(), preAggRuntime.AggKkGDevice(),
                    groupKk, edgeCount, kDim, stream, aggKkYDevice, &deviceErr) &&
                cudaKernelRuntime->ComputeSoftAggExpandDevice(
                    preAggRuntime.AggIjFDevice(), preAggRuntime.AggIjGDevice(),
                    groupIj, edgeCount, kDim, stream, aggIjYDevice, &deviceErr)) {
                if (!m_loggedCudaDeviceUpdateChain &&
                    EnvFlagEnabled(
                        "SMART_DRONE_DPVO_CUDA_DEVICE_UPDATE_CHAIN_VALIDATE", true)) {
                    std::vector<float> kkF;
                    std::vector<float> kkG;
                    std::vector<float> ijF;
                    std::vector<float> ijG;
                    std::vector<float> cudaAggKkY;
                    std::vector<float> cudaAggIjY;
                    std::vector<float> cpuAggKkY;
                    std::vector<float> cpuAggIjY;
                    const size_t dimValues = static_cast<size_t>(edgeCount) * kDim;
                    bool validationOk =
                        CopyFloatDeviceBufferToHost(preAggRuntime.AggKkFDevice(),
                                                    dimValues, stream, kkF,
                                                    &deviceErr) &&
                        CopyFloatDeviceBufferToHost(preAggRuntime.AggKkGDevice(),
                                                    dimValues, stream, kkG,
                                                    &deviceErr) &&
                        CopyFloatDeviceBufferToHost(preAggRuntime.AggIjFDevice(),
                                                    dimValues, stream, ijF,
                                                    &deviceErr) &&
                        CopyFloatDeviceBufferToHost(preAggRuntime.AggIjGDevice(),
                                                    dimValues, stream, ijG,
                                                    &deviceErr) &&
                        CopyFloatDeviceBufferToHost(aggKkYDevice, dimValues, stream,
                                                    cudaAggKkY, &deviceErr) &&
                        CopyFloatDeviceBufferToHost(aggIjYDevice, dimValues, stream,
                                                    cudaAggIjY, &deviceErr);
                    if (validationOk) {
                        SoftAggExpand(kkF, kkG, groupKk, edgeCount, kDim, &cpuAggKkY);
                        SoftAggExpand(ijF, ijG, groupIj, edgeCount, kDim, &cpuAggIjY);
                        const ErrorStats kkStats = CompareVectors(cpuAggKkY, cudaAggKkY);
                        const ErrorStats ijStats = CompareVectors(cpuAggIjY, cudaAggIjY);
                        std::cerr
                            << "[dpvo_cuda] device update chain softagg ready edges="
                            << edgeCount << " dim=" << kDim
                            << " kk_max_abs=" << kkStats.maxAbs
                            << " kk_rmse=" << kkStats.rmse
                            << " ij_max_abs=" << ijStats.maxAbs
                            << " ij_rmse=" << ijStats.rmse << "\n";
                        m_loggedCudaDeviceUpdateChain = true;
                        validationOk =
                            (kkStats.maxAbs <= 1e-3f && ijStats.maxAbs <= 1e-3f) ||
                            !EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_DEVICE_UPDATE_CHAIN_"
                                            "STRICT_VALIDATE",
                                            true);
                    }
                    if (!validationOk) {
                        deviceErr = "device update chain validation failed";
                    }
                }
                if (deviceErr.empty()) {
                    postAgg = postAggRuntime.RunDevice(postAggEngine, stream, edgeCount,
                                                       preAggRuntime.BaseNetDevice(),
                                                       aggKkYDevice, aggIjYDevice, err);
                    usedDeviceUpdateChain = postAgg.ok;
                }
            }
            if (!usedDeviceUpdateChain && !m_loggedCudaDeviceUpdateChainFailure) {
                std::cerr << "[dpvo_cuda] device update chain unavailable: "
                          << deviceErr << "; falling back to host update chain\n";
                m_loggedCudaDeviceUpdateChainFailure = true;
            }
        }

        if (!usedDeviceUpdateChain) {
            DpvoUpdatePreAggRun preAgg =
                preAggRuntime.Run(preAggEngine, stream, edgeCount, net, inp, corr,
                                  prevNet, nextNet, prevMask, nextMask, err);
            if (!preAgg.ok) {
                return false;
            }
            std::vector<float> aggKkY;
            std::vector<float> aggIjY;
            bool usedCudaSoftAgg = false;
            if (cudaKernelRuntime != nullptr && cudaKernelRuntime->Ready() &&
                EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_SOFTAGG", true)) {
                std::vector<float> cudaAggKkY;
                std::vector<float> cudaAggIjY;
                std::string cudaSoftAggErr;
                if (cudaKernelRuntime->ComputeSoftAggExpand(
                        preAgg.aggKkF, preAgg.aggKkG, groupKk, edgeCount, kDim,
                        stream, &cudaAggKkY, &cudaSoftAggErr) &&
                    cudaKernelRuntime->ComputeSoftAggExpand(
                        preAgg.aggIjF, preAgg.aggIjG, groupIj, edgeCount, kDim,
                        stream, &cudaAggIjY, &cudaSoftAggErr)) {
                    if (!m_loggedCudaSoftAgg ||
                        EnvFlagEnabled(
                            "SMART_DRONE_DPVO_CUDA_SOFTAGG_VALIDATE_EVERY_STEP",
                            false)) {
                        std::vector<float> cpuAggKkY;
                        std::vector<float> cpuAggIjY;
                        SoftAggExpand(preAgg.aggKkF, preAgg.aggKkG, groupKk, edgeCount,
                                      kDim, &cpuAggKkY);
                        SoftAggExpand(preAgg.aggIjF, preAgg.aggIjG, groupIj, edgeCount,
                                      kDim, &cpuAggIjY);
                        const ErrorStats kkStats = CompareVectors(cpuAggKkY, cudaAggKkY);
                        const ErrorStats ijStats = CompareVectors(cpuAggIjY, cudaAggIjY);
                        std::cerr << "[dpvo_cuda] softagg ready edges=" << edgeCount
                                  << " dim=" << kDim << " kk_max_abs=" << kkStats.maxAbs
                                  << " kk_rmse=" << kkStats.rmse
                                  << " ij_max_abs=" << ijStats.maxAbs
                                  << " ij_rmse=" << ijStats.rmse << "\n";
                        m_loggedCudaSoftAgg = true;
                        if ((kkStats.maxAbs > 1e-3f || ijStats.maxAbs > 1e-3f) &&
                            EnvFlagEnabled(
                                "SMART_DRONE_DPVO_CUDA_SOFTAGG_STRICT_VALIDATE", true)) {
                            cudaAggKkY = std::move(cpuAggKkY);
                            cudaAggIjY = std::move(cpuAggIjY);
                            std::cerr << "[dpvo_cuda] softagg validation exceeded "
                                         "threshold; using CPU SoftAgg for this step\n";
                        }
                    }
                    if (cudaAggKkY.size() == static_cast<size_t>(edgeCount) * kDim &&
                        cudaAggIjY.size() == static_cast<size_t>(edgeCount) * kDim) {
                        aggKkY.swap(cudaAggKkY);
                        aggIjY.swap(cudaAggIjY);
                        usedCudaSoftAgg = true;
                    }
                } else if (!m_loggedCudaSoftAggFailure) {
                    std::cerr << "[dpvo_cuda] softagg unavailable: " << cudaSoftAggErr
                              << "; falling back to CPU SoftAgg\n";
                    m_loggedCudaSoftAggFailure = true;
                }
            }
            if (!usedCudaSoftAgg) {
                SoftAggExpand(preAgg.aggKkF, preAgg.aggKkG, groupKk, edgeCount, kDim,
                              &aggKkY);
                SoftAggExpand(preAgg.aggIjF, preAgg.aggIjG, groupIj, edgeCount, kDim,
                              &aggIjY);
            }

            postAgg = postAggRuntime.Run(postAggEngine, stream, edgeCount,
                                         preAgg.baseNet, aggKkY, aggIjY, err);
        }
        if (!postAgg.ok) {
            return false;
        }

        for (int e = 0; e < edgeCount; ++e) {
            const DpvoEdgeState &edge = edges[static_cast<size_t>(e)];
            const DpvoFrameState &source =
                frames[static_cast<size_t>(edge.sourceFrame)];
            const int patchLocal = edge.patchGlobal % graph.PatchesPerFrame();
            const DpvoEdgeKey key{
                source.frameId,
                frames[static_cast<size_t>(edge.targetFrame)].frameId, patchLocal};
            std::vector<float> &slot = m_netByEdge[key];
            slot.assign(
                postAgg.updatedNet.begin() +
                    static_cast<std::ptrdiff_t>(static_cast<size_t>(e) * kDim),
                postAgg.updatedNet.begin() +
                    static_cast<std::ptrdiff_t>((static_cast<size_t>(e) + 1U) *
                                                kDim));
        }
        PruneEdgeNet(frames, edges, graph.PatchesPerFrame());

        std::vector<std::array<float, 2>> target(static_cast<size_t>(edgeCount));
        for (int e = 0; e < edgeCount; ++e) {
            const std::array<float, kPatchArea * 2> &edgeCoords =
                coords[static_cast<size_t>(e)];
            target[static_cast<size_t>(e)] = {
                edgeCoords[static_cast<size_t>(kPatchCenter) * 2U] +
                    postAgg.delta[static_cast<size_t>(e) * 2U],
                edgeCoords[static_cast<size_t>(kPatchCenter) * 2U + 1U] +
                    postAgg.delta[static_cast<size_t>(e) * 2U + 1U]};
        }
        const Sophus::SE3f beforeBaNewest = frames.back().Tcw;
        RunBundleAdjustment(frames, edges, graph.PatchesPerFrame(),
                            graph.OptimizationWindow(), intrinsics, target,
                            postAgg.weight);
        if (!AcceptPoseStep(beforeBaNewest, frames.back().Tcw)) {
            frames.back().Tcw = beforeBaNewest;
        }
        if (m_bootstrapComplete) {
            graph.MaybeRemoveKeyframe(intrinsics);
        }
    }
    m_bootstrapComplete = true;

    m_lastTcw = frames.back().Tcw;
    m_hasPose = true;
    if (updateMs != nullptr) {
        *updateMs = ElapsedMs(t0, std::chrono::steady_clock::now());
    }
    return true;
}

} // namespace DpvoTensorRtInternal

} // namespace SmartDrone::Adapters::Slam
