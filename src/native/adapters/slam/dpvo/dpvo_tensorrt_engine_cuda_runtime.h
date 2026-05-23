#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <dlfcn.h>
#include <initializer_list>
#include <string>
#include <unordered_map>
#include <vector>

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <nvrtc.h>

#include "adapters/slam/dpvo/dpvo_tensorrt_engine_cuda_buffers.h"
#include "adapters/slam/engine/slam_env.h"

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

class DpvoCudaKernelRuntime {
  public:
    ~DpvoCudaKernelRuntime()
    {
        Reset();
    }

    DpvoCudaKernelRuntime() = default;
    DpvoCudaKernelRuntime(const DpvoCudaKernelRuntime &) = delete;
    DpvoCudaKernelRuntime &operator=(const DpvoCudaKernelRuntime &) = delete;

    bool Initialize(cudaStream_t stream, std::string *err)
    {
        Reset();
        if (!EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_KERNELS", true)) {
            if (err != nullptr) {
                *err = "disabled by SMART_DRONE_DPVO_CUDA_KERNELS";
            }
            return false;
        }
        if (stream == nullptr) {
            if (err != nullptr) {
                *err = "CUDA stream is not initialized";
            }
            return false;
        }
        if (!OpenLibraries(err) || !LoadNvrtcSymbols(err) ||
            !LoadDriverSymbols(err) || !EnsureDriverContext(err) ||
            !CompileAndLoad(err) || !RunCorrelationSmoke(stream, err)) {
            Reset();
            return false;
        }
        m_ready = true;
        return true;
    }

    bool Ready() const
    {
        return m_ready;
    }
    float SmokeExpected() const
    {
        return m_smokeExpected;
    }
    float SmokeGot() const
    {
        return m_smokeGot;
    }

    bool ComputeCorrelationBatch(
        int edgeCount, const std::vector<float> &edgePatchGmap,
        const std::vector<float> &edgeCoords,
        const std::vector<int> &edgeTargetFrame,
        const std::vector<float> &fmapStorage,
        const std::vector<float> &fmapLevel4Storage,
        const std::vector<int> &fmapOffsets, const std::vector<int> &fmapHeights,
        const std::vector<int> &fmapWidths, const std::vector<int> &level4Offsets,
        const std::vector<int> &level4Heights,
        const std::vector<int> &level4Widths, cudaStream_t stream,
        std::vector<float> *outCorr, std::string *err)
    {
        static constexpr int kChannels = 128;
        static constexpr int kPatchArea = 9;
        static constexpr int kCorrRadius = 3;
        static constexpr int kCorrSide = 2 * kCorrRadius + 1;
        static constexpr int kCorrDim = 2 * kCorrSide * kCorrSide * kPatchArea;
        const size_t edgeCountSize = static_cast<size_t>(std::max(0, edgeCount));
        const size_t patchValues = edgeCountSize * kChannels * kPatchArea;
        const size_t coordValues = edgeCountSize * kPatchArea * 2U;
        const size_t corrValues = edgeCountSize * kCorrDim;
        if (!m_ready || m_corrBatchKernel == nullptr) {
            if (err != nullptr) {
                *err = "DPVO CUDA correlation kernel is not ready";
            }
            return false;
        }
        if (edgeCount <= 0 || stream == nullptr || outCorr == nullptr ||
            edgePatchGmap.size() != patchValues ||
            edgeCoords.size() != coordValues ||
            edgeTargetFrame.size() != edgeCountSize ||
            fmapOffsets.size() != fmapHeights.size() ||
            fmapOffsets.size() != fmapWidths.size() ||
            level4Offsets.size() != level4Heights.size() ||
            level4Offsets.size() != level4Widths.size() || fmapStorage.empty() ||
            fmapLevel4Storage.empty()) {
            if (err != nullptr) {
                *err = "invalid DPVO CUDA correlation batch input";
            }
            return false;
        }
        if (!CopyVectorToDevice(edgePatchGmap, m_edgePatchBuffer, stream,
                                "edge patch gmap", err) ||
            !CopyVectorToDevice(edgeCoords, m_edgeCoordsBuffer, stream,
                                "edge coords", err) ||
            !CopyVectorToDevice(edgeTargetFrame, m_edgeTargetFrameBuffer, stream,
                                "edge target frame", err) ||
            !CopyVectorToDevice(fmapStorage, m_fmapBuffer, stream, "fmap", err) ||
            !CopyVectorToDevice(fmapLevel4Storage, m_fmapLevel4Buffer, stream,
                                "fmap level4", err) ||
            !CopyVectorToDevice(fmapOffsets, m_fmapOffsetsBuffer, stream,
                                "fmap offsets", err) ||
            !CopyVectorToDevice(fmapHeights, m_fmapHeightsBuffer, stream,
                                "fmap heights", err) ||
            !CopyVectorToDevice(fmapWidths, m_fmapWidthsBuffer, stream,
                                "fmap widths", err) ||
            !CopyVectorToDevice(level4Offsets, m_level4OffsetsBuffer, stream,
                                "level4 offsets", err) ||
            !CopyVectorToDevice(level4Heights, m_level4HeightsBuffer, stream,
                                "level4 heights", err) ||
            !CopyVectorToDevice(level4Widths, m_level4WidthsBuffer, stream,
                                "level4 widths", err) ||
            !m_corrBuffer.Ensure(corrValues * sizeof(float), err)) {
            return false;
        }
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
        int edgeCountArg = edgeCount;
        void *args[] = {&edgePatchArg, &edgeCoordsArg, &edgeTargetArg,
                        &fmapArg, &fmapLevel4Arg, &fmapOffsetsArg,
                        &fmapHeightsArg, &fmapWidthsArg, &level4OffsetsArg,
                        &level4HeightsArg, &level4WidthsArg, &outArg,
                        &edgeCountArg};
        const unsigned int threads = 128;
        const unsigned int blocks = static_cast<unsigned int>(
            (corrValues + static_cast<size_t>(threads) - 1U) /
            static_cast<size_t>(threads));
        if (!CheckDriver(
                m_cuLaunchKernel(m_corrBatchKernel, blocks, 1, 1, threads, 1, 1, 0,
                                 reinterpret_cast<CUstream>(stream), args, nullptr),
                "cuLaunchKernel(dpvo_corr_batch)", err)) {
            return false;
        }
        outCorr->assign(corrValues, 0.0f);
        cudaError_t rc = cudaMemcpyAsync(outCorr->data(), m_corrBuffer.Data(),
                                         corrValues * sizeof(float),
                                         cudaMemcpyDeviceToHost, stream);
        if (rc == cudaSuccess) {
            rc = cudaStreamSynchronize(stream);
        }
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("DPVO CUDA correlation copy/sync failed: ") +
                       cudaGetErrorString(rc);
            }
            return false;
        }
        return true;
    }

    bool ComputeSoftAggExpand(const std::vector<float> &f,
                              const std::vector<float> &g,
                              const std::vector<int> &groupIds, int edgeCount,
                              int dim, cudaStream_t stream,
                              std::vector<float> *out, std::string *err)
    {
        if (!m_ready || m_softAggKernel == nullptr) {
            if (err != nullptr) {
                *err = "DPVO CUDA SoftAgg kernel is not ready";
            }
            return false;
        }
        if (edgeCount <= 0 || dim <= 0 || stream == nullptr || out == nullptr ||
            f.size() != static_cast<size_t>(edgeCount) * static_cast<size_t>(dim) ||
            g.size() != static_cast<size_t>(edgeCount) * static_cast<size_t>(dim) ||
            groupIds.size() != static_cast<size_t>(edgeCount)) {
            if (err != nullptr) {
                *err = "invalid DPVO CUDA SoftAgg input";
            }
            return false;
        }

        std::unordered_map<int, int> groupToDense;
        groupToDense.reserve(static_cast<size_t>(edgeCount));
        std::vector<std::vector<int>> groups;
        groups.reserve(static_cast<size_t>(edgeCount));
        for (int e = 0; e < edgeCount; ++e) {
            const int id = groupIds[static_cast<size_t>(e)];
            auto it = groupToDense.find(id);
            if (it == groupToDense.end()) {
                const int dense = static_cast<int>(groups.size());
                it = groupToDense.emplace(id, dense).first;
                groups.emplace_back();
            }
            groups[static_cast<size_t>(it->second)].push_back(e);
        }
        if (groups.empty()) {
            if (err != nullptr) {
                *err = "DPVO CUDA SoftAgg has no groups";
            }
            return false;
        }

        std::vector<int> groupStarts(groups.size() + 1U, 0);
        std::vector<int> groupIndices;
        groupIndices.reserve(static_cast<size_t>(edgeCount));
        for (size_t i = 0; i < groups.size(); ++i) {
            groupStarts[i] = static_cast<int>(groupIndices.size());
            groupIndices.insert(groupIndices.end(), groups[i].begin(),
                                groups[i].end());
        }
        groupStarts[groups.size()] = static_cast<int>(groupIndices.size());

        const size_t values =
            static_cast<size_t>(edgeCount) * static_cast<size_t>(dim);
        if (!CopyVectorToDevice(f, m_softAggFBuffer, stream, "softagg f", err) ||
            !CopyVectorToDevice(g, m_softAggGBuffer, stream, "softagg g", err) ||
            !CopyVectorToDevice(groupIndices, m_softAggGroupIndexBuffer, stream,
                                "softagg group indices", err) ||
            !CopyVectorToDevice(groupStarts, m_softAggGroupStartBuffer, stream,
                                "softagg group starts", err) ||
            !m_softAggOutBuffer.Ensure(values * sizeof(float), err)) {
            return false;
        }

        CUdeviceptr fArg = reinterpret_cast<CUdeviceptr>(m_softAggFBuffer.Data());
        CUdeviceptr gArg = reinterpret_cast<CUdeviceptr>(m_softAggGBuffer.Data());
        CUdeviceptr groupIndexArg =
            reinterpret_cast<CUdeviceptr>(m_softAggGroupIndexBuffer.Data());
        CUdeviceptr groupStartArg =
            reinterpret_cast<CUdeviceptr>(m_softAggGroupStartBuffer.Data());
        CUdeviceptr outArg =
            reinterpret_cast<CUdeviceptr>(m_softAggOutBuffer.Data());
        int groupCountArg = static_cast<int>(groups.size());
        int dimArg = dim;
        void *args[] = {&fArg, &gArg, &groupIndexArg, &groupStartArg,
                        &outArg, &groupCountArg, &dimArg};
        const unsigned int threads = 128;
        const size_t workItems = groups.size() * static_cast<size_t>(dim);
        const unsigned int blocks = static_cast<unsigned int>(
            (workItems + static_cast<size_t>(threads) - 1U) /
            static_cast<size_t>(threads));
        if (!CheckDriver(m_cuLaunchKernel(m_softAggKernel, blocks, 1, 1, threads, 1,
                                          1, 0, reinterpret_cast<CUstream>(stream),
                                          args, nullptr),
                         "cuLaunchKernel(dpvo_softagg_expand)", err)) {
            return false;
        }
        out->assign(values, 0.0f);
        cudaError_t rc =
            cudaMemcpyAsync(out->data(), m_softAggOutBuffer.Data(),
                            values * sizeof(float), cudaMemcpyDeviceToHost, stream);
        if (rc == cudaSuccess) {
            rc = cudaStreamSynchronize(stream);
        }
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("DPVO CUDA SoftAgg copy/sync failed: ") +
                       cudaGetErrorString(rc);
            }
            return false;
        }
        return true;
    }

    bool ComputeSoftAggExpandDevice(const CudaDeviceBuffer &fDevice,
                                    const CudaDeviceBuffer &gDevice,
                                    const std::vector<int> &groupIds,
                                    int edgeCount, int dim, cudaStream_t stream,
                                    CudaDeviceBuffer &outDevice,
                                    std::string *err)
    {
        if (!m_ready || m_softAggKernel == nullptr) {
            if (err != nullptr) {
                *err = "DPVO CUDA SoftAgg kernel is not ready";
            }
            return false;
        }
        const size_t values = static_cast<size_t>(std::max(0, edgeCount)) *
                              static_cast<size_t>(std::max(0, dim));
        if (edgeCount <= 0 || dim <= 0 || stream == nullptr ||
            fDevice.Bytes() < values * sizeof(float) ||
            gDevice.Bytes() < values * sizeof(float) ||
            groupIds.size() != static_cast<size_t>(edgeCount)) {
            if (err != nullptr) {
                *err = "invalid DPVO CUDA SoftAgg device input";
            }
            return false;
        }

        std::unordered_map<int, int> groupToDense;
        groupToDense.reserve(static_cast<size_t>(edgeCount));
        std::vector<std::vector<int>> groups;
        groups.reserve(static_cast<size_t>(edgeCount));
        for (int e = 0; e < edgeCount; ++e) {
            const int id = groupIds[static_cast<size_t>(e)];
            auto it = groupToDense.find(id);
            if (it == groupToDense.end()) {
                const int dense = static_cast<int>(groups.size());
                it = groupToDense.emplace(id, dense).first;
                groups.emplace_back();
            }
            groups[static_cast<size_t>(it->second)].push_back(e);
        }
        if (groups.empty()) {
            if (err != nullptr) {
                *err = "DPVO CUDA SoftAgg device path has no groups";
            }
            return false;
        }

        std::vector<int> groupStarts(groups.size() + 1U, 0);
        std::vector<int> groupIndices;
        groupIndices.reserve(static_cast<size_t>(edgeCount));
        for (size_t i = 0; i < groups.size(); ++i) {
            groupStarts[i] = static_cast<int>(groupIndices.size());
            groupIndices.insert(groupIndices.end(), groups[i].begin(),
                                groups[i].end());
        }
        groupStarts[groups.size()] = static_cast<int>(groupIndices.size());

        if (!CopyVectorToDevice(groupIndices, m_softAggGroupIndexBuffer, stream,
                                "softagg group indices", err) ||
            !CopyVectorToDevice(groupStarts, m_softAggGroupStartBuffer, stream,
                                "softagg group starts", err) ||
            !outDevice.Ensure(values * sizeof(float), err)) {
            return false;
        }

        CUdeviceptr fArg = reinterpret_cast<CUdeviceptr>(fDevice.Data());
        CUdeviceptr gArg = reinterpret_cast<CUdeviceptr>(gDevice.Data());
        CUdeviceptr groupIndexArg =
            reinterpret_cast<CUdeviceptr>(m_softAggGroupIndexBuffer.Data());
        CUdeviceptr groupStartArg =
            reinterpret_cast<CUdeviceptr>(m_softAggGroupStartBuffer.Data());
        CUdeviceptr outArg = reinterpret_cast<CUdeviceptr>(outDevice.Data());
        int groupCountArg = static_cast<int>(groups.size());
        int dimArg = dim;
        void *args[] = {&fArg, &gArg, &groupIndexArg, &groupStartArg,
                        &outArg, &groupCountArg, &dimArg};
        const unsigned int threads = 128;
        const size_t workItems = groups.size() * static_cast<size_t>(dim);
        const unsigned int blocks = static_cast<unsigned int>(
            (workItems + static_cast<size_t>(threads) - 1U) /
            static_cast<size_t>(threads));
        return CheckDriver(
            m_cuLaunchKernel(m_softAggKernel, blocks, 1, 1, threads, 1, 1, 0,
                             reinterpret_cast<CUstream>(stream), args, nullptr),
            "cuLaunchKernel(dpvo_softagg_expand device)", err);
    }

  private:
    template <typename T>
    bool LoadSymbol(void *handle, const char *name, T *out, std::string *err)
    {
        if (handle == nullptr || name == nullptr || out == nullptr) {
            return false;
        }
        dlerror();
        void *symbol = dlsym(handle, name);
        const char *dlErr = dlerror();
        if (dlErr != nullptr || symbol == nullptr) {
            if (err != nullptr) {
                *err = std::string("missing CUDA symbol ") + name + ": " +
                       (dlErr != nullptr ? dlErr : "not found");
            }
            return false;
        }
        *out = reinterpret_cast<T>(symbol);
        return true;
    }

    template <typename T>
    bool CopyVectorToDevice(const std::vector<T> &src, CudaDeviceBuffer &dst,
                            cudaStream_t stream, const char *name,
                            std::string *err)
    {
        if (src.empty()) {
            if (err != nullptr) {
                *err = std::string("empty DPVO CUDA buffer: ") +
                       (name != nullptr ? name : "unnamed");
            }
            return false;
        }
        const size_t bytes = src.size() * sizeof(T);
        if (!dst.Ensure(bytes, err)) {
            return false;
        }
        const cudaError_t rc = cudaMemcpyAsync(dst.Data(), src.data(), bytes,
                                               cudaMemcpyHostToDevice, stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("cudaMemcpyAsync failed for DPVO CUDA buffer ") +
                       (name != nullptr ? name : "unnamed") + ": " +
                       cudaGetErrorString(rc);
            }
            return false;
        }
        return true;
    }

    bool OpenLibraryAny(std::initializer_list<const char *> names, void **handle,
                        std::string *err);
    bool OpenLibraries(std::string *err);
    bool LoadNvrtcSymbols(std::string *err);
    bool LoadDriverSymbols(std::string *err);
    const char *DriverError(CUresult result) const;
    bool CheckDriver(CUresult result, const char *what, std::string *err) const;
    bool CheckNvrtc(nvrtcResult result, const char *what, std::string *err) const;
    bool EnsureDriverContext(std::string *err);
    bool CompileAndLoad(std::string *err);
    bool RunCorrelationSmoke(cudaStream_t stream, std::string *err);
    void Reset();

    void *m_nvrtcLib{nullptr};
    void *m_cudaLib{nullptr};
    CUmodule m_module{nullptr};
    CUfunction m_corrSmokeKernel{nullptr};
    CUfunction m_corrBatchKernel{nullptr};
    CUfunction m_softAggKernel{nullptr};
    bool m_ready{false};
    float m_smokeExpected{0.0f};
    float m_smokeGot{0.0f};

    nvrtcResult (*m_nvrtcCreateProgram)(nvrtcProgram *, const char *,
                                        const char *, int, const char *const *,
                                        const char *const *){nullptr};
    nvrtcResult (*m_nvrtcCompileProgram)(nvrtcProgram, int,
                                         const char *const *){nullptr};
    nvrtcResult (*m_nvrtcGetPTXSize)(nvrtcProgram, size_t *){nullptr};
    nvrtcResult (*m_nvrtcGetPTX)(nvrtcProgram, char *){nullptr};
    nvrtcResult (*m_nvrtcGetProgramLogSize)(nvrtcProgram, size_t *){nullptr};
    nvrtcResult (*m_nvrtcGetProgramLog)(nvrtcProgram, char *){nullptr};
    nvrtcResult (*m_nvrtcDestroyProgram)(nvrtcProgram *){nullptr};
    const char *(*m_nvrtcGetErrorString)(nvrtcResult){nullptr};

    CUresult (*m_cuInit)(unsigned int){nullptr};
    CUresult (*m_cuDeviceGet)(CUdevice *, int){nullptr};
    CUresult (*m_cuCtxCreate)(CUcontext *, unsigned int, CUdevice){nullptr};
    CUresult (*m_cuCtxGetCurrent)(CUcontext *){nullptr};
    CUresult (*m_cuModuleLoadData)(CUmodule *, const void *){nullptr};
    CUresult (*m_cuModuleGetFunction)(CUfunction *, CUmodule,
                                      const char *){nullptr};
    CUresult (*m_cuLaunchKernel)(CUfunction, unsigned int, unsigned int,
                                 unsigned int, unsigned int, unsigned int,
                                 unsigned int, unsigned int, CUstream, void **,
                                 void **){nullptr};
    CUresult (*m_cuModuleUnload)(CUmodule){nullptr};
    CUresult (*m_cuGetErrorString)(CUresult, const char **){nullptr};

    CudaDeviceBuffer m_edgePatchBuffer;
    CudaDeviceBuffer m_edgeCoordsBuffer;
    CudaDeviceBuffer m_edgeTargetFrameBuffer;
    CudaDeviceBuffer m_fmapBuffer;
    CudaDeviceBuffer m_fmapLevel4Buffer;
    CudaDeviceBuffer m_fmapOffsetsBuffer;
    CudaDeviceBuffer m_fmapHeightsBuffer;
    CudaDeviceBuffer m_fmapWidthsBuffer;
    CudaDeviceBuffer m_level4OffsetsBuffer;
    CudaDeviceBuffer m_level4HeightsBuffer;
    CudaDeviceBuffer m_level4WidthsBuffer;
    CudaDeviceBuffer m_corrBuffer;
    CudaDeviceBuffer m_softAggFBuffer;
    CudaDeviceBuffer m_softAggGBuffer;
    CudaDeviceBuffer m_softAggGroupIndexBuffer;
    CudaDeviceBuffer m_softAggGroupStartBuffer;
    CudaDeviceBuffer m_softAggOutBuffer;
};

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
