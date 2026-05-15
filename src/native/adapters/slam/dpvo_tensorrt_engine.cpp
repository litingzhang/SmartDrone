#include "adapters/slam/dpvo_tensorrt_engine.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <sophus/se3.hpp>

#include "Tracking.h"
#include "adapters/slam/slam_mode_common.h"
#include "adapters/slam/slam_mode_state.h"

#if defined(SMART_DRONE_DPVO_TENSORRT_AVAILABLE)
#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <dlfcn.h>
#include <nvrtc.h>
#endif

namespace smartdrone::adapters::slam {

namespace {

std::filesystem::path ResolveEnginePath(const std::string &explicitPath, const std::string &repoPath,
                                        const std::vector<std::string> &names)
{
    if (!explicitPath.empty() && std::filesystem::exists(explicitPath)) {
        return std::filesystem::path(explicitPath);
    }
    const std::filesystem::path repo(repoPath);
    if (!repo.empty()) {
        for (const std::string &name : names) {
            const std::filesystem::path candidate = repo / "weights" / name;
            if (std::filesystem::exists(candidate)) {
                return candidate;
            }
        }
    }
    return {};
}

double ElapsedMs(const std::chrono::steady_clock::time_point &start,
                 const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

bool TrackingStateCanPublishPose(int trackingState)
{
    return trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::RECENTLY_LOST ||
           trackingState == ORB_SLAM3::Tracking::OK_KLT;
}

core::ports::PoseEstimate PoseFromTwc(const Sophus::SE3f &twc)
{
    core::ports::PoseEstimate pose{};
    const Eigen::Vector3f t = twc.translation();
    const Eigen::Quaternionf q(twc.so3().unit_quaternion());
    pose.valid = std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z()) &&
                 std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z());
    pose.x = t.x();
    pose.y = t.y();
    pose.z = t.z();
    pose.qw = q.w();
    pose.qx = q.x();
    pose.qy = q.y();
    pose.qz = q.z();
    return pose;
}

} // namespace

#if defined(SMART_DRONE_DPVO_TENSORRT_AVAILABLE)

namespace {

class TensorRtLogger final : public nvinfer1::ILogger {
  public:
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kWARNING) {
            std::cerr << "[dpvo_trt] " << (msg != nullptr ? msg : "") << "\n";
        }
    }
};

struct DestroyRuntime {
    void operator()(nvinfer1::IRuntime *ptr) const
    {
        if (ptr != nullptr) {
            ptr->destroy();
        }
    }
};

struct DestroyEngine {
    void operator()(nvinfer1::ICudaEngine *ptr) const
    {
        if (ptr != nullptr) {
            ptr->destroy();
        }
    }
};

struct DestroyContext {
    void operator()(nvinfer1::IExecutionContext *ptr) const
    {
        if (ptr != nullptr) {
            ptr->destroy();
        }
    }
};

class TensorRtEngineHandle {
  public:
    bool Load(const std::filesystem::path &enginePath, const char *name, std::string *err)
    {
        std::ifstream in(enginePath, std::ios::binary);
        if (!in) {
            if (err != nullptr) {
                *err = std::string(name) + " TensorRT engine not found: " + enginePath.string();
            }
            return false;
        }
        std::vector<char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        if (bytes.empty()) {
            if (err != nullptr) {
                *err = std::string(name) + " TensorRT engine is empty: " + enginePath.string();
            }
            return false;
        }

        initLibNvInferPlugins(&m_logger, "");
        m_runtime.reset(nvinfer1::createInferRuntime(m_logger));
        if (!m_runtime) {
            if (err != nullptr) {
                *err = std::string("failed to create ") + name + " TensorRT runtime";
            }
            return false;
        }
        m_engine.reset(m_runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
        if (!m_engine) {
            if (err != nullptr) {
                *err = std::string("failed to deserialize ") + name + " TensorRT engine: " + enginePath.string();
            }
            return false;
        }
        m_context.reset(m_engine->createExecutionContext());
        if (!m_context) {
            if (err != nullptr) {
                *err = std::string("failed to create ") + name + " TensorRT execution context";
            }
            return false;
        }
        m_path = enginePath.string();
        return true;
    }

    bool Loaded() const { return static_cast<bool>(m_engine) && static_cast<bool>(m_context); }
    const std::string &Path() const { return m_path; }
    nvinfer1::ICudaEngine *Engine() const { return m_engine.get(); }
    nvinfer1::IExecutionContext *Context() const { return m_context.get(); }

  private:
    TensorRtLogger m_logger{};
    std::unique_ptr<nvinfer1::IRuntime, DestroyRuntime> m_runtime;
    std::unique_ptr<nvinfer1::ICudaEngine, DestroyEngine> m_engine;
    std::unique_ptr<nvinfer1::IExecutionContext, DestroyContext> m_context;
    std::string m_path;
};

struct CudaStreamHandle {
    cudaStream_t stream{nullptr};

    bool Create(std::string *err)
    {
        if (stream != nullptr) {
            return true;
        }
        const cudaError_t rc = cudaStreamCreate(&stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("cudaStreamCreate failed: ") + cudaGetErrorString(rc);
            }
            stream = nullptr;
            return false;
        }
        return true;
    }

    void Reset()
    {
        if (stream != nullptr) {
            cudaStreamDestroy(stream);
            stream = nullptr;
        }
    }

    ~CudaStreamHandle() { Reset(); }
};

class CudaDeviceBuffer {
  public:
    ~CudaDeviceBuffer() { Reset(); }

    CudaDeviceBuffer() = default;
    CudaDeviceBuffer(const CudaDeviceBuffer &) = delete;
    CudaDeviceBuffer &operator=(const CudaDeviceBuffer &) = delete;

    bool Ensure(size_t bytes, std::string *err)
    {
        if (bytes <= m_bytes) {
            return true;
        }
        Reset();
        const cudaError_t rc = cudaMalloc(&m_ptr, bytes);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("cudaMalloc failed bytes=") + std::to_string(bytes) + ": " +
                       cudaGetErrorString(rc);
            }
            m_ptr = nullptr;
            m_bytes = 0;
            return false;
        }
        m_bytes = bytes;
        return true;
    }

    void Reset()
    {
        if (m_ptr != nullptr) {
            cudaFree(m_ptr);
            m_ptr = nullptr;
        }
        m_bytes = 0;
    }

    void *Data() const { return m_ptr; }
    size_t Bytes() const { return m_bytes; }

  private:
    void *m_ptr{nullptr};
    size_t m_bytes{0};
};

class DpvoCudaKernelRuntime {
  public:
    ~DpvoCudaKernelRuntime() { Reset(); }

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
        if (!OpenLibraries(err) || !LoadNvrtcSymbols(err) || !LoadDriverSymbols(err) ||
            !EnsureDriverContext(err) || !CompileAndLoad(err) || !RunCorrelationSmoke(stream, err)) {
            Reset();
            return false;
        }
        m_ready = true;
        return true;
    }

    bool Ready() const { return m_ready; }
    float SmokeExpected() const { return m_smokeExpected; }
    float SmokeGot() const { return m_smokeGot; }

    bool ComputeCorrelationBatch(int edgeCount, const std::vector<float> &edgePatchGmap,
                                 const std::vector<float> &edgeCoords,
                                 const std::vector<int> &edgeTargetFrame,
                                 const std::vector<float> &fmapStorage,
                                 const std::vector<float> &fmapLevel4Storage,
                                 const std::vector<int> &fmapOffsets,
                                 const std::vector<int> &fmapHeights,
                                 const std::vector<int> &fmapWidths,
                                 const std::vector<int> &level4Offsets,
                                 const std::vector<int> &level4Heights,
                                 const std::vector<int> &level4Widths,
                                 cudaStream_t stream, std::vector<float> *outCorr,
                                 std::string *err)
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
            edgePatchGmap.size() != patchValues || edgeCoords.size() != coordValues ||
            edgeTargetFrame.size() != edgeCountSize || fmapOffsets.size() != fmapHeights.size() ||
            fmapOffsets.size() != fmapWidths.size() || level4Offsets.size() != level4Heights.size() ||
            level4Offsets.size() != level4Widths.size() || fmapStorage.empty() || fmapLevel4Storage.empty()) {
            if (err != nullptr) {
                *err = "invalid DPVO CUDA correlation batch input";
            }
            return false;
        }
        if (!CopyVectorToDevice(edgePatchGmap, m_edgePatchBuffer, stream, "edge patch gmap", err) ||
            !CopyVectorToDevice(edgeCoords, m_edgeCoordsBuffer, stream, "edge coords", err) ||
            !CopyVectorToDevice(edgeTargetFrame, m_edgeTargetFrameBuffer, stream, "edge target frame", err) ||
            !CopyVectorToDevice(fmapStorage, m_fmapBuffer, stream, "fmap", err) ||
            !CopyVectorToDevice(fmapLevel4Storage, m_fmapLevel4Buffer, stream, "fmap level4", err) ||
            !CopyVectorToDevice(fmapOffsets, m_fmapOffsetsBuffer, stream, "fmap offsets", err) ||
            !CopyVectorToDevice(fmapHeights, m_fmapHeightsBuffer, stream, "fmap heights", err) ||
            !CopyVectorToDevice(fmapWidths, m_fmapWidthsBuffer, stream, "fmap widths", err) ||
            !CopyVectorToDevice(level4Offsets, m_level4OffsetsBuffer, stream, "level4 offsets", err) ||
            !CopyVectorToDevice(level4Heights, m_level4HeightsBuffer, stream, "level4 heights", err) ||
            !CopyVectorToDevice(level4Widths, m_level4WidthsBuffer, stream, "level4 widths", err) ||
            !m_corrBuffer.Ensure(corrValues * sizeof(float), err)) {
            return false;
        }
        CUdeviceptr edgePatchArg = reinterpret_cast<CUdeviceptr>(m_edgePatchBuffer.Data());
        CUdeviceptr edgeCoordsArg = reinterpret_cast<CUdeviceptr>(m_edgeCoordsBuffer.Data());
        CUdeviceptr edgeTargetArg = reinterpret_cast<CUdeviceptr>(m_edgeTargetFrameBuffer.Data());
        CUdeviceptr fmapArg = reinterpret_cast<CUdeviceptr>(m_fmapBuffer.Data());
        CUdeviceptr fmapLevel4Arg = reinterpret_cast<CUdeviceptr>(m_fmapLevel4Buffer.Data());
        CUdeviceptr fmapOffsetsArg = reinterpret_cast<CUdeviceptr>(m_fmapOffsetsBuffer.Data());
        CUdeviceptr fmapHeightsArg = reinterpret_cast<CUdeviceptr>(m_fmapHeightsBuffer.Data());
        CUdeviceptr fmapWidthsArg = reinterpret_cast<CUdeviceptr>(m_fmapWidthsBuffer.Data());
        CUdeviceptr level4OffsetsArg = reinterpret_cast<CUdeviceptr>(m_level4OffsetsBuffer.Data());
        CUdeviceptr level4HeightsArg = reinterpret_cast<CUdeviceptr>(m_level4HeightsBuffer.Data());
        CUdeviceptr level4WidthsArg = reinterpret_cast<CUdeviceptr>(m_level4WidthsBuffer.Data());
        CUdeviceptr outArg = reinterpret_cast<CUdeviceptr>(m_corrBuffer.Data());
        int edgeCountArg = edgeCount;
        void *args[] = {&edgePatchArg,     &edgeCoordsArg,      &edgeTargetArg,    &fmapArg,
                        &fmapLevel4Arg,    &fmapOffsetsArg,     &fmapHeightsArg,   &fmapWidthsArg,
                        &level4OffsetsArg, &level4HeightsArg,   &level4WidthsArg,  &outArg,
                        &edgeCountArg};
        const unsigned int threads = 128;
        const unsigned int blocks =
            static_cast<unsigned int>((corrValues + static_cast<size_t>(threads) - 1U) /
                                      static_cast<size_t>(threads));
        if (!CheckDriver(m_cuLaunchKernel(m_corrBatchKernel, blocks, 1, 1, threads, 1, 1, 0,
                                          reinterpret_cast<CUstream>(stream), args, nullptr),
                         "cuLaunchKernel(dpvo_corr_batch)", err)) {
            return false;
        }
        outCorr->assign(corrValues, 0.0f);
        cudaError_t rc = cudaMemcpyAsync(outCorr->data(), m_corrBuffer.Data(), corrValues * sizeof(float),
                                         cudaMemcpyDeviceToHost, stream);
        if (rc == cudaSuccess) {
            rc = cudaStreamSynchronize(stream);
        }
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("DPVO CUDA correlation copy/sync failed: ") + cudaGetErrorString(rc);
            }
            return false;
        }
        return true;
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
                *err = std::string("missing CUDA symbol ") + name + ": " + (dlErr != nullptr ? dlErr : "not found");
            }
            return false;
        }
        *out = reinterpret_cast<T>(symbol);
        return true;
    }

    bool OpenLibraryAny(std::initializer_list<const char *> names, void **handle, std::string *err)
    {
        if (handle == nullptr) {
            return false;
        }
        for (const char *name : names) {
            if (name == nullptr) {
                continue;
            }
            void *h = dlopen(name, RTLD_NOW | RTLD_LOCAL);
            if (h != nullptr) {
                *handle = h;
                return true;
            }
        }
        if (err != nullptr) {
            *err = "failed to open CUDA/NVRTC shared library";
        }
        return false;
    }

    bool OpenLibraries(std::string *err)
    {
        return OpenLibraryAny({"libnvrtc.so",
                               "libnvrtc.so.11.4",
                               "libnvrtc.so.11.2",
                               "/usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvrtc.so",
                               "/usr/local/cuda/targets/aarch64-linux/lib/libnvrtc.so"},
                              &m_nvrtcLib, err) &&
               OpenLibraryAny({"libcuda.so",
                               "libcuda.so.1",
                               "/usr/lib/aarch64-linux-gnu/tegra/libcuda.so",
                               "/usr/lib/aarch64-linux-gnu/libcuda.so"},
                              &m_cudaLib, err);
    }

    bool LoadNvrtcSymbols(std::string *err)
    {
        return LoadSymbol(m_nvrtcLib, "nvrtcCreateProgram", &m_nvrtcCreateProgram, err) &&
               LoadSymbol(m_nvrtcLib, "nvrtcCompileProgram", &m_nvrtcCompileProgram, err) &&
               LoadSymbol(m_nvrtcLib, "nvrtcGetPTXSize", &m_nvrtcGetPTXSize, err) &&
               LoadSymbol(m_nvrtcLib, "nvrtcGetPTX", &m_nvrtcGetPTX, err) &&
               LoadSymbol(m_nvrtcLib, "nvrtcGetProgramLogSize", &m_nvrtcGetProgramLogSize, err) &&
               LoadSymbol(m_nvrtcLib, "nvrtcGetProgramLog", &m_nvrtcGetProgramLog, err) &&
               LoadSymbol(m_nvrtcLib, "nvrtcDestroyProgram", &m_nvrtcDestroyProgram, err) &&
               LoadSymbol(m_nvrtcLib, "nvrtcGetErrorString", &m_nvrtcGetErrorString, err);
    }

    bool LoadDriverSymbols(std::string *err)
    {
        return LoadSymbol(m_cudaLib, "cuInit", &m_cuInit, err) &&
               LoadSymbol(m_cudaLib, "cuDeviceGet", &m_cuDeviceGet, err) &&
               LoadSymbol(m_cudaLib, "cuCtxCreate_v2", &m_cuCtxCreate, err) &&
               LoadSymbol(m_cudaLib, "cuCtxGetCurrent", &m_cuCtxGetCurrent, err) &&
               LoadSymbol(m_cudaLib, "cuModuleLoadData", &m_cuModuleLoadData, err) &&
               LoadSymbol(m_cudaLib, "cuModuleGetFunction", &m_cuModuleGetFunction, err) &&
               LoadSymbol(m_cudaLib, "cuLaunchKernel", &m_cuLaunchKernel, err) &&
               LoadSymbol(m_cudaLib, "cuModuleUnload", &m_cuModuleUnload, err) &&
               LoadSymbol(m_cudaLib, "cuGetErrorString", &m_cuGetErrorString, err);
    }

    const char *DriverError(CUresult result) const
    {
        const char *text = nullptr;
        if (m_cuGetErrorString != nullptr && m_cuGetErrorString(result, &text) == CUDA_SUCCESS && text != nullptr) {
            return text;
        }
        return "unknown CUDA driver error";
    }

    bool CheckDriver(CUresult result, const char *what, std::string *err) const
    {
        if (result == CUDA_SUCCESS) {
            return true;
        }
        if (err != nullptr) {
            *err = std::string(what != nullptr ? what : "CUDA driver call") + " failed: " + DriverError(result);
        }
        return false;
    }

    bool CheckNvrtc(nvrtcResult result, const char *what, std::string *err) const
    {
        if (result == NVRTC_SUCCESS) {
            return true;
        }
        if (err != nullptr) {
            const char *text = m_nvrtcGetErrorString != nullptr ? m_nvrtcGetErrorString(result) : "unknown NVRTC error";
            *err = std::string(what != nullptr ? what : "NVRTC call") + " failed: " + (text != nullptr ? text : "");
        }
        return false;
    }

    bool EnsureDriverContext(std::string *err)
    {
        if (!CheckDriver(m_cuInit(0), "cuInit", err)) {
            return false;
        }
        const cudaError_t runtimeRc = cudaFree(nullptr);
        if (runtimeRc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("cudaFree(0) context initialization failed: ") +
                       cudaGetErrorString(runtimeRc);
            }
            return false;
        }
        CUcontext current = nullptr;
        if (!CheckDriver(m_cuCtxGetCurrent(&current), "cuCtxGetCurrent", err)) {
            return false;
        }
        if (current != nullptr) {
            m_context = current;
            m_ownsContext = false;
            return true;
        }
        if (err != nullptr) {
            *err = "no current CUDA context after cudaFree(0)";
        }
        return false;
    }

    bool CompileAndLoad(std::string *err)
    {
        static constexpr const char *kSource = R"CUDA(
extern "C" __global__ void dpvo_corr_patch3_smoke(
    const float *patch,
    const float *fmap,
    float *out,
    int channels,
    int height,
    int width,
    int x,
    int y)
{
    extern __shared__ float scratch[];
    const int tid = threadIdx.x;
    float acc = 0.0f;
    const int total = channels * 9;
    for (int idx = tid; idx < total; idx += blockDim.x) {
        const int p = idx % 9;
        const int c = idx / 9;
        const int py = p / 3;
        const int px = p - py * 3;
        const int yy = y + py - 1;
        const int xx = x + px - 1;
        float b = 0.0f;
        if (yy >= 0 && yy < height && xx >= 0 && xx < width) {
            b = fmap[(c * height + yy) * width + xx];
        }
        acc += patch[c * 9 + p] * b;
    }
    scratch[tid] = acc;
    __syncthreads();
    for (int stride = blockDim.x >> 1; stride > 0; stride >>= 1) {
        if (tid < stride) {
            scratch[tid] += scratch[tid + stride];
        }
        __syncthreads();
    }
    if (tid == 0) {
        out[0] = scratch[0];
    }
}

static __device__ float dpvo_feature_at(
    const float *data,
    int offset,
    int channels,
    int height,
    int width,
    int c,
    int y,
    int x)
{
    if (data == 0 || c < 0 || c >= channels || y < 0 || y >= height || x < 0 || x >= width) {
        return 0.0f;
    }
    return data[offset + (c * height + y) * width + x];
}

static __device__ float dpvo_sample_bilinear(
    const float *data,
    int offset,
    int channels,
    int height,
    int width,
    int c,
    float x,
    float y)
{
    if (data == 0 || channels <= 0 || height <= 0 || width <= 0) {
        return 0.0f;
    }
    const int x0 = static_cast<int>(floorf(x));
    const int y0 = static_cast<int>(floorf(y));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;
    const float dx = x - static_cast<float>(x0);
    const float dy = y - static_cast<float>(y0);
    const float v00 = dpvo_feature_at(data, offset, channels, height, width, c, y0, x0);
    const float v01 = dpvo_feature_at(data, offset, channels, height, width, c, y0, x1);
    const float v10 = dpvo_feature_at(data, offset, channels, height, width, c, y1, x0);
    const float v11 = dpvo_feature_at(data, offset, channels, height, width, c, y1, x1);
    return (1.0f - dy) * ((1.0f - dx) * v00 + dx * v01) +
           dy * ((1.0f - dx) * v10 + dx * v11);
}

extern "C" __global__ void dpvo_corr_batch(
    const float *edge_patch_gmap,
    const float *edge_coords,
    const int *edge_target_frame,
    const float *fmap,
    const float *fmap_level4,
    const int *fmap_offsets,
    const int *fmap_heights,
    const int *fmap_widths,
    const int *level4_offsets,
    const int *level4_heights,
    const int *level4_widths,
    float *out,
    int edge_count)
{
    const int channels = 128;
    const int patch_area = 9;
    const int corr_radius = 3;
    const int corr_side = 7;
    const int corr_dim = 2 * corr_side * corr_side * patch_area;
    const int linear = blockIdx.x * blockDim.x + threadIdx.x;
    const int total = edge_count * corr_dim;
    if (linear >= total) {
        return;
    }
    int rem = linear;
    const int level_index = rem % 2;
    rem /= 2;
    const int patch_index = rem % patch_area;
    rem /= patch_area;
    const int oy = rem % corr_side;
    rem /= corr_side;
    const int ox = rem % corr_side;
    const int edge = rem / corr_side;
    const int target_frame = edge_target_frame[edge];
    const int level = level_index == 0 ? 1 : 4;
    const int map_offset = level_index == 0 ? fmap_offsets[target_frame] : level4_offsets[target_frame];
    const int map_height = level_index == 0 ? fmap_heights[target_frame] : level4_heights[target_frame];
    const int map_width = level_index == 0 ? fmap_widths[target_frame] : level4_widths[target_frame];
    const float *map = level_index == 0 ? fmap : fmap_level4;
    const int dx = ox - corr_radius;
    const int dy = oy - corr_radius;
    const float sx = edge_coords[(edge * patch_area + patch_index) * 2] / static_cast<float>(level) +
                     static_cast<float>(dx);
    const float sy = edge_coords[(edge * patch_area + patch_index) * 2 + 1] / static_cast<float>(level) +
                     static_cast<float>(dy);
    float dot = 0.0f;
    for (int c = 0; c < channels; ++c) {
        const float a = edge_patch_gmap[(edge * channels + c) * patch_area + patch_index];
        const float b = dpvo_sample_bilinear(map, map_offset, channels, map_height, map_width, c, sx, sy);
        dot += a * b;
    }
    out[linear] = dot;
}
)CUDA";

        nvrtcProgram program = nullptr;
        if (!CheckNvrtc(m_nvrtcCreateProgram(&program, kSource, "dpvo_native_kernels.cu", 0, nullptr, nullptr),
                        "nvrtcCreateProgram", err)) {
            return false;
        }
        const char *options[] = {"--gpu-architecture=compute_72", "--std=c++14"};
        const nvrtcResult compileResult = m_nvrtcCompileProgram(program, 2, options);
        size_t logSize = 0;
        (void)m_nvrtcGetProgramLogSize(program, &logSize);
        std::string log;
        if (logSize > 1U) {
            log.resize(logSize);
            (void)m_nvrtcGetProgramLog(program, log.data());
        }
        if (compileResult != NVRTC_SUCCESS) {
            if (err != nullptr) {
                const char *text = m_nvrtcGetErrorString != nullptr ? m_nvrtcGetErrorString(compileResult)
                                                                    : "unknown NVRTC error";
                *err = std::string("nvrtcCompileProgram failed: ") + (text != nullptr ? text : "") +
                       (log.empty() ? std::string{} : "\n" + log);
            }
            (void)m_nvrtcDestroyProgram(&program);
            return false;
        }
        size_t ptxSize = 0;
        if (!CheckNvrtc(m_nvrtcGetPTXSize(program, &ptxSize), "nvrtcGetPTXSize", err)) {
            (void)m_nvrtcDestroyProgram(&program);
            return false;
        }
        std::vector<char> ptx(ptxSize);
        if (!CheckNvrtc(m_nvrtcGetPTX(program, ptx.data()), "nvrtcGetPTX", err)) {
            (void)m_nvrtcDestroyProgram(&program);
            return false;
        }
        (void)m_nvrtcDestroyProgram(&program);

        if (!CheckDriver(m_cuModuleLoadData(&m_module, ptx.data()), "cuModuleLoadData", err) ||
            !CheckDriver(m_cuModuleGetFunction(&m_corrSmokeKernel, m_module, "dpvo_corr_patch3_smoke"),
                         "cuModuleGetFunction", err) ||
            !CheckDriver(m_cuModuleGetFunction(&m_corrBatchKernel, m_module, "dpvo_corr_batch"),
                         "cuModuleGetFunction", err)) {
            return false;
        }
        return true;
    }

    bool RunCorrelationSmoke(cudaStream_t stream, std::string *err)
    {
        constexpr int channels = 4;
        constexpr int height = 5;
        constexpr int width = 5;
        constexpr int x = 2;
        constexpr int y = 2;
        std::vector<float> patch(static_cast<size_t>(channels) * 9U, 0.0f);
        std::vector<float> fmap(static_cast<size_t>(channels) * height * width, 0.0f);
        for (size_t i = 0; i < patch.size(); ++i) {
            patch[i] = 0.01f * static_cast<float>(i + 1U);
        }
        for (size_t i = 0; i < fmap.size(); ++i) {
            fmap[i] = 0.02f * static_cast<float>((i % 17U) + 1U);
        }
        float expected = 0.0f;
        for (int c = 0; c < channels; ++c) {
            for (int py = 0; py < 3; ++py) {
                for (int px = 0; px < 3; ++px) {
                    const int p = py * 3 + px;
                    const int yy = y + py - 1;
                    const int xx = x + px - 1;
                    expected += patch[static_cast<size_t>(c) * 9U + static_cast<size_t>(p)] *
                                fmap[(static_cast<size_t>(c) * height + static_cast<size_t>(yy)) * width +
                                     static_cast<size_t>(xx)];
                }
            }
        }

        void *patchDev = nullptr;
        void *fmapDev = nullptr;
        void *outDev = nullptr;
        auto cleanup = [&]() {
            if (patchDev != nullptr) {
                cudaFree(patchDev);
            }
            if (fmapDev != nullptr) {
                cudaFree(fmapDev);
            }
            if (outDev != nullptr) {
                cudaFree(outDev);
            }
        };
        if (cudaMalloc(&patchDev, patch.size() * sizeof(float)) != cudaSuccess ||
            cudaMalloc(&fmapDev, fmap.size() * sizeof(float)) != cudaSuccess ||
            cudaMalloc(&outDev, sizeof(float)) != cudaSuccess) {
            if (err != nullptr) {
                *err = "cudaMalloc failed during DPVO CUDA smoke test";
            }
            cleanup();
            return false;
        }
        cudaError_t rc = cudaMemcpyAsync(patchDev, patch.data(), patch.size() * sizeof(float),
                                         cudaMemcpyHostToDevice, stream);
        if (rc == cudaSuccess) {
            rc = cudaMemcpyAsync(fmapDev, fmap.data(), fmap.size() * sizeof(float), cudaMemcpyHostToDevice, stream);
        }
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("cudaMemcpyAsync failed during DPVO CUDA smoke test: ") + cudaGetErrorString(rc);
            }
            cleanup();
            return false;
        }

        CUdeviceptr patchArg = reinterpret_cast<CUdeviceptr>(patchDev);
        CUdeviceptr fmapArg = reinterpret_cast<CUdeviceptr>(fmapDev);
        CUdeviceptr outArg = reinterpret_cast<CUdeviceptr>(outDev);
        int channelsArg = channels;
        int heightArg = height;
        int widthArg = width;
        int xArg = x;
        int yArg = y;
        void *args[] = {&patchArg, &fmapArg, &outArg, &channelsArg, &heightArg, &widthArg, &xArg, &yArg};
        if (!CheckDriver(m_cuLaunchKernel(m_corrSmokeKernel, 1, 1, 1, 32, 1, 1, 32 * sizeof(float),
                                          reinterpret_cast<CUstream>(stream), args, nullptr),
                         "cuLaunchKernel", err)) {
            cleanup();
            return false;
        }
        float got = 0.0f;
        rc = cudaMemcpyAsync(&got, outDev, sizeof(float), cudaMemcpyDeviceToHost, stream);
        if (rc == cudaSuccess) {
            rc = cudaStreamSynchronize(stream);
        }
        cleanup();
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("CUDA smoke synchronization failed: ") + cudaGetErrorString(rc);
            }
            return false;
        }
        if (std::fabs(got - expected) > 1e-4f) {
            if (err != nullptr) {
                *err = "CUDA smoke mismatch expected=" + std::to_string(expected) +
                       " got=" + std::to_string(got);
            }
            return false;
        }
        m_smokeExpected = expected;
        m_smokeGot = got;
        return true;
    }

    template <typename T>
    bool CopyVectorToDevice(const std::vector<T> &src, CudaDeviceBuffer &dst, cudaStream_t stream,
                            const char *name, std::string *err)
    {
        if (src.empty()) {
            if (err != nullptr) {
                *err = std::string("empty DPVO CUDA buffer: ") + (name != nullptr ? name : "unnamed");
            }
            return false;
        }
        const size_t bytes = src.size() * sizeof(T);
        if (!dst.Ensure(bytes, err)) {
            return false;
        }
        const cudaError_t rc = cudaMemcpyAsync(dst.Data(), src.data(), bytes, cudaMemcpyHostToDevice, stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("cudaMemcpyAsync failed for DPVO CUDA buffer ") +
                       (name != nullptr ? name : "unnamed") + ": " + cudaGetErrorString(rc);
            }
            return false;
        }
        return true;
    }

    void Reset()
    {
        if (m_module != nullptr && m_cuModuleUnload != nullptr) {
            (void)m_cuModuleUnload(m_module);
        }
        m_module = nullptr;
        m_corrSmokeKernel = nullptr;
        m_corrBatchKernel = nullptr;
        m_ready = false;
        m_ownsContext = false;
        m_context = nullptr;
        if (m_cudaLib != nullptr) {
            dlclose(m_cudaLib);
            m_cudaLib = nullptr;
        }
        if (m_nvrtcLib != nullptr) {
            dlclose(m_nvrtcLib);
            m_nvrtcLib = nullptr;
        }
        m_nvrtcCreateProgram = nullptr;
        m_nvrtcCompileProgram = nullptr;
        m_nvrtcGetPTXSize = nullptr;
        m_nvrtcGetPTX = nullptr;
        m_nvrtcGetProgramLogSize = nullptr;
        m_nvrtcGetProgramLog = nullptr;
        m_nvrtcDestroyProgram = nullptr;
        m_nvrtcGetErrorString = nullptr;
        m_cuInit = nullptr;
        m_cuDeviceGet = nullptr;
        m_cuCtxCreate = nullptr;
        m_cuCtxGetCurrent = nullptr;
        m_cuModuleLoadData = nullptr;
        m_cuModuleGetFunction = nullptr;
        m_cuLaunchKernel = nullptr;
        m_cuModuleUnload = nullptr;
        m_cuGetErrorString = nullptr;
    }

    void *m_nvrtcLib{nullptr};
    void *m_cudaLib{nullptr};
    CUcontext m_context{nullptr};
    CUmodule m_module{nullptr};
    CUfunction m_corrSmokeKernel{nullptr};
    CUfunction m_corrBatchKernel{nullptr};
    bool m_ready{false};
    bool m_ownsContext{false};
    float m_smokeExpected{0.0f};
    float m_smokeGot{0.0f};

    nvrtcResult (*m_nvrtcCreateProgram)(nvrtcProgram *, const char *, const char *, int, const char *const *,
                                        const char *const *){nullptr};
    nvrtcResult (*m_nvrtcCompileProgram)(nvrtcProgram, int, const char *const *){nullptr};
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
    CUresult (*m_cuModuleGetFunction)(CUfunction *, CUmodule, const char *){nullptr};
    CUresult (*m_cuLaunchKernel)(CUfunction, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int,
                                 unsigned int, unsigned int, CUstream, void **, void **){nullptr};
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
};

size_t TensorRtDataTypeSize(nvinfer1::DataType type)
{
    switch (type) {
    case nvinfer1::DataType::kFLOAT:
        return 4;
    case nvinfer1::DataType::kHALF:
        return 2;
    case nvinfer1::DataType::kINT8:
    case nvinfer1::DataType::kBOOL:
        return 1;
    case nvinfer1::DataType::kINT32:
        return 4;
    default:
        return 0;
    }
}

int64_t DimsVolume(const nvinfer1::Dims &dims)
{
    int64_t volume = 1;
    for (int i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] < 0) {
            return -1;
        }
        volume *= dims.d[i];
    }
    return volume;
}

std::string DimsToString(const nvinfer1::Dims &dims)
{
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < dims.nbDims; ++i) {
        if (i > 0) {
            oss << "x";
        }
        oss << dims.d[i];
    }
    oss << "]";
    return oss.str();
}

bool SetBindingShape(nvinfer1::IExecutionContext &context, int bindingIndex, const nvinfer1::Dims &dims,
                     std::string *err)
{
    if (bindingIndex < 0) {
        if (err != nullptr) {
            *err = "invalid TensorRT binding index";
        }
        return false;
    }
    if (!context.setBindingDimensions(bindingIndex, dims)) {
        if (err != nullptr) {
            *err = "setBindingDimensions failed for binding " + std::to_string(bindingIndex) +
                   " dims=" + DimsToString(dims);
        }
        return false;
    }
    return true;
}

int FindBindingIndex(const TensorRtEngineHandle &handle, const char *name)
{
    nvinfer1::ICudaEngine *engine = handle.Engine();
    if (engine == nullptr || name == nullptr) {
        return -1;
    }
    return engine->getBindingIndex(name);
}

bool BindingBytes(const TensorRtEngineHandle &handle, int bindingIndex, size_t *bytes, nvinfer1::Dims *dimsOut,
                  std::string *err)
{
    if (bytes == nullptr) {
        return false;
    }
    nvinfer1::ICudaEngine *engine = handle.Engine();
    nvinfer1::IExecutionContext *context = handle.Context();
    if (engine == nullptr || context == nullptr || bindingIndex < 0) {
        if (err != nullptr) {
            *err = "invalid TensorRT engine/context/binding";
        }
        return false;
    }
    nvinfer1::Dims dims = context->getBindingDimensions(bindingIndex);
    const int64_t volume = DimsVolume(dims);
    const size_t elementSize = TensorRtDataTypeSize(engine->getBindingDataType(bindingIndex));
    if (volume <= 0 || elementSize == 0) {
        if (err != nullptr) {
            *err = "invalid TensorRT binding shape/type index=" + std::to_string(bindingIndex) +
                   " dims=" + DimsToString(dims);
        }
        return false;
    }
    *bytes = static_cast<size_t>(volume) * elementSize;
    if (dimsOut != nullptr) {
        *dimsOut = dims;
    }
    return true;
}

size_t BindingElementCount(const TensorRtEngineHandle &handle, int bindingIndex, nvinfer1::Dims *dimsOut,
                           std::string *err)
{
    size_t bytes = 0;
    nvinfer1::Dims dims{};
    if (!BindingBytes(handle, bindingIndex, &bytes, &dims, err)) {
        return 0;
    }
    const size_t elementSize = TensorRtDataTypeSize(handle.Engine()->getBindingDataType(bindingIndex));
    if (elementSize == 0) {
        if (err != nullptr) {
            *err = "invalid TensorRT binding element size index=" + std::to_string(bindingIndex);
        }
        return 0;
    }
    if (dimsOut != nullptr) {
        *dimsOut = dims;
    }
    return bytes / elementSize;
}

bool CopyFloatHostToBindingDevice(const TensorRtEngineHandle &handle, int bindingIndex, const float *src,
                                  size_t valueCount, CudaDeviceBuffer &device, cudaStream_t stream,
                                  std::vector<__half> &halfScratch, std::string *err)
{
    if (src == nullptr || stream == nullptr || handle.Engine() == nullptr) {
        if (err != nullptr) {
            *err = "invalid host/device copy input";
        }
        return false;
    }
    nvinfer1::Dims dims{};
    const size_t bindingCount = BindingElementCount(handle, bindingIndex, &dims, err);
    if (bindingCount == 0 || bindingCount != valueCount) {
        if (err != nullptr) {
            *err = "TensorRT input element count mismatch index=" + std::to_string(bindingIndex) +
                   " expected=" + std::to_string(bindingCount) + " got=" + std::to_string(valueCount) +
                   " dims=" + DimsToString(dims);
        }
        return false;
    }
    size_t bytes = 0;
    if (!BindingBytes(handle, bindingIndex, &bytes, nullptr, err) || !device.Ensure(bytes, err)) {
        return false;
    }
    const nvinfer1::DataType type = handle.Engine()->getBindingDataType(bindingIndex);
    if (type == nvinfer1::DataType::kFLOAT) {
        const cudaError_t rc = cudaMemcpyAsync(device.Data(), src, bytes, cudaMemcpyHostToDevice, stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("TensorRT float H2D copy failed: ") + cudaGetErrorString(rc);
            }
            return false;
        }
        return true;
    }
    if (type == nvinfer1::DataType::kHALF) {
        halfScratch.resize(valueCount);
        for (size_t i = 0; i < valueCount; ++i) {
            halfScratch[i] = __float2half(src[i]);
        }
        const cudaError_t rc =
            cudaMemcpyAsync(device.Data(), halfScratch.data(), bytes, cudaMemcpyHostToDevice, stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("TensorRT half H2D copy failed: ") + cudaGetErrorString(rc);
            }
            return false;
        }
        return true;
    }
    if (err != nullptr) {
        *err = "unsupported TensorRT input dtype index=" + std::to_string(bindingIndex);
    }
    return false;
}

bool CopyBindingDeviceToFloatHost(const TensorRtEngineHandle &handle, int bindingIndex, CudaDeviceBuffer &device,
                                  cudaStream_t stream, std::vector<float> &dst, std::vector<__half> &halfScratch,
                                  nvinfer1::Dims *dimsOut, std::string *err)
{
    if (stream == nullptr || handle.Engine() == nullptr) {
        if (err != nullptr) {
            *err = "invalid device/host copy input";
        }
        return false;
    }
    nvinfer1::Dims dims{};
    const size_t valueCount = BindingElementCount(handle, bindingIndex, &dims, err);
    if (valueCount == 0) {
        return false;
    }
    size_t bytes = 0;
    if (!BindingBytes(handle, bindingIndex, &bytes, nullptr, err) || !device.Ensure(bytes, err)) {
        return false;
    }
    const nvinfer1::DataType type = handle.Engine()->getBindingDataType(bindingIndex);
    dst.resize(valueCount);
    if (type == nvinfer1::DataType::kFLOAT) {
        const cudaError_t rc = cudaMemcpyAsync(dst.data(), device.Data(), bytes, cudaMemcpyDeviceToHost, stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("TensorRT float D2H copy failed: ") + cudaGetErrorString(rc);
            }
            return false;
        }
    } else if (type == nvinfer1::DataType::kHALF) {
        halfScratch.resize(valueCount);
        const cudaError_t rc =
            cudaMemcpyAsync(halfScratch.data(), device.Data(), bytes, cudaMemcpyDeviceToHost, stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("TensorRT half D2H copy failed: ") + cudaGetErrorString(rc);
            }
            return false;
        }
        const cudaError_t syncRc = cudaStreamSynchronize(stream);
        if (syncRc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("TensorRT half D2H synchronize failed: ") + cudaGetErrorString(syncRc);
            }
            return false;
        }
        for (size_t i = 0; i < valueCount; ++i) {
            dst[i] = __half2float(halfScratch[i]);
        }
        if (dimsOut != nullptr) {
            *dimsOut = dims;
        }
        return true;
    } else {
        if (err != nullptr) {
            *err = "unsupported TensorRT output dtype index=" + std::to_string(bindingIndex);
        }
        return false;
    }
    const cudaError_t syncRc = cudaStreamSynchronize(stream);
    if (syncRc != cudaSuccess) {
        if (err != nullptr) {
            *err = std::string("TensorRT float D2H synchronize failed: ") + cudaGetErrorString(syncRc);
        }
        return false;
    }
    if (dimsOut != nullptr) {
        *dimsOut = dims;
    }
    return true;
}

bool EnsureBindingBuffer(const TensorRtEngineHandle &handle, int bindingIndex, CudaDeviceBuffer &device,
                         std::string *err)
{
    size_t bytes = 0;
    return BindingBytes(handle, bindingIndex, &bytes, nullptr, err) && device.Ensure(bytes, err);
}

struct DpvoPatchifierRun {
    nvinfer1::Dims fmapDims{};
    nvinfer1::Dims imapDims{};
    const float *fmapHost{nullptr};
    const float *imapHost{nullptr};
    size_t fmapValueCount{0};
    size_t imapValueCount{0};
    double elapsedMs{0.0};
    bool ok{false};
};

class DpvoPatchifierRuntime {
  public:
    bool Initialize(const TensorRtEngineHandle &engine, int width, int height, std::string *err)
    {
        m_imageIndex = FindBindingIndex(engine, "image");
        m_fmapIndex = FindBindingIndex(engine, "fmap");
        m_imapIndex = FindBindingIndex(engine, "imap");
        if (m_imageIndex < 0 || m_fmapIndex < 0 || m_imapIndex < 0) {
            if (err != nullptr) {
                *err = "patchifier binding lookup failed: image=" + std::to_string(m_imageIndex) +
                       " fmap=" + std::to_string(m_fmapIndex) + " imap=" + std::to_string(m_imapIndex);
            }
            return false;
        }
        m_width = width;
        m_height = height;
        m_inputHost.assign(static_cast<size_t>(3) * static_cast<size_t>(width) * static_cast<size_t>(height), 0.0f);
        return true;
    }

    DpvoPatchifierRun Run(const cv::Mat &gray, TensorRtEngineHandle &engine, cudaStream_t stream,
                          bool copyFmapToHost, bool copyImapToHost, std::string *err)
    {
        DpvoPatchifierRun result{};
        if (gray.empty() || gray.cols != m_width || gray.rows != m_height || gray.type() != CV_8UC1) {
            if (err != nullptr) {
                *err = "patchifier expects CV_8UC1 " + std::to_string(m_width) + "x" + std::to_string(m_height);
            }
            return result;
        }

        nvinfer1::IExecutionContext *context = engine.Context();
        if (context == nullptr || stream == nullptr) {
            if (err != nullptr) {
                *err = "patchifier TensorRT context or CUDA stream is not initialized";
            }
            return result;
        }
        if (!SetBindingShape(*context, m_imageIndex, nvinfer1::Dims4{1, 3, m_height, m_width}, err)) {
            return result;
        }

        size_t imageBytes = 0;
        size_t fmapBytes = 0;
        size_t imapBytes = 0;
        if (!BindingBytes(engine, m_imageIndex, &imageBytes, nullptr, err) ||
            !BindingBytes(engine, m_fmapIndex, &fmapBytes, &result.fmapDims, err) ||
            !BindingBytes(engine, m_imapIndex, &imapBytes, &result.imapDims, err)) {
            return result;
        }
        if (imageBytes != m_inputHost.size() * sizeof(float)) {
            if (err != nullptr) {
                *err = "patchifier image binding byte mismatch expected=" +
                       std::to_string(m_inputHost.size() * sizeof(float)) + " got=" + std::to_string(imageBytes);
            }
            return result;
        }

        if (!m_imageDevice.Ensure(imageBytes, err) || !m_fmapDevice.Ensure(fmapBytes, err) ||
            !m_imapDevice.Ensure(imapBytes, err)) {
            return result;
        }

        FillInput(gray);
        const int nbBindings = engine.Engine() != nullptr ? engine.Engine()->getNbBindings() : 0;
        if (nbBindings <= 0) {
            if (err != nullptr) {
                *err = "patchifier engine has no bindings";
            }
            return result;
        }
        std::vector<void *> bindings(static_cast<size_t>(nbBindings), nullptr);
        bindings[static_cast<size_t>(m_imageIndex)] = m_imageDevice.Data();
        bindings[static_cast<size_t>(m_fmapIndex)] = m_fmapDevice.Data();
        bindings[static_cast<size_t>(m_imapIndex)] = m_imapDevice.Data();

        const auto t0 = std::chrono::steady_clock::now();
        cudaError_t rc = cudaMemcpyAsync(m_imageDevice.Data(), m_inputHost.data(), imageBytes,
                                         cudaMemcpyHostToDevice, stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("patchifier H2D copy failed: ") + cudaGetErrorString(rc);
            }
            return result;
        }
        if (!context->enqueueV2(bindings.data(), stream, nullptr)) {
            if (err != nullptr) {
                *err = "patchifier enqueueV2 failed";
            }
            return result;
        }
        rc = cudaStreamSynchronize(stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("patchifier synchronize failed: ") + cudaGetErrorString(rc);
            }
            return result;
        }
        if (copyFmapToHost) {
            if (!CopyBindingDeviceToFloatHost(engine, m_fmapIndex, m_fmapDevice, stream, m_fmapHost,
                                              m_halfScratch, nullptr, err)) {
                return result;
            }
            result.fmapHost = m_fmapHost.data();
            result.fmapValueCount = m_fmapHost.size();
        }
        if (copyImapToHost) {
            if (!CopyBindingDeviceToFloatHost(engine, m_imapIndex, m_imapDevice, stream, m_imapHost,
                                              m_halfScratch, nullptr, err)) {
                return result;
            }
            result.imapHost = m_imapHost.data();
            result.imapValueCount = m_imapHost.size();
        }
        result.elapsedMs = ElapsedMs(t0, std::chrono::steady_clock::now());
        result.ok = true;
        m_lastFmapDims = result.fmapDims;
        m_lastImapDims = result.imapDims;
        return result;
    }

    void *FmapDevice() const { return m_fmapDevice.Data(); }
    void *ImapDevice() const { return m_imapDevice.Data(); }
    nvinfer1::Dims LastFmapDims() const { return m_lastFmapDims; }
    nvinfer1::Dims LastImapDims() const { return m_lastImapDims; }

  private:
    void FillInput(const cv::Mat &gray)
    {
        const size_t plane = static_cast<size_t>(m_width) * static_cast<size_t>(m_height);
        for (int y = 0; y < m_height; ++y) {
            const uint8_t *src = gray.ptr<uint8_t>(y);
            for (int x = 0; x < m_width; ++x) {
                const float v = static_cast<float>(src[x]);
                const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(m_width) + static_cast<size_t>(x);
                m_inputHost[idx] = v;
                m_inputHost[plane + idx] = v;
                m_inputHost[2 * plane + idx] = v;
            }
        }
    }

    int m_imageIndex{-1};
    int m_fmapIndex{-1};
    int m_imapIndex{-1};
    int m_width{0};
    int m_height{0};
    std::vector<float> m_inputHost;
    std::vector<float> m_fmapHost;
    std::vector<float> m_imapHost;
    std::vector<__half> m_halfScratch;
    CudaDeviceBuffer m_imageDevice;
    CudaDeviceBuffer m_fmapDevice;
    CudaDeviceBuffer m_imapDevice;
    nvinfer1::Dims m_lastFmapDims{};
    nvinfer1::Dims m_lastImapDims{};
};

struct DpvoUpdateRun {
    double elapsedMs{0.0};
    bool ok{false};
};

struct DpvoUpdatePreAggRun {
    std::vector<float> baseNet;
    std::vector<float> aggKkF;
    std::vector<float> aggKkG;
    std::vector<float> aggIjF;
    std::vector<float> aggIjG;
    double elapsedMs{0.0};
    bool ok{false};
};

struct DpvoUpdatePostAggRun {
    std::vector<float> updatedNet;
    std::vector<float> delta;
    std::vector<float> weight;
    double elapsedMs{0.0};
    bool ok{false};
};

DpvoUpdateRun WarmupBindings(TensorRtEngineHandle &engine, cudaStream_t stream, std::initializer_list<int> indices,
                             std::array<CudaDeviceBuffer, 16> &buffers, const char *name, std::string *err)
{
    DpvoUpdateRun result{};
    if (engine.Engine() == nullptr || engine.Context() == nullptr || stream == nullptr) {
        if (err != nullptr) {
            *err = std::string(name != nullptr ? name : "engine") + " TensorRT context or CUDA stream is not initialized";
        }
        return result;
    }

    const int nbBindings = engine.Engine()->getNbBindings();
    if (nbBindings > static_cast<int>(buffers.size())) {
        if (err != nullptr) {
            *err = std::string(name != nullptr ? name : "engine") + " has more bindings than expected";
        }
        return result;
    }

    for (int index : indices) {
        size_t bytes = 0;
        if (!BindingBytes(engine, index, &bytes, nullptr, err)) {
            return result;
        }
        if (index < 0 || index >= static_cast<int>(buffers.size())) {
            if (err != nullptr) {
                *err = std::string(name != nullptr ? name : "engine") + " binding index exceeds local buffer table";
            }
            return result;
        }
        if (!buffers[static_cast<size_t>(index)].Ensure(bytes, err)) {
            return result;
        }
        if (engine.Engine()->bindingIsInput(index)) {
            const cudaError_t rc = cudaMemsetAsync(buffers[static_cast<size_t>(index)].Data(), 0, bytes, stream);
            if (rc != cudaSuccess) {
                if (err != nullptr) {
                    *err = std::string(name != nullptr ? name : "engine") +
                           " input memset failed: " + cudaGetErrorString(rc);
                }
                return result;
            }
        }
    }

    std::array<void *, 16> bindings{};
    for (int i = 0; i < nbBindings; ++i) {
        bindings[static_cast<size_t>(i)] = buffers[static_cast<size_t>(i)].Data();
    }

    const auto t0 = std::chrono::steady_clock::now();
    if (!engine.Context()->enqueueV2(bindings.data(), stream, nullptr)) {
        if (err != nullptr) {
            *err = std::string(name != nullptr ? name : "engine") + " enqueueV2 failed";
        }
        return result;
    }
    const cudaError_t rc = cudaStreamSynchronize(stream);
    if (rc != cudaSuccess) {
        if (err != nullptr) {
            *err = std::string(name != nullptr ? name : "engine") + " synchronize failed: " + cudaGetErrorString(rc);
        }
        return result;
    }
    result.elapsedMs = ElapsedMs(t0, std::chrono::steady_clock::now());
    result.ok = true;
    return result;
}

class DpvoUpdateRuntime {
  public:
    bool Initialize(const TensorRtEngineHandle &engine, std::string *err)
    {
        m_netIndex = FindBindingIndex(engine, "net");
        m_inpIndex = FindBindingIndex(engine, "inp");
        m_corrIndex = FindBindingIndex(engine, "corr");
        m_prevNetIndex = FindBindingIndex(engine, "prev_net");
        m_nextNetIndex = FindBindingIndex(engine, "next_net");
        m_prevMaskIndex = FindBindingIndex(engine, "prev_mask");
        m_nextMaskIndex = FindBindingIndex(engine, "next_mask");
        m_updatedNetIndex = FindBindingIndex(engine, "updated_net");
        m_deltaIndex = FindBindingIndex(engine, "delta");
        m_weightIndex = FindBindingIndex(engine, "weight");
        const bool ok = m_netIndex >= 0 && m_inpIndex >= 0 && m_corrIndex >= 0 && m_prevNetIndex >= 0 &&
                        m_nextNetIndex >= 0 && m_prevMaskIndex >= 0 && m_nextMaskIndex >= 0 &&
                        m_updatedNetIndex >= 0 && m_deltaIndex >= 0 && m_weightIndex >= 0;
        if (!ok && err != nullptr) {
            *err = "update binding lookup failed";
        }
        return ok;
    }

    DpvoUpdateRun Warmup(TensorRtEngineHandle &engine, cudaStream_t stream, int edges, std::string *err)
    {
        edges = std::clamp(edges, 1, 4096);
        nvinfer1::IExecutionContext *context = engine.Context();
        if (context == nullptr || stream == nullptr) {
            if (err != nullptr) {
                *err = "update TensorRT context or CUDA stream is not initialized";
            }
            return {};
        }
        if (!SetBindingShape(*context, m_netIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_inpIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_corrIndex, nvinfer1::Dims3{1, edges, kCorrDim}, err) ||
            !SetBindingShape(*context, m_prevNetIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_nextNetIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_prevMaskIndex, nvinfer1::Dims3{1, edges, 1}, err) ||
            !SetBindingShape(*context, m_nextMaskIndex, nvinfer1::Dims3{1, edges, 1}, err)) {
            return {};
        }
        return WarmupBindings(engine, stream,
                              {m_netIndex, m_inpIndex, m_corrIndex, m_prevNetIndex, m_nextNetIndex, m_prevMaskIndex,
                               m_nextMaskIndex, m_updatedNetIndex, m_deltaIndex, m_weightIndex},
                              m_buffers, "update", err);
    }

    static constexpr int kDim = 384;
    static constexpr int kCorrDim = 882;

  private:
    int m_netIndex{-1};
    int m_inpIndex{-1};
    int m_corrIndex{-1};
    int m_prevNetIndex{-1};
    int m_nextNetIndex{-1};
    int m_prevMaskIndex{-1};
    int m_nextMaskIndex{-1};
    int m_updatedNetIndex{-1};
    int m_deltaIndex{-1};
    int m_weightIndex{-1};
    std::array<CudaDeviceBuffer, 16> m_buffers;
};

class DpvoUpdatePreAggRuntime {
  public:
    bool Initialize(const TensorRtEngineHandle &engine, std::string *err)
    {
        m_netIndex = FindBindingIndex(engine, "net");
        m_inpIndex = FindBindingIndex(engine, "inp");
        m_corrIndex = FindBindingIndex(engine, "corr");
        m_prevNetIndex = FindBindingIndex(engine, "prev_net");
        m_nextNetIndex = FindBindingIndex(engine, "next_net");
        m_prevMaskIndex = FindBindingIndex(engine, "prev_mask");
        m_nextMaskIndex = FindBindingIndex(engine, "next_mask");
        m_baseNetIndex = FindBindingIndex(engine, "base_net");
        m_kkFIndex = FindBindingIndex(engine, "agg_kk_f");
        m_kkGIndex = FindBindingIndex(engine, "agg_kk_g");
        m_ijFIndex = FindBindingIndex(engine, "agg_ij_f");
        m_ijGIndex = FindBindingIndex(engine, "agg_ij_g");
        const bool ok = m_netIndex >= 0 && m_inpIndex >= 0 && m_corrIndex >= 0 && m_prevNetIndex >= 0 &&
                        m_nextNetIndex >= 0 && m_prevMaskIndex >= 0 && m_nextMaskIndex >= 0 &&
                        m_baseNetIndex >= 0 && m_kkFIndex >= 0 && m_kkGIndex >= 0 &&
                        m_ijFIndex >= 0 && m_ijGIndex >= 0;
        if (!ok && err != nullptr) {
            *err = "update-preagg binding lookup failed";
        }
        return ok;
    }

    DpvoUpdateRun Warmup(TensorRtEngineHandle &engine, cudaStream_t stream, int edges, std::string *err)
    {
        edges = std::clamp(edges, 1, 4096);
        nvinfer1::IExecutionContext *context = engine.Context();
        if (context == nullptr || stream == nullptr) {
            if (err != nullptr) {
                *err = "update-preagg TensorRT context or CUDA stream is not initialized";
            }
            return {};
        }
        if (!SetBindingShape(*context, m_netIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_inpIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_corrIndex, nvinfer1::Dims3{1, edges, kCorrDim}, err) ||
            !SetBindingShape(*context, m_prevNetIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_nextNetIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_prevMaskIndex, nvinfer1::Dims3{1, edges, 1}, err) ||
            !SetBindingShape(*context, m_nextMaskIndex, nvinfer1::Dims3{1, edges, 1}, err)) {
            return {};
        }
        return WarmupBindings(engine, stream,
                              {m_netIndex, m_inpIndex, m_corrIndex, m_prevNetIndex, m_nextNetIndex, m_prevMaskIndex,
                               m_nextMaskIndex, m_baseNetIndex, m_kkFIndex, m_kkGIndex, m_ijFIndex, m_ijGIndex},
                              m_buffers, "update-preagg", err);
    }

    DpvoUpdatePreAggRun Run(TensorRtEngineHandle &engine, cudaStream_t stream, int edges,
                            const std::vector<float> &net, const std::vector<float> &inp,
                            const std::vector<float> &corr, const std::vector<float> &prevNet,
                            const std::vector<float> &nextNet, const std::vector<float> &prevMask,
                            const std::vector<float> &nextMask, std::string *err)
    {
        DpvoUpdatePreAggRun result{};
        edges = std::clamp(edges, 1, 4096);
        nvinfer1::IExecutionContext *context = engine.Context();
        if (context == nullptr || stream == nullptr) {
            if (err != nullptr) {
                *err = "update-preagg TensorRT context or CUDA stream is not initialized";
            }
            return result;
        }
        if (!SetBindingShape(*context, m_netIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_inpIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_corrIndex, nvinfer1::Dims3{1, edges, kCorrDim}, err) ||
            !SetBindingShape(*context, m_prevNetIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_nextNetIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_prevMaskIndex, nvinfer1::Dims3{1, edges, 1}, err) ||
            !SetBindingShape(*context, m_nextMaskIndex, nvinfer1::Dims3{1, edges, 1}, err)) {
            return result;
        }

        const size_t dimValues = static_cast<size_t>(edges) * kDim;
        const size_t corrValues = static_cast<size_t>(edges) * kCorrDim;
        const size_t maskValues = static_cast<size_t>(edges);
        if (net.size() != dimValues || inp.size() != dimValues || corr.size() != corrValues ||
            prevNet.size() != dimValues || nextNet.size() != dimValues || prevMask.size() != maskValues ||
            nextMask.size() != maskValues) {
            if (err != nullptr) {
                *err = "update-preagg input vector size mismatch edges=" + std::to_string(edges);
            }
            return result;
        }

        if (!EnsureBindingBuffer(engine, m_baseNetIndex, m_buffers[static_cast<size_t>(m_baseNetIndex)], err) ||
            !EnsureBindingBuffer(engine, m_kkFIndex, m_buffers[static_cast<size_t>(m_kkFIndex)], err) ||
            !EnsureBindingBuffer(engine, m_kkGIndex, m_buffers[static_cast<size_t>(m_kkGIndex)], err) ||
            !EnsureBindingBuffer(engine, m_ijFIndex, m_buffers[static_cast<size_t>(m_ijFIndex)], err) ||
            !EnsureBindingBuffer(engine, m_ijGIndex, m_buffers[static_cast<size_t>(m_ijGIndex)], err)) {
            return result;
        }
        if (!CopyFloatHostToBindingDevice(engine, m_netIndex, net.data(), dimValues,
                                          m_buffers[static_cast<size_t>(m_netIndex)], stream, m_halfScratch, err) ||
            !CopyFloatHostToBindingDevice(engine, m_inpIndex, inp.data(), dimValues,
                                          m_buffers[static_cast<size_t>(m_inpIndex)], stream, m_halfScratch, err) ||
            !CopyFloatHostToBindingDevice(engine, m_corrIndex, corr.data(), corrValues,
                                          m_buffers[static_cast<size_t>(m_corrIndex)], stream, m_halfScratch, err) ||
            !CopyFloatHostToBindingDevice(engine, m_prevNetIndex, prevNet.data(), dimValues,
                                          m_buffers[static_cast<size_t>(m_prevNetIndex)], stream, m_halfScratch, err) ||
            !CopyFloatHostToBindingDevice(engine, m_nextNetIndex, nextNet.data(), dimValues,
                                          m_buffers[static_cast<size_t>(m_nextNetIndex)], stream, m_halfScratch, err) ||
            !CopyFloatHostToBindingDevice(engine, m_prevMaskIndex, prevMask.data(), maskValues,
                                          m_buffers[static_cast<size_t>(m_prevMaskIndex)], stream, m_halfScratch, err) ||
            !CopyFloatHostToBindingDevice(engine, m_nextMaskIndex, nextMask.data(), maskValues,
                                          m_buffers[static_cast<size_t>(m_nextMaskIndex)], stream, m_halfScratch, err)) {
            return result;
        }

        std::array<void *, 16> bindings{};
        const int nbBindings = engine.Engine() != nullptr ? engine.Engine()->getNbBindings() : 0;
        for (int i = 0; i < nbBindings; ++i) {
            bindings[static_cast<size_t>(i)] = m_buffers[static_cast<size_t>(i)].Data();
        }

        const auto t0 = std::chrono::steady_clock::now();
        if (!context->enqueueV2(bindings.data(), stream, nullptr)) {
            if (err != nullptr) {
                *err = "update-preagg enqueueV2 failed";
            }
            return result;
        }
        cudaError_t rc = cudaStreamSynchronize(stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("update-preagg synchronize failed: ") + cudaGetErrorString(rc);
            }
            return result;
        }
        if (!CopyBindingDeviceToFloatHost(engine, m_baseNetIndex, m_buffers[static_cast<size_t>(m_baseNetIndex)],
                                          stream, result.baseNet, m_halfScratch, nullptr, err) ||
            !CopyBindingDeviceToFloatHost(engine, m_kkFIndex, m_buffers[static_cast<size_t>(m_kkFIndex)],
                                          stream, result.aggKkF, m_halfScratch, nullptr, err) ||
            !CopyBindingDeviceToFloatHost(engine, m_kkGIndex, m_buffers[static_cast<size_t>(m_kkGIndex)],
                                          stream, result.aggKkG, m_halfScratch, nullptr, err) ||
            !CopyBindingDeviceToFloatHost(engine, m_ijFIndex, m_buffers[static_cast<size_t>(m_ijFIndex)],
                                          stream, result.aggIjF, m_halfScratch, nullptr, err) ||
            !CopyBindingDeviceToFloatHost(engine, m_ijGIndex, m_buffers[static_cast<size_t>(m_ijGIndex)],
                                          stream, result.aggIjG, m_halfScratch, nullptr, err)) {
            return result;
        }
        result.elapsedMs = ElapsedMs(t0, std::chrono::steady_clock::now());
        result.ok = true;
        return result;
    }

  private:
    static constexpr int kDim = 384;
    static constexpr int kCorrDim = 882;
    int m_netIndex{-1};
    int m_inpIndex{-1};
    int m_corrIndex{-1};
    int m_prevNetIndex{-1};
    int m_nextNetIndex{-1};
    int m_prevMaskIndex{-1};
    int m_nextMaskIndex{-1};
    int m_baseNetIndex{-1};
    int m_kkFIndex{-1};
    int m_kkGIndex{-1};
    int m_ijFIndex{-1};
    int m_ijGIndex{-1};
    std::array<CudaDeviceBuffer, 16> m_buffers;
    std::vector<__half> m_halfScratch;
};

class DpvoUpdatePostAggRuntime {
  public:
    bool Initialize(const TensorRtEngineHandle &engine, std::string *err)
    {
        m_baseNetIndex = FindBindingIndex(engine, "base_net");
        m_kkYIndex = FindBindingIndex(engine, "agg_kk_y");
        m_ijYIndex = FindBindingIndex(engine, "agg_ij_y");
        m_updatedNetIndex = FindBindingIndex(engine, "updated_net");
        m_deltaIndex = FindBindingIndex(engine, "delta");
        m_weightIndex = FindBindingIndex(engine, "weight");
        const bool ok = m_baseNetIndex >= 0 && m_kkYIndex >= 0 && m_ijYIndex >= 0 &&
                        m_updatedNetIndex >= 0 && m_deltaIndex >= 0 && m_weightIndex >= 0;
        if (!ok && err != nullptr) {
            *err = "update-postagg binding lookup failed";
        }
        return ok;
    }

    DpvoUpdateRun Warmup(TensorRtEngineHandle &engine, cudaStream_t stream, int edges, std::string *err)
    {
        edges = std::clamp(edges, 1, 4096);
        nvinfer1::IExecutionContext *context = engine.Context();
        if (context == nullptr || stream == nullptr) {
            if (err != nullptr) {
                *err = "update-postagg TensorRT context or CUDA stream is not initialized";
            }
            return {};
        }
        if (!SetBindingShape(*context, m_baseNetIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_kkYIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_ijYIndex, nvinfer1::Dims3{1, edges, kDim}, err)) {
            return {};
        }
        return WarmupBindings(engine, stream,
                              {m_baseNetIndex, m_kkYIndex, m_ijYIndex, m_updatedNetIndex, m_deltaIndex, m_weightIndex},
                              m_buffers, "update-postagg", err);
    }

    DpvoUpdatePostAggRun Run(TensorRtEngineHandle &engine, cudaStream_t stream, int edges,
                             const std::vector<float> &baseNet, const std::vector<float> &aggKkY,
                             const std::vector<float> &aggIjY, std::string *err)
    {
        DpvoUpdatePostAggRun result{};
        edges = std::clamp(edges, 1, 4096);
        nvinfer1::IExecutionContext *context = engine.Context();
        if (context == nullptr || stream == nullptr) {
            if (err != nullptr) {
                *err = "update-postagg TensorRT context or CUDA stream is not initialized";
            }
            return result;
        }
        if (!SetBindingShape(*context, m_baseNetIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_kkYIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_ijYIndex, nvinfer1::Dims3{1, edges, kDim}, err)) {
            return result;
        }
        const size_t dimValues = static_cast<size_t>(edges) * kDim;
        if (baseNet.size() != dimValues || aggKkY.size() != dimValues || aggIjY.size() != dimValues) {
            if (err != nullptr) {
                *err = "update-postagg input vector size mismatch edges=" + std::to_string(edges);
            }
            return result;
        }

        if (!EnsureBindingBuffer(engine, m_updatedNetIndex, m_buffers[static_cast<size_t>(m_updatedNetIndex)], err) ||
            !EnsureBindingBuffer(engine, m_deltaIndex, m_buffers[static_cast<size_t>(m_deltaIndex)], err) ||
            !EnsureBindingBuffer(engine, m_weightIndex, m_buffers[static_cast<size_t>(m_weightIndex)], err)) {
            return result;
        }
        if (!CopyFloatHostToBindingDevice(engine, m_baseNetIndex, baseNet.data(), dimValues,
                                          m_buffers[static_cast<size_t>(m_baseNetIndex)], stream, m_halfScratch, err) ||
            !CopyFloatHostToBindingDevice(engine, m_kkYIndex, aggKkY.data(), dimValues,
                                          m_buffers[static_cast<size_t>(m_kkYIndex)], stream, m_halfScratch, err) ||
            !CopyFloatHostToBindingDevice(engine, m_ijYIndex, aggIjY.data(), dimValues,
                                          m_buffers[static_cast<size_t>(m_ijYIndex)], stream, m_halfScratch, err)) {
            return result;
        }

        std::array<void *, 16> bindings{};
        const int nbBindings = engine.Engine() != nullptr ? engine.Engine()->getNbBindings() : 0;
        for (int i = 0; i < nbBindings; ++i) {
            bindings[static_cast<size_t>(i)] = m_buffers[static_cast<size_t>(i)].Data();
        }

        const auto t0 = std::chrono::steady_clock::now();
        if (!context->enqueueV2(bindings.data(), stream, nullptr)) {
            if (err != nullptr) {
                *err = "update-postagg enqueueV2 failed";
            }
            return result;
        }
        cudaError_t rc = cudaStreamSynchronize(stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("update-postagg synchronize failed: ") + cudaGetErrorString(rc);
            }
            return result;
        }
        if (!CopyBindingDeviceToFloatHost(engine, m_updatedNetIndex,
                                          m_buffers[static_cast<size_t>(m_updatedNetIndex)], stream,
                                          result.updatedNet, m_halfScratch, nullptr, err) ||
            !CopyBindingDeviceToFloatHost(engine, m_deltaIndex, m_buffers[static_cast<size_t>(m_deltaIndex)],
                                          stream, result.delta, m_halfScratch, nullptr, err) ||
            !CopyBindingDeviceToFloatHost(engine, m_weightIndex, m_buffers[static_cast<size_t>(m_weightIndex)],
                                          stream, result.weight, m_halfScratch, nullptr, err)) {
            return result;
        }
        result.elapsedMs = ElapsedMs(t0, std::chrono::steady_clock::now());
        result.ok = true;
        return result;
    }

  private:
    static constexpr int kDim = 384;
    int m_baseNetIndex{-1};
    int m_kkYIndex{-1};
    int m_ijYIndex{-1};
    int m_updatedNetIndex{-1};
    int m_deltaIndex{-1};
    int m_weightIndex{-1};
    std::array<CudaDeviceBuffer, 16> m_buffers;
    std::vector<__half> m_halfScratch;
};

struct DpvoPatchState {
    float x{0.0f};
    float y{0.0f};
    float invDepth{1.0f};
    float stereoPriorInvDepth{1.0f};
    bool hasStereoPrior{false};
};

struct DpvoFrameState {
    uint64_t frameId{0};
    int64_t timestampNs{0};
    Sophus::SE3f Tcw;
    std::vector<DpvoPatchState> patches;
    std::vector<float> fmap;
    std::vector<float> fmapLevel4;
    std::vector<float> patchImap;
    std::vector<float> patchGmap;
    int fmapChannels{0};
    int fmapHeight{0};
    int fmapWidth{0};
    int imapChannels{0};
    int imapHeight{0};
    int imapWidth{0};
    int imageWidth{0};
    int imageHeight{0};
};

struct DpvoEdgeState {
    int patchGlobal{0};
    int sourceFrame{0};
    int targetFrame{0};
};

struct DpvoIntrinsics {
    float fx{0.0f};
    float fy{0.0f};
    float cx{0.0f};
    float cy{0.0f};
};

class DpvoGraphState {
  public:
    void Reset(int patchesPerFrame, int optimizationWindow)
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
        m_persistentEdges = EnvFlagEnabled("SMART_DRONE_DPVO_PERSISTENT_EDGES", false);
        m_keyframeRemovalEnabled = EnvFlagEnabled("SMART_DRONE_DPVO_KEYFRAME", false);
        m_maxActiveEdges = std::clamp(EnvIntValue("SMART_DRONE_DPVO_MAX_EDGES", 1024), 512, 4096);
    }

    void PushFrame(uint64_t frameId, int64_t timestampNs, const cv::Mat &gray, const Sophus::SE3f &initialPose,
                   const DpvoPatchifierRun &patchRun)
    {
        DpvoFrameState frame{};
        frame.frameId = frameId;
        frame.timestampNs = timestampNs;
        frame.Tcw = initialPose;
        frame.imageWidth = gray.cols;
        frame.imageHeight = gray.rows;
        if (patchRun.ok && patchRun.fmapHost != nullptr && patchRun.fmapDims.nbDims == 4) {
            frame.fmapChannels = static_cast<int>(patchRun.fmapDims.d[1]);
            frame.fmapHeight = static_cast<int>(patchRun.fmapDims.d[2]);
            frame.fmapWidth = static_cast<int>(patchRun.fmapDims.d[3]);
            const size_t expected = static_cast<size_t>(frame.fmapChannels) *
                                    static_cast<size_t>(frame.fmapHeight) *
                                    static_cast<size_t>(frame.fmapWidth);
            if (expected > 0 && expected <= patchRun.fmapValueCount) {
                frame.fmap.assign(patchRun.fmapHost, patchRun.fmapHost + expected);
                frame.fmapLevel4 = BuildPooledFmap(frame.fmap, frame.fmapChannels, frame.fmapHeight, frame.fmapWidth, 4);
            }
        }
        if (patchRun.ok && patchRun.imapHost != nullptr && patchRun.imapDims.nbDims == 4) {
            frame.imapChannels = static_cast<int>(patchRun.imapDims.d[1]);
            frame.imapHeight = static_cast<int>(patchRun.imapDims.d[2]);
            frame.imapWidth = static_cast<int>(patchRun.imapDims.d[3]);
        }
        frame.patches = SelectPatches(frame, gray);
        if (frame.patches.size() < static_cast<size_t>(m_patchesPerFrame)) {
            frame.patches.resize(static_cast<size_t>(m_patchesPerFrame));
        }
        if (patchRun.ok && patchRun.imapHost != nullptr && frame.imapChannels == kDim &&
            frame.imapHeight > 0 && frame.imapWidth > 0 && patchRun.imapValueCount >=
                static_cast<size_t>(frame.imapChannels) * static_cast<size_t>(frame.imapHeight) *
                    static_cast<size_t>(frame.imapWidth)) {
            frame.patchImap.assign(static_cast<size_t>(m_patchesPerFrame) * kDim, 0.0f);
            for (int p = 0; p < m_patchesPerFrame; ++p) {
                SampleFeatureVector(patchRun.imapHost, frame.imapChannels, frame.imapHeight, frame.imapWidth,
                                    frame.patches[static_cast<size_t>(p)].x,
                                    frame.patches[static_cast<size_t>(p)].y,
                                    &frame.patchImap[static_cast<size_t>(p) * kDim]);
            }
        }
        if (!frame.fmap.empty() && frame.fmapChannels == kFmapChannels) {
            frame.patchGmap.assign(static_cast<size_t>(m_patchesPerFrame) * kFmapChannels * kPatchArea, 0.0f);
            for (int p = 0; p < m_patchesPerFrame; ++p) {
                SampleFeaturePatch3(frame.fmap.data(), frame.fmapChannels, frame.fmapHeight, frame.fmapWidth,
                                    frame.patches[static_cast<size_t>(p)].x,
                                    frame.patches[static_cast<size_t>(p)].y,
                                    &frame.patchGmap[static_cast<size_t>(p) * kFmapChannels * kPatchArea]);
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
        if (m_persistentEdges) {
            CapActiveEdges();
        }
        PruneFrames();
    }

    bool Initialized() const { return m_initialized; }
    int FrameCount() const { return static_cast<int>(m_frames.size()); }
    int EdgeCount() const { return static_cast<int>(m_edges.size()); }
    int PatchCount() const { return FrameCount() * m_patchesPerFrame; }
    int PatchesPerFrame() const { return m_patchesPerFrame; }
    int OptimizationWindow() const { return m_optimizationWindow; }
    int KeyframeRemovals() const { return m_keyframeRemovals; }
    const std::vector<DpvoFrameState> &Frames() const { return m_frames; }
    std::vector<DpvoFrameState> &MutableFrames() { return m_frames; }
    const std::vector<DpvoEdgeState> &Edges() const { return m_edges; }
    int LastStereoDepthUpdates() const { return m_lastStereoDepthUpdates; }
    void ApplyStereoDepthFromRightFmap(const DpvoPatchifierRun &rightRun, float fx, float baseline)
    {
        m_lastStereoDepthUpdates = 0;
        if (m_frames.empty() || !rightRun.ok || rightRun.fmapHost == nullptr || rightRun.fmapDims.nbDims != 4 ||
            !(fx > 0.0f) || !(baseline > 0.0f)) {
            return;
        }
        DpvoFrameState &frame = m_frames.back();
        const int rightChannels = static_cast<int>(rightRun.fmapDims.d[1]);
        const int rightHeight = static_cast<int>(rightRun.fmapDims.d[2]);
        const int rightWidth = static_cast<int>(rightRun.fmapDims.d[3]);
        const size_t expected = static_cast<size_t>(rightChannels) * static_cast<size_t>(rightHeight) *
                                static_cast<size_t>(rightWidth);
        if (rightChannels != kFmapChannels || rightHeight != frame.fmapHeight || rightWidth != frame.fmapWidth ||
            rightRun.fmapValueCount < expected || frame.patchGmap.size() <
                static_cast<size_t>(frame.patches.size()) * kFmapChannels * kPatchArea) {
            return;
        }

        int updated = 0;
        const int maxDisp = std::clamp(EnvIntValue("SMART_DRONE_DPVO_STEREO_MAX_DISP", 36), 2,
                                       std::max(2, rightWidth / 3));
        const float minScore =
            std::clamp(EnvFloatValue("SMART_DRONE_DPVO_STEREO_NCC_MIN", 0.05f), -1.0f, 1.0f);
        const float minMargin =
            std::clamp(EnvFloatValue("SMART_DRONE_DPVO_STEREO_NCC_MARGIN", 0.01f), 0.0f, 1.0f);
        for (int p = 0; p < static_cast<int>(frame.patches.size()); ++p) {
            DpvoPatchState &patch = frame.patches[static_cast<size_t>(p)];
            if (patch.x < 2.0f || patch.y < 2.0f || patch.x >= static_cast<float>(rightWidth - 2) ||
                patch.y >= static_cast<float>(rightHeight - 2)) {
                continue;
            }
            float bestScore = -std::numeric_limits<float>::infinity();
            float secondScore = -std::numeric_limits<float>::infinity();
            int bestDisp = 0;
            std::vector<float> scores(static_cast<size_t>(maxDisp + 1),
                                      -std::numeric_limits<float>::infinity());
            const int maxPatchDisp = std::min(maxDisp, std::max(1, static_cast<int>(std::floor(patch.x)) - 1));
            for (int disp = 1; disp <= maxPatchDisp; ++disp) {
                double ab = 0.0;
                double aa = 0.0;
                double bb = 0.0;
                const float x = patch.x - static_cast<float>(disp);
                const float y = patch.y;
                const size_t gmapOffset = static_cast<size_t>(p) * kFmapChannels * kPatchArea;
                for (int c = 0; c < kFmapChannels; ++c) {
                    for (int py = 0; py < kPatchSize; ++py) {
                        for (int px = 0; px < kPatchSize; ++px) {
                            const size_t k = static_cast<size_t>(py * kPatchSize + px);
                            const float a = frame.patchGmap[gmapOffset + static_cast<size_t>(c) * kPatchArea + k];
                            const float b = SampleFeatureBilinear(rightRun.fmapHost, rightChannels, rightHeight,
                                                                  rightWidth, c,
                                                                  x + static_cast<float>(px - kPatchRadius),
                                                                  y + static_cast<float>(py - kPatchRadius));
                            ab += static_cast<double>(a) * static_cast<double>(b);
                            aa += static_cast<double>(a) * static_cast<double>(a);
                            bb += static_cast<double>(b) * static_cast<double>(b);
                        }
                    }
                }
                const float score =
                    aa > 1e-9 && bb > 1e-9 ? static_cast<float>(ab / std::sqrt(aa * bb)) : -1.0f;
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
                (!std::isfinite(secondScore) || bestScore > secondScore + minMargin) &&
                bestScore > minScore) {
                float refinedDisp = static_cast<float>(bestDisp);
                if (bestDisp > 1 && bestDisp < maxPatchDisp) {
                    const float left = scores[static_cast<size_t>(bestDisp - 1)];
                    const float center = scores[static_cast<size_t>(bestDisp)];
                    const float right = scores[static_cast<size_t>(bestDisp + 1)];
                    const float denom = left - 2.0f * center + right;
                    if (std::isfinite(denom) && std::fabs(denom) > 1e-6f) {
                        refinedDisp += std::clamp(0.5f * (left - right) / denom, -0.5f, 0.5f);
                    }
                }
                patch.invDepth = std::clamp(refinedDisp / (fx * baseline), 1.0e-3f, 10.0f);
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
    bool FeatureMapsReady() const
    {
        return !m_frames.empty() &&
               std::all_of(m_frames.begin(), m_frames.end(), [](const DpvoFrameState &frame) {
                   return frame.patchImap.size() >= static_cast<size_t>(frame.patches.size()) * kDim &&
                          frame.patchGmap.size() >=
                              static_cast<size_t>(frame.patches.size()) * kFmapChannels * kPatchArea &&
                          !frame.fmap.empty() && !frame.fmapLevel4.empty();
               });
    }
    const DpvoFrameState *PreviousFrame() const
    {
        return m_frames.size() >= 2 ? &m_frames[m_frames.size() - 2] : nullptr;
    }
    const DpvoFrameState *NewestFrame() const
    {
        return m_frames.empty() ? nullptr : &m_frames.back();
    }
    bool MaybeRemoveKeyframe(const DpvoIntrinsics &intrinsics)
    {
        if (!m_keyframeRemovalEnabled || !m_initialized || FrameCount() <= kKeyframeIndex + 1) {
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
        const float m = MotionMagnitude(i, j, intrinsics) + MotionMagnitude(j, i, intrinsics);
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

  private:
    static float FeatureAt(const float *data, int channels, int height, int width, int c, int y, int x)
    {
        if (data == nullptr || c < 0 || c >= channels || y < 0 || y >= height || x < 0 || x >= width) {
            return 0.0f;
        }
        const size_t idx =
            (static_cast<size_t>(c) * static_cast<size_t>(height) + static_cast<size_t>(y)) *
                static_cast<size_t>(width) +
            static_cast<size_t>(x);
        return data[idx];
    }

    static float SampleFeatureBilinear(const float *data, int channels, int height, int width, int c, float x, float y)
    {
        if (data == nullptr || channels <= 0 || height <= 0 || width <= 0) {
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

    static void SampleFeatureVector(const float *data, int channels, int height, int width, float x, float y,
                                    float *out)
    {
        if (out == nullptr) {
            return;
        }
        for (int c = 0; c < channels; ++c) {
            out[c] = SampleFeatureBilinear(data, channels, height, width, c, x, y);
        }
    }

    static void SampleFeaturePatch3(const float *data, int channels, int height, int width, float x, float y,
                                    float *out)
    {
        if (out == nullptr) {
            return;
        }
        for (int c = 0; c < channels; ++c) {
            for (int py = 0; py < kPatchSize; ++py) {
                for (int px = 0; px < kPatchSize; ++px) {
                    const float sx = x + static_cast<float>(px - kPatchRadius);
                    const float sy = y + static_cast<float>(py - kPatchRadius);
                    const size_t idx =
                        (static_cast<size_t>(c) * kPatchArea) + static_cast<size_t>(py * kPatchSize + px);
                    out[idx] = SampleFeatureBilinear(data, channels, height, width, c, sx, sy);
                }
            }
        }
    }

    static std::vector<float> BuildPooledFmap(const std::vector<float> &src, int channels, int height, int width,
                                              int level)
    {
        if (src.empty() || channels <= 0 || height <= 0 || width <= 0 || level <= 1) {
            return src;
        }
        const int outHeight = height / level;
        const int outWidth = width / level;
        std::vector<float> dst(static_cast<size_t>(channels) * static_cast<size_t>(outHeight) *
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
                    dst[(static_cast<size_t>(c) * static_cast<size_t>(outHeight) + static_cast<size_t>(y)) *
                            static_cast<size_t>(outWidth) +
                        static_cast<size_t>(x)] = sum * denom;
                }
            }
        }
        return dst;
    }

    std::vector<DpvoPatchState> SelectPatches(const DpvoFrameState &frame, const cv::Mat &gray) const
    {
        std::vector<DpvoPatchState> patches;
        patches.reserve(static_cast<size_t>(m_patchesPerFrame));
        const int width = frame.fmapWidth > 2 ? frame.fmapWidth : std::max(3, gray.cols / 4);
        const int height = frame.fmapHeight > 2 ? frame.fmapHeight : std::max(3, gray.rows / 4);
        std::mt19937 rng(static_cast<uint32_t>(0x9e3779b9U ^ (m_counter * 0x85ebca6bU)));
        std::uniform_int_distribution<int> xDist(1, std::max(1, width - 2));
        std::uniform_int_distribution<int> yDist(1, std::max(1, height - 2));
        std::uniform_real_distribution<float> depthDist(0.2f, 1.0f);
        for (int i = 0; i < m_patchesPerFrame; ++i) {
            float invDepth = depthDist(rng);
            if (m_initialized && !m_frames.empty()) {
                invDepth = MedianRecentDepth();
            }
            patches.push_back({static_cast<float>(xDist(rng)), static_cast<float>(yDist(rng)), invDepth, invDepth, false});
        }
        return patches;
    }

    float MedianRecentDepth() const
    {
        std::vector<float> depths;
        const int start = std::max(0, FrameCount() - 3);
        for (int i = start; i < FrameCount(); ++i) {
            for (const DpvoPatchState &patch : m_frames[static_cast<size_t>(i)].patches) {
                if (std::isfinite(patch.invDepth) && patch.invDepth > 1e-3f && patch.invDepth < 10.0f) {
                    depths.push_back(patch.invDepth);
                }
            }
        }
        if (depths.empty()) {
            return 1.0f;
        }
        const size_t mid = depths.size() / 2;
        std::nth_element(depths.begin(), depths.begin() + static_cast<std::ptrdiff_t>(mid), depths.end());
        return std::clamp(depths[mid], 1e-3f, 10.0f);
    }

    static std::array<float, 2> ProjectPatchCenter(const DpvoFrameState &source, const DpvoFrameState &target,
                                                   const DpvoPatchState &patch, const Eigen::Matrix3f &overrideR,
                                                   bool useOverrideR, const DpvoIntrinsics &intrinsics,
                                                   bool *valid)
    {
        if (valid != nullptr) {
            *valid = false;
        }
        if (!(intrinsics.fx > 0.0f) || !(intrinsics.fy > 0.0f)) {
            return {patch.x, patch.y};
        }
        const Sophus::SE3f Tji = target.Tcw * source.Tcw.inverse();
        const Eigen::Matrix3f R = useOverrideR ? overrideR : Tji.so3().matrix();
        const Eigen::Vector3f t = Tji.translation();
        const Eigen::Vector3f Xi((patch.x - intrinsics.cx) / intrinsics.fx,
                                 (patch.y - intrinsics.cy) / intrinsics.fy, 1.0f);
        const Eigen::Vector3f Xj = R * Xi + patch.invDepth * t;
        if (!(Xj.z() > 0.2f) || !Xj.allFinite()) {
            return {patch.x, patch.y};
        }
        if (valid != nullptr) {
            *valid = true;
        }
        return {intrinsics.fx * (Xj.x() / Xj.z()) + intrinsics.cx,
                intrinsics.fy * (Xj.y() / Xj.z()) + intrinsics.cy};
    }

    float MotionMagnitude(int sourceFrame, int targetFrame, const DpvoIntrinsics &intrinsics) const
    {
        if (sourceFrame < 0 || targetFrame < 0 || sourceFrame >= FrameCount() || targetFrame >= FrameCount()) {
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
            const std::array<float, 2> full =
                ProjectPatchCenter(source, target, patch, identityR, false, intrinsics, &validFull);
            const std::array<float, 2> tonly =
                ProjectPatchCenter(source, target, patch, identityR, true, intrinsics, &validTonly);
            if (!validFull || !validTonly) {
                continue;
            }
            const float dxFull = full[0] - patch.x;
            const float dyFull = full[1] - patch.y;
            const float dxT = tonly[0] - patch.x;
            const float dyT = tonly[1] - patch.y;
            sum += 0.5 * std::sqrt(static_cast<double>(dxFull * dxFull + dyFull * dyFull)) +
                   0.5 * std::sqrt(static_cast<double>(dxT * dxT + dyT * dyT));
            ++count;
        }
        return count > 0 ? static_cast<float>(sum / static_cast<double>(count))
                         : std::numeric_limits<float>::infinity();
    }

    void AppendEdgesForNewest()
    {
        const int n = FrameCount();
        if (n <= 1) {
            return;
        }
        const int newest = n - 1;
        const int forwardPatchStart = m_patchesPerFrame * std::max(newest - m_patchLifetime, 0);
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

    void PruneOldEdges()
    {
        const int n = FrameCount();
        const int oldestActivePatchFrame = std::max(n - m_removalWindow, 0);
        m_edges.erase(std::remove_if(m_edges.begin(), m_edges.end(), [&](const DpvoEdgeState &edge) {
                          const int patchFrame = edge.patchGlobal / m_patchesPerFrame;
                          return edge.sourceFrame < 0 || edge.targetFrame < 0 || edge.sourceFrame >= n ||
                                 edge.targetFrame >= n || patchFrame < oldestActivePatchFrame || patchFrame >= n;
                      }),
                      m_edges.end());
    }

    void CapActiveEdges()
    {
        if (!m_persistentEdges || m_maxActiveEdges <= 0 || static_cast<int>(m_edges.size()) <= m_maxActiveEdges) {
            return;
        }
        const int removeCount = static_cast<int>(m_edges.size()) - m_maxActiveEdges;
        m_edges.erase(m_edges.begin(), m_edges.begin() + removeCount);
    }

    void RemoveFrameAt(int frameIndex)
    {
        if (frameIndex < 0 || frameIndex >= FrameCount()) {
            return;
        }
        m_frames.erase(m_frames.begin() + frameIndex);
        m_edges.erase(std::remove_if(m_edges.begin(), m_edges.end(), [&](DpvoEdgeState &edge) {
                          const int patchFrame = edge.patchGlobal / m_patchesPerFrame;
                          if (edge.sourceFrame == frameIndex || edge.targetFrame == frameIndex ||
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

    void PruneFrames()
    {
        const int maxFrames = std::max(36, m_removalWindow + m_optimizationWindow + 4);
        if (FrameCount() <= maxFrames) {
            return;
        }
        const int drop = FrameCount() - maxFrames;
        m_frames.erase(m_frames.begin(), m_frames.begin() + drop);
        m_edges.erase(std::remove_if(m_edges.begin(), m_edges.end(), [&](DpvoEdgeState &edge) {
                          const int patchFrame = edge.patchGlobal / m_patchesPerFrame;
                          if (edge.sourceFrame < drop || edge.targetFrame < drop || patchFrame < drop) {
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

    static constexpr int kInitializationFrames = 8;
    static constexpr int kDim = 384;
    static constexpr int kFmapChannels = 128;
    static constexpr int kPatchSize = 3;
    static constexpr int kPatchRadius = 1;
    static constexpr int kPatchArea = kPatchSize * kPatchSize;
    static constexpr int kKeyframeIndex = 4;
    static constexpr float kKeyframeThreshold = 15.0f;
    int m_patchesPerFrame{48};
    int m_optimizationWindow{7};
    int m_patchLifetime{11};
    int m_removalWindow{16};
    int m_maxActiveEdges{4096};
    int m_counter{0};
    int m_lastStereoDepthUpdates{0};
    int m_keyframeRemovals{0};
    bool m_persistentEdges{false};
    bool m_keyframeRemovalEnabled{false};
    bool m_initialized{false};
    std::vector<DpvoFrameState> m_frames;
    std::vector<DpvoEdgeState> m_edges;
};

struct DpvoEdgeKey {
    uint64_t sourceFrameId{0};
    uint64_t targetFrameId{0};
    int patchLocal{0};

    bool operator==(const DpvoEdgeKey &other) const
    {
        return sourceFrameId == other.sourceFrameId && targetFrameId == other.targetFrameId &&
               patchLocal == other.patchLocal;
    }
};

struct DpvoEdgeKeyHash {
    size_t operator()(const DpvoEdgeKey &key) const
    {
        uint64_t x = key.sourceFrameId + 0x9e3779b97f4a7c15ULL;
        x ^= key.targetFrameId + 0x9e3779b97f4a7c15ULL + (x << 6U) + (x >> 2U);
        x ^= static_cast<uint64_t>(key.patchLocal + 0x9e3779b9) + (x << 6U) + (x >> 2U);
        x ^= x >> 30U;
        x *= 0xbf58476d1ce4e5b9ULL;
        x ^= x >> 27U;
        x *= 0x94d049bb133111ebULL;
        x ^= x >> 31U;
        return static_cast<size_t>(x);
    }
};

class DpvoNativeSolver {
  public:
    void Reset()
    {
        m_netByEdge.clear();
        m_lastTcw = Sophus::SE3f();
        m_hasPose = false;
        m_bootstrapComplete = false;
        m_loggedCudaCorr = false;
        m_loggedCudaCorrFailure = false;
    }

    bool HasPose() const { return m_hasPose; }
    Sophus::SE3f LastTcw() const { return m_lastTcw; }

    bool Step(DpvoGraphState &graph, DpvoUpdatePreAggRuntime &preAggRuntime, TensorRtEngineHandle &preAggEngine,
              DpvoUpdatePostAggRuntime &postAggRuntime, TensorRtEngineHandle &postAggEngine,
              DpvoCudaKernelRuntime *cudaKernelRuntime, cudaStream_t stream, const DpvoIntrinsics &intrinsics,
              double *updateMs, std::string *err)
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

        const int updateIterations = m_bootstrapComplete ? 1 : 12;
        for (int updateIteration = 0; updateIteration < updateIterations; ++updateIteration) {
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
            std::vector<std::array<float, kPatchArea * 2>> coords(static_cast<size_t>(edgeCount));

            for (int e = 0; e < edgeCount; ++e) {
                const DpvoEdgeState &edge = edges[static_cast<size_t>(e)];
                const DpvoFrameState &source = frames[static_cast<size_t>(edge.sourceFrame)];
                const int patchLocal = edge.patchGlobal % graph.PatchesPerFrame();
                const DpvoEdgeKey key{source.frameId, frames[static_cast<size_t>(edge.targetFrame)].frameId, patchLocal};
                auto it = m_netByEdge.find(key);
                if (it != m_netByEdge.end() && it->second.size() == static_cast<size_t>(kDim)) {
                    std::copy(it->second.begin(), it->second.end(), net.begin() + static_cast<size_t>(e) * kDim);
                }
                const size_t imapOffset = static_cast<size_t>(patchLocal) * kDim;
                if (source.patchImap.size() >= imapOffset + kDim) {
                    std::copy(source.patchImap.begin() + static_cast<std::ptrdiff_t>(imapOffset),
                              source.patchImap.begin() + static_cast<std::ptrdiff_t>(imapOffset + kDim),
                              inp.begin() + static_cast<std::ptrdiff_t>(static_cast<size_t>(e) * kDim));
                }
                ReprojectPatch(frames, edge, graph.PatchesPerFrame(), intrinsics, coords[static_cast<size_t>(e)]);
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
                if (PackCorrelationCudaInputs(frames, edges, graph.PatchesPerFrame(), coords, &edgePatchGmap,
                                              &edgeCoords, &edgeTargetFrame, &fmapStorage, &fmapLevel4Storage,
                                              &fmapOffsets, &fmapHeights, &fmapWidths, &level4Offsets,
                                              &level4Heights, &level4Widths)) {
                    std::vector<float> cudaCorr;
                    std::string cudaCorrErr;
                    if (cudaKernelRuntime->ComputeCorrelationBatch(edgeCount, edgePatchGmap, edgeCoords,
                                                                   edgeTargetFrame, fmapStorage, fmapLevel4Storage,
                                                                   fmapOffsets, fmapHeights, fmapWidths,
                                                                   level4Offsets, level4Heights, level4Widths,
                                                                   stream, &cudaCorr, &cudaCorrErr)) {
                        if (!m_loggedCudaCorr ||
                            EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_CORR_VALIDATE_EVERY_STEP", false)) {
                            std::vector<float> cpuCorr(static_cast<size_t>(edgeCount) * kCorrDim, 0.0f);
                            for (int e = 0; e < edgeCount; ++e) {
                                ComputeCorrelation(frames, edges[static_cast<size_t>(e)], graph.PatchesPerFrame(),
                                                   coords[static_cast<size_t>(e)],
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
                            const double rmse = n > 0U ? std::sqrt(sq / static_cast<double>(n)) : 0.0;
                            std::cerr << "[dpvo_cuda] correlation batch ready edges=" << edgeCount
                                      << " values=" << n
                                      << " max_abs=" << maxAbs
                                      << " rmse=" << rmse << "\n";
                            m_loggedCudaCorr = true;
                            if (maxAbs > 1e-3f && EnvFlagEnabled("SMART_DRONE_DPVO_CUDA_CORR_STRICT_VALIDATE", true)) {
                                cudaCorr = std::move(cpuCorr);
                                std::cerr << "[dpvo_cuda] correlation validation exceeded threshold; using CPU corr for this step\n";
                            }
                        }
                        if (cudaCorr.size() == corr.size()) {
                            corr.swap(cudaCorr);
                            usedCudaCorrelation = true;
                        }
                    } else if (!m_loggedCudaCorrFailure) {
                        std::cerr << "[dpvo_cuda] correlation batch unavailable: " << cudaCorrErr
                                  << "; falling back to CPU correlation\n";
                        m_loggedCudaCorrFailure = true;
                    }
                } else if (!m_loggedCudaCorrFailure) {
                    std::cerr << "[dpvo_cuda] correlation batch input packing failed; falling back to CPU correlation\n";
                    m_loggedCudaCorrFailure = true;
                }
            }
            if (!usedCudaCorrelation) {
                for (int e = 0; e < edgeCount; ++e) {
                    ComputeCorrelation(frames, edges[static_cast<size_t>(e)], graph.PatchesPerFrame(),
                                       coords[static_cast<size_t>(e)],
                                       corr.data() + static_cast<size_t>(e) * kCorrDim);
                }
            }

            for (int e = 0; e < edgeCount; ++e) {
                if (prevEdge[static_cast<size_t>(e)] >= 0) {
                    prevMask[static_cast<size_t>(e)] = 1.0f;
                    const size_t src = static_cast<size_t>(prevEdge[static_cast<size_t>(e)]) * kDim;
                    std::copy(net.begin() + static_cast<std::ptrdiff_t>(src),
                              net.begin() + static_cast<std::ptrdiff_t>(src + kDim),
                              prevNet.begin() + static_cast<std::ptrdiff_t>(static_cast<size_t>(e) * kDim));
                }
                if (nextEdge[static_cast<size_t>(e)] >= 0) {
                    nextMask[static_cast<size_t>(e)] = 1.0f;
                    const size_t src = static_cast<size_t>(nextEdge[static_cast<size_t>(e)]) * kDim;
                    std::copy(net.begin() + static_cast<std::ptrdiff_t>(src),
                              net.begin() + static_cast<std::ptrdiff_t>(src + kDim),
                              nextNet.begin() + static_cast<std::ptrdiff_t>(static_cast<size_t>(e) * kDim));
                }
            }

            DpvoUpdatePreAggRun preAgg =
                preAggRuntime.Run(preAggEngine, stream, edgeCount, net, inp, corr, prevNet, nextNet, prevMask, nextMask, err);
            if (!preAgg.ok) {
                return false;
            }

            std::vector<int> groupKk(static_cast<size_t>(edgeCount), 0);
            std::vector<int> groupIj(static_cast<size_t>(edgeCount), 0);
            for (int e = 0; e < edgeCount; ++e) {
                const DpvoEdgeState &edge = edges[static_cast<size_t>(e)];
                groupKk[static_cast<size_t>(e)] = edge.patchGlobal;
                groupIj[static_cast<size_t>(e)] = edge.sourceFrame * 12345 + edge.targetFrame;
            }
            std::vector<float> aggKkY;
            std::vector<float> aggIjY;
            SoftAggExpand(preAgg.aggKkF, preAgg.aggKkG, groupKk, edgeCount, kDim, &aggKkY);
            SoftAggExpand(preAgg.aggIjF, preAgg.aggIjG, groupIj, edgeCount, kDim, &aggIjY);

            DpvoUpdatePostAggRun postAgg =
                postAggRuntime.Run(postAggEngine, stream, edgeCount, preAgg.baseNet, aggKkY, aggIjY, err);
            if (!postAgg.ok) {
                return false;
            }

            for (int e = 0; e < edgeCount; ++e) {
                const DpvoEdgeState &edge = edges[static_cast<size_t>(e)];
                const DpvoFrameState &source = frames[static_cast<size_t>(edge.sourceFrame)];
                const int patchLocal = edge.patchGlobal % graph.PatchesPerFrame();
                const DpvoEdgeKey key{source.frameId, frames[static_cast<size_t>(edge.targetFrame)].frameId, patchLocal};
                std::vector<float> &slot = m_netByEdge[key];
                slot.assign(postAgg.updatedNet.begin() + static_cast<std::ptrdiff_t>(static_cast<size_t>(e) * kDim),
                            postAgg.updatedNet.begin() +
                                static_cast<std::ptrdiff_t>((static_cast<size_t>(e) + 1U) * kDim));
            }
            PruneEdgeNet(frames, edges, graph.PatchesPerFrame());

            std::vector<std::array<float, 2>> target(static_cast<size_t>(edgeCount));
            for (int e = 0; e < edgeCount; ++e) {
                const std::array<float, kPatchArea * 2> &edgeCoords = coords[static_cast<size_t>(e)];
                target[static_cast<size_t>(e)] = {
                    edgeCoords[static_cast<size_t>(kPatchCenter) * 2U] + postAgg.delta[static_cast<size_t>(e) * 2U],
                    edgeCoords[static_cast<size_t>(kPatchCenter) * 2U + 1U] +
                        postAgg.delta[static_cast<size_t>(e) * 2U + 1U]};
            }
            const Sophus::SE3f beforeBaNewest = frames.back().Tcw;
            RunBundleAdjustment(frames, edges, graph.PatchesPerFrame(), graph.OptimizationWindow(), intrinsics, target,
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

  private:
    static constexpr int kDim = 384;
    static constexpr int kFmapChannels = 128;
    static constexpr int kPatchSize = 3;
    static constexpr int kPatchArea = kPatchSize * kPatchSize;
    static constexpr int kPatchCenter = 4;
    static constexpr int kCorrRadius = 3;
    static constexpr int kCorrSide = 2 * kCorrRadius + 1;
    static constexpr int kCorrDim = 2 * kCorrSide * kCorrSide * kPatchArea;

    static float FeatureAt(const std::vector<float> &data, int channels, int height, int width, int c, int y, int x)
    {
        if (data.empty() || c < 0 || c >= channels || y < 0 || y >= height || x < 0 || x >= width) {
            return 0.0f;
        }
        const size_t idx =
            (static_cast<size_t>(c) * static_cast<size_t>(height) + static_cast<size_t>(y)) *
                static_cast<size_t>(width) +
            static_cast<size_t>(x);
        return idx < data.size() ? data[idx] : 0.0f;
    }

    static float SampleFeatureBilinear(const std::vector<float> &data, int channels, int height, int width, int c,
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

    static void PredictNewestPose(std::vector<DpvoFrameState> &frames)
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

    static bool AcceptPoseStep(const Sophus::SE3f &reference, const Sophus::SE3f &candidate)
    {
        if (!EnvFlagEnabled("SMART_DRONE_DPVO_ACCEPT_GUARD", true)) {
            return true;
        }
        const Sophus::SE3f delta = candidate * reference.inverse();
        const Eigen::Matrix<float, 6, 1> xi = delta.log();
        const float maxTrans = std::max(0.0f, EnvFloatValue("SMART_DRONE_DPVO_ACCEPT_MAX_TRANS", 0.08f));
        const float maxRot = std::max(0.0f, EnvFloatValue("SMART_DRONE_DPVO_ACCEPT_MAX_ROT", 0.16f));
        return (maxTrans <= 0.0f || xi.template head<3>().norm() <= maxTrans) &&
               (maxRot <= 0.0f || xi.template tail<3>().norm() <= maxRot) &&
               std::isfinite(xi.norm());
    }

    static void BuildTemporalNeighbors(const std::vector<DpvoEdgeState> &edges, std::vector<int> *prevEdge,
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
                return edges[static_cast<size_t>(a)].targetFrame < edges[static_cast<size_t>(b)].targetFrame;
            });
            for (int i = 0; i < static_cast<int>(idx.size()); ++i) {
                (*prevEdge)[static_cast<size_t>(idx[static_cast<size_t>(i)])] = i > 0 ? idx[static_cast<size_t>(i - 1)] : -1;
                (*nextEdge)[static_cast<size_t>(idx[static_cast<size_t>(i)])] =
                    i + 1 < static_cast<int>(idx.size()) ? idx[static_cast<size_t>(i + 1)] : -1;
            }
        }
    }

    static void ReprojectPatch(const std::vector<DpvoFrameState> &frames, const DpvoEdgeState &edge,
                               int patchesPerFrame, const DpvoIntrinsics &intrinsics,
                               std::array<float, kPatchArea * 2> &coords)
    {
        coords.fill(0.0f);
        if (edge.sourceFrame < 0 || edge.targetFrame < 0 ||
            static_cast<size_t>(edge.sourceFrame) >= frames.size() ||
            static_cast<size_t>(edge.targetFrame) >= frames.size()) {
            return;
        }
        const DpvoFrameState &source = frames[static_cast<size_t>(edge.sourceFrame)];
        const DpvoFrameState &target = frames[static_cast<size_t>(edge.targetFrame)];
        const int patchLocal = edge.patchGlobal % patchesPerFrame;
        if (patchLocal < 0 || static_cast<size_t>(patchLocal) >= source.patches.size() ||
            !(intrinsics.fx > 0.0f) || !(intrinsics.fy > 0.0f)) {
            return;
        }
        const DpvoPatchState &patch = source.patches[static_cast<size_t>(patchLocal)];
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

    static void ComputeCorrelation(const std::vector<DpvoFrameState> &frames, const DpvoEdgeState &edge,
                                   int patchesPerFrame, const std::array<float, kPatchArea * 2> &coords,
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
        const DpvoFrameState &source = frames[static_cast<size_t>(edge.sourceFrame)];
        const DpvoFrameState &target = frames[static_cast<size_t>(edge.targetFrame)];
        const int patchLocal = edge.patchGlobal % patchesPerFrame;
        const size_t gmapOffset = static_cast<size_t>(patchLocal) * kFmapChannels * kPatchArea;
        if (source.patchGmap.size() < gmapOffset + kFmapChannels * kPatchArea || target.fmap.empty() ||
            target.fmapLevel4.empty()) {
            return;
        }

        for (int ox = 0; ox < kCorrSide; ++ox) {
            for (int oy = 0; oy < kCorrSide; ++oy) {
                const int dx = ox - kCorrRadius;
                const int dy = oy - kCorrRadius;
                for (int py = 0; py < kPatchSize; ++py) {
                    for (int px = 0; px < kPatchSize; ++px) {
                        const size_t coordIdx = static_cast<size_t>(py * kPatchSize + px) * 2U;
                        for (int levelIndex = 0; levelIndex < 2; ++levelIndex) {
                            const int level = levelIndex == 0 ? 1 : 4;
                            const std::vector<float> &targetMap = levelIndex == 0 ? target.fmap : target.fmapLevel4;
                            const int targetHeight = levelIndex == 0 ? target.fmapHeight : target.fmapHeight / 4;
                            const int targetWidth = levelIndex == 0 ? target.fmapWidth : target.fmapWidth / 4;
                            float dot = 0.0f;
                            const float sx = coords[coordIdx] / static_cast<float>(level) + static_cast<float>(dx);
                            const float sy = coords[coordIdx + 1U] / static_cast<float>(level) + static_cast<float>(dy);
                            for (int c = 0; c < kFmapChannels; ++c) {
                                const size_t gidx =
                                    gmapOffset + static_cast<size_t>(c) * kPatchArea +
                                    static_cast<size_t>(py * kPatchSize + px);
                                const float a = source.patchGmap[gidx];
                                const float b = SampleFeatureBilinear(targetMap, kFmapChannels, targetHeight,
                                                                      targetWidth, c, sx, sy);
                                dot += a * b;
                            }
                            const size_t outIdx =
                                (((static_cast<size_t>(ox) * kCorrSide + static_cast<size_t>(oy)) * kPatchSize +
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

    static bool PackCorrelationCudaInputs(const std::vector<DpvoFrameState> &frames,
                                          const std::vector<DpvoEdgeState> &edges,
                                          int patchesPerFrame,
                                          const std::vector<std::array<float, kPatchArea * 2>> &coords,
                                          std::vector<float> *edgePatchGmap,
                                          std::vector<float> *edgeCoords,
                                          std::vector<int> *edgeTargetFrame,
                                          std::vector<float> *fmapStorage,
                                          std::vector<float> *fmapLevel4Storage,
                                          std::vector<int> *fmapOffsets,
                                          std::vector<int> *fmapHeights,
                                          std::vector<int> *fmapWidths,
                                          std::vector<int> *level4Offsets,
                                          std::vector<int> *level4Heights,
                                          std::vector<int> *level4Widths)
    {
        if (edgePatchGmap == nullptr || edgeCoords == nullptr || edgeTargetFrame == nullptr ||
            fmapStorage == nullptr || fmapLevel4Storage == nullptr || fmapOffsets == nullptr ||
            fmapHeights == nullptr || fmapWidths == nullptr || level4Offsets == nullptr ||
            level4Heights == nullptr || level4Widths == nullptr || patchesPerFrame <= 0 ||
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
            if (frame.fmapChannels != kFmapChannels || frame.fmap.empty() || frame.fmapLevel4.empty() ||
                frame.fmapHeight <= 0 || frame.fmapWidth <= 0) {
                return false;
            }
            (*fmapOffsets)[i] = static_cast<int>(fmapStorage->size());
            (*fmapHeights)[i] = frame.fmapHeight;
            (*fmapWidths)[i] = frame.fmapWidth;
            fmapStorage->insert(fmapStorage->end(), frame.fmap.begin(), frame.fmap.end());
            (*level4Offsets)[i] = static_cast<int>(fmapLevel4Storage->size());
            (*level4Heights)[i] = frame.fmapHeight / 4;
            (*level4Widths)[i] = frame.fmapWidth / 4;
            fmapLevel4Storage->insert(fmapLevel4Storage->end(), frame.fmapLevel4.begin(), frame.fmapLevel4.end());
        }
        for (size_t e = 0; e < edgeCount; ++e) {
            const DpvoEdgeState &edge = edges[e];
            if (edge.sourceFrame < 0 || edge.targetFrame < 0 ||
                static_cast<size_t>(edge.sourceFrame) >= frames.size() ||
                static_cast<size_t>(edge.targetFrame) >= frames.size()) {
                return false;
            }
            const DpvoFrameState &source = frames[static_cast<size_t>(edge.sourceFrame)];
            const int patchLocal = edge.patchGlobal % patchesPerFrame;
            if (patchLocal < 0) {
                return false;
            }
            const size_t gmapOffset = static_cast<size_t>(patchLocal) * kFmapChannels * kPatchArea;
            if (source.patchGmap.size() < gmapOffset + kFmapChannels * kPatchArea) {
                return false;
            }
            std::copy(source.patchGmap.begin() + static_cast<std::ptrdiff_t>(gmapOffset),
                      source.patchGmap.begin() +
                          static_cast<std::ptrdiff_t>(gmapOffset + kFmapChannels * kPatchArea),
                      edgePatchGmap->begin() +
                          static_cast<std::ptrdiff_t>(e * kFmapChannels * kPatchArea));
            std::copy(coords[e].begin(), coords[e].end(),
                      edgeCoords->begin() + static_cast<std::ptrdiff_t>(e * kPatchArea * 2U));
            (*edgeTargetFrame)[e] = edge.targetFrame;
        }
        return !fmapStorage->empty() && !fmapLevel4Storage->empty();
    }

    static void SoftAggExpand(const std::vector<float> &f, const std::vector<float> &g,
                              const std::vector<int> &groupIds, int edgeCount, int dim, std::vector<float> *out)
    {
        if (out == nullptr) {
            return;
        }
        out->assign(static_cast<size_t>(std::max(0, edgeCount)) * static_cast<size_t>(dim), 0.0f);
        if (edgeCount <= 0 || dim <= 0 || f.size() < static_cast<size_t>(edgeCount) * dim ||
            g.size() < static_cast<size_t>(edgeCount) * dim || groupIds.size() < static_cast<size_t>(edgeCount)) {
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
                    maxLogit = std::max(maxLogit, g[static_cast<size_t>(e) * dim + static_cast<size_t>(c)]);
                }
                double denom = 0.0;
                double accum = 0.0;
                for (int e : idx) {
                    const float logit = g[static_cast<size_t>(e) * dim + static_cast<size_t>(c)];
                    const double w = std::exp(static_cast<double>(logit - maxLogit));
                    denom += w;
                    accum += static_cast<double>(f[static_cast<size_t>(e) * dim + static_cast<size_t>(c)]) * w;
                }
                const float value = denom > 0.0 ? static_cast<float>(accum / denom) : 0.0f;
                for (int e : idx) {
                    (*out)[static_cast<size_t>(e) * dim + static_cast<size_t>(c)] = value;
                }
            }
        }
    }

    static void AdjSE3(const Eigen::Vector3f &t, const Eigen::Matrix3f &R, const Eigen::Matrix<float, 6, 1> &x,
                       Eigen::Matrix<float, 6, 1> *y)
    {
        if (y == nullptr) {
            return;
        }
        const Eigen::Matrix3f Rt = R.transpose();
        y->template head<3>() = Rt * x.template head<3>();
        const Eigen::Vector3f u = x.template head<3>().cross(t);
        y->template tail<3>() = Rt * x.template tail<3>() + Rt * u;
    }

    static void AddBlock(Eigen::MatrixXf &H, int row, int col, const Eigen::VectorXf &a, const Eigen::VectorXf &b,
                         float scale)
    {
        if (row < 0 || col < 0) {
            return;
        }
        H.block(row, col, a.size(), b.size()).noalias() += scale * (a * b.transpose());
    }

    static void AddVector(Eigen::VectorXf &v, int row, const Eigen::VectorXf &a, float scale)
    {
        if (row < 0) {
            return;
        }
        v.segment(row, a.size()).noalias() += scale * a;
    }

    static void RunBundleAdjustment(std::vector<DpvoFrameState> &frames, const std::vector<DpvoEdgeState> &edges,
                                    int patchesPerFrame, int optimizationWindow, const DpvoIntrinsics &intrinsics,
                                    const std::vector<std::array<float, 2>> &target,
                                    const std::vector<float> &weight)
    {
        const int frameCount = static_cast<int>(frames.size());
        const int edgeCount = static_cast<int>(edges.size());
        if (frameCount < 2 || edgeCount <= 0 || target.size() < edges.size() ||
            weight.size() < static_cast<size_t>(edgeCount) * 2U || !(intrinsics.fx > 0.0f) ||
            !(intrinsics.fy > 0.0f)) {
            return;
        }
        const int poseStart = std::max(1, frameCount - std::max(1, optimizationWindow));
        const int poseEnd = frameCount;
        const int poseVars = std::max(0, poseEnd - poseStart);
        std::unordered_map<int, int> patchVar;
        patchVar.reserve(edges.size());
        for (const DpvoEdgeState &edge : edges) {
            patchVar.emplace(edge.patchGlobal, static_cast<int>(patchVar.size()));
        }
        const int depthVars = static_cast<int>(patchVar.size());
        const int poseDim = 6 * poseVars;
        const int totalDim = poseDim + depthVars;
        if (totalDim <= 0 || totalDim > 900) {
            return;
        }
        const bool directSolve = EnvFlagEnabled("SMART_DRONE_DPVO_DIRECT_BA", false);

        for (int itr = 0; itr < 2; ++itr) {
            Eigen::MatrixXf B = Eigen::MatrixXf::Zero(poseDim, poseDim);
            Eigen::MatrixXf E = Eigen::MatrixXf::Zero(poseDim, depthVars);
            Eigen::VectorXf C = Eigen::VectorXf::Zero(depthVars);
            Eigen::VectorXf v = Eigen::VectorXf::Zero(poseDim);
            Eigen::VectorXf u = Eigen::VectorXf::Zero(depthVars);

            for (int e = 0; e < edgeCount; ++e) {
                const DpvoEdgeState &edge = edges[static_cast<size_t>(e)];
                if (edge.sourceFrame < 0 || edge.targetFrame < 0 || edge.sourceFrame >= frameCount ||
                    edge.targetFrame >= frameCount) {
                    continue;
                }
                DpvoFrameState &source = frames[static_cast<size_t>(edge.sourceFrame)];
                DpvoFrameState &targetFrame = frames[static_cast<size_t>(edge.targetFrame)];
                const int patchLocal = edge.patchGlobal % patchesPerFrame;
                if (patchLocal < 0 || static_cast<size_t>(patchLocal) >= source.patches.size()) {
                    continue;
                }
                const DpvoPatchState &patch = source.patches[static_cast<size_t>(patchLocal)];
                const Sophus::SE3f Tji = targetFrame.Tcw * source.Tcw.inverse();
                const Eigen::Matrix3f R = Tji.so3().matrix();
                const Eigen::Vector3f t = Tji.translation();
                const Eigen::Vector3f Xi((patch.x - intrinsics.cx) / intrinsics.fx,
                                         (patch.y - intrinsics.cy) / intrinsics.fy, 1.0f);
                const Eigen::Vector3f Xj = R * Xi + patch.invDepth * t;
                const float X = Xj.x();
                const float Y = Xj.y();
                const float Z = Xj.z();
                const float W = patch.invDepth;
                if (!(Z > 0.2f) || !std::isfinite(Z)) {
                    continue;
                }
                const float invZ = 1.0f / Z;
                const float invZ2 = invZ * invZ;
                const float x1 = intrinsics.fx * X * invZ + intrinsics.cx;
                const float y1 = intrinsics.fy * Y * invZ + intrinsics.cy;
                const float rx = target[static_cast<size_t>(e)][0] - x1;
                const float ry = target[static_cast<size_t>(e)][1] - y1;
                const bool inBounds = std::sqrt(rx * rx + ry * ry) < 128.0f && x1 > -64.0f && y1 > -64.0f &&
                                      x1 < 2.0f * intrinsics.cx + 64.0f &&
                                      y1 < 2.0f * intrinsics.cy + 64.0f;
                if (!inBounds) {
                    continue;
                }

                const int srcPoseBase =
                    edge.sourceFrame >= poseStart ? 6 * (edge.sourceFrame - poseStart) : -1;
                const int dstPoseBase =
                    edge.targetFrame >= poseStart ? 6 * (edge.targetFrame - poseStart) : -1;
                const int depthBase = poseDim + patchVar[edge.patchGlobal];

                for (int row = 0; row < 2; ++row) {
                    const float residual = row == 0 ? rx : ry;
                    const float w = std::clamp(weight[static_cast<size_t>(e) * 2U + static_cast<size_t>(row)],
                                               0.0f, 1.0f);
                    if (!(w > 1e-6f)) {
                        continue;
                    }
                    Eigen::Matrix<float, 6, 1> Jj;
                    float Jz = 0.0f;
                    if (row == 0) {
                        Jz = intrinsics.fx * (t.x() * invZ - t.z() * (X * invZ2));
                        Jj << intrinsics.fx * W * invZ, 0.0f, intrinsics.fx * -X * W * invZ2,
                            intrinsics.fx * -X * Y * invZ2, intrinsics.fx * (1.0f + X * X * invZ2),
                            intrinsics.fx * -Y * invZ;
                    } else {
                        Jz = intrinsics.fy * (t.y() * invZ - t.z() * (Y * invZ2));
                        Jj << 0.0f, intrinsics.fy * W * invZ, intrinsics.fy * -Y * W * invZ2,
                            intrinsics.fy * (-1.0f - Y * Y * invZ2), intrinsics.fy * (X * Y * invZ2),
                            intrinsics.fy * X * invZ;
                    }
                    Eigen::Matrix<float, 6, 1> Ji;
                    AdjSE3(t, R, Jj, &Ji);

                    AddBlock(B, srcPoseBase, srcPoseBase, Ji, Ji, w);
                    AddBlock(B, dstPoseBase, dstPoseBase, Jj, Jj, w);
                    AddBlock(B, srcPoseBase, dstPoseBase, Ji, Jj, -w);
                    AddBlock(B, dstPoseBase, srcPoseBase, Jj, Ji, -w);

                    if (srcPoseBase >= 0) {
                        E.block(srcPoseBase, depthBase - poseDim, 6, 1).noalias() += -w * Jz * Ji;
                    }
                    if (dstPoseBase >= 0) {
                        E.block(dstPoseBase, depthBase - poseDim, 6, 1).noalias() += w * Jz * Jj;
                    }
                    C(depthBase - poseDim) += w * Jz * Jz;

                    AddVector(v, srcPoseBase, Ji, -w * residual);
                    AddVector(v, dstPoseBase, Jj, w * residual);
                    u(depthBase - poseDim) += w * residual * Jz;
                }
            }
            const float stereoDepthPriorWeight = EnvFloatValue("SMART_DRONE_DPVO_STEREO_DEPTH_PRIOR", 0.0f);
            if (stereoDepthPriorWeight > 0.0f) {
                for (const auto &entry : patchVar) {
                    const int patchGlobal = entry.first;
                    const int sourceFrame = patchGlobal / patchesPerFrame;
                    const int patchLocal = patchGlobal % patchesPerFrame;
                    if (sourceFrame < 0 || sourceFrame >= frameCount || patchLocal < 0 ||
                        static_cast<size_t>(patchLocal) >= frames[static_cast<size_t>(sourceFrame)].patches.size()) {
                        continue;
                    }
                    const DpvoPatchState &patch =
                        frames[static_cast<size_t>(sourceFrame)].patches[static_cast<size_t>(patchLocal)];
                    if (!patch.hasStereoPrior || !std::isfinite(patch.stereoPriorInvDepth)) {
                        continue;
                    }
                    C(entry.second) += stereoDepthPriorWeight;
                    u(entry.second) += stereoDepthPriorWeight * (patch.stereoPriorInvDepth - patch.invDepth);
                }
            }

            Eigen::VectorXf dxPose = Eigen::VectorXf::Zero(poseDim);
            Eigen::VectorXf dxDepth = Eigen::VectorXf::Zero(depthVars);
            if (directSolve) {
                Eigen::MatrixXf H = Eigen::MatrixXf::Zero(totalDim, totalDim);
                Eigen::VectorXf rhs = Eigen::VectorXf::Zero(totalDim);
                if (poseDim > 0) {
                    H.block(0, 0, poseDim, poseDim) = B;
                    H.block(0, poseDim, poseDim, depthVars) = E;
                    H.block(poseDim, 0, depthVars, poseDim) = E.transpose();
                    rhs.head(poseDim) = v;
                }
                H.block(poseDim, poseDim, depthVars, depthVars) = C.asDiagonal();
                rhs.tail(depthVars) = u;
                for (int i = 0; i < totalDim; ++i) {
                    const float base = i < poseDim ? 1.0f : 1e-4f;
                    H(i, i) += 1e-4f * std::max(std::fabs(H(i, i)), 1.0f) + base;
                }
                const Eigen::VectorXf dx = H.ldlt().solve(rhs);
                if (dx.size() != totalDim || !dx.allFinite()) {
                    return;
                }
                if (poseDim > 0) {
                    dxPose = dx.head(poseDim);
                }
                dxDepth = dx.tail(depthVars);
            } else {
                const Eigen::VectorXf Q = (C.array() + 1e-4f).inverse().matrix();
                if (poseDim > 0) {
                    const Eigen::MatrixXf EQ = E * Q.asDiagonal();
                    Eigen::MatrixXf S = B - EQ * E.transpose();
                    Eigen::VectorXf y = v - EQ * u;
                    for (int i = 0; i < poseDim; ++i) {
                        S(i, i) += 1e-4f * S(i, i) + 1.0f;
                    }
                    dxPose = S.ldlt().solve(y);
                    if (dxPose.size() != poseDim || !dxPose.allFinite()) {
                        return;
                    }
                    dxDepth = Q.asDiagonal() * (u - E.transpose() * dxPose);
                } else {
                    dxDepth = Q.asDiagonal() * u;
                }
            }
            if (dxDepth.size() != depthVars || !dxDepth.allFinite()) {
                return;
            }
            for (int i = 0; i < poseVars; ++i) {
                Eigen::Matrix<float, 6, 1> xi = dxPose.segment<6>(6 * i);
                const float maxTransStep =
                    std::max(0.0f, EnvFloatValue("SMART_DRONE_DPVO_BA_MAX_TRANS_STEP", 0.03f));
                const float maxRotStep =
                    std::max(0.0f, EnvFloatValue("SMART_DRONE_DPVO_BA_MAX_ROT_STEP", 0.08f));
                const float transNorm = xi.template head<3>().norm();
                const float rotNorm = xi.template tail<3>().norm();
                if (maxTransStep > 0.0f && transNorm > maxTransStep) {
                    xi.template head<3>() *= (maxTransStep / std::max(transNorm, 1e-6f));
                }
                if (maxRotStep > 0.0f && rotNorm > maxRotStep) {
                    xi.template tail<3>() *= (maxRotStep / std::max(rotNorm, 1e-6f));
                }
                frames[static_cast<size_t>(poseStart + i)].Tcw =
                    Sophus::SE3f::exp(xi) * frames[static_cast<size_t>(poseStart + i)].Tcw;
            }
            for (const auto &entry : patchVar) {
                const int patchGlobal = entry.first;
                const int sourceFrame = patchGlobal / patchesPerFrame;
                const int patchLocal = patchGlobal % patchesPerFrame;
                if (sourceFrame < 0 || sourceFrame >= frameCount || patchLocal < 0 ||
                    static_cast<size_t>(patchLocal) >= frames[static_cast<size_t>(sourceFrame)].patches.size()) {
                    continue;
                }
                float &depth = frames[static_cast<size_t>(sourceFrame)].patches[static_cast<size_t>(patchLocal)].invDepth;
                depth += dxDepth(entry.second);
                depth = std::clamp(depth, 1e-3f, 10.0f);
            }
        }
    }

    void PruneEdgeNet(const std::vector<DpvoFrameState> &frames, const std::vector<DpvoEdgeState> &edges,
                      int patchesPerFrame)
    {
        std::unordered_map<DpvoEdgeKey, bool, DpvoEdgeKeyHash> active;
        active.reserve(edges.size());
        for (const DpvoEdgeState &edge : edges) {
            if (edge.sourceFrame < 0 || edge.targetFrame < 0 ||
                static_cast<size_t>(edge.sourceFrame) >= frames.size() ||
                static_cast<size_t>(edge.targetFrame) >= frames.size()) {
                continue;
            }
            active.emplace(DpvoEdgeKey{frames[static_cast<size_t>(edge.sourceFrame)].frameId,
                                       frames[static_cast<size_t>(edge.targetFrame)].frameId,
                                       edge.patchGlobal % patchesPerFrame},
                           true);
        }
        for (auto it = m_netByEdge.begin(); it != m_netByEdge.end();) {
            if (active.find(it->first) == active.end()) {
                it = m_netByEdge.erase(it);
            } else {
                ++it;
            }
        }
    }

    std::unordered_map<DpvoEdgeKey, std::vector<float>, DpvoEdgeKeyHash> m_netByEdge;
    Sophus::SE3f m_lastTcw{Sophus::SE3f()};
    bool m_hasPose{false};
    bool m_bootstrapComplete{false};
    bool m_loggedCudaCorr{false};
    bool m_loggedCudaCorrFailure{false};
};

} // namespace

struct DpvoTensorRtEngine::Impl {
    explicit Impl(DpvoTensorRtConfig cfg) : config(std::move(cfg)) {}

    bool Start()
    {
        const std::filesystem::path patchPath =
            ResolveEnginePath(config.patchEnginePath, config.repoPath,
                              {"dpvo_patchifier_fp16.engine", "dpvo_patchifier.engine", "dpvo_patch.engine"});
        const std::filesystem::path updatePath =
            ResolveEnginePath(config.updateEnginePath, config.repoPath,
                              {"dpvo_update_fp16.engine", "dpvo_update.engine"});
        const std::filesystem::path updatePreAggPath =
            ResolveEnginePath(std::string{}, config.repoPath,
                              {"dpvo_update_preagg_fp16.engine", "dpvo_update_preagg.engine"});
        const std::filesystem::path updatePostAggPath =
            ResolveEnginePath(std::string{}, config.repoPath,
                              {"dpvo_update_postagg_fp16.engine", "dpvo_update_postagg.engine"});
        if (patchPath.empty() || updatePath.empty()) {
            std::cerr << "[dpvo_trt] missing engine(s): patch='" << config.patchEnginePath
                      << "' update='" << config.updateEnginePath << "' repo='" << config.repoPath << "'\n";
            return false;
        }

        std::string err;
        if (!patchEngine.Load(patchPath, "DPVO patchifier", &err)) {
            std::cerr << "[dpvo_trt] " << err << "\n";
            return false;
        }
        if (!updateEngine.Load(updatePath, "DPVO update", &err)) {
            std::cerr << "[dpvo_trt] " << err << "\n";
            return false;
        }
        softAggSplitReady = false;
        if (!updatePreAggPath.empty() && !updatePostAggPath.empty()) {
            if (!updatePreAggEngine.Load(updatePreAggPath, "DPVO update preagg", &err)) {
                std::cerr << "[dpvo_trt] " << err << "\n";
                return false;
            }
            if (!updatePostAggEngine.Load(updatePostAggPath, "DPVO update postagg", &err)) {
                std::cerr << "[dpvo_trt] " << err << "\n";
                return false;
            }
            softAggSplitReady = true;
        } else {
            std::cerr << "[dpvo_trt] split SoftAgg engines not found under repo='" << config.repoPath
                      << "'; using compatibility update warmup only\n";
        }
        if (!cudaStream.Create(&err)) {
            std::cerr << "[dpvo_trt] " << err << "\n";
            return false;
        }
        cudaKernelReady = false;
        std::string cudaKernelErr;
        if (cudaKernelRuntime.Initialize(cudaStream.stream, &cudaKernelErr)) {
            cudaKernelReady = true;
            std::cerr << "[dpvo_cuda] native CUDA kernels ready smoke_expected="
                      << cudaKernelRuntime.SmokeExpected()
                      << " smoke_got=" << cudaKernelRuntime.SmokeGot() << "\n";
        } else {
            std::cerr << "[dpvo_cuda] native CUDA kernels unavailable: " << cudaKernelErr << "\n";
        }
        if (!patchifierRuntime.Initialize(patchEngine, config.inputWidth, config.inputHeight, &err)) {
            std::cerr << "[dpvo_trt] " << err << "\n";
            return false;
        }
        if (!patchifierRightRuntime.Initialize(patchEngine, config.inputWidth, config.inputHeight, &err)) {
            std::cerr << "[dpvo_trt] right " << err << "\n";
            return false;
        }
        if (!updateRuntime.Initialize(updateEngine, &err)) {
            std::cerr << "[dpvo_trt] " << err << "\n";
            return false;
        }
        if (softAggSplitReady) {
            if (!updatePreAggRuntime.Initialize(updatePreAggEngine, &err)) {
                std::cerr << "[dpvo_trt] " << err << "\n";
                return false;
            }
            if (!updatePostAggRuntime.Initialize(updatePostAggEngine, &err)) {
                std::cerr << "[dpvo_trt] " << err << "\n";
                return false;
            }
        }
        graphState.Reset(config.patchesPerFrame, config.optimizationWindow);
        DpvoUpdateRun updateWarmup = updateRuntime.Warmup(updateEngine, cudaStream.stream,
                                                          std::max(1, config.patchesPerFrame), &err);
        if (!updateWarmup.ok) {
            std::cerr << "[dpvo_trt] update warmup failed: " << err << "\n";
            return false;
        }
        DpvoUpdateRun preAggWarmup{};
        DpvoUpdateRun postAggWarmup{};
        if (softAggSplitReady) {
            preAggWarmup = updatePreAggRuntime.Warmup(updatePreAggEngine, cudaStream.stream,
                                                      std::max(1, config.patchesPerFrame), &err);
            if (!preAggWarmup.ok) {
                std::cerr << "[dpvo_trt] update-preagg warmup failed: " << err << "\n";
                return false;
            }
            postAggWarmup = updatePostAggRuntime.Warmup(updatePostAggEngine, cudaStream.stream,
                                                        std::max(1, config.patchesPerFrame), &err);
            if (!postAggWarmup.ok) {
                std::cerr << "[dpvo_trt] update-postagg warmup failed: " << err << "\n";
                return false;
            }
        }
        nativeSolver.Reset();
        haveLastPose = false;
        lastPose = core::ports::PoseEstimate{};
        loggedKeyframeRemovals = 0;
        running = true;
        std::cerr << "[dpvo_trt] ready patch_engine=" << patchEngine.Path()
                  << " update_engine=" << updateEngine.Path()
                  << " update_preagg_engine=" << (softAggSplitReady ? updatePreAggEngine.Path() : std::string{"none"})
                  << " update_postagg_engine=" << (softAggSplitReady ? updatePostAggEngine.Path() : std::string{"none"})
                  << " input=" << config.inputWidth << "x" << config.inputHeight
                  << " patches=" << config.patchesPerFrame
                  << " opt_window=" << config.optimizationWindow
                  << " update_warmup_ms=" << updateWarmup.elapsedMs
                  << " preagg_warmup_ms=" << preAggWarmup.elapsedMs
                  << " postagg_warmup_ms=" << postAggWarmup.elapsedMs
                  << " native_cuda_kernels=" << (cudaKernelReady ? 1 : 0)
                  << " native_dpvo=1\n";
        if (!voState.LoadStereoCalibration(config.settingsPath)) {
            std::cerr << "[dpvo_trt] DPVO calibration unavailable settings='" << config.settingsPath
                      << "'; pose output disabled\n";
            running = false;
            return false;
        }
        voState.ResetTrackingState();
        return true;
    }

    void Stop()
    {
        running = false;
        nativeSolver.Reset();
        cudaKernelReady = false;
        cudaStream.Reset();
    }

    core::ports::SlamOutput Process(const core::ports::SlamInputBatch &input, bool extractFeatures,
                                    bool extractPointCloud)
    {
        (void)extractPointCloud;
        const auto start = std::chrono::steady_clock::now();
        core::ports::SlamOutput out{};
        out.frameId = input.frameId;
        out.captureTimestampNs = input.captureTimestampNs;
        out.mapId = 1;
        out.usedSuperPointFrontend = false;

        if (!running) {
            out.trackingState = ORB_SLAM3::Tracking::LOST;
            return out;
        }

        const auto prepareStart = std::chrono::steady_clock::now();
        cv::Mat leftGray = EnsureGray8(input.stereo.left.gray);
        cv::Mat rightGray = EnsureGray8(input.stereo.right.gray);
        if (leftGray.empty() || rightGray.empty()) {
            out.trackingState = ORB_SLAM3::Tracking::LOST;
            return out;
        }
        cv::Mat leftRect = leftGray;
        cv::Mat rightRect = rightGray;
        voState.EnsureStereoRectifier(leftGray.size());
        if (!voState.m_lkMap1x.empty() && !voState.m_lkMap2x.empty()) {
            const auto rectifyStart = std::chrono::steady_clock::now();
            cv::remap(leftGray, leftRect, voState.m_lkMap1x, voState.m_lkMap1y, cv::INTER_LINEAR);
            cv::remap(rightGray, rightRect, voState.m_lkMap2x, voState.m_lkMap2y, cv::INTER_LINEAR);
            out.lkRectifyMs = ElapsedMs(rectifyStart, std::chrono::steady_clock::now());
        }
        out.inputPrepareMs = ElapsedMs(prepareStart, std::chrono::steady_clock::now());

        if (leftRect.cols != config.inputWidth || leftRect.rows != config.inputHeight) {
            cv::resize(leftRect, resizedGray, cv::Size(config.inputWidth, config.inputHeight), 0.0, 0.0,
                       cv::INTER_AREA);
        } else {
            resizedGray = leftRect;
        }
        if (rightRect.cols != config.inputWidth || rightRect.rows != config.inputHeight) {
            cv::resize(rightRect, resizedRightGray, cv::Size(config.inputWidth, config.inputHeight), 0.0, 0.0,
                       cv::INTER_AREA);
        } else {
            resizedRightGray = rightRect;
        }

        std::string dpvoErr;
        std::string rightDpvoErr;
        const DpvoPatchifierRun patchRun =
            patchifierRuntime.Run(resizedGray, patchEngine, cudaStream.stream, true, true, &dpvoErr);
        const DpvoPatchifierRun rightPatchRun =
            patchifierRightRuntime.Run(resizedRightGray, patchEngine, cudaStream.stream, true, false, &rightDpvoErr);
        if (patchRun.ok) {
            out.superpointForwardMs = patchRun.elapsedMs;
            out.superpointStereoMatchMs = rightPatchRun.ok ? rightPatchRun.elapsedMs : 0.0;
            if (!loggedPatchifierShape) {
                std::cerr << "[dpvo_trt] patchifier active fmap=" << DimsToString(patchRun.fmapDims)
                          << " imap=" << DimsToString(patchRun.imapDims)
                          << " ms=" << patchRun.elapsedMs << "\n";
                loggedPatchifierShape = true;
            }
        } else if (!loggedPatchifierError) {
            std::cerr << "[dpvo_trt] patchifier inference disabled for this frame: " << dpvoErr << "\n";
            loggedPatchifierError = true;
        }
        if (!rightPatchRun.ok && !loggedRightPatchifierError) {
            std::cerr << "[dpvo_trt] right patchifier inference disabled for this frame: " << rightDpvoErr << "\n";
            loggedRightPatchifierError = true;
        }
        const float scaleX = static_cast<float>(config.inputWidth) / std::max(1, leftRect.cols);
        const float scaleY = static_cast<float>(config.inputHeight) / std::max(1, leftRect.rows);
        const DpvoIntrinsics intrinsics{voState.m_lkFx * scaleX * 0.25f, voState.m_lkFy * scaleY * 0.25f,
                                        voState.m_lkCx * scaleX * 0.25f, voState.m_lkCy * scaleY * 0.25f};
        graphState.PushFrame(input.frameId, input.captureTimestampNs, resizedGray,
                             nativeSolver.HasPose() ? nativeSolver.LastTcw() : Sophus::SE3f{}, patchRun);
        if (rightPatchRun.ok) {
            graphState.ApplyStereoDepthFromRightFmap(rightPatchRun, intrinsics.fx, voState.m_lkBaseline);
            if (!loggedStereoDepthInit && graphState.LastStereoDepthUpdates() > 0) {
                std::cerr << "[dpvo_trt] stereo fmap depth init updates="
                          << graphState.LastStereoDepthUpdates()
                          << " fx_feature=" << intrinsics.fx
                          << " baseline=" << voState.m_lkBaseline << "\n";
                loggedStereoDepthInit = true;
            }
        }
        const int keyframeRemovalsBefore = graphState.KeyframeRemovals();
        out.superpointRawLeftCount = graphState.PatchCount();
        out.superpointRawRightCount = graphState.LastStereoDepthUpdates();
        out.superpointMatchedStereoCount = graphState.EdgeCount();

        double nativeUpdateMs = 0.0;
        const bool poseUpdated = patchRun.ok && softAggSplitReady &&
                                 nativeSolver.Step(graphState, updatePreAggRuntime, updatePreAggEngine,
                                                   updatePostAggRuntime, updatePostAggEngine,
                                                   cudaKernelReady ? &cudaKernelRuntime : nullptr,
                                                   cudaStream.stream, intrinsics, &nativeUpdateMs, &dpvoErr);
        out.lkUpdateMs = nativeUpdateMs;
        out.frontendMs = out.superpointForwardMs + out.superpointStereoMatchMs + nativeUpdateMs;
        if (graphState.KeyframeRemovals() != keyframeRemovalsBefore &&
            graphState.KeyframeRemovals() != loggedKeyframeRemovals) {
            loggedKeyframeRemovals = graphState.KeyframeRemovals();
            std::cerr << "[dpvo_trt] keyframe removal count=" << loggedKeyframeRemovals
                      << " active_edges=" << graphState.EdgeCount()
                      << " active_frames=" << graphState.FrameCount() << "\n";
        }
        out.matchesInliers = graphState.EdgeCount();
        out.trackedMapPointCount = static_cast<uint32_t>(graphState.EdgeCount());
        out.localMapPointCount = static_cast<uint32_t>(graphState.PatchCount());

        if (poseUpdated || nativeSolver.HasPose() || graphState.FrameCount() > 0) {
            const Sophus::SE3f publishTcw = nativeSolver.HasPose() ? nativeSolver.LastTcw() : Sophus::SE3f{};
            lastPose = PoseFromTwc(publishTcw.inverse());
            haveLastPose = true;
        }

        if (poseUpdated) {
            out.trackingState = ORB_SLAM3::Tracking::OK;
        } else {
            if (!dpvoErr.empty() && !loggedNativeSolverWait) {
                std::cerr << "[dpvo_trt] native solver waiting: " << dpvoErr << "\n";
                loggedNativeSolverWait = true;
            }
            out.trackingState = haveLastPose ? ORB_SLAM3::Tracking::RECENTLY_LOST : ORB_SLAM3::Tracking::LOST;
        }

        out.poseValid = haveLastPose && TrackingStateCanPublishPose(out.trackingState);
        out.pose = lastPose;
        out.pose.valid = out.poseValid;
        if (extractFeatures) {
            const DpvoFrameState *newest = graphState.NewestFrame();
            if (newest != nullptr) {
                out.leftFeatures.reserve(newest->patches.size());
                for (const DpvoPatchState &patch : newest->patches) {
                    out.leftFeatures.emplace_back(patch.x * 4.0f / std::max(scaleX, 1e-6f),
                                                  patch.y * 4.0f / std::max(scaleY, 1e-6f));
                }
            }
        }
        out.orbTrackMs = ElapsedMs(start, std::chrono::steady_clock::now());
        return out;
    }

    DpvoTensorRtConfig config;
    TensorRtEngineHandle patchEngine;
    TensorRtEngineHandle updateEngine;
    TensorRtEngineHandle updatePreAggEngine;
    TensorRtEngineHandle updatePostAggEngine;
    CudaStreamHandle cudaStream;
    DpvoPatchifierRuntime patchifierRuntime;
    DpvoPatchifierRuntime patchifierRightRuntime;
    DpvoUpdateRuntime updateRuntime;
    DpvoUpdatePreAggRuntime updatePreAggRuntime;
    DpvoUpdatePostAggRuntime updatePostAggRuntime;
    DpvoCudaKernelRuntime cudaKernelRuntime;
    DpvoGraphState graphState;
    DpvoNativeSolver nativeSolver;
    SlamModeSharedState voState;
    cv::Mat resizedGray;
    cv::Mat resizedRightGray;
    core::ports::PoseEstimate lastPose{};
    bool haveLastPose{false};
    bool running{false};
    bool softAggSplitReady{false};
    bool cudaKernelReady{false};
    bool loggedPatchifierShape{false};
    bool loggedPatchifierError{false};
    bool loggedRightPatchifierError{false};
    bool loggedNativeSolverWait{false};
    bool loggedStereoDepthInit{false};
    int loggedKeyframeRemovals{0};
};

#else

struct DpvoTensorRtEngine::Impl {
    explicit Impl(DpvoTensorRtConfig cfg) : config(std::move(cfg)) {}

    bool Start()
    {
        std::cerr << "[dpvo_trt] native TensorRT backend was not compiled into this target\n";
        return false;
    }

    void Stop() {}

    core::ports::SlamOutput Process(const core::ports::SlamInputBatch &input, bool, bool)
    {
        core::ports::SlamOutput out{};
        out.frameId = input.frameId;
        out.captureTimestampNs = input.captureTimestampNs;
        out.trackingState = ORB_SLAM3::Tracking::LOST;
        return out;
    }

    DpvoTensorRtConfig config;
};

#endif

DpvoTensorRtEngine::DpvoTensorRtEngine(DpvoTensorRtConfig config) : m_impl(std::make_unique<Impl>(std::move(config))) {}
DpvoTensorRtEngine::~DpvoTensorRtEngine() = default;

bool DpvoTensorRtEngine::Start()
{
    return m_impl != nullptr && m_impl->Start();
}

void DpvoTensorRtEngine::Stop()
{
    if (m_impl != nullptr) {
        m_impl->Stop();
    }
}

core::ports::SlamOutput DpvoTensorRtEngine::Process(const core::ports::SlamInputBatch &input, bool extractFeatures,
                                                    bool extractPointCloud)
{
    return m_impl != nullptr ? m_impl->Process(input, extractFeatures, extractPointCloud) : core::ports::SlamOutput{};
}

DpvoTensorRtConfig MakeDpvoTensorRtConfig(const RuntimeConfig &runtime, const std::string &settingsPath)
{
    DpvoTensorRtConfig out{};
    out.repoPath = runtime.dpvoRepo;
    out.patchEnginePath = runtime.dpvoPatchEngine;
    out.updateEnginePath = runtime.dpvoUpdateEngine;
    out.settingsPath = settingsPath;
    out.inputWidth = std::clamp(runtime.dpvoInputWidth, 160, 1280);
    out.inputHeight = std::clamp(runtime.dpvoInputHeight, 120, 960);
    out.patchesPerFrame = std::clamp(runtime.dpvoPatchesPerFrame, 16, 256);
    out.optimizationWindow = std::clamp(runtime.dpvoOptimizationWindow, 4, 32);
    return out;
}

} // namespace smartdrone::adapters::slam
