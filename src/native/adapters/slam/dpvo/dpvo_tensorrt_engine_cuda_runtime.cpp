#include "adapters/slam/dpvo/dpvo_tensorrt_engine_cuda_runtime.h"

#include <dlfcn.h>

#include "adapters/slam/engine/slam_env.h"

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

DpvoCudaKernelRuntime::~DpvoCudaKernelRuntime()
{
    Reset();
}

bool DpvoCudaKernelRuntime::Initialize(cudaStream_t stream, std::string *err)
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

template <typename T>
bool DpvoCudaKernelRuntime::LoadSymbol(void *handle, const char *name, T *out,
                                       std::string *err)
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

bool DpvoCudaKernelRuntime::OpenLibraryAny(std::initializer_list<const char *> names, void **handle,
                    std::string *err)
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

bool DpvoCudaKernelRuntime::OpenLibraries(std::string *err)
{
    return OpenLibraryAny(
               {"libnvrtc.so", "libnvrtc.so.11.4", "libnvrtc.so.11.2",
                "/usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvrtc.so",
                "/usr/local/cuda/targets/aarch64-linux/lib/libnvrtc.so"},
               &m_nvrtcLib, err) &&
           OpenLibraryAny({"libcuda.so", "libcuda.so.1",
                           "/usr/lib/aarch64-linux-gnu/tegra/libcuda.so",
                           "/usr/lib/aarch64-linux-gnu/libcuda.so"},
                          &m_cudaLib, err);
}

bool DpvoCudaKernelRuntime::LoadNvrtcSymbols(std::string *err)
{
    return LoadSymbol(m_nvrtcLib, "nvrtcCreateProgram", &m_nvrtcCreateProgram,
                      err) &&
           LoadSymbol(m_nvrtcLib, "nvrtcCompileProgram", &m_nvrtcCompileProgram,
                      err) &&
           LoadSymbol(m_nvrtcLib, "nvrtcGetPTXSize", &m_nvrtcGetPTXSize, err) &&
           LoadSymbol(m_nvrtcLib, "nvrtcGetPTX", &m_nvrtcGetPTX, err) &&
           LoadSymbol(m_nvrtcLib, "nvrtcGetProgramLogSize",
                      &m_nvrtcGetProgramLogSize, err) &&
           LoadSymbol(m_nvrtcLib, "nvrtcGetProgramLog", &m_nvrtcGetProgramLog,
                      err) &&
           LoadSymbol(m_nvrtcLib, "nvrtcDestroyProgram", &m_nvrtcDestroyProgram,
                      err) &&
           LoadSymbol(m_nvrtcLib, "nvrtcGetErrorString", &m_nvrtcGetErrorString,
                      err);
}

bool DpvoCudaKernelRuntime::LoadDriverSymbols(std::string *err)
{
    return LoadSymbol(m_cudaLib, "cuInit", &m_cuInit, err) &&
           LoadSymbol(m_cudaLib, "cuDeviceGet", &m_cuDeviceGet, err) &&
           LoadSymbol(m_cudaLib, "cuCtxCreate_v2", &m_cuCtxCreate, err) &&
           LoadSymbol(m_cudaLib, "cuCtxGetCurrent", &m_cuCtxGetCurrent, err) &&
           LoadSymbol(m_cudaLib, "cuModuleLoadData", &m_cuModuleLoadData,
                      err) &&
           LoadSymbol(m_cudaLib, "cuModuleGetFunction", &m_cuModuleGetFunction,
                      err) &&
           LoadSymbol(m_cudaLib, "cuLaunchKernel", &m_cuLaunchKernel, err) &&
           LoadSymbol(m_cudaLib, "cuModuleUnload", &m_cuModuleUnload, err) &&
           LoadSymbol(m_cudaLib, "cuGetErrorString", &m_cuGetErrorString, err);
}

const char *DpvoCudaKernelRuntime::DriverError(CUresult result) const
{
    const char *text = nullptr;
    if (m_cuGetErrorString != nullptr &&
        m_cuGetErrorString(result, &text) == CUDA_SUCCESS && text != nullptr) {
        return text;
    }
    return "unknown CUDA driver error";
}

bool DpvoCudaKernelRuntime::CheckDriver(CUresult result, const char *what, std::string *err) const
{
    if (result == CUDA_SUCCESS) {
        return true;
    }
    if (err != nullptr) {
        *err = std::string(what != nullptr ? what : "CUDA driver call") +
               " failed: " + DriverError(result);
    }
    return false;
}

bool DpvoCudaKernelRuntime::CheckNvrtc(nvrtcResult result, const char *what,
                std::string *err) const
{
    if (result == NVRTC_SUCCESS) {
        return true;
    }
    if (err != nullptr) {
        const char *text = m_nvrtcGetErrorString != nullptr
                               ? m_nvrtcGetErrorString(result)
                               : "unknown NVRTC error";
        *err = std::string(what != nullptr ? what : "NVRTC call") +
               " failed: " + (text != nullptr ? text : "");
    }
    return false;
}

bool DpvoCudaKernelRuntime::EnsureDriverContext(std::string *err)
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
            return true;
        }
    if (err != nullptr) {
        *err = "no current CUDA context after cudaFree(0)";
    }
    return false;
}

const char *DpvoCudaKernelRuntime::CudaKernelSource() const
{
    return R"CUDA(
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

extern "C" __global__ void dpvo_softagg_expand(
const float *f,
const float *g,
const int *group_indices,
const int *group_starts,
float *out,
int group_count,
int dim)
{
const int linear = blockIdx.x * blockDim.x + threadIdx.x;
const int total = group_count * dim;
if (linear >= total) {
    return;
}
const int c = linear % dim;
const int group = linear / dim;
const int start = group_starts[group];
const int end = group_starts[group + 1];
if (start >= end) {
    return;
}
float max_logit = -3.4028234663852886e38f;
for (int i = start; i < end; ++i) {
    const int edge = group_indices[i];
    const float logit = g[edge * dim + c];
    max_logit = fmaxf(max_logit, logit);
}
float denom = 0.0f;
float accum = 0.0f;
for (int i = start; i < end; ++i) {
    const int edge = group_indices[i];
    const float w = expf(g[edge * dim + c] - max_logit);
    denom += w;
    accum += f[edge * dim + c] * w;
}
const float value = denom > 0.0f ? accum / denom : 0.0f;
for (int i = start; i < end; ++i) {
    const int edge = group_indices[i];
    out[edge * dim + c] = value;
}
}
)CUDA";
}

bool DpvoCudaKernelRuntime::CreateNvrtcProgram(nvrtcProgram &program,
                                               std::string *err) const
{
    return CheckNvrtc(m_nvrtcCreateProgram(&program, CudaKernelSource(),
                                           "dpvo_native_kernels.cu", 0,
                                           nullptr, nullptr),
                      "nvrtcCreateProgram", err);
}

bool DpvoCudaKernelRuntime::CompileNvrtcProgram(nvrtcProgram program,
                                                std::string *err) const
{
    const char *options[] = {"--gpu-architecture=compute_72", "--std=c++14"};
    const nvrtcResult compileResult =
        m_nvrtcCompileProgram(program, 2, options);
    if (compileResult == NVRTC_SUCCESS) {
        return true;
    }
    size_t logSize = 0;
    (void)m_nvrtcGetProgramLogSize(program, &logSize);
    std::string log;
    if (logSize > 1U) {
        log.resize(logSize);
        (void)m_nvrtcGetProgramLog(program, log.data());
    }
    if (err != nullptr) {
        const char *text = m_nvrtcGetErrorString != nullptr
                               ? m_nvrtcGetErrorString(compileResult)
                               : "unknown NVRTC error";
        *err = std::string("nvrtcCompileProgram failed: ") +
               (text != nullptr ? text : "") +
               (log.empty() ? std::string{} : "\n" + log);
    }
    return false;
}

bool DpvoCudaKernelRuntime::ExtractNvrtcPtx(nvrtcProgram program,
                                            std::vector<char> &ptx,
                                            std::string *err) const
{
    size_t ptxSize = 0;
    if (!CheckNvrtc(m_nvrtcGetPTXSize(program, &ptxSize), "nvrtcGetPTXSize",
                    err)) {
        return false;
    }
    ptx.resize(ptxSize);
    return CheckNvrtc(m_nvrtcGetPTX(program, ptx.data()), "nvrtcGetPTX", err);
}

bool DpvoCudaKernelRuntime::LoadCudaModule(const std::vector<char> &ptx,
                                           std::string *err)
{
    return CheckDriver(m_cuModuleLoadData(&m_module, ptx.data()),
                       "cuModuleLoadData", err);
}

bool DpvoCudaKernelRuntime::BindCudaKernel(CUfunction &function,
                                           const char *name, std::string *err)
{
    return CheckDriver(m_cuModuleGetFunction(&function, m_module, name),
                       "cuModuleGetFunction", err);
}

bool DpvoCudaKernelRuntime::BindCudaKernels(std::string *err)
{
    return BindCudaKernel(m_corrSmokeKernel, "dpvo_corr_patch3_smoke", err) &&
           BindCudaKernel(m_corrBatchKernel, "dpvo_corr_batch", err) &&
           BindCudaKernel(m_softAggKernel, "dpvo_softagg_expand", err);
}

bool DpvoCudaKernelRuntime::CompileAndLoad(std::string *err)
{
    nvrtcProgram program = nullptr;
    if (!CreateNvrtcProgram(program, err)) {
        return false;
    }
    std::vector<char> ptx;
    const bool compiled = CompileNvrtcProgram(program, err) &&
                          ExtractNvrtcPtx(program, ptx, err);
    (void)m_nvrtcDestroyProgram(&program);
    return compiled && LoadCudaModule(ptx, err) && BindCudaKernels(err);
}

DpvoCudaKernelRuntime::CorrelationSmokeCase
DpvoCudaKernelRuntime::BuildCorrelationSmokeCase() const
{
    CorrelationSmokeCase smoke;
    smoke.patch.assign(static_cast<size_t>(smoke.channels) * 9U, 0.0f);
    smoke.fmap.assign(static_cast<size_t>(smoke.channels) *
                          static_cast<size_t>(smoke.height) *
                          static_cast<size_t>(smoke.width),
                      0.0f);
    for (size_t i = 0; i < smoke.patch.size(); ++i) {
        smoke.patch[i] = 0.01f * static_cast<float>(i + 1U);
    }
    for (size_t i = 0; i < smoke.fmap.size(); ++i) {
        smoke.fmap[i] = 0.02f * static_cast<float>((i % 17U) + 1U);
    }
    smoke.expected = ComputeCorrelationSmokeExpected(smoke);
    return smoke;
}

float DpvoCudaKernelRuntime::ComputeCorrelationSmokeExpected(
    const CorrelationSmokeCase &smoke) const
{
    float expected = 0.0f;
    for (int c = 0; c < smoke.channels; ++c) {
        for (int py = 0; py < 3; ++py) {
            for (int px = 0; px < 3; ++px) {
                const int p = py * 3 + px;
                const int yy = smoke.y + py - 1;
                const int xx = smoke.x + px - 1;
                const size_t patchIndex =
                    static_cast<size_t>(c) * 9U + static_cast<size_t>(p);
                const size_t fmapIndex =
                    (static_cast<size_t>(c) *
                         static_cast<size_t>(smoke.height) +
                     static_cast<size_t>(yy)) *
                        static_cast<size_t>(smoke.width) +
                    static_cast<size_t>(xx);
                expected += smoke.patch[patchIndex] * smoke.fmap[fmapIndex];
            }
        }
    }
    return expected;
}

bool DpvoCudaKernelRuntime::AllocateCorrelationSmokeBuffers(
    const CorrelationSmokeCase &smoke, CorrelationSmokeDeviceBuffers &buffers,
    std::string *err) const
{
    if (cudaMalloc(&buffers.patch, smoke.patch.size() * sizeof(float)) !=
            cudaSuccess ||
        cudaMalloc(&buffers.fmap, smoke.fmap.size() * sizeof(float)) !=
            cudaSuccess ||
        cudaMalloc(&buffers.out, sizeof(float)) != cudaSuccess) {
        if (err != nullptr) {
            *err = "cudaMalloc failed during DPVO CUDA smoke test";
        }
        return false;
    }
    return true;
}

bool DpvoCudaKernelRuntime::CopyCorrelationSmokeInputs(
    const CorrelationSmokeCase &smoke,
    const CorrelationSmokeDeviceBuffers &buffers, cudaStream_t stream,
    std::string *err) const
{
    cudaError_t rc =
        cudaMemcpyAsync(buffers.patch, smoke.patch.data(),
                        smoke.patch.size() * sizeof(float),
                        cudaMemcpyHostToDevice, stream);
    if (rc == cudaSuccess) {
        rc = cudaMemcpyAsync(buffers.fmap, smoke.fmap.data(),
                             smoke.fmap.size() * sizeof(float),
                             cudaMemcpyHostToDevice, stream);
    }
    if (rc == cudaSuccess) {
        return true;
    }
    if (err != nullptr) {
        *err = std::string(
                   "cudaMemcpyAsync failed during DPVO CUDA smoke test: ") +
               cudaGetErrorString(rc);
    }
    return false;
}

bool DpvoCudaKernelRuntime::LaunchCorrelationSmokeKernel(
    const CorrelationSmokeCase &smoke,
    const CorrelationSmokeDeviceBuffers &buffers, cudaStream_t stream,
    std::string *err)
{
    CUdeviceptr patchArg = reinterpret_cast<CUdeviceptr>(buffers.patch);
    CUdeviceptr fmapArg = reinterpret_cast<CUdeviceptr>(buffers.fmap);
    CUdeviceptr outArg = reinterpret_cast<CUdeviceptr>(buffers.out);
    int channelsArg = smoke.channels;
    int heightArg = smoke.height;
    int widthArg = smoke.width;
    int xArg = smoke.x;
    int yArg = smoke.y;
    void *args[] = {&patchArg, &fmapArg, &outArg, &channelsArg,
                    &heightArg, &widthArg, &xArg, &yArg};
    return CheckDriver(
        m_cuLaunchKernel(m_corrSmokeKernel, 1, 1, 1, 32, 1, 1,
                         32 * sizeof(float), reinterpret_cast<CUstream>(stream),
                         args, nullptr),
        "cuLaunchKernel", err);
}

bool DpvoCudaKernelRuntime::ReadCorrelationSmokeOutput(
    const CorrelationSmokeDeviceBuffers &buffers, cudaStream_t stream,
    float &got, std::string *err) const
{
    cudaError_t rc = cudaMemcpyAsync(&got, buffers.out, sizeof(float),
                                     cudaMemcpyDeviceToHost, stream);
    if (rc == cudaSuccess) {
        rc = cudaStreamSynchronize(stream);
    }
    if (rc == cudaSuccess) {
        return true;
    }
    if (err != nullptr) {
        *err = std::string("CUDA smoke synchronization failed: ") +
               cudaGetErrorString(rc);
    }
    return false;
}

bool DpvoCudaKernelRuntime::ValidateCorrelationSmokeOutput(
    const CorrelationSmokeCase &smoke, float got, std::string *err) const
{
    if (std::fabs(got - smoke.expected) > 1e-4f) {
        if (err != nullptr) {
            *err = "CUDA smoke mismatch expected=" +
                   std::to_string(smoke.expected) +
                   " got=" + std::to_string(got);
        }
        return false;
    }
    return true;
}

void DpvoCudaKernelRuntime::ReleaseCorrelationSmokeBuffers(
    CorrelationSmokeDeviceBuffers &buffers) const
{
    if (buffers.patch != nullptr) {
        cudaFree(buffers.patch);
        buffers.patch = nullptr;
    }
    if (buffers.fmap != nullptr) {
        cudaFree(buffers.fmap);
        buffers.fmap = nullptr;
    }
    if (buffers.out != nullptr) {
        cudaFree(buffers.out);
        buffers.out = nullptr;
    }
}

bool DpvoCudaKernelRuntime::RunCorrelationSmoke(cudaStream_t stream,
                                                std::string *err)
{
    const CorrelationSmokeCase smoke = BuildCorrelationSmokeCase();
    CorrelationSmokeDeviceBuffers buffers;
    if (!AllocateCorrelationSmokeBuffers(smoke, buffers, err) ||
        !CopyCorrelationSmokeInputs(smoke, buffers, stream, err) ||
        !LaunchCorrelationSmokeKernel(smoke, buffers, stream, err)) {
        ReleaseCorrelationSmokeBuffers(buffers);
        return false;
    }
    float got = 0.0f;
    const bool outputOk =
        ReadCorrelationSmokeOutput(buffers, stream, got, err) &&
        ValidateCorrelationSmokeOutput(smoke, got, err);
    ReleaseCorrelationSmokeBuffers(buffers);
    if (!outputOk) {
        return false;
    }
    m_smokeExpected = smoke.expected;
    m_smokeGot = got;
    return true;
}

void DpvoCudaKernelRuntime::Reset()
{
    if (m_module != nullptr && m_cuModuleUnload != nullptr) {
        (void)m_cuModuleUnload(m_module);
    }
    m_module = nullptr;
    m_corrSmokeKernel = nullptr;
    m_corrBatchKernel = nullptr;
    m_softAggKernel = nullptr;
    m_ready = false;
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

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
