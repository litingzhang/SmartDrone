#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <string>
#include <unordered_map>
#include <vector>

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <nvrtc.h>

#include "adapters/slam/dpvo/dpvo_tensorrt_engine_cuda_buffers.h"

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

class DpvoCudaKernelRuntime {
  public:
    struct CorrelationBatchRequest {
        int edgeCount;
        const std::vector<float> &edgePatchGmap;
        const std::vector<float> &edgeCoords;
        const std::vector<int> &edgeTargetFrame;
        const std::vector<float> &fmapStorage;
        const std::vector<float> &fmapLevel4Storage;
        const std::vector<int> &fmapOffsets;
        const std::vector<int> &fmapHeights;
        const std::vector<int> &fmapWidths;
        const std::vector<int> &level4Offsets;
        const std::vector<int> &level4Heights;
        const std::vector<int> &level4Widths;
        cudaStream_t stream;
        std::vector<float> *outCorr;
        std::string *err;
    };
    struct SoftAggExpandRequest {
        const std::vector<float> &f;
        const std::vector<float> &g;
        const std::vector<int> &groupIds;
        int edgeCount;
        int dim;
        cudaStream_t stream;
        std::vector<float> *out;
        std::string *err;
    };
    struct SoftAggExpandDeviceRequest {
        const CudaDeviceBuffer &fDevice;
        const CudaDeviceBuffer &gDevice;
        const std::vector<int> &groupIds;
        int edgeCount;
        int dim;
        cudaStream_t stream;
        CudaDeviceBuffer &outDevice;
        std::string *err;
    };
    ~DpvoCudaKernelRuntime();

    DpvoCudaKernelRuntime() = default;
    DpvoCudaKernelRuntime(const DpvoCudaKernelRuntime &) = delete;
    DpvoCudaKernelRuntime &operator=(const DpvoCudaKernelRuntime &) = delete;

    bool Initialize(cudaStream_t stream, std::string *err);

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

    bool ComputeCorrelationBatch(const CorrelationBatchRequest &request);
    bool ComputeSoftAggExpand(const SoftAggExpandRequest &request);
    bool ComputeSoftAggExpandDevice(
        const SoftAggExpandDeviceRequest &request);

  private:
    struct CorrelationSmokeCase {
        int channels{4};
        int height{5};
        int width{5};
        int x{2};
        int y{2};
        std::vector<float> patch;
        std::vector<float> fmap;
        float expected{0.0f};
    };
    struct CorrelationSmokeDeviceBuffers {
        void *patch{nullptr};
        void *fmap{nullptr};
        void *out{nullptr};
    };
    struct CorrelationBatchShape {
        size_t edgeCount{0U};
        size_t patchValues{0U};
        size_t coordValues{0U};
        size_t corrValues{0U};
    };
    struct SoftAggGroups {
        std::vector<int> starts;
        std::vector<int> indices;
        int count{0};
    };
    struct SoftAggLaunchRequest {
        const CudaDeviceBuffer &fDevice;
        const CudaDeviceBuffer &gDevice;
        const CudaDeviceBuffer &outDevice;
        int groupCount;
        int dim;
        cudaStream_t stream;
        const char *kernelName;
        std::string *err;
    };

    template <typename T>
    bool LoadSymbol(void *handle, const char *name, T *out, std::string *err);

    template <typename T>
    bool CopyVectorToDevice(const std::vector<T> &src, CudaDeviceBuffer &dst,
                            cudaStream_t stream, const char *name,
                            std::string *err);

    bool OpenLibraryAny(std::initializer_list<const char *> names, void **handle,
                        std::string *err);
    bool OpenLibraries(std::string *err);
    bool LoadNvrtcSymbols(std::string *err);
    bool LoadDriverSymbols(std::string *err);
    const char *DriverError(CUresult result) const;
    bool CheckDriver(CUresult result, const char *what, std::string *err) const;
    bool CheckNvrtc(nvrtcResult result, const char *what, std::string *err) const;
    bool EnsureDriverContext(std::string *err);
    const char *CudaKernelSource() const;
    bool CreateNvrtcProgram(nvrtcProgram &program, std::string *err) const;
    bool CompileNvrtcProgram(nvrtcProgram program, std::string *err) const;
    bool ExtractNvrtcPtx(nvrtcProgram program, std::vector<char> &ptx,
                         std::string *err) const;
    bool LoadCudaModule(const std::vector<char> &ptx, std::string *err);
    bool BindCudaKernel(CUfunction &function, const char *name,
                        std::string *err);
    bool BindCudaKernels(std::string *err);
    bool CompileAndLoad(std::string *err);
    static CorrelationBatchShape CorrelationShape(int edgeCount);
    bool CorrelationBatchInputValid(
        const CorrelationBatchRequest &request,
        const CorrelationBatchShape &shape) const;
    bool CopyCorrelationBatchInputs(const CorrelationBatchRequest &request,
                                    const CorrelationBatchShape &shape);
    bool LaunchCorrelationBatchKernel(const CorrelationBatchRequest &request,
                                      size_t corrValues);
    bool CopyCorrelationBatchOutput(const CorrelationBatchRequest &request,
                                    size_t corrValues);
    bool SoftAggHostInputValid(const SoftAggExpandRequest &request,
                               size_t values) const;
    bool SoftAggDeviceInputValid(
        const SoftAggExpandDeviceRequest &request, size_t values) const;
    static SoftAggGroups BuildSoftAggGroups(
        const std::vector<int> &groupIds, int edgeCount);
    bool CopySoftAggGroupTables(const SoftAggGroups &groups,
                                cudaStream_t stream, std::string *err);
    bool LaunchSoftAggKernel(const SoftAggLaunchRequest &request);
    bool CopySoftAggOutput(const SoftAggExpandRequest &request, size_t values);
    bool RunCorrelationSmoke(cudaStream_t stream, std::string *err);
    CorrelationSmokeCase BuildCorrelationSmokeCase() const;
    float ComputeCorrelationSmokeExpected(
        const CorrelationSmokeCase &smoke) const;
    bool AllocateCorrelationSmokeBuffers(
        const CorrelationSmokeCase &smoke,
        CorrelationSmokeDeviceBuffers &buffers, std::string *err) const;
    bool CopyCorrelationSmokeInputs(
        const CorrelationSmokeCase &smoke,
        const CorrelationSmokeDeviceBuffers &buffers, cudaStream_t stream,
        std::string *err) const;
    bool LaunchCorrelationSmokeKernel(
        const CorrelationSmokeCase &smoke,
        const CorrelationSmokeDeviceBuffers &buffers, cudaStream_t stream,
        std::string *err);
    bool ReadCorrelationSmokeOutput(
        const CorrelationSmokeDeviceBuffers &buffers, cudaStream_t stream,
        float &got, std::string *err) const;
    bool ValidateCorrelationSmokeOutput(
        const CorrelationSmokeCase &smoke, float got, std::string *err) const;
    void ReleaseCorrelationSmokeBuffers(
        CorrelationSmokeDeviceBuffers &buffers) const;
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
