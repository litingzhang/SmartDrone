#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_common.h"
#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_utils.h"

namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal {

struct TensorRtSuperPointForwardRequest {
    const std::vector<float> &batch;
    int batchSize;
    int height;
    int width;
    TensorBlob &detector;
    TensorBlob &descriptors;
    TensorRtForwardStats *stats;
    std::string *err;
};

class TensorRtSuperPointEngine {
  public:
    ~TensorRtSuperPointEngine();

    bool Load(const std::filesystem::path &enginePath, std::string *err);
    bool Forward(const TensorRtSuperPointForwardRequest &request);
    bool PreferredInputSize(int &height, int &width) const;
    bool SupportsBatchSize(int batchSize) const;

  private:
    int FindBinding(std::initializer_list<const char *> names, bool input) const;
    bool ReadEngineFile(const std::filesystem::path &enginePath,
                        std::vector<char> &bytes, std::string *err) const;
    bool CreateRuntimeAndEngine(const std::filesystem::path &enginePath,
                                const std::vector<char> &bytes,
                                std::string *err);
    bool CreateContextAndStream(std::string *err);
    bool CreateTimingEvents(std::string *err);
    bool ResolveBindings(std::string *err);
    nvinfer1::Dims MakeInputDims(
        const TensorRtSuperPointForwardRequest &request) const;
    bool PrepareInput(const TensorRtSuperPointForwardRequest &request);
    bool CopyInputToDevice(const TensorRtSuperPointForwardRequest &request);
    bool PrepareOutputBuffers(const TensorRtSuperPointForwardRequest &request);
    bool Enqueue(const TensorRtSuperPointForwardRequest &request);
    bool ScheduleOutputs(const TensorRtSuperPointForwardRequest &request);
    bool Synchronize(const TensorRtSuperPointForwardRequest &request);
    void RecordGpuTiming(TensorRtForwardStats *stats) const;
    bool EnsureBindingBuffer(int index, const nvinfer1::Dims &dims,
                             nvinfer1::DataType type, std::string *err);
    bool BindTensorAddresses(std::string *err);
    bool ReadOutput(int index, TensorBlob &output, TensorRtForwardStats *stats,
                    std::string *err);
    void Release();

    TensorRtLogger m_logger;
    std::unique_ptr<nvinfer1::IRuntime, DeleteTensorRtObject<nvinfer1::IRuntime>>
        m_runtime;
    std::unique_ptr<nvinfer1::ICudaEngine,
                    DeleteTensorRtObject<nvinfer1::ICudaEngine>>
        m_engine;
    std::unique_ptr<nvinfer1::IExecutionContext,
                    DeleteTensorRtObject<nvinfer1::IExecutionContext>>
        m_context;
    cudaStream_t m_stream{nullptr};
    cudaEvent_t m_computeStartEvent{nullptr};
    cudaEvent_t m_computeEndEvent{nullptr};
    cudaEvent_t m_outputStartEvent{nullptr};
    cudaEvent_t m_outputEndEvent{nullptr};
    bool m_eventTimingEnabled{false};
    std::vector<void *> m_bindings;
    std::vector<size_t> m_bindingBytes;
    std::vector<CudaPinnedHostBuffer> m_pinnedHostOutputs;
    int m_inputIndex{-1};
    int m_detectorIndex{-1};
    int m_descriptorIndex{-1};
};

} // namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal
