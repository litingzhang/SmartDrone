#pragma once

#include <array>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_common.h"
#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_utils.h"

namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal {

struct TensorRtLightGlueForwardRequest {
    const std::vector<float> &keypoints0;
    const std::vector<float> &keypoints1;
    const std::vector<float> &descriptors0;
    const std::vector<float> &descriptors1;
    const std::array<float, 2> &imageSize0;
    const std::array<float, 2> &imageSize1;
    int pointCount;
    TensorBlob &scores;
    TensorRtForwardStats *stats;
    std::string *err;
};

struct TensorRtFloatInputRequest {
    int index;
    const std::vector<int> &dims;
    const std::vector<float> &data;
    TensorRtForwardStats *stats;
    std::string *err;
};

class TensorRtLightGlueEngine {
  public:
    ~TensorRtLightGlueEngine();

    bool Load(const std::filesystem::path &enginePath, std::string *err);
    int FixedPointCount() const;
    bool Forward(const TensorRtLightGlueForwardRequest &request);

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
    static nvinfer1::Dims MakeDims(const std::vector<int> &dims);
    bool SetInput(const TensorRtFloatInputRequest &request);
    bool SetFeatureInputs(const TensorRtLightGlueForwardRequest &request);
    bool SetImageSizeInputs(const TensorRtLightGlueForwardRequest &request);
    bool PrepareOutputBindings(const TensorRtLightGlueForwardRequest &request);
    bool Enqueue(const TensorRtLightGlueForwardRequest &request);
    bool ScheduleOutput(const TensorRtLightGlueForwardRequest &request);
    bool Synchronize(const TensorRtLightGlueForwardRequest &request);
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
    int m_kpts0Index{-1};
    int m_kpts1Index{-1};
    int m_desc0Index{-1};
    int m_desc1Index{-1};
    int m_size0Index{-1};
    int m_size1Index{-1};
    int m_scoresIndex{-1};
};

} // namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal
