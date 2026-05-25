#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_common.h"

namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal {

struct TensorRtOutputCopyRequest {
    void *devicePtr;
    nvinfer1::DataType dtype;
    int64_t volume;
    cudaStream_t stream;
    TensorBlob &output;
    CudaPinnedHostBuffer *pinnedHostBuffer;
    TensorRtForwardStats *stats;
    const char *engineName;
    std::string *err;
};

struct TensorRtOutputCopyPlan {
    void *hostPtr{nullptr};
    size_t bytes{0};
    bool pinned{false};
    const char *typeName{nullptr};
};

bool ScheduleTensorRtOutputCopy(const TensorRtOutputCopyRequest &request);
bool FinalizeTensorRtOutput(TensorBlob &output, TensorRtForwardStats *stats);

} // namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal
