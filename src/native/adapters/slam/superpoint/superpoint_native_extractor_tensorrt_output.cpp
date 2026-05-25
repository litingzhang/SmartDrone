#include "adapters/slam/superpoint/superpoint_native_extractor.h"

#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_output.h"

#include <chrono>
#include <cstring>
#include <string>

#include <cuda_runtime_api.h>

namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal {

namespace {

void SetTensorRtOutputCopyError(const TensorRtOutputCopyRequest &request,
                                const char *typeName)
{
    if (request.err != nullptr) {
        *request.err = std::string("TensorRT failed to copy ") +
                       request.engineName + " " + typeName + " output";
    }
}

void SetTensorRtOutputPendingHost(TensorBlob &output, const void *hostPtr,
                                  size_t elementCount,
                                  TensorBlob::HostStorage storage, bool pinned)
{
    output.pendingHostData = hostPtr;
    output.pendingElementCount = elementCount;
    output.hostStorage = storage;
    output.pinnedHostData = pinned;
}

bool TryUsePinnedTensorRtOutput(const TensorRtOutputCopyRequest &request,
                                TensorRtOutputCopyPlan &plan,
                                TensorBlob::HostStorage storage,
                                size_t elementCount)
{
    if (request.pinnedHostBuffer == nullptr ||
        !EnvFlag("SMART_DRONE_TRT_PINNED_HOST_OUTPUT", false) ||
        !request.pinnedHostBuffer->Ensure(plan.bytes)) {
        return false;
    }
    plan.hostPtr = request.pinnedHostBuffer->ptr;
    plan.pinned = true;
    SetTensorRtOutputPendingHost(request.output, plan.hostPtr, elementCount,
                                 storage, true);
    return true;
}

TensorRtOutputCopyPlan PrepareFloatTensorRtOutput(
    const TensorRtOutputCopyRequest &request, size_t elementCount)
{
    request.output.data.resize(elementCount);
    TensorRtOutputCopyPlan plan{request.output.data.data(),
                                elementCount * sizeof(float), false, "FP32"};
    TryUsePinnedTensorRtOutput(request, plan, TensorBlob::HostStorage::Float,
                               elementCount);
    return plan;
}

TensorRtOutputCopyPlan PrepareHalfTensorRtOutput(
    const TensorRtOutputCopyRequest &request, size_t elementCount)
{
    request.output.data.resize(elementCount);
    request.output.halfData.resize(elementCount);
    TensorRtOutputCopyPlan plan{request.output.halfData.data(),
                                elementCount * sizeof(uint16_t), false, "FP16"};
    if (!TryUsePinnedTensorRtOutput(request, plan, TensorBlob::HostStorage::Half,
                                   elementCount)) {
        SetTensorRtOutputPendingHost(request.output, plan.hostPtr, elementCount,
                                     TensorBlob::HostStorage::Half, false);
    }
    return plan;
}

TensorRtOutputCopyPlan PrepareIntTensorRtOutput(
    const TensorRtOutputCopyRequest &request, size_t elementCount)
{
    request.output.data.resize(elementCount);
    request.output.intData.resize(elementCount);
    TensorRtOutputCopyPlan plan{request.output.intData.data(),
                                elementCount * sizeof(int32_t), false, "INT32"};
    if (!TryUsePinnedTensorRtOutput(request, plan, TensorBlob::HostStorage::Int32,
                                   elementCount)) {
        SetTensorRtOutputPendingHost(request.output, plan.hostPtr, elementCount,
                                     TensorBlob::HostStorage::Int32, false);
    }
    return plan;
}

TensorRtOutputCopyPlan PrepareTensorRtOutputCopy(
    const TensorRtOutputCopyRequest &request, size_t elementCount)
{
    if (request.dtype == nvinfer1::DataType::kFLOAT) {
        return PrepareFloatTensorRtOutput(request, elementCount);
    }
    if (request.dtype == nvinfer1::DataType::kHALF) {
        return PrepareHalfTensorRtOutput(request, elementCount);
    }
    if (request.dtype == nvinfer1::DataType::kINT32) {
        return PrepareIntTensorRtOutput(request, elementCount);
    }
    return {};
}

void RecordTensorRtOutputCopy(const TensorRtOutputCopyRequest &request,
                              const std::chrono::steady_clock::time_point &start,
                              const std::chrono::steady_clock::time_point &end,
                              const TensorRtOutputCopyPlan &plan)
{
    if (request.stats == nullptr) {
        return;
    }
    request.stats->outputMs += DurationMs(start, end);
    request.stats->d2hBytes += plan.bytes;
    request.stats->pinnedHostOutput =
        request.stats->pinnedHostOutput || plan.pinned;
}

bool CopyTensorRtOutputToHost(const TensorRtOutputCopyRequest &request,
                              const TensorRtOutputCopyPlan &plan)
{
    const auto outputStartTp = std::chrono::steady_clock::now();
    const cudaError_t rc =
        cudaMemcpyAsync(plan.hostPtr, request.devicePtr, plan.bytes,
                        cudaMemcpyDeviceToHost, request.stream);
    if (rc != cudaSuccess) {
        SetTensorRtOutputCopyError(request, plan.typeName);
        return false;
    }
    const auto outputEndTp = std::chrono::steady_clock::now();
    RecordTensorRtOutputCopy(request, outputStartTp, outputEndTp, plan);
    return true;
}

void FinalizeFloatTensorRtOutput(TensorBlob &output)
{
    if (output.pinnedHostData &&
        EnvFlag("SMART_DRONE_TRT_PINNED_HOST_VIEW", false)) {
        output.floatData = static_cast<const float *>(output.pendingHostData);
        output.floatElementCount = output.pendingElementCount;
        return;
    }
    if (output.pinnedHostData) {
        std::memcpy(output.data.data(), output.pendingHostData,
                    output.pendingElementCount * sizeof(float));
    }
    output.floatData = output.data.data();
    output.floatElementCount = output.data.size();
}

void FinalizeHalfTensorRtOutput(TensorBlob &output)
{
    const auto *src = static_cast<const uint16_t *>(output.pendingHostData);
    for (size_t i = 0; i < output.pendingElementCount; ++i) {
        output.data[i] = HalfToFloat(src[i]);
    }
    output.floatData = output.data.data();
    output.floatElementCount = output.data.size();
}

void FinalizeIntTensorRtOutput(TensorBlob &output)
{
    const auto *src = static_cast<const int32_t *>(output.pendingHostData);
    for (size_t i = 0; i < output.pendingElementCount; ++i) {
        output.data[i] = static_cast<float>(src[i]);
    }
    output.floatData = output.data.data();
    output.floatElementCount = output.data.size();
}

void ClearTensorRtOutputPendingHost(TensorBlob &output)
{
    output.pendingHostData = nullptr;
    output.pendingElementCount = 0;
    output.hostStorage = TensorBlob::HostStorage::Float;
}

void RecordTensorRtOutputConvert(
    TensorRtForwardStats *stats,
    const std::chrono::steady_clock::time_point &start)
{
    if (stats != nullptr) {
        stats->outputConvertMs +=
            DurationMs(start, std::chrono::steady_clock::now());
    }
}

} // namespace

bool ScheduleTensorRtOutputCopy(const TensorRtOutputCopyRequest &request)
{
    request.output.ResetHostData();
    if (request.volume <= 0) {
        return false;
    }

    const size_t elementCount = static_cast<size_t>(request.volume);
    const TensorRtOutputCopyPlan plan =
        PrepareTensorRtOutputCopy(request, elementCount);
    if (plan.hostPtr != nullptr) {
        return CopyTensorRtOutputToHost(request, plan);
    }
    if (request.err != nullptr) {
        *request.err = std::string("TensorRT ") + request.engineName +
                       " output has unsupported data type";
    }
    return false;
}

bool FinalizeTensorRtOutput(TensorBlob &output, TensorRtForwardStats *stats)
{
    if (output.pendingHostData == nullptr || output.pendingElementCount == 0) {
        return true;
    }
    const auto convertStartTp = std::chrono::steady_clock::now();
    if (output.hostStorage == TensorBlob::HostStorage::Float) {
        FinalizeFloatTensorRtOutput(output);
    } else if (output.hostStorage == TensorBlob::HostStorage::Half) {
        FinalizeHalfTensorRtOutput(output);
    } else if (output.hostStorage == TensorBlob::HostStorage::Int32) {
        FinalizeIntTensorRtOutput(output);
    }
    RecordTensorRtOutputConvert(stats, convertStartTp);
    ClearTensorRtOutputPendingHost(output);
    return true;
}

} // namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal
