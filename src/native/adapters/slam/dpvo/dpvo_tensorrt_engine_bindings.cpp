#include "adapters/slam/dpvo/dpvo_tensorrt_engine.h"

#include <NvInfer.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "adapters/slam/dpvo/dpvo_tensorrt_engine_cuda_buffers.h"

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

#include "dpvo_tensorrt_engine_trt_handles.h"
#include "dpvo_tensorrt_engine_bindings.h"
#include "dpvo_tensorrt_engine_update_runtime.h"

namespace {

void SetTensorRtCopyError(std::string *err, const std::string &message)
{
    if (err != nullptr) {
        *err = message;
    }
}

bool ValidateHostToDeviceCopyRequest(
    const TensorRtHostToDeviceCopyRequest &request)
{
    if (request.src != nullptr && request.stream != nullptr &&
        request.handle.Engine() != nullptr) {
        return true;
    }
    SetTensorRtCopyError(request.err, "invalid host/device copy input");
    return false;
}

bool PrepareHostToDeviceBindingCopy(
    const TensorRtHostToDeviceCopyRequest &request, size_t *bytes)
{
    nvinfer1::Dims dims{};
    const size_t bindingCount =
        BindingElementCount(request.handle, request.bindingIndex, &dims,
                            request.err);
    if (bindingCount == request.valueCount && bindingCount > 0) {
        return BindingBytes(request.handle, request.bindingIndex, bytes,
                            nullptr, request.err) &&
               request.device.Ensure(*bytes, request.err);
    }
    SetTensorRtCopyError(
        request.err, "TensorRT input element count mismatch index=" +
                         std::to_string(request.bindingIndex) +
                         " expected=" + std::to_string(bindingCount) +
                         " got=" + std::to_string(request.valueCount) +
                         " dims=" + DimsToString(dims));
    return false;
}

bool CopyFloatHostToDeviceBuffer(
    const TensorRtHostToDeviceCopyRequest &request, size_t bytes)
{
    const cudaError_t rc =
        cudaMemcpyAsync(request.device.Data(), request.src, bytes,
                        cudaMemcpyHostToDevice, request.stream);
    if (rc == cudaSuccess) {
        return true;
    }
    SetTensorRtCopyError(request.err,
                         std::string("TensorRT float H2D copy failed: ") +
                             cudaGetErrorString(rc));
    return false;
}

bool CopyHalfHostToDeviceBuffer(
    const TensorRtHostToDeviceCopyRequest &request, size_t bytes)
{
    request.halfScratch.resize(request.valueCount);
    for (size_t i = 0; i < request.valueCount; ++i) {
        request.halfScratch[i] = __float2half(request.src[i]);
    }
    const cudaError_t rc =
        cudaMemcpyAsync(request.device.Data(), request.halfScratch.data(),
                        bytes, cudaMemcpyHostToDevice, request.stream);
    if (rc == cudaSuccess) {
        return true;
    }
    SetTensorRtCopyError(request.err,
                         std::string("TensorRT half H2D copy failed: ") +
                             cudaGetErrorString(rc));
    return false;
}

bool ValidateDeviceToHostCopyRequest(
    const TensorRtDeviceToHostCopyRequest &request)
{
    if (request.stream != nullptr && request.handle.Engine() != nullptr) {
        return true;
    }
    SetTensorRtCopyError(request.err, "invalid device/host copy input");
    return false;
}

bool PrepareDeviceToHostBindingCopy(
    const TensorRtDeviceToHostCopyRequest &request, size_t *valueCount,
    size_t *bytes, nvinfer1::Dims *dims)
{
    *valueCount = BindingElementCount(request.handle, request.bindingIndex,
                                      dims, request.err);
    if (*valueCount == 0) {
        return false;
    }
    return BindingBytes(request.handle, request.bindingIndex, bytes, nullptr,
                        request.err) &&
           request.device.Ensure(*bytes, request.err);
}

bool SynchronizeTensorRtCopy(cudaStream_t stream, const char *message,
                             std::string *err)
{
    const cudaError_t rc = cudaStreamSynchronize(stream);
    if (rc == cudaSuccess) {
        return true;
    }
    SetTensorRtCopyError(err, std::string(message) + cudaGetErrorString(rc));
    return false;
}

bool CopyFloatBindingDeviceToHost(
    const TensorRtDeviceToHostCopyRequest &request, size_t bytes)
{
    const cudaError_t rc =
        cudaMemcpyAsync(request.dst.data(), request.device.Data(), bytes,
                        cudaMemcpyDeviceToHost, request.stream);
    if (rc != cudaSuccess) {
        SetTensorRtCopyError(request.err,
                             std::string("TensorRT float D2H copy failed: ") +
                                 cudaGetErrorString(rc));
        return false;
    }
    return SynchronizeTensorRtCopy(
        request.stream, "TensorRT float D2H synchronize failed: ",
        request.err);
}

bool CopyHalfBindingDeviceToHost(
    const TensorRtDeviceToHostCopyRequest &request, size_t valueCount,
    size_t bytes)
{
    request.halfScratch.resize(valueCount);
    const cudaError_t rc =
        cudaMemcpyAsync(request.halfScratch.data(), request.device.Data(),
                        bytes, cudaMemcpyDeviceToHost, request.stream);
    if (rc != cudaSuccess) {
        SetTensorRtCopyError(request.err,
                             std::string("TensorRT half D2H copy failed: ") +
                                 cudaGetErrorString(rc));
        return false;
    }
    if (!SynchronizeTensorRtCopy(
            request.stream, "TensorRT half D2H synchronize failed: ",
            request.err)) {
        return false;
    }
    for (size_t i = 0; i < valueCount; ++i) {
        request.dst[i] = __half2float(request.halfScratch[i]);
    }
    return true;
}

void StoreOutputDims(const TensorRtDeviceToHostCopyRequest &request,
                     const nvinfer1::Dims &dims)
{
    if (request.dimsOut != nullptr) {
        *request.dimsOut = dims;
    }
}

std::string TensorRtEngineLabel(const char *name)
{
    return name != nullptr ? std::string(name) : std::string("engine");
}

double TensorRtElapsedMs(const std::chrono::steady_clock::time_point &start,
                         const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

bool ValidateWarmupRequest(const DpvoWarmupBindingsRequest &request)
{
    if (request.engine.Engine() != nullptr &&
        request.engine.Context() != nullptr && request.stream != nullptr) {
        return true;
    }
    SetTensorRtCopyError(
        request.err, TensorRtEngineLabel(request.name) +
                         " TensorRT context or CUDA stream is not initialized");
    return false;
}

bool WarmupBufferTableValid(const DpvoWarmupBindingsRequest &request)
{
    if (BindingCount(request.engine) <= static_cast<int>(request.buffers.size())) {
        return true;
    }
    SetTensorRtCopyError(request.err, TensorRtEngineLabel(request.name) +
                                          " has more bindings than expected");
    return false;
}

bool WarmupBindingIndexValid(const DpvoWarmupBindingsRequest &request,
                             int index)
{
    if (index >= 0 && index < static_cast<int>(request.buffers.size())) {
        return true;
    }
    SetTensorRtCopyError(request.err, TensorRtEngineLabel(request.name) +
                                          " binding index exceeds local buffer table");
    return false;
}

bool ZeroWarmupInputBinding(const DpvoWarmupBindingsRequest &request,
                            int index, size_t bytes)
{
    if (!BindingIsInput(request.engine, index)) {
        return true;
    }
    const cudaError_t rc = cudaMemsetAsync(
        request.buffers[static_cast<size_t>(index)].Data(), 0, bytes,
        request.stream);
    if (rc == cudaSuccess) {
        return true;
    }
    SetTensorRtCopyError(request.err, TensorRtEngineLabel(request.name) +
                                          " input memset failed: " +
                                          cudaGetErrorString(rc));
    return false;
}

bool PrepareWarmupBinding(const DpvoWarmupBindingsRequest &request, int index)
{
    size_t bytes = 0;
    if (!BindingBytes(request.engine, index, &bytes, nullptr, request.err) ||
        !WarmupBindingIndexValid(request, index)) {
        return false;
    }
    CudaDeviceBuffer &buffer = request.buffers[static_cast<size_t>(index)];
    return buffer.Ensure(bytes, request.err) &&
           ZeroWarmupInputBinding(request, index, bytes);
}

bool PrepareWarmupBindings(const DpvoWarmupBindingsRequest &request)
{
    for (int index : request.indices) {
        if (!PrepareWarmupBinding(request, index)) {
            return false;
        }
    }
    return true;
}

bool RunWarmupInference(const DpvoWarmupBindingsRequest &request)
{
    if (!SetBindingBufferAddresses(request.engine, request.buffers,
                                   request.err) ||
        !EnqueueTensorRt(request.engine, request.stream, request.name,
                         request.err)) {
        return false;
    }
    const cudaError_t rc = cudaStreamSynchronize(request.stream);
    if (rc == cudaSuccess) {
        return true;
    }
    SetTensorRtCopyError(request.err, TensorRtEngineLabel(request.name) +
                                          " synchronize failed: " +
                                          cudaGetErrorString(rc));
    return false;
}

} // namespace

size_t TensorRtDataTypeSize(nvinfer1::DataType type)
{
    switch (type) {
    case nvinfer1::DataType::kFLOAT:
    case nvinfer1::DataType::kINT32:
        return 4;
    case nvinfer1::DataType::kHALF:
        return 2;
    case nvinfer1::DataType::kINT8:
    case nvinfer1::DataType::kBOOL:
        return 1;
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

int BindingCount(const TensorRtEngineHandle &handle)
{
    return handle.Engine() != nullptr ? handle.Engine()->getNbIOTensors() : 0;
}

const char *BindingName(const TensorRtEngineHandle &handle, int bindingIndex)
{
    nvinfer1::ICudaEngine *engine = handle.Engine();
    if (engine == nullptr || bindingIndex < 0 ||
        bindingIndex >= engine->getNbIOTensors()) {
        return nullptr;
    }
    return engine->getIOTensorName(bindingIndex);
}

bool BindingIsInput(const TensorRtEngineHandle &handle, int bindingIndex)
{
    nvinfer1::ICudaEngine *engine = handle.Engine();
    const char *name = BindingName(handle, bindingIndex);
    return engine != nullptr && name != nullptr &&
           engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT;
}

nvinfer1::DataType BindingDataType(const TensorRtEngineHandle &handle,
                                   int bindingIndex)
{
    nvinfer1::ICudaEngine *engine = handle.Engine();
    const char *name = BindingName(handle, bindingIndex);
    if (engine == nullptr || name == nullptr) {
        return nvinfer1::DataType::kFLOAT;
    }
    return engine->getTensorDataType(name);
}

bool SetBindingShape(TensorRtEngineHandle &handle, int bindingIndex,
                     const nvinfer1::Dims &dims, std::string *err)
{
    if (bindingIndex < 0) {
        SetTensorRtCopyError(err, "invalid TensorRT tensor index");
        return false;
    }
    nvinfer1::IExecutionContext *context = handle.Context();
    const char *name = BindingName(handle, bindingIndex);
    if (context == nullptr || name == nullptr) {
        SetTensorRtCopyError(err, "invalid TensorRT context/tensor");
        return false;
    }
    if (!context->setInputShape(name, dims)) {
        SetTensorRtCopyError(err, "setInputShape failed for tensor " +
                                      std::string(name) +
                                      " dims=" + DimsToString(dims));
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
    for (int i = 0; i < engine->getNbIOTensors(); ++i) {
        const char *tensorName = engine->getIOTensorName(i);
        if (tensorName != nullptr && std::string(tensorName) == name) {
            return i;
        }
    }
    return -1;
}

bool BindingBytes(const TensorRtEngineHandle &handle, int bindingIndex,
                  size_t *bytes, nvinfer1::Dims *dimsOut, std::string *err)
{
    if (bytes == nullptr) {
        return false;
    }
    nvinfer1::ICudaEngine *engine = handle.Engine();
    nvinfer1::IExecutionContext *context = handle.Context();
    const char *name = BindingName(handle, bindingIndex);
    if (engine == nullptr || context == nullptr || name == nullptr) {
        SetTensorRtCopyError(err, "invalid TensorRT engine/context/tensor");
        return false;
    }
    nvinfer1::Dims dims = context->getTensorShape(name);
    const int64_t volume = DimsVolume(dims);
    const size_t elementSize = TensorRtDataTypeSize(
        engine->getTensorDataType(name));
    if (volume <= 0 || elementSize == 0) {
        SetTensorRtCopyError(err, "invalid TensorRT tensor shape/type index=" +
                                      std::to_string(bindingIndex) +
                                      " dims=" + DimsToString(dims));
        return false;
    }
    *bytes = static_cast<size_t>(volume) * elementSize;
    if (dimsOut != nullptr) {
        *dimsOut = dims;
    }
    return true;
}

size_t BindingElementCount(const TensorRtEngineHandle &handle, int bindingIndex,
                           nvinfer1::Dims *dimsOut, std::string *err)
{
    size_t bytes = 0;
    nvinfer1::Dims dims{};
    if (!BindingBytes(handle, bindingIndex, &bytes, &dims, err)) {
        return 0;
    }
    const size_t elementSize = TensorRtDataTypeSize(
        BindingDataType(handle, bindingIndex));
    if (elementSize == 0) {
        SetTensorRtCopyError(err, "invalid TensorRT tensor element size index=" +
                                      std::to_string(bindingIndex));
        return 0;
    }
    if (dimsOut != nullptr) {
        *dimsOut = dims;
    }
    return bytes / elementSize;
}

bool EnsureBindingBuffer(const TensorRtEngineHandle &handle, int bindingIndex,
                         CudaDeviceBuffer &device, std::string *err)
{
    size_t bytes = 0;
    return BindingBytes(handle, bindingIndex, &bytes, nullptr, err) &&
           device.Ensure(bytes, err);
}

bool BindingIsFloat(const TensorRtEngineHandle &handle, int bindingIndex)
{
    return handle.Engine() != nullptr && bindingIndex >= 0 &&
           BindingDataType(handle, bindingIndex) == nvinfer1::DataType::kFLOAT;
}

bool SetBindingAddress(TensorRtEngineHandle &handle, int bindingIndex,
                       void *data, std::string *err)
{
    nvinfer1::IExecutionContext *context = handle.Context();
    const char *name = BindingName(handle, bindingIndex);
    if (context == nullptr || name == nullptr || data == nullptr) {
        SetTensorRtCopyError(err, "invalid TensorRT tensor address index=" +
                                      std::to_string(bindingIndex));
        return false;
    }
    if (!context->setTensorAddress(name, data)) {
        SetTensorRtCopyError(err,
                             "setTensorAddress failed for tensor " +
                                 std::string(name));
        return false;
    }
    return true;
}

bool SetBindingBufferAddresses(TensorRtEngineHandle &handle,
                               std::array<CudaDeviceBuffer, 16> &buffers,
                               std::string *err)
{
    const int nbBindings = BindingCount(handle);
    if (nbBindings <= 0 || nbBindings > static_cast<int>(buffers.size())) {
        SetTensorRtCopyError(err,
                             "TensorRT tensor count exceeds local buffer table");
        return false;
    }
    for (int i = 0; i < nbBindings; ++i) {
        if (!SetBindingAddress(handle, i,
                               buffers[static_cast<size_t>(i)].Data(), err)) {
            return false;
        }
    }
    return true;
}

bool EnqueueTensorRt(TensorRtEngineHandle &handle, cudaStream_t stream,
                     const char *name, std::string *err)
{
    const std::string engineName =
        name != nullptr ? std::string(name) : std::string("engine");
    if (handle.Context() == nullptr || stream == nullptr) {
        SetTensorRtCopyError(
            err, engineName + " TensorRT context or CUDA stream is not initialized");
        return false;
    }
    if (!handle.Context()->enqueueV3(stream)) {
        SetTensorRtCopyError(err, engineName + " enqueueV3 failed");
        return false;
    }
    return true;
}

bool CopyFloatDeviceBufferToHost(const CudaDeviceBuffer &device,
                                 size_t valueCount, cudaStream_t stream,
                                 std::vector<float> &dst, std::string *err)
{
    if (device.Bytes() < valueCount * sizeof(float) || stream == nullptr) {
        SetTensorRtCopyError(err, "invalid float D2H buffer copy input");
        return false;
    }
    dst.assign(valueCount, 0.0F);
    cudaError_t rc =
        cudaMemcpyAsync(dst.data(), device.Data(), valueCount * sizeof(float),
                        cudaMemcpyDeviceToHost, stream);
    if (rc == cudaSuccess) {
        rc = cudaStreamSynchronize(stream);
    }
    if (rc != cudaSuccess) {
        SetTensorRtCopyError(err, std::string("float D2H buffer copy failed: ") +
                                      cudaGetErrorString(rc));
        return false;
    }
    return true;
}

bool DpvoUpdateRuntime::Initialize(const TensorRtEngineHandle &engine,
                                   std::string *err)
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
    const bool ok = m_netIndex >= 0 && m_inpIndex >= 0 && m_corrIndex >= 0 &&
                    m_prevNetIndex >= 0 && m_nextNetIndex >= 0 &&
                    m_prevMaskIndex >= 0 && m_nextMaskIndex >= 0 &&
                    m_updatedNetIndex >= 0 && m_deltaIndex >= 0 &&
                    m_weightIndex >= 0;
    if (!ok) {
        SetTensorRtCopyError(err, "update binding lookup failed");
    }
    return ok;
}

DpvoUpdateRun DpvoUpdateRuntime::Warmup(TensorRtEngineHandle &engine,
                                        cudaStream_t stream, int edges,
                                        std::string *err)
{
    edges = std::clamp(edges, 1, 4096);
    if (engine.Context() == nullptr || stream == nullptr) {
        SetTensorRtCopyError(
            err, "update TensorRT context or CUDA stream is not initialized");
        return {};
    }
    if (!SetBindingShape(engine, m_netIndex, nvinfer1::Dims3{1, edges, DIM},
                         err) ||
        !SetBindingShape(engine, m_inpIndex, nvinfer1::Dims3{1, edges, DIM},
                         err) ||
        !SetBindingShape(engine, m_corrIndex,
                         nvinfer1::Dims3{1, edges, CORR_DIM}, err) ||
        !SetBindingShape(engine, m_prevNetIndex,
                         nvinfer1::Dims3{1, edges, DIM}, err) ||
        !SetBindingShape(engine, m_nextNetIndex,
                         nvinfer1::Dims3{1, edges, DIM}, err) ||
        !SetBindingShape(engine, m_prevMaskIndex,
                         nvinfer1::Dims3{1, edges, 1}, err) ||
        !SetBindingShape(engine, m_nextMaskIndex,
                         nvinfer1::Dims3{1, edges, 1}, err)) {
        return {};
    }
    return WarmupBindings(
        {engine, stream,
         {m_netIndex, m_inpIndex, m_corrIndex, m_prevNetIndex, m_nextNetIndex,
          m_prevMaskIndex, m_nextMaskIndex, m_updatedNetIndex, m_deltaIndex,
          m_weightIndex},
         m_buffers, "update", err});
}

bool CopyFloatHostToBindingDevice(
    const TensorRtHostToDeviceCopyRequest &request)
{
    if (!ValidateHostToDeviceCopyRequest(request)) {
        return false;
    }
    size_t bytes = 0;
    if (!PrepareHostToDeviceBindingCopy(request, &bytes)) {
        return false;
    }
    const nvinfer1::DataType type =
        BindingDataType(request.handle, request.bindingIndex);
    if (type == nvinfer1::DataType::kFLOAT) {
        return CopyFloatHostToDeviceBuffer(request, bytes);
    }
    if (type == nvinfer1::DataType::kHALF) {
        return CopyHalfHostToDeviceBuffer(request, bytes);
    }
    SetTensorRtCopyError(request.err,
                         "unsupported TensorRT input dtype index=" +
                             std::to_string(request.bindingIndex));
    return false;
}

bool CopyBindingDeviceToFloatHost(
    const TensorRtDeviceToHostCopyRequest &request)
{
    if (!ValidateDeviceToHostCopyRequest(request)) {
        return false;
    }
    nvinfer1::Dims dims{};
    size_t valueCount = 0;
    size_t bytes = 0;
    if (!PrepareDeviceToHostBindingCopy(request, &valueCount, &bytes, &dims)) {
        return false;
    }
    const nvinfer1::DataType type =
        BindingDataType(request.handle, request.bindingIndex);
    request.dst.resize(valueCount);
    if (type == nvinfer1::DataType::kFLOAT) {
        if (!CopyFloatBindingDeviceToHost(request, bytes)) {
            return false;
        }
    } else if (type == nvinfer1::DataType::kHALF) {
        if (!CopyHalfBindingDeviceToHost(request, valueCount, bytes)) {
            return false;
        }
    } else {
        SetTensorRtCopyError(request.err,
                             "unsupported TensorRT output dtype index=" +
                                 std::to_string(request.bindingIndex));
        return false;
    }
    StoreOutputDims(request, dims);
    return true;
}

DpvoUpdateRun WarmupBindings(const DpvoWarmupBindingsRequest &request)
{
    DpvoUpdateRun result{};
    if (!ValidateWarmupRequest(request) || !WarmupBufferTableValid(request) ||
        !PrepareWarmupBindings(request)) {
        return result;
    }
    const auto t0 = std::chrono::steady_clock::now();
    if (!RunWarmupInference(request)) {
        return result;
    }
    result.elapsedMs =
        TensorRtElapsedMs(t0, std::chrono::steady_clock::now());
    result.ok = true;
    return result;
}

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
