#include "adapters/slam/dpvo/dpvo_tensorrt_engine.h"

#include <NvInfer.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
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
#include "dpvo_tensorrt_engine_update_postagg_runtime.h"

namespace {

double PostAggElapsedMs(const std::chrono::steady_clock::time_point &start,
                        const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

void SetPostAggError(std::string *err, const std::string &message)
{
    if (err != nullptr) {
        *err = message;
    }
}

bool ContextReady(TensorRtEngineHandle &engine, cudaStream_t stream,
                  const char *name, std::string *err)
{
    if (engine.Context() != nullptr && stream != nullptr) {
        return true;
    }
    SetPostAggError(err, std::string(name) +
                             " TensorRT context or CUDA stream is not initialized");
    return false;
}

bool DeviceInputsAreFloat(TensorRtEngineHandle &engine, int baseNetIndex,
                          int kkYIndex, int ijYIndex, std::string *err)
{
    if (BindingIsFloat(engine, baseNetIndex) &&
        BindingIsFloat(engine, kkYIndex) && BindingIsFloat(engine, ijYIndex)) {
        return true;
    }
    SetPostAggError(err,
                    "update-postagg device path requires FP32 input bindings");
    return false;
}

} // namespace

bool DpvoUpdatePostAggRuntime::Initialize(const TensorRtEngineHandle &engine,
                                          std::string *err)
{
    m_baseNetIndex = FindBindingIndex(engine, "base_net");
    m_kkYIndex = FindBindingIndex(engine, "agg_kk_y");
    m_ijYIndex = FindBindingIndex(engine, "agg_ij_y");
    m_updatedNetIndex = FindBindingIndex(engine, "updated_net");
    m_deltaIndex = FindBindingIndex(engine, "delta");
    m_weightIndex = FindBindingIndex(engine, "weight");
    const bool ok = m_baseNetIndex >= 0 && m_kkYIndex >= 0 &&
                    m_ijYIndex >= 0 && m_updatedNetIndex >= 0 &&
                    m_deltaIndex >= 0 && m_weightIndex >= 0;
    if (!ok) {
        SetPostAggError(err, "update-postagg binding lookup failed");
    }
    return ok;
}

DpvoUpdateRun DpvoUpdatePostAggRuntime::Warmup(TensorRtEngineHandle &engine,
                                               cudaStream_t stream, int edges,
                                               std::string *err)
{
    edges = std::clamp(edges, 1, 4096);
    if (!ContextReady(engine, stream, "update-postagg", err) ||
        !ConfigureInputShapes(engine, edges, err)) {
        return {};
    }
    return WarmupBindings({engine, stream,
                           {m_baseNetIndex, m_kkYIndex, m_ijYIndex,
                            m_updatedNetIndex, m_deltaIndex, m_weightIndex},
                           m_buffers, "update-postagg", err});
}

DpvoUpdatePostAggRun DpvoUpdatePostAggRuntime::Run(
    const RunRequest &request)
{
    DpvoUpdatePostAggRun result{};
    const int edges = std::clamp(request.edges, 1, 4096);
    if (!ContextReady(request.engine, request.stream, "update-postagg",
                      request.err) ||
        !ConfigureInputShapes(request.engine, edges, request.err) ||
        !HostInputSizesValid(request, edges) ||
        !EnsureOutputBuffers(request.engine, request.err) ||
        !CopyHostInputs(request, edges)) {
        return result;
    }
    const auto t0 = std::chrono::steady_clock::now();
    if (!RunInference(request.engine, request.stream, "update-postagg",
                      request.err) ||
        !CopyOutputs(request.engine, request.stream, &result, request.err)) {
        return result;
    }
    result.elapsedMs = PostAggElapsedMs(t0, std::chrono::steady_clock::now());
    result.ok = true;
    return result;
}

DpvoUpdatePostAggRun DpvoUpdatePostAggRuntime::RunDevice(
    const RunDeviceRequest &request)
{
    DpvoUpdatePostAggRun result{};
    const int edges = std::clamp(request.edges, 1, 4096);
    if (!ContextReady(request.engine, request.stream, "update-postagg device",
                      request.err) ||
        !ConfigureInputShapes(request.engine, edges, request.err) ||
        !DeviceInputsAreFloat(request.engine, m_baseNetIndex, m_kkYIndex,
                              m_ijYIndex, request.err) ||
        !DeviceInputSizesValid(request, edges) ||
        !EnsureOutputBuffers(request.engine, request.err) ||
        !BindDeviceInputs(request, edges)) {
        return result;
    }
    const auto t0 = std::chrono::steady_clock::now();
    if (!EnqueueAndSynchronize(request.engine, request.stream,
                               "update-postagg device", request.err) ||
        !CopyOutputs(request.engine, request.stream, &result, request.err)) {
        return result;
    }
    result.elapsedMs = PostAggElapsedMs(t0, std::chrono::steady_clock::now());
    result.ok = true;
    return result;
}

bool DpvoUpdatePostAggRuntime::ConfigureInputShapes(
    TensorRtEngineHandle &engine, int edges, std::string *err) const
{
    return SetBindingShape(engine, m_baseNetIndex,
                           nvinfer1::Dims3{1, edges, DIM}, err) &&
           SetBindingShape(engine, m_kkYIndex,
                           nvinfer1::Dims3{1, edges, DIM}, err) &&
           SetBindingShape(engine, m_ijYIndex,
                           nvinfer1::Dims3{1, edges, DIM}, err);
}

bool DpvoUpdatePostAggRuntime::HostInputSizesValid(
    const RunRequest &request, int edges) const
{
    const size_t dimValues = static_cast<size_t>(edges) * DIM;
    if (request.baseNet.size() == dimValues &&
        request.aggKkY.size() == dimValues &&
        request.aggIjY.size() == dimValues) {
        return true;
    }
    SetPostAggError(request.err,
                    "update-postagg input vector size mismatch edges=" +
                        std::to_string(edges));
    return false;
}

bool DpvoUpdatePostAggRuntime::DeviceInputSizesValid(
    const RunDeviceRequest &request, int edges) const
{
    const size_t dimBytes =
        static_cast<size_t>(edges) * DIM * sizeof(float);
    if (request.baseNetDevice.Bytes() >= dimBytes &&
        request.aggKkYDevice.Bytes() >= dimBytes &&
        request.aggIjYDevice.Bytes() >= dimBytes) {
        return true;
    }
    SetPostAggError(request.err,
                    "update-postagg device input buffer size mismatch edges=" +
                        std::to_string(edges));
    return false;
}

bool DpvoUpdatePostAggRuntime::EnsureOutputBuffers(
    TensorRtEngineHandle &engine, std::string *err)
{
    return EnsureBindingBuffer(engine, m_updatedNetIndex,
                               m_buffers[static_cast<size_t>(m_updatedNetIndex)],
                               err) &&
           EnsureBindingBuffer(engine, m_deltaIndex,
                               m_buffers[static_cast<size_t>(m_deltaIndex)],
                               err) &&
           EnsureBindingBuffer(engine, m_weightIndex,
                               m_buffers[static_cast<size_t>(m_weightIndex)],
                               err);
}

bool DpvoUpdatePostAggRuntime::CopyHostInputs(
    const RunRequest &request, int edges)
{
    const size_t dimValues = static_cast<size_t>(edges) * DIM;
    return CopyFloatHostToBindingDevice(
               {request.engine, m_baseNetIndex, request.baseNet.data(),
                dimValues, m_buffers[static_cast<size_t>(m_baseNetIndex)],
                request.stream, m_halfScratch, request.err}) &&
           CopyFloatHostToBindingDevice(
               {request.engine, m_kkYIndex, request.aggKkY.data(), dimValues,
                m_buffers[static_cast<size_t>(m_kkYIndex)], request.stream,
                m_halfScratch, request.err}) &&
           CopyFloatHostToBindingDevice(
               {request.engine, m_ijYIndex, request.aggIjY.data(), dimValues,
                m_buffers[static_cast<size_t>(m_ijYIndex)], request.stream,
                m_halfScratch, request.err});
}

bool DpvoUpdatePostAggRuntime::BindDeviceInputs(
    const RunDeviceRequest &request, int edges)
{
    static_cast<void>(edges);
    return SetBindingBufferAddresses(request.engine, m_buffers, request.err) &&
           SetBindingAddress(request.engine, m_baseNetIndex,
                             const_cast<void *>(request.baseNetDevice.Data()),
                             request.err) &&
           SetBindingAddress(request.engine, m_kkYIndex,
                             const_cast<void *>(request.aggKkYDevice.Data()),
                             request.err) &&
           SetBindingAddress(request.engine, m_ijYIndex,
                             const_cast<void *>(request.aggIjYDevice.Data()),
                             request.err);
}

bool DpvoUpdatePostAggRuntime::RunInference(TensorRtEngineHandle &engine,
                                            cudaStream_t stream,
                                            const char *name, std::string *err)
{
    if (!SetBindingBufferAddresses(engine, m_buffers, err)) {
        return false;
    }
    return EnqueueAndSynchronize(engine, stream, name, err);
}

bool DpvoUpdatePostAggRuntime::EnqueueAndSynchronize(
    TensorRtEngineHandle &engine, cudaStream_t stream, const char *name,
    std::string *err)
{
    if (!EnqueueTensorRt(engine, stream, name, err)) {
        return false;
    }
    const cudaError_t rc = cudaStreamSynchronize(stream);
    if (rc == cudaSuccess) {
        return true;
    }
    SetPostAggError(err, std::string(name) +
                             " synchronize failed: " + cudaGetErrorString(rc));
    return false;
}

bool DpvoUpdatePostAggRuntime::CopyOutputs(
    TensorRtEngineHandle &engine, cudaStream_t stream,
    DpvoUpdatePostAggRun *result, std::string *err)
{
    if (result == nullptr) {
        return false;
    }
    return CopyBindingDeviceToFloatHost(
               {engine, m_updatedNetIndex,
                m_buffers[static_cast<size_t>(m_updatedNetIndex)], stream,
                result->updatedNet, m_halfScratch, nullptr, err}) &&
           CopyBindingDeviceToFloatHost(
               {engine, m_deltaIndex,
                m_buffers[static_cast<size_t>(m_deltaIndex)], stream,
                result->delta, m_halfScratch, nullptr, err}) &&
           CopyBindingDeviceToFloatHost(
               {engine, m_weightIndex,
                m_buffers[static_cast<size_t>(m_weightIndex)], stream,
                result->weight, m_halfScratch, nullptr, err});
}

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
