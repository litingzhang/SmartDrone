#include "adapters/slam/dpvo/dpvo_tensorrt_engine.h"

#include <NvInfer.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
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
#include "dpvo_tensorrt_engine_update_preagg_runtime.h"

namespace {

double PreAggElapsedMs(const std::chrono::steady_clock::time_point &start,
                       const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

void SetPreAggError(std::string *err, const std::string &message)
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
    SetPreAggError(err, std::string(name) +
                            " TensorRT context or CUDA stream is not initialized");
    return false;
}

} // namespace

bool DpvoUpdatePreAggRuntime::Initialize(const TensorRtEngineHandle &engine,
                                         std::string *err)
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
    const bool ok = m_netIndex >= 0 && m_inpIndex >= 0 && m_corrIndex >= 0 &&
                    m_prevNetIndex >= 0 && m_nextNetIndex >= 0 &&
                    m_prevMaskIndex >= 0 && m_nextMaskIndex >= 0 &&
                    m_baseNetIndex >= 0 && m_kkFIndex >= 0 &&
                    m_kkGIndex >= 0 && m_ijFIndex >= 0 && m_ijGIndex >= 0;
    if (!ok) {
        SetPreAggError(err, "update-preagg binding lookup failed");
    }
    return ok;
}

DpvoUpdateRun DpvoUpdatePreAggRuntime::Warmup(TensorRtEngineHandle &engine,
                                              cudaStream_t stream, int edges,
                                              std::string *err)
{
    edges = std::clamp(edges, 1, 4096);
    if (!ContextReady(engine, stream, "update-preagg", err) ||
        !ConfigureInputShapes(engine, edges, err)) {
        return {};
    }
    return WarmupBindings(
        {engine, stream,
         {m_netIndex, m_inpIndex, m_corrIndex, m_prevNetIndex, m_nextNetIndex,
          m_prevMaskIndex, m_nextMaskIndex, m_baseNetIndex, m_kkFIndex,
          m_kkGIndex, m_ijFIndex, m_ijGIndex},
         m_buffers, "update-preagg", err});
}

DpvoUpdatePreAggRun DpvoUpdatePreAggRuntime::Run(
    const RunRequest &request)
{
    DpvoUpdatePreAggRun result{};
    const int edges = std::clamp(request.edges, 1, 4096);
    if (!ContextReady(request.engine, request.stream, "update-preagg",
                      request.err) ||
        !ConfigureInputShapes(request.engine, edges, request.err) ||
        !HostInputSizesValid(
            request, edges, "update-preagg input vector size mismatch edges=") ||
        !EnsureOutputBuffers(request.engine, request.err) ||
        !CopyInputs(request, edges)) {
        return result;
    }
    const auto t0 = std::chrono::steady_clock::now();
    if (!RunInference(request.engine, request.stream, "update-preagg",
                      request.err) ||
        !CopyOutputs(request.engine, request.stream, &result, request.err)) {
        return result;
    }
    result.elapsedMs = PreAggElapsedMs(t0, std::chrono::steady_clock::now());
    result.ok = true;
    return result;
}

DpvoUpdatePreAggDeviceRun DpvoUpdatePreAggRuntime::RunDevice(
    const RunRequest &request)
{
    DpvoUpdatePreAggDeviceRun result{};
    const int edges = std::clamp(request.edges, 1, 4096);
    if (!ContextReady(request.engine, request.stream, "update-preagg device",
                      request.err) ||
        !ConfigureInputShapes(request.engine, edges, request.err) ||
        !OutputBindingsAreFloat(request.engine, request.err) ||
        !HostInputSizesValid(
            request, edges,
            "update-preagg device input vector size mismatch edges=") ||
        !EnsureOutputBuffers(request.engine, request.err) ||
        !CopyInputs(request, edges)) {
        return result;
    }
    const auto t0 = std::chrono::steady_clock::now();
    if (!RunInference(request.engine, request.stream, "update-preagg device",
                      request.err)) {
        return result;
    }
    result.elapsedMs = PreAggElapsedMs(t0, std::chrono::steady_clock::now());
    result.ok = true;
    return result;
}

bool DpvoUpdatePreAggRuntime::ConfigureInputShapes(
    TensorRtEngineHandle &engine, int edges, std::string *err) const
{
    return SetBindingShape(engine, m_netIndex,
                           nvinfer1::Dims3{1, edges, DIM}, err) &&
           SetBindingShape(engine, m_inpIndex,
                           nvinfer1::Dims3{1, edges, DIM}, err) &&
           SetBindingShape(engine, m_corrIndex,
                           nvinfer1::Dims3{1, edges, CORR_DIM}, err) &&
           SetBindingShape(engine, m_prevNetIndex,
                           nvinfer1::Dims3{1, edges, DIM}, err) &&
           SetBindingShape(engine, m_nextNetIndex,
                           nvinfer1::Dims3{1, edges, DIM}, err) &&
           SetBindingShape(engine, m_prevMaskIndex,
                           nvinfer1::Dims3{1, edges, 1}, err) &&
           SetBindingShape(engine, m_nextMaskIndex,
                           nvinfer1::Dims3{1, edges, 1}, err);
}

bool DpvoUpdatePreAggRuntime::HostInputSizesValid(
    const RunRequest &request, int edges, const char *messagePrefix) const
{
    const size_t dimValues = static_cast<size_t>(edges) * DIM;
    const size_t corrValues = static_cast<size_t>(edges) * CORR_DIM;
    const size_t maskValues = static_cast<size_t>(edges);
    if (request.net.size() == dimValues && request.inp.size() == dimValues &&
        request.corr.size() == corrValues &&
        request.prevNet.size() == dimValues &&
        request.nextNet.size() == dimValues &&
        request.prevMask.size() == maskValues &&
        request.nextMask.size() == maskValues) {
        return true;
    }
    SetPreAggError(request.err, std::string(messagePrefix) +
                                    std::to_string(edges));
    return false;
}

bool DpvoUpdatePreAggRuntime::OutputBindingsAreFloat(
    TensorRtEngineHandle &engine, std::string *err) const
{
    if (BindingIsFloat(engine, m_baseNetIndex) &&
        BindingIsFloat(engine, m_kkFIndex) &&
        BindingIsFloat(engine, m_kkGIndex) &&
        BindingIsFloat(engine, m_ijFIndex) &&
        BindingIsFloat(engine, m_ijGIndex)) {
        return true;
    }
    SetPreAggError(err, "update-preagg device path requires FP32 output bindings");
    return false;
}

bool DpvoUpdatePreAggRuntime::EnsureOutputBuffers(
    TensorRtEngineHandle &engine, std::string *err)
{
    return EnsureBindingBuffer(engine, m_baseNetIndex,
                               m_buffers[static_cast<size_t>(m_baseNetIndex)],
                               err) &&
           EnsureBindingBuffer(engine, m_kkFIndex,
                               m_buffers[static_cast<size_t>(m_kkFIndex)],
                               err) &&
           EnsureBindingBuffer(engine, m_kkGIndex,
                               m_buffers[static_cast<size_t>(m_kkGIndex)],
                               err) &&
           EnsureBindingBuffer(engine, m_ijFIndex,
                               m_buffers[static_cast<size_t>(m_ijFIndex)],
                               err) &&
           EnsureBindingBuffer(engine, m_ijGIndex,
                               m_buffers[static_cast<size_t>(m_ijGIndex)],
                               err);
}

bool DpvoUpdatePreAggRuntime::CopyInputs(const RunRequest &request, int edges)
{
    const size_t dimValues = static_cast<size_t>(edges) * DIM;
    const size_t corrValues = static_cast<size_t>(edges) * CORR_DIM;
    const size_t maskValues = static_cast<size_t>(edges);
    return CopyFloatHostToBindingDevice(
               {request.engine, m_netIndex, request.net.data(), dimValues,
                m_buffers[static_cast<size_t>(m_netIndex)], request.stream,
                m_halfScratch, request.err}) &&
           CopyFloatHostToBindingDevice(
               {request.engine, m_inpIndex, request.inp.data(), dimValues,
                m_buffers[static_cast<size_t>(m_inpIndex)], request.stream,
                m_halfScratch, request.err}) &&
           CopyFloatHostToBindingDevice(
               {request.engine, m_corrIndex, request.corr.data(), corrValues,
                m_buffers[static_cast<size_t>(m_corrIndex)], request.stream,
                m_halfScratch, request.err}) &&
           CopyFloatHostToBindingDevice(
               {request.engine, m_prevNetIndex, request.prevNet.data(),
                dimValues, m_buffers[static_cast<size_t>(m_prevNetIndex)],
                request.stream, m_halfScratch, request.err}) &&
           CopyFloatHostToBindingDevice(
               {request.engine, m_nextNetIndex, request.nextNet.data(),
                dimValues, m_buffers[static_cast<size_t>(m_nextNetIndex)],
                request.stream, m_halfScratch, request.err}) &&
           CopyFloatHostToBindingDevice(
               {request.engine, m_prevMaskIndex, request.prevMask.data(),
                maskValues, m_buffers[static_cast<size_t>(m_prevMaskIndex)],
                request.stream, m_halfScratch, request.err}) &&
           CopyFloatHostToBindingDevice(
               {request.engine, m_nextMaskIndex, request.nextMask.data(),
                maskValues, m_buffers[static_cast<size_t>(m_nextMaskIndex)],
                request.stream, m_halfScratch, request.err});
}

bool DpvoUpdatePreAggRuntime::RunInference(TensorRtEngineHandle &engine,
                                           cudaStream_t stream,
                                           const char *name, std::string *err)
{
    if (!SetBindingBufferAddresses(engine, m_buffers, err) ||
        !EnqueueTensorRt(engine, stream, name, err)) {
        return false;
    }
    const cudaError_t rc = cudaStreamSynchronize(stream);
    if (rc == cudaSuccess) {
        return true;
    }
    SetPreAggError(err, std::string(name) +
                            " synchronize failed: " + cudaGetErrorString(rc));
    return false;
}

bool DpvoUpdatePreAggRuntime::CopyOutputs(
    TensorRtEngineHandle &engine, cudaStream_t stream,
    DpvoUpdatePreAggRun *result, std::string *err)
{
    if (result == nullptr) {
        return false;
    }
    return CopyBindingDeviceToFloatHost(
               {engine, m_baseNetIndex,
                m_buffers[static_cast<size_t>(m_baseNetIndex)], stream,
                result->baseNet, m_halfScratch, nullptr, err}) &&
           CopyBindingDeviceToFloatHost(
               {engine, m_kkFIndex,
                m_buffers[static_cast<size_t>(m_kkFIndex)], stream,
                result->aggKkF, m_halfScratch, nullptr, err}) &&
           CopyBindingDeviceToFloatHost(
               {engine, m_kkGIndex,
                m_buffers[static_cast<size_t>(m_kkGIndex)], stream,
                result->aggKkG, m_halfScratch, nullptr, err}) &&
           CopyBindingDeviceToFloatHost(
               {engine, m_ijFIndex,
                m_buffers[static_cast<size_t>(m_ijFIndex)], stream,
                result->aggIjF, m_halfScratch, nullptr, err}) &&
           CopyBindingDeviceToFloatHost(
               {engine, m_ijGIndex,
                m_buffers[static_cast<size_t>(m_ijGIndex)], stream,
                result->aggIjG, m_halfScratch, nullptr, err});
}

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
