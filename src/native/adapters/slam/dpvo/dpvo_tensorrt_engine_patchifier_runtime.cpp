#include "adapters/slam/dpvo/dpvo_tensorrt_engine.h"

#include <NvInfer.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <opencv2/core.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "adapters/slam/dpvo/dpvo_tensorrt_engine_cuda_buffers.h"
#include "adapters/slam/dpvo/dpvo_tensorrt_engine_patchifier_run.h"

namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal {

#include "dpvo_tensorrt_engine_trt_handles.h"
#include "dpvo_tensorrt_engine_bindings.h"
#include "dpvo_tensorrt_engine_patchifier_runtime.h"

namespace {

double PatchifierElapsedMs(const std::chrono::steady_clock::time_point &start,
                           const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

void SetPatchifierError(std::string *err, const std::string &message)
{
    if (err != nullptr) {
        *err = message;
    }
}

} // namespace

bool DpvoPatchifierRuntime::Initialize(const TensorRtEngineHandle &engine,
                                       int width, int height, std::string *err)
{
    m_imageIndex = FindBindingIndex(engine, "image");
    m_fmapIndex = FindBindingIndex(engine, "fmap");
    m_imapIndex = FindBindingIndex(engine, "imap");
    if (m_imageIndex < 0 || m_fmapIndex < 0 || m_imapIndex < 0) {
        SetPatchifierError(err, "patchifier binding lookup failed: image=" +
                                    std::to_string(m_imageIndex) +
                                    " fmap=" + std::to_string(m_fmapIndex) +
                                    " imap=" + std::to_string(m_imapIndex));
        return false;
    }
    m_width = width;
    m_height = height;
    m_inputHost.assign(static_cast<size_t>(3) * static_cast<size_t>(width) *
                           static_cast<size_t>(height),
                       0.0f);
    return true;
}

DpvoPatchifierRun DpvoPatchifierRuntime::Run(const RunRequest &request)
{
    DpvoPatchifierRun result{};
    if (!ValidateRunRequest(request) ||
        !SetBindingShape(request.engine, m_imageIndex,
                         nvinfer1::Dims4{1, 3, m_height, m_width},
                         request.err)) {
        return result;
    }
    size_t imageBytes = 0;
    if (!PrepareRunBindings(request, &result, &imageBytes)) {
        return result;
    }
    FillInput(request.gray);
    const auto t0 = std::chrono::steady_clock::now();
    if (!CopyInputToDevice(request, imageBytes) || !Execute(request) ||
        !CopyRequestedOutputs(request, &result)) {
        return result;
    }
    result.elapsedMs =
        PatchifierElapsedMs(t0, std::chrono::steady_clock::now());
    result.ok = true;
    m_lastFmapDims = result.fmapDims;
    m_lastImapDims = result.imapDims;
    return result;
}

void *DpvoPatchifierRuntime::FmapDevice() const
{
    return m_fmapDevice.Data();
}

void *DpvoPatchifierRuntime::ImapDevice() const
{
    return m_imapDevice.Data();
}

nvinfer1::Dims DpvoPatchifierRuntime::LastFmapDims() const
{
    return m_lastFmapDims;
}

nvinfer1::Dims DpvoPatchifierRuntime::LastImapDims() const
{
    return m_lastImapDims;
}

bool DpvoPatchifierRuntime::ValidateRunRequest(
    const RunRequest &request) const
{
    const cv::Mat &gray = request.gray;
    if (gray.empty() || gray.cols != m_width || gray.rows != m_height ||
        gray.type() != CV_8UC1) {
        SetPatchifierError(request.err,
                           "patchifier expects CV_8UC1 " +
                               std::to_string(m_width) + "x" +
                               std::to_string(m_height));
        return false;
    }
    if (request.engine.Context() != nullptr && request.stream != nullptr) {
        return true;
    }
    SetPatchifierError(
        request.err, "patchifier TensorRT context or CUDA stream is not initialized");
    return false;
}

bool DpvoPatchifierRuntime::PrepareRunBindings(
    const RunRequest &request, DpvoPatchifierRun *result, size_t *imageBytes)
{
    size_t fmapBytes = 0;
    size_t imapBytes = 0;
    if (!BindingBytes(request.engine, m_imageIndex, imageBytes, nullptr,
                      request.err) ||
        !BindingBytes(request.engine, m_fmapIndex, &fmapBytes,
                      &result->fmapDims, request.err) ||
        !BindingBytes(request.engine, m_imapIndex, &imapBytes,
                      &result->imapDims, request.err)) {
        return false;
    }
    const size_t expectedBytes = m_inputHost.size() * sizeof(float);
    if (*imageBytes != expectedBytes) {
        SetPatchifierError(request.err,
                           "patchifier image binding byte mismatch expected=" +
                               std::to_string(expectedBytes) +
                               " got=" + std::to_string(*imageBytes));
        return false;
    }
    return m_imageDevice.Ensure(*imageBytes, request.err) &&
           m_fmapDevice.Ensure(fmapBytes, request.err) &&
           m_imapDevice.Ensure(imapBytes, request.err);
}

bool DpvoPatchifierRuntime::CopyInputToDevice(
    const RunRequest &request, size_t imageBytes)
{
    const cudaError_t rc =
        cudaMemcpyAsync(m_imageDevice.Data(), m_inputHost.data(), imageBytes,
                        cudaMemcpyHostToDevice, request.stream);
    if (rc == cudaSuccess) {
        return true;
    }
    SetPatchifierError(request.err, std::string("patchifier H2D copy failed: ") +
                                        cudaGetErrorString(rc));
    return false;
}

bool DpvoPatchifierRuntime::Execute(const RunRequest &request)
{
    if (!SetBindingAddress(request.engine, m_imageIndex, m_imageDevice.Data(),
                           request.err) ||
        !SetBindingAddress(request.engine, m_fmapIndex, m_fmapDevice.Data(),
                           request.err) ||
        !SetBindingAddress(request.engine, m_imapIndex, m_imapDevice.Data(),
                           request.err) ||
        !EnqueueTensorRt(request.engine, request.stream, "patchifier",
                         request.err)) {
        return false;
    }
    const cudaError_t rc = cudaStreamSynchronize(request.stream);
    if (rc == cudaSuccess) {
        return true;
    }
    SetPatchifierError(request.err,
                       std::string("patchifier synchronize failed: ") +
                           cudaGetErrorString(rc));
    return false;
}

bool DpvoPatchifierRuntime::CopyRequestedOutputs(
    const RunRequest &request, DpvoPatchifierRun *result)
{
    if (request.copyFmapToHost) {
        if (!CopyBindingDeviceToFloatHost(
                {request.engine, m_fmapIndex, m_fmapDevice, request.stream,
                 m_fmapHost, m_halfScratch, nullptr, request.err})) {
            return false;
        }
        result->fmapHost = m_fmapHost.data();
        result->fmapValueCount = m_fmapHost.size();
    }
    if (!request.copyImapToHost) {
        return true;
    }
    if (!CopyBindingDeviceToFloatHost(
            {request.engine, m_imapIndex, m_imapDevice, request.stream,
             m_imapHost, m_halfScratch, nullptr, request.err})) {
        return false;
    }
    result->imapHost = m_imapHost.data();
    result->imapValueCount = m_imapHost.size();
    return true;
}

void DpvoPatchifierRuntime::FillInput(const cv::Mat &gray)
{
    const size_t plane =
        static_cast<size_t>(m_width) * static_cast<size_t>(m_height);
    for (int y = 0; y < m_height; ++y) {
        const uint8_t *src = gray.ptr<uint8_t>(y);
        for (int x = 0; x < m_width; ++x) {
            const float value = static_cast<float>(src[x]);
            const size_t idx =
                static_cast<size_t>(y) * static_cast<size_t>(m_width) +
                static_cast<size_t>(x);
            m_inputHost[idx] = value;
            m_inputHost[plane + idx] = value;
            m_inputHost[2U * plane + idx] = value;
        }
    }
}

} // namespace SmartDrone::Adapters::Slam::DpvoTensorRtInternal
