#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_superpoint.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <NvInferPlugin.h>
#include <cuda_runtime_api.h>

#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_output.h"

namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal {

TensorRtSuperPointEngine::~TensorRtSuperPointEngine()
{
    Release();
}

bool TensorRtSuperPointEngine::Load(const std::filesystem::path &enginePath,
                                    std::string *err)
{
    Release();
    std::vector<char> bytes;
    return ReadEngineFile(enginePath, bytes, err) &&
           CreateRuntimeAndEngine(enginePath, bytes, err) &&
           CreateContextAndStream(err) && CreateTimingEvents(err) &&
           ResolveBindings(err);
}

bool TensorRtSuperPointEngine::Forward(
    const TensorRtSuperPointForwardRequest &request)
{
    if (request.stats != nullptr) {
        *request.stats = TensorRtForwardStats{};
        request.stats->eventTimingEnabled = m_eventTimingEnabled;
    }
    if (!PrepareInput(request) || !PrepareOutputBuffers(request) ||
        !BindTensorAddresses(request.err) || !Enqueue(request) ||
        !ScheduleOutputs(request)) {
        return false;
    }
    if (!Synchronize(request)) {
        return false;
    }
    const bool outputsReady =
        FinalizeTensorRtOutput(request.detector, request.stats) &&
        FinalizeTensorRtOutput(request.descriptors, request.stats);
    return outputsReady && !request.detector.Empty() &&
           !request.descriptors.Empty();
}

bool TensorRtSuperPointEngine::PreferredInputSize(int &height, int &width) const
{
    if (!m_engine || m_inputIndex < 0) {
        return false;
    }
    nvinfer1::Dims dims =
        TensorRtTensorShape(m_engine.get(), nullptr, m_inputIndex);
    if (m_engine->getNbOptimizationProfiles() > 0) {
        const char *name = TensorRtTensorName(m_engine.get(), m_inputIndex);
        dims = name != nullptr ? m_engine->getProfileShape(
                   name, 0, nvinfer1::OptProfileSelector::kOPT)
                               : nvinfer1::Dims{};
    }
    if (dims.nbDims == 4 && dims.d[2] > 0 && dims.d[3] > 0) {
        height = dims.d[2];
        width = dims.d[3];
        return true;
    }
    return false;
}

bool TensorRtSuperPointEngine::SupportsBatchSize(int batchSize) const
{
    if (!m_engine || m_inputIndex < 0 || batchSize <= 0) {
        return batchSize == 1;
    }
    const nvinfer1::Dims dims =
        TensorRtTensorShape(m_engine.get(), nullptr, m_inputIndex);
    if (dims.nbDims != 4) {
        return batchSize == 1;
    }
    if (dims.d[0] > 0) {
        return dims.d[0] == batchSize;
    }
    const int profiles = m_engine->getNbOptimizationProfiles();
    for (int profile = 0; profile < profiles; ++profile) {
        const char *name = TensorRtTensorName(m_engine.get(), m_inputIndex);
        const nvinfer1::Dims minDims =
            name != nullptr ? m_engine->getProfileShape(
                                  name, profile,
                                  nvinfer1::OptProfileSelector::kMIN)
                            : nvinfer1::Dims{};
        const nvinfer1::Dims maxDims =
            name != nullptr ? m_engine->getProfileShape(
                                  name, profile,
                                  nvinfer1::OptProfileSelector::kMAX)
                            : nvinfer1::Dims{};
        if (minDims.nbDims == 4 && maxDims.nbDims == 4 &&
            minDims.d[0] <= batchSize && batchSize <= maxDims.d[0]) {
            return true;
        }
    }
    return batchSize == 1;
}

int TensorRtSuperPointEngine::FindBinding(
    std::initializer_list<const char *> names, bool input) const
{
    return TensorRtFindTensor(m_engine.get(), names, input, true);
}

bool TensorRtSuperPointEngine::ReadEngineFile(
    const std::filesystem::path &enginePath, std::vector<char> &bytes,
    std::string *err) const
{
    std::ifstream input(enginePath, std::ios::binary);
    if (!input) {
        if (err != nullptr) {
            *err = "failed to open SuperPoint TensorRT engine: " +
                   enginePath.string();
        }
        return false;
    }
    bytes.assign(std::istreambuf_iterator<char>(input),
                 std::istreambuf_iterator<char>());
    if (!bytes.empty()) {
        return true;
    }
    if (err != nullptr) {
        *err = "SuperPoint TensorRT engine is empty: " + enginePath.string();
    }
    return false;
}

bool TensorRtSuperPointEngine::CreateRuntimeAndEngine(
    const std::filesystem::path &enginePath, const std::vector<char> &bytes,
    std::string *err)
{
    initLibNvInferPlugins(&m_logger, "");
    m_runtime.reset(nvinfer1::createInferRuntime(m_logger));
    if (!m_runtime) {
        if (err != nullptr) {
            *err = "failed to create SuperPoint TensorRT runtime";
        }
        return false;
    }
    m_engine.reset(
        m_runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (m_engine) {
        return true;
    }
    if (err != nullptr) {
        *err = "failed to deserialize SuperPoint TensorRT engine: " +
               enginePath.string();
    }
    return false;
}

bool TensorRtSuperPointEngine::CreateContextAndStream(std::string *err)
{
    m_context.reset(m_engine->createExecutionContext());
    if (!m_context) {
        if (err != nullptr) {
            *err = "failed to create SuperPoint TensorRT execution context";
        }
        return false;
    }
    if (cudaStreamCreate(&m_stream) == cudaSuccess) {
        return true;
    }
    if (err != nullptr) {
        *err = "failed to create SuperPoint TensorRT CUDA stream";
    }
    return false;
}

bool TensorRtSuperPointEngine::CreateTimingEvents(std::string *err)
{
    m_eventTimingEnabled = EnvFlag("SMART_DRONE_TRT_EVENT_TIMING", false);
    if (!m_eventTimingEnabled) {
        return true;
    }
    if (cudaEventCreate(&m_computeStartEvent) == cudaSuccess &&
        cudaEventCreate(&m_computeEndEvent) == cudaSuccess &&
        cudaEventCreate(&m_outputStartEvent) == cudaSuccess &&
        cudaEventCreate(&m_outputEndEvent) == cudaSuccess) {
        return true;
    }
    if (err != nullptr) {
        *err = "failed to create SuperPoint TensorRT CUDA timing events";
    }
    return false;
}

bool TensorRtSuperPointEngine::ResolveBindings(std::string *err)
{
    m_inputIndex = FindBinding({"image", "images", "input"}, true);
    m_detectorIndex =
        FindBinding({"detector_logits", "scores", "output0"}, false);
    m_descriptorIndex =
        FindBinding({"dense_descriptors", "descriptors", "output1"}, false);
    if (m_inputIndex >= 0 && m_detectorIndex >= 0 &&
        m_descriptorIndex >= 0) {
        return true;
    }
    if (err != nullptr) {
        *err = "SuperPoint TensorRT engine bindings are missing expected "
               "input/output names";
    }
    return false;
}

nvinfer1::Dims TensorRtSuperPointEngine::MakeInputDims(
    const TensorRtSuperPointForwardRequest &request) const
{
    nvinfer1::Dims inputDims{};
    inputDims.nbDims = 4;
    inputDims.d[0] = request.batchSize;
    inputDims.d[1] = 1;
    inputDims.d[2] = request.height;
    inputDims.d[3] = request.width;
    return inputDims;
}

bool TensorRtSuperPointEngine::PrepareInput(
    const TensorRtSuperPointForwardRequest &request)
{
    const nvinfer1::Dims inputDims = MakeInputDims(request);
    if (!TensorRtSetInputShape({m_engine.get(), m_context.get(), m_inputIndex,
                                inputDims, "SuperPoint", request.err}) ||
        !TensorRtInferShapes(m_context.get(), "SuperPoint", request.err) ||
        !EnsureBindingBuffer(m_inputIndex, inputDims,
                             TensorRtTensorDataType(m_engine.get(), m_inputIndex),
                             request.err)) {
        return false;
    }
    return CopyInputToDevice(request);
}

bool TensorRtSuperPointEngine::CopyInputToDevice(
    const TensorRtSuperPointForwardRequest &request)
{
    const size_t inputBytes = request.batch.size() * sizeof(float);
    const auto h2dStartTp = std::chrono::steady_clock::now();
    const cudaError_t rc =
        cudaMemcpyAsync(m_bindings[static_cast<size_t>(m_inputIndex)],
                        request.batch.data(), inputBytes,
                        cudaMemcpyHostToDevice, m_stream);
    if (rc != cudaSuccess) {
        if (request.err != nullptr) {
            *request.err = "TensorRT failed to copy SuperPoint input";
        }
        return false;
    }
    const auto h2dEndTp = std::chrono::steady_clock::now();
    if (request.stats != nullptr) {
        request.stats->h2dMs += DurationMs(h2dStartTp, h2dEndTp);
        request.stats->h2dBytes += inputBytes;
    }
    return true;
}

bool TensorRtSuperPointEngine::PrepareOutputBuffers(
    const TensorRtSuperPointForwardRequest &request)
{
    for (int index : {m_detectorIndex, m_descriptorIndex}) {
        const nvinfer1::Dims dims =
            TensorRtTensorShape(m_engine.get(), m_context.get(), index);
        if (!EnsureBindingBuffer(index, dims,
                                 TensorRtTensorDataType(m_engine.get(), index),
                                 request.err)) {
            return false;
        }
    }
    return true;
}

bool TensorRtSuperPointEngine::Enqueue(
    const TensorRtSuperPointForwardRequest &request)
{
    const auto enqueueStartTp = std::chrono::steady_clock::now();
    if (request.stats != nullptr && m_eventTimingEnabled) {
        cudaEventRecord(m_computeStartEvent, m_stream);
    }
    if (!m_context->enqueueV3(m_stream)) {
        if (request.err != nullptr) {
            *request.err = "TensorRT SuperPoint enqueue failed";
        }
        return false;
    }
    if (request.stats != nullptr && m_eventTimingEnabled) {
        cudaEventRecord(m_computeEndEvent, m_stream);
    }
    const auto enqueueEndTp = std::chrono::steady_clock::now();
    if (request.stats != nullptr) {
        request.stats->enqueueMs += DurationMs(enqueueStartTp, enqueueEndTp);
    }
    return true;
}

bool TensorRtSuperPointEngine::ScheduleOutputs(
    const TensorRtSuperPointForwardRequest &request)
{
    if (request.stats != nullptr && m_eventTimingEnabled) {
        cudaEventRecord(m_outputStartEvent, m_stream);
    }
    if (!ReadOutput(m_detectorIndex, request.detector, request.stats,
                    request.err) ||
        !ReadOutput(m_descriptorIndex, request.descriptors, request.stats,
                    request.err)) {
        return false;
    }
    if (request.stats != nullptr && m_eventTimingEnabled) {
        cudaEventRecord(m_outputEndEvent, m_stream);
    }
    return true;
}

bool TensorRtSuperPointEngine::Synchronize(
    const TensorRtSuperPointForwardRequest &request)
{
    const auto syncStartTp = std::chrono::steady_clock::now();
    const cudaError_t rc = cudaStreamSynchronize(m_stream);
    if (rc == cudaSuccess) {
        const auto syncEndTp = std::chrono::steady_clock::now();
        if (request.stats != nullptr) {
            request.stats->syncMs += DurationMs(syncStartTp, syncEndTp);
            RecordGpuTiming(request.stats);
        }
        return true;
    }
    if (request.err != nullptr) {
        *request.err = "TensorRT SuperPoint stream synchronize failed";
    }
    return false;
}

void TensorRtSuperPointEngine::RecordGpuTiming(
    TensorRtForwardStats *stats) const
{
    if (stats == nullptr || !m_eventTimingEnabled) {
        return;
    }
    float computeMs = 0.0f;
    float outputMs = 0.0f;
    if (cudaEventElapsedTime(&computeMs, m_computeStartEvent,
                             m_computeEndEvent) == cudaSuccess) {
        stats->gpuComputeMs += static_cast<double>(computeMs);
    }
    if (cudaEventElapsedTime(&outputMs, m_outputStartEvent,
                             m_outputEndEvent) == cudaSuccess) {
        stats->gpuOutputMs += static_cast<double>(outputMs);
    }
}

bool TensorRtSuperPointEngine::EnsureBindingBuffer(
    int index, const nvinfer1::Dims &dims, nvinfer1::DataType type,
    std::string *err)
{
    const size_t bindingIndex = static_cast<size_t>(index);
    const size_t tensorCount =
        static_cast<size_t>(TensorRtTensorCount(m_engine.get()));
    if (m_bindings.size() < tensorCount) {
        m_bindings.assign(tensorCount, nullptr);
        m_bindingBytes.assign(tensorCount, 0);
        m_pinnedHostOutputs.clear();
        m_pinnedHostOutputs.resize(tensorCount);
    }
    const int64_t volume = TensorRtVolume(dims);
    const size_t elementSize = TensorRtElementSize(type);
    if (volume <= 0 || elementSize == 0) {
        if (err != nullptr) {
            *err = "invalid SuperPoint TensorRT binding dimensions";
        }
        return false;
    }
    const size_t bytes = static_cast<size_t>(volume) * elementSize;
    if (m_bindingBytes[bindingIndex] >= bytes &&
        m_bindings[bindingIndex] != nullptr) {
        return true;
    }
    if (m_bindings[bindingIndex] != nullptr) {
        cudaFree(m_bindings[bindingIndex]);
        m_bindings[bindingIndex] = nullptr;
    }
    if (cudaMalloc(&m_bindings[bindingIndex], bytes) != cudaSuccess) {
        if (err != nullptr) {
            *err = "failed to allocate SuperPoint TensorRT binding buffer";
        }
        return false;
    }
    m_bindingBytes[bindingIndex] = bytes;
    return true;
}

bool TensorRtSuperPointEngine::BindTensorAddresses(std::string *err)
{
    const int tensorCount = TensorRtTensorCount(m_engine.get());
    for (int i = 0; i < tensorCount; ++i) {
        if (!TensorRtSetTensorAddress(
                {m_engine.get(), m_context.get(), i,
                 m_bindings[static_cast<size_t>(i)], "SuperPoint", err})) {
            return false;
        }
    }
    return true;
}

bool TensorRtSuperPointEngine::ReadOutput(
    int index, TensorBlob &output, TensorRtForwardStats *stats, std::string *err)
{
    const nvinfer1::Dims dims =
        TensorRtTensorShape(m_engine.get(), m_context.get(), index);
    output.dims = TensorRtDimsToVector(dims);
    const int64_t volume = TensorRtVolume(dims);
    if (volume <= 0) {
        return false;
    }
    const nvinfer1::DataType dtype =
        TensorRtTensorDataType(m_engine.get(), index);
    return ScheduleTensorRtOutputCopy(
        {m_bindings[static_cast<size_t>(index)], dtype, volume, m_stream,
         output, &m_pinnedHostOutputs[static_cast<size_t>(index)], stats,
         "SuperPoint", err});
}

void TensorRtSuperPointEngine::Release()
{
    if (m_stream != nullptr) {
        cudaStreamSynchronize(m_stream);
    }
    m_context.reset();
    if (m_computeStartEvent != nullptr) {
        cudaEventDestroy(m_computeStartEvent);
        m_computeStartEvent = nullptr;
    }
    if (m_computeEndEvent != nullptr) {
        cudaEventDestroy(m_computeEndEvent);
        m_computeEndEvent = nullptr;
    }
    if (m_outputStartEvent != nullptr) {
        cudaEventDestroy(m_outputStartEvent);
        m_outputStartEvent = nullptr;
    }
    if (m_outputEndEvent != nullptr) {
        cudaEventDestroy(m_outputEndEvent);
        m_outputEndEvent = nullptr;
    }
    for (void *ptr : m_bindings) {
        if (ptr != nullptr) {
            cudaFree(ptr);
        }
    }
    m_bindings.clear();
    m_bindingBytes.clear();
    m_pinnedHostOutputs.clear();
    if (m_stream != nullptr) {
        cudaStreamDestroy(m_stream);
        m_stream = nullptr;
    }
    m_engine.reset();
    m_runtime.reset();
}

} // namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal
