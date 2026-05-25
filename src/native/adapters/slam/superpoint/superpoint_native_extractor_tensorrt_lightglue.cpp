#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_lightglue.h"

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

TensorRtLightGlueEngine::~TensorRtLightGlueEngine()
{
    Release();
}

bool TensorRtLightGlueEngine::Load(const std::filesystem::path &enginePath,
                                   std::string *err)
{
    Release();
    std::vector<char> bytes;
    return ReadEngineFile(enginePath, bytes, err) &&
           CreateRuntimeAndEngine(enginePath, bytes, err) &&
           CreateContextAndStream(err) && CreateTimingEvents(err) &&
           ResolveBindings(err);
}

int TensorRtLightGlueEngine::FixedPointCount() const
{
    if (!m_engine || m_kpts0Index < 0) {
        return 0;
    }
    nvinfer1::Dims dims =
        TensorRtTensorShape(m_engine.get(), nullptr, m_kpts0Index);
    if (dims.nbDims == 3 && dims.d[1] > 0) {
        return dims.d[1];
    }
    const int profiles = m_engine->getNbOptimizationProfiles();
    for (int profile = 0; profile < profiles; ++profile) {
        const char *name = TensorRtTensorName(m_engine.get(), m_kpts0Index);
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
        if (minDims.nbDims == 3 && maxDims.nbDims == 3 && minDims.d[1] > 0 &&
            minDims.d[1] == maxDims.d[1]) {
            return minDims.d[1];
        }
    }
    return 0;
}

bool TensorRtLightGlueEngine::Forward(
    const TensorRtLightGlueForwardRequest &request)
{
    if (request.stats != nullptr) {
        *request.stats = TensorRtForwardStats{};
        request.stats->eventTimingEnabled = m_eventTimingEnabled;
    }
    if (request.pointCount <= 0) {
        return false;
    }
    if (!SetFeatureInputs(request) || !SetImageSizeInputs(request) ||
        !PrepareOutputBindings(request) || !Enqueue(request) ||
        !ScheduleOutput(request)) {
        return false;
    }
    if (!Synchronize(request)) {
        return false;
    }
    const bool outputReady =
        FinalizeTensorRtOutput(request.scores, request.stats);
    return outputReady && !request.scores.Empty();
}

int TensorRtLightGlueEngine::FindBinding(
    std::initializer_list<const char *> names, bool input) const
{
    return TensorRtFindTensor(m_engine.get(), names, input, false);
}

bool TensorRtLightGlueEngine::ReadEngineFile(
    const std::filesystem::path &enginePath, std::vector<char> &bytes,
    std::string *err) const
{
    std::ifstream input(enginePath, std::ios::binary);
    if (!input) {
        if (err != nullptr) {
            *err = "failed to open LightGlue TensorRT engine: " +
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
        *err = "LightGlue TensorRT engine is empty: " + enginePath.string();
    }
    return false;
}

bool TensorRtLightGlueEngine::CreateRuntimeAndEngine(
    const std::filesystem::path &enginePath, const std::vector<char> &bytes,
    std::string *err)
{
    initLibNvInferPlugins(&m_logger, "");
    m_runtime.reset(nvinfer1::createInferRuntime(m_logger));
    if (!m_runtime) {
        if (err != nullptr) {
            *err = "failed to create LightGlue TensorRT runtime";
        }
        return false;
    }
    m_engine.reset(
        m_runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (m_engine) {
        return true;
    }
    if (err != nullptr) {
        *err = "failed to deserialize LightGlue TensorRT engine: " +
               enginePath.string();
    }
    return false;
}

bool TensorRtLightGlueEngine::CreateContextAndStream(std::string *err)
{
    m_context.reset(m_engine->createExecutionContext());
    if (!m_context) {
        if (err != nullptr) {
            *err = "failed to create LightGlue TensorRT execution context";
        }
        return false;
    }
    if (cudaStreamCreate(&m_stream) == cudaSuccess) {
        return true;
    }
    if (err != nullptr) {
        *err = "failed to create LightGlue TensorRT CUDA stream";
    }
    return false;
}

bool TensorRtLightGlueEngine::CreateTimingEvents(std::string *err)
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
        *err = "failed to create LightGlue TensorRT CUDA timing events";
    }
    return false;
}

bool TensorRtLightGlueEngine::ResolveBindings(std::string *err)
{
    m_kpts0Index = FindBinding({"keypoints0"}, true);
    m_kpts1Index = FindBinding({"keypoints1"}, true);
    m_desc0Index = FindBinding({"descriptors0"}, true);
    m_desc1Index = FindBinding({"descriptors1"}, true);
    m_size0Index = FindBinding({"image_size0"}, true);
    m_size1Index = FindBinding({"image_size1"}, true);
    m_scoresIndex =
        FindBinding({"assignment_scores", "scores", "output0"}, false);
    if (m_kpts0Index >= 0 && m_kpts1Index >= 0 && m_desc0Index >= 0 &&
        m_desc1Index >= 0 && m_size0Index >= 0 && m_size1Index >= 0 &&
        m_scoresIndex >= 0) {
        return true;
    }
    if (err != nullptr) {
        *err = "LightGlue TensorRT engine bindings are missing expected "
               "input/output names";
    }
    return false;
}

nvinfer1::Dims TensorRtLightGlueEngine::MakeDims(
    const std::vector<int> &dims)
{
    nvinfer1::Dims out{};
    out.nbDims = static_cast<int>(std::min<size_t>(dims.size(), 8));
    for (int i = 0; i < out.nbDims; ++i) {
        out.d[i] = dims[static_cast<size_t>(i)];
    }
    return out;
}

bool TensorRtLightGlueEngine::SetInput(const TensorRtFloatInputRequest &request)
{
    const nvinfer1::Dims trtDims = MakeDims(request.dims);
    if (!TensorRtSetInputShape({m_engine.get(), m_context.get(),
                                request.index, trtDims, "LightGlue",
                                request.err})) {
        return false;
    }
    if (!EnsureBindingBuffer(request.index, trtDims,
                             TensorRtTensorDataType(m_engine.get(),
                                                    request.index),
                             request.err)) {
        return false;
    }
    const size_t bytes = request.data.size() * sizeof(float);
    const auto h2dStartTp = std::chrono::steady_clock::now();
    if (cudaMemcpyAsync(m_bindings[static_cast<size_t>(request.index)],
                        request.data.data(), bytes, cudaMemcpyHostToDevice,
                        m_stream) != cudaSuccess) {
        if (request.err != nullptr) {
            *request.err = "TensorRT failed to copy LightGlue input";
        }
        return false;
    }
    const auto h2dEndTp = std::chrono::steady_clock::now();
    if (request.stats != nullptr) {
        request.stats->h2dMs += DurationMs(h2dStartTp, h2dEndTp);
        request.stats->h2dBytes += bytes;
    }
    return true;
}

bool TensorRtLightGlueEngine::SetFeatureInputs(
    const TensorRtLightGlueForwardRequest &request)
{
    const std::vector<int> keypointDims{1, request.pointCount, 2};
    const std::vector<int> descriptorDims{
        1, request.pointCount, SUPER_POINT_DESCRIPTOR_DIM};
    return SetInput({m_kpts0Index, keypointDims, request.keypoints0,
                     request.stats, request.err}) &&
           SetInput({m_kpts1Index, keypointDims, request.keypoints1,
                     request.stats, request.err}) &&
           SetInput({m_desc0Index, descriptorDims, request.descriptors0,
                     request.stats, request.err}) &&
           SetInput({m_desc1Index, descriptorDims, request.descriptors1,
                     request.stats, request.err});
}

bool TensorRtLightGlueEngine::SetImageSizeInputs(
    const TensorRtLightGlueForwardRequest &request)
{
    const std::vector<float> size0{request.imageSize0[0],
                                   request.imageSize0[1]};
    const std::vector<float> size1{request.imageSize1[0],
                                   request.imageSize1[1]};
    const std::vector<int> sizeDims{1, 2};
    return SetInput(
               {m_size0Index, sizeDims, size0, request.stats, request.err}) &&
           SetInput(
               {m_size1Index, sizeDims, size1, request.stats, request.err});
}

bool TensorRtLightGlueEngine::PrepareOutputBindings(
    const TensorRtLightGlueForwardRequest &request)
{
    if (!TensorRtInferShapes(m_context.get(), "LightGlue", request.err)) {
        return false;
    }
    const nvinfer1::Dims dims =
        TensorRtTensorShape(m_engine.get(), m_context.get(), m_scoresIndex);
    return EnsureBindingBuffer(
        m_scoresIndex, dims,
        TensorRtTensorDataType(m_engine.get(), m_scoresIndex), request.err) &&
           BindTensorAddresses(request.err);
}

bool TensorRtLightGlueEngine::Enqueue(
    const TensorRtLightGlueForwardRequest &request)
{
    const auto enqueueStartTp = std::chrono::steady_clock::now();
    if (request.stats != nullptr && m_eventTimingEnabled) {
        cudaEventRecord(m_computeStartEvent, m_stream);
    }
    if (!m_context->enqueueV3(m_stream)) {
        if (request.err != nullptr) {
            *request.err = "TensorRT LightGlue enqueue failed";
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

bool TensorRtLightGlueEngine::ScheduleOutput(
    const TensorRtLightGlueForwardRequest &request)
{
    if (request.stats != nullptr && m_eventTimingEnabled) {
        cudaEventRecord(m_outputStartEvent, m_stream);
    }
    if (!ReadOutput(m_scoresIndex, request.scores, request.stats,
                    request.err)) {
        return false;
    }
    if (request.stats != nullptr && m_eventTimingEnabled) {
        cudaEventRecord(m_outputEndEvent, m_stream);
    }
    return true;
}

bool TensorRtLightGlueEngine::Synchronize(
    const TensorRtLightGlueForwardRequest &request)
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
        *request.err = "TensorRT LightGlue stream synchronize failed";
    }
    return false;
}

void TensorRtLightGlueEngine::RecordGpuTiming(TensorRtForwardStats *stats) const
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

bool TensorRtLightGlueEngine::EnsureBindingBuffer(
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
            *err = "invalid LightGlue TensorRT binding dimensions";
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
            *err = "failed to allocate LightGlue TensorRT binding buffer";
        }
        return false;
    }
    m_bindingBytes[bindingIndex] = bytes;
    return true;
}

bool TensorRtLightGlueEngine::BindTensorAddresses(std::string *err)
{
    const int tensorCount = TensorRtTensorCount(m_engine.get());
    for (int i = 0; i < tensorCount; ++i) {
        if (!TensorRtSetTensorAddress(
                {m_engine.get(), m_context.get(), i,
                 m_bindings[static_cast<size_t>(i)], "LightGlue", err})) {
            return false;
        }
    }
    return true;
}

bool TensorRtLightGlueEngine::ReadOutput(
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
         "LightGlue", err});
}

void TensorRtLightGlueEngine::Release()
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
