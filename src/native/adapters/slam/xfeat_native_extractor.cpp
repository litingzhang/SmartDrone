#include "adapters/slam/xfeat_native_extractor.h"

#include "adapters/slam/xfeat_frontend_client.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <utility>

#include <opencv2/imgproc.hpp>

#if defined(SMART_DRONE_XFEAT_TENSORRT_AVAILABLE)
#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_runtime_api.h>
#endif

namespace smartdrone::adapters::slam {

XFeatNativeExtractor::XFeatNativeExtractor() = default;

namespace {

constexpr int kDescriptorDim = 64;
constexpr int kXFeatCellSize = 8;

double DurationMs(const std::chrono::steady_clock::time_point &start,
                  const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

std::string LowerCopy(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

std::filesystem::path ResolveTensorRtEnginePath(const std::string &repoPath)
{
    const std::filesystem::path repo(repoPath);
    if (!repo.empty()) {
        for (const char *filename : {"xfeat_trt_fp16.engine", "xfeat_trt_fp32.engine", "xfeat.engine"}) {
            const std::filesystem::path candidate = repo / "weights" / filename;
            if (std::filesystem::exists(candidate)) {
                return candidate;
            }
        }
    }
    return {};
}

struct TensorBlob {
    std::vector<int> dims;
    std::vector<float> data;

    int Dim(size_t index) const { return index < dims.size() ? dims[index] : 0; }
    bool Empty() const { return dims.empty() || data.empty(); }
};

struct Candidate {
    int batch{0};
    int x{0};
    int y{0};
    float score{0.0f};
};

float At4D(const TensorBlob &blob, int b, int c, int y, int x)
{
    const int channels = blob.Dim(1);
    const int height = blob.Dim(2);
    const int width = blob.Dim(3);
    const size_t index = (((static_cast<size_t>(b) * channels + c) * height + y) * width + x);
    return blob.data[index];
}

void NormalizeVector(float *values, int count)
{
    float normSq = 0.0f;
    for (int i = 0; i < count; ++i) {
        normSq += values[i] * values[i];
    }
    const float invNorm = normSq > 1.0e-12f ? 1.0f / std::sqrt(normSq) : 1.0f;
    for (int i = 0; i < count; ++i) {
        values[i] *= invNorm;
    }
}

std::vector<float> BuildInputBatch(const std::vector<cv::Mat> &images, int targetHeight, int targetWidth)
{
    std::vector<float> batch(static_cast<size_t>(images.size()) * targetHeight * targetWidth);
    for (size_t i = 0; i < images.size(); ++i) {
        cv::Mat resized;
        if (images[i].rows != targetHeight || images[i].cols != targetWidth) {
            cv::resize(images[i], resized, cv::Size(targetWidth, targetHeight), 0.0, 0.0, cv::INTER_LINEAR);
        } else {
            resized = images[i];
        }
        float *dst = batch.data() + i * static_cast<size_t>(targetHeight) * static_cast<size_t>(targetWidth);
        for (int y = 0; y < targetHeight; ++y) {
            const uint8_t *src = resized.ptr<uint8_t>(y);
            for (int x = 0; x < targetWidth; ++x) {
                dst[static_cast<size_t>(y) * targetWidth + x] = static_cast<float>(src[x]) / 255.0f;
            }
        }
    }
    return batch;
}

float KeypointHeatmapAt(const TensorBlob &keypointLogits, int batch, int y, int x)
{
    const int gridY = y / kXFeatCellSize;
    const int gridX = x / kXFeatCellSize;
    const int offsetY = y % kXFeatCellSize;
    const int offsetX = x % kXFeatCellSize;
    const int channel = offsetY * kXFeatCellSize + offsetX;
    float maxLogit = At4D(keypointLogits, batch, 0, gridY, gridX);
    for (int c = 1; c < 65; ++c) {
        maxLogit = std::max(maxLogit, At4D(keypointLogits, batch, c, gridY, gridX));
    }
    float denom = 0.0f;
    for (int c = 0; c < 65; ++c) {
        denom += std::exp(At4D(keypointLogits, batch, c, gridY, gridX) - maxLogit);
    }
    return denom > 0.0f ? std::exp(At4D(keypointLogits, batch, channel, gridY, gridX) - maxLogit) / denom : 0.0f;
}

float BilinearAt(const TensorBlob &blob, int batch, int channel, float x, float y)
{
    const int height = blob.Dim(2);
    const int width = blob.Dim(3);
    const float gx = std::clamp(x, 0.0f, static_cast<float>(width - 1));
    const float gy = std::clamp(y, 0.0f, static_cast<float>(height - 1));
    const int x0 = static_cast<int>(std::floor(gx));
    const int y0 = static_cast<int>(std::floor(gy));
    const int x1 = std::min(x0 + 1, width - 1);
    const int y1 = std::min(y0 + 1, height - 1);
    const float wx = gx - static_cast<float>(x0);
    const float wy = gy - static_cast<float>(y0);
    const float v00 = At4D(blob, batch, channel, y0, x0);
    const float v01 = At4D(blob, batch, channel, y0, x1);
    const float v10 = At4D(blob, batch, channel, y1, x0);
    const float v11 = At4D(blob, batch, channel, y1, x1);
    return (1.0f - wx) * (1.0f - wy) * v00 + wx * (1.0f - wy) * v01 + (1.0f - wx) * wy * v10 + wx * wy * v11;
}

std::vector<Candidate> ExtractCandidates(const TensorBlob &keypointLogits, const TensorBlob &reliability, int batch,
                                         int maxPoints)
{
    const int gridHeight = keypointLogits.Dim(2);
    const int gridWidth = keypointLogits.Dim(3);
    std::vector<Candidate> candidates;
    candidates.reserve(static_cast<size_t>(gridHeight * gridWidth));
    constexpr float kThreshold = 0.05f;
    constexpr float kNmsRadiusPx = 4.0f;
    constexpr float kNmsRadiusSq = kNmsRadiusPx * kNmsRadiusPx;
    for (int gy = 0; gy < gridHeight; ++gy) {
        for (int gx = 0; gx < gridWidth; ++gx) {
            float maxLogit = At4D(keypointLogits, batch, 0, gy, gx);
            for (int c = 1; c < 65; ++c) {
                maxLogit = std::max(maxLogit, At4D(keypointLogits, batch, c, gy, gx));
            }
            float denom = 0.0f;
            for (int c = 0; c < 65; ++c) {
                denom += std::exp(At4D(keypointLogits, batch, c, gy, gx) - maxLogit);
            }
            if (!(denom > 0.0f)) {
                continue;
            }
            int bestChannel = -1;
            float bestHeat = 0.0f;
            for (int c = 0; c < 64; ++c) {
                const float heat = std::exp(At4D(keypointLogits, batch, c, gy, gx) - maxLogit) / denom;
                if (heat > bestHeat) {
                    bestHeat = heat;
                    bestChannel = c;
                }
            }
            if (bestChannel < 0 || bestHeat <= kThreshold) {
                continue;
            }
            const int offsetY = bestChannel / kXFeatCellSize;
            const int offsetX = bestChannel % kXFeatCellSize;
            const int x = gx * kXFeatCellSize + offsetX;
            const int y = gy * kXFeatCellSize + offsetY;
            const float relX = static_cast<float>(gx);
            const float relY = static_cast<float>(gy);
            const float score = bestHeat * BilinearAt(reliability, batch, 0, relX, relY);
            if (score > 0.0f) {
                candidates.push_back(Candidate{batch, x, y, score});
            }
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate &lhs, const Candidate &rhs) {
        return lhs.score > rhs.score;
    });

    std::vector<Candidate> selected;
    selected.reserve(static_cast<size_t>(std::min(std::max(1, maxPoints), static_cast<int>(candidates.size()))));
    for (const Candidate &candidate : candidates) {
        bool keep = true;
        for (const Candidate &accepted : selected) {
            const float dx = static_cast<float>(candidate.x - accepted.x);
            const float dy = static_cast<float>(candidate.y - accepted.y);
            if (dx * dx + dy * dy < kNmsRadiusSq) {
                keep = false;
                break;
            }
        }
        if (!keep) {
            continue;
        }
        selected.push_back(candidate);
        if (static_cast<int>(selected.size()) >= std::max(1, maxPoints)) {
            break;
        }
    }
    return selected;
}

#if defined(SMART_DRONE_XFEAT_TENSORRT_AVAILABLE)

class TensorRtLogger final : public nvinfer1::ILogger {
  public:
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kWARNING) {
            std::cerr << "[xfeat_trt] " << msg << "\n";
        }
    }
};

size_t TensorRtElementSize(nvinfer1::DataType type)
{
    switch (type) {
    case nvinfer1::DataType::kFLOAT:
        return sizeof(float);
    case nvinfer1::DataType::kHALF:
        return 2;
    case nvinfer1::DataType::kINT8:
        return 1;
    case nvinfer1::DataType::kINT32:
        return 4;
    case nvinfer1::DataType::kBOOL:
        return 1;
    default:
        return 0;
    }
}

int64_t TensorRtVolume(const nvinfer1::Dims &dims)
{
    int64_t volume = 1;
    for (int i = 0; i < dims.nbDims; ++i) {
        if (dims.d[i] <= 0) {
            return 0;
        }
        volume *= dims.d[i];
    }
    return volume;
}

std::vector<int> TensorRtDimsToVector(const nvinfer1::Dims &dims)
{
    std::vector<int> out;
    out.reserve(static_cast<size_t>(dims.nbDims));
    for (int i = 0; i < dims.nbDims; ++i) {
        out.push_back(dims.d[i]);
    }
    return out;
}

class TensorRtXFeatEngine {
  public:
    ~TensorRtXFeatEngine() { Release(); }

    bool Load(const std::filesystem::path &enginePath, std::string *err)
    {
        Release();
        std::ifstream input(enginePath, std::ios::binary);
        if (!input) {
            if (err != nullptr) {
                *err = "failed to open xfeat TensorRT engine: " + enginePath.string();
            }
            return false;
        }
        std::vector<char> bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        if (bytes.empty()) {
            if (err != nullptr) {
                *err = "xfeat TensorRT engine is empty: " + enginePath.string();
            }
            return false;
        }
        initLibNvInferPlugins(&m_logger, "");
        m_runtime.reset(nvinfer1::createInferRuntime(m_logger));
        if (!m_runtime) {
            if (err != nullptr) {
                *err = "failed to create TensorRT runtime";
            }
            return false;
        }
        m_engine.reset(m_runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
        if (!m_engine) {
            if (err != nullptr) {
                *err = "failed to deserialize TensorRT engine: " + enginePath.string();
            }
            return false;
        }
        m_context.reset(m_engine->createExecutionContext());
        if (!m_context) {
            if (err != nullptr) {
                *err = "failed to create TensorRT execution context";
            }
            return false;
        }
        if (cudaStreamCreate(&m_stream) != cudaSuccess) {
            if (err != nullptr) {
                *err = "failed to create TensorRT CUDA stream";
            }
            return false;
        }
        m_inputIndex = FindBinding({"images", "input", "x"}, true);
        m_denseIndex = FindBinding({"dense_features", "features", "output0"}, false);
        m_keypointIndex = FindBinding({"keypoint_logits", "keypoints", "output1"}, false);
        m_reliabilityIndex = FindBinding({"reliability", "heatmap", "output2"}, false);
        if (m_inputIndex < 0 || m_denseIndex < 0 || m_keypointIndex < 0 || m_reliabilityIndex < 0) {
            if (err != nullptr) {
                *err = "TensorRT XFeat engine bindings are missing expected input/output names";
            }
            return false;
        }
        return true;
    }

    bool Forward(const std::vector<float> &batch, int batchSize, int height, int width, TensorBlob &denseFeatures,
                 TensorBlob &keypointLogits, TensorBlob &reliability, std::string *err)
    {
        if (!m_engine || !m_context || batch.empty() || batchSize <= 0 || height <= 0 || width <= 0) {
            return false;
        }
        nvinfer1::Dims inputDims{};
        inputDims.nbDims = 4;
        inputDims.d[0] = batchSize;
        inputDims.d[1] = 1;
        inputDims.d[2] = height;
        inputDims.d[3] = width;
        if (!m_context->setBindingDimensions(m_inputIndex, inputDims)) {
            if (err != nullptr) {
                *err = "TensorRT failed to set XFeat input dimensions";
            }
            return false;
        }
        if (!m_context->allInputDimensionsSpecified()) {
            if (err != nullptr) {
                *err = "TensorRT XFeat input dimensions are incomplete";
            }
            return false;
        }
        if (!EnsureBindingBuffer(m_inputIndex, inputDims, m_engine->getBindingDataType(m_inputIndex), err)) {
            return false;
        }
        const size_t inputBytes = batch.size() * sizeof(float);
        if (cudaMemcpyAsync(m_bindings[static_cast<size_t>(m_inputIndex)], batch.data(), inputBytes,
                            cudaMemcpyHostToDevice, m_stream) != cudaSuccess) {
            if (err != nullptr) {
                *err = "TensorRT failed to copy XFeat input";
            }
            return false;
        }
        for (int index : {m_denseIndex, m_keypointIndex, m_reliabilityIndex}) {
            const nvinfer1::Dims dims = m_context->getBindingDimensions(index);
            if (!EnsureBindingBuffer(index, dims, m_engine->getBindingDataType(index), err)) {
                return false;
            }
        }
        if (!m_context->enqueueV2(m_bindings.data(), m_stream, nullptr)) {
            if (err != nullptr) {
                *err = "TensorRT XFeat enqueue failed";
            }
            return false;
        }
        if (!ReadOutput(m_denseIndex, denseFeatures, err) || !ReadOutput(m_keypointIndex, keypointLogits, err) ||
            !ReadOutput(m_reliabilityIndex, reliability, err)) {
            return false;
        }
        if (cudaStreamSynchronize(m_stream) != cudaSuccess) {
            if (err != nullptr) {
                *err = "TensorRT XFeat stream synchronize failed";
            }
            return false;
        }
        return !denseFeatures.Empty() && !keypointLogits.Empty() && !reliability.Empty();
    }

    bool PreferredInputSize(int &height, int &width) const
    {
        if (!m_engine || m_inputIndex < 0 || m_engine->getNbOptimizationProfiles() <= 0) {
            return false;
        }
        const nvinfer1::Dims optDims =
            m_engine->getProfileDimensions(m_inputIndex, 0, nvinfer1::OptProfileSelector::kOPT);
        if (optDims.nbDims == 4 && optDims.d[2] > 0 && optDims.d[3] > 0) {
            height = optDims.d[2];
            width = optDims.d[3];
            return true;
        }
        return false;
    }

  private:
    struct DestroyRuntime {
        void operator()(nvinfer1::IRuntime *ptr) const
        {
            if (ptr != nullptr) {
                ptr->destroy();
            }
        }
    };
    struct DestroyEngine {
        void operator()(nvinfer1::ICudaEngine *ptr) const
        {
            if (ptr != nullptr) {
                ptr->destroy();
            }
        }
    };
    struct DestroyContext {
        void operator()(nvinfer1::IExecutionContext *ptr) const
        {
            if (ptr != nullptr) {
                ptr->destroy();
            }
        }
    };

    int FindBinding(std::initializer_list<const char *> names, bool input) const
    {
        for (const char *name : names) {
            const int index = m_engine->getBindingIndex(name);
            if (index >= 0 && m_engine->bindingIsInput(index) == input) {
                return index;
            }
        }
        if (input) {
            for (int i = 0; i < m_engine->getNbBindings(); ++i) {
                if (m_engine->bindingIsInput(i)) {
                    return i;
                }
            }
        }
        return -1;
    }

    bool EnsureBindingBuffer(int index, const nvinfer1::Dims &dims, nvinfer1::DataType type, std::string *err)
    {
        if (index < 0) {
            return false;
        }
        const size_t bindingIndex = static_cast<size_t>(index);
        if (m_bindings.size() < static_cast<size_t>(m_engine->getNbBindings())) {
            m_bindings.assign(static_cast<size_t>(m_engine->getNbBindings()), nullptr);
            m_bindingBytes.assign(static_cast<size_t>(m_engine->getNbBindings()), 0);
        }
        const int64_t volume = TensorRtVolume(dims);
        const size_t elementSize = TensorRtElementSize(type);
        if (volume <= 0 || elementSize == 0) {
            if (err != nullptr) {
                *err = "invalid TensorRT binding dimensions for XFeat";
            }
            return false;
        }
        const size_t bytes = static_cast<size_t>(volume) * elementSize;
        if (m_bindingBytes[bindingIndex] >= bytes && m_bindings[bindingIndex] != nullptr) {
            return true;
        }
        if (m_bindings[bindingIndex] != nullptr) {
            cudaFree(m_bindings[bindingIndex]);
            m_bindings[bindingIndex] = nullptr;
            m_bindingBytes[bindingIndex] = 0;
        }
        if (cudaMalloc(&m_bindings[bindingIndex], bytes) != cudaSuccess) {
            if (err != nullptr) {
                *err = "failed to allocate TensorRT XFeat binding buffer";
            }
            return false;
        }
        m_bindingBytes[bindingIndex] = bytes;
        return true;
    }

    bool ReadOutput(int index, TensorBlob &output, std::string *err)
    {
        const nvinfer1::Dims dims = m_context->getBindingDimensions(index);
        if (m_engine->getBindingDataType(index) != nvinfer1::DataType::kFLOAT) {
            if (err != nullptr) {
                *err = "TensorRT XFeat output is not FP32";
            }
            return false;
        }
        output.dims = TensorRtDimsToVector(dims);
        const int64_t volume = TensorRtVolume(dims);
        if (volume <= 0) {
            return false;
        }
        output.data.resize(static_cast<size_t>(volume));
        const size_t bytes = static_cast<size_t>(volume) * sizeof(float);
        if (cudaMemcpyAsync(output.data.data(), m_bindings[static_cast<size_t>(index)], bytes, cudaMemcpyDeviceToHost,
                            m_stream) != cudaSuccess) {
            if (err != nullptr) {
                *err = "TensorRT failed to copy XFeat output";
            }
            return false;
        }
        return true;
    }

    void Release()
    {
        if (m_stream != nullptr) {
            cudaStreamSynchronize(m_stream);
        }
        m_context.reset();
        for (void *ptr : m_bindings) {
            if (ptr != nullptr) {
                cudaFree(ptr);
            }
        }
        m_bindings.clear();
        m_bindingBytes.clear();
        if (m_stream != nullptr) {
            cudaStreamDestroy(m_stream);
            m_stream = nullptr;
        }
        m_engine.reset();
        m_runtime.reset();
    }

    TensorRtLogger m_logger;
    std::unique_ptr<nvinfer1::IRuntime, DestroyRuntime> m_runtime;
    std::unique_ptr<nvinfer1::ICudaEngine, DestroyEngine> m_engine;
    std::unique_ptr<nvinfer1::IExecutionContext, DestroyContext> m_context;
    cudaStream_t m_stream{nullptr};
    std::vector<void *> m_bindings;
    std::vector<size_t> m_bindingBytes;
    int m_inputIndex{-1};
    int m_denseIndex{-1};
    int m_keypointIndex{-1};
    int m_reliabilityIndex{-1};
};

#endif

} // namespace

struct XFeatNativeExtractor::Impl {
#if defined(SMART_DRONE_XFEAT_TENSORRT_AVAILABLE)
    std::unique_ptr<TensorRtXFeatEngine> trtEngine;
    TensorBlob denseFeatures;
    TensorBlob keypointLogits;
    TensorBlob reliability;
    int inputHeight{0};
    int inputWidth{0};

    bool Load(const std::string &repoPath, const std::string &deviceText, std::string *err)
    {
        const std::string device = LowerCopy(deviceText);
        if (!device.empty() && device != "auto" && device != "cuda") {
            if (err != nullptr) {
                *err = "native TensorRT XFeat only supports device=auto|cuda";
            }
            return false;
        }
        const std::filesystem::path enginePath = ResolveTensorRtEnginePath(repoPath);
        if (enginePath.empty()) {
            if (err != nullptr) {
                *err = "xfeat TensorRT engine not found under repo: " + repoPath +
                       " (expected weights/xfeat_trt_fp16.engine)";
            }
            return false;
        }
        auto candidate = std::make_unique<TensorRtXFeatEngine>();
        if (!candidate->Load(enginePath, err)) {
            return false;
        }
        if (!candidate->PreferredInputSize(inputHeight, inputWidth)) {
            inputHeight = 0;
            inputWidth = 0;
        }
        trtEngine = std::move(candidate);
        std::cerr << "[xfeat_trt] loaded engine=" << enginePath.string() << "\n";
        return true;
    }

    bool DetectAndComputeBatch(const std::vector<cv::Mat> &grayImages, int maxPoints,
                               std::vector<XFeatFeatureSet> &outputs, double *inputMs, double *forwardMs,
                               double *postMs, std::string *err)
    {
        outputs.assign(grayImages.size(), XFeatFeatureSet{});
        if (!trtEngine || grayImages.empty()) {
            if (err != nullptr) {
                *err = "xfeat TensorRT backend is not ready";
            }
            return false;
        }
        int targetHeight = inputHeight > 0 ? inputHeight : std::max(32, ((grayImages.front().rows + 31) / 32) * 32);
        int targetWidth = inputWidth > 0 ? inputWidth : std::max(32, ((grayImages.front().cols + 31) / 32) * 32);
        const double ratioH = static_cast<double>(grayImages.front().rows) / static_cast<double>(targetHeight);
        const double ratioW = static_cast<double>(grayImages.front().cols) / static_cast<double>(targetWidth);
        const auto inputStartTp = std::chrono::steady_clock::now();
        const std::vector<float> input = BuildInputBatch(grayImages, targetHeight, targetWidth);
        const auto inputEndTp = std::chrono::steady_clock::now();
        const auto forwardStartTp = inputEndTp;
        if (!trtEngine->Forward(input, static_cast<int>(grayImages.size()), targetHeight, targetWidth, denseFeatures,
                                keypointLogits, reliability, err)) {
            return false;
        }
        const auto forwardEndTp = std::chrono::steady_clock::now();
        if (denseFeatures.Dim(1) != kDescriptorDim || keypointLogits.Dim(1) < 65 || reliability.Dim(1) < 1) {
            if (err != nullptr) {
                *err = "TensorRT XFeat outputs have unexpected shapes";
            }
            return false;
        }
        const auto postStartTp = forwardEndTp;
        for (size_t batchIndex = 0; batchIndex < grayImages.size(); ++batchIndex) {
            std::vector<Candidate> candidates =
                ExtractCandidates(keypointLogits, reliability, static_cast<int>(batchIndex), maxPoints);
            XFeatFeatureSet &output = outputs[batchIndex];
            output.keypoints.reserve(candidates.size());
            if (!candidates.empty()) {
                output.descriptors = cv::Mat(static_cast<int>(candidates.size()), kDescriptorDim, CV_32F);
            }
            for (size_t i = 0; i < candidates.size(); ++i) {
                const Candidate &candidate = candidates[i];
                output.keypoints.emplace_back(static_cast<float>(candidate.x * ratioW),
                                              static_cast<float>(candidate.y * ratioH));
                float *descriptor = output.descriptors.ptr<float>(static_cast<int>(i));
                const float featureX = static_cast<float>(candidate.x) / static_cast<float>(kXFeatCellSize);
                const float featureY = static_cast<float>(candidate.y) / static_cast<float>(kXFeatCellSize);
                for (int c = 0; c < kDescriptorDim; ++c) {
                    descriptor[c] = BilinearAt(denseFeatures, candidate.batch, c, featureX, featureY);
                }
                NormalizeVector(descriptor, kDescriptorDim);
            }
        }
        const auto postEndTp = std::chrono::steady_clock::now();
        if (inputMs != nullptr) {
            *inputMs = DurationMs(inputStartTp, inputEndTp);
        }
        if (forwardMs != nullptr) {
            *forwardMs = DurationMs(forwardStartTp, forwardEndTp);
        }
        if (postMs != nullptr) {
            *postMs = DurationMs(postStartTp, postEndTp);
        }
        return true;
    }
#endif
};

XFeatNativeExtractor::~XFeatNativeExtractor() = default;

bool XFeatNativeExtractor::PrepareGrayImage(const cv::Mat &gray, cv::Mat &gray8, std::string *err)
{
    if (gray.type() == CV_8UC1 && gray.isContinuous()) {
        gray8 = gray;
        return true;
    }
    if (gray.channels() == 1) {
        gray.convertTo(gray8, CV_8UC1);
    } else {
        cv::cvtColor(gray, gray8, cv::COLOR_BGR2GRAY);
    }
    if (!gray8.isContinuous()) {
        gray8 = gray8.clone();
    }
    if (gray8.empty()) {
        if (err != nullptr) {
            *err = "xfeat native gray image preparation failed";
        }
        return false;
    }
    return true;
}

bool XFeatNativeExtractor::Start(const std::string &repoPath, const std::string &device, int topK, int maxPoints,
                                 std::string *err)
{
    m_repoPath = repoPath;
    m_device = device;
    m_topK = topK;
    m_maxPoints = maxPoints;
    m_lastStats = Stats{};
#if defined(SMART_DRONE_XFEAT_TENSORRT_AVAILABLE)
    m_impl = std::make_unique<Impl>();
    if (!m_impl->Load(repoPath, device, err)) {
        m_impl.reset();
        m_running = false;
        return false;
    }
    m_running = true;
    return true;
#else
    m_running = false;
    if (err != nullptr) {
        *err = "native TensorRT XFeat backend is unavailable in this target";
    }
    return false;
#endif
}

void XFeatNativeExtractor::Stop()
{
    m_running = false;
    m_lastStats = Stats{};
    m_impl.reset();
}

bool XFeatNativeExtractor::Running() const { return m_running; }

XFeatNativeExtractor::Stats XFeatNativeExtractor::LastStats() const { return m_lastStats; }

bool XFeatNativeExtractor::Detect(const cv::Mat &gray, std::vector<cv::Point2f> &outPoints, std::string *err)
{
    XFeatFeatureSet features;
    if (!DetectAndCompute(gray, features, err)) {
        return false;
    }
    outPoints = std::move(features.keypoints);
    return true;
}

bool XFeatNativeExtractor::DetectAndCompute(const cv::Mat &gray, XFeatFeatureSet &outFeatures, std::string *err)
{
    outFeatures.keypoints.clear();
    outFeatures.descriptors.release();
    m_lastStats = Stats{};
#if defined(SMART_DRONE_XFEAT_TENSORRT_AVAILABLE)
    if (!m_running || !m_impl) {
        if (err != nullptr) {
            *err = "xfeat TensorRT backend not running";
        }
        return false;
    }
    const auto totalStartTp = std::chrono::steady_clock::now();
    cv::Mat gray8;
    const auto prepareStartTp = totalStartTp;
    if (!PrepareGrayImage(gray, gray8, err)) {
        return false;
    }
    const auto prepareEndTp = std::chrono::steady_clock::now();
    std::vector<XFeatFeatureSet> outputs;
    const auto inferStartTp = prepareEndTp;
    double inputMs = 0.0;
    double forwardMs = 0.0;
    double postMs = 0.0;
    if (!m_impl->DetectAndComputeBatch({gray8}, m_maxPoints, outputs, &inputMs, &forwardMs, &postMs, err) ||
        outputs.empty()) {
        return false;
    }
    const auto inferEndTp = std::chrono::steady_clock::now();
    outFeatures = std::move(outputs.front());
    m_lastStats.prepareMs = DurationMs(prepareStartTp, prepareEndTp);
    m_lastStats.inputMs = inputMs;
    m_lastStats.forwardMs = forwardMs;
    m_lastStats.postMs = postMs;
    m_lastStats.inferMs = DurationMs(inferStartTp, inferEndTp);
    m_lastStats.totalMs = DurationMs(totalStartTp, inferEndTp);
    m_lastStats.imageCount = 1;
    m_lastStats.payloadBytes = static_cast<uint32_t>(gray8.total());
    return true;
#else
    if (err != nullptr) {
        *err = "native TensorRT XFeat backend is compiled out";
    }
    (void)gray;
    return false;
#endif
}

bool XFeatNativeExtractor::DetectAndComputeStereo(const cv::Mat &leftGray, const cv::Mat &rightGray,
                                                  XFeatFeatureSet &leftFeatures, XFeatFeatureSet &rightFeatures,
                                                  std::string *err)
{
    leftFeatures.keypoints.clear();
    leftFeatures.descriptors.release();
    rightFeatures.keypoints.clear();
    rightFeatures.descriptors.release();
    m_lastStats = Stats{};
#if defined(SMART_DRONE_XFEAT_TENSORRT_AVAILABLE)
    if (!m_running || !m_impl) {
        if (err != nullptr) {
            *err = "xfeat TensorRT backend not running";
        }
        return false;
    }
    const auto totalStartTp = std::chrono::steady_clock::now();
    cv::Mat leftGray8;
    cv::Mat rightGray8;
    const auto prepareStartTp = totalStartTp;
    if (!PrepareGrayImage(leftGray, leftGray8, err) || !PrepareGrayImage(rightGray, rightGray8, err)) {
        return false;
    }
    const auto prepareEndTp = std::chrono::steady_clock::now();
    std::vector<XFeatFeatureSet> outputs;
    const auto inferStartTp = prepareEndTp;
    double inputMs = 0.0;
    double forwardMs = 0.0;
    double postMs = 0.0;
    if (!m_impl->DetectAndComputeBatch({leftGray8, rightGray8}, m_maxPoints, outputs, &inputMs, &forwardMs, &postMs,
                                       err) ||
        outputs.size() != 2) {
        return false;
    }
    const auto inferEndTp = std::chrono::steady_clock::now();
    leftFeatures = std::move(outputs[0]);
    rightFeatures = std::move(outputs[1]);
    m_lastStats.prepareMs = DurationMs(prepareStartTp, prepareEndTp);
    m_lastStats.inputMs = inputMs;
    m_lastStats.forwardMs = forwardMs;
    m_lastStats.postMs = postMs;
    m_lastStats.inferMs = DurationMs(inferStartTp, inferEndTp);
    m_lastStats.totalMs = DurationMs(totalStartTp, inferEndTp);
    m_lastStats.imageCount = 2;
    m_lastStats.payloadBytes = static_cast<uint32_t>(leftGray8.total() + rightGray8.total());
    std::cerr << "[xfeat_trt_perf] batch=2 input_ms=" << inputMs << " gpu_forward_ms=" << forwardMs
              << " cpu_post_ms=" << postMs << " total_ms=" << m_lastStats.totalMs
              << " left_pts=" << leftFeatures.keypoints.size() << " right_pts=" << rightFeatures.keypoints.size()
              << "\n";
    return true;
#else
    if (err != nullptr) {
        *err = "native TensorRT XFeat backend is compiled out";
    }
    (void)leftGray;
    (void)rightGray;
    return false;
#endif
}

} // namespace smartdrone::adapters::slam
