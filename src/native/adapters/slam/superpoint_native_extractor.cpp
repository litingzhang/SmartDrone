#include "adapters/slam/superpoint_native_extractor.h"

#include "adapters/slam/superpoint_lightglue_frontend_client.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <utility>

#include <opencv2/imgproc.hpp>

#if defined(SMART_DRONE_SUPERPOINT_TENSORRT_AVAILABLE)
#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_runtime_api.h>
#endif

namespace smartdrone::adapters::slam {

SuperPointNativeExtractor::SuperPointNativeExtractor() = default;

namespace {

constexpr int kSuperPointCellSize = 8;
constexpr int kSuperPointDescriptorDim = 256;
constexpr float kSuperPointThreshold = 0.0005f;
constexpr int kSuperPointBorder = 4;
constexpr int kSuperPointNmsRadius = 4;
constexpr float kStereoMaxYDiffPx = 2.0f;
constexpr float kStereoRatio = 0.92f;

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

int EnvInt(const char *name, int fallback)
{
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    return std::max(1, std::atoi(value));
}

float EnvFloat(const char *name, float fallback)
{
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    char *end = nullptr;
    const float parsed = std::strtof(value, &end);
    return end != value && std::isfinite(parsed) ? parsed : fallback;
}

std::filesystem::path ResolveSuperPointEnginePath(const std::string &repoPath, int widthHint, int heightHint)
{
    const char *envEngine = std::getenv("SMART_DRONE_SUPERPOINT_TRT_ENGINE");
    if (envEngine != nullptr && envEngine[0] != '\0' && std::filesystem::exists(envEngine)) {
        return std::filesystem::path(envEngine);
    }

    const std::filesystem::path repo(repoPath);
    if (!repo.empty()) {
        std::vector<std::string> names;
        if (widthHint > 0 && heightHint > 0) {
            names.push_back("superpoint_dense_" + std::to_string(widthHint) + "x" + std::to_string(heightHint) +
                            "_fp16.engine");
            names.push_back("superpoint_dense_" + std::to_string(widthHint) + "x" + std::to_string(heightHint) +
                            "_fp32.engine");
        }
        names.push_back("superpoint_dense_640x480_fp16.engine");
        names.push_back("superpoint_dense_640x480_fp32.engine");
        names.push_back("superpoint_dense_640x409_fp16.engine");
        names.push_back("superpoint_dense_640x409_fp32.engine");
        names.push_back("superpoint.engine");
        for (const std::string &name : names) {
            const std::filesystem::path candidate = repo / "weights" / name;
            if (std::filesystem::exists(candidate)) {
                return candidate;
            }
        }
    }
    return {};
}

std::filesystem::path ResolveLightGlueEnginePath(const std::string &repoPath, int maxPoints)
{
    const char *envEngine = std::getenv("SMART_DRONE_LIGHTGLUE_TRT_ENGINE");
    if (envEngine != nullptr && envEngine[0] != '\0' && std::filesystem::exists(envEngine)) {
        return std::filesystem::path(envEngine);
    }

    const std::filesystem::path repo(repoPath);
    if (!repo.empty()) {
        const int n = std::max(1, maxPoints);
        const std::vector<std::string> names = {
            "lightglue_superpoint_" + std::to_string(n) + "_fp16.engine",
            "lightglue_superpoint_" + std::to_string(n) + "_fp32.engine",
            "lightglue_superpoint_fp16.engine",
            "lightglue_superpoint_fp32.engine",
            "lightglue.engine",
        };
        for (const std::string &name : names) {
            const std::filesystem::path candidate = repo / "weights" / name;
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

float HalfToFloat(uint16_t value)
{
    const uint32_t sign = (static_cast<uint32_t>(value & 0x8000u)) << 16u;
    uint32_t exponent = (value >> 10u) & 0x1fu;
    uint32_t mantissa = value & 0x03ffu;
    uint32_t bits = 0;
    if (exponent == 0) {
        if (mantissa == 0) {
            bits = sign;
        } else {
            exponent = 1;
            while ((mantissa & 0x0400u) == 0) {
                mantissa <<= 1u;
                --exponent;
            }
            mantissa &= 0x03ffu;
            bits = sign | ((exponent + 112u) << 23u) | (mantissa << 13u);
        }
    } else if (exponent == 31) {
        bits = sign | 0x7f800000u | (mantissa << 13u);
    } else {
        bits = sign | ((exponent + 112u) << 23u) | (mantissa << 13u);
    }
    float out = 0.0f;
    std::memcpy(&out, &bits, sizeof(out));
    return out;
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

std::vector<float> BuildHeatmap(const TensorBlob &detector, int batch)
{
    const int gridHeight = detector.Dim(2);
    const int gridWidth = detector.Dim(3);
    std::vector<float> heatmap(static_cast<size_t>(gridHeight * kSuperPointCellSize) *
                               static_cast<size_t>(gridWidth * kSuperPointCellSize));
    const int fullWidth = gridWidth * kSuperPointCellSize;
    for (int gy = 0; gy < gridHeight; ++gy) {
        for (int gx = 0; gx < gridWidth; ++gx) {
            float maxLogit = At4D(detector, batch, 0, gy, gx);
            for (int c = 1; c < 65; ++c) {
                maxLogit = std::max(maxLogit, At4D(detector, batch, c, gy, gx));
            }
            float denom = 0.0f;
            for (int c = 0; c < 65; ++c) {
                denom += std::exp(At4D(detector, batch, c, gy, gx) - maxLogit);
            }
            if (!(denom > 0.0f)) {
                continue;
            }
            for (int c = 0; c < 64; ++c) {
                const int y = gy * kSuperPointCellSize + c / kSuperPointCellSize;
                const int x = gx * kSuperPointCellSize + c % kSuperPointCellSize;
                heatmap[static_cast<size_t>(y) * fullWidth + x] =
                    std::exp(At4D(detector, batch, c, gy, gx) - maxLogit) / denom;
            }
        }
    }
    return heatmap;
}

std::vector<Candidate> ExtractCandidates(const TensorBlob &detector, int batch, int targetWidth, int targetHeight,
                                         int maxPoints)
{
    std::vector<float> heatmap = BuildHeatmap(detector, batch);
    const int stride = targetWidth;
    std::vector<Candidate> candidates;
    for (int y = kSuperPointBorder; y < targetHeight - kSuperPointBorder; ++y) {
        for (int x = kSuperPointBorder; x < targetWidth - kSuperPointBorder; ++x) {
            const float score = heatmap[static_cast<size_t>(y) * stride + x];
            if (score <= kSuperPointThreshold) {
                continue;
            }
            bool isMax = true;
            for (int yy = std::max(kSuperPointBorder, y - kSuperPointNmsRadius);
                 yy <= std::min(targetHeight - kSuperPointBorder - 1, y + kSuperPointNmsRadius) && isMax; ++yy) {
                for (int xx = std::max(kSuperPointBorder, x - kSuperPointNmsRadius);
                     xx <= std::min(targetWidth - kSuperPointBorder - 1, x + kSuperPointNmsRadius); ++xx) {
                    if ((xx != x || yy != y) && heatmap[static_cast<size_t>(yy) * stride + xx] > score) {
                        isMax = false;
                        break;
                    }
                }
            }
            if (isMax) {
                candidates.push_back(Candidate{x, y, score});
            }
        }
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate &lhs, const Candidate &rhs) { return lhs.score > rhs.score; });
    if (static_cast<int>(candidates.size()) > std::max(1, maxPoints)) {
        candidates.resize(static_cast<size_t>(std::max(1, maxPoints)));
    }
    return candidates;
}

void MatchStereoPairs(const SuperPointFeatureSet &leftRaw, const SuperPointFeatureSet &rightRaw, int maxPoints,
                      SuperPointFeatureSet &leftOut, SuperPointFeatureSet &rightOut)
{
    leftOut = SuperPointFeatureSet{};
    rightOut = SuperPointFeatureSet{};
    if (leftRaw.descriptors.empty() || rightRaw.descriptors.empty() || leftRaw.descriptors.type() != CV_32F ||
        rightRaw.descriptors.type() != CV_32F || leftRaw.descriptors.cols != rightRaw.descriptors.cols) {
        return;
    }

    const int leftCount = std::min(static_cast<int>(leftRaw.keypoints.size()), leftRaw.descriptors.rows);
    const int rightCount = std::min(static_cast<int>(rightRaw.keypoints.size()), rightRaw.descriptors.rows);
    const int descriptorDim = leftRaw.descriptors.cols;
    if (leftCount <= 0 || rightCount <= 0 || descriptorDim <= 0) {
        return;
    }

    std::vector<int> bestRightForLeft(static_cast<size_t>(leftCount), -1);
    std::vector<float> bestDistForLeft(static_cast<size_t>(leftCount), std::numeric_limits<float>::infinity());
    std::vector<float> secondDistForLeft(static_cast<size_t>(leftCount), std::numeric_limits<float>::infinity());
    std::vector<int> bestLeftForRight(static_cast<size_t>(rightCount), -1);
    std::vector<float> bestDistForRight(static_cast<size_t>(rightCount), std::numeric_limits<float>::infinity());

    for (int li = 0; li < leftCount; ++li) {
        const float *ld = leftRaw.descriptors.ptr<float>(li);
        for (int ri = 0; ri < rightCount; ++ri) {
            const cv::Point2f &lp = leftRaw.keypoints[static_cast<size_t>(li)];
            const cv::Point2f &rp = rightRaw.keypoints[static_cast<size_t>(ri)];
            if (std::abs(lp.y - rp.y) > kStereoMaxYDiffPx || lp.x <= rp.x) {
                continue;
            }
            const float *rd = rightRaw.descriptors.ptr<float>(ri);
            float dist = 0.0f;
            for (int d = 0; d < descriptorDim; ++d) {
                const float delta = ld[d] - rd[d];
                dist += delta * delta;
            }
            if (!std::isfinite(dist)) {
                continue;
            }
            if (dist < bestDistForLeft[static_cast<size_t>(li)]) {
                secondDistForLeft[static_cast<size_t>(li)] = bestDistForLeft[static_cast<size_t>(li)];
                bestDistForLeft[static_cast<size_t>(li)] = dist;
                bestRightForLeft[static_cast<size_t>(li)] = ri;
            } else if (dist < secondDistForLeft[static_cast<size_t>(li)]) {
                secondDistForLeft[static_cast<size_t>(li)] = dist;
            }
            if (dist < bestDistForRight[static_cast<size_t>(ri)]) {
                bestDistForRight[static_cast<size_t>(ri)] = dist;
                bestLeftForRight[static_cast<size_t>(ri)] = li;
            }
        }
    }

    struct Pair {
        int left{0};
        int right{0};
        float distance{0.0f};
        float disparity{0.0f};
    };
    std::vector<Pair> pairs;
    for (int li = 0; li < leftCount; ++li) {
        const int ri = bestRightForLeft[static_cast<size_t>(li)];
        if (ri < 0 || bestLeftForRight[static_cast<size_t>(ri)] != li) {
            continue;
        }
        const float bestDist = bestDistForLeft[static_cast<size_t>(li)];
        const float secondDist = secondDistForLeft[static_cast<size_t>(li)];
        if (std::isfinite(secondDist) && bestDist >= (kStereoRatio * kStereoRatio) * secondDist) {
            continue;
        }
        const cv::Point2f &lp = leftRaw.keypoints[static_cast<size_t>(li)];
        const cv::Point2f &rp = rightRaw.keypoints[static_cast<size_t>(ri)];
        const float disparity = lp.x - rp.x;
        pairs.push_back(Pair{li, ri, bestDist, disparity});
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair &lhs, const Pair &rhs) {
        if (std::abs(lhs.distance - rhs.distance) > 1.0e-6f) {
            return lhs.distance < rhs.distance;
        }
        return lhs.disparity > rhs.disparity;
    });
    if (static_cast<int>(pairs.size()) > std::max(1, maxPoints)) {
        pairs.resize(static_cast<size_t>(std::max(1, maxPoints)));
    }

    leftOut.keypoints.reserve(pairs.size());
    rightOut.keypoints.reserve(pairs.size());
    leftOut.descriptors = cv::Mat(static_cast<int>(pairs.size()), leftRaw.descriptors.cols, CV_32F);
    rightOut.descriptors = cv::Mat(static_cast<int>(pairs.size()), rightRaw.descriptors.cols, CV_32F);
    for (size_t i = 0; i < pairs.size(); ++i) {
        leftOut.keypoints.push_back(leftRaw.keypoints[static_cast<size_t>(pairs[i].left)]);
        rightOut.keypoints.push_back(rightRaw.keypoints[static_cast<size_t>(pairs[i].right)]);
        leftRaw.descriptors.row(pairs[i].left).copyTo(leftOut.descriptors.row(static_cast<int>(i)));
        rightRaw.descriptors.row(pairs[i].right).copyTo(rightOut.descriptors.row(static_cast<int>(i)));
    }
}

#if defined(SMART_DRONE_SUPERPOINT_TENSORRT_AVAILABLE)

class TensorRtLogger final : public nvinfer1::ILogger {
  public:
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kWARNING) {
            std::cerr << "[superpoint_trt] " << msg << "\n";
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

class TensorRtSuperPointEngine {
  public:
    ~TensorRtSuperPointEngine() { Release(); }

    bool Load(const std::filesystem::path &enginePath, std::string *err)
    {
        Release();
        std::ifstream input(enginePath, std::ios::binary);
        if (!input) {
            if (err != nullptr) {
                *err = "failed to open SuperPoint TensorRT engine: " + enginePath.string();
            }
            return false;
        }
        std::vector<char> bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        if (bytes.empty()) {
            if (err != nullptr) {
                *err = "SuperPoint TensorRT engine is empty: " + enginePath.string();
            }
            return false;
        }
        initLibNvInferPlugins(&m_logger, "");
        m_runtime.reset(nvinfer1::createInferRuntime(m_logger));
        if (!m_runtime) {
            if (err != nullptr) {
                *err = "failed to create SuperPoint TensorRT runtime";
            }
            return false;
        }
        m_engine.reset(m_runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
        if (!m_engine) {
            if (err != nullptr) {
                *err = "failed to deserialize SuperPoint TensorRT engine: " + enginePath.string();
            }
            return false;
        }
        m_context.reset(m_engine->createExecutionContext());
        if (!m_context) {
            if (err != nullptr) {
                *err = "failed to create SuperPoint TensorRT execution context";
            }
            return false;
        }
        if (cudaStreamCreate(&m_stream) != cudaSuccess) {
            if (err != nullptr) {
                *err = "failed to create SuperPoint TensorRT CUDA stream";
            }
            return false;
        }
        m_inputIndex = FindBinding({"image", "images", "input"}, true);
        m_detectorIndex = FindBinding({"detector_logits", "scores", "output0"}, false);
        m_descriptorIndex = FindBinding({"dense_descriptors", "descriptors", "output1"}, false);
        if (m_inputIndex < 0 || m_detectorIndex < 0 || m_descriptorIndex < 0) {
            if (err != nullptr) {
                *err = "SuperPoint TensorRT engine bindings are missing expected input/output names";
            }
            return false;
        }
        return true;
    }

    bool Forward(const std::vector<float> &batch, int batchSize, int height, int width, TensorBlob &detector,
                 TensorBlob &descriptors, std::string *err)
    {
        nvinfer1::Dims inputDims{};
        inputDims.nbDims = 4;
        inputDims.d[0] = batchSize;
        inputDims.d[1] = 1;
        inputDims.d[2] = height;
        inputDims.d[3] = width;
        if (!m_context->setBindingDimensions(m_inputIndex, inputDims)) {
            if (err != nullptr) {
                *err = "TensorRT failed to set SuperPoint input dimensions";
            }
            return false;
        }
        if (!m_context->allInputDimensionsSpecified()) {
            if (err != nullptr) {
                *err = "TensorRT SuperPoint input dimensions are incomplete";
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
                *err = "TensorRT failed to copy SuperPoint input";
            }
            return false;
        }
        for (int index : {m_detectorIndex, m_descriptorIndex}) {
            const nvinfer1::Dims dims = m_context->getBindingDimensions(index);
            if (!EnsureBindingBuffer(index, dims, m_engine->getBindingDataType(index), err)) {
                return false;
            }
        }
        if (!m_context->enqueueV2(m_bindings.data(), m_stream, nullptr)) {
            if (err != nullptr) {
                *err = "TensorRT SuperPoint enqueue failed";
            }
            return false;
        }
        if (!ReadOutput(m_detectorIndex, detector, err) || !ReadOutput(m_descriptorIndex, descriptors, err)) {
            return false;
        }
        if (cudaStreamSynchronize(m_stream) != cudaSuccess) {
            if (err != nullptr) {
                *err = "TensorRT SuperPoint stream synchronize failed";
            }
            return false;
        }
        return !detector.Empty() && !descriptors.Empty();
    }

    bool PreferredInputSize(int &height, int &width) const
    {
        if (!m_engine || m_inputIndex < 0) {
            return false;
        }
        nvinfer1::Dims dims = m_engine->getBindingDimensions(m_inputIndex);
        if (m_engine->getNbOptimizationProfiles() > 0) {
            dims = m_engine->getProfileDimensions(m_inputIndex, 0, nvinfer1::OptProfileSelector::kOPT);
        }
        if (dims.nbDims == 4 && dims.d[2] > 0 && dims.d[3] > 0) {
            height = dims.d[2];
            width = dims.d[3];
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
            return -1;
        }
        for (int i = 0; i < m_engine->getNbBindings(); ++i) {
            if (!m_engine->bindingIsInput(i)) {
                return i;
            }
        }
        return -1;
    }

    bool EnsureBindingBuffer(int index, const nvinfer1::Dims &dims, nvinfer1::DataType type, std::string *err)
    {
        const size_t bindingIndex = static_cast<size_t>(index);
        if (m_bindings.size() < static_cast<size_t>(m_engine->getNbBindings())) {
            m_bindings.assign(static_cast<size_t>(m_engine->getNbBindings()), nullptr);
            m_bindingBytes.assign(static_cast<size_t>(m_engine->getNbBindings()), 0);
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
        if (m_bindingBytes[bindingIndex] >= bytes && m_bindings[bindingIndex] != nullptr) {
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

    bool ReadOutput(int index, TensorBlob &output, std::string *err)
    {
        const nvinfer1::Dims dims = m_context->getBindingDimensions(index);
        output.dims = TensorRtDimsToVector(dims);
        const int64_t volume = TensorRtVolume(dims);
        if (volume <= 0) {
            return false;
        }
        const nvinfer1::DataType dtype = m_engine->getBindingDataType(index);
        if (dtype == nvinfer1::DataType::kFLOAT) {
            output.data.resize(static_cast<size_t>(volume));
            const size_t bytes = static_cast<size_t>(volume) * sizeof(float);
            if (cudaMemcpyAsync(output.data.data(), m_bindings[static_cast<size_t>(index)], bytes,
                                cudaMemcpyDeviceToHost, m_stream) != cudaSuccess) {
                if (err != nullptr) {
                    *err = "TensorRT failed to copy SuperPoint FP32 output";
                }
                return false;
            }
            return true;
        }
        if (dtype == nvinfer1::DataType::kHALF) {
            std::vector<uint16_t> halfData(static_cast<size_t>(volume));
            const size_t bytes = static_cast<size_t>(volume) * sizeof(uint16_t);
            if (cudaMemcpyAsync(halfData.data(), m_bindings[static_cast<size_t>(index)], bytes, cudaMemcpyDeviceToHost,
                                m_stream) != cudaSuccess) {
                if (err != nullptr) {
                    *err = "TensorRT failed to copy SuperPoint FP16 output";
                }
                return false;
            }
            output.data.resize(static_cast<size_t>(volume));
            for (size_t i = 0; i < halfData.size(); ++i) {
                output.data[i] = HalfToFloat(halfData[i]);
            }
            return true;
        }
        if (err != nullptr) {
            *err = "TensorRT SuperPoint output has unsupported data type";
        }
        return false;
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
    int m_detectorIndex{-1};
    int m_descriptorIndex{-1};
};

class TensorRtLightGlueEngine {
  public:
    ~TensorRtLightGlueEngine() { Release(); }

    bool Load(const std::filesystem::path &enginePath, std::string *err)
    {
        Release();
        std::ifstream input(enginePath, std::ios::binary);
        if (!input) {
            if (err != nullptr) {
                *err = "failed to open LightGlue TensorRT engine: " + enginePath.string();
            }
            return false;
        }
        std::vector<char> bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        if (bytes.empty()) {
            if (err != nullptr) {
                *err = "LightGlue TensorRT engine is empty: " + enginePath.string();
            }
            return false;
        }
        initLibNvInferPlugins(&m_logger, "");
        m_runtime.reset(nvinfer1::createInferRuntime(m_logger));
        if (!m_runtime) {
            if (err != nullptr) {
                *err = "failed to create LightGlue TensorRT runtime";
            }
            return false;
        }
        m_engine.reset(m_runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
        if (!m_engine) {
            if (err != nullptr) {
                *err = "failed to deserialize LightGlue TensorRT engine: " + enginePath.string();
            }
            return false;
        }
        m_context.reset(m_engine->createExecutionContext());
        if (!m_context) {
            if (err != nullptr) {
                *err = "failed to create LightGlue TensorRT execution context";
            }
            return false;
        }
        if (cudaStreamCreate(&m_stream) != cudaSuccess) {
            if (err != nullptr) {
                *err = "failed to create LightGlue TensorRT CUDA stream";
            }
            return false;
        }
        m_kpts0Index = FindBinding({"keypoints0"}, true);
        m_kpts1Index = FindBinding({"keypoints1"}, true);
        m_desc0Index = FindBinding({"descriptors0"}, true);
        m_desc1Index = FindBinding({"descriptors1"}, true);
        m_size0Index = FindBinding({"image_size0"}, true);
        m_size1Index = FindBinding({"image_size1"}, true);
        m_scoresIndex = FindBinding({"assignment_scores", "scores", "output0"}, false);
        if (m_kpts0Index < 0 || m_kpts1Index < 0 || m_desc0Index < 0 || m_desc1Index < 0 ||
            m_size0Index < 0 || m_size1Index < 0 || m_scoresIndex < 0) {
            if (err != nullptr) {
                *err = "LightGlue TensorRT engine bindings are missing expected input/output names";
            }
            return false;
        }
        return true;
    }

    bool Forward(const std::vector<float> &keypoints0, const std::vector<float> &keypoints1,
                 const std::vector<float> &descriptors0, const std::vector<float> &descriptors1,
                 const std::array<float, 2> &imageSize0, const std::array<float, 2> &imageSize1, int pointCount,
                 TensorBlob &scores, std::string *err)
    {
        if (pointCount <= 0) {
            return false;
        }
        if (!SetInput(m_kpts0Index, {1, pointCount, 2}, keypoints0, err) ||
            !SetInput(m_kpts1Index, {1, pointCount, 2}, keypoints1, err) ||
            !SetInput(m_desc0Index, {1, pointCount, kSuperPointDescriptorDim}, descriptors0, err) ||
            !SetInput(m_desc1Index, {1, pointCount, kSuperPointDescriptorDim}, descriptors1, err)) {
            return false;
        }
        const std::vector<float> size0{imageSize0[0], imageSize0[1]};
        const std::vector<float> size1{imageSize1[0], imageSize1[1]};
        if (!SetInput(m_size0Index, {1, 2}, size0, err) || !SetInput(m_size1Index, {1, 2}, size1, err)) {
            return false;
        }
        if (!m_context->allInputDimensionsSpecified()) {
            if (err != nullptr) {
                *err = "TensorRT LightGlue input dimensions are incomplete";
            }
            return false;
        }
        for (int index : {m_scoresIndex}) {
            const nvinfer1::Dims dims = m_context->getBindingDimensions(index);
            if (!EnsureBindingBuffer(index, dims, m_engine->getBindingDataType(index), err)) {
                return false;
            }
        }
        if (!m_context->enqueueV2(m_bindings.data(), m_stream, nullptr)) {
            if (err != nullptr) {
                *err = "TensorRT LightGlue enqueue failed";
            }
            return false;
        }
        if (!ReadOutput(m_scoresIndex, scores, err)) {
            return false;
        }
        if (cudaStreamSynchronize(m_stream) != cudaSuccess) {
            if (err != nullptr) {
                *err = "TensorRT LightGlue stream synchronize failed";
            }
            return false;
        }
        return !scores.Empty();
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
        return -1;
    }

    static nvinfer1::Dims MakeDims(const std::vector<int> &dims)
    {
        nvinfer1::Dims out{};
        out.nbDims = static_cast<int>(std::min<size_t>(dims.size(), 8));
        for (int i = 0; i < out.nbDims; ++i) {
            out.d[i] = dims[static_cast<size_t>(i)];
        }
        return out;
    }

    bool SetInput(int index, const std::vector<int> &dims, const std::vector<float> &data, std::string *err)
    {
        const nvinfer1::Dims trtDims = MakeDims(dims);
        if (!m_context->setBindingDimensions(index, trtDims)) {
            if (err != nullptr) {
                *err = "TensorRT failed to set LightGlue input dimensions";
            }
            return false;
        }
        if (!EnsureBindingBuffer(index, trtDims, m_engine->getBindingDataType(index), err)) {
            return false;
        }
        const size_t bytes = data.size() * sizeof(float);
        if (cudaMemcpyAsync(m_bindings[static_cast<size_t>(index)], data.data(), bytes, cudaMemcpyHostToDevice,
                            m_stream) != cudaSuccess) {
            if (err != nullptr) {
                *err = "TensorRT failed to copy LightGlue input";
            }
            return false;
        }
        return true;
    }

    bool EnsureBindingBuffer(int index, const nvinfer1::Dims &dims, nvinfer1::DataType type, std::string *err)
    {
        const size_t bindingIndex = static_cast<size_t>(index);
        if (m_bindings.size() < static_cast<size_t>(m_engine->getNbBindings())) {
            m_bindings.assign(static_cast<size_t>(m_engine->getNbBindings()), nullptr);
            m_bindingBytes.assign(static_cast<size_t>(m_engine->getNbBindings()), 0);
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
        if (m_bindingBytes[bindingIndex] >= bytes && m_bindings[bindingIndex] != nullptr) {
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

    bool ReadOutput(int index, TensorBlob &output, std::string *err)
    {
        const nvinfer1::Dims dims = m_context->getBindingDimensions(index);
        output.dims = TensorRtDimsToVector(dims);
        const int64_t volume = TensorRtVolume(dims);
        if (volume <= 0) {
            return false;
        }
        const nvinfer1::DataType dtype = m_engine->getBindingDataType(index);
        if (dtype == nvinfer1::DataType::kFLOAT) {
            output.data.resize(static_cast<size_t>(volume));
            const size_t bytes = static_cast<size_t>(volume) * sizeof(float);
            if (cudaMemcpyAsync(output.data.data(), m_bindings[static_cast<size_t>(index)], bytes,
                                cudaMemcpyDeviceToHost, m_stream) != cudaSuccess) {
                if (err != nullptr) {
                    *err = "TensorRT failed to copy LightGlue FP32 output";
                }
                return false;
            }
            return true;
        }
        if (dtype == nvinfer1::DataType::kHALF) {
            std::vector<uint16_t> halfData(static_cast<size_t>(volume));
            const size_t bytes = static_cast<size_t>(volume) * sizeof(uint16_t);
            if (cudaMemcpyAsync(halfData.data(), m_bindings[static_cast<size_t>(index)], bytes, cudaMemcpyDeviceToHost,
                                m_stream) != cudaSuccess) {
                if (err != nullptr) {
                    *err = "TensorRT failed to copy LightGlue FP16 output";
                }
                return false;
            }
            output.data.resize(static_cast<size_t>(volume));
            for (size_t i = 0; i < halfData.size(); ++i) {
                output.data[i] = HalfToFloat(halfData[i]);
            }
            return true;
        }
        if (dtype == nvinfer1::DataType::kINT32) {
            std::vector<int32_t> intData(static_cast<size_t>(volume));
            const size_t bytes = static_cast<size_t>(volume) * sizeof(int32_t);
            if (cudaMemcpyAsync(intData.data(), m_bindings[static_cast<size_t>(index)], bytes,
                                cudaMemcpyDeviceToHost, m_stream) != cudaSuccess) {
                if (err != nullptr) {
                    *err = "TensorRT failed to copy LightGlue INT32 output";
                }
                return false;
            }
            output.data.resize(static_cast<size_t>(volume));
            for (size_t i = 0; i < intData.size(); ++i) {
                output.data[i] = static_cast<float>(intData[i]);
            }
            return true;
        }
        if (err != nullptr) {
            *err = "TensorRT LightGlue output has unsupported data type";
        }
        return false;
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
    int m_kpts0Index{-1};
    int m_kpts1Index{-1};
    int m_desc0Index{-1};
    int m_desc1Index{-1};
    int m_size0Index{-1};
    int m_size1Index{-1};
    int m_scoresIndex{-1};
};

#endif

} // namespace

struct SuperPointNativeExtractor::Impl {
#if defined(SMART_DRONE_SUPERPOINT_TENSORRT_AVAILABLE)
    std::unique_ptr<TensorRtSuperPointEngine> trtEngine;
    std::unique_ptr<TensorRtLightGlueEngine> lightGlueEngine;
    TensorBlob detector;
    TensorBlob descriptors;
    TensorBlob lightGlueScores;
    int inputHeight{0};
    int inputWidth{0};
    int lightGluePointCount{0};
    float lightGlueMinScore{0.02f};
    float lightGlueMaxYDiffPx{1.5f};
    float lightGlueMinDisparityPx{0.8f};

    bool Load(const std::string &repoPath, const std::string &deviceText, std::string *err)
    {
        const std::string device = LowerCopy(deviceText);
        if (!device.empty() && device != "auto" && device != "cuda") {
            if (err != nullptr) {
                *err = "native TensorRT SuperPoint only supports device=auto|cuda";
            }
            return false;
        }
        const int widthHint = EnvInt("SMART_DRONE_SUPERPOINT_INPUT_MAX_WIDTH", 640);
        const int heightHint = EnvInt("SMART_DRONE_SUPERPOINT_INPUT_MAX_HEIGHT", 480);
        const std::filesystem::path enginePath = ResolveSuperPointEnginePath(repoPath, widthHint, heightHint);
        if (enginePath.empty()) {
            if (err != nullptr) {
                *err = "SuperPoint TensorRT engine not found under repo: " + repoPath;
            }
            return false;
        }
        auto candidate = std::make_unique<TensorRtSuperPointEngine>();
        if (!candidate->Load(enginePath, err)) {
            return false;
        }
        if (!candidate->PreferredInputSize(inputHeight, inputWidth)) {
            inputHeight = heightHint;
            inputWidth = widthHint;
        }
        trtEngine = std::move(candidate);
        std::cerr << "[superpoint_trt] loaded engine=" << enginePath.string()
                  << " input=" << inputWidth << "x" << inputHeight << "\n";

        const std::filesystem::path lightGluePath = ResolveLightGlueEnginePath(repoPath, maxPointsForLightGlue());
        if (!lightGluePath.empty()) {
            auto matcher = std::make_unique<TensorRtLightGlueEngine>();
            std::string lgErr;
            if (matcher->Load(lightGluePath, &lgErr)) {
                lightGlueEngine = std::move(matcher);
                lightGluePointCount = maxPointsForLightGlue();
                lightGlueMinScore = EnvFloat("SMART_DRONE_LIGHTGLUE_MIN_SCORE", 0.02f);
                lightGlueMaxYDiffPx = EnvFloat("SMART_DRONE_LIGHTGLUE_MAX_Y_DIFF_PX", 1.5f);
                lightGlueMinDisparityPx = EnvFloat("SMART_DRONE_LIGHTGLUE_MIN_DISPARITY_PX", 0.8f);
                std::cerr << "[lightglue_trt] loaded engine=" << lightGluePath.string()
                          << " points=" << lightGluePointCount
                          << " min_score=" << lightGlueMinScore
                          << " max_y_diff_px=" << lightGlueMaxYDiffPx
                          << " min_disparity_px=" << lightGlueMinDisparityPx << "\n";
            } else {
                std::cerr << "[lightglue_trt] warning: failed to load engine=" << lightGluePath.string()
                          << " err=" << lgErr << "; fallback=descriptor_mutual\n";
            }
        } else {
            std::cerr << "[lightglue_trt] engine not found; fallback=descriptor_mutual\n";
        }
        return true;
    }

    int maxPointsForLightGlue() const { return EnvInt("SMART_DRONE_LIGHTGLUE_POINTS", 768); }

    bool DetectAndComputeBatch(const std::vector<cv::Mat> &grayImages, int maxPoints,
                               std::vector<SuperPointFeatureSet> &outputs, double *inputMs, double *forwardMs,
                               double *postMs, std::string *err)
    {
        outputs.assign(grayImages.size(), SuperPointFeatureSet{});
        if (!trtEngine || grayImages.empty()) {
            if (err != nullptr) {
                *err = "SuperPoint TensorRT backend is not ready";
            }
            return false;
        }
        double inputTotalMs = 0.0;
        double forwardTotalMs = 0.0;
        double postTotalMs = 0.0;
        for (size_t batchIndex = 0; batchIndex < grayImages.size(); ++batchIndex) {
            const int targetHeight = inputHeight > 0 ? inputHeight : grayImages[batchIndex].rows;
            const int targetWidth = inputWidth > 0 ? inputWidth : grayImages[batchIndex].cols;
            const double ratioH = static_cast<double>(grayImages[batchIndex].rows) / static_cast<double>(targetHeight);
            const double ratioW = static_cast<double>(grayImages[batchIndex].cols) / static_cast<double>(targetWidth);
            const auto inputStartTp = std::chrono::steady_clock::now();
            const std::vector<float> input = BuildInputBatch({grayImages[batchIndex]}, targetHeight, targetWidth);
            const auto inputEndTp = std::chrono::steady_clock::now();
            const auto forwardStartTp = inputEndTp;
            if (!trtEngine->Forward(input, 1, targetHeight, targetWidth, detector, descriptors, err)) {
                return false;
            }
            const auto forwardEndTp = std::chrono::steady_clock::now();
            if (detector.Dim(1) < 65 || descriptors.Dim(1) != kSuperPointDescriptorDim) {
                if (err != nullptr) {
                    *err = "TensorRT SuperPoint outputs have unexpected shapes";
                }
                return false;
            }
            const int heatmapWidth = detector.Dim(3) * kSuperPointCellSize;
            const int heatmapHeight = detector.Dim(2) * kSuperPointCellSize;
            if (heatmapWidth != targetWidth || heatmapHeight != targetHeight) {
                if (err != nullptr) {
                    *err = "TensorRT SuperPoint detector output size does not match input";
                }
                return false;
            }
            const auto postStartTp = forwardEndTp;
            std::vector<Candidate> candidates = ExtractCandidates(detector, 0, targetWidth, targetHeight, maxPoints);
            SuperPointFeatureSet &output = outputs[batchIndex];
            output.keypoints.reserve(candidates.size());
            if (!candidates.empty()) {
                output.descriptors = cv::Mat(static_cast<int>(candidates.size()), kSuperPointDescriptorDim, CV_32F);
            }
            for (size_t i = 0; i < candidates.size(); ++i) {
                const Candidate &candidate = candidates[i];
                output.keypoints.emplace_back(static_cast<float>(candidate.x * ratioW),
                                              static_cast<float>(candidate.y * ratioH));
                float *descriptor = output.descriptors.ptr<float>(static_cast<int>(i));
                const float sampleX =
                    (static_cast<float>(candidate.x) - kSuperPointCellSize / 2.0f + 0.5f) /
                    (static_cast<float>(descriptors.Dim(3) * kSuperPointCellSize) - kSuperPointCellSize / 2.0f -
                     0.5f) *
                    static_cast<float>(descriptors.Dim(3) - 1);
                const float sampleY =
                    (static_cast<float>(candidate.y) - kSuperPointCellSize / 2.0f + 0.5f) /
                    (static_cast<float>(descriptors.Dim(2) * kSuperPointCellSize) - kSuperPointCellSize / 2.0f -
                     0.5f) *
                    static_cast<float>(descriptors.Dim(2) - 1);
                for (int c = 0; c < kSuperPointDescriptorDim; ++c) {
                    descriptor[c] = BilinearAt(descriptors, 0, c, sampleX, sampleY);
                }
                NormalizeVector(descriptor, kSuperPointDescriptorDim);
            }
            const auto postEndTp = std::chrono::steady_clock::now();
            inputTotalMs += DurationMs(inputStartTp, inputEndTp);
            forwardTotalMs += DurationMs(forwardStartTp, forwardEndTp);
            postTotalMs += DurationMs(postStartTp, postEndTp);
        }
        if (inputMs != nullptr) {
            *inputMs = inputTotalMs;
        }
        if (forwardMs != nullptr) {
            *forwardMs = forwardTotalMs;
        }
        if (postMs != nullptr) {
            *postMs = postTotalMs;
        }
        return true;
    }

    bool MatchWithLightGlue(const SuperPointFeatureSet &leftRaw, const SuperPointFeatureSet &rightRaw, int maxPoints,
                            int imageWidth, int imageHeight, SuperPointFeatureSet &leftOut, SuperPointFeatureSet &rightOut,
                            double *matchMs, std::string *err)
    {
        leftOut = SuperPointFeatureSet{};
        rightOut = SuperPointFeatureSet{};
        if (!lightGlueEngine || lightGluePointCount <= 0 || leftRaw.descriptors.empty() ||
            rightRaw.descriptors.empty() || leftRaw.descriptors.type() != CV_32F ||
            rightRaw.descriptors.type() != CV_32F || leftRaw.descriptors.cols != kSuperPointDescriptorDim ||
            rightRaw.descriptors.cols != kSuperPointDescriptorDim) {
            return false;
        }

        const int leftCount = std::min({static_cast<int>(leftRaw.keypoints.size()), leftRaw.descriptors.rows,
                                        lightGluePointCount});
        const int rightCount = std::min({static_cast<int>(rightRaw.keypoints.size()), rightRaw.descriptors.rows,
                                         lightGluePointCount});
        if (leftCount <= 0 || rightCount <= 0) {
            return false;
        }

        const size_t kptsSize = static_cast<size_t>(lightGluePointCount) * 2;
        const size_t descSize = static_cast<size_t>(lightGluePointCount) * kSuperPointDescriptorDim;
        std::vector<float> kpts0(kptsSize, -1000.0f);
        std::vector<float> kpts1(kptsSize, -1000.0f);
        std::vector<float> desc0(descSize, 0.0f);
        std::vector<float> desc1(descSize, 0.0f);
        for (int i = 0; i < leftCount; ++i) {
            kpts0[static_cast<size_t>(i) * 2] = leftRaw.keypoints[static_cast<size_t>(i)].x;
            kpts0[static_cast<size_t>(i) * 2 + 1] = leftRaw.keypoints[static_cast<size_t>(i)].y;
            const float *src = leftRaw.descriptors.ptr<float>(i);
            std::copy(src, src + kSuperPointDescriptorDim,
                      desc0.data() + static_cast<size_t>(i) * kSuperPointDescriptorDim);
        }
        for (int i = 0; i < rightCount; ++i) {
            kpts1[static_cast<size_t>(i) * 2] = rightRaw.keypoints[static_cast<size_t>(i)].x;
            kpts1[static_cast<size_t>(i) * 2 + 1] = rightRaw.keypoints[static_cast<size_t>(i)].y;
            const float *src = rightRaw.descriptors.ptr<float>(i);
            std::copy(src, src + kSuperPointDescriptorDim,
                      desc1.data() + static_cast<size_t>(i) * kSuperPointDescriptorDim);
        }

        const auto matchStart = std::chrono::steady_clock::now();
        const std::array<float, 2> imageSize{static_cast<float>(imageWidth), static_cast<float>(imageHeight)};
        if (!lightGlueEngine->Forward(kpts0, kpts1, desc0, desc1, imageSize, imageSize, lightGluePointCount,
                                      lightGlueScores, err)) {
            return false;
        }
        const auto matchEnd = std::chrono::steady_clock::now();
        if (matchMs != nullptr) {
            *matchMs = DurationMs(matchStart, matchEnd);
        }

        const int matrixRows = lightGlueScores.dims.size() == 3 ? lightGlueScores.Dim(1) : lightGlueScores.Dim(0);
        const int matrixCols = lightGlueScores.dims.size() == 3 ? lightGlueScores.Dim(2) : lightGlueScores.Dim(1);
        const int outLeftCount = std::min(leftCount, matrixRows);
        const int outRightCount = std::min(rightCount, matrixCols);
        struct Pair {
            int left{0};
            int right{0};
            float score{0.0f};
            float disparity{0.0f};
        };
        std::vector<Pair> pairs;
        std::vector<int> bestRightForLeft(static_cast<size_t>(outLeftCount), -1);
        std::vector<float> bestScoreForLeft(static_cast<size_t>(outLeftCount), -1.0f);
        std::vector<int> bestLeftForRight(static_cast<size_t>(outRightCount), -1);
        std::vector<float> bestScoreForRight(static_cast<size_t>(outRightCount), -1.0f);
        auto scoreAt = [&](int li, int ri) {
            return lightGlueScores.data[static_cast<size_t>(li) * static_cast<size_t>(matrixCols) +
                                        static_cast<size_t>(ri)];
        };
        for (int li = 0; li < outLeftCount; ++li) {
            for (int ri = 0; ri < outRightCount; ++ri) {
                const float score = scoreAt(li, ri);
                if (score > bestScoreForLeft[static_cast<size_t>(li)]) {
                    bestScoreForLeft[static_cast<size_t>(li)] = score;
                    bestRightForLeft[static_cast<size_t>(li)] = ri;
                }
                if (score > bestScoreForRight[static_cast<size_t>(ri)]) {
                    bestScoreForRight[static_cast<size_t>(ri)] = score;
                    bestLeftForRight[static_cast<size_t>(ri)] = li;
                }
            }
        }
        pairs.reserve(static_cast<size_t>(outLeftCount));
        for (int li = 0; li < outLeftCount; ++li) {
            const int ri = bestRightForLeft[static_cast<size_t>(li)];
            const float score = bestScoreForLeft[static_cast<size_t>(li)];
            if (ri < 0 || ri >= outRightCount || bestLeftForRight[static_cast<size_t>(ri)] != li ||
                score < lightGlueMinScore) {
                continue;
            }
            const cv::Point2f &lp = leftRaw.keypoints[static_cast<size_t>(li)];
            const cv::Point2f &rp = rightRaw.keypoints[static_cast<size_t>(ri)];
            const float disparity = lp.x - rp.x;
            if (std::abs(lp.y - rp.y) > lightGlueMaxYDiffPx || disparity <= lightGlueMinDisparityPx) {
                continue;
            }
            pairs.push_back(Pair{li, ri, score, disparity});
        }
        std::sort(pairs.begin(), pairs.end(), [](const Pair &lhs, const Pair &rhs) {
            if (std::abs(lhs.score - rhs.score) > 1.0e-6f) {
                return lhs.score > rhs.score;
            }
            return lhs.disparity > rhs.disparity;
        });
        if (static_cast<int>(pairs.size()) > std::max(1, maxPoints)) {
            pairs.resize(static_cast<size_t>(std::max(1, maxPoints)));
        }

        leftOut.keypoints.reserve(pairs.size());
        rightOut.keypoints.reserve(pairs.size());
        leftOut.descriptors = cv::Mat(static_cast<int>(pairs.size()), kSuperPointDescriptorDim, CV_32F);
        rightOut.descriptors = cv::Mat(static_cast<int>(pairs.size()), kSuperPointDescriptorDim, CV_32F);
        for (size_t i = 0; i < pairs.size(); ++i) {
            leftOut.keypoints.push_back(leftRaw.keypoints[static_cast<size_t>(pairs[i].left)]);
            rightOut.keypoints.push_back(rightRaw.keypoints[static_cast<size_t>(pairs[i].right)]);
            leftRaw.descriptors.row(pairs[i].left).copyTo(leftOut.descriptors.row(static_cast<int>(i)));
            rightRaw.descriptors.row(pairs[i].right).copyTo(rightOut.descriptors.row(static_cast<int>(i)));
        }
        return true;
    }
#endif
};

SuperPointNativeExtractor::~SuperPointNativeExtractor() = default;

bool SuperPointNativeExtractor::PrepareGrayImage(const cv::Mat &gray, cv::Mat &gray8, std::string *err)
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
            *err = "SuperPoint native gray image preparation failed";
        }
        return false;
    }
    return true;
}

bool SuperPointNativeExtractor::Start(const std::string &repoPath, const std::string &device, int topK, int maxPoints,
                                      std::string *err)
{
    m_topK = topK;
    m_maxPoints = maxPoints;
    m_lastStats = Stats{};
#if defined(SMART_DRONE_SUPERPOINT_TENSORRT_AVAILABLE)
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
        *err = "native TensorRT SuperPoint backend is unavailable in this target";
    }
    return false;
#endif
}

void SuperPointNativeExtractor::Stop()
{
    m_running = false;
    m_lastStats = Stats{};
    m_impl.reset();
}

bool SuperPointNativeExtractor::Running() const { return m_running; }

SuperPointNativeExtractor::Stats SuperPointNativeExtractor::LastStats() const { return m_lastStats; }

bool SuperPointNativeExtractor::Detect(const cv::Mat &gray, std::vector<cv::Point2f> &outPoints, std::string *err)
{
    SuperPointFeatureSet features;
    if (!DetectAndCompute(gray, features, err)) {
        return false;
    }
    outPoints = std::move(features.keypoints);
    return true;
}

bool SuperPointNativeExtractor::DetectAndCompute(const cv::Mat &gray, SuperPointFeatureSet &outFeatures, std::string *err)
{
    outFeatures = SuperPointFeatureSet{};
    m_lastStats = Stats{};
#if defined(SMART_DRONE_SUPERPOINT_TENSORRT_AVAILABLE)
    if (!m_running || !m_impl) {
        if (err != nullptr) {
            *err = "SuperPoint TensorRT backend not running";
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
    std::vector<SuperPointFeatureSet> outputs;
    const auto inferStartTp = prepareEndTp;
    double inputMs = 0.0;
    double forwardMs = 0.0;
    double postMs = 0.0;
    if (!m_impl->DetectAndComputeBatch({gray8}, std::max(m_topK, m_maxPoints), outputs, &inputMs, &forwardMs, &postMs,
                                       err) ||
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
        *err = "native TensorRT SuperPoint backend is compiled out";
    }
    (void)gray;
    return false;
#endif
}

bool SuperPointNativeExtractor::DetectAndComputeStereo(const cv::Mat &leftGray, const cv::Mat &rightGray,
                                                       SuperPointFeatureSet &leftFeatures, SuperPointFeatureSet &rightFeatures,
                                                       std::string *err)
{
    leftFeatures = SuperPointFeatureSet{};
    rightFeatures = SuperPointFeatureSet{};
    m_lastStats = Stats{};
#if defined(SMART_DRONE_SUPERPOINT_TENSORRT_AVAILABLE)
    if (!m_running || !m_impl) {
        if (err != nullptr) {
            *err = "SuperPoint TensorRT backend not running";
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
    std::vector<SuperPointFeatureSet> rawOutputs;
    const auto inferStartTp = prepareEndTp;
    double inputMs = 0.0;
    double forwardMs = 0.0;
    double postMs = 0.0;
    const int extractionBudget = std::max(m_topK, std::max(1, m_maxPoints) * 2);
    if (!m_impl->DetectAndComputeBatch({leftGray8, rightGray8}, extractionBudget, rawOutputs, &inputMs, &forwardMs,
                                       &postMs, err) ||
        rawOutputs.size() != 2) {
        return false;
    }
    double lightGlueMatchMs = 0.0;
    bool usedLightGlue = false;
    if (m_impl->MatchWithLightGlue(rawOutputs[0], rawOutputs[1], std::max(1, m_maxPoints), leftGray8.cols,
                                   leftGray8.rows, leftFeatures, rightFeatures, &lightGlueMatchMs, err)) {
        usedLightGlue = true;
    } else {
        MatchStereoPairs(rawOutputs[0], rawOutputs[1], std::max(1, m_maxPoints), leftFeatures, rightFeatures);
    }
    const auto inferEndTp = std::chrono::steady_clock::now();
    m_lastStats.prepareMs = DurationMs(prepareStartTp, prepareEndTp);
    m_lastStats.inputMs = inputMs;
    m_lastStats.forwardMs = forwardMs;
    m_lastStats.postMs = postMs;
    m_lastStats.inferMs = DurationMs(inferStartTp, inferEndTp);
    m_lastStats.totalMs = DurationMs(totalStartTp, inferEndTp);
    m_lastStats.imageCount = 2;
    m_lastStats.payloadBytes = static_cast<uint32_t>(leftGray8.total() + rightGray8.total());
    std::cerr << "[superpoint_trt_perf] batch=2 input_ms=" << inputMs << " gpu_forward_ms=" << forwardMs
              << " cpu_post_ms=" << postMs
              << " lightglue=" << (usedLightGlue ? "Y" : "N")
              << " lightglue_ms=" << lightGlueMatchMs << " total_ms=" << m_lastStats.totalMs
              << " left_pts=" << leftFeatures.keypoints.size() << " right_pts=" << rightFeatures.keypoints.size()
              << "\n";
    return true;
#else
    if (err != nullptr) {
        *err = "native TensorRT SuperPoint backend is compiled out";
    }
    (void)leftGray;
    (void)rightGray;
    return false;
#endif
}

} // namespace smartdrone::adapters::slam
