#include "adapters/slam/dpvo_tensorrt_engine.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <sophus/se3.hpp>

#include "Tracking.h"
#include "adapters/slam/slam_mode_common.h"
#include "adapters/slam/slam_mode_state.h"

#if defined(SMART_DRONE_DPVO_TENSORRT_AVAILABLE)
#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_runtime_api.h>
#endif

namespace smartdrone::adapters::slam {

namespace {

std::filesystem::path ResolveEnginePath(const std::string &explicitPath, const std::string &repoPath,
                                        const std::vector<std::string> &names)
{
    if (!explicitPath.empty() && std::filesystem::exists(explicitPath)) {
        return std::filesystem::path(explicitPath);
    }
    const std::filesystem::path repo(repoPath);
    if (!repo.empty()) {
        for (const std::string &name : names) {
            const std::filesystem::path candidate = repo / "weights" / name;
            if (std::filesystem::exists(candidate)) {
                return candidate;
            }
        }
    }
    return {};
}

double ElapsedMs(const std::chrono::steady_clock::time_point &start,
                 const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

bool TrackingStateCanPublishPose(int trackingState)
{
    return trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::RECENTLY_LOST ||
           trackingState == ORB_SLAM3::Tracking::OK_KLT;
}

core::ports::PoseEstimate PoseFromTwc(const Sophus::SE3f &twc)
{
    core::ports::PoseEstimate pose{};
    const Eigen::Vector3f t = twc.translation();
    const Eigen::Quaternionf q(twc.so3().unit_quaternion());
    pose.valid = std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z()) &&
                 std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z());
    pose.x = t.x();
    pose.y = t.y();
    pose.z = t.z();
    pose.qw = q.w();
    pose.qx = q.x();
    pose.qy = q.y();
    pose.qz = q.z();
    return pose;
}

} // namespace

#if defined(SMART_DRONE_DPVO_TENSORRT_AVAILABLE)

namespace {

class TensorRtLogger final : public nvinfer1::ILogger {
  public:
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kWARNING) {
            std::cerr << "[dpvo_trt] " << (msg != nullptr ? msg : "") << "\n";
        }
    }
};

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

class TensorRtEngineHandle {
  public:
    bool Load(const std::filesystem::path &enginePath, const char *name, std::string *err)
    {
        std::ifstream in(enginePath, std::ios::binary);
        if (!in) {
            if (err != nullptr) {
                *err = std::string(name) + " TensorRT engine not found: " + enginePath.string();
            }
            return false;
        }
        std::vector<char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        if (bytes.empty()) {
            if (err != nullptr) {
                *err = std::string(name) + " TensorRT engine is empty: " + enginePath.string();
            }
            return false;
        }

        initLibNvInferPlugins(&m_logger, "");
        m_runtime.reset(nvinfer1::createInferRuntime(m_logger));
        if (!m_runtime) {
            if (err != nullptr) {
                *err = std::string("failed to create ") + name + " TensorRT runtime";
            }
            return false;
        }
        m_engine.reset(m_runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
        if (!m_engine) {
            if (err != nullptr) {
                *err = std::string("failed to deserialize ") + name + " TensorRT engine: " + enginePath.string();
            }
            return false;
        }
        m_context.reset(m_engine->createExecutionContext());
        if (!m_context) {
            if (err != nullptr) {
                *err = std::string("failed to create ") + name + " TensorRT execution context";
            }
            return false;
        }
        m_path = enginePath.string();
        return true;
    }

    bool Loaded() const { return static_cast<bool>(m_engine) && static_cast<bool>(m_context); }
    const std::string &Path() const { return m_path; }
    nvinfer1::ICudaEngine *Engine() const { return m_engine.get(); }
    nvinfer1::IExecutionContext *Context() const { return m_context.get(); }

  private:
    TensorRtLogger m_logger{};
    std::unique_ptr<nvinfer1::IRuntime, DestroyRuntime> m_runtime;
    std::unique_ptr<nvinfer1::ICudaEngine, DestroyEngine> m_engine;
    std::unique_ptr<nvinfer1::IExecutionContext, DestroyContext> m_context;
    std::string m_path;
};

struct CudaStreamHandle {
    cudaStream_t stream{nullptr};

    bool Create(std::string *err)
    {
        if (stream != nullptr) {
            return true;
        }
        const cudaError_t rc = cudaStreamCreate(&stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("cudaStreamCreate failed: ") + cudaGetErrorString(rc);
            }
            stream = nullptr;
            return false;
        }
        return true;
    }

    void Reset()
    {
        if (stream != nullptr) {
            cudaStreamDestroy(stream);
            stream = nullptr;
        }
    }

    ~CudaStreamHandle() { Reset(); }
};

class CudaDeviceBuffer {
  public:
    ~CudaDeviceBuffer() { Reset(); }

    CudaDeviceBuffer() = default;
    CudaDeviceBuffer(const CudaDeviceBuffer &) = delete;
    CudaDeviceBuffer &operator=(const CudaDeviceBuffer &) = delete;

    bool Ensure(size_t bytes, std::string *err)
    {
        if (bytes <= m_bytes) {
            return true;
        }
        Reset();
        const cudaError_t rc = cudaMalloc(&m_ptr, bytes);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("cudaMalloc failed bytes=") + std::to_string(bytes) + ": " +
                       cudaGetErrorString(rc);
            }
            m_ptr = nullptr;
            m_bytes = 0;
            return false;
        }
        m_bytes = bytes;
        return true;
    }

    void Reset()
    {
        if (m_ptr != nullptr) {
            cudaFree(m_ptr);
            m_ptr = nullptr;
        }
        m_bytes = 0;
    }

    void *Data() const { return m_ptr; }
    size_t Bytes() const { return m_bytes; }

  private:
    void *m_ptr{nullptr};
    size_t m_bytes{0};
};

size_t TensorRtDataTypeSize(nvinfer1::DataType type)
{
    switch (type) {
    case nvinfer1::DataType::kFLOAT:
        return 4;
    case nvinfer1::DataType::kHALF:
        return 2;
    case nvinfer1::DataType::kINT8:
    case nvinfer1::DataType::kBOOL:
        return 1;
    case nvinfer1::DataType::kINT32:
        return 4;
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

bool SetBindingShape(nvinfer1::IExecutionContext &context, int bindingIndex, const nvinfer1::Dims &dims,
                     std::string *err)
{
    if (bindingIndex < 0) {
        if (err != nullptr) {
            *err = "invalid TensorRT binding index";
        }
        return false;
    }
    if (!context.setBindingDimensions(bindingIndex, dims)) {
        if (err != nullptr) {
            *err = "setBindingDimensions failed for binding " + std::to_string(bindingIndex) +
                   " dims=" + DimsToString(dims);
        }
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
    return engine->getBindingIndex(name);
}

bool BindingBytes(const TensorRtEngineHandle &handle, int bindingIndex, size_t *bytes, nvinfer1::Dims *dimsOut,
                  std::string *err)
{
    if (bytes == nullptr) {
        return false;
    }
    nvinfer1::ICudaEngine *engine = handle.Engine();
    nvinfer1::IExecutionContext *context = handle.Context();
    if (engine == nullptr || context == nullptr || bindingIndex < 0) {
        if (err != nullptr) {
            *err = "invalid TensorRT engine/context/binding";
        }
        return false;
    }
    nvinfer1::Dims dims = context->getBindingDimensions(bindingIndex);
    const int64_t volume = DimsVolume(dims);
    const size_t elementSize = TensorRtDataTypeSize(engine->getBindingDataType(bindingIndex));
    if (volume <= 0 || elementSize == 0) {
        if (err != nullptr) {
            *err = "invalid TensorRT binding shape/type index=" + std::to_string(bindingIndex) +
                   " dims=" + DimsToString(dims);
        }
        return false;
    }
    *bytes = static_cast<size_t>(volume) * elementSize;
    if (dimsOut != nullptr) {
        *dimsOut = dims;
    }
    return true;
}

struct DpvoPatchifierRun {
    nvinfer1::Dims fmapDims{};
    nvinfer1::Dims imapDims{};
    double elapsedMs{0.0};
    bool ok{false};
};

class DpvoPatchifierRuntime {
  public:
    bool Initialize(const TensorRtEngineHandle &engine, int width, int height, std::string *err)
    {
        m_imageIndex = FindBindingIndex(engine, "image");
        m_fmapIndex = FindBindingIndex(engine, "fmap");
        m_imapIndex = FindBindingIndex(engine, "imap");
        if (m_imageIndex < 0 || m_fmapIndex < 0 || m_imapIndex < 0) {
            if (err != nullptr) {
                *err = "patchifier binding lookup failed: image=" + std::to_string(m_imageIndex) +
                       " fmap=" + std::to_string(m_fmapIndex) + " imap=" + std::to_string(m_imapIndex);
            }
            return false;
        }
        m_width = width;
        m_height = height;
        m_inputHost.assign(static_cast<size_t>(3) * static_cast<size_t>(width) * static_cast<size_t>(height), 0.0f);
        return true;
    }

    DpvoPatchifierRun Run(const cv::Mat &gray, TensorRtEngineHandle &engine, cudaStream_t stream, std::string *err)
    {
        DpvoPatchifierRun result{};
        if (gray.empty() || gray.cols != m_width || gray.rows != m_height || gray.type() != CV_8UC1) {
            if (err != nullptr) {
                *err = "patchifier expects CV_8UC1 " + std::to_string(m_width) + "x" + std::to_string(m_height);
            }
            return result;
        }

        nvinfer1::IExecutionContext *context = engine.Context();
        if (context == nullptr || stream == nullptr) {
            if (err != nullptr) {
                *err = "patchifier TensorRT context or CUDA stream is not initialized";
            }
            return result;
        }
        if (!SetBindingShape(*context, m_imageIndex, nvinfer1::Dims4{1, 3, m_height, m_width}, err)) {
            return result;
        }

        size_t imageBytes = 0;
        size_t fmapBytes = 0;
        size_t imapBytes = 0;
        if (!BindingBytes(engine, m_imageIndex, &imageBytes, nullptr, err) ||
            !BindingBytes(engine, m_fmapIndex, &fmapBytes, &result.fmapDims, err) ||
            !BindingBytes(engine, m_imapIndex, &imapBytes, &result.imapDims, err)) {
            return result;
        }
        if (imageBytes != m_inputHost.size() * sizeof(float)) {
            if (err != nullptr) {
                *err = "patchifier image binding byte mismatch expected=" +
                       std::to_string(m_inputHost.size() * sizeof(float)) + " got=" + std::to_string(imageBytes);
            }
            return result;
        }

        if (!m_imageDevice.Ensure(imageBytes, err) || !m_fmapDevice.Ensure(fmapBytes, err) ||
            !m_imapDevice.Ensure(imapBytes, err)) {
            return result;
        }

        FillInput(gray);
        const int nbBindings = engine.Engine() != nullptr ? engine.Engine()->getNbBindings() : 0;
        if (nbBindings <= 0) {
            if (err != nullptr) {
                *err = "patchifier engine has no bindings";
            }
            return result;
        }
        std::vector<void *> bindings(static_cast<size_t>(nbBindings), nullptr);
        bindings[static_cast<size_t>(m_imageIndex)] = m_imageDevice.Data();
        bindings[static_cast<size_t>(m_fmapIndex)] = m_fmapDevice.Data();
        bindings[static_cast<size_t>(m_imapIndex)] = m_imapDevice.Data();

        const auto t0 = std::chrono::steady_clock::now();
        cudaError_t rc = cudaMemcpyAsync(m_imageDevice.Data(), m_inputHost.data(), imageBytes,
                                         cudaMemcpyHostToDevice, stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("patchifier H2D copy failed: ") + cudaGetErrorString(rc);
            }
            return result;
        }
        if (!context->enqueueV2(bindings.data(), stream, nullptr)) {
            if (err != nullptr) {
                *err = "patchifier enqueueV2 failed";
            }
            return result;
        }
        rc = cudaStreamSynchronize(stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("patchifier synchronize failed: ") + cudaGetErrorString(rc);
            }
            return result;
        }
        result.elapsedMs = ElapsedMs(t0, std::chrono::steady_clock::now());
        result.ok = true;
        m_lastFmapDims = result.fmapDims;
        m_lastImapDims = result.imapDims;
        return result;
    }

    void *FmapDevice() const { return m_fmapDevice.Data(); }
    void *ImapDevice() const { return m_imapDevice.Data(); }
    nvinfer1::Dims LastFmapDims() const { return m_lastFmapDims; }
    nvinfer1::Dims LastImapDims() const { return m_lastImapDims; }

  private:
    void FillInput(const cv::Mat &gray)
    {
        const size_t plane = static_cast<size_t>(m_width) * static_cast<size_t>(m_height);
        for (int y = 0; y < m_height; ++y) {
            const uint8_t *src = gray.ptr<uint8_t>(y);
            for (int x = 0; x < m_width; ++x) {
                const float v = static_cast<float>(src[x]);
                const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(m_width) + static_cast<size_t>(x);
                m_inputHost[idx] = v;
                m_inputHost[plane + idx] = v;
                m_inputHost[2 * plane + idx] = v;
            }
        }
    }

    int m_imageIndex{-1};
    int m_fmapIndex{-1};
    int m_imapIndex{-1};
    int m_width{0};
    int m_height{0};
    std::vector<float> m_inputHost;
    CudaDeviceBuffer m_imageDevice;
    CudaDeviceBuffer m_fmapDevice;
    CudaDeviceBuffer m_imapDevice;
    nvinfer1::Dims m_lastFmapDims{};
    nvinfer1::Dims m_lastImapDims{};
};

struct DpvoUpdateRun {
    double elapsedMs{0.0};
    bool ok{false};
};

class DpvoUpdateRuntime {
  public:
    bool Initialize(const TensorRtEngineHandle &engine, std::string *err)
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
        const bool ok = m_netIndex >= 0 && m_inpIndex >= 0 && m_corrIndex >= 0 && m_prevNetIndex >= 0 &&
                        m_nextNetIndex >= 0 && m_prevMaskIndex >= 0 && m_nextMaskIndex >= 0 &&
                        m_updatedNetIndex >= 0 && m_deltaIndex >= 0 && m_weightIndex >= 0;
        if (!ok && err != nullptr) {
            *err = "update binding lookup failed";
        }
        return ok;
    }

    DpvoUpdateRun Warmup(TensorRtEngineHandle &engine, cudaStream_t stream, int edges, std::string *err)
    {
        DpvoUpdateRun result{};
        edges = std::clamp(edges, 1, 4096);
        nvinfer1::IExecutionContext *context = engine.Context();
        if (context == nullptr || stream == nullptr) {
            if (err != nullptr) {
                *err = "update TensorRT context or CUDA stream is not initialized";
            }
            return result;
        }
        if (!SetBindingShape(*context, m_netIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_inpIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_corrIndex, nvinfer1::Dims3{1, edges, kCorrDim}, err) ||
            !SetBindingShape(*context, m_prevNetIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_nextNetIndex, nvinfer1::Dims3{1, edges, kDim}, err) ||
            !SetBindingShape(*context, m_prevMaskIndex, nvinfer1::Dims3{1, edges, 1}, err) ||
            !SetBindingShape(*context, m_nextMaskIndex, nvinfer1::Dims3{1, edges, 1}, err)) {
            return result;
        }

        std::array<int, 10> indices{m_netIndex, m_inpIndex, m_corrIndex, m_prevNetIndex, m_nextNetIndex,
                                    m_prevMaskIndex, m_nextMaskIndex, m_updatedNetIndex, m_deltaIndex, m_weightIndex};
        for (int index : indices) {
            size_t bytes = 0;
            if (!BindingBytes(engine, index, &bytes, nullptr, err)) {
                return result;
            }
            if (index >= static_cast<int>(m_buffers.size())) {
                if (err != nullptr) {
                    *err = "update binding index exceeds local buffer table";
                }
                return result;
            }
            if (!m_buffers[static_cast<size_t>(index)].Ensure(bytes, err)) {
                return result;
            }
            if (engine.Engine()->bindingIsInput(index)) {
                const cudaError_t rc = cudaMemsetAsync(m_buffers[static_cast<size_t>(index)].Data(), 0, bytes, stream);
                if (rc != cudaSuccess) {
                    if (err != nullptr) {
                        *err = std::string("update input memset failed: ") + cudaGetErrorString(rc);
                    }
                    return result;
                }
            }
        }

        std::array<void *, 16> bindings{};
        const int nbBindings = engine.Engine()->getNbBindings();
        if (nbBindings > static_cast<int>(bindings.size())) {
            if (err != nullptr) {
                *err = "update engine has more bindings than expected";
            }
            return result;
        }
        for (int i = 0; i < nbBindings; ++i) {
            bindings[static_cast<size_t>(i)] = m_buffers[static_cast<size_t>(i)].Data();
        }

        const auto t0 = std::chrono::steady_clock::now();
        if (!context->enqueueV2(bindings.data(), stream, nullptr)) {
            if (err != nullptr) {
                *err = "update enqueueV2 failed";
            }
            return result;
        }
        const cudaError_t rc = cudaStreamSynchronize(stream);
        if (rc != cudaSuccess) {
            if (err != nullptr) {
                *err = std::string("update synchronize failed: ") + cudaGetErrorString(rc);
            }
            return result;
        }
        result.elapsedMs = ElapsedMs(t0, std::chrono::steady_clock::now());
        result.ok = true;
        return result;
    }

    static constexpr int kDim = 384;
    static constexpr int kCorrDim = 882;

  private:
    int m_netIndex{-1};
    int m_inpIndex{-1};
    int m_corrIndex{-1};
    int m_prevNetIndex{-1};
    int m_nextNetIndex{-1};
    int m_prevMaskIndex{-1};
    int m_nextMaskIndex{-1};
    int m_updatedNetIndex{-1};
    int m_deltaIndex{-1};
    int m_weightIndex{-1};
    std::array<CudaDeviceBuffer, 16> m_buffers;
};

struct DpvoPatchState {
    float x{0.0f};
    float y{0.0f};
    float invDepth{1.0f};
};

struct DpvoFrameState {
    uint64_t frameId{0};
    int64_t timestampNs{0};
    Sophus::SE3f Twc;
    std::vector<DpvoPatchState> patches;
};

struct DpvoEdgeState {
    int patchGlobal{0};
    int sourceFrame{0};
    int targetFrame{0};
};

class DpvoGraphState {
  public:
    void Reset(int patchesPerFrame, int optimizationWindow)
    {
        m_patchesPerFrame = std::clamp(patchesPerFrame, 16, 256);
        m_optimizationWindow = std::clamp(optimizationWindow, 4, 32);
        m_patchLifetime = std::max(8, m_optimizationWindow + 4);
        m_removalWindow = std::max(12, m_optimizationWindow + 9);
        m_frames.clear();
        m_edges.clear();
        m_initialized = false;
        m_counter = 0;
    }

    void PushFrame(uint64_t frameId, int64_t timestampNs, const cv::Mat &gray, const Sophus::SE3f &initialPose)
    {
        DpvoFrameState frame{};
        frame.frameId = frameId;
        frame.timestampNs = timestampNs;
        frame.Twc = initialPose;
        frame.patches = SelectPatches(gray);
        if (frame.patches.size() < static_cast<size_t>(m_patchesPerFrame)) {
            frame.patches.resize(static_cast<size_t>(m_patchesPerFrame));
        }
        m_frames.push_back(std::move(frame));
        ++m_counter;
        if (m_frames.size() >= kInitializationFrames) {
            m_initialized = true;
        }
        RebuildEdges();
        PruneFrames();
    }

    bool Initialized() const { return m_initialized; }
    int FrameCount() const { return static_cast<int>(m_frames.size()); }
    int EdgeCount() const { return static_cast<int>(m_edges.size()); }
    int PatchCount() const { return FrameCount() * m_patchesPerFrame; }
    int PatchesPerFrame() const { return m_patchesPerFrame; }

  private:
    std::vector<DpvoPatchState> SelectPatches(const cv::Mat &gray) const
    {
        std::vector<cv::Point2f> points;
        if (!gray.empty()) {
            cv::goodFeaturesToTrack(gray, points, m_patchesPerFrame * 3, 0.01, 8.0, cv::Mat(), 7, false, 0.04);
        }
        points = SelectGfttPointsGridBalanced(points, gray.size(), m_patchesPerFrame, 8);
        std::vector<DpvoPatchState> patches;
        patches.reserve(static_cast<size_t>(m_patchesPerFrame));
        for (const cv::Point2f &pt : points) {
            if (patches.size() >= static_cast<size_t>(m_patchesPerFrame)) {
                break;
            }
            patches.push_back({pt.x * 0.25f, pt.y * 0.25f, 1.0f});
        }
        for (int y = 1; patches.size() < static_cast<size_t>(m_patchesPerFrame) && y < 7; ++y) {
            for (int x = 1; patches.size() < static_cast<size_t>(m_patchesPerFrame) && x < 9; ++x) {
                patches.push_back({(static_cast<float>(gray.cols) * x / 9.0f) * 0.25f,
                                   (static_cast<float>(gray.rows) * y / 7.0f) * 0.25f, 1.0f});
            }
        }
        return patches;
    }

    void RebuildEdges()
    {
        m_edges.clear();
        const int n = FrameCount();
        if (n <= 1) {
            return;
        }
        const int newest = n - 1;
        const int forwardPatchStart = m_patchesPerFrame * std::max(newest - m_patchLifetime, 0);
        const int forwardPatchEnd = m_patchesPerFrame * std::max(newest, 0);
        for (int k = forwardPatchStart; k < forwardPatchEnd; ++k) {
            m_edges.push_back({k, k / m_patchesPerFrame, newest});
        }
        const int backPatchStart = m_patchesPerFrame * newest;
        const int backPatchEnd = m_patchesPerFrame * n;
        const int backTargetStart = std::max(n - m_patchLifetime, 0);
        for (int k = backPatchStart; k < backPatchEnd; ++k) {
            for (int j = backTargetStart; j < n; ++j) {
                m_edges.push_back({k, newest, j});
            }
        }
        const int oldestActivePatchFrame = std::max(n - m_removalWindow, 0);
        m_edges.erase(std::remove_if(m_edges.begin(), m_edges.end(), [&](const DpvoEdgeState &edge) {
                          return edge.patchGlobal / m_patchesPerFrame < oldestActivePatchFrame;
                      }),
                      m_edges.end());
    }

    void PruneFrames()
    {
        const int maxFrames = std::max(36, m_removalWindow + m_optimizationWindow + 4);
        if (FrameCount() <= maxFrames) {
            return;
        }
        const int drop = FrameCount() - maxFrames;
        m_frames.erase(m_frames.begin(), m_frames.begin() + drop);
        RebuildEdges();
    }

    static constexpr int kInitializationFrames = 8;
    int m_patchesPerFrame{48};
    int m_optimizationWindow{7};
    int m_patchLifetime{11};
    int m_removalWindow{16};
    int m_counter{0};
    bool m_initialized{false};
    std::vector<DpvoFrameState> m_frames;
    std::vector<DpvoEdgeState> m_edges;
};

} // namespace

struct DpvoTensorRtEngine::Impl {
    explicit Impl(DpvoTensorRtConfig cfg) : config(std::move(cfg)) {}

    bool Start()
    {
        const std::filesystem::path patchPath =
            ResolveEnginePath(config.patchEnginePath, config.repoPath,
                              {"dpvo_patchifier_fp16.engine", "dpvo_patchifier.engine", "dpvo_patch.engine"});
        const std::filesystem::path updatePath =
            ResolveEnginePath(config.updateEnginePath, config.repoPath,
                              {"dpvo_update_fp16.engine", "dpvo_update.engine"});
        if (patchPath.empty() || updatePath.empty()) {
            std::cerr << "[dpvo_trt] missing engine(s): patch='" << config.patchEnginePath
                      << "' update='" << config.updateEnginePath << "' repo='" << config.repoPath << "'\n";
            return false;
        }

        std::string err;
        if (!patchEngine.Load(patchPath, "DPVO patchifier", &err)) {
            std::cerr << "[dpvo_trt] " << err << "\n";
            return false;
        }
        if (!updateEngine.Load(updatePath, "DPVO update", &err)) {
            std::cerr << "[dpvo_trt] " << err << "\n";
            return false;
        }
        if (!cudaStream.Create(&err)) {
            std::cerr << "[dpvo_trt] " << err << "\n";
            return false;
        }
        if (!patchifierRuntime.Initialize(patchEngine, config.inputWidth, config.inputHeight, &err)) {
            std::cerr << "[dpvo_trt] " << err << "\n";
            return false;
        }
        if (!updateRuntime.Initialize(updateEngine, &err)) {
            std::cerr << "[dpvo_trt] " << err << "\n";
            return false;
        }
        graphState.Reset(config.patchesPerFrame, config.optimizationWindow);
        DpvoUpdateRun updateWarmup = updateRuntime.Warmup(updateEngine, cudaStream.stream,
                                                          std::max(1, config.patchesPerFrame), &err);
        if (!updateWarmup.ok) {
            std::cerr << "[dpvo_trt] update warmup failed: " << err << "\n";
            return false;
        }
        running = true;
        std::cerr << "[dpvo_trt] ready patch_engine=" << patchEngine.Path()
                  << " update_engine=" << updateEngine.Path()
                  << " input=" << config.inputWidth << "x" << config.inputHeight
                  << " patches=" << config.patchesPerFrame
                  << " opt_window=" << config.optimizationWindow
                  << " update_warmup_ms=" << updateWarmup.elapsedMs << "\n";
        if (!voState.LoadStereoCalibration(config.settingsPath)) {
            std::cerr << "[dpvo_trt] stereo VO calibration unavailable settings='" << config.settingsPath
                      << "'; pose output disabled\n";
            running = false;
            return false;
        }
        voState.ResetTrackingState();
        return true;
    }

    void Stop()
    {
        running = false;
        cudaStream.Reset();
    }

    core::ports::SlamOutput Process(const core::ports::SlamInputBatch &input, bool extractFeatures,
                                    bool extractPointCloud)
    {
        (void)extractPointCloud;
        const auto start = std::chrono::steady_clock::now();
        core::ports::SlamOutput out{};
        out.frameId = input.frameId;
        out.captureTimestampNs = input.captureTimestampNs;
        out.mapId = 1;
        out.usedSuperPointFrontend = false;

        if (!running) {
            out.trackingState = ORB_SLAM3::Tracking::LOST;
            return out;
        }

        const auto prepareStart = std::chrono::steady_clock::now();
        cv::Mat leftGray = EnsureGray8(input.stereo.left.gray);
        cv::Mat rightGray = EnsureGray8(input.stereo.right.gray);
        if (leftGray.empty() || rightGray.empty()) {
            out.trackingState = ORB_SLAM3::Tracking::LOST;
            return out;
        }
        cv::Mat leftRect = leftGray;
        cv::Mat rightRect = rightGray;
        voState.EnsureStereoRectifier(leftGray.size());
        if (!voState.m_lkMap1x.empty() && !voState.m_lkMap2x.empty()) {
            const auto rectifyStart = std::chrono::steady_clock::now();
            cv::remap(leftGray, leftRect, voState.m_lkMap1x, voState.m_lkMap1y, cv::INTER_LINEAR);
            cv::remap(rightGray, rightRect, voState.m_lkMap2x, voState.m_lkMap2y, cv::INTER_LINEAR);
            out.lkRectifyMs = ElapsedMs(rectifyStart, std::chrono::steady_clock::now());
        }
        out.inputPrepareMs = ElapsedMs(prepareStart, std::chrono::steady_clock::now());

        if (leftRect.cols != config.inputWidth || leftRect.rows != config.inputHeight) {
            cv::resize(leftRect, resizedGray, cv::Size(config.inputWidth, config.inputHeight), 0.0, 0.0,
                       cv::INTER_AREA);
        } else {
            resizedGray = leftRect;
        }

        std::string dpvoErr;
        const DpvoPatchifierRun patchRun = patchifierRuntime.Run(resizedGray, patchEngine, cudaStream.stream, &dpvoErr);
        if (patchRun.ok) {
            out.superpointForwardMs = patchRun.elapsedMs;
            if (!loggedPatchifierShape) {
                std::cerr << "[dpvo_trt] patchifier active fmap=" << DimsToString(patchRun.fmapDims)
                          << " imap=" << DimsToString(patchRun.imapDims)
                          << " ms=" << patchRun.elapsedMs << "\n";
                loggedPatchifierShape = true;
            }
        } else if (!loggedPatchifierError) {
            std::cerr << "[dpvo_trt] patchifier inference disabled for this frame: " << dpvoErr << "\n";
            loggedPatchifierError = true;
        }
        graphState.PushFrame(input.frameId, input.captureTimestampNs, resizedGray,
                             voState.m_lkHavePrev ? voState.m_lkTwc : Sophus::SE3f{});
        out.superpointRawLeftCount = graphState.PatchCount();
        out.superpointMatchedStereoCount = graphState.EdgeCount();

        if (!voState.m_lkHavePrev) {
            voState.m_lkPrevLeft = leftRect.clone();
            voState.m_lkPrevRight = rightRect.clone();
            voState.m_lkTwc = Sophus::SE3f();
            voState.m_lkHavePrev = true;
            voState.m_lkFrameCount = 1;
            haveLastPose = true;
            lastPose = PoseFromTwc(voState.m_lkTwc);
            out.trackingState = ORB_SLAM3::Tracking::OK;
            out.poseValid = true;
            out.pose = lastPose;
            out.pose.valid = true;
            out.orbTrackMs = ElapsedMs(start, std::chrono::steady_clock::now());
            return out;
        }

        const auto disparityStart = std::chrono::steady_clock::now();
        cv::Mat disp;
        if (!voState.m_lkPerFrameSgbm) {
            const int numDisparities = std::max(16, ((leftRect.cols / 8 + 15) / 16) * 16);
            voState.m_lkPerFrameSgbm =
                cv::StereoSGBM::create(0, numDisparities, 5, 8 * 5 * 5, 32 * 5 * 5, 1, 31, 8, 60, 2,
                                       cv::StereoSGBM::MODE_SGBM_3WAY);
        }
        cv::Mat disp16;
        voState.m_lkPerFrameSgbm->compute(voState.m_lkPrevLeft, voState.m_lkPrevRight, disp16);
        disp16.convertTo(disp, CV_32F, 1.0 / 16.0);
        out.lkDisparityMs = ElapsedMs(disparityStart, std::chrono::steady_clock::now());

        const auto gfttStart = std::chrono::steady_clock::now();
        std::vector<cv::Point2f> rawPts0;
        cv::goodFeaturesToTrack(voState.m_lkPrevLeft, rawPts0, kLkGfttPerFrameMaxCorners,
                                kLkGfttQualityLevel, kLkGfttMinDistancePx, cv::Mat(),
                                kLkGfttBlockSize, false, kLkGfttHarrisK);
        std::vector<cv::Point2f> pts0 =
            SelectGfttPointsGridBalanced(rawPts0, voState.m_lkPrevLeft.size(), kLkGfttPerFrameMaxCorners,
                                         kLkGfttPerFrameMaxCornersPerCell);
        out.lkGfttMs = ElapsedMs(gfttStart, std::chrono::steady_clock::now());

        const auto flowStart = std::chrono::steady_clock::now();
        std::vector<cv::Point2f> pts1;
        std::vector<uint8_t> status;
        std::vector<float> err;
        if (!pts0.empty()) {
            cv::calcOpticalFlowPyrLK(voState.m_lkPrevLeft, leftRect, pts0, pts1, status, err,
                                     cv::Size(21, 21), 3);
        }
        out.lkFlowMs = ElapsedMs(flowStart, std::chrono::steady_clock::now());

        struct Candidate {
            cv::Point3f object;
            cv::Point2f image;
            cv::Point2f prev;
            float depth{0.0f};
        };
        std::vector<Candidate> candidates;
        candidates.reserve(pts0.size());
        const auto candidateStart = std::chrono::steady_clock::now();
        for (size_t i = 0; i < pts0.size() && i < pts1.size(); ++i) {
            if (i >= status.size() || !status[i]) {
                continue;
            }
            const cv::Point2f &p0 = pts0[i];
            const cv::Point2f &p1 = pts1[i];
            if (p0.x < 1.0f || p0.y < 1.0f || p0.x >= disp.cols - 1 || p0.y >= disp.rows - 1 ||
                p1.x < 1.0f || p1.y < 1.0f || p1.x >= leftRect.cols - 1 || p1.y >= leftRect.rows - 1) {
                continue;
            }
            if (cv::norm(p1 - p0) > kLkMaxFlowPx) {
                continue;
            }
            float d = 0.0f;
            if (!ReadConsistentDisparity(disp, p0, d)) {
                continue;
            }
            const float z = voState.m_lkFx * voState.m_lkBaseline / d;
            if (!(z >= kLkMinDepthMeters) || z > kLkMaxDepthMeters || !std::isfinite(z)) {
                continue;
            }
            candidates.push_back({cv::Point3f((p0.x - voState.m_lkCx) * z / voState.m_lkFx,
                                              (p0.y - voState.m_lkCy) * z / voState.m_lkFy, z),
                                  p1, p0, z});
        }

        std::array<int, kLkPerFramePnPSelectGridCols * kLkPerFramePnPSelectGridRows * kLkPerFramePnPDepthBins>
            bucketCounts{};
        std::vector<cv::Point3f> objectPoints;
        std::vector<cv::Point2f> imagePoints;
        objectPoints.reserve(candidates.size());
        imagePoints.reserve(candidates.size());
        for (const Candidate &candidate : candidates) {
            const int gx = std::clamp(static_cast<int>(candidate.prev.x * kLkPerFramePnPSelectGridCols /
                                                       std::max(1, voState.m_lkPrevLeft.cols)),
                                      0, kLkPerFramePnPSelectGridCols - 1);
            const int gy = std::clamp(static_cast<int>(candidate.prev.y * kLkPerFramePnPSelectGridRows /
                                                       std::max(1, voState.m_lkPrevLeft.rows)),
                                      0, kLkPerFramePnPSelectGridRows - 1);
            const int dz = LkPerFrameDepthBin(candidate.depth);
            const int bucket = ((gy * kLkPerFramePnPSelectGridCols) + gx) * kLkPerFramePnPDepthBins + dz;
            if (bucketCounts[static_cast<size_t>(bucket)] >= kLkPerFramePnPMaxPerGridDepthBin) {
                continue;
            }
            ++bucketCounts[static_cast<size_t>(bucket)];
            objectPoints.push_back(candidate.object);
            imagePoints.push_back(candidate.image);
        }
        out.lkCandidateMs = ElapsedMs(candidateStart, std::chrono::steady_clock::now());

        int inlierCount = 0;
        bool poseUpdated = false;
        const auto pnpStart = std::chrono::steady_clock::now();
        if (objectPoints.size() >= static_cast<size_t>(kLkMinPnPPoints)) {
            cv::Mat rvec;
            cv::Mat tvec;
            cv::Mat inliers;
            const cv::Mat K = MakeCameraMatrix(voState.m_lkFx, voState.m_lkFy, voState.m_lkCx, voState.m_lkCy);
            bool ok = false;
            try {
                ok = cv::solvePnPRansac(objectPoints, imagePoints, K, cv::Mat(), rvec, tvec, false,
                                        kLkPerFrameDefaultPnPIterations, kLkPerFramePnPReprojThresholdPx,
                                        kLkPerFrameDefaultPnPConfidence, inliers, cv::SOLVEPNP_EPNP);
                inlierCount = inliers.rows;
            } catch (const cv::Exception &e) {
                std::cerr << "[dpvo_trt_vo] solvePnPRansac skipped points=" << objectPoints.size()
                          << " error=" << e.what() << "\n";
            }
            if (ok && inlierCount >= kLkMinPnPInliers) {
                cv::Mat Rcv;
                cv::Rodrigues(rvec, Rcv);
                Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
                Eigen::Vector3f t = Eigen::Vector3f::Zero();
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        R(r, c) = static_cast<float>(Rcv.at<double>(r, c));
                    }
                    t(r) = static_cast<float>(tvec.at<double>(r, 0));
                }
                if (std::isfinite(t.norm()) && t.norm() <= kLkMaxStepMeters) {
                    const Sophus::SE3f TcurrPrev(Sophus::SO3f(R), t);
                    voState.m_lkTwc = voState.m_lkTwc * StabilizeLkCameraDelta(TcurrPrev.inverse());
                    poseUpdated = true;
                }
            }
        }
        out.lkPnpMs = ElapsedMs(pnpStart, std::chrono::steady_clock::now());
        out.frontendMs = out.lkDisparityMs + out.lkGfttMs + out.lkFlowMs + out.lkCandidateMs + out.lkPnpMs;
        out.matchesInliers = inlierCount;
        out.trackedMapPointCount = static_cast<uint32_t>(inlierCount);
        out.localMapPointCount = static_cast<uint32_t>(objectPoints.size());

        voState.m_lkPrevLeft = leftRect.clone();
        voState.m_lkPrevRight = rightRect.clone();
        ++voState.m_lkFrameCount;

        if (poseUpdated) {
            lastPose = PoseFromTwc(voState.m_lkTwc);
            haveLastPose = true;
            out.trackingState = ORB_SLAM3::Tracking::OK;
        } else {
            out.trackingState = haveLastPose ? ORB_SLAM3::Tracking::RECENTLY_LOST : ORB_SLAM3::Tracking::LOST;
        }

        out.poseValid = haveLastPose && TrackingStateCanPublishPose(out.trackingState);
        out.pose = lastPose;
        out.pose.valid = out.poseValid;
        if (extractFeatures) {
            out.leftFeatures = std::move(pts1);
        }
        out.orbTrackMs = ElapsedMs(start, std::chrono::steady_clock::now());
        return out;
    }

    DpvoTensorRtConfig config;
    TensorRtEngineHandle patchEngine;
    TensorRtEngineHandle updateEngine;
    CudaStreamHandle cudaStream;
    DpvoPatchifierRuntime patchifierRuntime;
    DpvoUpdateRuntime updateRuntime;
    DpvoGraphState graphState;
    SlamModeSharedState voState;
    cv::Mat resizedGray;
    core::ports::PoseEstimate lastPose{};
    bool haveLastPose{false};
    bool running{false};
    bool loggedPatchifierShape{false};
    bool loggedPatchifierError{false};
};

#else

struct DpvoTensorRtEngine::Impl {
    explicit Impl(DpvoTensorRtConfig cfg) : config(std::move(cfg)) {}

    bool Start()
    {
        std::cerr << "[dpvo_trt] native TensorRT backend was not compiled into this target\n";
        return false;
    }

    void Stop() {}

    core::ports::SlamOutput Process(const core::ports::SlamInputBatch &input, bool, bool)
    {
        core::ports::SlamOutput out{};
        out.frameId = input.frameId;
        out.captureTimestampNs = input.captureTimestampNs;
        out.trackingState = ORB_SLAM3::Tracking::LOST;
        return out;
    }

    DpvoTensorRtConfig config;
};

#endif

DpvoTensorRtEngine::DpvoTensorRtEngine(DpvoTensorRtConfig config) : m_impl(std::make_unique<Impl>(std::move(config))) {}
DpvoTensorRtEngine::~DpvoTensorRtEngine() = default;

bool DpvoTensorRtEngine::Start()
{
    return m_impl != nullptr && m_impl->Start();
}

void DpvoTensorRtEngine::Stop()
{
    if (m_impl != nullptr) {
        m_impl->Stop();
    }
}

core::ports::SlamOutput DpvoTensorRtEngine::Process(const core::ports::SlamInputBatch &input, bool extractFeatures,
                                                    bool extractPointCloud)
{
    return m_impl != nullptr ? m_impl->Process(input, extractFeatures, extractPointCloud) : core::ports::SlamOutput{};
}

DpvoTensorRtConfig MakeDpvoTensorRtConfig(const RuntimeConfig &runtime, const std::string &settingsPath)
{
    DpvoTensorRtConfig out{};
    out.repoPath = runtime.dpvoRepo;
    out.patchEnginePath = runtime.dpvoPatchEngine;
    out.updateEnginePath = runtime.dpvoUpdateEngine;
    out.settingsPath = settingsPath;
    out.inputWidth = std::clamp(runtime.dpvoInputWidth, 160, 1280);
    out.inputHeight = std::clamp(runtime.dpvoInputHeight, 120, 960);
    out.patchesPerFrame = std::clamp(runtime.dpvoPatchesPerFrame, 16, 256);
    out.optimizationWindow = std::clamp(runtime.dpvoOptimizationWindow, 4, 32);
    return out;
}

} // namespace smartdrone::adapters::slam
