#include "adapters/slam/dpvo_tensorrt_engine.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "Tracking.h"

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

  private:
    TensorRtLogger m_logger{};
    std::unique_ptr<nvinfer1::IRuntime, DestroyRuntime> m_runtime;
    std::unique_ptr<nvinfer1::ICudaEngine, DestroyEngine> m_engine;
    std::unique_ptr<nvinfer1::IExecutionContext, DestroyContext> m_context;
    std::string m_path;
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
        running = true;
        std::cerr << "[dpvo_trt] ready patch_engine=" << patchEngine.Path()
                  << " update_engine=" << updateEngine.Path()
                  << " input=" << config.inputWidth << "x" << config.inputHeight
                  << " patches=" << config.patchesPerFrame
                  << " opt_window=" << config.optimizationWindow << "\n";
        return true;
    }

    void Stop() { running = false; }

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

        cv::Mat leftGray = input.stereo.left.gray;
        if (leftGray.channels() != 1) {
            cv::cvtColor(leftGray, leftGray, cv::COLOR_BGR2GRAY);
        }
        if (leftGray.empty()) {
            out.trackingState = ORB_SLAM3::Tracking::LOST;
            return out;
        }
        if (leftGray.cols != config.inputWidth || leftGray.rows != config.inputHeight) {
            cv::resize(leftGray, resizedGray, cv::Size(config.inputWidth, config.inputHeight), 0.0, 0.0,
                       cv::INTER_AREA);
        } else {
            resizedGray = leftGray;
        }

        // The TensorRT engines are loaded and the runtime path is selected here. The full DPVO state machine still
        // needs its native CUDA correlation/BA kernels wired before publishing pose; fail closed instead of emitting
        // placeholder motion.
        out.trackingState = ORB_SLAM3::Tracking::RECENTLY_LOST;
        out.poseValid = haveLastPose;
        out.pose = lastPose;
        out.pose.valid = haveLastPose;
        out.orbTrackMs = ElapsedMs(start, std::chrono::steady_clock::now());
        if (extractFeatures) {
            out.leftFeatures.clear();
            out.rightFeatures.clear();
        }
        return out;
    }

    DpvoTensorRtConfig config;
    TensorRtEngineHandle patchEngine;
    TensorRtEngineHandle updateEngine;
    cv::Mat resizedGray;
    core::ports::PoseEstimate lastPose{};
    bool haveLastPose{false};
    bool running{false};
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

DpvoTensorRtConfig MakeDpvoTensorRtConfig(const RuntimeConfig &runtime)
{
    DpvoTensorRtConfig out{};
    out.repoPath = runtime.dpvoRepo;
    out.patchEnginePath = runtime.dpvoPatchEngine;
    out.updateEnginePath = runtime.dpvoUpdateEngine;
    out.inputWidth = std::clamp(runtime.dpvoInputWidth, 160, 1280);
    out.inputHeight = std::clamp(runtime.dpvoInputHeight, 120, 960);
    out.patchesPerFrame = std::clamp(runtime.dpvoPatchesPerFrame, 16, 256);
    out.optimizationWindow = std::clamp(runtime.dpvoOptimizationWindow, 4, 32);
    return out;
}

} // namespace smartdrone::adapters::slam
