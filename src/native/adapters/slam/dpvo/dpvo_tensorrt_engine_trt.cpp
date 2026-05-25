#include "adapters/slam/dpvo/dpvo_tensorrt_engine.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <sophus/se3.hpp>

#include "adapters/slam/engine/slam_engine_factory.h"
#include "adapters/slam/engine/slam_env.h"
#include "adapters/slam/engine/slam_image_utils.h"
#include "adapters/slam/engine/slam_mode_state.h"
#include "adapters/slam/engine/slam_pose_utils.h"
#include "adapters/slam/dpvo/dpvo_runtime_options.h"
#include "core/ports/slam_tracking_state.h"
#include "adapters/slam/dpvo/dpvo_tensorrt_engine_cuda_runtime.h"
#include "adapters/slam/dpvo/dpvo_tensorrt_engine_graph_state.h"
#include "adapters/slam/dpvo/dpvo_tensorrt_engine_native_solver.h"
#include "adapters/slam/dpvo/dpvo_tensorrt_engine_patchifier_run.h"

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <dlfcn.h>
#include <nvrtc.h>

namespace SmartDrone::Adapters::Slam {

namespace {

std::filesystem::path ResolveEnginePath(const std::string &explicitPath,
                                        const std::string &repoPath,
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

int64_t SteadyNowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

} // namespace

namespace DpvoTensorRtInternal {

#include "dpvo_tensorrt_engine_trt_handles.h"
#include "dpvo_tensorrt_engine_bindings.h"
#include "dpvo_tensorrt_engine_patchifier_runtime.h"
#include "dpvo_tensorrt_engine_update_runtime.h"
#include "dpvo_tensorrt_engine_update_preagg_runtime.h"
#include "dpvo_tensorrt_engine_update_postagg_runtime.h"

} // namespace DpvoTensorRtInternal

#include "dpvo_tensorrt_engine_impl_trt.h"

namespace DpvoTensorRtInternal {

bool TensorRtEngineHandle::Load(const std::filesystem::path &enginePath,
                                const char *name, std::string *err)
{
    std::vector<char> bytes;
    if (!ReadEngineBytes(enginePath, name, err, bytes)) {
        return false;
    }
    if (!CreateRuntime(name, err) ||
        !DeserializeEngine(enginePath, name, bytes, err) ||
        !CreateContext(name, err)) {
        return false;
    }
    m_path = enginePath.string();
    return true;
}

bool TensorRtEngineHandle::ReadEngineBytes(
    const std::filesystem::path &enginePath, const char *name,
    std::string *err, std::vector<char> &bytes)
{
    std::ifstream in(enginePath, std::ios::binary);
    if (!in) {
        SetError(err, std::string(name) + " TensorRT engine not found: " +
                          enginePath.string());
        return false;
    }
    bytes.assign(std::istreambuf_iterator<char>(in),
                 std::istreambuf_iterator<char>());
    if (bytes.empty()) {
        SetError(err, std::string(name) + " TensorRT engine is empty: " +
                          enginePath.string());
        return false;
    }
    return true;
}

bool TensorRtEngineHandle::CreateRuntime(const char *name, std::string *err)
{
    initLibNvInferPlugins(&m_logger, "");
    m_runtime.reset(nvinfer1::createInferRuntime(m_logger));
    if (!m_runtime) {
        SetError(err,
                 std::string("failed to create ") + name + " TensorRT runtime");
        return false;
    }
    return true;
}

bool TensorRtEngineHandle::DeserializeEngine(
    const std::filesystem::path &enginePath, const char *name,
    const std::vector<char> &bytes, std::string *err)
{
    m_engine.reset(
        m_runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!m_engine) {
        SetError(err, std::string("failed to deserialize ") + name +
                          " TensorRT engine: " + enginePath.string());
        return false;
    }
    return true;
}

bool TensorRtEngineHandle::CreateContext(const char *name, std::string *err)
{
    m_context.reset(m_engine->createExecutionContext());
    if (!m_context) {
        SetError(err, std::string("failed to create ") + name +
                          " TensorRT execution context");
        return false;
    }
    return true;
}

void TensorRtEngineHandle::SetError(std::string *err, std::string message)
{
    if (err != nullptr) {
        *err = std::move(message);
    }
}

bool CudaStreamHandle::Create(std::string *err)
{
    if (stream != nullptr) {
        return true;
    }
    const cudaError_t rc = cudaStreamCreate(&stream);
    if (rc != cudaSuccess) {
        if (err != nullptr) {
            *err = std::string("cudaStreamCreate failed: ") +
                   cudaGetErrorString(rc);
        }
        stream = nullptr;
        return false;
    }
    return true;
}

void CudaStreamHandle::Reset()
{
    if (stream != nullptr) {
        cudaStreamDestroy(stream);
        stream = nullptr;
    }
}

CudaStreamHandle::~CudaStreamHandle()
{
    Reset();
}

} // namespace DpvoTensorRtInternal

DpvoTensorRtEngine::Impl::DpvoEnginePaths
DpvoTensorRtEngine::Impl::ResolveStartEnginePaths() const
{
    return {
        ResolveEnginePath(config.patchEnginePath, config.repoPath,
                          {"dpvo_patchifier_fp16.engine",
                           "dpvo_patchifier.engine", "dpvo_patch.engine"}),
        ResolveEnginePath(config.updateEnginePath, config.repoPath,
                          {"dpvo_update_fp16.engine", "dpvo_update.engine"}),
        ResolveEnginePath(
            std::string{}, config.repoPath,
            {"dpvo_update_preagg_fp16.engine", "dpvo_update_preagg.engine"}),
        ResolveEnginePath(
            std::string{}, config.repoPath,
            {"dpvo_update_postagg_fp16.engine",
             "dpvo_update_postagg.engine"})};
}

bool DpvoTensorRtEngine::Impl::CheckRequiredEnginePaths(
    const DpvoEnginePaths &paths) const
{
    if (!paths.patch.empty() && !paths.update.empty()) {
        return true;
    }
    std::cerr << "[dpvo_trt] missing engine(s): patch='"
              << config.patchEnginePath << "' update='"
              << config.updateEnginePath << "' repo='" << config.repoPath
              << "'\n";
    return false;
}

bool DpvoTensorRtEngine::Impl::LoadBaseEngines(
    const DpvoEnginePaths &paths, std::string &err)
{
    if (!patchEngine.Load(paths.patch, "DPVO patchifier", &err)) {
        std::cerr << "[dpvo_trt] " << err << "\n";
        return false;
    }
    if (!updateEngine.Load(paths.update, "DPVO update", &err)) {
        std::cerr << "[dpvo_trt] " << err << "\n";
        return false;
    }
    return true;
}

bool DpvoTensorRtEngine::Impl::LoadSplitSoftAggEngines(
    const DpvoEnginePaths &paths, std::string &err)
{
    softAggSplitReady = false;
    if (!paths.preAgg.empty() && !paths.postAgg.empty()) {
        if (!updatePreAggEngine.Load(paths.preAgg, "DPVO update preagg",
                                     &err)) {
            std::cerr << "[dpvo_trt] " << err << "\n";
            return false;
        }
        if (!updatePostAggEngine.Load(paths.postAgg, "DPVO update postagg",
                                      &err)) {
            std::cerr << "[dpvo_trt] " << err << "\n";
            return false;
        }
        softAggSplitReady = true;
        return true;
    }
    std::cerr << "[dpvo_trt] split SoftAgg engines not found under repo='"
              << config.repoPath
              << "'; using compatibility update warmup only\n";
    return true;
}

bool DpvoTensorRtEngine::Impl::InitializeCudaSupport(std::string &err)
{
    if (!cudaStream.Create(&err)) {
        std::cerr << "[dpvo_trt] " << err << "\n";
        return false;
    }
    cudaKernelReady = false;
    std::string cudaKernelErr;
    if (cudaKernelRuntime.Initialize(cudaStream.stream, &cudaKernelErr)) {
        cudaKernelReady = true;
        std::cerr << "[dpvo_cuda] native CUDA kernels ready smoke_expected="
                  << cudaKernelRuntime.SmokeExpected()
                  << " smoke_got=" << cudaKernelRuntime.SmokeGot() << "\n";
    } else {
        std::cerr << "[dpvo_cuda] native CUDA kernels unavailable: "
                  << cudaKernelErr << "\n";
    }
    return true;
}

bool DpvoTensorRtEngine::Impl::InitializeTensorRtRuntimes(
    std::string &err)
{
    if (!patchifierRuntime.Initialize(patchEngine, config.inputWidth,
                                      config.inputHeight, &err)) {
        std::cerr << "[dpvo_trt] " << err << "\n";
        return false;
    }
    if (!patchifierRightRuntime.Initialize(patchEngine, config.inputWidth,
                                           config.inputHeight, &err)) {
        std::cerr << "[dpvo_trt] right " << err << "\n";
        return false;
    }
    if (!updateRuntime.Initialize(updateEngine, &err)) {
        std::cerr << "[dpvo_trt] " << err << "\n";
        return false;
    }
    if (softAggSplitReady) {
        if (!updatePreAggRuntime.Initialize(updatePreAggEngine, &err)) {
            std::cerr << "[dpvo_trt] " << err << "\n";
            return false;
        }
        if (!updatePostAggRuntime.Initialize(updatePostAggEngine, &err)) {
            std::cerr << "[dpvo_trt] " << err << "\n";
            return false;
        }
    }
    return true;
}

bool DpvoTensorRtEngine::Impl::WarmupTensorRtRuntimes(
    DpvoStartWarmupRuns &warmups, std::string &err)
{
    graphState.Reset(config.patchesPerFrame, config.optimizationWindow);
    warmups.update =
        updateRuntime.Warmup(updateEngine, cudaStream.stream,
                             std::max(1, config.patchesPerFrame), &err);
    if (!warmups.update.ok) {
        std::cerr << "[dpvo_trt] update warmup failed: " << err << "\n";
        return false;
    }
    if (!softAggSplitReady) {
        return true;
    }
    warmups.preAgg =
        updatePreAggRuntime.Warmup(updatePreAggEngine, cudaStream.stream,
                                   std::max(1, config.patchesPerFrame), &err);
    if (!warmups.preAgg.ok) {
        std::cerr << "[dpvo_trt] update-preagg warmup failed: " << err << "\n";
        return false;
    }
    warmups.postAgg = updatePostAggRuntime.Warmup(
        updatePostAggEngine, cudaStream.stream,
        std::max(1, config.patchesPerFrame), &err);
    if (!warmups.postAgg.ok) {
        std::cerr << "[dpvo_trt] update-postagg warmup failed: " << err << "\n";
        return false;
    }
    return true;
}

void DpvoTensorRtEngine::Impl::ResetStartState()
{
    nativeSolver.Reset();
    haveLastPose = false;
    lastPose = Core::Ports::PoseEstimate{};
    loggedKeyframeRemovals = 0;
    processedFrameCount = 0;
}

void DpvoTensorRtEngine::Impl::ConfigurePacing()
{
    const int defaultHeavyEveryN =
        EnvFlagEnabled("SMART_DRONE_DPVO_EPG_PACING", false) ? 3 : 1;
    heavyEveryN =
        std::clamp(EnvIntValue("SMART_DRONE_DPVO_EPG_HEAVY_EVERY_N",
                               EnvIntValue("SMART_DRONE_DPVO_HEAVY_EVERY_N",
                                           defaultHeavyEveryN)),
                   1, 30);
    rightEveryN = std::clamp(
        EnvIntValue("SMART_DRONE_DPVO_RIGHT_EVERY_N", heavyEveryN), 1, 30);
    heavyIntervalMs = std::clamp(
        EnvIntValue("SMART_DRONE_DPVO_EPG_HEAVY_INTERVAL_MS", 0), 0, 1000);
    warmupFullFrames =
        std::clamp(EnvIntValue("SMART_DRONE_DPVO_WARMUP_FULL_FRAMES",
                               std::max(8, config.optimizationWindow + 1)),
                   1, 60);
    loggedEpgPacing = false;
    lastHeavyUpdateEndNs.store(0, std::memory_order_relaxed);
}

void DpvoTensorRtEngine::Impl::LogReady(
    const DpvoStartWarmupRuns &warmups) const
{
    std::cerr << "[dpvo_trt] ready patch_engine=" << patchEngine.Path()
              << " update_engine=" << updateEngine.Path()
              << " update_preagg_engine="
              << (softAggSplitReady ? updatePreAggEngine.Path()
                                    : std::string{"none"})
              << " update_postagg_engine="
              << (softAggSplitReady ? updatePostAggEngine.Path()
                                    : std::string{"none"})
              << " input=" << config.inputWidth << "x" << config.inputHeight
              << " patches=" << config.patchesPerFrame
              << " opt_window=" << config.optimizationWindow
              << " update_warmup_ms=" << warmups.update.elapsedMs
              << " preagg_warmup_ms=" << warmups.preAgg.elapsedMs
              << " postagg_warmup_ms=" << warmups.postAgg.elapsedMs
              << " native_cuda_kernels=" << (cudaKernelReady ? 1 : 0)
              << " native_dpvo=1\n";
}

void DpvoTensorRtEngine::Impl::LogPacingOnce()
{
    if (!loggedEpgPacing) {
        std::cerr << "[dpvo_trt] epg pacing heavy_every_n=" << heavyEveryN
                  << " heavy_interval_ms=" << heavyIntervalMs
                  << " right_every_n=" << rightEveryN
                  << " warmup_full_frames=" << warmupFullFrames << "\n";
        loggedEpgPacing = true;
    }
}

bool DpvoTensorRtEngine::Impl::LoadCalibration()
{
    if (!voState.LoadStereoCalibration(config.settingsPath)) {
        std::cerr << "[dpvo_trt] DPVO calibration unavailable settings='"
                  << config.settingsPath << "'; pose output disabled\n";
        running.store(false, std::memory_order_release);
        return false;
    }
    voState.ResetTrackingState();
    return true;
}

bool DpvoTensorRtEngine::Impl::Start()
{
    const DpvoEnginePaths paths = ResolveStartEnginePaths();
    if (!CheckRequiredEnginePaths(paths)) {
        return false;
    }
    std::string err;
    DpvoStartWarmupRuns warmups;
    if (!LoadBaseEngines(paths, err) || !LoadSplitSoftAggEngines(paths, err) ||
        !InitializeCudaSupport(err) || !InitializeTensorRtRuntimes(err) ||
        !WarmupTensorRtRuntimes(warmups, err)) {
        return false;
    }
    ResetStartState();
    ConfigurePacing();
    running.store(true, std::memory_order_release);
    LogReady(warmups);
    LogPacingOnce();
    return LoadCalibration();
}

Core::Ports::SlamOutput DpvoTensorRtEngine::Impl::BuildBaseOutput(
    const Core::Ports::SlamInputBatch &input) const
{
    Core::Ports::SlamOutput out{};
    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    out.mapId = 1;
    out.usedVisualFeatureFrontend = false;
    return out;
}

void DpvoTensorRtEngine::Impl::ResizeFrameInput(const cv::Mat &input,
                                                cv::Mat &output)
{
    if (input.cols == config.inputWidth && input.rows == config.inputHeight) {
        output = input;
        return;
    }
    cv::resize(input, output, cv::Size(config.inputWidth, config.inputHeight),
               0.0, 0.0, cv::INTER_AREA);
}

DpvoTensorRtEngine::Impl::DpvoFramePreparation
DpvoTensorRtEngine::Impl::PrepareFrameInput(
    const Core::Ports::SlamInputBatch &input,
    Core::Ports::SlamOutput &out)
{
    DpvoFramePreparation preparation{};
    const auto prepareStart = std::chrono::steady_clock::now();
    cv::Mat leftGray = EnsureGray8(input.stereo.left.gray);
    cv::Mat rightGray = EnsureGray8(input.stereo.right.gray);
    if (leftGray.empty() || rightGray.empty()) {
        out.trackingState = Core::Ports::SLAM_TRACKING_LOST;
        return preparation;
    }
    cv::Mat leftRect = leftGray;
    cv::Mat rightRect = rightGray;
    voState.EnsureStereoRectifier(leftGray.size());
    if (!voState.m_lkMap1x.empty() && !voState.m_lkMap2x.empty()) {
        const auto rectifyStart = std::chrono::steady_clock::now();
        cv::remap(leftGray, leftRect, voState.m_lkMap1x, voState.m_lkMap1y,
                  cv::INTER_LINEAR);
        cv::remap(rightGray, rightRect, voState.m_lkMap2x, voState.m_lkMap2y,
                  cv::INTER_LINEAR);
        out.lkRectifyMs =
            ElapsedMs(rectifyStart, std::chrono::steady_clock::now());
    }
    out.inputPrepareMs =
        ElapsedMs(prepareStart, std::chrono::steady_clock::now());
    preparation.scaleX =
        static_cast<float>(config.inputWidth) / std::max(1, leftRect.cols);
    preparation.scaleY =
        static_cast<float>(config.inputHeight) / std::max(1, leftRect.rows);
    ResizeFrameInput(leftRect, resizedGray);
    ResizeFrameInput(rightRect, resizedRightGray);
    preparation.valid = true;
    return preparation;
}

DpvoTensorRtEngine::Impl::DpvoFrameSchedule
DpvoTensorRtEngine::Impl::ResolveFrameSchedule()
{
    ++processedFrameCount;
    const bool warmupFrame =
        !nativeSolver.HasPose() || graphState.FrameCount() < warmupFullFrames;
    const int64_t lastUpdateNs =
        lastHeavyUpdateEndNs.load(std::memory_order_acquire);
    const int64_t nowNs = SteadyNowNs();
    const bool intervalReady =
        heavyIntervalMs <= 0 || lastUpdateNs == 0 ||
        nowNs - lastUpdateNs >=
            static_cast<int64_t>(heavyIntervalMs) * 1000000LL;
    const bool cadenceReady =
        heavyEveryN <= 1 ||
        ((processedFrameCount - 1U) % static_cast<uint64_t>(heavyEveryN)) == 0U;
    DpvoFrameSchedule schedule{};
    schedule.heavyFrame = warmupFrame || (intervalReady && cadenceReady);
    schedule.rightFrame =
        schedule.heavyFrame &&
        (warmupFrame || rightEveryN <= 1 ||
         ((processedFrameCount - 1U) % static_cast<uint64_t>(rightEveryN)) == 0U);
    return schedule;
}

Core::Ports::SlamOutput
DpvoTensorRtEngine::Impl::BuildPacedFrameOutput(
    const std::chrono::steady_clock::time_point &start)
{
    Core::Ports::SlamOutput out{};
    out.visualFeatureRawLeftCount = graphState.PatchCount();
    out.visualFeatureRawRightCount = graphState.LastStereoDepthUpdates();
    out.visualFeatureMatchedStereoCount = graphState.EdgeCount();
    out.matchesInliers = graphState.EdgeCount();
    out.trackedMapPointCount = static_cast<uint32_t>(graphState.EdgeCount());
    out.localMapPointCount = static_cast<uint32_t>(graphState.PatchCount());
    out.trackingState = Core::Ports::SLAM_TRACKING_OK;
    out.poseValid = true;
    out.pose = lastPose;
    out.pose.valid = true;
    out.orbTrackMs = ElapsedMs(start, std::chrono::steady_clock::now());
    return out;
}

DpvoTensorRtEngine::Impl::DpvoPatchifierFrameRuns
DpvoTensorRtEngine::Impl::RunPatchifierFrame(bool rightFrame)
{
    DpvoPatchifierFrameRuns runs{};
    runs.rightRequested = rightFrame;
    runs.left = patchifierRuntime.Run(
        {resizedGray, patchEngine, cudaStream.stream, true, true,
         &runs.leftError});
    runs.right =
        rightFrame ? patchifierRightRuntime.Run({resizedRightGray, patchEngine,
                                                 cudaStream.stream, true, false,
                                                 &runs.rightError})
                   : DpvoPatchifierRun{};
    return runs;
}

DpvoIntrinsics DpvoTensorRtEngine::Impl::MakeFrameIntrinsics(
    const DpvoFramePreparation &preparation) const
{
    return {voState.m_lkFx * preparation.scaleX * 0.25F,
            voState.m_lkFy * preparation.scaleY * 0.25F,
            voState.m_lkCx * preparation.scaleX * 0.25F,
            voState.m_lkCy * preparation.scaleY * 0.25F};
}

void DpvoTensorRtEngine::Impl::LogPatchifierShape(
    const DpvoPatchifierRun &patchRun)
{
    if (loggedPatchifierShape) {
        return;
    }
    std::cerr << "[dpvo_trt] patchifier active fmap="
              << DimsToString(patchRun.fmapDims)
              << " imap=" << DimsToString(patchRun.imapDims)
              << " ms=" << patchRun.elapsedMs << "\n";
    loggedPatchifierShape = true;
}

void DpvoTensorRtEngine::Impl::LogRightPatchifierError(
    const DpvoPatchifierFrameRuns &runs)
{
    if (!runs.rightRequested || runs.right.ok || loggedRightPatchifierError) {
        return;
    }
    std::cerr << "[dpvo_trt] right patchifier inference disabled for this frame: "
              << runs.rightError << "\n";
    loggedRightPatchifierError = true;
}

void DpvoTensorRtEngine::Impl::ApplyRightPatchifierDepth(
    const DpvoPatchifierRun &rightPatchRun, const DpvoIntrinsics &intrinsics)
{
    if (!rightPatchRun.ok) {
        return;
    }
    graphState.ApplyStereoDepthFromRightFmap(rightPatchRun, intrinsics.fx,
                                             voState.m_lkBaseline);
    if (!loggedStereoDepthInit && graphState.LastStereoDepthUpdates() > 0) {
        std::cerr << "[dpvo_trt] stereo fmap depth init updates="
                  << graphState.LastStereoDepthUpdates()
                  << " fx_feature=" << intrinsics.fx
                  << " baseline=" << voState.m_lkBaseline << "\n";
        loggedStereoDepthInit = true;
    }
}

void DpvoTensorRtEngine::Impl::ApplyPatchifierRuns(
    const DpvoPatchifierFrameRuns &runs, const DpvoIntrinsics &intrinsics,
    const Core::Ports::SlamInputBatch &input, Core::Ports::SlamOutput &out)
{
    if (runs.left.ok) {
        out.visualFeatureForwardMs = runs.left.elapsedMs;
        out.visualFeatureStereoMatchMs =
            runs.right.ok ? runs.right.elapsedMs : 0.0;
        LogPatchifierShape(runs.left);
    } else if (!loggedPatchifierError) {
        std::cerr << "[dpvo_trt] patchifier inference disabled for this frame: "
                  << runs.leftError << "\n";
        loggedPatchifierError = true;
    }
    LogRightPatchifierError(runs);
    graphState.PushFrame(input.frameId, input.captureTimestampNs, resizedGray,
                         nativeSolver.HasPose() ? nativeSolver.LastTcw()
                                                : Sophus::SE3f{},
                         runs.left);
    ApplyRightPatchifierDepth(runs.right, intrinsics);
}

DpvoTensorRtEngine::Impl::DpvoNativeStepResult
DpvoTensorRtEngine::Impl::RunNativeStep(
    const DpvoPatchifierRun &patchRun, const DpvoIntrinsics &intrinsics)
{
    DpvoNativeStepResult result{};
    const DpvoNativeSolverStepRequest request{
        graphState,
        updatePreAggRuntime,
        updatePreAggEngine,
        updatePostAggRuntime,
        updatePostAggEngine,
        cudaKernelReady ? &cudaKernelRuntime : nullptr,
        cudaStream.stream,
        intrinsics,
        &result.nativeUpdateMs,
        &result.error};
    result.poseUpdated =
        patchRun.ok && softAggSplitReady && nativeSolver.Step(request);
    lastHeavyUpdateEndNs.store(SteadyNowNs(), std::memory_order_release);
    return result;
}

void DpvoTensorRtEngine::Impl::PopulateGraphCounts(
    Core::Ports::SlamOutput &out) const
{
    out.visualFeatureRawLeftCount = graphState.PatchCount();
    out.visualFeatureRawRightCount = graphState.LastStereoDepthUpdates();
    out.visualFeatureMatchedStereoCount = graphState.EdgeCount();
    out.matchesInliers = graphState.EdgeCount();
    out.trackedMapPointCount = static_cast<uint32_t>(graphState.EdgeCount());
    out.localMapPointCount = static_cast<uint32_t>(graphState.PatchCount());
}

void DpvoTensorRtEngine::Impl::LogKeyframeRemovals(
    int keyframeRemovalsBefore)
{
    if (graphState.KeyframeRemovals() == keyframeRemovalsBefore ||
        graphState.KeyframeRemovals() == loggedKeyframeRemovals) {
        return;
    }
    loggedKeyframeRemovals = graphState.KeyframeRemovals();
    std::cerr << "[dpvo_trt] keyframe removal count=" << loggedKeyframeRemovals
              << " active_edges=" << graphState.EdgeCount()
              << " active_frames=" << graphState.FrameCount() << "\n";
}

void DpvoTensorRtEngine::Impl::UpdateLastPose(bool poseUpdated)
{
    if (!poseUpdated && !nativeSolver.HasPose() && graphState.FrameCount() == 0) {
        return;
    }
    const Sophus::SE3f publishTcw =
        nativeSolver.HasPose() ? nativeSolver.LastTcw() : Sophus::SE3f{};
    lastPose = PoseFromTwc(publishTcw.inverse());
    haveLastPose = true;
}

void DpvoTensorRtEngine::Impl::ResolveTrackingState(
    const DpvoNativeStepResult &step, Core::Ports::SlamOutput &out)
{
    if (step.poseUpdated) {
        out.trackingState = Core::Ports::SLAM_TRACKING_OK;
        return;
    }
    if (!step.error.empty() && !loggedNativeSolverWait) {
        std::cerr << "[dpvo_trt] native solver waiting: " << step.error << "\n";
        loggedNativeSolverWait = true;
    }
    out.trackingState = haveLastPose ? Core::Ports::SLAM_TRACKING_RECENTLY_LOST
                                     : Core::Ports::SLAM_TRACKING_LOST;
}

void DpvoTensorRtEngine::Impl::ApplyNativeStepResult(
    const DpvoNativeStepResult &step, Core::Ports::SlamOutput &out)
{
    out.lkUpdateMs = step.nativeUpdateMs;
    out.visualFeatureFrontendMs = out.visualFeatureForwardMs +
                                  out.visualFeatureStereoMatchMs +
                                  step.nativeUpdateMs;
    out.frontendMs = out.visualFeatureFrontendMs;
    out.matchesInliers = graphState.EdgeCount();
    out.trackedMapPointCount = static_cast<uint32_t>(graphState.EdgeCount());
    out.localMapPointCount = static_cast<uint32_t>(graphState.PatchCount());
    UpdateLastPose(step.poseUpdated);
    ResolveTrackingState(step, out);
}

void DpvoTensorRtEngine::Impl::PopulateFeatureOutput(
    bool extractFeatures, float scaleX, float scaleY,
    Core::Ports::SlamOutput &out)
{
    if (!extractFeatures) {
        return;
    }
    const DpvoFrameState *newest = graphState.NewestFrame();
    if (newest == nullptr) {
        return;
    }
    out.leftFeatures.reserve(newest->patches.size());
    for (const DpvoPatchState &patch : newest->patches) {
        out.leftFeatures.emplace_back(patch.x * 4.0F / std::max(scaleX, 1e-6F),
                                      patch.y * 4.0F / std::max(scaleY, 1e-6F));
    }
}

Core::Ports::SlamOutput
DpvoTensorRtEngine::Impl::FinishFrameOutput(
    const std::chrono::steady_clock::time_point &start, bool extractFeatures,
    const DpvoFramePreparation &preparation, Core::Ports::SlamOutput &out)
{
    out.poseValid =
        haveLastPose && TrackingStateCanPublishPose(out.trackingState);
    out.pose = lastPose;
    out.pose.valid = out.poseValid;
    PopulateFeatureOutput(extractFeatures, preparation.scaleX,
                          preparation.scaleY, out);
    out.orbTrackMs = ElapsedMs(start, std::chrono::steady_clock::now());
    return out;
}

Core::Ports::SlamOutput
DpvoTensorRtEngine::Impl::Process(const Core::Ports::SlamInputBatch &input,
                                  bool extractFeatures, bool extractPointCloud)
{
    (void)extractPointCloud;
    const auto start = std::chrono::steady_clock::now();
    Core::Ports::SlamOutput out = BuildBaseOutput(input);
    if (!running.load(std::memory_order_acquire)) {
        out.trackingState = Core::Ports::SLAM_TRACKING_LOST;
        return out;
    }
    const DpvoFramePreparation preparation = PrepareFrameInput(input, out);
    if (!preparation.valid) {
        return out;
    }
    const DpvoFrameSchedule schedule = ResolveFrameSchedule();
    if (!schedule.heavyFrame && haveLastPose) {
        Core::Ports::SlamOutput paced = BuildPacedFrameOutput(start);
        paced.frameId = out.frameId;
        paced.captureTimestampNs = out.captureTimestampNs;
        paced.mapId = out.mapId;
        paced.usedVisualFeatureFrontend = out.usedVisualFeatureFrontend;
        return paced;
    }
    const DpvoIntrinsics intrinsics = MakeFrameIntrinsics(preparation);
    const DpvoPatchifierFrameRuns runs = RunPatchifierFrame(schedule.rightFrame);
    ApplyPatchifierRuns(runs, intrinsics, input, out);
    const int keyframeRemovalsBefore = graphState.KeyframeRemovals();
    PopulateGraphCounts(out);
    const DpvoNativeStepResult step = RunNativeStep(runs.left, intrinsics);
    ApplyNativeStepResult(step, out);
    LogKeyframeRemovals(keyframeRemovalsBefore);
    return FinishFrameOutput(start, extractFeatures, preparation, out);
}


DpvoTensorRtEngine::DpvoTensorRtEngine(DpvoTensorRtConfig config)
    : m_impl(std::make_unique<Impl>(std::move(config)))
{
}
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

Core::Ports::SlamOutput
DpvoTensorRtEngine::Process(const Core::Ports::SlamInputBatch &input,
                            bool extractFeatures, bool extractPointCloud)
{
    return m_impl != nullptr
               ? m_impl->Process(input, extractFeatures, extractPointCloud)
               : Core::Ports::SlamOutput{};
}

namespace {

ControlledSlamEngine
CreateDpvoTensorRtSlamEngine(const SlamEngineFactoryConfig &config)
{
    DpvoTensorRtConfig dpvoConfig =
        MakeDpvoTensorRtConfig(config.dpvoRuntime, config.settingsPath);
    ControlledSlamEngine out{};
    out.engine = std::make_unique<DpvoTensorRtEngine>(std::move(dpvoConfig));
    return out;
}

const SlamEngineFactoryRegistrar
    DPVO_TENSORRT_SLAM_ENGINE_REGISTRAR(SlamBackend::DpvoTensorRt,
                                     CreateDpvoTensorRtSlamEngine);

} // namespace

} // namespace SmartDrone::Adapters::Slam
