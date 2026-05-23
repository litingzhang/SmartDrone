using DpvoTensorRtInternal::CudaStreamHandle;
using DpvoTensorRtInternal::DimsToString;
using DpvoTensorRtInternal::DpvoCudaKernelRuntime;
using DpvoTensorRtInternal::DpvoFrameState;
using DpvoTensorRtInternal::DpvoGraphState;
using DpvoTensorRtInternal::DpvoIntrinsics;
using DpvoTensorRtInternal::DpvoNativeSolver;
using DpvoTensorRtInternal::DpvoPatchifierRun;
using DpvoTensorRtInternal::DpvoPatchifierRuntime;
using DpvoTensorRtInternal::DpvoPatchState;
using DpvoTensorRtInternal::DpvoUpdatePostAggRuntime;
using DpvoTensorRtInternal::DpvoUpdatePreAggRuntime;
using DpvoTensorRtInternal::DpvoUpdateRun;
using DpvoTensorRtInternal::DpvoUpdateRuntime;
using DpvoTensorRtInternal::TensorRtEngineHandle;

struct DpvoTensorRtEngine::Impl {
    explicit Impl(DpvoTensorRtConfig cfg)
        : config(std::move(cfg))
    {
    }
    ~Impl()
    {
        Stop();
    }

    bool Start()
    {
        const std::filesystem::path patchPath =
            ResolveEnginePath(config.patchEnginePath, config.repoPath,
                              {"dpvo_patchifier_fp16.engine",
                               "dpvo_patchifier.engine", "dpvo_patch.engine"});
        const std::filesystem::path updatePath =
            ResolveEnginePath(config.updateEnginePath, config.repoPath,
                              {"dpvo_update_fp16.engine", "dpvo_update.engine"});
        const std::filesystem::path updatePreAggPath = ResolveEnginePath(
            std::string{}, config.repoPath,
            {"dpvo_update_preagg_fp16.engine", "dpvo_update_preagg.engine"});
        const std::filesystem::path updatePostAggPath = ResolveEnginePath(
            std::string{}, config.repoPath,
            {"dpvo_update_postagg_fp16.engine", "dpvo_update_postagg.engine"});
        if (patchPath.empty() || updatePath.empty()) {
            std::cerr << "[dpvo_trt] missing engine(s): patch='"
                      << config.patchEnginePath << "' update='"
                      << config.updateEnginePath << "' repo='" << config.repoPath
                      << "'\n";
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
        softAggSplitReady = false;
        if (!updatePreAggPath.empty() && !updatePostAggPath.empty()) {
            if (!updatePreAggEngine.Load(updatePreAggPath, "DPVO update preagg",
                                         &err)) {
                std::cerr << "[dpvo_trt] " << err << "\n";
                return false;
            }
            if (!updatePostAggEngine.Load(updatePostAggPath, "DPVO update postagg",
                                          &err)) {
                std::cerr << "[dpvo_trt] " << err << "\n";
                return false;
            }
            softAggSplitReady = true;
        } else {
            std::cerr << "[dpvo_trt] split SoftAgg engines not found under repo='"
                      << config.repoPath
                      << "'; using compatibility update warmup only\n";
        }
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
        graphState.Reset(config.patchesPerFrame, config.optimizationWindow);
        DpvoUpdateRun updateWarmup =
            updateRuntime.Warmup(updateEngine, cudaStream.stream,
                                 std::max(1, config.patchesPerFrame), &err);
        if (!updateWarmup.ok) {
            std::cerr << "[dpvo_trt] update warmup failed: " << err << "\n";
            return false;
        }
        DpvoUpdateRun preAggWarmup{};
        DpvoUpdateRun postAggWarmup{};
        if (softAggSplitReady) {
            preAggWarmup =
                updatePreAggRuntime.Warmup(updatePreAggEngine, cudaStream.stream,
                                           std::max(1, config.patchesPerFrame), &err);
            if (!preAggWarmup.ok) {
                std::cerr << "[dpvo_trt] update-preagg warmup failed: " << err << "\n";
                return false;
            }
            postAggWarmup = updatePostAggRuntime.Warmup(
                updatePostAggEngine, cudaStream.stream,
                std::max(1, config.patchesPerFrame), &err);
            if (!postAggWarmup.ok) {
                std::cerr << "[dpvo_trt] update-postagg warmup failed: " << err << "\n";
                return false;
            }
        }
        nativeSolver.Reset();
        haveLastPose = false;
        lastPose = Core::Ports::PoseEstimate{};
        loggedKeyframeRemovals = 0;
        processedFrameCount = 0;
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
        running.store(true, std::memory_order_release);
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
                  << " update_warmup_ms=" << updateWarmup.elapsedMs
                  << " preagg_warmup_ms=" << preAggWarmup.elapsedMs
                  << " postagg_warmup_ms=" << postAggWarmup.elapsedMs
                  << " native_cuda_kernels=" << (cudaKernelReady ? 1 : 0)
                  << " native_dpvo=1\n";
        if (!loggedEpgPacing) {
            std::cerr << "[dpvo_trt] epg pacing heavy_every_n=" << heavyEveryN
                      << " heavy_interval_ms=" << heavyIntervalMs
                      << " right_every_n=" << rightEveryN
                      << " warmup_full_frames=" << warmupFullFrames << "\n";
            loggedEpgPacing = true;
        }
        if (!voState.LoadStereoCalibration(config.settingsPath)) {
            std::cerr << "[dpvo_trt] DPVO calibration unavailable settings='"
                      << config.settingsPath << "'; pose output disabled\n";
            running.store(false, std::memory_order_release);
            return false;
        }
        voState.ResetTrackingState();
        return true;
    }

    void Stop()
    {
        running.store(false, std::memory_order_release);
        nativeSolver.Reset();
        cudaKernelReady = false;
        cudaStream.Reset();
    }

    Core::Ports::SlamOutput Process(const Core::Ports::SlamInputBatch &input,
                                    bool extractFeatures,
                                    bool extractPointCloud)
    {
        (void)extractPointCloud;
        const auto start = std::chrono::steady_clock::now();
        Core::Ports::SlamOutput out{};
        out.frameId = input.frameId;
        out.captureTimestampNs = input.captureTimestampNs;
        out.mapId = 1;
        out.usedVisualFeatureFrontend = false;

        if (!running.load(std::memory_order_acquire)) {
            out.trackingState = Core::Ports::kSlamTrackingLost;
            return out;
        }

        const auto prepareStart = std::chrono::steady_clock::now();
        cv::Mat leftGray = EnsureGray8(input.stereo.left.gray);
        cv::Mat rightGray = EnsureGray8(input.stereo.right.gray);
        if (leftGray.empty() || rightGray.empty()) {
            out.trackingState = Core::Ports::kSlamTrackingLost;
            return out;
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

        if (leftRect.cols != config.inputWidth ||
            leftRect.rows != config.inputHeight) {
            cv::resize(leftRect, resizedGray,
                       cv::Size(config.inputWidth, config.inputHeight), 0.0, 0.0,
                       cv::INTER_AREA);
        } else {
            resizedGray = leftRect;
        }
        if (rightRect.cols != config.inputWidth ||
            rightRect.rows != config.inputHeight) {
            cv::resize(rightRect, resizedRightGray,
                       cv::Size(config.inputWidth, config.inputHeight), 0.0, 0.0,
                       cv::INTER_AREA);
        } else {
            resizedRightGray = rightRect;
        }

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
        const bool heavyFrame = warmupFrame || (intervalReady && cadenceReady);
        const bool rightFrame =
            heavyFrame && (warmupFrame || rightEveryN <= 1 ||
                           ((processedFrameCount - 1U) %
                            static_cast<uint64_t>(rightEveryN)) == 0U);
        if (!heavyFrame && haveLastPose) {
            out.visualFeatureRawLeftCount = graphState.PatchCount();
            out.visualFeatureRawRightCount = graphState.LastStereoDepthUpdates();
            out.visualFeatureMatchedStereoCount = graphState.EdgeCount();
            out.matchesInliers = graphState.EdgeCount();
            out.trackedMapPointCount = static_cast<uint32_t>(graphState.EdgeCount());
            out.localMapPointCount = static_cast<uint32_t>(graphState.PatchCount());
            out.trackingState = Core::Ports::kSlamTrackingOk;
            out.poseValid = true;
            out.pose = lastPose;
            out.pose.valid = true;
            out.orbTrackMs = ElapsedMs(start, std::chrono::steady_clock::now());
            out.frontendMs = 0.0;
            out.lkUpdateMs = 0.0;
            return out;
        }

        std::string dpvoErr;
        std::string rightDpvoErr;
        const DpvoPatchifierRun patchRun = patchifierRuntime.Run(
            resizedGray, patchEngine, cudaStream.stream, true, true, &dpvoErr);
        const DpvoPatchifierRun rightPatchRun =
            rightFrame ? patchifierRightRuntime.Run(resizedRightGray, patchEngine,
                                                    cudaStream.stream, true, false,
                                                    &rightDpvoErr)
                       : DpvoPatchifierRun{};
        if (patchRun.ok) {
            out.visualFeatureForwardMs = patchRun.elapsedMs;
            out.visualFeatureStereoMatchMs =
                rightPatchRun.ok ? rightPatchRun.elapsedMs : 0.0;
            if (!loggedPatchifierShape) {
                std::cerr << "[dpvo_trt] patchifier active fmap="
                          << DimsToString(patchRun.fmapDims)
                          << " imap=" << DimsToString(patchRun.imapDims)
                          << " ms=" << patchRun.elapsedMs << "\n";
                loggedPatchifierShape = true;
            }
        } else if (!loggedPatchifierError) {
            std::cerr << "[dpvo_trt] patchifier inference disabled for this frame: "
                      << dpvoErr << "\n";
            loggedPatchifierError = true;
        }
        if (rightFrame && !rightPatchRun.ok && !loggedRightPatchifierError) {
            std::cerr
                << "[dpvo_trt] right patchifier inference disabled for this frame: "
                << rightDpvoErr << "\n";
            loggedRightPatchifierError = true;
        }
        const float scaleX =
            static_cast<float>(config.inputWidth) / std::max(1, leftRect.cols);
        const float scaleY =
            static_cast<float>(config.inputHeight) / std::max(1, leftRect.rows);
        const DpvoIntrinsics intrinsics{
            voState.m_lkFx * scaleX * 0.25f, voState.m_lkFy * scaleY * 0.25f,
            voState.m_lkCx * scaleX * 0.25f, voState.m_lkCy * scaleY * 0.25f};
        graphState.PushFrame(input.frameId, input.captureTimestampNs, resizedGray,
                             nativeSolver.HasPose() ? nativeSolver.LastTcw()
                                                    : Sophus::SE3f{},
                             patchRun);
        if (rightPatchRun.ok) {
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
        const int keyframeRemovalsBefore = graphState.KeyframeRemovals();
        out.visualFeatureRawLeftCount = graphState.PatchCount();
        out.visualFeatureRawRightCount = graphState.LastStereoDepthUpdates();
        out.visualFeatureMatchedStereoCount = graphState.EdgeCount();

        double nativeUpdateMs = 0.0;
        const bool poseUpdated =
            patchRun.ok && softAggSplitReady &&
            nativeSolver.Step(graphState, updatePreAggRuntime, updatePreAggEngine,
                              updatePostAggRuntime, updatePostAggEngine,
                              cudaKernelReady ? &cudaKernelRuntime : nullptr,
                              cudaStream.stream, intrinsics, &nativeUpdateMs,
                              &dpvoErr);
        lastHeavyUpdateEndNs.store(SteadyNowNs(), std::memory_order_release);
        out.lkUpdateMs = nativeUpdateMs;
        out.visualFeatureFrontendMs = out.visualFeatureForwardMs +
                                      out.visualFeatureStereoMatchMs +
                                      nativeUpdateMs;
        out.frontendMs = out.visualFeatureFrontendMs;
        if (graphState.KeyframeRemovals() != keyframeRemovalsBefore &&
            graphState.KeyframeRemovals() != loggedKeyframeRemovals) {
            loggedKeyframeRemovals = graphState.KeyframeRemovals();
            std::cerr << "[dpvo_trt] keyframe removal count="
                      << loggedKeyframeRemovals
                      << " active_edges=" << graphState.EdgeCount()
                      << " active_frames=" << graphState.FrameCount() << "\n";
        }
        out.matchesInliers = graphState.EdgeCount();
        out.trackedMapPointCount = static_cast<uint32_t>(graphState.EdgeCount());
        out.localMapPointCount = static_cast<uint32_t>(graphState.PatchCount());

        if (poseUpdated || nativeSolver.HasPose() || graphState.FrameCount() > 0) {
            const Sophus::SE3f publishTcw =
                nativeSolver.HasPose() ? nativeSolver.LastTcw() : Sophus::SE3f{};
            lastPose = PoseFromTwc(publishTcw.inverse());
            haveLastPose = true;
        }

        if (poseUpdated) {
            out.trackingState = Core::Ports::kSlamTrackingOk;
        } else {
            if (!dpvoErr.empty() && !loggedNativeSolverWait) {
                std::cerr << "[dpvo_trt] native solver waiting: " << dpvoErr << "\n";
                loggedNativeSolverWait = true;
            }
            out.trackingState = haveLastPose ? Core::Ports::kSlamTrackingRecentlyLost
                                             : Core::Ports::kSlamTrackingLost;
        }

        out.poseValid =
            haveLastPose && TrackingStateCanPublishPose(out.trackingState);
        out.pose = lastPose;
        out.pose.valid = out.poseValid;
        if (extractFeatures) {
            const DpvoFrameState *newest = graphState.NewestFrame();
            if (newest != nullptr) {
                out.leftFeatures.reserve(newest->patches.size());
                for (const DpvoPatchState &patch : newest->patches) {
                    out.leftFeatures.emplace_back(
                        patch.x * 4.0f / std::max(scaleX, 1e-6f),
                        patch.y * 4.0f / std::max(scaleY, 1e-6f));
                }
            }
        }
        out.orbTrackMs = ElapsedMs(start, std::chrono::steady_clock::now());
        return out;
    }

    DpvoTensorRtConfig config;
    TensorRtEngineHandle patchEngine;
    TensorRtEngineHandle updateEngine;
    TensorRtEngineHandle updatePreAggEngine;
    TensorRtEngineHandle updatePostAggEngine;
    CudaStreamHandle cudaStream;
    DpvoPatchifierRuntime patchifierRuntime;
    DpvoPatchifierRuntime patchifierRightRuntime;
    DpvoUpdateRuntime updateRuntime;
    DpvoUpdatePreAggRuntime updatePreAggRuntime;
    DpvoUpdatePostAggRuntime updatePostAggRuntime;
    DpvoCudaKernelRuntime cudaKernelRuntime;
    DpvoGraphState graphState;
    DpvoNativeSolver nativeSolver;
    SlamModeSharedState voState;
    cv::Mat resizedGray;
    cv::Mat resizedRightGray;
    Core::Ports::PoseEstimate lastPose{};
    uint64_t processedFrameCount{0};
    int heavyEveryN{1};
    int rightEveryN{1};
    int heavyIntervalMs{0};
    int warmupFullFrames{8};
    std::atomic<int64_t> lastHeavyUpdateEndNs{0};
    bool haveLastPose{false};
    std::atomic<bool> running{false};
    bool softAggSplitReady{false};
    bool cudaKernelReady{false};
    bool loggedEpgPacing{false};
    bool loggedPatchifierShape{false};
    bool loggedPatchifierError{false};
    bool loggedRightPatchifierError{false};
    bool loggedNativeSolverWait{false};
    bool loggedStereoDepthInit{false};
    int loggedKeyframeRemovals{0};
};
