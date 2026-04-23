#include "core/application/session/slam_frame_processor.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "common/logger.h"

namespace smartdrone::core::application {

namespace {

constexpr uint64_t kSlamDfxLogEveryNFrames = 30;
constexpr int kXFeatStereoWeakMatchThreshold = 24;

uint8_t ComposeResetCounter(uint8_t sessionBase, uint8_t continuityCounter)
{
    return static_cast<uint8_t>(sessionBase + continuityCounter);
}

uint16_t ComposeResetMapCount(uint16_t sessionBase, uint16_t continuityResetMapCount)
{
    return static_cast<uint16_t>(sessionBase + continuityResetMapCount);
}

Sophus::SE3f BuildBodyToCamFromRuntimeOverride(float tx, float ty, float tz, float rollDeg, float pitchDeg, float yawDeg)
{
    constexpr float kDegToRad = 0.017453292519943295769f;
    const float rollRad = rollDeg * kDegToRad;
    const float pitchRad = pitchDeg * kDegToRad;
    const float yawRad = yawDeg * kDegToRad;
    const Eigen::AngleAxisf rollRotation(rollRad, Eigen::Vector3f::UnitX());
    const Eigen::AngleAxisf pitchRotation(pitchRad, Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf yawRotation(yawRad, Eigen::Vector3f::UnitZ());
    const Eigen::Quaternionf q = yawRotation * pitchRotation * rollRotation;
    return Sophus::SE3f(Sophus::SO3f(q), Eigen::Vector3f(tx, ty, tz));
}

Sophus::SE3f BuildBodyToCamPitchDelta(float pitchDeg)
{
    constexpr float kDegToRad = 0.017453292519943295769f;
    const Eigen::AngleAxisf pitchRotation(pitchDeg * kDegToRad, Eigen::Vector3f::UnitY());
    return Sophus::SE3f(Sophus::SO3f(Eigen::Quaternionf(pitchRotation)), Eigen::Vector3f::Zero());
}

} // namespace

SlamFrameProcessor::SlamFrameProcessor(Context &context, State &state) : m_ctx(context), m_state(state) {}

SlamFrameProcessor::StepResult SlamFrameProcessor::ProcessNextFrame(bool &sessionOk)
{
    const auto frameStartTp = std::chrono::steady_clock::now();
    const auto configuredSlamMode = static_cast<smartdrone::core::domain::SlamOperationMode>(
        m_ctx.tuning.slamOperationMode.load(std::memory_order_relaxed));
    if (configuredSlamMode != m_state.requestedSlamMode) {
        m_state.requestedSlamMode = configuredSlamMode;
        m_ctx.autoSlamModeController.Reset();
        m_state.effectiveSlamMode = m_state.requestedSlamMode == smartdrone::core::domain::SlamOperationMode::Auto
                                        ? smartdrone::core::domain::SlamOperationMode::Mapping
                                        : m_state.requestedSlamMode;
        m_ctx.slamEngine.SetOperationMode(m_state.effectiveSlamMode);
        std::cerr << "[slam] operation_mode -> " << smartdrone::core::domain::ToString(m_state.requestedSlamMode);
        if (m_state.requestedSlamMode == smartdrone::core::domain::SlamOperationMode::Auto) {
            std::cerr << " effective_mode=" << smartdrone::core::domain::ToString(m_state.effectiveSlamMode);
        }
        std::cerr << "\n";
        if (m_state.requestedSlamMode == smartdrone::core::domain::SlamOperationMode::Relocalization ||
            m_state.requestedSlamMode == smartdrone::core::domain::SlamOperationMode::TrackingOnly) {
            std::cerr << "[slam] note: requested mode currently maps to ORB-SLAM3 localization-only mode\n";
        }
        m_ctx.livePose.SetSlamMode(ToRuntimeSlamModeValue(m_state.effectiveSlamMode));
    }

    const int slamInputFps =
        m_ctx.perceptionPipeline.ClampTargetFps(m_ctx.tuning.slamInputFps.load(std::memory_order_relaxed));
    const FeatureFrontend configuredFrontend =
        static_cast<FeatureFrontend>(m_ctx.tuning.featureFrontend.load(std::memory_order_relaxed));
    m_ctx.slamEngine.SetFeatureFrontend(configuredFrontend);
    if (slamInputFps != m_state.lastLoggedSlamInputFps) {
        std::cerr << "[slam] target_input_fps=" << slamInputFps << " camera_fps=" << m_ctx.aliases.fps
                  << " frame_drop=" << (slamInputFps < m_ctx.aliases.fps ? "enabled" : "disabled") << "\n";
        m_state.lastLoggedSlamInputFps = slamInputFps;
    }

    const auto acquireStartTp = std::chrono::steady_clock::now();
    StereoBatch stereoBatch{};
    const StereoAcquireStatus acquireStatus = m_ctx.perceptionPipeline.AcquireNextStereoBatch(
        m_ctx.cameraProvider, slamInputFps, 1000, stereoBatch, &m_ctx.frameTimingTracker);
    const auto acquireEndTp = std::chrono::steady_clock::now();
    if (acquireStatus == StereoAcquireStatus::Timeout) {
        return StepResult::Continue;
    }
    if (acquireStatus == StereoAcquireStatus::CameraUnhealthy) {
        std::cerr << "[slam] camera pipeline unhealthy, aborting session\n";
        sessionOk = false;
        return StepResult::SessionAbort;
    }
    if (acquireStatus == StereoAcquireStatus::DroppedByRateLimiter) {
        ++m_state.rateLimitedDrops;
        return StepResult::Continue;
    }

    auto &L = stereoBatch.stereo.left;
    auto &R = stereoBatch.stereo.right;
    const bool sendImage = m_ctx.tuning.sendImage.load(std::memory_order_relaxed);
    const bool sendFeature = m_ctx.tuning.sendFeature.load(std::memory_order_relaxed);
    const bool sendMap = m_ctx.tuning.sendMap.load(std::memory_order_relaxed);
    const auto cameraDiag = m_ctx.cameraProvider.GetDiagnostics();
    const int64_t pairDtMs = cameraDiag.lastPairDtMs;
    const double rejectDtMs = static_cast<double>(cameraDiag.lastRejectDtUs) / 1000.0;
    const uint64_t dropUnpairedL = cameraDiag.droppedUnpairedL;
    const uint64_t dropUnpairedR = cameraDiag.droppedUnpairedR;
    const size_t pendingL = cameraDiag.pendingL;
    const size_t pendingR = cameraDiag.pendingR;
    const int64_t captureTimestampNs =
        m_ctx.monoMode ? static_cast<int64_t>(R.timestampNs) : stereoBatch.captureTimestampNs;
    const int64_t logicalFrameTimestampNs =
        m_ctx.monoMode ? captureTimestampNs : stereoBatch.logicalFrameTimestampNs;
    const double frameTime = static_cast<double>(captureTimestampNs) * 1e-9;
    const double frameGapMs = (m_state.lastPublishedFrameNs != 0)
                                  ? static_cast<double>(logicalFrameTimestampNs - m_state.lastPublishedFrameNs) * 1e-6
                                  : 0.0;
    if (frameGapMs > 0.0 && slamInputFps > 0) {
        const double expectedFrameGapMs = 1000.0 / static_cast<double>(slamInputFps);
        if (frameGapMs > expectedFrameGapMs) {
            constexpr int64_t kGapWarnMinIntervalNs = 1000000000LL; // 1s
            if (m_state.lastFrameGapWarnLogNs == 0 ||
                (logicalFrameTimestampNs - m_state.lastFrameGapWarnLogNs) >= kGapWarnMinIntervalNs) {
                std::cerr << "[slam_gap_warn] frame_gap_ms=" << frameGapMs
                          << " expected_gap_ms=" << expectedFrameGapMs << " target_input_fps=" << slamInputFps
                          << " camera_fps=" << m_ctx.aliases.fps << " frame=" << stereoBatch.frameId << "\n";
                m_state.lastFrameGapWarnLogNs = logicalFrameTimestampNs;
            }
        }
    }
    const double monoStepMs = static_cast<double>(stereoBatch.monotonicFrameStepNs) * 1e-6;
    double meanL = 0.0, stdL = 0.0, meanR = 0.0, stdR = 0.0;
    ComputeImageStats(L.gray, meanL, stdL);
    ComputeImageStats(R.gray, meanR, stdR);
    const double sharpL = ComputeSharpnessLaplacianVar(L.gray);
    const double sharpR = ComputeSharpnessLaplacianVar(R.gray);

    smartdrone::core::ports::SlamInputBatch slamInput{};
    const auto imuStartTp = std::chrono::steady_clock::now();
    if (m_ctx.useImu && m_state.lastFrameNs != 0) {
        const std::vector<smartdrone::core::ports::ImuReading> imuSamples =
            m_ctx.imuProvider.PopWindow(m_state.lastFrameNs, captureTimestampNs);
        slamInput.imu = ToOrbImuPoints(imuSamples);
        ImuWindowValidation imuWindow{};
        const double prevFrameTime = static_cast<double>(m_state.lastFrameNs) * 1e-9;
        const double expectedImuDtSec = 1.0 / std::max(1, m_ctx.aliases.imuHz);
        const bool imuWindowOk =
            SanitizeImuWindow(slamInput.imu, prevFrameTime, frameTime, expectedImuDtSec, imuWindow);
        if (!imuWindowOk) {
            return StepResult::Continue;
        }
        if (slamInput.imu.empty() && !m_ctx.aliases.allowEmptyImu) {
            return StepResult::Continue;
        }
    }
    const auto imuEndTp = std::chrono::steady_clock::now();
    m_state.lastFrameNs = captureTimestampNs;

    PrepareStereoPairForSlam(stereoBatch.stereo, meanL, stdL, meanR, stdR, sharpL, sharpR,
                             m_ctx.aliases.slamLowLightEnhance, slamInput.stereo);
    slamInput.frameId = stereoBatch.frameId;
    slamInput.captureTimestampNs = captureTimestampNs;
    slamInput.frameTimeSec = frameTime;
    const bool debugRightOnlyFeatures = m_ctx.aliases.debugRightOnlyFeatures;
    const bool extractFeatures =
        sendFeature || m_state.requestedSlamMode == smartdrone::core::domain::SlamOperationMode::Auto;
    const bool updatePointCloud =
        !debugRightOnlyFeatures &&
        sendMap &&
        (captureTimestampNs - m_state.lastPointCloudUpdateNs) >= kPointCloudUpdateIntervalNs;

    const auto slamStartTp = std::chrono::steady_clock::now();
    const uint64_t slamInputTimestampNs = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(slamStartTp.time_since_epoch()).count());
    m_ctx.frameTimingTracker.MarkSlamIn(slamInput.frameId, slamInputTimestampNs);
    smartdrone::core::ports::SlamOutput slamOutput{};
    slamOutput.frameId = slamInput.frameId;
    slamOutput.captureTimestampNs = slamInput.captureTimestampNs;
    if (debugRightOnlyFeatures) {
        slamOutput.leftFeatures.clear();
        slamOutput.rightFeatures = ComputeOrbDebugFeatures(R.gray);
    } else {
        slamOutput = m_ctx.slamEngine.Process(slamInput, extractFeatures, updatePointCloud);
    }
    const auto slamEndTp = std::chrono::steady_clock::now();
    const uint64_t slamOutputTimestampNs = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(slamEndTp.time_since_epoch()).count());
    m_ctx.frameTimingTracker.MarkSlamOut(slamInput.frameId, slamOutputTimestampNs);

    const auto cloudStartTp = std::chrono::steady_clock::now();
    size_t pointCount = 0;
    if (m_ctx.aliases.udpEnable && (sendImage || sendFeature || sendMap)) {
        if (updatePointCloud) {
            m_ctx.livePose.UpdatePointCloud(slamOutput.pointCloudXyz);
            m_state.lastPointCloudUpdateNs = captureTimestampNs;
        }
        pointCount = slamOutput.pointCloudXyz.size() / 3;
    }
    const auto cloudEndTp = std::chrono::steady_clock::now();

    const auto udpStartTp = std::chrono::steady_clock::now();
    if (m_ctx.aliases.udpEnable && (sendImage || sendFeature || sendMap)) {
        if (m_ctx.monoMode) {
            m_ctx.udpSender.Enqueue(1, slamOutput.frameId, R.sequence, frameTime, R.gray, slamOutput.rightFeatures,
                                    sendImage, sendFeature);
        } else {
            m_ctx.udpSender.Enqueue(0, slamOutput.frameId, L.sequence, frameTime, L.gray, slamOutput.leftFeatures,
                                    sendImage, sendFeature);
            m_ctx.udpSender.Enqueue(1, slamOutput.frameId, R.sequence, frameTime, R.gray, slamOutput.rightFeatures,
                                    sendImage, sendFeature);
        }
    }
    const auto udpEndTp = std::chrono::steady_clock::now();

    const int state = debugRightOnlyFeatures ? ORB_SLAM3::Tracking::LOST : slamOutput.trackingState;
    const bool trackingUsable = !debugRightOnlyFeatures && IsTrackingPoseUsable(state);
    const unsigned long mapId = debugRightOnlyFeatures ? 0UL : slamOutput.mapId;
    const bool mapIdChanged =
        mapId != PosePostprocessor::ContinuityMapper::kInvalidMapId && mapId != m_state.lastRawMapId;
    if (mapIdChanged) {
        m_state.lastRawMapId = mapId;
        if (!m_ctx.useImu) {
            m_state.stereoReferencePoseSet = false;
        }
    }

    Sophus::SE3f twcRaw = m_state.haveLastValidTwcRaw ? m_state.lastValidTwcRaw : Sophus::SE3f();
    if (!debugRightOnlyFeatures && slamOutput.poseValid) {
        const Eigen::Quaternionf rawQ(slamOutput.pose.qw, slamOutput.pose.qx, slamOutput.pose.qy, slamOutput.pose.qz);
        twcRaw =
            Sophus::SE3f(Sophus::SO3f(rawQ), Eigen::Vector3f(slamOutput.pose.x, slamOutput.pose.y, slamOutput.pose.z));
        m_state.lastValidTwcRaw = twcRaw;
        m_state.haveLastValidTwcRaw = true;
    }

    const auto postStartTp = std::chrono::steady_clock::now();
    bool useStereoBodyExtrinsics = m_ctx.stereoBodyExtrinsics.loaded;
    Sophus::SE3f stereoBodyExtrinsics = m_ctx.stereoBodyExtrinsics.Tbc;
    if (!m_ctx.useImu && !m_ctx.monoMode && m_ctx.tuning.useCustomTbc.load(std::memory_order_relaxed)) {
        const float pitchDeg = m_ctx.tuning.tbcPitchDeg.load(std::memory_order_relaxed);
        if (m_ctx.stereoBodyExtrinsics.loaded) {
            // Runtime pitch is an incremental gimbal motion on top of the
            // calibrated body->camera extrinsics. Right-multiplication makes a
            // forward-facing camera sweep toward downward view as pitch grows.
            stereoBodyExtrinsics = m_ctx.stereoBodyExtrinsics.Tbc * BuildBodyToCamPitchDelta(pitchDeg);
        } else {
            const float tx = m_ctx.tuning.tbcTx.load(std::memory_order_relaxed);
            const float ty = m_ctx.tuning.tbcTy.load(std::memory_order_relaxed);
            const float tz = m_ctx.tuning.tbcTz.load(std::memory_order_relaxed);
            const float rollDeg = m_ctx.tuning.tbcRollDeg.load(std::memory_order_relaxed);
            const float yawDeg = m_ctx.tuning.tbcYawDeg.load(std::memory_order_relaxed);
            stereoBodyExtrinsics = BuildBodyToCamFromRuntimeOverride(tx, ty, tz, rollDeg, pitchDeg, yawDeg);
        }
        useStereoBodyExtrinsics = true;
    }

    const auto poseResult = m_ctx.posePostprocessor.ProcessPose(
        twcRaw, m_ctx.useImu, trackingUsable, state, mapId, useStereoBodyExtrinsics, stereoBodyExtrinsics,
        m_state.stereoReferencePoseSet, m_state.stereoReferencePose, captureTimestampNs, m_ctx.mav);
    const uint8_t effectiveResetCounter =
        ComposeResetCounter(m_state.sessionResetCounterBase, poseResult.resetCounter);
    const uint16_t effectiveResetMapCount =
        ComposeResetMapCount(m_state.sessionResetMapCountBase, poseResult.resetMapCount);
    const auto postEndTp = std::chrono::steady_clock::now();

    const auto livePoseStartTp = std::chrono::steady_clock::now();
    m_ctx.livePose.UpdatePose(RUNTIME_MODE_SLAM, static_cast<uint8_t>(state), effectiveResetCounter,
                              effectiveResetMapCount, poseResult.alignedPose,
                              poseResult.quality == smartdrone::core::ports::PoseQuality::Good ? OdomQualityMode::GOOD
                              : poseResult.quality == smartdrone::core::ports::PoseQuality::Weak
                                  ? OdomQualityMode::WEAK
                                  : OdomQualityMode::LOST);
    const auto livePoseEndTp = std::chrono::steady_clock::now();

    const auto publishStartTp = std::chrono::steady_clock::now();
    m_ctx.posePublisher.PublishPose(slamOutput.frameId, poseResult.poseEstimate, poseResult.velocityEstimate,
                                    effectiveResetCounter, effectiveResetMapCount, state, poseResult.quality);
    const auto publishEndTp = std::chrono::steady_clock::now();

    if (m_state.requestedSlamMode == smartdrone::core::domain::SlamOperationMode::Auto) {
        const auto autoEffectiveMode =
            m_ctx.autoSlamModeController.Observe(trackingUsable, poseResult.quality, frameGapMs,
                                                 slamOutput.leftFeatures.size(), slamOutput.rightFeatures.size());
        if (autoEffectiveMode != m_state.effectiveSlamMode) {
            m_state.effectiveSlamMode = autoEffectiveMode;
            m_ctx.slamEngine.SetOperationMode(m_state.effectiveSlamMode);
            m_ctx.livePose.SetSlamMode(ToRuntimeSlamModeValue(m_state.effectiveSlamMode));
            std::cerr << "[slam_auto] effective_mode -> "
                      << smartdrone::core::domain::ToString(m_state.effectiveSlamMode)
                      << " quality=" << static_cast<int>(poseResult.quality) << " state=" << state
                      << " featL=" << slamOutput.leftFeatures.size() << " featR=" << slamOutput.rightFeatures.size()
                      << " frame_gap_ms=" << frameGapMs << "\n";
        }
    }

    ++m_state.frameIndex;
    m_state.lastPublishedFrameNs = logicalFrameTimestampNs;
    const bool xfeatStereoWeak =
        slamOutput.usedXFeatFrontend && !m_ctx.monoMode &&
        slamOutput.xfeatMatchedStereoCount < kXFeatStereoWeakMatchThreshold;
    const double totalMs = DurationMs(frameStartTp, publishEndTp);
    const bool slamDfxPeriodic = (kSlamDfxLogEveryNFrames > 0) && ((m_state.frameIndex % kSlamDfxLogEveryNFrames) == 0);
    const bool slamDfxAbnormal = !poseResult.poseEstimate.valid || state <= 0 || totalMs > 80.0 ||
                                 slamOutput.leftFeatures.empty() || slamOutput.rightFeatures.empty() ||
                                 xfeatStereoWeak;
    if (slamDfxPeriodic || slamDfxAbnormal) {
        char dfxLine[896];
        if (m_ctx.aliases.jsonDiagnostics) {
            std::snprintf(
                dfxLine, sizeof(dfxLine),
                "{\"tag\":\"slam_dfx\",\"frame\":%llu,\"state\":%d,\"quality\":%d,\"pose_valid\":%d,"
                "\"reset_counter\":%u,\"reset_map_count\":%u,"
                "\"imu_count\":%zu,\"feat_left\":%zu,\"feat_right\":%zu,\"points\":%zu,"
                "\"xfeat_used\":%d,\"xfeat_stereo_weak\":%d,\"xfeat_raw_left\":%d,\"xfeat_raw_right\":%d,"
                "\"xfeat_match_stereo\":%d,\"xfeat_injected_left\":%d,\"xfeat_injected_right\":%d,"
                "\"xfeat_prepare_ms\":%.3f,\"xfeat_write_ms\":%.3f,\"xfeat_read_ms\":%.3f,"
                "\"xfeat_worker_ms\":%.3f,\"xfeat_match_ms\":%.3f,\"xfeat_total_ms\":%.3f,"
                "\"xfeat_image_count\":%u,\"xfeat_payload_bytes\":%u,"
                "\"pair_dt_ms\":%.3f,\"reject_dt_ms\":%.3f,\"pending_left\":%zu,\"pending_right\":%zu,"
                "\"drop_left\":%llu,\"drop_right\":%llu,\"rate_drop\":%llu,"
                "\"img_std_left\":%.2f,\"img_std_right\":%.2f,\"sharp_left\":%.2f,\"sharp_right\":%.2f,"
                "\"gap_ms\":%.3f,\"mono_step_ms\":%.3f,"
                "\"acquire_ms\":%.3f,\"imu_ms\":%.3f,\"slam_ms\":%.3f,\"cloud_ms\":%.3f,\"udp_ms\":%.3f,"
                "\"post_ms\":%.3f,\"live_ms\":%.3f,\"publish_ms\":%.3f,\"total_ms\":%.3f}",
                static_cast<unsigned long long>(slamOutput.frameId), state, static_cast<int>(poseResult.quality),
                poseResult.poseEstimate.valid ? 1 : 0, static_cast<unsigned>(effectiveResetCounter),
                static_cast<unsigned>(effectiveResetMapCount), slamInput.imu.size(), slamOutput.leftFeatures.size(),
                slamOutput.rightFeatures.size(), pointCount, slamOutput.usedXFeatFrontend ? 1 : 0,
                xfeatStereoWeak ? 1 : 0, slamOutput.xfeatRawLeftCount, slamOutput.xfeatRawRightCount,
                slamOutput.xfeatMatchedStereoCount, slamOutput.xfeatInjectedLeftCount,
                slamOutput.xfeatInjectedRightCount, slamOutput.xfeatPrepareMs, slamOutput.xfeatWorkerWriteMs,
                slamOutput.xfeatWorkerReadMs, slamOutput.xfeatWorkerTotalMs, slamOutput.xfeatStereoMatchMs,
                slamOutput.xfeatTotalMs, slamOutput.xfeatImageCount, slamOutput.xfeatPayloadBytes,
                static_cast<double>(pairDtMs), rejectDtMs, pendingL, pendingR,
                static_cast<unsigned long long>(dropUnpairedL), static_cast<unsigned long long>(dropUnpairedR),
                static_cast<unsigned long long>(m_state.rateLimitedDrops), stdL, stdR, sharpL, sharpR, frameGapMs,
                monoStepMs, DurationMs(acquireStartTp, acquireEndTp), DurationMs(imuStartTp, imuEndTp),
                DurationMs(slamStartTp, slamEndTp), DurationMs(cloudStartTp, cloudEndTp), DurationMs(udpStartTp, udpEndTp),
                DurationMs(postStartTp, postEndTp), DurationMs(livePoseStartTp, livePoseEndTp),
                DurationMs(publishStartTp, publishEndTp), totalMs);
        } else {
            std::snprintf(
                dfxLine, sizeof(dfxLine),
                "[slam_dfx] frame=%llu state=%d quality=%d pose_valid=%d reset=%u/%u "
                "imu=%zu feat=%zu/%zu points=%zu "
                "xfeat=%s stereo_warn=%s raw=%d/%d stereo=%d injected=%d/%d "
                "xfeat_ms=prep %.3f write %.3f read %.3f worker %.3f match %.3f total %.3f "
                "xfeat_io=%uimg/%ubytes "
                "pair_dt=%.3f reject_dt=%.3f pend=%zu/%zu drop=%llu/%llu rate_drop=%llu "
                "img_std=%.2f/%.2f sharp=%.2f/%.2f "
                "timing_ms gap=%.3f mono=%.3f acquire=%.3f imu=%.3f slam=%.3f cloud=%.3f udp=%.3f post=%.3f "
                "live=%.3f publish=%.3f total=%.3f",
                static_cast<unsigned long long>(slamOutput.frameId), state, static_cast<int>(poseResult.quality),
                poseResult.poseEstimate.valid ? 1 : 0, static_cast<unsigned>(effectiveResetCounter),
                static_cast<unsigned>(effectiveResetMapCount), slamInput.imu.size(), slamOutput.leftFeatures.size(),
                slamOutput.rightFeatures.size(), pointCount, slamOutput.usedXFeatFrontend ? "on" : "off",
                xfeatStereoWeak ? "weak" : "ok", slamOutput.xfeatRawLeftCount, slamOutput.xfeatRawRightCount,
                slamOutput.xfeatMatchedStereoCount, slamOutput.xfeatInjectedLeftCount,
                slamOutput.xfeatInjectedRightCount, slamOutput.xfeatPrepareMs, slamOutput.xfeatWorkerWriteMs,
                slamOutput.xfeatWorkerReadMs, slamOutput.xfeatWorkerTotalMs, slamOutput.xfeatStereoMatchMs,
                slamOutput.xfeatTotalMs, slamOutput.xfeatImageCount, slamOutput.xfeatPayloadBytes,
                static_cast<double>(pairDtMs), rejectDtMs, pendingL, pendingR,
                static_cast<unsigned long long>(dropUnpairedL), static_cast<unsigned long long>(dropUnpairedR),
                static_cast<unsigned long long>(m_state.rateLimitedDrops), stdL, stdR, sharpL, sharpR, frameGapMs,
                monoStepMs, DurationMs(acquireStartTp, acquireEndTp), DurationMs(imuStartTp, imuEndTp),
                DurationMs(slamStartTp, slamEndTp), DurationMs(cloudStartTp, cloudEndTp), DurationMs(udpStartTp, udpEndTp),
                DurationMs(postStartTp, postEndTp), DurationMs(livePoseStartTp, livePoseEndTp),
                DurationMs(publishStartTp, publishEndTp), totalMs);
        }
        Logger::Logf(Logger::INFO, "%s", dfxLine);
        std::fprintf(stderr, "%s\n", dfxLine);
    }

    return StepResult::Continue;
}

} // namespace smartdrone::core::application
