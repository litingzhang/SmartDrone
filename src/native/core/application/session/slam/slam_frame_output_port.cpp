#include "core/application/session/slam/slam_frame_output_port.h"

#include <chrono>
#include <cstdio>
#include <iomanip>
#include <sstream>

#include "common/logger.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/session/slam/slam_processing_support.h"
#include "core/application/session/stream/preview_output_port.h"
#include "core/application/state/live_pose_state.h"

namespace SmartDrone::core::application {
namespace {

constexpr uint64_t kSlamDfxLogEveryNFrames = 30;
constexpr int kVisualFeatureStereoWeakMatchThreshold = 24;
constexpr double kDfxTimingEmaAlpha = 0.18;

double UpdateEma(double current, double sample)
{
    if (!(sample > 0.0)) {
        return current;
    }
    if (!(current > 0.0)) {
        return sample;
    }
    return (1.0 - kDfxTimingEmaAlpha) * current +
           kDfxTimingEmaAlpha * sample;
}

LivePoseQuality ToLivePoseQuality(
    SmartDrone::core::ports::PoseQuality quality)
{
    if (quality == SmartDrone::core::ports::PoseQuality::Good) {
        return LivePoseQuality::Good;
    }
    if (quality == SmartDrone::core::ports::PoseQuality::Weak) {
        return LivePoseQuality::Weak;
    }
    return LivePoseQuality::Lost;
}

} // namespace

SlamFrameOutputPort::SlamFrameOutputPort(SlamFrameOutputContext &context,
                                         SlamFrameOutputState &state)
    : m_ctx(context), m_state(state)
{
}

SlamFrameStepResult SlamFrameOutputPort::EmitPointCloud(
    SlamPublishedFrameData &published)
{
    if (!published.frame || !published.frame->frame) {
        return SlamFrameStepResult::Continue;
    }
    auto &tracked = *published.frame;
    auto &frame = *tracked.frame;
    auto &slamOutput = tracked.slamOutput;
    const bool sendImage = frame.sendImage;
    const bool sendFeature = frame.sendFeature;
    const bool sendMap = frame.sendMap;
    const bool updatePointCloud = frame.updatePointCloud;

    const auto cloudStartTp = std::chrono::steady_clock::now();
    size_t pointCount = 0;
    if (m_ctx.aliases.udpEnable && (sendImage || sendFeature || sendMap)) {
        if (updatePointCloud) {
            m_ctx.livePose.UpdatePointCloud(slamOutput.pointCloudXyz);
            m_state.lastPointCloudUpdateNs.store(frame.captureTimestampNs);
        }
        pointCount = slamOutput.pointCloudXyz.size() / 3;
    }
    published.cloudStartTp = cloudStartTp;
    published.cloudEndTp = std::chrono::steady_clock::now();
    published.pointCount = pointCount;
    return SlamFrameStepResult::Continue;
}

SlamFrameStepResult SlamFrameOutputPort::EmitLivePose(
    SlamPublishedFrameData &published)
{
    if (!published.frame || !published.frame->frame) {
        return SlamFrameStepResult::Continue;
    }
    auto &tracked = *published.frame;
    auto &slamOutput = tracked.slamOutput;
    const auto &poseResult = published.poseResult;
    const auto livePoseStartTp = std::chrono::steady_clock::now();
    const bool livePoseValid =
        slamOutput.poseValid && poseResult.poseEstimate.valid &&
        published.trackingUsable &&
        poseResult.quality != SmartDrone::core::ports::PoseQuality::Lost;
    LivePoseUpdate update{};
    update.runtimeMode = RUNTIME_MODE_SLAM;
    update.trackingState = static_cast<uint8_t>(published.trackingState);
    update.resetCounter = published.effectiveResetCounter;
    update.resetMapCount = published.effectiveResetMapCount;
    update.pose.x = poseResult.poseEstimate.x;
    update.pose.y = poseResult.poseEstimate.y;
    update.pose.z = poseResult.poseEstimate.z;
    update.pose.qw = poseResult.poseEstimate.qw;
    update.pose.qx = poseResult.poseEstimate.qx;
    update.pose.qy = poseResult.poseEstimate.qy;
    update.pose.qz = poseResult.poseEstimate.qz;
    update.quality = ToLivePoseQuality(poseResult.quality);
    update.poseValid = livePoseValid;
    m_ctx.livePose.UpdatePose(update);
    published.livePoseStartTp = livePoseStartTp;
    published.livePoseEndTp = std::chrono::steady_clock::now();
    return SlamFrameStepResult::Continue;
}

SlamFrameStepResult SlamFrameOutputPort::EmitMavlink(
    SlamPublishedFrameData &published)
{
    if (!published.frame || !published.frame->frame) {
        return SlamFrameStepResult::Continue;
    }
    auto &tracked = *published.frame;
    auto &slamOutput = tracked.slamOutput;
    const auto &poseResult = published.poseResult;
    const auto publishStartTp = std::chrono::steady_clock::now();
    m_ctx.posePublisher.PublishPose(
        slamOutput.frameId, poseResult.poseEstimate,
        poseResult.velocityEstimate, published.effectiveResetCounter,
        published.effectiveResetMapCount, published.trackingState,
        poseResult.quality);
    published.publishStartTp = publishStartTp;
    published.publishEndTp = std::chrono::steady_clock::now();
    return SlamFrameStepResult::Continue;
}

SlamFrameStepResult SlamFrameOutputPort::EmitUdp(
    SlamPublishedFrameData &published)
{
    if (!published.frame || !published.frame->frame) {
        return SlamFrameStepResult::Continue;
    }
    auto &tracked = *published.frame;
    auto &frame = *tracked.frame;
    auto &stereoBatch = frame.stereoBatch;
    auto &slamOutput = tracked.slamOutput;
    auto &L = stereoBatch.stereo.left;
    auto &R = stereoBatch.stereo.right;
    const bool sendImage = frame.sendImage;
    const bool sendFeature = frame.sendFeature;
    const bool sendMap = frame.sendMap;
    const double frameTime = frame.frameTime;

    const auto udpStartTp = std::chrono::steady_clock::now();
    if (m_ctx.aliases.udpEnable && (sendImage || sendFeature || sendMap)) {
        if (m_ctx.monoMode) {
            m_ctx.previewOutput.Enqueue(
                {1, slamOutput.frameId, R.sequence, frameTime, R.gray,
                 slamOutput.rightFeatures, sendImage, sendFeature});
        } else {
            m_ctx.previewOutput.Enqueue(
                {0, slamOutput.frameId, L.sequence, frameTime, L.gray,
                 slamOutput.leftFeatures, sendImage, sendFeature});
            m_ctx.previewOutput.Enqueue(
                {1, slamOutput.frameId, R.sequence, frameTime, R.gray,
                 slamOutput.rightFeatures, sendImage, sendFeature});
        }
        m_ctx.previewOutput.StepAll();
    }
    published.udpStartTp = udpStartTp;
    published.udpEndTp = std::chrono::steady_clock::now();
    return SlamFrameStepResult::Continue;
}

SlamFrameStepResult SlamFrameOutputPort::EmitDfx(
    SlamPublishedFrameData &published)
{
    if (!published.frame || !published.frame->frame) {
        return SlamFrameStepResult::Continue;
    }
    const DfxSample sample = MakeDfxSample(published);
    m_state.frameIndex.fetch_add(1);
    m_state.lastPublishedFrameNs.store(sample.frame.logicalFrameTimestampNs);
    if (ShouldLogDfx(sample)) {
        LogDfxLine(sample);
    }
    UpdateDfxAverages(sample);
    return SlamFrameStepResult::Continue;
}

SlamFrameOutputPort::DfxSample SlamFrameOutputPort::MakeDfxSample(
    SlamPublishedFrameData &published) const
{
    auto &tracked = *published.frame;
    auto &frame = *tracked.frame;
    auto &slamOutput = tracked.slamOutput;
    const bool stereoWeak = slamOutput.usedVisualFeatureFrontend &&
                            !m_ctx.monoMode &&
                            slamOutput.visualFeatureMatchedStereoCount <
                                kVisualFeatureStereoWeakMatchThreshold;
    return {frame, tracked, published, frame.slamInput, slamOutput,
            published.poseResult, MakeDfxTiming(frame, tracked, published),
            stereoWeak};
}

SlamFrameOutputPort::DfxTiming SlamFrameOutputPort::MakeDfxTiming(
    const SlamPreparedFrameData &frame,
    const SlamTrackedFrameData &tracked,
    const SlamPublishedFrameData &published) const
{
    return {
        DurationMs(frame.acquireStartTp, frame.acquireEndTp),
        DurationMs(frame.imuStartTp, frame.imuEndTp),
        DurationMs(tracked.slamStartTp, tracked.slamEndTp),
        DurationMs(published.cloudStartTp, published.cloudEndTp),
        DurationMs(published.udpStartTp, published.udpEndTp),
        DurationMs(published.postStartTp, published.postEndTp),
        DurationMs(published.livePoseStartTp, published.livePoseEndTp),
        DurationMs(published.publishStartTp, published.publishEndTp),
        DurationMs(frame.frameStartTp, published.publishEndTp),
    };
}

bool SlamFrameOutputPort::ShouldLogDfx(const DfxSample &sample) const
{
    const auto &output = sample.slamOutput;
    const uint64_t frameIndex = m_state.frameIndex.load();
    const bool periodic = kSlamDfxLogEveryNFrames > 0 &&
                          (frameIndex % kSlamDfxLogEveryNFrames) == 0;
    const bool abnormal = !sample.poseResult.poseEstimate.valid ||
                          sample.published.trackingState <= 0 ||
                          sample.timing.totalMs > 80.0 ||
                          output.leftFeatures.empty() ||
                          output.rightFeatures.empty() ||
                          sample.visualFeatureStereoWeak;
    return periodic || abnormal;
}

void SlamFrameOutputPort::LogDfxLine(const DfxSample &sample) const
{
    const std::string line = m_ctx.aliases.jsonDiagnostics
                                 ? BuildJsonDfxLine(sample)
                                 : BuildTextDfxLine(sample);
    Logger::Logf(Logger::INFO, "%s", line.c_str());
    std::fprintf(stderr, "%s\n", line.c_str());
}

void SlamFrameOutputPort::UpdateDfxAverages(const DfxSample &sample)
{
    m_state.smoothedAcquireMs.store(
        UpdateEma(m_state.smoothedAcquireMs.load(), sample.timing.acquireMs));
    m_state.smoothedSlamMs.store(
        UpdateEma(m_state.smoothedSlamMs.load(), sample.timing.slamMs));
    m_state.smoothedTotalMs.store(
        UpdateEma(m_state.smoothedTotalMs.load(), sample.timing.totalMs));
}

std::string SlamFrameOutputPort::BuildJsonDfxLine(
    const DfxSample &sample) const
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{\"tag\":\"slam_dfx\",";
    AppendDfxJsonCore(out, sample);
    AppendDfxJsonBackend(out, sample);
    AppendDfxJsonVisual(out, sample);
    AppendDfxJsonFrameStats(out, sample);
    AppendDfxJsonTiming(out, sample);
    out << "}";
    return out.str();
}

std::string SlamFrameOutputPort::BuildTextDfxLine(
    const DfxSample &sample) const
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << "[slam_dfx]";
    AppendDfxTextCore(out, sample);
    AppendDfxTextBackend(out, sample);
    AppendDfxTextVisual(out, sample);
    AppendDfxTextFrameStats(out, sample);
    AppendDfxTextTiming(out, sample);
    return out.str();
}

void SlamFrameOutputPort::AppendDfxJsonCore(std::ostream &out,
                                            const DfxSample &sample) const
{
    const auto &output = sample.slamOutput;
    const auto &published = sample.published;
    out << "\"frame\":" << static_cast<unsigned long long>(output.frameId)
        << ",\"state\":" << published.trackingState
        << ",\"quality\":" << static_cast<int>(sample.poseResult.quality)
        << ",\"pose_valid\":"
        << (sample.poseResult.poseEstimate.valid ? 1 : 0)
        << ",\"reset_counter\":"
        << static_cast<unsigned>(published.effectiveResetCounter)
        << ",\"reset_map_count\":"
        << static_cast<unsigned>(published.effectiveResetMapCount)
        << ",\"imu_count\":" << sample.slamInput.imu.size()
        << ",\"feat_left\":" << output.leftFeatures.size()
        << ",\"feat_right\":" << output.rightFeatures.size()
        << ",\"points\":" << published.pointCount;
}

void SlamFrameOutputPort::AppendDfxJsonBackend(std::ostream &out,
                                               const DfxSample &sample) const
{
    const auto &output = sample.slamOutput;
    out << ",\"track_points\":" << output.trackedMapPointCount
        << ",\"local_points\":" << output.localMapPointCount
        << ",\"close_points\":" << output.closeMapPointCount
        << ",\"inliers\":" << output.matchesInliers
        << ",\"orb_frame_id\":"
        << static_cast<unsigned long long>(output.orbFrameId)
        << ",\"ref_kf\":"
        << static_cast<long long>(output.referenceKeyFrameId)
        << ",\"last_kf\":" << static_cast<long long>(output.lastKeyFrameId)
        << ",\"last_kf_frame\":"
        << static_cast<long long>(output.lastKeyFrameFrameId)
        << ",\"keyframes_in_map\":" << output.keyFramesInMap
        << ",\"stereo_feature_init_frame\":"
        << output.stereoFeatureInitFrameId
        << ",\"stereo_feature_injected\":"
        << (output.stereoFeatureInjected ? 1 : 0)
        << ",\"stereo_feature_bootstrap\":"
        << (output.stereoFeatureBootstrap ? 1 : 0)
        << ",\"stereo_feature_stabilizing\":"
        << (output.stereoFeatureStabilizing ? 1 : 0)
        << ",\"realtime_pose_gate\":"
        << (output.realtimePoseQualityGate ? 1 : 0)
        << ",\"raw_pose_step_m\":" << output.rawPoseStepMeters
        << ",\"gated_pose_step_m\":" << output.gatedPoseStepMeters;
}

void SlamFrameOutputPort::AppendDfxJsonVisual(std::ostream &out,
                                              const DfxSample &sample) const
{
    const auto &output = sample.slamOutput;
    out << ",\"visual_feature_used\":"
        << (output.usedVisualFeatureFrontend ? 1 : 0)
        << ",\"visual_feature_stereo_weak\":"
        << (sample.visualFeatureStereoWeak ? 1 : 0)
        << ",\"visual_feature_raw_left\":"
        << output.visualFeatureRawLeftCount
        << ",\"visual_feature_raw_right\":"
        << output.visualFeatureRawRightCount
        << ",\"visual_feature_match_stereo\":"
        << output.visualFeatureMatchedStereoCount
        << ",\"visual_feature_injected_left\":"
        << output.visualFeatureInjectedLeftCount
        << ",\"visual_feature_injected_right\":"
        << output.visualFeatureInjectedRightCount
        << ",\"visual_feature_lg_every_n\":"
        << output.visualFeatureMatchEveryN
        << ",\"visual_feature_prepare_ms\":" << output.visualFeaturePrepareMs
        << ",\"visual_feature_input_ms\":" << output.visualFeatureInputMs
        << ",\"visual_feature_forward_ms\":" << output.visualFeatureForwardMs
        << ",\"visual_feature_frontend_ms\":"
        << output.visualFeatureFrontendMs
        << ",\"visual_feature_match_ms\":"
        << output.visualFeatureStereoMatchMs
        << ",\"visual_feature_total_ms\":" << output.visualFeatureTotalMs
        << ",\"orb_track_ms\":" << output.orbTrackMs
        << ",\"orb_extract_ms\":" << output.orbExtractMs
        << ",\"orb_stereo_ms\":" << output.orbStereoMatchMs;
}

void SlamFrameOutputPort::AppendDfxJsonFrameStats(
    std::ostream &out,
    const DfxSample &sample) const
{
    const auto &frame = sample.frame;
    const auto &output = sample.slamOutput;
    out << ",\"local_mapping_wait_ms\":" << output.localMappingWaitMs
        << ",\"local_mapping_wait_timeout_ms\":"
        << output.localMappingWaitTimeoutMs
        << ",\"local_mapping_queue_before\":"
        << output.localMappingWaitQueueBefore
        << ",\"local_mapping_queue_after\":"
        << output.localMappingWaitQueueAfter
        << ",\"local_mapping_accept_before\":"
        << (output.localMappingAcceptingBefore ? 1 : 0)
        << ",\"local_mapping_accept_after\":"
        << (output.localMappingAcceptingAfter ? 1 : 0)
        << ",\"local_mapping_wait_requested\":"
        << (output.localMappingWaitRequested ? 1 : 0)
        << ",\"local_mapping_wait_timeout\":"
        << (output.localMappingWaitTimedOut ? 1 : 0)
        << ",\"visual_feature_image_count\":"
        << output.visualFeatureImageCount
        << ",\"visual_feature_payload_bytes\":"
        << output.visualFeaturePayloadBytes
        << ",\"pair_dt_ms\":" << static_cast<double>(frame.pairDtMs)
        << ",\"reject_dt_ms\":" << frame.rejectDtMs
        << ",\"pending_left\":" << frame.pendingL
        << ",\"pending_right\":" << frame.pendingR
        << ",\"drop_left\":"
        << static_cast<unsigned long long>(frame.dropUnpairedL)
        << ",\"drop_right\":"
        << static_cast<unsigned long long>(frame.dropUnpairedR)
        << ",\"rate_drop\":"
        << static_cast<unsigned long long>(m_state.rateLimitedDrops.load());
}

void SlamFrameOutputPort::AppendDfxJsonTiming(
    std::ostream &out,
    const DfxSample &sample) const
{
    const auto &frame = sample.frame;
    const auto &timing = sample.timing;
    out << ",\"img_std_left\":" << frame.stdL
        << ",\"img_std_right\":" << frame.stdR
        << ",\"sharp_left\":" << frame.sharpL
        << ",\"sharp_right\":" << frame.sharpR
        << ",\"gap_ms\":" << frame.frameGapMs
        << ",\"mono_step_ms\":" << frame.monoStepMs
        << ",\"acquire_ms\":" << timing.acquireMs
        << ",\"imu_ms\":" << timing.imuMs
        << ",\"slam_ms\":" << timing.slamMs
        << ",\"cloud_ms\":" << timing.cloudMs
        << ",\"udp_ms\":" << timing.udpMs
        << ",\"post_ms\":" << timing.postMs
        << ",\"live_ms\":" << timing.livePoseMs
        << ",\"publish_ms\":" << timing.publishMs
        << ",\"total_ms\":" << timing.totalMs;
}

void SlamFrameOutputPort::AppendDfxTextCore(std::ostream &out,
                                            const DfxSample &sample) const
{
    const auto &output = sample.slamOutput;
    const auto &published = sample.published;
    out << " frame=" << static_cast<unsigned long long>(output.frameId)
        << " state=" << published.trackingState
        << " quality=" << static_cast<int>(sample.poseResult.quality)
        << " pose_valid="
        << (sample.poseResult.poseEstimate.valid ? 1 : 0)
        << " reset="
        << static_cast<unsigned>(published.effectiveResetCounter) << "/"
        << static_cast<unsigned>(published.effectiveResetMapCount)
        << " imu=" << sample.slamInput.imu.size()
        << " feat=" << output.leftFeatures.size() << "/"
        << output.rightFeatures.size()
        << " points=" << published.pointCount;
}

void SlamFrameOutputPort::AppendDfxTextBackend(std::ostream &out,
                                               const DfxSample &sample) const
{
    const auto &output = sample.slamOutput;
    out << " track=" << output.trackedMapPointCount
        << " local=" << output.localMapPointCount
        << " close=" << output.closeMapPointCount
        << " inliers=" << output.matchesInliers
        << " orb_frame=" << static_cast<unsigned long long>(output.orbFrameId)
        << " ref_kf="
        << static_cast<long long>(output.referenceKeyFrameId)
        << " last_kf=" << static_cast<long long>(output.lastKeyFrameId)
        << " last_kf_frame="
        << static_cast<long long>(output.lastKeyFrameFrameId)
        << " kfs=" << output.keyFramesInMap
        << " stereo_feature=init:" << output.stereoFeatureInitFrameId
        << " injected:" << (output.stereoFeatureInjected ? 1 : 0)
        << " bootstrap:" << (output.stereoFeatureBootstrap ? 1 : 0)
        << " stabilizing:" << (output.stereoFeatureStabilizing ? 1 : 0)
        << " pose_gate=" << (output.realtimePoseQualityGate ? 1 : 0)
        << " raw_step=" << std::setprecision(4) << output.rawPoseStepMeters
        << " gated_step=" << output.gatedPoseStepMeters
        << std::setprecision(3);
}

void SlamFrameOutputPort::AppendDfxTextVisual(std::ostream &out,
                                              const DfxSample &sample) const
{
    const auto &output = sample.slamOutput;
    out << " visual_feature="
        << (output.usedVisualFeatureFrontend ? "on" : "off")
        << " stereo_warn=" << (sample.visualFeatureStereoWeak ? "weak" : "ok")
        << " raw=" << output.visualFeatureRawLeftCount << "/"
        << output.visualFeatureRawRightCount
        << " stereo=" << output.visualFeatureMatchedStereoCount
        << " injected=" << output.visualFeatureInjectedLeftCount << "/"
        << output.visualFeatureInjectedRightCount
        << " lg_every_n=" << output.visualFeatureMatchEveryN
        << " visual_feature_ms=prep " << output.visualFeaturePrepareMs
        << " input " << output.visualFeatureInputMs
        << " forward " << output.visualFeatureForwardMs
        << " frontend " << output.visualFeatureFrontendMs
        << " match " << output.visualFeatureStereoMatchMs
        << " total " << output.visualFeatureTotalMs
        << " orb_ms=track " << output.orbTrackMs
        << " extract " << output.orbExtractMs
        << " stereo " << output.orbStereoMatchMs;
}

void SlamFrameOutputPort::AppendDfxTextFrameStats(
    std::ostream &out,
    const DfxSample &sample) const
{
    const auto &frame = sample.frame;
    const auto &output = sample.slamOutput;
    out << " local_mapping_wait=" << output.localMappingWaitMs
        << "ms timeout_ms=" << output.localMappingWaitTimeoutMs
        << " queue=" << output.localMappingWaitQueueBefore << "/"
        << output.localMappingWaitQueueAfter
        << " accept=" << (output.localMappingAcceptingBefore ? 1 : 0) << "/"
        << (output.localMappingAcceptingAfter ? 1 : 0)
        << " requested=" << (output.localMappingWaitRequested ? 1 : 0)
        << " timeout=" << (output.localMappingWaitTimedOut ? 1 : 0)
        << " visual_feature_io=" << output.visualFeatureImageCount
        << "img/" << output.visualFeaturePayloadBytes << "bytes"
        << " pair_dt=" << static_cast<double>(frame.pairDtMs)
        << " reject_dt=" << frame.rejectDtMs
        << " pend=" << frame.pendingL << "/" << frame.pendingR
        << " drop=" << static_cast<unsigned long long>(frame.dropUnpairedL)
        << "/" << static_cast<unsigned long long>(frame.dropUnpairedR)
        << " rate_drop="
        << static_cast<unsigned long long>(m_state.rateLimitedDrops.load());
}

void SlamFrameOutputPort::AppendDfxTextTiming(
    std::ostream &out,
    const DfxSample &sample) const
{
    const auto &frame = sample.frame;
    const auto &timing = sample.timing;
    out << " img_std=" << frame.stdL << "/" << frame.stdR
        << " sharp=" << frame.sharpL << "/" << frame.sharpR
        << " timing_ms gap=" << frame.frameGapMs
        << " mono=" << frame.monoStepMs
        << " acquire=" << timing.acquireMs
        << " imu=" << timing.imuMs
        << " slam=" << timing.slamMs
        << " cloud=" << timing.cloudMs
        << " udp=" << timing.udpMs
        << " post=" << timing.postMs
        << " live=" << timing.livePoseMs
        << " publish=" << timing.publishMs
        << " total=" << timing.totalMs;
}

} // namespace SmartDrone::core::application
