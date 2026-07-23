#include "core/application/state/pose_postprocessor.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "common/environment.h"
#include "common/time_utils.h"

namespace SmartDrone::Core::Application {

namespace {

using PoseEstimate = SmartDrone::Core::Ports::PoseEstimate;

void NormalizeQuat(float &w, float &x, float &y, float &z)
{
    const float norm = std::sqrt(w * w + x * x + y * y + z * z);
    if (norm > 1.0e-9f) {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
        return;
    }
    w = 1.0f;
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
}

PoseEstimate MakePoseEstimate(const Sophus::SE3f &pose)
{
    const Eigen::Vector3f t = pose.translation();
    const Eigen::Quaternionf q(pose.so3().unit_quaternion());
    PoseEstimate out{};
    out.valid = true;
    out.x = t.x();
    out.y = t.y();
    out.z = t.z();
    out.qw = q.w();
    out.qx = q.x();
    out.qy = q.y();
    out.qz = q.z();
    NormalizeQuat(out.qw, out.qx, out.qy, out.qz);
    return out;
}

PoseEstimate EnuToNed(const PoseEstimate &poseEnu)
{
    PoseEstimate out{};
    out.valid = poseEnu.valid;
    out.x = poseEnu.y;
    out.y = poseEnu.x;
    out.z = -poseEnu.z;

    const float s = 0.7071067811865476f;
    const Eigen::Quaternionf rotation(0.0f, s, s, 0.0f);
    const Eigen::Quaternionf input(poseEnu.qw, poseEnu.qx, poseEnu.qy, poseEnu.qz);
    const Eigen::Quaternionf converted = rotation * input * rotation.conjugate();
    out.qw = converted.w();
    out.qx = converted.x();
    out.qy = converted.y();
    out.qz = converted.z();
    NormalizeQuat(out.qw, out.qx, out.qy, out.qz);
    return out;
}

bool IsFinitePose(const PoseEstimate &pose)
{
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z) &&
           std::isfinite(pose.qw) && std::isfinite(pose.qx) && std::isfinite(pose.qy) &&
           std::isfinite(pose.qz);
}

float TranslationDistance(const PoseEstimate &a, const PoseEstimate &b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

float QuaternionAngularDistanceRad(const PoseEstimate &a, const PoseEstimate &b)
{
    float dot = a.qw * b.qw + a.qx * b.qx + a.qy * b.qy + a.qz * b.qz;
    dot = std::clamp(std::fabs(dot), 0.0f, 1.0f);
    return 2.0f * std::acos(dot);
}

void BlendQuatToward(const PoseEstimate &from, const PoseEstimate &to, float scale, PoseEstimate &out)
{
    float tw = to.qw;
    float tx = to.qx;
    float ty = to.qy;
    float tz = to.qz;
    const float dot = from.qw * tw + from.qx * tx + from.qy * ty + from.qz * tz;
    if (dot < 0.0f) {
        tw = -tw;
        tx = -tx;
        ty = -ty;
        tz = -tz;
    }
    const float a = std::clamp(scale, 0.0f, 1.0f);
    out.qw = from.qw + (tw - from.qw) * a;
    out.qx = from.qx + (tx - from.qx) * a;
    out.qy = from.qy + (ty - from.qy) * a;
    out.qz = from.qz + (tz - from.qz) * a;
    NormalizeQuat(out.qw, out.qx, out.qy, out.qz);
}

} // namespace

Sophus::SE3f PosePostprocessor::ContinuityMapper::MapPose(unsigned long mapId, bool trackingUsable,
                                                          const Sophus::SE3f &rawPoseWc)
{
    if (mapId != INVALID_MAP_ID && (!haveMapId || mapId != lastMapId)) {
        pendingReset = haveMapId;
        haveMapId = true;
        lastMapId = mapId;
    }

    if (!trackingUsable) {
        return bridgeRawToContinuous * rawPoseWc;
    }

    if (!bridgeInitialized) {
        bridgeRawToContinuous = Sophus::SE3f();
        bridgeInitialized = true;
    } else if (pendingReset) {
        if (haveLastContinuousPose) {
            bridgeRawToContinuous = lastContinuousPose * rawPoseWc.inverse();
        } else {
            bridgeRawToContinuous = Sophus::SE3f();
        }
        pendingReset = false;
        ++resetCounter;
        ++resetMapCount;
    }

    const Sophus::SE3f continuousPose = bridgeRawToContinuous * rawPoseWc;
    lastContinuousPose = continuousPose;
    haveLastContinuousPose = true;
    return continuousPose;
}

uint8_t PosePostprocessor::ContinuityMapper::GetResetCounter() const
{
    return static_cast<uint8_t>(resetCounter & 0xFFu);
}

uint16_t PosePostprocessor::ContinuityMapper::GetResetMapCount() const
{
    return static_cast<uint16_t>(resetMapCount & 0xFFFFu);
}

PoseEstimate PosePostprocessor::StartupAligner::AlignPose(const PoseEstimate &poseNed, bool trackingUsable,
                                                          const ReadRangeSensorFn &readRangeSensor,
                                                          PoseQuality &outQuality)
{
    const uint64_t nowUs = MonoTimeUs();
    RefreshRangeSensor(readRangeSensor);

    if (!trackingUsable) {
        PoseEstimate out = ComputeLostPose(nowUs);
        outQuality = PoseQuality::Lost;
        trackingUsablePrev = false;
        return out;
    }

    if (!haveZOffset) {
        const float worldZ = havePublishedPose ? holdPose.z : 0.0f;
        zOffset = worldZ - poseNed.z;
        haveZOffset = true;
        weakUntilUs = nowUs + WEAK_HOLD_US;
    }

    PoseEstimate out = poseNed;
    out.z += zOffset;

    trackingUsablePrev = true;
    lossActive = false;
    holdPose = out;
    havePublishedPose = true;
    outQuality = (nowUs < weakUntilUs) ? PoseQuality::Weak : PoseQuality::Good;
    return out;
}

void PosePostprocessor::StartupAligner::SetPublishedPose(const PoseEstimate &pose)
{
    holdPose = pose;
    havePublishedPose = true;
}

Sophus::SE3f PoseFromEstimate(const PoseEstimate &pose)
{
    Eigen::Quaternionf q(pose.qw, pose.qx, pose.qy, pose.qz);
    q.normalize();
    return Sophus::SE3f(Sophus::SO3f(q),
                        Eigen::Vector3f(pose.x, pose.y, pose.z));
}

void AppendTransformedPoint(const Sophus::SE3f &transform,
                            const float *rawPoint,
                            std::vector<float> &out)
{
    if (!std::isfinite(rawPoint[0]) || !std::isfinite(rawPoint[1]) ||
        !std::isfinite(rawPoint[2])) {
        return;
    }
    const Eigen::Vector3f point =
        transform * Eigen::Vector3f(rawPoint[0], rawPoint[1], rawPoint[2]);
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z())) {
        return;
    }
    out.push_back(point.x());
    out.push_back(point.y());
    out.push_back(point.z());
}

bool HasPointCloudInput(const PosePostprocessor::ProcessRequest &request)
{
    return request.trackingUsable && request.rawPointCloudXyz != nullptr &&
           request.rawPointCloudXyz->size() >= 3;
}

void PosePostprocessor::StartupAligner::RefreshRangeSensor(const ReadRangeSensorFn &readRangeSensor)
{
    if (!readRangeSensor) {
        return;
    }
    RangeSensorSnapshot snapshot{};
    if (!readRangeSensor(snapshot)) {
        return;
    }
    latestRange = snapshot;
    haveLatestRange = true;
}

bool PosePostprocessor::StartupAligner::HasFreshRange() const
{
    return haveLatestRange && std::isfinite(latestRange.currentDistance);
}

PoseEstimate PosePostprocessor::StartupAligner::ComputeLostPose(uint64_t nowUs)
{
    PoseEstimate out = holdPose;
    if (!lossActive) {
        lossActive = true;
        if (havePublishedPose) {
            holdPose = out;
        }
    }
    ApplyRangeProtection(out, nowUs);
    holdPose = out;
    return out;
}

void PosePostprocessor::StartupAligner::ApplyRangeProtection(PoseEstimate &, uint64_t)
{
    if (!HasFreshRange()) {
        return;
    }
    if (latestRange.signalQuality == 0 || !std::isfinite(latestRange.currentDistance)) {
        return;
    }
    if (latestRange.currentDistance >= RANGE_HARD_FLOOR_M) {
        return;
    }
}

PosePostprocessor::OutputGuard::GuardResult
PosePostprocessor::OutputGuard::BuildResult(const PoseEstimate &pose, PoseQuality quality, bool guardApplied,
                                            float rawStepM, float maxStepM) const
{
    return GuardResult{pose, quality, guardApplied, rawStepM, maxStepM};
}

PosePostprocessor::OutputGuard::GuardResult
PosePostprocessor::OutputGuard::HandleInvalidPose(const GuardRequest &request)
{
    if (!haveLastPose) {
        return BuildResult(request.pose, request.quality, false, 0.0f, 0.0f);
    }
    ++guardHitCount;
    if (SmartDrone::Common::EnvFlagEnabled(
            "SMART_DRONE_ONLINE_POSE_GUARD_DFX", false)) {
        std::cerr << "[pose_guard] invalid pose replaced with last published pose hits=" << guardHitCount << "\n";
    }
    CommitPose(lastPose, request.frameNs);
    return BuildResult(lastPose, PoseQuality::Weak, true, 0.0f, 0.0f);
}

PosePostprocessor::OutputGuard::GuardResult
PosePostprocessor::OutputGuard::ClampPose(const GuardRequest &request, float rawStepM, float maxStepM,
                                          float rawRotRad, float maxRotRad)
{
    PoseEstimate guarded = request.pose;
    if (std::isfinite(rawStepM) && rawStepM > maxStepM) {
        const float scale = maxStepM / std::max(rawStepM, 1.0e-6f);
        guarded.x = lastPose.x + (request.pose.x - lastPose.x) * scale;
        guarded.y = lastPose.y + (request.pose.y - lastPose.y) * scale;
        guarded.z = lastPose.z + (request.pose.z - lastPose.z) * scale;
    }
    if (std::isfinite(rawRotRad) && rawRotRad > maxRotRad) {
        const float scale = maxRotRad / std::max(rawRotRad, 1.0e-6f);
        BlendQuatToward(lastPose, request.pose, scale, guarded);
    } else {
        NormalizeQuat(guarded.qw, guarded.qx, guarded.qy, guarded.qz);
    }
    ++guardHitCount;
    MaybeLogClamp(request.frameNs, rawStepM, maxStepM, rawRotRad, maxRotRad);
    CommitPose(guarded, request.frameNs);
    return BuildResult(guarded, PoseQuality::Weak, true, rawStepM, maxStepM);
}

void PosePostprocessor::OutputGuard::MaybeLogClamp(int64_t frameNs, float rawStepM, float maxStepM, float rawRotRad,
                                                   float maxRotRad)
{
    if (!SmartDrone::Common::EnvFlagEnabled(
            "SMART_DRONE_ONLINE_POSE_GUARD_DFX", false)) {
        return;
    }
    const bool timeToLog = frameNs <= 0 || lastLogFrameNs <= 0 || frameNs - lastLogFrameNs > 500000000LL;
    if (guardHitCount > 10 && (guardHitCount % 20ULL) != 0ULL && !timeToLog) {
        return;
    }
    std::cerr << "[pose_guard] clamped online pose step raw_m=" << rawStepM << " max_m=" << maxStepM
              << " raw_rot_rad=" << rawRotRad << " max_rot_rad=" << maxRotRad
              << " frame_ns=" << frameNs << " hits=" << guardHitCount << "\n";
    lastLogFrameNs = frameNs;
}

PosePostprocessor::OutputGuard::GuardResult
PosePostprocessor::OutputGuard::GuardPose(const GuardRequest &request)
{
    if (!SmartDrone::Common::EnvFlagEnabled(
            "SMART_DRONE_ONLINE_POSE_STEP_GUARD", true)) {
        if (request.trackingUsable && request.quality != PoseQuality::Lost && IsFinitePose(request.pose)) {
            CommitPose(request.pose, request.frameNs);
        }
        return BuildResult(request.pose, request.quality, false, 0.0f, 0.0f);
    }

    if (!request.trackingUsable || request.quality == PoseQuality::Lost) {
        if (request.quality != PoseQuality::Lost && IsFinitePose(request.pose)) {
            CommitPose(request.pose, request.frameNs);
        }
        return BuildResult(request.pose, request.quality, false, 0.0f, 0.0f);
    }

    if (!IsFinitePose(request.pose)) {
        return HandleInvalidPose(request);
    }

    if (!haveLastPose) {
        CommitPose(request.pose, request.frameNs);
        return BuildResult(request.pose, request.quality, false, 0.0f, 0.0f);
    }

    const float rawStepM = TranslationDistance(request.pose, lastPose);
    const float maxStepM = ComputeAllowedStep(request.frameNs);
    const float rawRotRad = QuaternionAngularDistanceRad(request.pose, lastPose);
    const float maxRotRad = ComputeAllowedRotation(request.frameNs);
    if (std::isfinite(rawStepM) && rawStepM <= maxStepM &&
        std::isfinite(rawRotRad) && rawRotRad <= maxRotRad) {
        CommitPose(request.pose, request.frameNs);
        return BuildResult(request.pose, request.quality, false, rawStepM, maxStepM);
    }

    return ClampPose(request, rawStepM, maxStepM, rawRotRad, maxRotRad);
}

float PosePostprocessor::OutputGuard::ComputeAllowedStep(int64_t frameNs) const
{
    const float maxFrameStep =
        SmartDrone::Common::EnvFloatValueClamped(
            "SMART_DRONE_ONLINE_POSE_GUARD_MAX_STEP_M", 0.18f, 0.02f, 2.0f);
    const float maxSpeed =
        SmartDrone::Common::EnvFloatValueClamped(
            "SMART_DRONE_ONLINE_POSE_GUARD_MAX_SPEED_MPS", 3.0f, 0.1f, 20.0f);
    if (!haveLastPose || frameNs <= lastFrameNs) {
        return maxFrameStep;
    }

    const float dt = static_cast<float>(frameNs - lastFrameNs) * 1.0e-9f;
    if (!std::isfinite(dt) || dt < 0.005f || dt > 0.25f) {
        return maxFrameStep;
    }
    return std::max(0.02f, std::min(maxFrameStep, maxSpeed * dt));
}

float PosePostprocessor::OutputGuard::ComputeAllowedRotation(int64_t frameNs) const
{
    const float maxFrameRot =
        SmartDrone::Common::EnvFloatValueClamped(
            "SMART_DRONE_ONLINE_POSE_GUARD_MAX_ROT_RAD", 0.12f, 0.01f, 1.5f);
    const float maxRotSpeed =
        SmartDrone::Common::EnvFloatValueClamped(
            "SMART_DRONE_ONLINE_POSE_GUARD_MAX_ROT_RPS", 2.5f, 0.1f, 20.0f);
    if (!haveLastPose || frameNs <= lastFrameNs) {
        return maxFrameRot;
    }

    const float dt = static_cast<float>(frameNs - lastFrameNs) * 1.0e-9f;
    if (!std::isfinite(dt) || dt < 0.005f || dt > 0.25f) {
        return maxFrameRot;
    }
    return std::max(0.01f, std::min(maxFrameRot, maxRotSpeed * dt));
}

void PosePostprocessor::OutputGuard::CommitPose(const PoseEstimate &pose, int64_t frameNs)
{
    lastPose = pose;
    lastFrameNs = frameNs;
    haveLastPose = true;
}

SmartDrone::Core::Ports::VelocityEstimate
PosePostprocessor::VelocityTracker::Update(const PoseEstimate &pose, int64_t frameNs, PoseQuality quality,
                                           uint16_t resetMapCount)
{
    SmartDrone::Core::Ports::VelocityEstimate out{};
    if (ShouldReset(frameNs, quality, resetMapCount)) {
        ResetState(pose, frameNs, quality, resetMapCount);
        return out;
    }

    const float dt = static_cast<float>(frameNs - lastFrameNs) * 1e-9f;
    if (!(dt >= 0.005f) || !(dt <= 0.2f)) {
        ResetState(pose, frameNs, quality, resetMapCount);
        return out;
    }

    const RawVelocity rawVelocity = ComputeRawVelocity(pose, dt);
    if (!RawVelocityUsable(rawVelocity)) {
        ResetState(pose, frameNs, quality, resetMapCount);
        return out;
    }

    UpdateFilteredVelocity(rawVelocity, dt);
    out = filteredVelocity;
    CommitPoseState(pose, frameNs, quality, resetMapCount);
    return out;
}

bool PosePostprocessor::VelocityTracker::ShouldReset(int64_t frameNs, PoseQuality quality,
                                                     uint16_t resetMapCount) const
{
    return quality != PoseQuality::Good || !haveLastPose || lastQuality != PoseQuality::Good ||
           resetMapCount != lastResetMapCount || frameNs <= lastFrameNs;
}

PosePostprocessor::VelocityTracker::RawVelocity
PosePostprocessor::VelocityTracker::ComputeRawVelocity(const PoseEstimate &pose, float dt) const
{
    return RawVelocity{(pose.x - lastPose.x) / dt, (pose.y - lastPose.y) / dt, (pose.z - lastPose.z) / dt};
}

bool PosePostprocessor::VelocityTracker::RawVelocityUsable(const RawVelocity &velocity) const
{
    if (!std::isfinite(velocity.vx) || !std::isfinite(velocity.vy) || !std::isfinite(velocity.vz)) {
        return false;
    }
    return std::fabs(velocity.vx) <= MAX_HORIZONTAL_SPEED_MPS && std::fabs(velocity.vy) <= MAX_HORIZONTAL_SPEED_MPS &&
           std::fabs(velocity.vz) <= MAX_VERTICAL_SPEED_MPS;
}

void PosePostprocessor::VelocityTracker::UpdateFilteredVelocity(const RawVelocity &velocity, float dt)
{
    const float alphaXY = dt / (HORIZONTAL_TAU_SEC + dt);
    const float alphaZ = dt / (VERTICAL_TAU_SEC + dt);
    if (!haveFilteredVelocity) {
        filteredVelocity.vx = velocity.vx;
        filteredVelocity.vy = velocity.vy;
        filteredVelocity.vz = velocity.vz;
        filteredVelocity.valid = true;
        haveFilteredVelocity = true;
        return;
    }
    filteredVelocity.vx += alphaXY * (velocity.vx - filteredVelocity.vx);
    filteredVelocity.vy += alphaXY * (velocity.vy - filteredVelocity.vy);
    filteredVelocity.vz += alphaZ * (velocity.vz - filteredVelocity.vz);
}

void PosePostprocessor::VelocityTracker::CommitPoseState(const PoseEstimate &pose, int64_t frameNs,
                                                         PoseQuality quality, uint16_t resetMapCount)
{
    lastPose = pose;
    lastFrameNs = frameNs;
    lastQuality = quality;
    lastResetMapCount = resetMapCount;
    haveLastPose = true;
}

void PosePostprocessor::VelocityTracker::ResetState(const PoseEstimate &pose, int64_t frameNs, PoseQuality quality,
                                                    uint16_t resetMapCount)
{
    lastPose = pose;
    lastFrameNs = frameNs;
    lastQuality = quality;
    lastResetMapCount = resetMapCount;
    filteredVelocity = {};
    haveFilteredVelocity = false;
    haveLastPose = true;
}

PosePostprocessor::PreparedPose PosePostprocessor::PreparePose(const ProcessRequest &request)
{
    PreparedPose prepared{};
    prepared.twc = request.twcRaw;
    prepared.result.debug.cameraX = request.twcRaw.translation().x();
    prepared.result.debug.cameraY = request.twcRaw.translation().y();
    prepared.result.debug.cameraZ = request.twcRaw.translation().z();

    if (!request.useImu && request.stereoExtrinsicsLoaded) {
        // Pure stereo/LK estimates the left optical camera pose. Convert it to body FRD using T_b_c1
        // before the startup reference is applied, so published local motion is +X forward, +Y right, +Z down.
        const Sophus::SE3f rawToBody = request.stereoBodyExtrinsics.inverse();
        prepared.twc = prepared.twc * rawToBody;
        prepared.result.debug.stereoExtrinsicsApplied = true;
    }
    prepared.result.debug.bodyX = prepared.twc.translation().x();
    prepared.result.debug.bodyY = prepared.twc.translation().y();
    prepared.result.debug.bodyZ = prepared.twc.translation().z();
    return prepared;
}

void PosePostprocessor::ApplyStereoReference(const ProcessRequest &request, PreparedPose &prepared)
{
    const Sophus::SE3f beforeReference = prepared.twc;
    prepared.twc = m_continuity.MapPose(request.mapId, request.trackingUsable, prepared.twc);
    prepared.preparedToLocal = prepared.twc * beforeReference.inverse();
    prepared.result.resetCounter = m_continuity.GetResetCounter();
    prepared.result.resetMapCount = m_continuity.GetResetMapCount();

    if (!request.useImu && request.trackingUsable && request.stereoReferencePoseSet != nullptr &&
        request.stereoReferencePose != nullptr) {
        if (!*request.stereoReferencePoseSet) {
            *request.stereoReferencePose = prepared.twc;
            *request.stereoReferencePoseSet = true;
        }
        prepared.twc = request.stereoReferencePose->inverse() * prepared.twc;
        prepared.preparedToLocal =
            request.stereoReferencePose->inverse() * prepared.preparedToLocal;
        prepared.result.debug.referenceApplied = true;
    }
    prepared.result.debug.localX = prepared.twc.translation().x();
    prepared.result.debug.localY = prepared.twc.translation().y();
    prepared.result.debug.localZ = prepared.twc.translation().z();
}

void PosePostprocessor::PopulateOutputPose(const ProcessRequest &request, PreparedPose &prepared)
{
    const PoseEstimate poseSlam = MakePoseEstimate(prepared.twc);
    const PoseEstimate poseRaw = request.useImu ? EnuToNed(poseSlam) : poseSlam;
    PoseQuality quality = PoseQuality::Lost;
    const PoseEstimate aligned = m_aligner.AlignPose(poseRaw, request.trackingUsable, request.readRangeSensor, quality);
    const auto guard = m_outputGuard.GuardPose(
        PosePostprocessor::OutputGuard::GuardRequest{aligned, request.frameNs, request.trackingUsable, quality});
    if (guard.guardApplied) {
        m_aligner.SetPublishedPose(guard.pose);
    }
    const auto velocity = m_velocity.Update(guard.pose, request.frameNs, guard.quality, prepared.result.resetMapCount);

    prepared.result.poseEstimate = guard.pose;
    prepared.result.poseEstimate.valid = true;
    prepared.result.velocityEstimate = velocity;
    prepared.result.referenceFrame =
        request.useImu ? SmartDrone::Core::Ports::PoseReferenceFrame::LocalNed
                       : SmartDrone::Core::Ports::PoseReferenceFrame::LocalFrd;
    prepared.result.quality = guard.quality;
    prepared.result.debug.outputGuardApplied = guard.guardApplied;
    prepared.result.debug.outputGuardRawStepM = guard.rawStepM;
    prepared.result.debug.outputGuardMaxStepM = guard.maxStepM;
}

void PosePostprocessor::PopulatePointCloud(const ProcessRequest &request,
                                           PreparedPose &prepared)
{
    if (!HasPointCloudInput(request) ||
        prepared.result.quality == PoseQuality::Lost) {
        return;
    }
    const Sophus::SE3f rawToLocal =
        PoseFromEstimate(prepared.result.poseEstimate) * prepared.twc.inverse() *
        prepared.preparedToLocal;
    const auto &cloud = *request.rawPointCloudXyz;
    prepared.result.localPointCloudXyz.reserve(cloud.size());
    for (size_t index = 0; index + 2 < cloud.size(); index += 3) {
        AppendTransformedPoint(rawToLocal, &cloud[index],
                               prepared.result.localPointCloudXyz);
    }
}

PosePostprocessor::Result PosePostprocessor::ProcessPose(const ProcessRequest &request)
{
    PreparedPose prepared = PreparePose(request);
    ApplyStereoReference(request, prepared);
    PopulateOutputPose(request, prepared);
    PopulatePointCloud(request, prepared);
    return prepared.result;
}

} // namespace SmartDrone::Core::Application
