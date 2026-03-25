#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <cstdio>
#include <ctime>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include "ImuTypes.h"
#include "System.h"
#include "app_args.hpp"
#include "drdy_gpio.hpp"
#include "icm42688_imu.hpp"
#include "imu_buffer.hpp"
#include "logger.hpp"
#include "mavlink_pose_sender.hpp"
#include "stereo_ov9281.hpp"
#include "udp_image_sender.hpp"

#include "common/tlv/mavlink_hooks.hpp"
#include "common/tlv/tlv_cmd_router.hpp"
#include "common/tlv/tlv_pack.hpp"
#include "common/tlv/tlv_parser.hpp"
#include "common/tlv/tlv_protocol.hpp"
#include "common/tlv/udp_server.hpp"
#include <sophus/se3.hpp>

namespace fs = std::filesystem;

namespace {

enum class UnifiedMode : uint8_t { Idle = RUNTIME_MODE_IDLE, Slam = RUNTIME_MODE_SLAM, Calib = RUNTIME_MODE_CALIB };

struct CalibConfig {
    std::string root{"./calib_runs"};
    int maxFrames{0};
};

struct UnifiedConfig {
    AppConfig app;
    CalibConfig calib;
};

struct RemoteRuntimeConfig {
    int exposureUs{3000};
    float gain{2.0f};
    int pairMs{2};
    std::string udpIp;
    SensorMode sensorMode{SensorMode::Stereo};
    bool sendImage{true};
    bool sendFeature{true};
    bool sendMap{true};
};

struct MainRuntimeAliases {
    SensorMode sensorMode{SensorMode::Stereo};
    int width{}, height{}, fps{}, leftCamIndex{}, rightCamIndex{}, exposureUs{}, pairMs{}, keepMs{}, pairQueue{};
    bool aeDisable{}, requestY8{}, r16Norm{}, udpEnable{}, allowEmptyImu{}, rtImu{};
    bool sendImage{true}, sendFeature{true}, sendMap{true};
    float gain{};
    std::string udpIp, spiDev, gpiochip;
    int udpPort{}, cmdPort{}, udpJpegQ{}, udpPayload{}, udpQueue{}, imuHz{}, accelFsG{}, gyroFsDps{},
        rtPrio{};
    uint32_t spiSpeed{};
    uint8_t spiMode{}, spiBits{}, imuStartReg{};
    unsigned drdyLine{};
};

struct StereoBodyExtrinsics {
    Sophus::SE3f Tbc{Sophus::SE3f()};
    bool loaded{false};
};

struct ImuThreadState {
    ImuBuffer imuBuffer;
    std::atomic<bool> imuOk{false};
    std::atomic<uint64_t> imuCnt{0};
    std::atomic<uint64_t> imuDrop{0};
    std::atomic<float> accelLsbPerG{0.0f};
    std::atomic<float> gyroLsbPerDps{0.0f};
};

struct LivePoseState {
    struct Snapshot {
        bool hasPeer{false};
        UdpPeer peer{};
        bool poseValid{false};
        uint8_t runtimeMode{RUNTIME_MODE_IDLE};
        uint8_t trackingState{0xFF};
        OdomQualityMode odomQuality{OdomQualityMode::LOST};
        uint16_t resetCounter{0};
        uint16_t resetMapCount{0};
        float x{0.0f}, y{0.0f}, z{0.0f};
        float qw{1.0f}, qx{0.0f}, qy{0.0f}, qz{0.0f};
        uint32_t seq{0};
        std::vector<float> pointCloudXyz;
        uint32_t pointCloudSeq{0};
    };

    void UpdatePeer(const UdpPeer& peer)
    {
        if (!peer.valid) return;
        std::lock_guard<std::mutex> lock(mu);
        latestPeer = peer;
        hasPeer = true;
    }

    void SetRuntimeMode(uint8_t mode)
    {
        std::lock_guard<std::mutex> lock(mu);
        runtimeMode = mode;
        if (mode != RUNTIME_MODE_SLAM) {
            poseValid = false;
            trackingState = 0xFF;
            odomQuality = OdomQualityMode::LOST;
        }
        dirty = true;
    }

    void UpdatePose(uint8_t mode, uint8_t tracking, uint16_t resetCounterIn, uint16_t resetMapCountIn,
                    const MavlinkSerial::Pose& p, OdomQualityMode quality)
    {
        std::lock_guard<std::mutex> lock(mu);
        runtimeMode = mode;
        trackingState = tracking;
        odomQuality = quality;
        resetCounter = resetCounterIn;
        resetMapCount = resetMapCountIn;
        x = p.x; y = p.y; z = p.z;
        qw = p.qw; qx = p.qx; qy = p.qy; qz = p.qz;
        poseValid = true;
        dirty = true;
    }

    void UpdatePointCloud(std::vector<float> xyz)
    {
        std::lock_guard<std::mutex> lock(mu);
        pointCloudXyz = std::move(xyz);
        ++pointCloudSeq;
        dirty = true;
    }

    bool ConsumeSnapshot(Snapshot& out)
    {
        std::lock_guard<std::mutex> lock(mu);
        if (!hasPeer || !dirty) return false;
        out.hasPeer = hasPeer;
        out.peer = latestPeer;
        out.poseValid = poseValid;
        out.runtimeMode = runtimeMode;
        out.trackingState = trackingState;
        out.odomQuality = odomQuality;
        out.resetCounter = resetCounter;
        out.resetMapCount = resetMapCount;
        out.x = x; out.y = y; out.z = z;
        out.qw = qw; out.qx = qx; out.qy = qy; out.qz = qz;
        out.seq = ++txSeq;
        out.pointCloudXyz = pointCloudXyz;
        out.pointCloudSeq = pointCloudSeq;
        dirty = false;
        return true;
    }

    bool ReadSnapshot(Snapshot& out) const
    {
        std::lock_guard<std::mutex> lock(mu);
        if (!hasPeer) return false;
        out.hasPeer = hasPeer;
        out.peer = latestPeer;
        out.poseValid = poseValid;
        out.runtimeMode = runtimeMode;
        out.trackingState = trackingState;
        out.odomQuality = odomQuality;
        out.resetCounter = resetCounter;
        out.resetMapCount = resetMapCount;
        out.x = x; out.y = y; out.z = z;
        out.qw = qw; out.qx = qx; out.qy = qy; out.qz = qz;
        out.seq = txSeq;
        out.pointCloudXyz = pointCloudXyz;
        out.pointCloudSeq = pointCloudSeq;
        return true;
    }

    mutable std::mutex mu;
    UdpPeer latestPeer{};
    bool hasPeer{false};
    bool poseValid{false};
    uint8_t runtimeMode{RUNTIME_MODE_IDLE};
    uint8_t trackingState{0xFF};
    OdomQualityMode odomQuality{OdomQualityMode::LOST};
    uint16_t resetCounter{0};
    uint16_t resetMapCount{0};
    float x{0.0f}, y{0.0f}, z{0.0f};
    float qw{1.0f}, qx{0.0f}, qy{0.0f}, qz{0.0f};
    std::vector<float> pointCloudXyz;
    uint32_t pointCloudSeq{0};
    uint32_t txSeq{1};
    bool dirty{false};
};

constexpr uint8_t CMD_POINT_CLOUD = 0xF2;
constexpr uint16_t POINT_CLOUD_HEADER_LEN = 4;
constexpr size_t MAX_POINT_CLOUD_POINTS_TX = 120;
constexpr int64_t POINT_CLOUD_UPDATE_INTERVAL_NS = 200000000LL;

bool IsTrackingPoseUsable(int trackingState)
{
    return trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::OK_KLT;
}

bool IsOdomQualityUsable(OdomQualityMode quality)
{
    return quality != OdomQualityMode::LOST;
}

struct ImuWindowValidation {
    size_t inputCount{0};
    size_t outputCount{0};
    size_t droppedNonFinite{0};
    size_t droppedNonMonotonic{0};
    size_t droppedOutOfRange{0};
    double largestGapSec{0.0};
    double firstLeadSec{0.0};
    double tailLagSec{0.0};
    const char* failureReason{nullptr};
};

bool IsFiniteImuPoint(const ORB_SLAM3::IMU::Point& p)
{
    return std::isfinite(p.t) &&
           std::isfinite(p.a.x()) && std::isfinite(p.a.y()) && std::isfinite(p.a.z()) &&
           std::isfinite(p.w.x()) && std::isfinite(p.w.y()) && std::isfinite(p.w.z());
}

bool SanitizeImuWindow(std::vector<ORB_SLAM3::IMU::Point>& vImu,
                       double prevFrameTime,
                       double frameTime,
                       double expectedImuDtSec,
                       ImuWindowValidation& stats)
{
    constexpr float kMaxAccelNormMps2 = 200.0f;
    constexpr float kMaxGyroNormRadps = 40.0f;
    constexpr double kMinSampleDtSec = 1e-6;

    stats = ImuWindowValidation{};
    stats.inputCount = vImu.size();

    std::vector<ORB_SLAM3::IMU::Point> filtered;
    filtered.reserve(vImu.size());

    double lastT = 0.0;
    bool haveLastT = false;
    for (const auto& sample : vImu) {
        if (!IsFiniteImuPoint(sample)) {
            ++stats.droppedNonFinite;
            continue;
        }

        const float accelNorm = sample.a.norm();
        const float gyroNorm = sample.w.norm();
        if (!(accelNorm <= kMaxAccelNormMps2) || !(gyroNorm <= kMaxGyroNormRadps)) {
            ++stats.droppedOutOfRange;
            continue;
        }

        if (haveLastT) {
            const double dt = sample.t - lastT;
            if (!(dt > kMinSampleDtSec)) {
                ++stats.droppedNonMonotonic;
                continue;
            }
            stats.largestGapSec = std::max(stats.largestGapSec, dt);
        }

        filtered.push_back(sample);
        lastT = sample.t;
        haveLastT = true;
    }

    vImu.swap(filtered);
    stats.outputCount = vImu.size();

    if (vImu.size() < 2) {
        stats.failureReason = "too_few_samples";
        return false;
    }

    stats.firstLeadSec = vImu.front().t - prevFrameTime;
    stats.tailLagSec = frameTime - vImu.back().t;

    const double boundarySlackSec = std::max(6.0 * expectedImuDtSec, 0.010);
    const double maxGapSec = std::max(12.0 * expectedImuDtSec, 0.030);

    if (stats.firstLeadSec > boundarySlackSec) {
        stats.failureReason = "missing_leading_coverage";
        return false;
    }
    if (stats.tailLagSec > boundarySlackSec) {
        stats.failureReason = "missing_trailing_coverage";
        return false;
    }
    if (stats.largestGapSec > maxGapSec) {
        stats.failureReason = "large_internal_gap";
        return false;
    }

    return true;
}

struct PoseContinuityMapper {
    Sophus::SE3f MapPose(unsigned long mapId, int trackingState, const Sophus::SE3f& rawPoseWc)
    {
        if (mapId != kInvalidMapId && (!haveMapId || mapId != lastMapId)) {
            pendingReset = haveMapId;
            haveMapId = true;
            lastMapId = mapId;
        }

        if (!IsTrackingPoseUsable(trackingState)) {
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
            std::cerr << "[pose] map reset bridged map_id=" << mapId
                      << " reset_counter=" << int(GetResetCounter())
                      << " reset_map_count=" << resetMapCount << "\n";
        }

        const Sophus::SE3f continuousPose = bridgeRawToContinuous * rawPoseWc;
        lastContinuousPose = continuousPose;
        haveLastContinuousPose = true;
        return continuousPose;
    }

    uint8_t GetResetCounter() const { return static_cast<uint8_t>(resetCounter & 0xFFu); }
    uint16_t GetResetMapCount() const { return static_cast<uint16_t>(resetMapCount & 0xFFFFu); }

    static constexpr unsigned long kInvalidMapId = std::numeric_limits<unsigned long>::max();

    Sophus::SE3f bridgeRawToContinuous{Sophus::SE3f()};
    Sophus::SE3f lastContinuousPose{Sophus::SE3f()};
    bool bridgeInitialized{false};
    bool haveLastContinuousPose{false};
    bool haveMapId{false};
    bool pendingReset{false};
    unsigned long lastMapId{kInvalidMapId};
    uint32_t resetCounter{0};
    uint32_t resetMapCount{0};
};

struct VioStartupAligner {
    MavlinkSerial::Pose AlignPose(const MavlinkSerial::Pose& poseNed, bool trackingUsable,
                                  MavlinkSerial& mavlink, OdomQualityMode& outQuality)
    {
        const uint64_t nowUs = MonoTimeUs();
        RefreshPx4LocalZ(mavlink);
        RefreshRangeSensor(mavlink);

        if (!trackingUsable) {
            MavlinkSerial::Pose out = ComputeLostPose(nowUs);
            outQuality = havePublishedPose ? OdomQualityMode::WEAK : OdomQualityMode::LOST;
            trackingUsablePrev = false;
            trackingReadySinceUs = 0;
            return out;
        }

        if (!trackingUsablePrev) {
            trackingReadySinceUs = nowUs;
        }
        const bool trackingRecovered = !trackingUsablePrev;
        if (!TryAlignZ(poseNed, nowUs, trackingRecovered)) {
            MavlinkSerial::Pose out = havePublishedPose ? ComputeLostPose(nowUs) : poseNed;
            outQuality = havePublishedPose ? OdomQualityMode::WEAK : OdomQualityMode::LOST;
            trackingUsablePrev = true;
            return out;
        }

        MavlinkSerial::Pose out = poseNed;
        out.z += zOffset;

        trackingUsablePrev = true;
        lossActive = false;
        holdPose = out;
        havePublishedPose = true;
        outQuality = (!zOffsetFromPx4 || nowUs < weakUntilUs) ? OdomQualityMode::WEAK : OdomQualityMode::GOOD;
        return out;
    }

private:
    void RefreshPx4LocalZ(MavlinkSerial& mavlink)
    {
        MavlinkSerial::LocalPositionNed px4Local{};
        if (mavlink.GetLocalPositionNed(px4Local, kPx4LocalPositionMaxAgeUs)) {
            latestPx4Z = px4Local.z;
            latestPx4ZReceivedUs = px4Local.receivedUs;
            haveLatestPx4Z = true;
        }
    }

    void RefreshRangeSensor(MavlinkSerial& mavlink)
    {
        MavlinkSerial::DownwardDistanceSensor rng{};
        if (mavlink.GetDownwardDistanceSensor(rng, kRangeSensorMaxAgeUs)) {
            latestRange = rng;
            haveLatestRange = true;
        }
    }

    bool HasFreshPx4LocalZ(uint64_t nowUs) const
    {
        return haveLatestPx4Z && (nowUs - latestPx4ZReceivedUs) <= kPx4LocalPositionMaxAgeUs;
    }

    bool HasFreshRange() const
    {
        return haveLatestRange && std::isfinite(latestRange.currentDistance);
    }

    MavlinkSerial::Pose ComputeLostPose(uint64_t nowUs)
    {
        MavlinkSerial::Pose out = holdPose;

        if (!lossActive) {
            lossActive = true;
            if (havePublishedPose) {
                holdPose = out;
            }
            if ((lastLossLogUs == 0) || (nowUs - lastLossLogUs >= 1000000ULL)) {
                std::cerr << "[pose] VIO tracking lost, freezing world pose and height\n";
                lastLossLogUs = nowUs;
            }
        }

        ApplyRangeProtection(out, nowUs);
        holdPose = out;
        return out;
    }

    void ApplyRangeProtection(MavlinkSerial::Pose& pose, uint64_t nowUs)
    {
        if (!HasFreshRange()) {
            return;
        }

        // Reject terrain changes by default. Only let range sensor move the hold altitude
        // if the vehicle is getting dangerously close to the ground/obstacle.
        if (latestRange.signalQuality == 0 || !std::isfinite(latestRange.currentDistance)) {
            return;
        }

        if (latestRange.currentDistance >= kRangeHardFloorM) {
            return;
        }

        const float clearanceDeficit = std::min(kRangeProtectionMaxStepM, kRangeHardFloorM - latestRange.currentDistance);

        if ((lastRangeProtectLogUs == 0) || (nowUs - lastRangeProtectLogUs >= 500000ULL)) {
            std::cerr << "[pose] range protection alert"
                      << " rng=" << latestRange.currentDistance
                      << " hard_floor=" << kRangeHardFloorM
                      << " clearance_deficit=" << clearanceDeficit
                      << " z_override=disabled\n";
            lastRangeProtectLogUs = nowUs;
        }
    }

    bool TryAlignZ(const MavlinkSerial::Pose& poseNed, uint64_t nowUs, bool trackingRecovered)
    {
        if (haveZOffset) {
            if (!zOffsetFromPx4 && HasFreshPx4LocalZ(nowUs)) {
                zOffset = latestPx4Z - poseNed.z;
                zOffsetFromPx4 = true;
                weakUntilUs = nowUs + kWeakHoldUs;
                std::cerr << "[pose] PX4 local height became available, VIO z realigned"
                          << " px4_z=" << latestPx4Z
                          << " vio_z=" << poseNed.z
                          << " z_offset=" << zOffset << "\n";
                return true;
            }
            if ((trackingRecovered || lossActive) && havePublishedPose) {
                zOffset = holdPose.z - poseNed.z;
                weakUntilUs = nowUs + kWeakHoldUs;
                std::cerr << "[pose] VIO z realigned to held world height"
                          << " hold_z=" << holdPose.z
                          << " vio_z=" << poseNed.z
                          << " z_offset=" << zOffset << "\n";
            }
            return true;
        }

        if (HasFreshPx4LocalZ(nowUs)) {
            zOffset = latestPx4Z - poseNed.z;
            haveZOffset = true;
            zOffsetFromPx4 = true;
            weakUntilUs = nowUs + kWeakHoldUs;
            std::cerr << "[pose] VIO z aligned to PX4 local height at startup"
                      << " px4_z=" << latestPx4Z
                      << " vio_z=" << poseNed.z
                      << " z_offset=" << zOffset << "\n";
            return true;
        }

        if (trackingReadySinceUs != 0 && (nowUs - trackingReadySinceUs) >= kStartupFallbackAlignUs) {
            const float worldZ = havePublishedPose ? holdPose.z : 0.0f;
            zOffset = worldZ - poseNed.z;
            haveZOffset = true;
            zOffsetFromPx4 = false;
            weakUntilUs = nowUs + kWeakHoldUs;
            std::cerr << "[pose] PX4 LOCAL_POSITION_NED missing, fallback z alignment enabled"
                      << " world_z=" << worldZ
                      << " vio_z=" << poseNed.z
                      << " z_offset=" << zOffset << "\n";
            return true;
        }

        return false;
    }

    static constexpr uint64_t kPx4LocalPositionMaxAgeUs = 500000ULL;
    static constexpr uint64_t kRangeSensorMaxAgeUs = 200000ULL;
    static constexpr uint64_t kWeakHoldUs = 1500000ULL;
    static constexpr uint64_t kStartupFallbackAlignUs = 2000000ULL;
    static constexpr float kRangeHardFloorM = 0.35f;
    static constexpr float kRangeProtectionMaxStepM = 0.30f;

    float zOffset{0.0f};
    float latestPx4Z{0.0f};
    uint64_t weakUntilUs{0};
    uint64_t latestPx4ZReceivedUs{0};
    uint64_t lastLossLogUs{0};
    uint64_t lastRangeProtectLogUs{0};
    uint64_t trackingReadySinceUs{0};
    bool haveZOffset{false};
    bool zOffsetFromPx4{false};
    bool haveLatestPx4Z{false};
    bool lossActive{false};
    bool trackingUsablePrev{false};
    bool havePublishedPose{false};
    bool haveLatestRange{false};
    MavlinkSerial::Pose holdPose{};
    MavlinkSerial::DownwardDistanceSensor latestRange{};
};

struct OdomVelocityTracker {
    MavlinkSerial::LinearVelocityNed Update(const MavlinkSerial::Pose& pose,
                                           int64_t frameNs,
                                           OdomQualityMode quality,
                                           uint16_t resetMapCount)
    {
        MavlinkSerial::LinearVelocityNed out{};

        if (quality != OdomQualityMode::GOOD || !haveLastPose || lastQuality != OdomQualityMode::GOOD ||
            resetMapCount != lastResetMapCount || frameNs <= lastFrameNs) {
            ResetState(pose, frameNs, quality, resetMapCount);
            return out;
        }

        const float dt = static_cast<float>(frameNs - lastFrameNs) * 1e-9f;
        if (!(dt >= 0.005f) || !(dt <= 0.2f)) {
            ResetState(pose, frameNs, quality, resetMapCount);
            return out;
        }

        const float rawVx = (pose.x - lastPose.x) / dt;
        const float rawVy = (pose.y - lastPose.y) / dt;
        const float rawVz = (pose.z - lastPose.z) / dt;

        if (!std::isfinite(rawVx) || !std::isfinite(rawVy) || !std::isfinite(rawVz) ||
            std::fabs(rawVx) > kMaxHorizontalSpeedMps || std::fabs(rawVy) > kMaxHorizontalSpeedMps ||
            std::fabs(rawVz) > kMaxVerticalSpeedMps) {
            ResetState(pose, frameNs, quality, resetMapCount);
            return out;
        }

        const float alphaXY = dt / (kHorizontalTauSec + dt);
        const float alphaZ = dt / (kVerticalTauSec + dt);

        if (!haveFilteredVelocity) {
            filteredVelocity.x = rawVx;
            filteredVelocity.y = rawVy;
            filteredVelocity.z = rawVz;
            haveFilteredVelocity = true;
        } else {
            filteredVelocity.x += alphaXY * (rawVx - filteredVelocity.x);
            filteredVelocity.y += alphaXY * (rawVy - filteredVelocity.y);
            filteredVelocity.z += alphaZ * (rawVz - filteredVelocity.z);
        }

        out = filteredVelocity;
        lastPose = pose;
        lastFrameNs = frameNs;
        lastQuality = quality;
        lastResetMapCount = resetMapCount;
        haveLastPose = true;
        return out;
    }

private:
    void ResetState(const MavlinkSerial::Pose& pose,
                    int64_t frameNs,
                    OdomQualityMode quality,
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

    static constexpr float kHorizontalTauSec = 0.18f;
    static constexpr float kVerticalTauSec = 0.30f;
    static constexpr float kMaxHorizontalSpeedMps = 8.0f;
    static constexpr float kMaxVerticalSpeedMps = 4.0f;

    MavlinkSerial::Pose lastPose{};
    MavlinkSerial::LinearVelocityNed filteredVelocity{};
    int64_t lastFrameNs{0};
    OdomQualityMode lastQuality{OdomQualityMode::LOST};
    uint16_t lastResetMapCount{0};
    bool haveLastPose{false};
    bool haveFilteredVelocity{false};
};

float ClampSignedUnit(float value)
{
    return std::max(-1.0f, std::min(1.0f, value));
}

float YawFromQuat(float qw, float qx, float qy, float qz)
{
    const float siny = 2.0f * (qw * qz + qx * qy);
    const float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
    return std::atan2(siny, cosy);
}

std::optional<Sophus::SE3f> ReadSe3Node(const cv::FileNode& node)
{
    if (node.empty()) return std::nullopt;

    cv::Mat mat;
    node >> mat;
    if (mat.empty() || mat.rows != 4 || mat.cols != 4) return std::nullopt;

    cv::Mat mat32f;
    mat.convertTo(mat32f, CV_32F);
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            T(r, c) = mat32f.at<float>(r, c);
        }
    }
    return Sophus::SE3f(T);
}

StereoBodyExtrinsics LoadStereoBodyExtrinsics(const std::string& settingsPath)
{
    StereoBodyExtrinsics extrinsics;

    cv::FileStorage fs(settingsPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[pose] warning: failed to open settings for stereo body extrinsics: " << settingsPath << "\n";
        return extrinsics;
    }

    const auto maybeTbc = ReadSe3Node(fs["T_b_c1"]);
    const auto maybeImuTbc = ReadSe3Node(fs["IMU.T_b_c1"]);
    if (maybeTbc.has_value()) {
        extrinsics.Tbc = *maybeTbc;
        extrinsics.loaded = true;
    } else if (maybeImuTbc.has_value()) {
        extrinsics.Tbc = *maybeImuTbc;
        extrinsics.loaded = true;
    } else {
        std::cerr << "[pose] info: no T_b_c1/IMU.T_b_c1 in settings, pure stereo pose stays in camera frame\n";
    }

    if (extrinsics.loaded) {
        const Eigen::Vector3f t = extrinsics.Tbc.translation();
        std::cerr << "[pose] pure stereo will publish body pose using T_b_c1"
                  << " tx=" << t.x() << " ty=" << t.y() << " tz=" << t.z() << "\n";
    }

    return extrinsics;
}

class Px4UdpHooks final : public MavlinkHooks {
public:
    Px4UdpHooks(MavlinkSerial& mavlink, LivePoseState& livePose) : m_mavlink(mavlink), m_livePose(livePose)
    {
        m_manualLoop = std::thread([this]() { ManualControlLoop(); });
    }

    ~Px4UdpHooks() override
    {
        m_manualLoopStop.store(true, std::memory_order_relaxed);
        if (m_manualLoop.joinable()) {
            m_manualLoop.join();
        }
    }

    VehicleGate GetGate() const override
    {
        LivePoseState::Snapshot snapshot{};
        const bool hasSnapshot = m_livePose.ReadSnapshot(snapshot);
        const bool vioOk = hasSnapshot &&
                           snapshot.runtimeMode == RUNTIME_MODE_SLAM &&
                           snapshot.poseValid &&
                           IsTrackingPoseUsable(snapshot.trackingState) &&
                           IsOdomQualityUsable(snapshot.odomQuality);
        VehicleGate gate{};
        gate.vioOk = vioOk;
        gate.offboardReady = vioOk;
        return gate;
    }
    bool Arm(std::string* err) override
    {
        EnsureManualControlStream();
        SendManualControlSnapshot();
        if (!m_mavlink.Arm(true)) {
            if (err) *err = "px4 arm rejected";
            return false;
        }
        return true;
    }
    bool Disarm(std::string* err) override
    {
        SetManualControlNeutral();
        SendManualControlSnapshot();
        DisableRemoteControl(true);
        if (!m_mavlink.Arm(false)) {
            if (err) *err = "px4 disarm rejected";
            return false;
        }
        return true;
    }
    bool EmergencyStop(std::string* err) override
    {
        m_mavlink.StopSetpointStream();
        m_streamStarted.store(false, std::memory_order_relaxed);
        DisableRemoteControl(true);
        if (!m_mavlink.EmergencyStop()) {
            if (err) *err = "px4 emergency stop rejected";
            return false;
        }
        return true;
    }
    bool SetOffboard(std::string* err) override
    {
        EnsureManualControlStream();
        m_remoteModeRequested.store(true, std::memory_order_relaxed);
        WarmupManualControlLink();
        if (!MaybeSyncRemoteFlightMode(true, err)) {
            m_remoteModeRequested.store(false, std::memory_order_relaxed);
            if (err && err->empty()) *err = "px4 remote mode rejected";
            return false;
        }
        return true;
    }
    bool Hold(std::string* err) override
    {
        EnsureManualControlStream();
        SetManualControlNeutral();
        SendManualControlSnapshot();
        m_remoteModeRequested.store(true, std::memory_order_relaxed);
        if (!MaybeSyncRemoteFlightMode(true, err)) {
            m_remoteModeRequested.store(false, std::memory_order_relaxed);
            if (err && err->empty()) *err = "px4 hold mode rejected";
            return false;
        }
        return true;
    }
    bool Land(std::string* err) override
    {
        m_mavlink.StopSetpointStream();
        m_streamStarted.store(false, std::memory_order_relaxed);
        SetManualControlNeutral();
        DisableRemoteControl(true);
        if (!m_mavlink.SendLand()) {
            if (err) *err = "px4 land rejected";
            return false;
        }
        return true;
    }
    bool SetMoveGoal(const MoveGoal& goal, std::string* err) override
    {
        if (goal.isRcJoystick) {
            MavlinkSerial::ManualControlInput input{};
            input.throttleNorm = ClampSignedUnit(goal.throttleNorm);
            input.yawNorm = ClampSignedUnit(goal.yawNorm);
            input.pitchNorm = ClampSignedUnit(goal.pitchNorm);
            input.rollNorm = ClampSignedUnit(goal.rollNorm);

            EnsureManualControlStream();
            {
                std::lock_guard<std::mutex> lock(m_manualControlMtx);
                m_manualControlInput = input;
            }
            SendManualControlSnapshot();
            return true;
        }

        if (!EnsureSetpointStream()) {
            if (err) *err = "setpoint stream start failed";
            return false;
        }

        MavlinkSerial::SetpointLocalNED setpoint{};
        if (goal.isVelocity) {
            setpoint.x = NAN; setpoint.y = NAN; setpoint.z = NAN;
            setpoint.vx = goal.vx; setpoint.vy = goal.vy; setpoint.vz = goal.vz;
            setpoint.yaw = NAN; setpoint.yawspeed = goal.yawRate;
        } else {
            setpoint.x = goal.x; setpoint.y = goal.y; setpoint.z = goal.z;
            setpoint.vx = NAN; setpoint.vy = NAN; setpoint.vz = NAN;
            setpoint.yaw = goal.yaw; setpoint.yawspeed = NAN;
        }
        m_mavlink.UpdateStreamSetpoint(setpoint);
        return true;
    }

private:
    enum class RemoteFlightMode : uint8_t {
        Altitude = MavlinkSerial::PX4_CUSTOM_MAIN_MODE_ALTCTL,
        Position = MavlinkSerial::PX4_CUSTOM_MAIN_MODE_POSCTL,
    };

    bool IsVioControlUsable() const
    {
        LivePoseState::Snapshot snapshot{};
        return m_livePose.ReadSnapshot(snapshot) &&
               snapshot.runtimeMode == RUNTIME_MODE_SLAM &&
               snapshot.poseValid &&
               IsTrackingPoseUsable(snapshot.trackingState) &&
               IsOdomQualityUsable(snapshot.odomQuality);
    }

    RemoteFlightMode DesiredRemoteFlightMode() const
    {
        return IsVioControlUsable() ? RemoteFlightMode::Position : RemoteFlightMode::Altitude;
    }

    static const char* RemoteFlightModeToString(RemoteFlightMode mode)
    {
        return mode == RemoteFlightMode::Position ? "POSCTL" : "ALTCTL";
    }

    bool EnsureSetpointStream()
    {
        bool expected = false;
        if (m_streamStarted.compare_exchange_strong(expected, true, std::memory_order_relaxed)) {
            m_mavlink.StartSetpointStreamHz(20.0);
        }
        return true;
    }

    void EnsureManualControlStream()
    {
        m_manualControlStreaming.store(true, std::memory_order_relaxed);
    }

    void DisableRemoteControl(bool stopManualStream)
    {
        m_remoteModeRequested.store(false, std::memory_order_relaxed);
        if (stopManualStream) {
            m_manualControlStreaming.store(false, std::memory_order_relaxed);
        }
        std::lock_guard<std::mutex> lock(m_remoteModeMtx);
        m_lastRequestedRemoteMode.reset();
        m_lastRemoteModeRequest = std::chrono::steady_clock::time_point{};
    }

    void SetManualControlNeutral()
    {
        std::lock_guard<std::mutex> lock(m_manualControlMtx);
        m_manualControlInput = MavlinkSerial::ManualControlInput{};
    }

    MavlinkSerial::ManualControlInput GetManualControlSnapshot() const
    {
        std::lock_guard<std::mutex> lock(m_manualControlMtx);
        return m_manualControlInput;
    }

    void SendManualControlSnapshot()
    {
        m_mavlink.SendManualControl(GetManualControlSnapshot());
    }

    void WarmupManualControlLink()
    {
        for (int i = 0; i < 3; ++i) {
            SendManualControlSnapshot();
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }
    }

    bool MaybeSyncRemoteFlightMode(bool force, std::string* err)
    {
        const RemoteFlightMode desired = DesiredRemoteFlightMode();
        MavlinkSerial::FlightModeInfo currentMode{};
        if (m_mavlink.GetFlightModeInfo(currentMode) &&
            currentMode.mainMode == static_cast<uint8_t>(desired)) {
            return true;
        }

        const auto now = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(m_remoteModeMtx);
            if (!force &&
                m_lastRequestedRemoteMode &&
                *m_lastRequestedRemoteMode == desired &&
                (now - m_lastRemoteModeRequest) < std::chrono::milliseconds(600)) {
                return true;
            }
            m_lastRequestedRemoteMode = desired;
            m_lastRemoteModeRequest = now;
        }

        const bool ok = (desired == RemoteFlightMode::Position)
                            ? m_mavlink.SetModePosition()
                            : m_mavlink.SetModeAltitude();
        if (!ok) {
            if (err) {
                *err = std::string("px4 ") + RemoteFlightModeToString(desired) + " rejected";
            }
            return false;
        }

        std::cout << "[px4] remote manual mode -> " << RemoteFlightModeToString(desired) << "\n";
        return true;
    }

    void ManualControlLoop()
    {
        while (!m_manualLoopStop.load(std::memory_order_relaxed)) {
            if (m_manualControlStreaming.load(std::memory_order_relaxed)) {
                SendManualControlSnapshot();
            }

            if (m_remoteModeRequested.load(std::memory_order_relaxed)) {
                MaybeSyncRemoteFlightMode(false, nullptr);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    MavlinkSerial& m_mavlink;
    LivePoseState& m_livePose;
    std::atomic<bool> m_streamStarted{false};
    std::atomic<bool> m_manualControlStreaming{false};
    std::atomic<bool> m_remoteModeRequested{false};
    std::atomic<bool> m_manualLoopStop{false};
    std::thread m_manualLoop;
    mutable std::mutex m_manualControlMtx;
    MavlinkSerial::ManualControlInput m_manualControlInput{};
    mutable std::mutex m_remoteModeMtx;
    std::optional<RemoteFlightMode> m_lastRequestedRemoteMode;
    std::chrono::steady_clock::time_point m_lastRemoteModeRequest{};
};

MainRuntimeAliases BuildRuntimeAliases(const AppConfig& c)
{
    MainRuntimeAliases a{};
    a.sensorMode = c.sensorMode;
    a.width = c.camera.width; a.height = c.camera.height; a.fps = c.camera.fps;
    a.leftCamIndex = c.camera.leftCamIndex; a.rightCamIndex = c.camera.rightCamIndex;
    a.aeDisable = c.camera.aeDisable; a.exposureUs = c.camera.exposureUs; a.gain = c.camera.gain;
    a.requestY8 = c.camera.requestY8; a.r16Norm = c.camera.r16Norm; a.pairMs = c.camera.pairMs;
    a.keepMs = c.camera.keepMs; a.pairQueue = c.camera.pairQueue;
    a.udpEnable = c.udp.enable; a.udpIp = c.udp.ip; a.udpPort = c.udp.port;
    a.cmdPort = c.udp.cmdPort; a.udpJpegQ = c.udp.jpegQ; a.udpPayload = c.udp.payload;
    a.sendImage = c.udp.sendImage; a.sendFeature = c.udp.sendFeature; a.sendMap = c.udp.sendMap;
    a.udpQueue = c.udp.queue; a.spiDev = c.imu.spiDev; a.spiSpeed = c.imu.spiSpeed;
    a.spiMode = c.imu.spiMode; a.spiBits = c.imu.spiBits; a.gpiochip = c.imu.gpiochip;
    a.drdyLine = c.imu.drdyLine; a.imuHz = c.imu.imuHz; a.accelFsG = c.imu.accelFsG;
    a.gyroFsDps = c.imu.gyroFsDps; a.imuStartReg = c.imu.imuStartReg; a.allowEmptyImu = c.runtime.allowEmptyImu;
    a.rtImu = c.imu.rtImu; a.rtPrio = c.imu.rtPrio;
    return a;
}

void InstallSignalHandlers()
{
    signal(SIGINT, SigIntHandler);
    signal(SIGTERM, SigIntHandler);
}

uint16_t ReadU16Le(const uint8_t* p)
{
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t ReadU32Le(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

float ReadF32Le(const uint8_t* p)
{
    const uint32_t raw = ReadU32Le(p);
    float out = 0.0f;
    std::memcpy(&out, &raw, sizeof(out));
    return out;
}

uint32_t MonoTimeMs32()
{
    return static_cast<uint32_t>((MonoTimeUs() / 1000ULL) & 0xFFFFFFFFu);
}

std::string PeerToIpString(const UdpPeer& peer)
{
    if (!peer.valid) {
        return {};
    }
    char ipText[INET_ADDRSTRLEN] = {};
    const void* src = &(peer.addr.sin_addr);
    if (::inet_ntop(AF_INET, src, ipText, sizeof(ipText)) == nullptr) {
        return {};
    }
    return std::string(ipText);
}

bool SamePeer(const UdpPeer& a, const UdpPeer& b)
{
    if (!a.valid || !b.valid) {
        return false;
    }
    return a.addr.sin_family == b.addr.sin_family &&
           a.addr.sin_port == b.addr.sin_port &&
           a.addr.sin_addr.s_addr == b.addr.sin_addr.s_addr;
}

class CommandPeerGate {
public:
    static constexpr auto kPeerTimeout = std::chrono::seconds(5);

    bool Accept(const UdpPeer& peer, const std::chrono::steady_clock::time_point& now)
    {
        if (!peer.valid) {
            return false;
        }
        if (!m_lockedPeer.valid || (now - m_lastSeen) > kPeerTimeout) {
            m_lockedPeer = peer;
            m_lastSeen = now;
            return true;
        }
        if (SamePeer(peer, m_lockedPeer)) {
            m_lastSeen = now;
            return true;
        }
        return false;
    }

private:
    UdpPeer m_lockedPeer{};
    std::chrono::steady_clock::time_point m_lastSeen{};
};

void PrintStartupConfig(const AppConfig& app, const MainRuntimeAliases& a, UnifiedMode mode)
{
    std::cerr << "mode="
              << (mode == UnifiedMode::Slam ? "slam" : mode == UnifiedMode::Calib ? "calib" : "idle")
              << "\n";
    std::cerr << "cam " << a.width << "x" << a.height << " @" << a.fps
              << " aeDisable=" << (a.aeDisable ? "true" : "false")
              << " exp_us=" << a.exposureUs << " gain=" << a.gain << " pixelFormat=R16\n";
    std::cerr << "cam_select left_index=" << a.leftCamIndex
              << " right_index=" << a.rightCamIndex << "\n";
    std::cerr << "pair_thresh=" << a.pairMs << "ms keep_window=" << a.keepMs
              << "ms pair_queue=" << a.pairQueue << "\n";
    std::cerr << "imuHz=" << a.imuHz << " udp=" << (a.udpEnable ? "Y" : "N")
              << " udpPort=" << a.udpPort << " cmdPort=" << a.cmdPort << "\n";
    std::cerr << "stream img=" << (a.sendImage ? "Y" : "N")
              << " feat=" << (a.sendFeature ? "Y" : "N")
              << " map=" << (a.sendMap ? "Y" : "N") << "\n";
    std::cerr << "vocab=" << app.vocab << "\nsettings=" << app.settings << "\n";
}

void EnsureDir(const fs::path& p)
{
    std::error_code ec;
    fs::create_directories(p, ec);
}

std::string TsToName(int64_t tNs) { return std::to_string(tNs) + ".png"; }

void SetupFileBuffer(FILE* f, size_t bytes)
{
    if (f) setvbuf(f, nullptr, _IOFBF, bytes);
}

bool TryParseCalibIndex(const std::string& name, int& indexOut)
{
    static const std::string prefix = "calib_data_";
    if (name.rfind(prefix, 0) != 0) {
        return false;
    }
    const std::string suffix = name.substr(prefix.size());
    if (suffix.empty()) {
        return false;
    }
    for (char c : suffix) {
        if (c < '0' || c > '9') {
            return false;
        }
    }
    try {
        indexOut = std::stoi(suffix);
        return indexOut >= 0;
    } catch (...) {
        return false;
    }
}

std::string MakeCalibSessionDir(const std::string& root)
{
    EnsureDir(fs::path(root));
    int maxIndex = -1;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(root, ec)) {
        if (ec || !entry.is_directory()) {
            continue;
        }
        int index = -1;
        if (TryParseCalibIndex(entry.path().filename().string(), index)) {
            maxIndex = std::max(maxIndex, index);
        }
    }
    return (fs::path(root) / ("calib_data_" + std::to_string(maxIndex + 1))).string();
}

int CleanupCalibDataDirs(const std::string& root)
{
    int removed = 0;
    std::error_code ec;
    if (!fs::exists(root, ec)) {
        return 0;
    }
    for (const auto& entry : fs::directory_iterator(root, ec)) {
        if (ec || !entry.is_directory()) {
            continue;
        }
        int index = -1;
        if (!TryParseCalibIndex(entry.path().filename().string(), index)) {
            continue;
        }
        std::error_code rmEc;
        const auto n = fs::remove_all(entry.path(), rmEc);
        if (!rmEc && n > 0) {
            removed++;
        }
    }
    return removed;
}

std::thread StartImuThread(const MainRuntimeAliases& a, ImuThreadState& s, std::atomic<bool>& stop)
{
    ImuScale init{};
    s.accelLsbPerG.store(init.accelLsbPerG);
    s.gyroLsbPerDps.store(init.gyroLsbPerDps);
    return std::thread([&a, &s, &stop]() {
        if (a.rtImu) SetThreadRealtime(a.rtPrio);
        SpiDev spi(a.spiDev);
        if (!spi.Open(a.spiSpeed, a.spiMode, a.spiBits)) return;
        ImuScale scale{};
        if (!IcmResetAndConfig(spi, a.imuHz, a.accelFsG, a.gyroFsDps, scale)) return;
        s.accelLsbPerG.store(scale.accelLsbPerG);
        s.gyroLsbPerDps.store(scale.gyroLsbPerDps);
        DrdyGpio drdy;
        if (!drdy.Open(a.gpiochip, a.drdyLine)) return;
        s.imuOk.store(true);
        uint8_t raw12[12]{}; uint8_t st = 0; spi.ReadReg(REG_INT_STATUS, st);
        int64_t lastAcceptedTsNs = 0;
        uint64_t lastNonMonotonicLogUs = 0;
        while (g_runningFlag.load() && !stop.load()) {
            int64_t tNs = 0;
            if (!drdy.WaitTs(1000, tNs)) continue;
            if (lastAcceptedTsNs != 0 && tNs <= lastAcceptedTsNs) {
                s.imuDrop.fetch_add(1, std::memory_order_relaxed);
                const uint64_t nowUs = MonoTimeUs();
                if ((lastNonMonotonicLogUs == 0) || (nowUs - lastNonMonotonicLogUs >= 1000000ULL)) {
                    std::cerr << "[imu] dropped non-monotonic DRDY timestamp"
                              << " prev_ns=" << lastAcceptedTsNs
                              << " cur_ns=" << tNs
                              << "\n";
                    lastNonMonotonicLogUs = nowUs;
                }
                continue;
            }
            ImuSample sample{}; sample.tNs = tNs;
            spi.ReadReg(REG_INT_STATUS, st);
            if (!spi.ReadRegs(a.imuStartReg, raw12, sizeof(raw12))) {
                s.imuDrop.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            ImuScale cur{}; cur.accelLsbPerG = s.accelLsbPerG.load(); cur.gyroLsbPerDps = s.gyroLsbPerDps.load();
            ConvertRaw12AccelGyroToSi(raw12, cur, sample);
            s.imuBuffer.Push(sample);
            lastAcceptedTsNs = tNs;
            s.imuCnt.fetch_add(1, std::memory_order_relaxed);
        }
    });
}

std::thread StartCalibImuWriterThread(const MainRuntimeAliases& a, FILE* fImu, std::atomic<bool>& imuOk, std::atomic<bool>& stop)
{
    return std::thread([&a, fImu, &imuOk, &stop]() {
        SpiDev spi(a.spiDev);
        if (!spi.Open(a.spiSpeed, a.spiMode, a.spiBits)) return;
        ImuScale scale{};
        if (!IcmResetAndConfig(spi, a.imuHz, a.accelFsG, a.gyroFsDps, scale)) return;
        DrdyGpio drdy;
        if (!drdy.Open(a.gpiochip, a.drdyLine)) return;
        imuOk.store(true);
        uint8_t raw12[12]{}; uint8_t st = 0; spi.ReadReg(REG_INT_STATUS, st);
        int lines = 0;
        while (g_runningFlag.load() && !stop.load()) {
            int64_t tNs = 0;
            if (!drdy.WaitTs(1000, tNs)) continue;
            ImuSample sample{}; sample.tNs = tNs;
            spi.ReadReg(REG_INT_STATUS, st);
            if (!spi.ReadRegs(a.imuStartReg, raw12, sizeof(raw12))) continue;
            ConvertRaw12AccelGyroToSi(raw12, scale, sample);
            std::fprintf(fImu, "%lld,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                (long long)sample.tNs, (double)sample.gx, (double)sample.gy, (double)sample.gz,
                (double)sample.ax, (double)sample.ay, (double)sample.az);
            if ((++lines % 800) == 0) std::fflush(fImu);
        }
        std::fflush(fImu);
    });
}

bool OpenCamera(LibcameraStereoOV9281_TsPair& cam, const MainRuntimeAliases& a)
{
    return cam.Open(a.width, a.height, a.fps, a.aeDisable, a.exposureUs, a.gain, a.requestY8,
                    (int64_t)a.pairMs * 1000000LL, (int64_t)a.keepMs * 1000000LL, a.pairQueue,
                    a.r16Norm, a.leftCamIndex, a.rightCamIndex);
}

bool RunSlamSession(const UnifiedConfig& cfg, MavlinkSerial& mav, std::atomic<bool>& stop, LivePoseState& livePose)
{
    const MainRuntimeAliases a = BuildRuntimeAliases(cfg.app);
    PrintStartupConfig(cfg.app, a, UnifiedMode::Slam);
    livePose.SetRuntimeMode(RUNTIME_MODE_SLAM);
    Logger::Init("./stereo_vslam.log", 32 * 1024 * 1024, Logger::INFO, true);
    ORB_SLAM3::System SLAM(cfg.app.vocab, cfg.app.settings,
                           a.sensorMode == SensorMode::StereoImu ? ORB_SLAM3::System::IMU_STEREO
                                                                 : ORB_SLAM3::System::STEREO,
                           false);
    const StereoBodyExtrinsics stereoBodyExtrinsics =
        (a.sensorMode == SensorMode::Stereo) ? LoadStereoBodyExtrinsics(cfg.app.settings) : StereoBodyExtrinsics{};
    UdpImageSender udp;
    if (a.udpEnable && (a.sendImage || a.sendFeature)) {
        udp.Open(a.udpIp, a.udpPort, a.udpJpegQ, a.udpPayload, a.udpQueue);
    }
    ImuThreadState imuState;
    std::thread imuThread;
    const bool useImu = (a.sensorMode == SensorMode::StereoImu);
    if (useImu) imuThread = StartImuThread(a, imuState, stop);
    LibcameraStereoOV9281_TsPair cam;
    if (!OpenCamera(cam, a)) {
        stop.store(true);
        if (imuThread.joinable()) imuThread.join();
        SLAM.Shutdown();
        return false;
    }
    int64_t lastFrameNs = 0;
    const int64_t frameStepNs = 1000000000LL / std::max(1, a.fps);
    const int64_t imuDtNs = 1000000000LL / std::max(1, a.imuHz);
    const int64_t slackBeforeNs = std::max<int64_t>(2 * imuDtNs, 5000000);
    const int64_t slackAfterNs = std::max<int64_t>(2 * imuDtNs, 5000000);
    const uint64_t imuWarmupSamples = static_cast<uint64_t>(std::max(20, a.imuHz / 2));
    auto lastImuWindowLog = std::chrono::steady_clock::now();
    auto lastImuWarmupLog = std::chrono::steady_clock::now();
    auto lastImuRejectLog = std::chrono::steady_clock::now();
    int64_t lastPointCloudUpdateNs = 0;
    Sophus::SE3f stereoReferencePose{Sophus::SE3f()};
    bool stereoReferencePoseSet = false;
    unsigned long lastRawMapId = PoseContinuityMapper::kInvalidMapId;
    PoseContinuityMapper continuityMapper{};
    VioStartupAligner startupAligner{};
    OdomVelocityTracker odomVelocityTracker{};
    bool sessionOk = true;
    while (g_runningFlag.load() && !stop.load()) {
        if (useImu) {
            const uint64_t imuCnt = imuState.imuCnt.load(std::memory_order_relaxed);
            const bool imuReady = imuState.imuOk.load(std::memory_order_relaxed) && imuCnt >= imuWarmupSamples;
            if (!imuReady) {
                const auto now = std::chrono::steady_clock::now();
                if (now - lastImuWarmupLog >= std::chrono::seconds(1)) {
                    std::cerr << "[imu-warmup] waiting imu_ok="
                              << (imuState.imuOk.load(std::memory_order_relaxed) ? "true" : "false")
                              << " imu_cnt=" << imuCnt
                              << " need=" << imuWarmupSamples
                              << "\n";
                    lastImuWarmupLog = now;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
        }
        FrameItem L, R;
        if (!cam.GrabPair(L, R, 1000, true)) {
            if (!cam.Healthy()) {
                std::cerr << "[slam] camera pipeline unhealthy, aborting session\n";
                sessionOk = false;
                break;
            }
            continue;
        }
        int64_t frameNs = (int64_t)((L.tsNs + R.tsNs) / 2);
        if (lastFrameNs != 0 && frameNs <= lastFrameNs) frameNs = lastFrameNs + frameStepNs;
        std::vector<ORB_SLAM3::IMU::Point> vImu;
        if (useImu && lastFrameNs != 0) {
            vImu = imuState.imuBuffer.PopBetweenNs(lastFrameNs, frameNs, slackBeforeNs, slackAfterNs);
            ImuWindowValidation imuWindow{};
            const double prevFrameTime = static_cast<double>(lastFrameNs) * 1e-9;
            const double frameTimeSec = static_cast<double>(frameNs) * 1e-9;
            const double expectedImuDtSec = 1.0 / std::max(1, a.imuHz);
            const bool imuWindowOk = SanitizeImuWindow(vImu, prevFrameTime, frameTimeSec, expectedImuDtSec, imuWindow);
            const auto now = std::chrono::steady_clock::now();
            if (now - lastImuWindowLog >= std::chrono::seconds(1)) {
                std::cerr << "[imu-win] frame_dt_ms=" << ((frameNs - lastFrameNs) / 1e6)
                          << " slack_before_ms=" << (slackBeforeNs / 1e6)
                          << " slack_after_ms=" << (slackAfterNs / 1e6)
                          << " imu_samples=" << vImu.size()
                          << " imu_cnt=" << imuState.imuCnt.load(std::memory_order_relaxed)
                          << " imu_drop=" << imuState.imuDrop.load(std::memory_order_relaxed)
                          << " first_offset_ms=" << (vImu.empty() ? 0.0 : ((vImu.front().t - prevFrameTime) * 1e3))
                          << " last_offset_ms=" << (vImu.empty() ? 0.0 : ((frameTimeSec - vImu.back().t) * 1e3))
                          << " max_gap_ms=" << (imuWindow.largestGapSec * 1e3)
                          << " empty=" << (vImu.empty() ? "true" : "false")
                          << "\n";
                lastImuWindowLog = now;
            }
            if (!imuWindowOk) {
                if (now - lastImuRejectLog >= std::chrono::seconds(1)) {
                    std::cerr << "[imu-win] rejected"
                              << " reason=" << (imuWindow.failureReason ? imuWindow.failureReason : "unknown")
                              << " in=" << imuWindow.inputCount
                              << " out=" << imuWindow.outputCount
                              << " drop_non_finite=" << imuWindow.droppedNonFinite
                              << " drop_non_mono=" << imuWindow.droppedNonMonotonic
                              << " drop_range=" << imuWindow.droppedOutOfRange
                              << " first_offset_ms=" << (imuWindow.firstLeadSec * 1e3)
                              << " last_offset_ms=" << (imuWindow.tailLagSec * 1e3)
                              << " max_gap_ms=" << (imuWindow.largestGapSec * 1e3)
                              << "\n";
                    lastImuRejectLog = now;
                }
                continue;
            }
            if (vImu.empty() && !a.allowEmptyImu) continue;
        }
        lastFrameNs = frameNs;
        const double frameTime = (double)frameNs * 1e-9;
        Sophus::SE3f Tcw = useImu ? SLAM.TrackStereo(L.gray, R.gray, frameTime, vImu)
                                  : SLAM.TrackStereo(L.gray, R.gray, frameTime);
        std::vector<cv::Point2f> trackedLeftFeatures;
        std::vector<cv::Point2f> trackedRightFeatures;
        if (a.udpEnable && (a.sendImage || a.sendFeature || a.sendMap)) {
            const bool updatePointCloud =
                a.sendMap && (frameNs - lastPointCloudUpdateNs) >= POINT_CLOUD_UPDATE_INTERVAL_NS;
            ORB_SLAM3::TrackedVisualData visual = SLAM.ExtractTrackedVisualData(
                L.gray.cols,
                L.gray.rows,
                R.gray.cols,
                R.gray.rows,
                updatePointCloud,
                MAX_POINT_CLOUD_POINTS_TX);
            trackedLeftFeatures = std::move(visual.leftFeatures);
            trackedRightFeatures = std::move(visual.rightFeatures);
            if (updatePointCloud) {
                livePose.UpdatePointCloud(std::move(visual.pointCloudXyz));
                lastPointCloudUpdateNs = frameNs;
            }
            udp.Enqueue(0, L.seq, frameTime, L.gray, trackedLeftFeatures, a.sendImage, a.sendFeature);
            udp.Enqueue(1, R.seq, frameTime, R.gray, trackedRightFeatures, a.sendImage, a.sendFeature);
        }
        const int state = SLAM.GetTrackingState();
        const bool trackingUsable = IsTrackingPoseUsable(state);
        const unsigned long mapId = SLAM.GetCurrentMapId();
        const bool mapIdChanged = mapId != PoseContinuityMapper::kInvalidMapId && mapId != lastRawMapId;
        if (mapIdChanged) {
            lastRawMapId = mapId;
            if (!useImu) {
                stereoReferencePoseSet = false;
            }
        }
        Sophus::SE3f Twc = Tcw.inverse();
        if (!useImu && stereoBodyExtrinsics.loaded) {
            Twc = Twc * stereoBodyExtrinsics.Tbc.inverse();
        }
        if (!useImu && trackingUsable) {
            if (!stereoReferencePoseSet) {
                stereoReferencePose = Twc;
                stereoReferencePoseSet = true;
            }
            Twc = stereoReferencePose.inverse() * Twc;
        }
        const Sophus::SE3f TwcContinuous = continuityMapper.MapPose(mapId, state, Twc);
        const Eigen::Vector3f t = TwcContinuous.translation();
        const Eigen::Quaternionf q(TwcContinuous.so3().unit_quaternion());
        MavlinkSerial::Pose pSlam{};
        pSlam.x = t.x(); pSlam.y = t.y(); pSlam.z = t.z();
        pSlam.qw = q.w(); pSlam.qx = q.x(); pSlam.qy = q.y(); pSlam.qz = q.z();
        MavlinkSerial::NormalizeQuat(pSlam.qw, pSlam.qx, pSlam.qy, pSlam.qz);
        const MavlinkSerial::Pose pRaw = useImu ? MavlinkSerial::EnuToNed(pSlam) : pSlam;
        OdomQualityMode odomQuality = OdomQualityMode::LOST;
        const MavlinkSerial::Pose p = startupAligner.AlignPose(pRaw, trackingUsable, mav, odomQuality);
        const MavlinkSerial::LinearVelocityNed velNed =
            odomVelocityTracker.Update(p, frameNs, odomQuality, continuityMapper.GetResetMapCount());
        livePose.UpdatePose(RUNTIME_MODE_SLAM, static_cast<uint8_t>(state),
                            continuityMapper.GetResetCounter(), continuityMapper.GetResetMapCount(), p, odomQuality);
        mav.SendOdometry(MonoTimeUs(), p, velNed, MAV_FRAME_LOCAL_NED, MAV_FRAME_BODY_FRD,
                         continuityMapper.GetResetCounter(),
                         odomQuality);
    }
    cam.Close();
    std::cerr << "[session] slam camera closed\n";
    if (imuThread.joinable()) imuThread.join();
    std::cerr << "[session] slam imu joined\n";
    if (a.udpEnable && (a.sendImage || a.sendFeature)) {
        udp.Close();
        std::cerr << "[session] slam udp closed\n";
    }
    mav.StopSetpointStream();
    std::cerr << "[session] slam setpoint stopped\n";
    SLAM.Shutdown();
    std::cerr << "[session] slam shutdown complete\n";
    livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
    std::cerr << "[session] slam exit\n";
    return sessionOk;
}

bool RunCalibSession(const UnifiedConfig& cfg, std::atomic<bool>& stop, LivePoseState& livePose)
{
    const MainRuntimeAliases a = BuildRuntimeAliases(cfg.app);
    PrintStartupConfig(cfg.app, a, UnifiedMode::Calib);
    livePose.SetRuntimeMode(RUNTIME_MODE_CALIB);
    const std::string outRoot = MakeCalibSessionDir(cfg.calib.root);
    const fs::path root(outRoot), cam0Dir = root / "cam0", cam1Dir = root / "cam1";
    EnsureDir(cam0Dir); EnsureDir(cam1Dir);
    FILE* fCam0 = std::fopen((cam0Dir / "data.csv").string().c_str(), "w");
    FILE* fCam1 = std::fopen((cam1Dir / "data.csv").string().c_str(), "w");
    FILE* fImu = std::fopen((root / "imu.csv").string().c_str(), "w");
    if (!fCam0 || !fCam1 || !fImu) return false;
    std::cerr << "[calib] out=" << outRoot << "\n";
    SetupFileBuffer(fCam0, 1 << 20); SetupFileBuffer(fCam1, 1 << 20); SetupFileBuffer(fImu, 4 << 20);
    std::fprintf(fCam0, "#timestamp [ns],filename\n");
    std::fprintf(fCam1, "#timestamp [ns],filename\n");
    std::fprintf(fImu, "#timestamp [ns],wX [rad/s],wY [rad/s],wZ [rad/s],aX [m/s^2],aY [m/s^2],aZ [m/s^2]\n");
    UdpImageSender udp;
    if (a.udpEnable && a.sendImage) udp.Open(a.udpIp, a.udpPort, a.udpJpegQ, a.udpPayload, a.udpQueue);
    std::atomic<bool> imuOk{false};
    std::thread imuThread = StartCalibImuWriterThread(a, fImu, imuOk, stop);
    LibcameraStereoOV9281_TsPair cam;
    if (!OpenCamera(cam, a)) {
        stop.store(true);
        if (imuThread.joinable()) imuThread.join();
        std::fclose(fCam0); std::fclose(fCam1); std::fclose(fImu);
        return false;
    }
    int saved = 0; int64_t lastPairNs = 0;
    const int64_t maxSaveDtNs = (int64_t)std::max(a.pairMs, 1) * 1000000LL;
    bool sessionOk = true;
    while (g_runningFlag.load() && !stop.load()) {
        if (cfg.calib.maxFrames > 0 && saved >= cfg.calib.maxFrames) break;
        FrameItem L, R;
        if (!cam.GrabPair(L, R, 1000)) {
            if (!cam.Healthy()) {
                std::cerr << "[calib] camera pipeline unhealthy, aborting session\n";
                sessionOk = false;
                break;
            }
            continue;
        }
        const int64_t absDtLr = Abs64((int64_t)L.tsNs - (int64_t)R.tsNs);
        if (absDtLr > maxSaveDtNs) {
            static int droppedWide = 0;
            ++droppedWide;
            if ((droppedWide % 10) == 1) {
                std::cerr << "[calib-drop] dt_lr_us=" << (absDtLr / 1000.0)
                          << " exceeds max_save_dt_us=" << (maxSaveDtNs / 1000.0)
                          << " dropped=" << droppedWide
                          << "\n";
            }
            continue;
        }
        int64_t pairNs = (int64_t)((L.tsNs + R.tsNs) / 2);
        if (lastPairNs != 0 && pairNs <= lastPairNs) pairNs = lastPairNs + 1;
        lastPairNs = pairNs;
        const std::string name = TsToName(pairNs);
        const fs::path fnL = cam0Dir / name, fnR = cam1Dir / name;
        if (L.gray.empty() || R.gray.empty()) {
            std::cerr << "[calib-write] empty image"
                      << " seqL=" << L.seq
                      << " seqR=" << R.seq
                      << " rowsL=" << L.gray.rows
                      << " colsL=" << L.gray.cols
                      << " rowsR=" << R.gray.rows
                      << " colsR=" << R.gray.cols
                      << "\n";
            continue;
        }
        const bool okL = cv::imwrite(fnL.string(), L.gray);
        const bool okR = cv::imwrite(fnR.string(), R.gray);
        if (!okL || !okR) {
            std::cerr << "[calib-write] imwrite failed"
                      << " okL=" << (okL ? "true" : "false")
                      << " okR=" << (okR ? "true" : "false")
                      << " pathL=" << fnL.string()
                      << " pathR=" << fnR.string()
                      << " typeL=" << L.gray.type()
                      << " typeR=" << R.gray.type()
                      << " rowsL=" << L.gray.rows
                      << " colsL=" << L.gray.cols
                      << " rowsR=" << R.gray.rows
                      << " colsR=" << R.gray.cols
                      << "\n";
            continue;
        }
        std::fprintf(fCam0, "%lld,%s\n", (long long)pairNs, name.c_str());
        std::fprintf(fCam1, "%lld,%s\n", (long long)pairNs, name.c_str());
        if (a.udpEnable && a.sendImage) {
            udp.Enqueue(0, L.seq, pairNs * 1e-9, L.gray, {}, true, false);
            udp.Enqueue(1, R.seq, pairNs * 1e-9, R.gray, {}, true, false);
        }
        if ((saved % 30) == 0) {
            std::cerr << "[calib-save] saved=" << (saved + 1)
                      << " pathL=" << fnL.string()
                      << " pathR=" << fnR.string()
                      << "\n";
        }
        if ((++saved % 50) == 0) {
            std::fflush(fCam0); std::fflush(fCam1);
        }
    }
    cam.Close();
    std::cerr << "[session] calib camera closed\n";
    stop.store(true);
    if (imuThread.joinable()) imuThread.join();
    std::cerr << "[session] calib imu joined\n";
    if (a.udpEnable && a.sendImage) {
        udp.Close();
        std::cerr << "[session] calib udp closed\n";
    }
    std::fflush(fCam0); std::fflush(fCam1); std::fflush(fImu);
    std::fclose(fCam0); std::fclose(fCam1); std::fclose(fImu);
    std::cerr << "[calib] out=" << outRoot << " saved=" << saved << " imuOk=" << (imuOk.load() ? "true" : "false") << "\n";
    livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
    std::cerr << "[session] calib exit\n";
    return sessionOk;
}

class UnifiedRuntimeController {
public:
    UnifiedRuntimeController(UnifiedConfig initialConfig, MavlinkSerial& mav, LivePoseState& livePose)
        : m_config(std::move(initialConfig)), m_mav(mav), m_livePose(livePose) {}
    void Start() { m_worker = std::thread([this]() { Loop(); }); }
    void Stop()
    {
        {
            std::lock_guard<std::mutex> lock(m_mu);
            m_stopping = true;
            m_desiredMode = UnifiedMode::Idle;
            m_sessionStop.store(true);
        }
        m_cv.notify_all();
        if (m_worker.joinable()) m_worker.join();
        JoinSession();
    }
    bool SetMode(UnifiedMode mode, std::string* err)
    {
        std::lock_guard<std::mutex> lock(m_mu);
        if (m_stopping) {
            if (err) *err = "runtime stopping";
            return false;
        }
        m_desiredMode = mode;
        m_livePose.SetRuntimeMode(static_cast<uint8_t>(mode));
        m_restartRequested = true;
        m_cv.notify_all();
        return true;
    }
    bool UpdateRemoteConfig(const RemoteRuntimeConfig& r, std::string* err)
    {
        if (r.exposureUs <= 0 || !(r.gain > 0.0f) || r.pairMs <= 0) {
            if (err) *err = "bad runtime config";
            return false;
        }
        std::lock_guard<std::mutex> lock(m_mu);
        CameraConfig& cam = m_config.app.camera;
        cam.exposureUs = r.exposureUs;
        cam.gain = r.gain;
        cam.pairMs = r.pairMs;
        m_config.app.sensorMode = r.sensorMode;
        m_config.app.settings = DefaultSettingsForSensorMode(r.sensorMode);
        m_config.app.udp.ip = r.udpIp;
        m_config.app.udp.enable = !r.udpIp.empty();
        m_config.app.udp.sendImage = r.sendImage;
        m_config.app.udp.sendFeature = r.sendFeature;
        m_config.app.udp.sendMap = r.sendMap;
        if (m_desiredMode != UnifiedMode::Idle) {
            m_restartRequested = true;
            m_sessionStop.store(true);
        }
        m_cv.notify_all();
        return true;
    }
    bool CleanupCalibData(std::string* msg)
    {
        {
            std::unique_lock<std::mutex> lock(m_mu);
            if (m_stopping) {
                if (msg) *msg = "runtime stopping";
                return false;
            }

            // Allow "StopCalib -> CleanCalib" back-to-back by forcing the runtime
            // into idle and waiting for the session thread to fully join.
            if (m_activeMode != UnifiedMode::Idle || m_desiredMode != UnifiedMode::Idle || m_session.joinable()) {
                m_desiredMode = UnifiedMode::Idle;
                m_restartRequested = true;
                m_sessionStop.store(true);
                m_cv.notify_all();
                const bool idleReady = m_cv.wait_for(lock, std::chrono::seconds(5), [this]() {
                    return m_stopping
                        || (m_activeMode == UnifiedMode::Idle
                            && m_desiredMode == UnifiedMode::Idle
                            && !m_session.joinable()
                            && !m_sessionDone);
                });
                if (!idleReady) {
                    if (msg) *msg = "runtime busy";
                    return false;
                }
                if (m_stopping) {
                    if (msg) *msg = "runtime stopping";
                    return false;
                }
            }
        }
        const int removed = CleanupCalibDataDirs(m_config.calib.root);
        if (msg) *msg = "calib clean removed=" + std::to_string(removed);
        return true;
    }
    UnifiedConfig CurrentConfig()
    {
        std::lock_guard<std::mutex> lock(m_mu);
        return m_config;
    }

private:
    void JoinSession() { if (m_session.joinable()) m_session.join(); }
    void Loop()
    {
        while (g_runningFlag.load()) {
            UnifiedMode startMode = UnifiedMode::Idle;
            UnifiedConfig cfg{};
            bool startSession = false, needJoin = false;
            {
                std::unique_lock<std::mutex> lock(m_mu);
                m_cv.wait_for(lock, std::chrono::milliseconds(100), [this]() {
                    return m_stopping || m_restartRequested || m_sessionDone;
                });
                if (m_sessionDone) {
                    needJoin = true;
                }
                if (m_stopping) m_sessionStop.store(true);
                if ((m_restartRequested || m_desiredMode != m_activeMode) && m_activeMode != UnifiedMode::Idle) {
                    std::cerr << "[runtime] request stop active="
                              << (m_activeMode == UnifiedMode::Slam ? "slam" : m_activeMode == UnifiedMode::Calib ? "calib" : "idle")
                              << " desired="
                              << (m_desiredMode == UnifiedMode::Slam ? "slam" : m_desiredMode == UnifiedMode::Calib ? "calib" : "idle")
                              << " restart=" << (m_restartRequested ? "true" : "false") << "\n";
                    m_sessionStop.store(true);
                    needJoin = true;
                }
            }
            if (needJoin) {
                std::cerr << "[runtime] joining session\n";
                JoinSession();
                std::cerr << "[runtime] session joined\n";
            }
            {
                std::lock_guard<std::mutex> lock(m_mu);
                if (needJoin) {
                    m_sessionDone = false;
                    m_activeMode = UnifiedMode::Idle;
                    m_cv.notify_all();
                }
                if (m_stopping) break;
                if (m_activeMode != UnifiedMode::Idle && m_desiredMode == m_activeMode && !m_restartRequested) continue;
                m_activeMode = UnifiedMode::Idle;
                if (m_desiredMode != UnifiedMode::Idle) {
                    cfg = m_config;
                    startMode = m_desiredMode;
                    m_activeMode = startMode;
                    m_sessionStop.store(false);
                    startSession = true;
                }
                m_restartRequested = false;
            }
            if (startSession) {
                std::cerr << "[runtime] starting session mode="
                          << (startMode == UnifiedMode::Slam ? "slam" : startMode == UnifiedMode::Calib ? "calib" : "idle")
                          << "\n";
                m_session = std::thread([this, cfg, startMode]() mutable {
                    bool ok = false;
                    if (startMode == UnifiedMode::Slam) ok = RunSlamSession(cfg, m_mav, m_sessionStop, m_livePose);
                    else if (startMode == UnifiedMode::Calib) ok = RunCalibSession(cfg, m_sessionStop, m_livePose);
                    std::cerr << "[runtime] session function returned mode="
                              << (startMode == UnifiedMode::Slam ? "slam" : startMode == UnifiedMode::Calib ? "calib" : "idle")
                              << " ok=" << (ok ? "true" : "false") << "\n";
                    std::lock_guard<std::mutex> lock(m_mu);
                    m_sessionDone = true;
                    std::cerr << "[runtime] session done mode="
                              << (startMode == UnifiedMode::Slam ? "slam" : startMode == UnifiedMode::Calib ? "calib" : "idle")
                              << "\n";
                    m_cv.notify_all();
                });
            }
        }
    }

    std::mutex m_mu;
    std::condition_variable m_cv;
    UnifiedConfig m_config;
    MavlinkSerial& m_mav;
    LivePoseState& m_livePose;
    UnifiedMode m_desiredMode{UnifiedMode::Idle}, m_activeMode{UnifiedMode::Idle};
    bool m_restartRequested{false}, m_sessionDone{false}, m_stopping{false};
    std::atomic<bool> m_sessionStop{false};
    std::thread m_worker, m_session;
};

RouteResult HandleRuntimeModeFrame(const TlvFrame& frame, UnifiedRuntimeController& c)
{
    if (frame.len != RUNTIME_MODE_PAYLOAD_LEN) return {ACK_E_BAD_LEN, "bad runtime mode len"};
    UnifiedMode mode = UnifiedMode::Idle;
    if (frame.payload[0] == RUNTIME_MODE_SLAM) mode = UnifiedMode::Slam;
    else if (frame.payload[0] == RUNTIME_MODE_CALIB) mode = UnifiedMode::Calib;
    else if (frame.payload[0] != RUNTIME_MODE_IDLE) return {ACK_E_BAD_ARGS, "bad runtime mode"};
    std::string err;
    if (!c.SetMode(mode, &err)) return {ACK_E_BAD_STATE, err.empty() ? "set mode failed" : err};
    return {ACK_OK, mode == UnifiedMode::Idle ? "runtime -> idle" : mode == UnifiedMode::Slam ? "runtime -> slam" : "runtime -> calib"};
}

RouteResult HandleRuntimeConfigFrame(const TlvFrame& frame, const UdpPeer& peer, UnifiedRuntimeController& c)
{
    if (frame.len != RUNTIME_CONFIG_PAYLOAD_LEN && frame.len != RUNTIME_CONFIG_PAYLOAD_LEN_LEGACY) {
        return {ACK_E_BAD_LEN, "bad runtime cfg len"};
    }
    const uint8_t* p = frame.payload.data();
    const UnifiedConfig currentCfg = c.CurrentConfig();
    RemoteRuntimeConfig r{};
    r.exposureUs = (int)ReadU32Le(&p[0]);
    r.gain = ReadF32Le(&p[4]);
    r.pairMs = std::max(currentCfg.app.camera.pairMs, 1);
    if (r.exposureUs <= 0 || !std::isfinite(r.gain)) {
        return {ACK_E_BAD_ARGS, "bad runtime cfg args"};
    }
    r.sensorMode = (p[8] == RUNTIME_SENSOR_STEREO_IMU) ? SensorMode::StereoImu : SensorMode::Stereo;
    const uint8_t streamFlags = p[9];
    if (streamFlags == 0) {
        r.sendImage = true;
        r.sendFeature = true;
        r.sendMap = true;
    } else {
        r.sendImage = (streamFlags & RUNTIME_CFG_FLAG_SEND_IMAGE) != 0;
        r.sendFeature = (streamFlags & RUNTIME_CFG_FLAG_SEND_FEATURE) != 0;
        r.sendMap = (streamFlags & RUNTIME_CFG_FLAG_SEND_MAP) != 0;
    }
    size_t ipOffset = 10;
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN) {
        const int pairMs = (int)ReadU16Le(&p[RUNTIME_CONFIG_PAIR_MS_OFFSET]);
        if (pairMs > 0) r.pairMs = pairMs;
        ipOffset = RUNTIME_CONFIG_IP_OFFSET;
    }
    const char* ipChars = reinterpret_cast<const char*>(&p[ipOffset]);
    size_t ipLen = 0;
    while (ipLen < RUNTIME_CONFIG_IP_LEN && ipChars[ipLen] != '\0') {
        ++ipLen;
    }
    r.udpIp.assign(ipChars, ipLen);
    const std::string peerIp = PeerToIpString(peer);
    if (!peerIp.empty()) {
        r.udpIp = peerIp;
    }
    std::string err;
    if (!c.UpdateRemoteConfig(r, &err)) return {ACK_E_BAD_ARGS, err.empty() ? "runtime cfg failed" : err};
    return {ACK_OK, "runtime cfg updated udp=" + r.udpIp +
                    " sensor=" + std::string(r.sensorMode == SensorMode::StereoImu ? "stereo-imu" : "stereo") +
                    " settings=" + std::string(DefaultSettingsForSensorMode(r.sensorMode)) +
                    " pair_ms=" + std::to_string(r.pairMs) +
                    " img=" + (r.sendImage ? "on" : "off") +
                    " feat=" + (r.sendFeature ? "on" : "off") +
                    " map=" + (r.sendMap ? "on" : "off")};
}

RouteResult HandleCalibCleanFrame(const TlvFrame& frame, UnifiedRuntimeController& c)
{
    if (frame.len != 0) {
        return {ACK_E_BAD_LEN, "bad calib clean len"};
    }
    std::string msg;
    if (!c.CleanupCalibData(&msg)) {
        return {ACK_E_BAD_STATE, msg.empty() ? "calib clean failed" : msg};
    }
    return {ACK_OK, msg};
}

std::thread StartUdpCommandThread(int port, Px4UdpHooks& hooks, UnifiedRuntimeController& controller, LivePoseState& livePose)
{
    return std::thread([port, &hooks, &controller, &livePose]() {
        UdpServer server;
        if (!server.Open((uint16_t)port)) {
            std::cerr << "[udp_cmd] open failed on 0.0.0.0:" << port << "\n";
            return;
        }
        TlvCmdRouter router(hooks);
        router.RegisterDefaults();
        TlvParser parser;
        CommandPeerGate peerGate;
        uint8_t rx[2048]{};
        auto lastStateTx = std::chrono::steady_clock::now();
        auto lastRejectedPeerLog = std::chrono::steady_clock::time_point{};
        while (g_runningFlag.load()) {
            UdpPeer peer{};
            const int n = server.Recv(rx, sizeof(rx), &peer);
            if (n <= 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            } else {
                const auto now = std::chrono::steady_clock::now();
                if (!peerGate.Accept(peer, now)) {
                    if (now - lastRejectedPeerLog >= std::chrono::seconds(1)) {
                        lastRejectedPeerLog = now;
                        std::cerr << "[udp_cmd] rejected packet from non-active peer ip="
                                  << PeerToIpString(peer) << " port=" << ntohs(peer.addr.sin_port) << "\n";
                    }
                    continue;
                }
                livePose.UpdatePeer(peer);
                parser.Push(rx, (size_t)n);
                while (auto frame = parser.TryPop()) {
                    RouteResult rr{};
                    if (frame->cmd == CMD_RUNTIME_MODE) rr = HandleRuntimeModeFrame(*frame, controller);
                    else if (frame->cmd == CMD_RUNTIME_CONFIG) rr = HandleRuntimeConfigFrame(*frame, peer, controller);
                    else if (frame->cmd == CMD_CALIB_CLEAN) rr = HandleCalibCleanFrame(*frame, controller);
                    else rr = router.Handle(*frame);
                    std::vector<uint8_t> ack = MakeAckFrame(frame->seq, frame->tMs, frame->cmd, frame->seq, rr.status);
                    server.SendTo(peer, ack.data(), ack.size());
                    if (!rr.msg.empty()) std::cerr << "[udp_cmd] " << rr.msg << "\n";
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - lastStateTx >= std::chrono::milliseconds(100)) {
                lastStateTx = now;
                LivePoseState::Snapshot snap{};
                if (livePose.ConsumeSnapshot(snap) && snap.hasPeer) {
                    const UnifiedConfig currentCfg = controller.CurrentConfig();
                    if (currentCfg.app.udp.sendMap) {
                        std::vector<uint8_t> payload;
                        payload.reserve(STATE_POSE_PAYLOAD_LEN);
                        payload.push_back(snap.runtimeMode);
                        payload.push_back(snap.trackingState);
                        WriteU16Le(payload, snap.resetCounter);
                        WriteU16Le(payload, snap.resetMapCount);
                        WriteF32Le(payload, snap.x);
                        WriteF32Le(payload, snap.y);
                        WriteF32Le(payload, snap.z);
                        WriteF32Le(payload, snap.qw);
                        WriteF32Le(payload, snap.qx);
                        WriteF32Le(payload, snap.qy);
                        WriteF32Le(payload, snap.qz);
                        std::vector<uint8_t> stateFrame =
                            MakeFrame(TLV_VER, CMD_STATE, 0, snap.seq, MonoTimeMs32(),
                                      payload.data(), static_cast<uint16_t>(payload.size()));
                        server.SendTo(snap.peer, stateFrame.data(), stateFrame.size());

                        const size_t pointCount = snap.pointCloudXyz.size() / 3;
                        if (pointCount > 0) {
                            std::vector<uint8_t> cloudPayload;
                            cloudPayload.reserve(POINT_CLOUD_HEADER_LEN + pointCount * 12);
                            WriteU16Le(cloudPayload, static_cast<uint16_t>(std::min<size_t>(pointCount, 0xFFFF)));
                            WriteU16Le(cloudPayload, static_cast<uint16_t>(snap.pointCloudSeq & 0xFFFFu));
                            for (size_t i = 0; i < pointCount * 3; ++i) {
                                WriteF32Le(cloudPayload, snap.pointCloudXyz[i]);
                            }
                            std::vector<uint8_t> cloudFrame =
                                MakeFrame(TLV_VER, CMD_POINT_CLOUD, 0, snap.seq, MonoTimeMs32(),
                                          cloudPayload.data(), static_cast<uint16_t>(cloudPayload.size()));
                            server.SendTo(snap.peer, cloudFrame.data(), cloudFrame.size());
                        }
                    }
                }
            }
        }
    });
}

}  // namespace

int main(int argc, char** argv)
{
    InstallSignalHandlers();
    UnifiedConfig cfg{};
    cfg.app = ParseAppConfig(argc, argv);
    ArgReader args(argc, argv);
    cfg.calib.root = args.GetString("--calib-root", "./calib_runs");
    cfg.calib.maxFrames = args.GetInt("--calib-max-frames", 0);
    std::string autoModeText = args.GetString("--auto-mode", "idle");
    std::transform(autoModeText.begin(), autoModeText.end(), autoModeText.begin(), [](unsigned char c) {
        return (char)std::tolower(c);
    });
    UnifiedMode autoMode = UnifiedMode::Idle;
    if (autoModeText == "slam") autoMode = UnifiedMode::Slam;
    else if (autoModeText == "calib") autoMode = UnifiedMode::Calib;

    const MainRuntimeAliases aliases = BuildRuntimeAliases(cfg.app);
    PrintStartupConfig(cfg.app, aliases, UnifiedMode::Idle);

    MavlinkSerial mav("/dev/ttyAMA0", 921600);
    mav.StartRx();
    LivePoseState livePose;
    Px4UdpHooks hooks(mav, livePose);
    UnifiedRuntimeController controller(cfg, mav, livePose);
    controller.Start();
    if (autoMode != UnifiedMode::Idle) controller.SetMode(autoMode, nullptr);
    std::thread udpCmdThread = StartUdpCommandThread(aliases.cmdPort, hooks, controller, livePose);
    while (g_runningFlag.load()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    controller.Stop();
    mav.StopSetpointStream();
    mav.StopRx();
    if (udpCmdThread.joinable()) udpCmdThread.join();
    return 0;
}
