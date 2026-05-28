#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <unistd.h>

#include "common/environment.h"
#include "common/numeric_parse.h"
#include "adapters/imu/icm42688_imu_provider.h"
#include "adapters/slam/engine/slam_engine_factory.h"
#include "adapters/slam/engine/slam_mode_state.h"
#include "adapters/slam/superpoint/visual_feature_frontend_client.h"
#include "core/application/config/app_args.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/config/slam_backend_availability.h"
#include "core/application/epg/epg_runtime_optimizer.h"
#include "core/application/epg/epg_task_manifest.h"
#include "core/application/runtime/application_runtime_factories.h"
#include "core/application/runtime/epg_dfx_snapshot.h"
#include "core/application/runtime/runtime_aliases.h"
#include "core/application/sensors/imu_runtime_state.h"
#include "core/application/session/epg/slam_session_graph_service.h"
#include "core/application/session/slam/slam_runtime_control_port.h"
#include "core/application/session/stream/preview_output_port.h"
#include "core/application/state/live_pose_state.h"
#include "core/domain/runtime_mode.h"
#include "core/ports/pose_publisher.h"
#include "core/ports/slam_session_telemetry.h"
#include "support/replay_dataset.h"
#include "support/replay_slam_runner.h"
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace fs = std::filesystem;

namespace {

struct OfflineReplayOptions {
    fs::path datasetRoot{"tests/euroc/data"};
    fs::path outputCsv{"build/offline_replay_pose.csv"};
    fs::path summaryJson{};
    fs::path finalEurocTrajectory{};
    fs::path epgProfileOut{};
    fs::path epgOptimizedOut{};
    fs::path epgSolverReportOut{};
    int epgDrainMs{1500};
    std::string vocab{};
    std::string settings{"config/stereo.yaml"};
    SensorMode sensorMode{SensorMode::StereoImu};
    SlamBackend slamBackend{SlamBackend::Klt};
    FeatureFrontend featureFrontend{FeatureFrontend::LkGfttPerFrame};
    SmartDrone::Core::Domain::SlamOperationMode slamMode{
        SmartDrone::Core::Domain::SlamOperationMode::Mapping};
    std::string visualFeatureRepo{"LightGlue"};
    std::string visualFeatureDevice{"auto"};
    int visualFeatureTopK{1024};
    int visualFeatureMaxPoints{512};
    int visualFeatureInputMaxWidth{640};
    int visualFeatureInputMaxHeight{409};
    int cameraFps{60};
    int slamInputFps{20};
    int staleFrameThresholdMs{1000};
    size_t maxFrames{0};
    bool lkLoopClosure{false};
    bool lkRuntimeLoopClosure{false};
    float lkLoopScale{1.20f};
    float lkLoopRelaxation{1.40f};
    std::string lkPerFrameAcceleration{"cpu"};
    std::string orbAcceleration{"cpu"};
    std::string dpvoRepo{};
    std::string dpvoPatchEngine{};
    std::string dpvoUpdateEngine{};
    int dpvoInputWidth{640};
    int dpvoInputHeight{400};
    int dpvoPatchesPerFrame{48};
    int dpvoOptimizationWindow{7};
};

int64_t EnvTimestampOffsetNs()
{
    const double offsetMs =
        SmartDrone::Common::EnvDoubleValue(
            "SMART_DRONE_EUROC_OUTPUT_TIMESTAMP_OFFSET_MS", 0.0);
    return static_cast<int64_t>(std::llround(offsetMs * 1000000.0));
}

double EnvOutputPositionScale()
{
    return SmartDrone::Common::EnvDoubleValue(
        "SMART_DRONE_EUROC_OUTPUT_POSITION_SCALE", 1.0);
}

std::optional<Sophus::SE3f> ReadSe3Node(const cv::FileNode &node)
{
    if (node.empty()) {
        return std::nullopt;
    }

    cv::Mat mat;
    node >> mat;
    if (mat.empty() || mat.rows != 4 || mat.cols != 4) {
        return std::nullopt;
    }

    cv::Mat mat32f;
    mat.convertTo(mat32f, CV_32F);
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            transform(row, col) = mat32f.at<float>(row, col);
        }
    }
    return Sophus::SE3f(transform);
}

std::optional<Sophus::SE3f>
LoadBodyToCameraExtrinsics(const std::string &settingsPath)
{
    cv::FileStorage fs(settingsPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return std::nullopt;
    }

    if (auto tbc = ReadSe3Node(fs["T_b_c1"])) {
        return tbc;
    }
    return ReadSe3Node(fs["IMU.T_b_c1"]);
}

void ApplyBodyOutputExtrinsics(SmartDrone::Tests::ReplayPoseSample &sample,
                               const Sophus::SE3f &tbc)
{
    if (!sample.poseValid || !sample.pose.valid) {
        return;
    }

    const Eigen::Quaternionf qwc(sample.pose.qw, sample.pose.qx, sample.pose.qy,
                                 sample.pose.qz);
    const Sophus::SE3f twc(
        Sophus::SO3f(qwc.normalized()),
        Eigen::Vector3f(sample.pose.x, sample.pose.y, sample.pose.z));
    const Sophus::SE3f twb = twc * tbc.inverse();
    const Eigen::Vector3f t = twb.translation();
    const Eigen::Quaternionf q(twb.so3().unit_quaternion());
    sample.pose.x = t.x();
    sample.pose.y = t.y();
    sample.pose.z = t.z();
    sample.pose.qw = q.w();
    sample.pose.qx = q.x();
    sample.pose.qy = q.y();
    sample.pose.qz = q.z();
}

int64_t SaturatingTimestampAdd(int64_t timestampNs, int64_t offsetNs)
{
    if (offsetNs > 0 &&
        timestampNs > std::numeric_limits<int64_t>::max() - offsetNs) {
        return std::numeric_limits<int64_t>::max();
    }
    if (offsetNs < 0 &&
        timestampNs < std::numeric_limits<int64_t>::min() - offsetNs) {
        return std::numeric_limits<int64_t>::min();
    }
    return timestampNs + offsetNs;
}

SmartDrone::Tests::ReplayPoseSample AdjustReplayOutputSample(
    SmartDrone::Tests::ReplayPoseSample sample, int64_t timestampOffsetNs,
    double positionScale,
    const std::optional<Sophus::SE3f> &bodyFromCameraExtrinsics)
{
    if (bodyFromCameraExtrinsics.has_value()) {
        ApplyBodyOutputExtrinsics(sample, *bodyFromCameraExtrinsics);
    }
    if (timestampOffsetNs != 0) {
        sample.captureTimestampNs =
            SaturatingTimestampAdd(sample.captureTimestampNs, timestampOffsetNs);
    }
    if (positionScale != 1.0) {
        sample.pose.x *= positionScale;
        sample.pose.y *= positionScale;
        sample.pose.z *= positionScale;
    }
    return sample;
}

struct MetricAccumulator {
    double sum{0.0};
    double max{0.0};

    void Add(double value)
    {
        sum += value;
        max = std::max(max, value);
    }

    double Mean(size_t count) const
    {
        return sum / static_cast<double>(std::max<size_t>(1, count));
    }
};

void WriteReplayCsvHeader(std::ostream &csv)
{
    csv << "frame_id,capture_timestamp_ns,tracking_state,map_id,pose_valid,x,y,z,"
           "qw,qx,qy,qz,imu_samples,"
           "superpoint_used,superpoint_raw_left,superpoint_raw_right,superpoint_"
           "stereo,superpoint_injected_left,superpoint_injected_right,superpoint_"
           "external_hash,"
           "superpoint_lg_every_n,superpoint_frontend_ms,superpoint_match_ms,"
           "superpoint_total_ms,replay_acquire_ms,replay_imu_ms,slam_total_ms,"
           "slam_backend_step_ms,"
           "input_prepare_ms,frontend_ms,stereo_pair_ms,external_pack_ms,mono_"
           "augment_ms,"
           "lk_rectify_ms,lk_disparity_ms,lk_gftt_ms,lk_flow_ms,lk_candidate_ms,"
           "lk_pnp_ms,lk_update_ms,"
           "orb_track_ms,orb_extract_ms,orb_stereo_ms,local_mapping_wait_ms,"
           "local_mapping_wait_timeout_ms,"
           "local_mapping_queue_before,local_mapping_queue_after,local_mapping_"
           "accept_before,"
           "local_mapping_accept_after,local_mapping_wait_requested,local_"
           "mapping_wait_timeout,"
           "inliers,tracked_map,local_map,local_map_hash,matched_map_hash_before_"
           "po,tracked_map_hash,close_map,orb_frame_id,ref_kf,last_kf,last_kf_"
           "frame,keyframes_in_map,"
           "external_init_frame,external_injected,external_bootstrap,external_"
           "stabilizing,"
           "realtime_pose_gate,raw_pose_step_m,gated_pose_step_m,"
           "visual_feature_used,visual_feature_raw_left,visual_feature_raw_right,"
           "visual_feature_stereo,visual_feature_injected_left,visual_feature_"
           "injected_right,visual_feature_hash,"
           "visual_feature_every_n,visual_feature_frontend_ms,visual_feature_"
           "match_ms,visual_feature_total_ms,feature_pack_ms,"
           "stereo_feature_init_frame,stereo_feature_injected,stereo_feature_"
           "bootstrap,stereo_feature_stabilizing\n";
}

void WriteReplayCsvPoseFields(
    std::ostream &csv, const SmartDrone::Tests::ReplayPoseSample &sample)
{
    csv << sample.frameId << ',' << sample.captureTimestampNs << ','
        << sample.trackingState << ',' << sample.mapId << ','
        << (sample.poseValid ? 1 : 0) << ',' << sample.pose.x << ','
        << sample.pose.y << ',' << sample.pose.z << ',' << sample.pose.qw << ','
        << sample.pose.qx << ',' << sample.pose.qy << ',' << sample.pose.qz << ','
        << sample.imuSampleCount << ',';
}

void WriteReplayCsvVisualFeatureFields(
    std::ostream &csv, const SmartDrone::Tests::ReplayPoseSample &sample)
{
    csv << (sample.usedVisualFeatureFrontend ? 1 : 0) << ','
        << sample.visualFeatureRawLeftCount << ','
        << sample.visualFeatureRawRightCount << ','
        << sample.visualFeatureMatchedStereoCount << ','
        << sample.visualFeatureInjectedLeftCount << ','
        << sample.visualFeatureInjectedRightCount << ','
        << sample.visualFeatureObservationHash << ','
        << sample.visualFeatureMatchEveryN << ','
        << sample.visualFeatureFrontendMs << ','
        << sample.visualFeatureStereoMatchMs << ',' << sample.visualFeatureTotalMs
        << ',';
}

void WriteReplayCsvTimingFields(
    std::ostream &csv, const SmartDrone::Tests::ReplayPoseSample &sample)
{
    csv << sample.replayAcquireMs << ',' << sample.replayImuMs << ','
        << sample.slamTotalMs << ',' << sample.slamBackendStepMs << ','
        << sample.inputPrepareMs << ',' << sample.frontendMs << ','
        << sample.stereoPairMs << ',' << sample.featurePackMs << ','
        << sample.monoAugmentMs << ',' << sample.lkRectifyMs << ','
        << sample.lkDisparityMs << ',' << sample.lkGfttMs << ','
        << sample.lkFlowMs << ',' << sample.lkCandidateMs << ','
        << sample.lkPnpMs << ',' << sample.lkUpdateMs << ','
        << sample.orbTrackMs << ',' << sample.orbExtractMs << ','
        << sample.orbStereoMatchMs << ',';
}

void WriteReplayCsvMappingFields(
    std::ostream &csv, const SmartDrone::Tests::ReplayPoseSample &sample)
{
    csv << sample.localMappingWaitMs << ',' << sample.localMappingWaitTimeoutMs
        << ',' << sample.localMappingWaitQueueBefore << ','
        << sample.localMappingWaitQueueAfter << ','
        << (sample.localMappingAcceptingBefore ? 1 : 0) << ','
        << (sample.localMappingAcceptingAfter ? 1 : 0) << ','
        << (sample.localMappingWaitRequested ? 1 : 0) << ','
        << (sample.localMappingWaitTimedOut ? 1 : 0) << ','
        << sample.matchesInliers << ',' << sample.trackedMapPointCount << ','
        << sample.localMapPointCount << ',' << sample.localMapPointHash << ','
        << sample.matchedMapPointHashBeforePoseOptimization << ','
        << sample.trackedMapPointHash << ',' << sample.closeMapPointCount << ','
        << sample.orbFrameId << ',' << sample.referenceKeyFrameId << ','
        << sample.lastKeyFrameId << ',' << sample.lastKeyFrameFrameId << ','
        << sample.keyFramesInMap << ',' << sample.stereoFeatureInitFrameId << ','
        << (sample.stereoFeatureInjected ? 1 : 0) << ','
        << (sample.stereoFeatureBootstrap ? 1 : 0) << ','
        << (sample.stereoFeatureStabilizing ? 1 : 0) << ','
        << (sample.realtimePoseQualityGate ? 1 : 0) << ','
        << sample.rawPoseStepMeters << ',' << sample.gatedPoseStepMeters << ',';
}

void WriteReplayCsvSample(std::ostream &csv,
                          const SmartDrone::Tests::ReplayPoseSample &sample)
{
    WriteReplayCsvPoseFields(csv, sample);
    WriteReplayCsvVisualFeatureFields(csv, sample);
    WriteReplayCsvTimingFields(csv, sample);
    WriteReplayCsvMappingFields(csv, sample);
    WriteReplayCsvVisualFeatureFields(csv, sample);
    csv << sample.featurePackMs << ',' << sample.stereoFeatureInitFrameId << ','
        << (sample.stereoFeatureInjected ? 1 : 0) << ','
        << (sample.stereoFeatureBootstrap ? 1 : 0) << ','
        << (sample.stereoFeatureStabilizing ? 1 : 0) << '\n';
}

constexpr const char *OFFLINE_REPLAY_USAGE_TEXT =
    "Usage: smart_drone_offline_replay [options]\n"
    "  --dataset <dir>       Replay dataset root, default tests/data; "
    "accepts tests/data or EuRoC mav0 layout\n"
    "  --out <file>          Output CSV path, default "
    "build/offline_replay_pose.csv\n"
    "  --summary-json <file> Optional summary JSON output path\n"
    "  --final-euroc-trajectory <file> Optional final ORB-SLAM3 EuRoC "
    "trajectory after shutdown\n"
    "  --epg-profile-out <file> Copy SLAM EPG profile JSON after EPG replay\n"
    "  --epg-optimized-out <file> Optional optimized SLAM EPG config output "
    "for --epg-profile-out\n"
    "  --epg-solver-report-out <file> Optional solver report output for "
    "--epg-profile-out\n"
    "  --epg-drain-ms <n> Extra profile drain time after dataset end, "
    "default 1500\n"
    "  --vocab <file>        ORB vocabulary path, used only with "
    "--slam-backend orbslam3\n"
    "  --settings <file>     Camera/SLAM settings YAML path\n"
    "  --sensor-mode <mode>  stereo|stereo-imu|mono|mono-imu\n"
    "  --stereo-only         Shortcut for --sensor-mode stereo\n"
    "  --slam-backend <mode> klt|dpvo|openvins|orbslam3, default klt\n"
    "  --feature-frontend <mode> "
    "orb|superpoint_lightglue|lk|lk_gftt_per_frame, default "
    "lk_gftt_per_frame\n"
    "  --visual-feature-repo <dir>    Visual feature frontend repo root "
    "containing TensorRT engines\n"
    "  --visual-feature-device <dev>  TensorRT device auto|cuda, default "
    "auto\n"
    "  --visual-feature-top-k <n>     Visual feature top-k candidate "
    "count, default 1024\n"
    "  --visual-feature-max-points <n> Visual feature injected point "
    "budget, default 512\n"
    "  --superpoint-repo <dir>    Deprecated alias for "
    "--visual-feature-repo\n"
    "  --superpoint-device <dev>  Deprecated alias for "
    "--visual-feature-device\n"
    "  --superpoint-top-k <n>     Deprecated alias for "
    "--visual-feature-top-k\n"
    "  --superpoint-max-points <n> Deprecated alias for "
    "--visual-feature-max-points\n"
    "  --visual-feature-input-max-width <n>  Visual feature input width "
    "limit, default 640\n"
    "  --visual-feature-input-max-height <n> Visual feature input height "
    "limit, default 409\n"
    "  --superpoint-input-max-width <n>  Deprecated alias for "
    "--visual-feature-input-max-width\n"
    "  --superpoint-input-max-height <n> Deprecated alias for "
    "--visual-feature-input-max-height\n"
    "  --slam-mode <mode>    "
    "mapping|localization|relocalization|tracking-only|auto\n"
    "  --fps <n>             Camera FPS for replay pacing, default 60\n"
    "  --slam-fps <n>        SLAM input FPS, default 20\n"
    "  --stale-frame-threshold-ms <n> Stereo stale diagnostic threshold, default 1000\n"
    "  --timeout-ms <n>      Deprecated alias for --stale-frame-threshold-ms\n"
    "  --max-frames <n>      Maximum output frames, default 0(all)\n"
    "  --lk-loop-closure     Compatibility flag; disabled for realtime "
    "CSV output\n"
    "  --lk-runtime-loop-closure Enable image-based LK keyframe loop "
    "closure during replay\n"
    "  --lk-loop-scale <f>   Sim3 scale used by LK loop closure, default "
    "1.20\n"
    "  --lk-loop-relax <f>   Loop residual relaxation factor, default "
    "1.40\n"
    "  --lk-per-frame-accel <auto|cpu|vpi-cuda>  Per-frame LK GFTT stereo "
    "backend, default cpu\n"
    "  --orb-accel <cpu|cuda|vpi-remap>  ORB acceleration/preprocess "
    "mode, default cpu\n"
    "  --dpvo-repo <dir>     DPVO repo root used to locate TensorRT "
    "engines under weights/\n"
    "  --dpvo-patch-engine <file>  Explicit DPVO patchifier TensorRT "
    "engine\n"
    "  --dpvo-update-engine <file> Explicit DPVO update TensorRT engine\n"
    "  --dpvo-input-width <n>      DPVO TensorRT input width, default "
    "640\n"
    "  --dpvo-input-height <n>     DPVO TensorRT input height, default "
    "400\n"
    "  --dpvo-patches-per-frame <n> DPVO patch budget, default 48\n"
    "  --dpvo-optimization-window <n> DPVO optimization window, default "
    "7\n";

const char *UsageText()
{
    return OFFLINE_REPLAY_USAGE_TEXT;
}

std::string NormalizeOrbAccelerationText(std::string value)
{
    std::transform(
        value.begin(), value.end(), value.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (value == "cuda" || value == "gpu" || value == "opencv_cuda" ||
        value == "opencv-cuda") {
        return "cuda";
    }
    if (value == "vpi" || value == "vpi_remap" || value == "vpi-remap" ||
        value == "vpi_cuda_remap" || value == "vpi-cuda-remap") {
        return "vpi-remap";
    }
    return "cpu";
}

void ApplyOrbAccelerationEnvironment(const std::string &acceleration)
{
    const std::string normalized = NormalizeOrbAccelerationText(acceleration);
    if (normalized == "cuda") {
        setenv("SMART_DRONE_ORB_ACCEL", "cuda", 1);
        unsetenv("SMART_DRONE_ORB_VPI_REMAP");
        unsetenv("SMART_DRONE_ORB_CUDA_PYRAMID");
    } else if (normalized == "vpi-remap") {
        unsetenv("SMART_DRONE_ORB_ACCEL");
        setenv("SMART_DRONE_ORB_VPI_REMAP", "1", 1);
        unsetenv("SMART_DRONE_ORB_CUDA_PYRAMID");
    } else {
        unsetenv("SMART_DRONE_ORB_ACCEL");
        unsetenv("SMART_DRONE_ORB_VPI_REMAP");
        unsetenv("SMART_DRONE_ORB_CUDA_PYRAMID");
    }
}

std::string GetOptionValue(int argc, char **argv, const char *name,
                           const std::string &defaultValue)
{
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == name) {
            return argv[i + 1];
        }
    }
    return defaultValue;
}

int GetOptionInt(int argc, char **argv, const char *name, int defaultValue)
{
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == name) {
            int parsed = defaultValue;
            if (!SmartDrone::Common::TryParseIntPrefix(argv[i + 1], 10,
                                                       parsed)) {
                throw std::invalid_argument(argv[i + 1]);
            }
            return parsed;
        }
    }
    return defaultValue;
}

int GetOptionIntWithLegacyFallback(int argc, char **argv, const char *name,
                                   const char *legacyName, int defaultValue)
{
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == name) {
            int parsed = defaultValue;
            if (!SmartDrone::Common::TryParseIntPrefix(argv[i + 1], 10,
                                                       parsed)) {
                throw std::invalid_argument(argv[i + 1]);
            }
            return parsed;
        }
    }
    return GetOptionInt(argc, argv, legacyName, defaultValue);
}

std::string GetOptionValueWithLegacyFallback(int argc, char **argv,
                                             const char *name,
                                             const char *legacyName,
                                             const std::string &defaultValue)
{
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == name) {
            return argv[i + 1];
        }
    }
    return GetOptionValue(argc, argv, legacyName, defaultValue);
}

float GetOptionFloat(int argc, char **argv, const char *name,
                     float defaultValue)
{
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == name) {
            float parsed = defaultValue;
            if (!SmartDrone::Common::TryParseFloatPrefix(argv[i + 1],
                                                         parsed)) {
                throw std::invalid_argument(argv[i + 1]);
            }
            return parsed;
        }
    }
    return defaultValue;
}

size_t GetOptionSize(int argc, char **argv, const char *name,
                     size_t defaultValue)
{
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == name) {
            size_t parsed = defaultValue;
            if (!SmartDrone::Common::TryParseSizePrefix(argv[i + 1],
                                                        parsed)) {
                throw std::invalid_argument(argv[i + 1]);
            }
            return parsed;
        }
    }
    return defaultValue;
}

bool HasFlag(int argc, char **argv, const char *name)
{
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == name) {
            return true;
        }
    }
    return false;
}

const char *ProgramPath(int argc, char **argv)
{
    return argc > 0 ? argv[0] : nullptr;
}

void ParseReplayIoOptions(int argc, char **argv, OfflineReplayOptions &opts)
{
    opts.datasetRoot = fs::path(ResolveRuntimePath(
        GetOptionValue(argc, argv, "--dataset", opts.datasetRoot.string()),
        ProgramPath(argc, argv)));
    opts.outputCsv =
        fs::path(GetOptionValue(argc, argv, "--out", opts.outputCsv.string()));
    opts.summaryJson = fs::path(GetOptionValue(argc, argv, "--summary-json", ""));
    opts.finalEurocTrajectory =
        fs::path(GetOptionValue(argc, argv, "--final-euroc-trajectory", ""));
    opts.epgProfileOut =
        fs::path(GetOptionValue(argc, argv, "--epg-profile-out", ""));
    opts.epgOptimizedOut =
        fs::path(GetOptionValue(argc, argv, "--epg-optimized-out", ""));
    opts.epgSolverReportOut =
        fs::path(GetOptionValue(argc, argv, "--epg-solver-report-out", ""));
    opts.epgDrainMs =
        GetOptionInt(argc, argv, "--epg-drain-ms", opts.epgDrainMs);
}

void ParseReplayModeOptions(int argc, char **argv, OfflineReplayOptions &opts)
{
    opts.sensorMode = ParseSensorModeText(
        GetOptionValue(argc, argv, "--sensor-mode", "stereo-imu"));
    if (HasFlag(argc, argv, "--stereo-only")) {
        opts.sensorMode = SensorMode::Stereo;
    }
    opts.slamBackend = NormalizeSlamBackendForBuild(ParseSlamBackendText(
        GetOptionValue(argc, argv, "--slam-backend", "klt")));
    opts.sensorMode =
        NormalizeSensorModeForSlamBackend(opts.sensorMode, opts.slamBackend);
    opts.featureFrontend = ParseFeatureFrontendText(
        GetOptionValue(argc, argv, "--feature-frontend", "lk_gftt_per_frame"));
    opts.slamMode = ParseSlamOperationModeText(
        GetOptionValue(argc, argv, "--slam-mode", "mapping"));
    opts.cameraFps = GetOptionInt(argc, argv, "--fps", opts.cameraFps);
    opts.slamInputFps = GetOptionInt(argc, argv, "--slam-fps", opts.slamInputFps);
    opts.staleFrameThresholdMs = GetOptionIntWithLegacyFallback(
        argc, argv, "--stale-frame-threshold-ms", "--timeout-ms",
        opts.staleFrameThresholdMs);
    opts.maxFrames = GetOptionSize(argc, argv, "--max-frames", opts.maxFrames);
}

void ParseVisualFeatureOptions(int argc, char **argv, OfflineReplayOptions &opts)
{
    opts.visualFeatureRepo =
        ResolveRuntimePath(GetOptionValueWithLegacyFallback(
                               argc, argv, "--visual-feature-repo",
                               "--superpoint-repo", opts.visualFeatureRepo),
                           ProgramPath(argc, argv));
    opts.visualFeatureDevice = GetOptionValueWithLegacyFallback(
        argc, argv, "--visual-feature-device", "--superpoint-device",
        opts.visualFeatureDevice);
    opts.visualFeatureTopK = GetOptionIntWithLegacyFallback(
        argc, argv, "--visual-feature-top-k", "--superpoint-top-k",
        opts.visualFeatureTopK);
    opts.visualFeatureMaxPoints = GetOptionIntWithLegacyFallback(
        argc, argv, "--visual-feature-max-points", "--superpoint-max-points",
        opts.visualFeatureMaxPoints);
    opts.visualFeatureInputMaxWidth = GetOptionIntWithLegacyFallback(
        argc, argv, "--visual-feature-input-max-width",
        "--superpoint-input-max-width", opts.visualFeatureInputMaxWidth);
    opts.visualFeatureInputMaxHeight = GetOptionIntWithLegacyFallback(
        argc, argv, "--visual-feature-input-max-height",
        "--superpoint-input-max-height", opts.visualFeatureInputMaxHeight);
}

void ParseLkOptions(int argc, char **argv, OfflineReplayOptions &opts)
{
    opts.lkLoopClosure = HasFlag(argc, argv, "--lk-loop-closure");
    opts.lkRuntimeLoopClosure = HasFlag(argc, argv, "--lk-runtime-loop-closure");
    opts.lkLoopScale =
        GetOptionFloat(argc, argv, "--lk-loop-scale", opts.lkLoopScale);
    opts.lkLoopRelaxation =
        GetOptionFloat(argc, argv, "--lk-loop-relax", opts.lkLoopRelaxation);
    opts.lkPerFrameAcceleration =
        GetOptionValue(argc, argv, "--lk-per-frame-accel", "cpu");
    opts.orbAcceleration = NormalizeOrbAccelerationText(
        GetOptionValue(argc, argv, "--orb-accel", "cpu"));
}

void ParseDpvoOptions(int argc, char **argv, OfflineReplayOptions &opts)
{
    opts.dpvoRepo =
        ResolveRuntimePath(GetOptionValue(argc, argv, "--dpvo-repo", ""),
                           ProgramPath(argc, argv));
    opts.dpvoPatchEngine =
        ResolveRuntimePath(GetOptionValue(argc, argv, "--dpvo-patch-engine", ""),
                           ProgramPath(argc, argv));
    opts.dpvoUpdateEngine =
        ResolveRuntimePath(GetOptionValue(argc, argv, "--dpvo-update-engine", ""),
                           ProgramPath(argc, argv));
    opts.dpvoInputWidth =
        GetOptionInt(argc, argv, "--dpvo-input-width", opts.dpvoInputWidth);
    opts.dpvoInputHeight =
        GetOptionInt(argc, argv, "--dpvo-input-height", opts.dpvoInputHeight);
    opts.dpvoPatchesPerFrame = GetOptionInt(
        argc, argv, "--dpvo-patches-per-frame", opts.dpvoPatchesPerFrame);
    opts.dpvoOptimizationWindow = GetOptionInt(
        argc, argv, "--dpvo-optimization-window", opts.dpvoOptimizationWindow);
}

void ParseSlamAssetOptions(int argc, char **argv, OfflineReplayOptions &opts)
{
    opts.vocab = ResolveRuntimePath(
        GetOptionValue(argc, argv, "--vocab",
                       opts.slamBackend == SlamBackend::OrbSlam3 ? "ORBvoc.txt"
                                                                 : ""),
        ProgramPath(argc, argv));
    opts.settings = ResolveRuntimePath(
        GetOptionValue(argc, argv, "--settings",
                       DefaultSettingsForSlamBackend(opts.sensorMode,
                                                     opts.slamBackend)),
        ProgramPath(argc, argv));
}

OfflineReplayOptions ParseOptions(int argc, char **argv)
{
    OfflineReplayOptions opts;
    if (HasFlag(argc, argv, "--help") || HasFlag(argc, argv, "-h")) {
        std::cout << UsageText();
        std::exit(0);
    }

    ParseReplayIoOptions(argc, argv, opts);
    ParseReplayModeOptions(argc, argv, opts);
    ParseVisualFeatureOptions(argc, argv, opts);
    ParseLkOptions(argc, argv, opts);
    ParseDpvoOptions(argc, argv, opts);
    ParseSlamAssetOptions(argc, argv, opts);
    return opts;
}

bool UseImu(SensorMode mode)
{
    return mode == SensorMode::StereoImu || mode == SensorMode::MonoImu;
}

SmartDrone::Adapters::Slam::SlamInputMode ResolveSlamInputMode(SensorMode mode)
{
    return (mode == SensorMode::Mono || mode == SensorMode::MonoImu)
               ? SmartDrone::Adapters::Slam::SlamInputMode::MonoRight
               : SmartDrone::Adapters::Slam::SlamInputMode::Stereo;
}

int RunEpgProfileReplay(const OfflineReplayOptions &opts);

#include "offline_replay_runtime.h"
#include "offline_replay_summary.h"
#include "offline_replay_execute.h"
#include "offline_replay_epg_profile.h"

} // namespace

int main(int argc, char **argv)
{
    try {
        return RunOfflineReplay(ParseOptions(argc, argv));
    } catch (const std::exception &ex) {
        std::cerr << "offline replay failed: " << ex.what() << "\n";
        return 1;
    }
}
