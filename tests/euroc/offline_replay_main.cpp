#include <algorithm>
#include <cctype>
#include <cerrno>
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
#include <vector>

#include "adapters/slam/slam_engine_factory.h"
#include "adapters/slam/visual_feature_frontend_client.h"
#include "core/application/config/app_args.h"
#include "core/domain/runtime_mode.h"
#include "support/replay_dataset.h"
#include "support/replay_slam_runner.h"
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace fs = std::filesystem;

namespace {

struct OfflineReplayOptions {
    fs::path datasetRoot{fs::path(TESTS_SOURCE_DIR) / "data"};
    fs::path outputCsv{"build/offline_replay_pose.csv"};
    fs::path summaryJson{};
    fs::path finalEurocTrajectory{};
    std::string vocab{};
    std::string settings{"config/stereo.yaml"};
    SensorMode sensorMode{SensorMode::StereoImu};
    SlamBackend slamBackend{SlamBackend::Klt};
    FeatureFrontend featureFrontend{FeatureFrontend::LkGfttPerFrame};
    SmartDrone::core::domain::SlamOperationMode slamMode{
        SmartDrone::core::domain::SlamOperationMode::Mapping};
    std::string visualFeatureRepo{"LightGlue"};
    std::string visualFeatureDevice{"auto"};
    int visualFeatureTopK{1024};
    int visualFeatureMaxPoints{512};
    int visualFeatureInputMaxWidth{640};
    int visualFeatureInputMaxHeight{409};
    int cameraFps{60};
    int slamInputFps{20};
    int timeoutMs{1000};
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

double EnvDoubleValue(const char *name, double fallback)
{
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    char *end = nullptr;
    errno = 0;
    const double parsed = std::strtod(value, &end);
    if (errno != 0 || end == value || !std::isfinite(parsed)) {
        return fallback;
    }
    return parsed;
}

int64_t EnvTimestampOffsetNs()
{
    const double offsetMs =
        EnvDoubleValue("SMART_DRONE_EUROC_OUTPUT_TIMESTAMP_OFFSET_MS", 0.0);
    return static_cast<int64_t>(std::llround(offsetMs * 1000000.0));
}

double EnvOutputPositionScale()
{
    return EnvDoubleValue("SMART_DRONE_EUROC_OUTPUT_POSITION_SCALE", 1.0);
}

bool EnvFlagEnabled(const char *name, bool fallback)
{
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    const std::string text(value);
    return text == "1" || text == "true" || text == "TRUE" || text == "on" ||
           text == "ON" || text == "yes" || text == "YES";
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

void ApplyBodyOutputExtrinsics(SmartDrone::tests::ReplayPoseSample &sample,
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

SmartDrone::tests::ReplayPoseSample AdjustReplayOutputSample(
    SmartDrone::tests::ReplayPoseSample sample, int64_t timestampOffsetNs,
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

void WriteReplayCsvSample(std::ostream &csv,
                          const SmartDrone::tests::ReplayPoseSample &sample)
{
    csv << sample.frameId << ',' << sample.captureTimestampNs << ','
        << sample.trackingState << ',' << sample.mapId << ','
        << (sample.poseValid ? 1 : 0) << ',' << sample.pose.x << ','
        << sample.pose.y << ',' << sample.pose.z << ',' << sample.pose.qw << ','
        << sample.pose.qx << ',' << sample.pose.qy << ',' << sample.pose.qz << ','
        << sample.imuSampleCount << ','
        << (sample.usedVisualFeatureFrontend ? 1 : 0) << ','
        << sample.visualFeatureRawLeftCount << ','
        << sample.visualFeatureRawRightCount << ','
        << sample.visualFeatureMatchedStereoCount << ','
        << sample.visualFeatureInjectedLeftCount << ','
        << sample.visualFeatureInjectedRightCount << ','
        << sample.visualFeatureObservationHash << ','
        << sample.visualFeatureMatchEveryN << ','
        << sample.visualFeatureFrontendMs << ','
        << sample.visualFeatureStereoMatchMs << ',' << sample.visualFeatureTotalMs
        << ',' << sample.replayAcquireMs << ',' << sample.replayImuMs << ','
        << sample.slamTotalMs << ',' << sample.inputPrepareMs << ','
        << sample.frontendMs << ',' << sample.stereoPairMs << ','
        << sample.featurePackMs << ',' << sample.monoAugmentMs << ','
        << sample.lkRectifyMs << ',' << sample.lkDisparityMs << ','
        << sample.lkGfttMs << ',' << sample.lkFlowMs << ','
        << sample.lkCandidateMs << ',' << sample.lkPnpMs << ','
        << sample.lkUpdateMs << ',' << sample.orbTrackMs << ','
        << sample.orbExtractMs << ',' << sample.orbStereoMatchMs << ','
        << sample.localMappingWaitMs << ',' << sample.localMappingWaitTimeoutMs
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
        << sample.rawPoseStepMeters << ',' << sample.gatedPoseStepMeters << ','
        << (sample.usedVisualFeatureFrontend ? 1 : 0) << ','
        << sample.visualFeatureRawLeftCount << ','
        << sample.visualFeatureRawRightCount << ','
        << sample.visualFeatureMatchedStereoCount << ','
        << sample.visualFeatureInjectedLeftCount << ','
        << sample.visualFeatureInjectedRightCount << ','
        << sample.visualFeatureObservationHash << ','
        << sample.visualFeatureMatchEveryN << ','
        << sample.visualFeatureFrontendMs << ','
        << sample.visualFeatureStereoMatchMs << ',' << sample.visualFeatureTotalMs
        << ',' << sample.featurePackMs << ',' << sample.stereoFeatureInitFrameId
        << ',' << (sample.stereoFeatureInjected ? 1 : 0) << ','
        << (sample.stereoFeatureBootstrap ? 1 : 0) << ','
        << (sample.stereoFeatureStabilizing ? 1 : 0) << '\n';
}

const char *UsageText()
{
    return "Usage: smart_drone_offline_replay [options]\n"
           "  --dataset <dir>       Replay dataset root, default tests/data; "
           "accepts tests/data or EuRoC mav0 layout\n"
           "  --out <file>          Output CSV path, default "
           "build/offline_replay_pose.csv\n"
           "  --summary-json <file> Optional summary JSON output path\n"
           "  --final-euroc-trajectory <file> Optional final ORB-SLAM3 EuRoC "
           "trajectory after shutdown\n"
           "  --vocab <file>        ORB vocabulary path, used only with "
           "--slam-backend orbslam3\n"
           "  --settings <file>     Camera/SLAM settings YAML path\n"
           "  --sensor-mode <mode>  stereo|stereo-imu|mono|mono-imu\n"
           "  --stereo-only         Shortcut for --sensor-mode stereo\n"
           "  --slam-backend <mode> klt|dpvo|orbslam3, default klt\n"
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
           "  --timeout-ms <n>      Batch acquire timeout, default 1000\n"
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
            return std::stoi(argv[i + 1]);
        }
    }
    return defaultValue;
}

int GetOptionIntWithLegacyFallback(int argc, char **argv, const char *name,
                                   const char *legacyName, int defaultValue)
{
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == name) {
            return std::stoi(argv[i + 1]);
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
            return std::stof(argv[i + 1]);
        }
    }
    return defaultValue;
}

size_t GetOptionSize(int argc, char **argv, const char *name,
                     size_t defaultValue)
{
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == name) {
            return static_cast<size_t>(std::stoull(argv[i + 1]));
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

OfflineReplayOptions ParseOptions(int argc, char **argv)
{
    OfflineReplayOptions opts;
    if (HasFlag(argc, argv, "--help") || HasFlag(argc, argv, "-h")) {
        std::cout << UsageText();
        std::exit(0);
    }

    opts.datasetRoot = fs::path(
        GetOptionValue(argc, argv, "--dataset", opts.datasetRoot.string()));
    opts.outputCsv =
        fs::path(GetOptionValue(argc, argv, "--out", opts.outputCsv.string()));
    opts.summaryJson = fs::path(GetOptionValue(argc, argv, "--summary-json", ""));
    opts.finalEurocTrajectory =
        fs::path(GetOptionValue(argc, argv, "--final-euroc-trajectory", ""));
    opts.sensorMode = ParseSensorModeText(
        GetOptionValue(argc, argv, "--sensor-mode", "stereo-imu"));
    if (HasFlag(argc, argv, "--stereo-only")) {
        opts.sensorMode = SensorMode::Stereo;
    }
    opts.slamBackend = NormalizeSlamBackendForBuild(ParseSlamBackendText(
        GetOptionValue(argc, argv, "--slam-backend", "klt")));
    opts.featureFrontend = ParseFeatureFrontendText(
        GetOptionValue(argc, argv, "--feature-frontend", "lk_gftt_per_frame"));
    opts.slamMode = ParseSlamOperationModeText(
        GetOptionValue(argc, argv, "--slam-mode", "mapping"));
    opts.cameraFps = GetOptionInt(argc, argv, "--fps", opts.cameraFps);
    opts.slamInputFps = GetOptionInt(argc, argv, "--slam-fps", opts.slamInputFps);
    opts.timeoutMs = GetOptionInt(argc, argv, "--timeout-ms", opts.timeoutMs);
    opts.maxFrames = GetOptionSize(argc, argv, "--max-frames", opts.maxFrames);
    opts.visualFeatureRepo =
        ResolveRuntimePath(GetOptionValueWithLegacyFallback(
                               argc, argv, "--visual-feature-repo",
                               "--superpoint-repo", opts.visualFeatureRepo),
                           argc > 0 ? argv[0] : nullptr);
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
    opts.dpvoRepo =
        ResolveRuntimePath(GetOptionValue(argc, argv, "--dpvo-repo", ""),
                           argc > 0 ? argv[0] : nullptr);
    opts.dpvoPatchEngine =
        ResolveRuntimePath(GetOptionValue(argc, argv, "--dpvo-patch-engine", ""),
                           argc > 0 ? argv[0] : nullptr);
    opts.dpvoUpdateEngine =
        ResolveRuntimePath(GetOptionValue(argc, argv, "--dpvo-update-engine", ""),
                           argc > 0 ? argv[0] : nullptr);
    opts.dpvoInputWidth =
        GetOptionInt(argc, argv, "--dpvo-input-width", opts.dpvoInputWidth);
    opts.dpvoInputHeight =
        GetOptionInt(argc, argv, "--dpvo-input-height", opts.dpvoInputHeight);
    opts.dpvoPatchesPerFrame = GetOptionInt(
        argc, argv, "--dpvo-patches-per-frame", opts.dpvoPatchesPerFrame);
    opts.dpvoOptimizationWindow = GetOptionInt(
        argc, argv, "--dpvo-optimization-window", opts.dpvoOptimizationWindow);
    opts.vocab = ResolveRuntimePath(
        GetOptionValue(argc, argv, "--vocab",
                       opts.slamBackend == SlamBackend::OrbSlam3 ? "ORBvoc.txt"
                                                                 : ""),
        argc > 0 ? argv[0] : nullptr);
    opts.settings = ResolveRuntimePath(
        GetOptionValue(argc, argv, "--settings",
                       DefaultSettingsForSensorMode(opts.sensorMode)),
        argc > 0 ? argv[0] : nullptr);
    return opts;
}

bool UseImu(SensorMode mode)
{
    return mode == SensorMode::StereoImu || mode == SensorMode::MonoImu;
}

SmartDrone::adapters::slam::SlamInputMode
ResolveSlamInputMode(SensorMode mode)
{
    return (mode == SensorMode::Mono || mode == SensorMode::MonoImu)
               ? SmartDrone::adapters::slam::SlamInputMode::MonoRight
               : SmartDrone::adapters::slam::SlamInputMode::Stereo;
}

int RunOfflineReplay(const OfflineReplayOptions &opts)
{
    const int64_t eurocOutputTimestampOffsetNs = EnvTimestampOffsetNs();
    const double eurocOutputPositionScale = EnvOutputPositionScale();
    const SmartDrone::tests::ReplayDataset dataset =
        SmartDrone::tests::ReplayDataset::Load(opts.datasetRoot);
    if (dataset.Empty()) {
        std::cerr << "dataset is empty: " << opts.datasetRoot << "\n";
        return 1;
    }
    std::cerr << "[offline_replay] slam_backend="
              << ToSlamBackendText(opts.slamBackend) << " feature_frontend="
              << ToFeatureFrontendText(opts.featureFrontend) << "\n";
    if (opts.slamBackend == SlamBackend::DpvoTensorRt) {
        std::cerr << "[offline_replay] dpvo_repo=" << opts.dpvoRepo
                  << " patch_engine=" << opts.dpvoPatchEngine
                  << " update_engine=" << opts.dpvoUpdateEngine << "\n";
    }

    SmartDrone::tests::ReplayCameraProvider camera(dataset);
    SmartDrone::tests::ReplayImuProvider imu(dataset);
    std::unique_ptr<SmartDrone::core::ports::ISlamEngine> slamEngine;
    SmartDrone::adapters::slam::ISlamRuntimeControl *slamControl = nullptr;

#if defined(SMART_DRONE_ENABLE_ORB_SLAM3)
    if (opts.slamBackend == SlamBackend::OrbSlam3) {
        ApplyOrbAccelerationEnvironment(opts.orbAcceleration);
    }
#else
    if (opts.slamBackend == SlamBackend::OrbSlam3) {
        std::cerr << "error: ORB-SLAM3 backend is not compiled into this offline "
                     "replay target\n";
        return 2;
    }
#endif

    RuntimeConfig replayRuntime{};
    replayRuntime.slamBackend = opts.slamBackend;
    replayRuntime.featureFrontend = opts.featureFrontend;
    replayRuntime.slamOperationMode = opts.slamMode;
    replayRuntime.dpvoRepo = opts.dpvoRepo;
    replayRuntime.dpvoPatchEngine = opts.dpvoPatchEngine;
    replayRuntime.dpvoUpdateEngine = opts.dpvoUpdateEngine;
    replayRuntime.dpvoInputWidth = opts.dpvoInputWidth;
    replayRuntime.dpvoInputHeight = opts.dpvoInputHeight;
    replayRuntime.dpvoPatchesPerFrame = opts.dpvoPatchesPerFrame;
    replayRuntime.dpvoOptimizationWindow = opts.dpvoOptimizationWindow;
    replayRuntime.visualFeatureRepo = opts.visualFeatureRepo;
    replayRuntime.visualFeatureDevice = opts.visualFeatureDevice;
    replayRuntime.visualFeatureTopK = opts.visualFeatureTopK;
    replayRuntime.visualFeatureMaxPoints = opts.visualFeatureMaxPoints;
    replayRuntime.visualFeatureInputMaxWidth = opts.visualFeatureInputMaxWidth;
    replayRuntime.visualFeatureInputMaxHeight = opts.visualFeatureInputMaxHeight;
    replayRuntime.lkLoopClosure = opts.lkRuntimeLoopClosure;
    replayRuntime.lkLoopScale = opts.lkLoopScale;
    replayRuntime.lkLoopRelaxation = opts.lkLoopRelaxation;
    replayRuntime.lkPerFrameAcceleration = opts.lkPerFrameAcceleration;
    replayRuntime.orbAcceleration = opts.orbAcceleration;

    SmartDrone::adapters::slam::SlamEngineFactoryConfig engineConfig{};
    engineConfig.backend = opts.slamBackend;
    engineConfig.vocabularyPath = opts.vocab;
    engineConfig.settingsPath = opts.settings;
    engineConfig.sensorMode = opts.sensorMode;
    engineConfig.useViewer = false;
    engineConfig.useImu = UseImu(opts.sensorMode);
    engineConfig.inputMode = ResolveSlamInputMode(opts.sensorMode);
    engineConfig.runtime = replayRuntime;
    SmartDrone::adapters::slam::ControlledSlamEngine controlled =
        SmartDrone::adapters::slam::CreateSlamEngine(engineConfig);
    if (controlled.engine == nullptr) {
        std::cerr << "error: SLAM backend failed to initialize\n";
        return 2;
    }
    slamControl = controlled.control;
    slamEngine = std::move(controlled.engine);

    if (slamControl != nullptr) {
        slamControl->SetOperationMode(opts.slamMode);
        slamControl->SetFeatureFrontend(opts.featureFrontend);
        slamControl->SetVisualFeatureInputSizeLimit(
            opts.visualFeatureInputMaxWidth, opts.visualFeatureInputMaxHeight);
        slamControl->SetStereoVoLoopClosure(
            opts.lkRuntimeLoopClosure, opts.lkLoopScale, opts.lkLoopRelaxation);
        slamControl->SetStereoVoPerFrameAcceleration(opts.lkPerFrameAcceleration);
    }

    std::unique_ptr<SmartDrone::adapters::slam::IManagedVisualFeatureFrontend>
        visualFeatureFrontendClient;
    if (slamControl != nullptr &&
        IsVisualFeatureLightGlueFrontend(opts.featureFrontend) &&
        SmartDrone::adapters::slam::VisualFeatureFrontendClientEnabled(
            opts.featureFrontend)) {
        SmartDrone::adapters::slam::VisualFeatureFrontendRuntimeConfig
            featureConfig{};
        featureConfig.repoPath =
            SmartDrone::adapters::slam::ResolveVisualFeatureFrontendRepo(
                opts.featureFrontend, opts.visualFeatureRepo);
        featureConfig.device = opts.visualFeatureDevice;
        featureConfig.topK = opts.visualFeatureTopK;
        featureConfig.maxPoints = opts.visualFeatureMaxPoints;
        featureConfig.inputMaxWidth = opts.visualFeatureInputMaxWidth;
        featureConfig.inputMaxHeight = opts.visualFeatureInputMaxHeight;
        SmartDrone::adapters::slam::ConfigureVisualFeatureFrontendDefaults(
            opts.featureFrontend, featureConfig);
        visualFeatureFrontendClient =
            SmartDrone::adapters::slam::CreateVisualFeatureFrontendClient(
                opts.featureFrontend);
        if (visualFeatureFrontendClient == nullptr) {
            std::cerr << "error: no visual feature frontend client registered for "
                      << ToFeatureFrontendText(opts.featureFrontend) << "\n";
            return 2;
        }
        std::string featureErr;
        if (!visualFeatureFrontendClient->Start(featureConfig, &featureErr)) {
            std::cerr << "error: " << ToFeatureFrontendText(opts.featureFrontend)
                      << " frontend start failed: " << featureErr << "\n";
            return 2;
        }
        slamControl->SetVisualFeatureFrontend(visualFeatureFrontendClient.get());
    }

    if (!opts.outputCsv.parent_path().empty()) {
        fs::create_directories(opts.outputCsv.parent_path());
    }
    if (!opts.summaryJson.empty() && !opts.summaryJson.parent_path().empty()) {
        fs::create_directories(opts.summaryJson.parent_path());
    }
    if (!opts.finalEurocTrajectory.empty() &&
        !opts.finalEurocTrajectory.parent_path().empty()) {
        fs::create_directories(opts.finalEurocTrajectory.parent_path());
    }

    std::ofstream realtimeCsv(opts.outputCsv);
    if (!realtimeCsv) {
        std::cerr << "failed to open output csv: " << opts.outputCsv << "\n";
        return 1;
    }
    WriteReplayCsvHeader(realtimeCsv);

    std::optional<Sophus::SE3f> bodyFromCameraExtrinsics;
    if (!UseImu(opts.sensorMode) && opts.sensorMode == SensorMode::Stereo &&
        EnvFlagEnabled("SMART_DRONE_EUROC_OUTPUT_BODY_FRAME", false)) {
        bodyFromCameraExtrinsics = LoadBodyToCameraExtrinsics(opts.settings);
        if (bodyFromCameraExtrinsics.has_value()) {
            const Eigen::Vector3f t = bodyFromCameraExtrinsics->translation();
            std::cerr << "[offline_replay] pure stereo realtime CSV uses body frame "
                         "via T_b_c1"
                      << " tx=" << t.x() << " ty=" << t.y() << " tz=" << t.z()
                      << "\n";
        }
    }

    SmartDrone::tests::ReplaySlamRunner runner(camera, imu, *slamEngine,
                                               {.cameraFps = opts.cameraFps,
                                                .slamInputFps = opts.slamInputFps,
                                                .useImu = UseImu(opts.sensorMode),
                                                .preferLatestFrame = true,
                                                .timeoutMs = opts.timeoutMs,
                                                .shutdownEngineOnFinish = false});
    SmartDrone::core::application::FrameTimingTracker timingTracker(512);
    auto outputs =
        runner.Run(opts.maxFrames, &timingTracker,
                   [&](const SmartDrone::tests::ReplayPoseSample &sample) {
                       WriteReplayCsvSample(
                           realtimeCsv,
                           AdjustReplayOutputSample(
                               sample, eurocOutputTimestampOffsetNs,
                               eurocOutputPositionScale, bodyFromCameraExtrinsics));
                       realtimeCsv.flush();
                   });
    realtimeCsv.flush();

    if (outputs.empty()) {
        slamEngine->Stop();
        if (visualFeatureFrontendClient != nullptr) {
            visualFeatureFrontendClient->Stop();
        }
        std::cerr << "offline replay failed: no output frames; check dataset, "
                     "camera provider, or SLAM backend startup\n";
        return 3;
    }

    if (opts.lkLoopClosure && opts.featureFrontend == FeatureFrontend::LK) {
        std::cerr << "warning: --lk-loop-closure is disabled for euroc_pose.csv "
                     "because replay output is realtime-only\n";
    }

    size_t poseValidCount = 0;
    size_t trackingOkCount = 0;
    size_t trackingLostCount = 0;
    size_t identityPoseCount = 0;
    double orbTrackMsSum = 0.0;
    double orbExtractMsSum = 0.0;
    double orbStereoMsSum = 0.0;
    double orbTrackMsMax = 0.0;
    double orbExtractMsMax = 0.0;
    double orbStereoMsMax = 0.0;
    MetricAccumulator replayAcquireMs;
    MetricAccumulator replayImuMs;
    MetricAccumulator slamTotalMs;
    MetricAccumulator inputPrepareMs;
    MetricAccumulator frontendMs;
    MetricAccumulator stereoPairMs;
    MetricAccumulator featurePackMs;
    MetricAccumulator monoAugmentMs;
    MetricAccumulator lkRectifyMs;
    MetricAccumulator lkDisparityMs;
    MetricAccumulator lkGfttMs;
    MetricAccumulator lkFlowMs;
    MetricAccumulator lkCandidateMs;
    MetricAccumulator lkPnpMs;
    MetricAccumulator lkUpdateMs;
    MetricAccumulator visualFeatureFrontendMs;
    MetricAccumulator visualFeatureMatchMs;
    MetricAccumulator visualFeatureTotalMs;
    for (const auto &sample : outputs) {
        if (sample.poseValid) {
            ++poseValidCount;
        }
        if (sample.trackingState == 2 || sample.trackingState == 3) {
            ++trackingOkCount;
        }
        if (sample.trackingState == 1) {
            ++trackingLostCount;
        }
        const bool identityPose = sample.poseValid && sample.pose.x == 0.0f &&
                                  sample.pose.y == 0.0f && sample.pose.z == 0.0f &&
                                  sample.pose.qw == 1.0f &&
                                  sample.pose.qx == 0.0f &&
                                  sample.pose.qy == 0.0f && sample.pose.qz == 0.0f;
        if (identityPose) {
            ++identityPoseCount;
        }
        orbTrackMsSum += sample.orbTrackMs;
        orbExtractMsSum += sample.orbExtractMs;
        orbStereoMsSum += sample.orbStereoMatchMs;
        orbTrackMsMax = std::max(orbTrackMsMax, sample.orbTrackMs);
        orbExtractMsMax = std::max(orbExtractMsMax, sample.orbExtractMs);
        orbStereoMsMax = std::max(orbStereoMsMax, sample.orbStereoMatchMs);
        replayAcquireMs.Add(sample.replayAcquireMs);
        replayImuMs.Add(sample.replayImuMs);
        slamTotalMs.Add(sample.slamTotalMs);
        inputPrepareMs.Add(sample.inputPrepareMs);
        frontendMs.Add(sample.frontendMs);
        stereoPairMs.Add(sample.stereoPairMs);
        featurePackMs.Add(sample.featurePackMs);
        monoAugmentMs.Add(sample.monoAugmentMs);
        lkRectifyMs.Add(sample.lkRectifyMs);
        lkDisparityMs.Add(sample.lkDisparityMs);
        lkGfttMs.Add(sample.lkGfttMs);
        lkFlowMs.Add(sample.lkFlowMs);
        lkCandidateMs.Add(sample.lkCandidateMs);
        lkPnpMs.Add(sample.lkPnpMs);
        lkUpdateMs.Add(sample.lkUpdateMs);
        visualFeatureFrontendMs.Add(sample.visualFeatureFrontendMs);
        visualFeatureMatchMs.Add(sample.visualFeatureStereoMatchMs);
        visualFeatureTotalMs.Add(sample.visualFeatureTotalMs);
    }
    const double frameCountForMean =
        static_cast<double>(std::max<size_t>(1, outputs.size()));
    const double orbTrackMsMean = orbTrackMsSum / frameCountForMean;
    const double orbExtractMsMean = orbExtractMsSum / frameCountForMean;
    const double orbStereoMsMean = orbStereoMsSum / frameCountForMean;

    if (!opts.finalEurocTrajectory.empty()) {
        if (slamControl == nullptr) {
            std::cerr << "failed to save final EuRoC trajectory: final trajectory "
                         "export is ORB-SLAM3 only\n";
            return 1;
        }
        if (!slamControl->ShutdownAndSaveTrajectoryEuRoC(
                opts.finalEurocTrajectory.string())) {
            std::cerr << "failed to save final EuRoC trajectory: "
                      << opts.finalEurocTrajectory << "\n";
            return 1;
        }
    } else {
        slamEngine->Stop();
    }

    std::cout << "offline replay complete\n";
    std::cout << "  dataset: " << opts.datasetRoot << "\n";
    std::cout << "  settings: " << opts.settings << "\n";
    if (opts.slamBackend == SlamBackend::OrbSlam3) {
        std::cout << "  vocab: " << opts.vocab << "\n";
    }
    std::cout << "  sensor_mode: " << ToSensorModeText(opts.sensorMode) << "\n";
    std::cout << "  slam_backend: " << ToSlamBackendText(opts.slamBackend)
              << "\n";
    std::cout << "  feature_frontend: "
              << ToFeatureFrontendText(opts.featureFrontend) << "\n";
    std::cout << "  euroc_output_timestamp_offset_ms: "
              << (static_cast<double>(eurocOutputTimestampOffsetNs) / 1000000.0)
              << "\n";
    std::cout << "  euroc_output_position_scale: " << eurocOutputPositionScale
              << "\n";
    std::cout << "  orb_accel: " << opts.orbAcceleration << "\n";
    std::cout << "  lk_per_frame_accel: " << opts.lkPerFrameAcceleration << "\n";
    if (opts.slamBackend == SlamBackend::DpvoTensorRt) {
        std::cout << "  dpvo_repo: " << opts.dpvoRepo << "\n";
        std::cout << "  dpvo_patch_engine: " << opts.dpvoPatchEngine << "\n";
        std::cout << "  dpvo_update_engine: " << opts.dpvoUpdateEngine << "\n";
        std::cout << "  dpvo_input: " << std::clamp(opts.dpvoInputWidth, 160, 1280)
                  << "x" << std::clamp(opts.dpvoInputHeight, 120, 960) << "\n";
    }
    std::cout << "  frames_out: " << outputs.size() << "\n";
    std::cout << "  pose_valid_frames: " << poseValidCount << "\n";
    std::cout << "  tracking_ok_frames: " << trackingOkCount << "\n";
    std::cout << "  tracking_lost_frames: " << trackingLostCount << "\n";
    std::cout << "  identity_pose_frames: " << identityPoseCount << "\n";
    std::cout << "  replay_acquire_ms_mean/max: "
              << replayAcquireMs.Mean(outputs.size()) << "/"
              << replayAcquireMs.max << "\n";
    std::cout << "  replay_imu_ms_mean/max: " << replayImuMs.Mean(outputs.size())
              << "/" << replayImuMs.max << "\n";
    std::cout << "  slam_total_ms_mean/max: " << slamTotalMs.Mean(outputs.size())
              << "/" << slamTotalMs.max << "\n";
    std::cout << "  input_prepare_ms_mean/max: "
              << inputPrepareMs.Mean(outputs.size()) << "/" << inputPrepareMs.max
              << "\n";
    std::cout << "  frontend_ms_mean/max: " << frontendMs.Mean(outputs.size())
              << "/" << frontendMs.max << "\n";
    std::cout << "  stereo_pair_ms_mean/max: "
              << stereoPairMs.Mean(outputs.size()) << "/" << stereoPairMs.max
              << "\n";
    std::cout << "  feature_pack_ms_mean/max: "
              << featurePackMs.Mean(outputs.size()) << "/" << featurePackMs.max
              << "\n";
    std::cout << "  external_pack_ms_mean/max: "
              << featurePackMs.Mean(outputs.size()) << "/" << featurePackMs.max
              << "\n";
    std::cout << "  lk_disparity_ms_mean/max: "
              << lkDisparityMs.Mean(outputs.size()) << "/" << lkDisparityMs.max
              << "\n";
    std::cout << "  lk_gftt_ms_mean/max: " << lkGfttMs.Mean(outputs.size()) << "/"
              << lkGfttMs.max << "\n";
    std::cout << "  lk_flow_ms_mean/max: " << lkFlowMs.Mean(outputs.size()) << "/"
              << lkFlowMs.max << "\n";
    std::cout << "  lk_candidate_ms_mean/max: "
              << lkCandidateMs.Mean(outputs.size()) << "/" << lkCandidateMs.max
              << "\n";
    std::cout << "  lk_pnp_ms_mean/max: " << lkPnpMs.Mean(outputs.size()) << "/"
              << lkPnpMs.max << "\n";
    std::cout << "  orb_track_ms_mean/max: " << orbTrackMsMean << "/"
              << orbTrackMsMax << "\n";
    std::cout << "  orb_extract_ms_mean/max: " << orbExtractMsMean << "/"
              << orbExtractMsMax << "\n";
    std::cout << "  orb_stereo_ms_mean/max: " << orbStereoMsMean << "/"
              << orbStereoMsMax << "\n";
    std::cout << "  output_csv: " << opts.outputCsv << "\n";
    if (!opts.finalEurocTrajectory.empty()) {
        std::cout << "  final_euroc_trajectory: " << opts.finalEurocTrajectory
                  << "\n";
    }

    if (!opts.summaryJson.empty()) {
        std::ofstream summary(opts.summaryJson);
        if (!summary) {
            std::cerr << "failed to open summary json: " << opts.summaryJson << "\n";
            return 1;
        }
        summary
            << "{\n"
            << "  \"dataset\": \"" << opts.datasetRoot.string() << "\",\n"
            << "  \"settings\": \"" << opts.settings << "\",\n"
            << "  \"vocab\": \""
            << (opts.slamBackend == SlamBackend::OrbSlam3 ? opts.vocab : "")
            << "\",\n"
            << "  \"sensor_mode\": \"" << ToSensorModeText(opts.sensorMode)
            << "\",\n"
            << "  \"slam_backend\": \"" << ToSlamBackendText(opts.slamBackend)
            << "\",\n"
            << "  \"feature_frontend\": \""
            << ToFeatureFrontendText(opts.featureFrontend) << "\",\n"
            << "  \"euroc_output_timestamp_offset_ms\": "
            << (static_cast<double>(eurocOutputTimestampOffsetNs) / 1000000.0)
            << ",\n"
            << "  \"euroc_output_position_scale\": " << eurocOutputPositionScale
            << ",\n"
            << "  \"orb_accel\": \"" << opts.orbAcceleration << "\",\n"
            << "  \"dpvo_repo\": \"" << opts.dpvoRepo << "\",\n"
            << "  \"dpvo_patch_engine\": \"" << opts.dpvoPatchEngine << "\",\n"
            << "  \"dpvo_update_engine\": \"" << opts.dpvoUpdateEngine << "\",\n"
            << "  \"dpvo_input_width\": "
            << std::clamp(opts.dpvoInputWidth, 160, 1280) << ",\n"
            << "  \"dpvo_input_height\": "
            << std::clamp(opts.dpvoInputHeight, 120, 960) << ",\n"
            << "  \"dpvo_patches_per_frame\": "
            << std::clamp(opts.dpvoPatchesPerFrame, 16, 256) << ",\n"
            << "  \"dpvo_optimization_window\": "
            << std::clamp(opts.dpvoOptimizationWindow, 4, 32) << ",\n"
            << "  \"lk_per_frame_accel\": \"" << opts.lkPerFrameAcceleration
            << "\",\n"
            << "  \"frames_out\": " << outputs.size() << ",\n"
            << "  \"pose_valid_frames\": " << poseValidCount << ",\n"
            << "  \"tracking_ok_frames\": " << trackingOkCount << ",\n"
            << "  \"tracking_lost_frames\": " << trackingLostCount << ",\n"
            << "  \"identity_pose_frames\": " << identityPoseCount << ",\n"
            << "  \"replay_acquire_ms_mean\": "
            << replayAcquireMs.Mean(outputs.size()) << ",\n"
            << "  \"replay_acquire_ms_max\": " << replayAcquireMs.max << ",\n"
            << "  \"replay_imu_ms_mean\": " << replayImuMs.Mean(outputs.size())
            << ",\n"
            << "  \"replay_imu_ms_max\": " << replayImuMs.max << ",\n"
            << "  \"slam_total_ms_mean\": " << slamTotalMs.Mean(outputs.size())
            << ",\n"
            << "  \"slam_total_ms_max\": " << slamTotalMs.max << ",\n"
            << "  \"input_prepare_ms_mean\": "
            << inputPrepareMs.Mean(outputs.size()) << ",\n"
            << "  \"input_prepare_ms_max\": " << inputPrepareMs.max << ",\n"
            << "  \"frontend_ms_mean\": " << frontendMs.Mean(outputs.size())
            << ",\n"
            << "  \"frontend_ms_max\": " << frontendMs.max << ",\n"
            << "  \"stereo_pair_ms_mean\": " << stereoPairMs.Mean(outputs.size())
            << ",\n"
            << "  \"stereo_pair_ms_max\": " << stereoPairMs.max << ",\n"
            << "  \"feature_pack_ms_mean\": " << featurePackMs.Mean(outputs.size())
            << ",\n"
            << "  \"feature_pack_ms_max\": " << featurePackMs.max << ",\n"
            << "  \"external_pack_ms_mean\": " << featurePackMs.Mean(outputs.size())
            << ",\n"
            << "  \"external_pack_ms_max\": " << featurePackMs.max << ",\n"
            << "  \"mono_augment_ms_mean\": " << monoAugmentMs.Mean(outputs.size())
            << ",\n"
            << "  \"mono_augment_ms_max\": " << monoAugmentMs.max << ",\n"
            << "  \"lk_rectify_ms_mean\": " << lkRectifyMs.Mean(outputs.size())
            << ",\n"
            << "  \"lk_rectify_ms_max\": " << lkRectifyMs.max << ",\n"
            << "  \"lk_disparity_ms_mean\": " << lkDisparityMs.Mean(outputs.size())
            << ",\n"
            << "  \"lk_disparity_ms_max\": " << lkDisparityMs.max << ",\n"
            << "  \"lk_gftt_ms_mean\": " << lkGfttMs.Mean(outputs.size()) << ",\n"
            << "  \"lk_gftt_ms_max\": " << lkGfttMs.max << ",\n"
            << "  \"lk_flow_ms_mean\": " << lkFlowMs.Mean(outputs.size()) << ",\n"
            << "  \"lk_flow_ms_max\": " << lkFlowMs.max << ",\n"
            << "  \"lk_candidate_ms_mean\": " << lkCandidateMs.Mean(outputs.size())
            << ",\n"
            << "  \"lk_candidate_ms_max\": " << lkCandidateMs.max << ",\n"
            << "  \"lk_pnp_ms_mean\": " << lkPnpMs.Mean(outputs.size()) << ",\n"
            << "  \"lk_pnp_ms_max\": " << lkPnpMs.max << ",\n"
            << "  \"lk_update_ms_mean\": " << lkUpdateMs.Mean(outputs.size())
            << ",\n"
            << "  \"lk_update_ms_max\": " << lkUpdateMs.max << ",\n"
            << "  \"visual_feature_frontend_ms_mean\": "
            << visualFeatureFrontendMs.Mean(outputs.size()) << ",\n"
            << "  \"visual_feature_frontend_ms_max\": "
            << visualFeatureFrontendMs.max << ",\n"
            << "  \"visual_feature_match_ms_mean\": "
            << visualFeatureMatchMs.Mean(outputs.size()) << ",\n"
            << "  \"visual_feature_match_ms_max\": " << visualFeatureMatchMs.max
            << ",\n"
            << "  \"visual_feature_total_ms_mean\": "
            << visualFeatureTotalMs.Mean(outputs.size()) << ",\n"
            << "  \"visual_feature_total_ms_max\": " << visualFeatureTotalMs.max
            << ",\n"
            << "  \"superpoint_frontend_ms_mean\": "
            << visualFeatureFrontendMs.Mean(outputs.size()) << ",\n"
            << "  \"superpoint_frontend_ms_max\": " << visualFeatureFrontendMs.max
            << ",\n"
            << "  \"superpoint_match_ms_mean\": "
            << visualFeatureMatchMs.Mean(outputs.size()) << ",\n"
            << "  \"superpoint_match_ms_max\": " << visualFeatureMatchMs.max
            << ",\n"
            << "  \"superpoint_total_ms_mean\": "
            << visualFeatureTotalMs.Mean(outputs.size()) << ",\n"
            << "  \"superpoint_total_ms_max\": " << visualFeatureTotalMs.max
            << ",\n"
            << "  \"orb_track_ms_mean\": " << orbTrackMsMean << ",\n"
            << "  \"orb_track_ms_max\": " << orbTrackMsMax << ",\n"
            << "  \"orb_extract_ms_mean\": " << orbExtractMsMean << ",\n"
            << "  \"orb_extract_ms_max\": " << orbExtractMsMax << ",\n"
            << "  \"orb_stereo_ms_mean\": " << orbStereoMsMean << ",\n"
            << "  \"orb_stereo_ms_max\": " << orbStereoMsMax << ",\n"
            << "  \"output_csv\": \"" << opts.outputCsv.string() << "\",\n"
            << "  \"final_euroc_trajectory\": \""
            << opts.finalEurocTrajectory.string() << "\"\n"
            << "}\n";
        summary.flush();
        std::cout << "  summary_json: " << opts.summaryJson << "\n";
    }

    if (visualFeatureFrontendClient != nullptr) {
        visualFeatureFrontendClient->Stop();
    }
    std::cout.flush();
    std::_Exit(0);
}

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
