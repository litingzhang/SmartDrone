#include <cstdlib>
#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "System.h"
#include "adapters/slam/slam_engine_adapter.h"
#include "adapters/slam/superpoint_lightglue_frontend_client.h"
#include "core/application/config/app_args.h"
#include "core/domain/runtime_mode.h"
#include "support/replay_dataset.h"
#include "support/replay_slam_runner.h"

namespace fs = std::filesystem;

namespace {

struct OfflineReplayOptions {
    fs::path datasetRoot{fs::path(TESTS_SOURCE_DIR) / "data"};
    fs::path outputCsv{"build/offline_replay_pose.csv"};
    fs::path summaryJson{};
    fs::path finalEurocTrajectory{};
    std::string vocab{"ORB_SLAM3/Vocabulary/ORBvoc.txt"};
    std::string settings{"config/stereo.yaml"};
    SensorMode sensorMode{SensorMode::StereoImu};
    FeatureFrontend featureFrontend{FeatureFrontend::Orb};
    smartdrone::core::domain::SlamOperationMode slamMode{smartdrone::core::domain::SlamOperationMode::Mapping};
    std::string superpointRepo{"LightGlue"};
    std::string superpointDevice{"auto"};
    int superpointTopK{1024};
    int superpointMaxPoints{512};
    int superpointInputMaxWidth{640};
    int superpointInputMaxHeight{409};
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
};

bool SuperPointLightGlueInjectionEnabled()
{
    const char *value = std::getenv("SMART_DRONE_SUPERPOINT_LIGHTGLUE_INJECT");
    if (value == nullptr || value[0] == '\0') {
        return true;
    }
    const std::string text(value);
    return !(text == "0" || text == "false" || text == "FALSE" || text == "off" || text == "OFF");
}

bool EnvVarIsUnsetOrEmpty(const char *name)
{
    const char *value = std::getenv(name);
    return value == nullptr || value[0] == '\0';
}

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
    const double offsetMs = EnvDoubleValue("SMART_DRONE_EUROC_OUTPUT_TIMESTAMP_OFFSET_MS", 0.0);
    return static_cast<int64_t>(std::llround(offsetMs * 1000000.0));
}

double EnvOutputPositionScale()
{
    return EnvDoubleValue("SMART_DRONE_EUROC_OUTPUT_POSITION_SCALE", 1.0);
}

int64_t SaturatingTimestampAdd(int64_t timestampNs, int64_t offsetNs)
{
    if (offsetNs > 0 && timestampNs > std::numeric_limits<int64_t>::max() - offsetNs) {
        return std::numeric_limits<int64_t>::max();
    }
    if (offsetNs < 0 && timestampNs < std::numeric_limits<int64_t>::min() - offsetNs) {
        return std::numeric_limits<int64_t>::min();
    }
    return timestampNs + offsetNs;
}

smartdrone::tests::ReplayPoseSample AdjustReplayOutputSample(smartdrone::tests::ReplayPoseSample sample,
                                                             int64_t timestampOffsetNs,
                                                             double positionScale)
{
    if (timestampOffsetNs != 0) {
        sample.captureTimestampNs = SaturatingTimestampAdd(sample.captureTimestampNs, timestampOffsetNs);
    }
    if (positionScale != 1.0) {
        sample.pose.x *= positionScale;
        sample.pose.y *= positionScale;
        sample.pose.z *= positionScale;
    }
    return sample;
}

void SetEnvIfUnset(const char *name, const char *value)
{
    if (!EnvVarIsUnsetOrEmpty(name)) {
        return;
    }
    setenv(name, value, 0);
}

void ConfigureSuperPointLightGlueReplayDefaults()
{
    SetEnvIfUnset("SMART_DRONE_SP_LG_FILTERED_STEREO_INJECT", "1");
    SetEnvIfUnset("SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_FEATURES", "72");
    SetEnvIfUnset("SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_CLOSE_POINTS", "24");
    SetEnvIfUnset("SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_CLOSE_RATIO", "0.30");
    SetEnvIfUnset("SMART_DRONE_EXTERNAL_STEREO_DEPTH_SCALE", "0.965");
    SetEnvIfUnset("SMART_DRONE_SP_LG_INIT_TRUST_FRONTEND_PAIRS", "1");
    SetEnvIfUnset("SMART_DRONE_SP_LG_RECOVERY_TRUST_FRONTEND_PAIRS", "1");
    SetEnvIfUnset("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_FRONTEND_PAIRS", "1");
    SetEnvIfUnset("SMART_DRONE_SP_LG_TRUST_FRONTEND_PAIRS_OK_STREAK", "120");
    SetEnvIfUnset("SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE", "1");
    SetEnvIfUnset("SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE_TIMEOUT_MS", "35");
    SetEnvIfUnset("SMART_DRONE_ORB_LIVE_EUROC_TRAJECTORY_POSE", "0");
    SetEnvIfUnset("SMART_DRONE_REALTIME_POSE_CONTINUITY", "1");
}

struct MetricAccumulator {
    double sum{0.0};
    double max{0.0};

    void Add(double value)
    {
        sum += value;
        max = std::max(max, value);
    }

    double Mean(size_t count) const { return sum / static_cast<double>(std::max<size_t>(1, count)); }
};

void WriteReplayCsvHeader(std::ostream &csv)
{
    csv << "frame_id,capture_timestamp_ns,tracking_state,map_id,pose_valid,x,y,z,qw,qx,qy,qz,imu_samples,"
           "superpoint_used,superpoint_raw_left,superpoint_raw_right,superpoint_stereo,superpoint_injected_left,superpoint_injected_right,superpoint_external_hash,"
           "superpoint_lg_every_n,superpoint_frontend_ms,superpoint_match_ms,superpoint_total_ms,replay_acquire_ms,replay_imu_ms,slam_total_ms,"
           "input_prepare_ms,frontend_ms,stereo_pair_ms,external_pack_ms,mono_augment_ms,"
           "lk_rectify_ms,lk_disparity_ms,lk_gftt_ms,lk_flow_ms,lk_candidate_ms,lk_pnp_ms,lk_update_ms,"
           "orb_track_ms,orb_extract_ms,orb_stereo_ms,local_mapping_wait_ms,local_mapping_wait_timeout_ms,"
           "local_mapping_queue_before,local_mapping_queue_after,local_mapping_accept_before,"
           "local_mapping_accept_after,local_mapping_wait_requested,local_mapping_wait_timeout,"
           "inliers,tracked_map,local_map,local_map_hash,matched_map_hash_before_po,tracked_map_hash,close_map,orb_frame_id,ref_kf,last_kf,last_kf_frame,keyframes_in_map,"
           "external_init_frame,external_injected,external_bootstrap,external_stabilizing,"
           "realtime_pose_gate,raw_pose_step_m,gated_pose_step_m\n";
}

void WriteReplayCsvSample(std::ostream &csv, const smartdrone::tests::ReplayPoseSample &sample)
{
    csv << sample.frameId << ',' << sample.captureTimestampNs << ',' << sample.trackingState << ','
        << sample.mapId << ',' << (sample.poseValid ? 1 : 0) << ',' << sample.pose.x << ',' << sample.pose.y
        << ',' << sample.pose.z << ',' << sample.pose.qw << ',' << sample.pose.qx << ',' << sample.pose.qy
        << ',' << sample.pose.qz << ',' << sample.imuSampleCount << ',' << (sample.usedSuperPointFrontend ? 1 : 0)
        << ',' << sample.superpointRawLeftCount << ',' << sample.superpointRawRightCount << ','
        << sample.superpointMatchedStereoCount << ',' << sample.superpointInjectedLeftCount << ','
        << sample.superpointInjectedRightCount << ',' << sample.superpointExternalHash << ','
        << sample.superpointLightGlueEveryN << ','
        << sample.superpointFrontendMs << ','
        << sample.superpointStereoMatchMs << ',' << sample.superpointTotalMs << ',' << sample.replayAcquireMs << ','
        << sample.replayImuMs << ',' << sample.slamTotalMs << ',' << sample.inputPrepareMs << ','
        << sample.frontendMs << ',' << sample.stereoPairMs << ',' << sample.externalPackMs << ','
        << sample.monoAugmentMs << ',' << sample.lkRectifyMs << ',' << sample.lkDisparityMs << ','
        << sample.lkGfttMs << ',' << sample.lkFlowMs << ',' << sample.lkCandidateMs << ','
        << sample.lkPnpMs << ',' << sample.lkUpdateMs << ',' << sample.orbTrackMs << ','
        << sample.orbExtractMs << ',' << sample.orbStereoMatchMs << ',' << sample.localMappingWaitMs << ','
        << sample.localMappingWaitTimeoutMs << ',' << sample.localMappingWaitQueueBefore << ','
        << sample.localMappingWaitQueueAfter << ',' << (sample.localMappingAcceptingBefore ? 1 : 0) << ','
        << (sample.localMappingAcceptingAfter ? 1 : 0) << ',' << (sample.localMappingWaitRequested ? 1 : 0)
        << ',' << (sample.localMappingWaitTimedOut ? 1 : 0) << ',' << sample.matchesInliers << ','
        << sample.trackedMapPointCount << ',' << sample.localMapPointCount << ',' << sample.localMapPointHash
        << ',' << sample.matchedMapPointHashBeforePoseOptimization << ',' << sample.trackedMapPointHash
        << ',' << sample.closeMapPointCount
        << ',' << sample.orbFrameId << ',' << sample.referenceKeyFrameId << ',' << sample.lastKeyFrameId << ','
        << sample.lastKeyFrameFrameId << ',' << sample.keyFramesInMap << ',' << sample.externalStereoInitFrameId
        << ',' << (sample.externalStereoInjected ? 1 : 0) << ',' << (sample.externalStereoBootstrap ? 1 : 0)
        << ',' << (sample.externalStereoStabilizing ? 1 : 0) << ','
        << (sample.realtimePoseQualityGate ? 1 : 0) << ',' << sample.rawPoseStepMeters << ','
        << sample.gatedPoseStepMeters << '\n';
}

const char *UsageText()
{
    return
        "Usage: smart_drone_offline_replay [options]\n"
        "  --dataset <dir>       Replay dataset root, default tests/data; accepts tests/data or EuRoC mav0 layout\n"
        "  --out <file>          Output CSV path, default build/offline_replay_pose.csv\n"
        "  --summary-json <file> Optional summary JSON output path\n"
        "  --final-euroc-trajectory <file> Optional final ORB-SLAM3 EuRoC trajectory after shutdown\n"
        "  --vocab <file>        ORB vocabulary path\n"
        "  --settings <file>     ORB settings YAML path\n"
        "  --sensor-mode <mode>  stereo|stereo-imu|mono|mono-imu\n"
        "  --stereo-only         Shortcut for --sensor-mode stereo\n"
        "  --feature-frontend <mode> orb|superpoint_lightglue|lk|lk_gftt_per_frame, default orb\n"
        "  --superpoint-repo <dir>    SuperPoint/LightGlue repo root containing TensorRT engines\n"
        "  --superpoint-device <dev>  TensorRT device auto|cuda, default auto\n"
        "  --superpoint-top-k <n>     SuperPoint top-k candidate count, default 1024\n"
        "  --superpoint-max-points <n> SuperPoint injected point budget, default 512\n"
        "  --superpoint-input-max-width <n>  SuperPoint input width limit, default 640\n"
        "  --superpoint-input-max-height <n> SuperPoint input height limit, default 409\n"
        "  --slam-mode <mode>    mapping|localization|relocalization|tracking-only|auto\n"
        "  --fps <n>             Camera FPS for replay pacing, default 60\n"
        "  --slam-fps <n>        SLAM input FPS, default 20\n"
        "  --timeout-ms <n>      Batch acquire timeout, default 1000\n"
        "  --max-frames <n>      Maximum output frames, default 0(all)\n"
        "  --lk-loop-closure     Compatibility flag; disabled for realtime CSV output\n"
        "  --lk-runtime-loop-closure Enable image-based LK keyframe loop closure during replay\n"
        "  --lk-loop-scale <f>   Sim3 scale used by LK loop closure, default 1.20\n"
        "  --lk-loop-relax <f>   Loop residual relaxation factor, default 1.40\n"
        "  --lk-per-frame-accel <auto|cpu|vpi-cuda>  Per-frame LK GFTT stereo backend, default cpu\n"
        "  --orb-accel <cpu|cuda|vpi-remap>  ORB acceleration/preprocess mode, default cpu\n";
}

std::string NormalizeOrbAccelerationText(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (value == "cuda" || value == "gpu" || value == "opencv_cuda" || value == "opencv-cuda") {
        return "cuda";
    }
    if (value == "vpi" || value == "vpi_remap" || value == "vpi-remap" || value == "vpi_cuda_remap" ||
        value == "vpi-cuda-remap") {
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

std::string GetOptionValue(int argc, char **argv, const char *name, const std::string &defaultValue)
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

float GetOptionFloat(int argc, char **argv, const char *name, float defaultValue)
{
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == name) {
            return std::stof(argv[i + 1]);
        }
    }
    return defaultValue;
}

size_t GetOptionSize(int argc, char **argv, const char *name, size_t defaultValue)
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

    opts.datasetRoot = fs::path(GetOptionValue(argc, argv, "--dataset", opts.datasetRoot.string()));
    opts.outputCsv = fs::path(GetOptionValue(argc, argv, "--out", opts.outputCsv.string()));
    opts.summaryJson = fs::path(GetOptionValue(argc, argv, "--summary-json", ""));
    opts.finalEurocTrajectory = fs::path(GetOptionValue(argc, argv, "--final-euroc-trajectory", ""));
    opts.sensorMode = ParseSensorModeText(GetOptionValue(argc, argv, "--sensor-mode", "stereo-imu"));
    if (HasFlag(argc, argv, "--stereo-only")) {
        opts.sensorMode = SensorMode::Stereo;
    }
    opts.featureFrontend = ParseFeatureFrontendText(GetOptionValue(argc, argv, "--feature-frontend", "orb"));
    opts.slamMode = ParseSlamOperationModeText(GetOptionValue(argc, argv, "--slam-mode", "mapping"));
    opts.cameraFps = GetOptionInt(argc, argv, "--fps", opts.cameraFps);
    opts.slamInputFps = GetOptionInt(argc, argv, "--slam-fps", opts.slamInputFps);
    opts.timeoutMs = GetOptionInt(argc, argv, "--timeout-ms", opts.timeoutMs);
    opts.maxFrames = GetOptionSize(argc, argv, "--max-frames", opts.maxFrames);
    opts.superpointRepo =
        ResolveRuntimePath(GetOptionValue(argc, argv, "--superpoint-repo", opts.superpointRepo), argc > 0 ? argv[0] : nullptr);
    opts.superpointDevice = GetOptionValue(argc, argv, "--superpoint-device", opts.superpointDevice);
    opts.superpointTopK = GetOptionInt(argc, argv, "--superpoint-top-k", opts.superpointTopK);
    opts.superpointMaxPoints = GetOptionInt(argc, argv, "--superpoint-max-points", opts.superpointMaxPoints);
    opts.superpointInputMaxWidth = GetOptionInt(argc, argv, "--superpoint-input-max-width", opts.superpointInputMaxWidth);
    opts.superpointInputMaxHeight = GetOptionInt(argc, argv, "--superpoint-input-max-height", opts.superpointInputMaxHeight);
    opts.lkLoopClosure = HasFlag(argc, argv, "--lk-loop-closure");
    opts.lkRuntimeLoopClosure = HasFlag(argc, argv, "--lk-runtime-loop-closure");
    opts.lkLoopScale = GetOptionFloat(argc, argv, "--lk-loop-scale", opts.lkLoopScale);
    opts.lkLoopRelaxation = GetOptionFloat(argc, argv, "--lk-loop-relax", opts.lkLoopRelaxation);
    opts.lkPerFrameAcceleration = GetOptionValue(argc, argv, "--lk-per-frame-accel", "cpu");
    opts.orbAcceleration = NormalizeOrbAccelerationText(GetOptionValue(argc, argv, "--orb-accel", "cpu"));
    opts.vocab = ResolveRuntimePath(GetOptionValue(argc, argv, "--vocab", opts.vocab), argc > 0 ? argv[0] : nullptr);
    opts.settings =
        ResolveRuntimePath(GetOptionValue(argc, argv, "--settings", DefaultSettingsForSensorMode(opts.sensorMode)),
                           argc > 0 ? argv[0] : nullptr);
    return opts;
}

ORB_SLAM3::System::eSensor ResolveOrbSensor(SensorMode mode)
{
    switch (mode) {
    case SensorMode::MonoImu:
        return ORB_SLAM3::System::IMU_MONOCULAR;
    case SensorMode::Mono:
        return ORB_SLAM3::System::MONOCULAR;
    case SensorMode::StereoImu:
        return ORB_SLAM3::System::IMU_STEREO;
    case SensorMode::Stereo:
    default:
        return ORB_SLAM3::System::STEREO;
    }
}

smartdrone::adapters::slam::SlamInputMode ResolveSlamInputMode(SensorMode mode)
{
    return (mode == SensorMode::Mono || mode == SensorMode::MonoImu) ? smartdrone::adapters::slam::SlamInputMode::MonoRight
                                                                     : smartdrone::adapters::slam::SlamInputMode::Stereo;
}

bool UseImu(SensorMode mode) { return mode == SensorMode::StereoImu || mode == SensorMode::MonoImu; }

int RunOfflineReplay(const OfflineReplayOptions &opts)
{
    const int64_t eurocOutputTimestampOffsetNs = EnvTimestampOffsetNs();
    const double eurocOutputPositionScale = EnvOutputPositionScale();
    const smartdrone::tests::ReplayDataset dataset = smartdrone::tests::ReplayDataset::Load(opts.datasetRoot);
    if (dataset.Empty()) {
        std::cerr << "dataset is empty: " << opts.datasetRoot << "\n";
        return 1;
    }

    smartdrone::tests::ReplayCameraProvider camera(dataset);
    smartdrone::tests::ReplayImuProvider imu(dataset);
    const auto sensor = ResolveOrbSensor(opts.sensorMode);
    ApplyOrbAccelerationEnvironment(opts.orbAcceleration);
    auto orbSystem = std::make_unique<ORB_SLAM3::System>(opts.vocab, opts.settings, sensor, false);
    smartdrone::adapters::slam::SlamEngineAdapter slamEngine(std::move(orbSystem), ResolveSlamInputMode(opts.sensorMode),
                                                          UseImu(opts.sensorMode), opts.settings);
    slamEngine.SetOperationMode(opts.slamMode);
    slamEngine.SetFeatureFrontend(opts.featureFrontend);
    slamEngine.SetExternalFeatureInputSizeLimit(opts.superpointInputMaxWidth, opts.superpointInputMaxHeight);
    slamEngine.SetStereoVoLoopClosure(opts.lkRuntimeLoopClosure, opts.lkLoopScale, opts.lkLoopRelaxation);
    slamEngine.SetStereoVoPerFrameAcceleration(opts.lkPerFrameAcceleration);
    smartdrone::adapters::slam::SuperPointLightGlueFrontendClient superpointFrontendClient;
    std::string superpointErr;
    if (opts.featureFrontend == FeatureFrontend::SuperPointLightGlue && SuperPointLightGlueInjectionEnabled()) {
        ConfigureSuperPointLightGlueReplayDefaults();
        slamEngine.SetExternalFeatureFrontendClient(&superpointFrontendClient);
        if (!superpointFrontendClient.Start(opts.superpointRepo, opts.superpointDevice, opts.superpointTopK, opts.superpointMaxPoints,
                                       &superpointErr)) {
            std::cerr << "error: superpoint_lightglue TensorRT start failed: " << superpointErr << "\n";
            return 2;
        }
    }

    if (!opts.outputCsv.parent_path().empty()) {
        fs::create_directories(opts.outputCsv.parent_path());
    }
    if (!opts.summaryJson.empty() && !opts.summaryJson.parent_path().empty()) {
        fs::create_directories(opts.summaryJson.parent_path());
    }
    if (!opts.finalEurocTrajectory.empty() && !opts.finalEurocTrajectory.parent_path().empty()) {
        fs::create_directories(opts.finalEurocTrajectory.parent_path());
    }

    std::ofstream realtimeCsv(opts.outputCsv);
    if (!realtimeCsv) {
        std::cerr << "failed to open output csv: " << opts.outputCsv << "\n";
        return 1;
    }
    WriteReplayCsvHeader(realtimeCsv);

    smartdrone::tests::ReplaySlamRunner runner(camera, imu, slamEngine,
                                               {.cameraFps = opts.cameraFps,
                                                .slamInputFps = opts.slamInputFps,
                                                .useImu = UseImu(opts.sensorMode),
                                                .preferLatestFrame = true,
                                                .timeoutMs = opts.timeoutMs,
                                                .shutdownEngineOnFinish = false});
    smartdrone::core::application::FrameTimingTracker timingTracker(512);
    auto outputs = runner.Run(opts.maxFrames, &timingTracker, [&](const smartdrone::tests::ReplayPoseSample &sample) {
        WriteReplayCsvSample(realtimeCsv, AdjustReplayOutputSample(sample, eurocOutputTimestampOffsetNs,
                                                                    eurocOutputPositionScale));
        realtimeCsv.flush();
    });
    realtimeCsv.flush();

    if (opts.lkLoopClosure && opts.featureFrontend == FeatureFrontend::LK) {
        std::cerr << "warning: --lk-loop-closure is disabled for euroc_pose.csv because replay output is realtime-only\n";
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
    MetricAccumulator externalPackMs;
    MetricAccumulator monoAugmentMs;
    MetricAccumulator lkRectifyMs;
    MetricAccumulator lkDisparityMs;
    MetricAccumulator lkGfttMs;
    MetricAccumulator lkFlowMs;
    MetricAccumulator lkCandidateMs;
    MetricAccumulator lkPnpMs;
    MetricAccumulator lkUpdateMs;
    MetricAccumulator superpointFrontendMs;
    MetricAccumulator superpointMatchMs;
    MetricAccumulator superpointTotalMs;
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
        const bool identityPose = sample.poseValid && sample.pose.x == 0.0f && sample.pose.y == 0.0f &&
                                  sample.pose.z == 0.0f && sample.pose.qw == 1.0f && sample.pose.qx == 0.0f &&
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
        externalPackMs.Add(sample.externalPackMs);
        monoAugmentMs.Add(sample.monoAugmentMs);
        lkRectifyMs.Add(sample.lkRectifyMs);
        lkDisparityMs.Add(sample.lkDisparityMs);
        lkGfttMs.Add(sample.lkGfttMs);
        lkFlowMs.Add(sample.lkFlowMs);
        lkCandidateMs.Add(sample.lkCandidateMs);
        lkPnpMs.Add(sample.lkPnpMs);
        lkUpdateMs.Add(sample.lkUpdateMs);
        superpointFrontendMs.Add(sample.superpointFrontendMs);
        superpointMatchMs.Add(sample.superpointStereoMatchMs);
        superpointTotalMs.Add(sample.superpointTotalMs);
    }
    const double frameCountForMean = static_cast<double>(std::max<size_t>(1, outputs.size()));
    const double orbTrackMsMean = orbTrackMsSum / frameCountForMean;
    const double orbExtractMsMean = orbExtractMsSum / frameCountForMean;
    const double orbStereoMsMean = orbStereoMsSum / frameCountForMean;

    if (!opts.finalEurocTrajectory.empty()) {
        if (!slamEngine.ShutdownAndSaveTrajectoryEuRoC(opts.finalEurocTrajectory.string())) {
            std::cerr << "failed to save final EuRoC trajectory: " << opts.finalEurocTrajectory << "\n";
            return 1;
        }
    } else {
        slamEngine.Stop();
    }

    std::cout << "offline replay complete\n";
    std::cout << "  dataset: " << opts.datasetRoot << "\n";
    std::cout << "  settings: " << opts.settings << "\n";
    std::cout << "  vocab: " << opts.vocab << "\n";
    std::cout << "  sensor_mode: " << ToSensorModeText(opts.sensorMode) << "\n";
    std::cout << "  feature_frontend: " << ToFeatureFrontendText(opts.featureFrontend) << "\n";
    std::cout << "  euroc_output_timestamp_offset_ms: "
              << (static_cast<double>(eurocOutputTimestampOffsetNs) / 1000000.0) << "\n";
    std::cout << "  euroc_output_position_scale: " << eurocOutputPositionScale << "\n";
    std::cout << "  orb_accel: " << opts.orbAcceleration << "\n";
    std::cout << "  lk_per_frame_accel: " << opts.lkPerFrameAcceleration << "\n";
    std::cout << "  frames_out: " << outputs.size() << "\n";
    std::cout << "  pose_valid_frames: " << poseValidCount << "\n";
    std::cout << "  tracking_ok_frames: " << trackingOkCount << "\n";
    std::cout << "  tracking_lost_frames: " << trackingLostCount << "\n";
    std::cout << "  identity_pose_frames: " << identityPoseCount << "\n";
    std::cout << "  replay_acquire_ms_mean/max: " << replayAcquireMs.Mean(outputs.size()) << "/"
              << replayAcquireMs.max << "\n";
    std::cout << "  replay_imu_ms_mean/max: " << replayImuMs.Mean(outputs.size()) << "/" << replayImuMs.max << "\n";
    std::cout << "  slam_total_ms_mean/max: " << slamTotalMs.Mean(outputs.size()) << "/" << slamTotalMs.max << "\n";
    std::cout << "  input_prepare_ms_mean/max: " << inputPrepareMs.Mean(outputs.size()) << "/"
              << inputPrepareMs.max << "\n";
    std::cout << "  frontend_ms_mean/max: " << frontendMs.Mean(outputs.size()) << "/" << frontendMs.max << "\n";
    std::cout << "  stereo_pair_ms_mean/max: " << stereoPairMs.Mean(outputs.size()) << "/" << stereoPairMs.max
              << "\n";
    std::cout << "  external_pack_ms_mean/max: " << externalPackMs.Mean(outputs.size()) << "/" << externalPackMs.max
              << "\n";
    std::cout << "  lk_disparity_ms_mean/max: " << lkDisparityMs.Mean(outputs.size()) << "/" << lkDisparityMs.max
              << "\n";
    std::cout << "  lk_gftt_ms_mean/max: " << lkGfttMs.Mean(outputs.size()) << "/" << lkGfttMs.max << "\n";
    std::cout << "  lk_flow_ms_mean/max: " << lkFlowMs.Mean(outputs.size()) << "/" << lkFlowMs.max << "\n";
    std::cout << "  lk_candidate_ms_mean/max: " << lkCandidateMs.Mean(outputs.size()) << "/" << lkCandidateMs.max
              << "\n";
    std::cout << "  lk_pnp_ms_mean/max: " << lkPnpMs.Mean(outputs.size()) << "/" << lkPnpMs.max << "\n";
    std::cout << "  orb_track_ms_mean/max: " << orbTrackMsMean << "/" << orbTrackMsMax << "\n";
    std::cout << "  orb_extract_ms_mean/max: " << orbExtractMsMean << "/" << orbExtractMsMax << "\n";
    std::cout << "  orb_stereo_ms_mean/max: " << orbStereoMsMean << "/" << orbStereoMsMax << "\n";
    std::cout << "  output_csv: " << opts.outputCsv << "\n";
    if (!opts.finalEurocTrajectory.empty()) {
        std::cout << "  final_euroc_trajectory: " << opts.finalEurocTrajectory << "\n";
    }

    if (!opts.summaryJson.empty()) {
        std::ofstream summary(opts.summaryJson);
        if (!summary) {
            std::cerr << "failed to open summary json: " << opts.summaryJson << "\n";
            return 1;
        }
        summary << "{\n"
                << "  \"dataset\": \"" << opts.datasetRoot.string() << "\",\n"
                << "  \"settings\": \"" << opts.settings << "\",\n"
                << "  \"vocab\": \"" << opts.vocab << "\",\n"
                << "  \"sensor_mode\": \"" << ToSensorModeText(opts.sensorMode) << "\",\n"
                << "  \"feature_frontend\": \"" << ToFeatureFrontendText(opts.featureFrontend) << "\",\n"
                << "  \"euroc_output_timestamp_offset_ms\": "
                << (static_cast<double>(eurocOutputTimestampOffsetNs) / 1000000.0) << ",\n"
                << "  \"euroc_output_position_scale\": " << eurocOutputPositionScale << ",\n"
                << "  \"orb_accel\": \"" << opts.orbAcceleration << "\",\n"
                << "  \"lk_per_frame_accel\": \"" << opts.lkPerFrameAcceleration << "\",\n"
                << "  \"frames_out\": " << outputs.size() << ",\n"
                << "  \"pose_valid_frames\": " << poseValidCount << ",\n"
                << "  \"tracking_ok_frames\": " << trackingOkCount << ",\n"
                << "  \"tracking_lost_frames\": " << trackingLostCount << ",\n"
                << "  \"identity_pose_frames\": " << identityPoseCount << ",\n"
                << "  \"replay_acquire_ms_mean\": " << replayAcquireMs.Mean(outputs.size()) << ",\n"
                << "  \"replay_acquire_ms_max\": " << replayAcquireMs.max << ",\n"
                << "  \"replay_imu_ms_mean\": " << replayImuMs.Mean(outputs.size()) << ",\n"
                << "  \"replay_imu_ms_max\": " << replayImuMs.max << ",\n"
                << "  \"slam_total_ms_mean\": " << slamTotalMs.Mean(outputs.size()) << ",\n"
                << "  \"slam_total_ms_max\": " << slamTotalMs.max << ",\n"
                << "  \"input_prepare_ms_mean\": " << inputPrepareMs.Mean(outputs.size()) << ",\n"
                << "  \"input_prepare_ms_max\": " << inputPrepareMs.max << ",\n"
                << "  \"frontend_ms_mean\": " << frontendMs.Mean(outputs.size()) << ",\n"
                << "  \"frontend_ms_max\": " << frontendMs.max << ",\n"
                << "  \"stereo_pair_ms_mean\": " << stereoPairMs.Mean(outputs.size()) << ",\n"
                << "  \"stereo_pair_ms_max\": " << stereoPairMs.max << ",\n"
                << "  \"external_pack_ms_mean\": " << externalPackMs.Mean(outputs.size()) << ",\n"
                << "  \"external_pack_ms_max\": " << externalPackMs.max << ",\n"
                << "  \"mono_augment_ms_mean\": " << monoAugmentMs.Mean(outputs.size()) << ",\n"
                << "  \"mono_augment_ms_max\": " << monoAugmentMs.max << ",\n"
                << "  \"lk_rectify_ms_mean\": " << lkRectifyMs.Mean(outputs.size()) << ",\n"
                << "  \"lk_rectify_ms_max\": " << lkRectifyMs.max << ",\n"
                << "  \"lk_disparity_ms_mean\": " << lkDisparityMs.Mean(outputs.size()) << ",\n"
                << "  \"lk_disparity_ms_max\": " << lkDisparityMs.max << ",\n"
                << "  \"lk_gftt_ms_mean\": " << lkGfttMs.Mean(outputs.size()) << ",\n"
                << "  \"lk_gftt_ms_max\": " << lkGfttMs.max << ",\n"
                << "  \"lk_flow_ms_mean\": " << lkFlowMs.Mean(outputs.size()) << ",\n"
                << "  \"lk_flow_ms_max\": " << lkFlowMs.max << ",\n"
                << "  \"lk_candidate_ms_mean\": " << lkCandidateMs.Mean(outputs.size()) << ",\n"
                << "  \"lk_candidate_ms_max\": " << lkCandidateMs.max << ",\n"
                << "  \"lk_pnp_ms_mean\": " << lkPnpMs.Mean(outputs.size()) << ",\n"
                << "  \"lk_pnp_ms_max\": " << lkPnpMs.max << ",\n"
                << "  \"lk_update_ms_mean\": " << lkUpdateMs.Mean(outputs.size()) << ",\n"
                << "  \"lk_update_ms_max\": " << lkUpdateMs.max << ",\n"
                << "  \"superpoint_frontend_ms_mean\": " << superpointFrontendMs.Mean(outputs.size()) << ",\n"
                << "  \"superpoint_frontend_ms_max\": " << superpointFrontendMs.max << ",\n"
                << "  \"superpoint_match_ms_mean\": " << superpointMatchMs.Mean(outputs.size()) << ",\n"
                << "  \"superpoint_match_ms_max\": " << superpointMatchMs.max << ",\n"
                << "  \"superpoint_total_ms_mean\": " << superpointTotalMs.Mean(outputs.size()) << ",\n"
                << "  \"superpoint_total_ms_max\": " << superpointTotalMs.max << ",\n"
                << "  \"orb_track_ms_mean\": " << orbTrackMsMean << ",\n"
                << "  \"orb_track_ms_max\": " << orbTrackMsMax << ",\n"
                << "  \"orb_extract_ms_mean\": " << orbExtractMsMean << ",\n"
                << "  \"orb_extract_ms_max\": " << orbExtractMsMax << ",\n"
                << "  \"orb_stereo_ms_mean\": " << orbStereoMsMean << ",\n"
                << "  \"orb_stereo_ms_max\": " << orbStereoMsMax << ",\n"
                << "  \"output_csv\": \"" << opts.outputCsv.string() << "\",\n"
                << "  \"final_euroc_trajectory\": \"" << opts.finalEurocTrajectory.string() << "\"\n"
                << "}\n";
        summary.flush();
        std::cout << "  summary_json: " << opts.summaryJson << "\n";
    }

    superpointFrontendClient.Stop();
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
