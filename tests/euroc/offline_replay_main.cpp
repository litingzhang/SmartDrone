#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>

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
    SetEnvIfUnset("SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_FEATURES", "200");
    SetEnvIfUnset("SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_CLOSE_RATIO", "0.48");
    SetEnvIfUnset("SMART_DRONE_EXTERNAL_STEREO_DEPTH_SCALE", "0.985");
}

struct LoopClosureCorrectionSummary {
    bool enabled{false};
    size_t anchors{0};
    size_t visualLoopEdges{0};
    float pathLengthMeters{0.0f};
    float terminalDriftMeters{0.0f};
    float scale{1.0f};
    float relaxation{1.0f};
};

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

bool ConvertOrbEuRoCTrajectoryToReplayCsv(const fs::path &trajectoryPath, const fs::path &csvPath)
{
    std::ifstream input(trajectoryPath);
    if (!input) {
        std::cerr << "failed to open ORB EuRoC trajectory: " << trajectoryPath << "\n";
        return false;
    }

    std::ofstream csv(csvPath);
    if (!csv) {
        std::cerr << "failed to open output csv: " << csvPath << "\n";
        return false;
    }

    csv << "frame_id,capture_timestamp_ns,tracking_state,map_id,pose_valid,x,y,z,qw,qx,qy,qz,imu_samples,"
           "superpoint_used,superpoint_raw_left,superpoint_raw_right,superpoint_stereo,superpoint_injected_left,superpoint_injected_right,"
           "superpoint_lg_every_n,superpoint_frontend_ms,superpoint_match_ms,superpoint_total_ms,replay_acquire_ms,replay_imu_ms,slam_total_ms,"
           "input_prepare_ms,frontend_ms,stereo_pair_ms,external_pack_ms,mono_augment_ms,"
           "lk_rectify_ms,lk_disparity_ms,lk_gftt_ms,lk_flow_ms,lk_candidate_ms,lk_pnp_ms,lk_update_ms,"
           "orb_track_ms,orb_extract_ms,orb_stereo_ms,"
           "inliers,tracked_map,local_map\n";

    size_t frameId = 0;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::stringstream ss(line);
        double timestampNs = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double qx = 0.0;
        double qy = 0.0;
        double qz = 0.0;
        double qw = 1.0;
        if (!(ss >> timestampNs >> x >> y >> z >> qx >> qy >> qz >> qw)) {
            continue;
        }
        csv << frameId++ << ',' << static_cast<uint64_t>(std::llround(timestampNs)) << ",2,0,1," << x << ',' << y
            << ',' << z << ',' << qw << ',' << qx << ',' << qy << ',' << qz;
        for (int i = 0; i < 31; ++i) {
            csv << ",0";
        }
        csv << '\n';
    }

    return frameId > 0;
}

struct Vec3f {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

struct VisualLoopEdge {
    size_t from{0};
    size_t to{0};
    int matches{0};
};

const char *UsageText()
{
    return
        "Usage: smart_drone_offline_replay [options]\n"
        "  --dataset <dir>       Replay dataset root, default tests/data; accepts tests/data or EuRoC mav0 layout\n"
        "  --out <file>          Output CSV path, default build/offline_replay_pose.csv\n"
        "  --summary-json <file> Optional summary JSON output path\n"
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
        "  --lk-loop-closure     Apply offline LK pose-graph loop-closure smoothing\n"
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

Vec3f PosePosition(const smartdrone::tests::ReplayPoseSample &sample)
{
    return {sample.pose.x, sample.pose.y, sample.pose.z};
}

float Distance(const Vec3f &lhs, const Vec3f &rhs)
{
    const float dx = lhs.x - rhs.x;
    const float dy = lhs.y - rhs.y;
    const float dz = lhs.z - rhs.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

float PathLength(const std::vector<smartdrone::tests::ReplayPoseSample> &samples)
{
    float length = 0.0f;
    for (size_t i = 1; i < samples.size(); ++i) {
        if (!samples[i - 1].poseValid || !samples[i].poseValid) {
            continue;
        }
        length += Distance(PosePosition(samples[i - 1]), PosePosition(samples[i]));
    }
    return length;
}

std::vector<size_t> DetectLoopClosureAnchors(const std::vector<smartdrone::tests::ReplayPoseSample> &samples)
{
    std::vector<size_t> anchors;
    if (samples.size() < 200 || !samples.front().poseValid || !samples.back().poseValid) {
        return anchors;
    }

    anchors.push_back(0);
    const Vec3f origin = PosePosition(samples.front());
    const size_t firstSearch = samples.size() / 6;
    const size_t lastSearch = static_cast<size_t>(static_cast<double>(samples.size()) * 0.80);
    constexpr float kReturnRadiusMeters = 0.50f;
    constexpr size_t kClusterGapFrames = 50;
    constexpr size_t kMinAnchorGapFrames = 160;

    size_t clusterBestIndex = 0;
    float clusterBestDistance = std::numeric_limits<float>::infinity();
    size_t clusterLastIndex = 0;
    bool inCluster = false;
    auto flushCluster = [&]() {
        if (!inCluster) {
            return;
        }
        if (clusterBestIndex > 0 && clusterBestIndex - anchors.back() >= kMinAnchorGapFrames) {
            anchors.push_back(clusterBestIndex);
        }
        inCluster = false;
        clusterBestIndex = 0;
        clusterBestDistance = std::numeric_limits<float>::infinity();
        clusterLastIndex = 0;
    };

    for (size_t i = firstSearch; i < std::min(lastSearch, samples.size()); ++i) {
        if (!samples[i].poseValid) {
            continue;
        }
        const float distance = Distance(PosePosition(samples[i]), origin);
        if (distance > kReturnRadiusMeters) {
            if (inCluster && i > clusterLastIndex + kClusterGapFrames) {
                flushCluster();
            }
            continue;
        }
        if (!inCluster) {
            inCluster = true;
            clusterBestIndex = i;
            clusterBestDistance = distance;
        } else if (distance < clusterBestDistance) {
            clusterBestIndex = i;
            clusterBestDistance = distance;
        }
        clusterLastIndex = i;
    }
    flushCluster();

    if (samples.size() - 1 - anchors.back() >= kMinAnchorGapFrames) {
        anchors.push_back(samples.size() - 1);
    }
    return anchors;
}

std::vector<smartdrone::tests::ReplayPoseSample> ApplyLkLoopClosureCorrection(
    const std::vector<smartdrone::tests::ReplayPoseSample> &samples, const smartdrone::tests::ReplayDataset &dataset,
    float scale, float relaxation, LoopClosureCorrectionSummary &summary)
{
    summary = LoopClosureCorrectionSummary{};
    summary.enabled = true;
    summary.relaxation = relaxation;
    if (samples.size() < 2) {
        return samples;
    }

    std::vector<VisualLoopEdge> loopEdges;
    constexpr size_t kKeyframeStride = 30;
    constexpr size_t kMinLoopAgeFrames = 300;
    constexpr int kMinLoopMatches = 400;
    constexpr int kStrongLoopMatches = 450;
    constexpr float kWeakLoopMaxResidualMeters = 4.0f;
    cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);
    cv::BFMatcher matcher(cv::NORM_HAMMING, true);
    struct VisualKeyframe {
        size_t sampleIndex{0};
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
    };
    std::vector<VisualKeyframe> visualKeyframes;
    const auto &leftFrames = dataset.LeftFrames();
    for (size_t i = 0; i < samples.size() && i < leftFrames.size(); i += kKeyframeStride) {
        cv::Mat gray = cv::imread(leftFrames[i].path.string(), cv::IMREAD_GRAYSCALE);
        if (gray.empty()) {
            continue;
        }
        VisualKeyframe current;
        current.sampleIndex = i;
        orb->detectAndCompute(gray, cv::noArray(), current.keypoints, current.descriptors);
        if (current.descriptors.empty()) {
            continue;
        }
        VisualLoopEdge bestEdge{};
        for (const VisualKeyframe &prior : visualKeyframes) {
            if (current.sampleIndex <= prior.sampleIndex + kMinLoopAgeFrames || prior.descriptors.empty()) {
                continue;
            }
            std::vector<cv::DMatch> matches;
            matcher.match(current.descriptors, prior.descriptors, matches);
            int goodMatches = 0;
            for (const cv::DMatch &match : matches) {
                if (match.distance < 45.0f) {
                    ++goodMatches;
                }
            }
            if (goodMatches > bestEdge.matches) {
                bestEdge = VisualLoopEdge{prior.sampleIndex, current.sampleIndex, goodMatches};
            }
        }
        const bool strongVisualLoop = bestEdge.matches >= kStrongLoopMatches;
        const bool geometricallyPlausibleLoop =
            bestEdge.matches >= kMinLoopMatches && samples[bestEdge.from].poseValid && samples[bestEdge.to].poseValid &&
            Distance(PosePosition(samples[bestEdge.from]), PosePosition(samples[bestEdge.to])) * scale <=
                kWeakLoopMaxResidualMeters;
        if ((strongVisualLoop || geometricallyPlausibleLoop) &&
            (loopEdges.empty() || bestEdge.to >= loopEdges.back().to + kMinLoopAgeFrames / 2)) {
            loopEdges.push_back(bestEdge);
        }
        visualKeyframes.push_back(std::move(current));
    }

    std::vector<size_t> anchors;
    anchors.push_back(0);
    for (const VisualLoopEdge &edge : loopEdges) {
        anchors.push_back(edge.to);
    }
    summary.anchors = anchors.size();
    summary.visualLoopEdges = loopEdges.size();
    const float effectiveScale = loopEdges.empty() ? 1.0f : scale;
    summary.scale = effectiveScale;
    summary.pathLengthMeters = PathLength(samples);
    summary.terminalDriftMeters =
        samples.front().poseValid && samples.back().poseValid ? Distance(PosePosition(samples.front()), PosePosition(samples.back()))
                                                              : 0.0f;
    if (summary.pathLengthMeters < 20.0f) {
        return samples;
    }

    std::vector<smartdrone::tests::ReplayPoseSample> corrected = samples;
    for (auto &sample : corrected) {
        if (!sample.poseValid) {
            continue;
        }
        sample.pose.x *= effectiveScale;
        sample.pose.y *= effectiveScale;
        sample.pose.z *= effectiveScale;
    }
    for (const VisualLoopEdge &edge : loopEdges) {
        const size_t start = edge.from;
        const size_t end = edge.to;
        if (end <= start || !samples[start].poseValid || !samples[end].poseValid) {
            continue;
        }
        const Vec3f startPos = PosePosition(corrected[start]);
        const Vec3f endPos = PosePosition(corrected[end]);
        const Vec3f drift{relaxation * (endPos.x - startPos.x), relaxation * (endPos.y - startPos.y),
                          relaxation * (endPos.z - startPos.z)};
        const float denom = static_cast<float>(std::max<size_t>(1, end - start));
        for (size_t i = start; i < corrected.size(); ++i) {
            if (!corrected[i].poseValid) {
                continue;
            }
            const float alpha = std::min(1.0f, static_cast<float>(i - start) / denom);
            corrected[i].pose.x -= alpha * drift.x;
            corrected[i].pose.y -= alpha * drift.y;
            corrected[i].pose.z -= alpha * drift.z;
        }
    }
    return corrected;
}

int RunOfflineReplay(const OfflineReplayOptions &opts)
{
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

    smartdrone::tests::ReplaySlamRunner runner(camera, imu, slamEngine,
                                               {.cameraFps = opts.cameraFps,
                                                .slamInputFps = opts.slamInputFps,
                                                .useImu = UseImu(opts.sensorMode),
                                                .preferLatestFrame = true,
                                                .timeoutMs = opts.timeoutMs,
                                                .shutdownEngineOnFinish = false});
    smartdrone::core::application::FrameTimingTracker timingTracker(512);
    auto outputs = runner.Run(opts.maxFrames, &timingTracker);
    LoopClosureCorrectionSummary loopClosureSummary{};
    if (opts.lkLoopClosure && opts.featureFrontend == FeatureFrontend::LK) {
        outputs = ApplyLkLoopClosureCorrection(outputs, dataset, opts.lkLoopScale, opts.lkLoopRelaxation,
                                               loopClosureSummary);
    }

    if (!opts.outputCsv.parent_path().empty()) {
        fs::create_directories(opts.outputCsv.parent_path());
    }
    if (!opts.summaryJson.empty() && !opts.summaryJson.parent_path().empty()) {
        fs::create_directories(opts.summaryJson.parent_path());
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

    const bool useOrbSlamFinalEuRoC =
        (opts.featureFrontend == FeatureFrontend::Orb || opts.featureFrontend == FeatureFrontend::SuperPointLightGlue) &&
        !UseImu(opts.sensorMode) && opts.sensorMode == SensorMode::Stereo;
    fs::path orbEuRoCTrajectoryPath;
    if (useOrbSlamFinalEuRoC) {
        const fs::path liveOutputCsv = opts.outputCsv.string() + ".live.csv";
        std::ofstream liveCsv(liveOutputCsv);
        if (liveCsv) {
            liveCsv << "frame_id,capture_timestamp_ns,tracking_state,map_id,pose_valid,x,y,z,qw,qx,qy,qz,imu_samples,"
                       "superpoint_used,superpoint_raw_left,superpoint_raw_right,superpoint_stereo,superpoint_injected_left,superpoint_injected_right,"
                       "superpoint_lg_every_n,superpoint_frontend_ms,superpoint_match_ms,superpoint_total_ms,replay_acquire_ms,replay_imu_ms,slam_total_ms,"
                       "input_prepare_ms,frontend_ms,stereo_pair_ms,external_pack_ms,mono_augment_ms,"
                       "lk_rectify_ms,lk_disparity_ms,lk_gftt_ms,lk_flow_ms,lk_candidate_ms,lk_pnp_ms,lk_update_ms,"
                       "orb_track_ms,orb_extract_ms,orb_stereo_ms,"
                       "inliers,tracked_map,local_map\n";
            for (const auto &sample : outputs) {
                liveCsv << sample.frameId << ',' << sample.captureTimestampNs << ',' << sample.trackingState << ','
                        << sample.mapId << ',' << (sample.poseValid ? 1 : 0) << ',' << sample.pose.x << ','
                        << sample.pose.y << ',' << sample.pose.z << ',' << sample.pose.qw << ',' << sample.pose.qx
                        << ',' << sample.pose.qy << ',' << sample.pose.qz << ',' << sample.imuSampleCount << ','
                        << (sample.usedSuperPointFrontend ? 1 : 0) << ',' << sample.superpointRawLeftCount << ','
                        << sample.superpointRawRightCount << ',' << sample.superpointMatchedStereoCount << ','
                        << sample.superpointInjectedLeftCount << ',' << sample.superpointInjectedRightCount << ','
                        << sample.superpointLightGlueEveryN << ',' << sample.superpointFrontendMs << ','
                        << sample.superpointStereoMatchMs << ',' << sample.superpointTotalMs << ','
                        << sample.replayAcquireMs << ',' << sample.replayImuMs << ',' << sample.slamTotalMs << ','
                        << sample.inputPrepareMs << ',' << sample.frontendMs << ',' << sample.stereoPairMs << ','
                        << sample.externalPackMs << ',' << sample.monoAugmentMs << ',' << sample.lkRectifyMs << ','
                        << sample.lkDisparityMs << ',' << sample.lkGfttMs << ',' << sample.lkFlowMs << ','
                        << sample.lkCandidateMs << ',' << sample.lkPnpMs << ',' << sample.lkUpdateMs << ','
                        << sample.orbTrackMs << ',' << sample.orbExtractMs << ',' << sample.orbStereoMatchMs << ','
                        << sample.matchesInliers << ',' << sample.trackedMapPointCount << ','
                        << sample.localMapPointCount << '\n';
            }
            liveCsv.flush();
        }
        orbEuRoCTrajectoryPath = opts.outputCsv;
        orbEuRoCTrajectoryPath += ".orb_euroc.txt";
        if (!slamEngine.ShutdownAndSaveTrajectoryEuRoC(orbEuRoCTrajectoryPath.string()) ||
            !ConvertOrbEuRoCTrajectoryToReplayCsv(orbEuRoCTrajectoryPath, opts.outputCsv)) {
            std::cerr << "failed to export ORB-SLAM final EuRoC trajectory\n";
            return 1;
        }
    } else {
        std::ofstream csv(opts.outputCsv);
        if (!csv) {
            std::cerr << "failed to open output csv: " << opts.outputCsv << "\n";
            return 1;
        }

        csv << "frame_id,capture_timestamp_ns,tracking_state,map_id,pose_valid,x,y,z,qw,qx,qy,qz,imu_samples,"
               "superpoint_used,superpoint_raw_left,superpoint_raw_right,superpoint_stereo,superpoint_injected_left,superpoint_injected_right,"
               "superpoint_lg_every_n,superpoint_frontend_ms,superpoint_match_ms,superpoint_total_ms,replay_acquire_ms,replay_imu_ms,slam_total_ms,"
               "input_prepare_ms,frontend_ms,stereo_pair_ms,external_pack_ms,mono_augment_ms,"
               "lk_rectify_ms,lk_disparity_ms,lk_gftt_ms,lk_flow_ms,lk_candidate_ms,lk_pnp_ms,lk_update_ms,"
               "orb_track_ms,orb_extract_ms,orb_stereo_ms,"
               "inliers,tracked_map,local_map\n";
        for (const auto &sample : outputs) {
            csv << sample.frameId << ',' << sample.captureTimestampNs << ',' << sample.trackingState << ','
                << sample.mapId << ',' << (sample.poseValid ? 1 : 0) << ',' << sample.pose.x << ',' << sample.pose.y
                << ',' << sample.pose.z << ',' << sample.pose.qw << ',' << sample.pose.qx << ',' << sample.pose.qy
                << ',' << sample.pose.qz << ',' << sample.imuSampleCount << ',' << (sample.usedSuperPointFrontend ? 1 : 0)
                << ',' << sample.superpointRawLeftCount << ',' << sample.superpointRawRightCount << ','
                << sample.superpointMatchedStereoCount << ',' << sample.superpointInjectedLeftCount << ','
                << sample.superpointInjectedRightCount << ',' << sample.superpointLightGlueEveryN << ','
                << sample.superpointFrontendMs << ','
                << sample.superpointStereoMatchMs << ',' << sample.superpointTotalMs << ',' << sample.replayAcquireMs << ','
                << sample.replayImuMs << ',' << sample.slamTotalMs << ',' << sample.inputPrepareMs << ','
                << sample.frontendMs << ',' << sample.stereoPairMs << ',' << sample.externalPackMs << ','
                << sample.monoAugmentMs << ',' << sample.lkRectifyMs << ',' << sample.lkDisparityMs << ','
                << sample.lkGfttMs << ',' << sample.lkFlowMs << ',' << sample.lkCandidateMs << ','
                << sample.lkPnpMs << ',' << sample.lkUpdateMs << ',' << sample.orbTrackMs << ','
                << sample.orbExtractMs << ',' << sample.orbStereoMatchMs << ',' << sample.matchesInliers << ','
                << sample.trackedMapPointCount << ',' << sample.localMapPointCount << '\n';
        }
        csv.flush();
    }

    std::cout << "offline replay complete\n";
    std::cout << "  dataset: " << opts.datasetRoot << "\n";
    std::cout << "  settings: " << opts.settings << "\n";
    std::cout << "  vocab: " << opts.vocab << "\n";
    std::cout << "  sensor_mode: " << ToSensorModeText(opts.sensorMode) << "\n";
    std::cout << "  feature_frontend: " << ToFeatureFrontendText(opts.featureFrontend) << "\n";
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
    if (!orbEuRoCTrajectoryPath.empty()) {
        std::cout << "  orb_euroc_trajectory: " << orbEuRoCTrajectoryPath << "\n";
    }
    if (loopClosureSummary.enabled) {
        std::cout << "  lk_loop_closure_anchors: " << loopClosureSummary.anchors << "\n";
        std::cout << "  lk_loop_closure_visual_edges: " << loopClosureSummary.visualLoopEdges << "\n";
        std::cout << "  lk_loop_closure_path_m: " << loopClosureSummary.pathLengthMeters << "\n";
        std::cout << "  lk_loop_closure_terminal_drift_m: " << loopClosureSummary.terminalDriftMeters << "\n";
        std::cout << "  lk_loop_closure_scale: " << loopClosureSummary.scale << "\n";
        std::cout << "  lk_loop_closure_relaxation: " << loopClosureSummary.relaxation << "\n";
    }
    std::cout << "  output_csv: " << opts.outputCsv << "\n";

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
                << "  \"lk_loop_closure_enabled\": " << (loopClosureSummary.enabled ? "true" : "false") << ",\n"
                << "  \"lk_loop_closure_anchors\": " << loopClosureSummary.anchors << ",\n"
                << "  \"lk_loop_closure_visual_edges\": " << loopClosureSummary.visualLoopEdges << ",\n"
                << "  \"lk_loop_closure_path_m\": " << loopClosureSummary.pathLengthMeters << ",\n"
                << "  \"lk_loop_closure_terminal_drift_m\": " << loopClosureSummary.terminalDriftMeters << ",\n"
                << "  \"lk_loop_closure_scale\": " << loopClosureSummary.scale << ",\n"
                << "  \"lk_loop_closure_relaxation\": " << loopClosureSummary.relaxation << ",\n"
                << "  \"orb_euroc_trajectory\": \"" << orbEuRoCTrajectoryPath.string() << "\",\n"
                << "  \"output_csv\": \"" << opts.outputCsv.string() << "\"\n"
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
