#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "System.h"
#include "adapters/slam/orbslam3_engine.h"
#include "core/application/config/app_args.h"
#include "core/domain/runtime_mode.h"
#include "test_support/replay_dataset.h"
#include "test_support/replay_slam_runner.h"

namespace fs = std::filesystem;

namespace {

struct OfflineReplayOptions {
    fs::path datasetRoot{fs::path(TESTS_SOURCE_DIR) / "data"};
    fs::path outputCsv{"build/offline_replay_pose.csv"};
    fs::path summaryJson{};
    std::string vocab{"ORB_SLAM3/Vocabulary/ORBvoc.txt"};
    std::string settings{"config/stereo.yaml"};
    SensorMode sensorMode{SensorMode::StereoImu};
    smartdrone::core::domain::SlamOperationMode slamMode{smartdrone::core::domain::SlamOperationMode::Mapping};
    int cameraFps{60};
    int slamInputFps{20};
    int timeoutMs{1000};
    size_t maxFrames{0};
};

const char *UsageText()
{
    return
        "Usage: smart_drone_offline_replay [options]\n"
        "  --dataset <dir>       Replay dataset root, default tests/data\n"
        "  --out <file>          Output CSV path, default build/offline_replay_pose.csv\n"
        "  --summary-json <file> Optional summary JSON output path\n"
        "  --vocab <file>        ORB vocabulary path\n"
        "  --settings <file>     ORB settings YAML path\n"
        "  --sensor-mode <mode>  stereo|stereo-imu|mono|mono-imu\n"
        "  --stereo-only         Shortcut for --sensor-mode stereo\n"
        "  --slam-mode <mode>    mapping|localization|relocalization|tracking-only|auto\n"
        "  --fps <n>             Camera FPS for replay pacing, default 60\n"
        "  --slam-fps <n>        SLAM input FPS, default 20\n"
        "  --timeout-ms <n>      Batch acquire timeout, default 1000\n"
        "  --max-frames <n>      Maximum output frames, default 0(all)\n";
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
    opts.slamMode = ParseSlamOperationModeText(GetOptionValue(argc, argv, "--slam-mode", "mapping"));
    opts.cameraFps = GetOptionInt(argc, argv, "--fps", opts.cameraFps);
    opts.slamInputFps = GetOptionInt(argc, argv, "--slam-fps", opts.slamInputFps);
    opts.timeoutMs = GetOptionInt(argc, argv, "--timeout-ms", opts.timeoutMs);
    opts.maxFrames = GetOptionSize(argc, argv, "--max-frames", opts.maxFrames);
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

smartdrone::adapters::slam::OrbInputMode ResolveOrbInputMode(SensorMode mode)
{
    return (mode == SensorMode::Mono || mode == SensorMode::MonoImu) ? smartdrone::adapters::slam::OrbInputMode::MonoRight
                                                                     : smartdrone::adapters::slam::OrbInputMode::Stereo;
}

bool UseImu(SensorMode mode) { return mode == SensorMode::StereoImu || mode == SensorMode::MonoImu; }

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
    auto orbSystem = std::make_unique<ORB_SLAM3::System>(opts.vocab, opts.settings, sensor, false);
    smartdrone::adapters::slam::OrbSlam3Engine slamEngine(std::move(orbSystem), ResolveOrbInputMode(opts.sensorMode),
                                                          UseImu(opts.sensorMode));
    slamEngine.SetOperationMode(opts.slamMode);

    smartdrone::tests::ReplaySlamRunner runner(camera, imu, slamEngine,
                                               {.cameraFps = opts.cameraFps,
                                                .slamInputFps = opts.slamInputFps,
                                                .useImu = UseImu(opts.sensorMode),
                                                .preferLatestFrame = true,
                                                .timeoutMs = opts.timeoutMs,
                                                .shutdownEngineOnFinish = false});
    smartdrone::core::application::FrameTimingTracker timingTracker(512);
    const auto outputs = runner.Run(opts.maxFrames, &timingTracker);

    if (!opts.outputCsv.parent_path().empty()) {
        fs::create_directories(opts.outputCsv.parent_path());
    }
    if (!opts.summaryJson.empty() && !opts.summaryJson.parent_path().empty()) {
        fs::create_directories(opts.summaryJson.parent_path());
    }
    std::ofstream csv(opts.outputCsv);
    if (!csv) {
        std::cerr << "failed to open output csv: " << opts.outputCsv << "\n";
        return 1;
    }

    csv << "frame_id,capture_timestamp_ns,tracking_state,map_id,pose_valid,x,y,z,qw,qx,qy,qz,imu_samples\n";
    size_t poseValidCount = 0;
    size_t trackingOkCount = 0;
    size_t trackingLostCount = 0;
    size_t identityPoseCount = 0;
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
        csv << sample.frameId << ',' << sample.captureTimestampNs << ',' << sample.trackingState << ','
            << sample.mapId << ',' << (sample.poseValid ? 1 : 0) << ',' << sample.pose.x << ',' << sample.pose.y
            << ',' << sample.pose.z << ',' << sample.pose.qw << ',' << sample.pose.qx << ',' << sample.pose.qy
            << ',' << sample.pose.qz << ',' << sample.imuSampleCount << '\n';
    }

    std::cout << "offline replay complete\n";
    std::cout << "  dataset: " << opts.datasetRoot << "\n";
    std::cout << "  settings: " << opts.settings << "\n";
    std::cout << "  vocab: " << opts.vocab << "\n";
    std::cout << "  sensor_mode: " << ToSensorModeText(opts.sensorMode) << "\n";
    std::cout << "  frames_out: " << outputs.size() << "\n";
    std::cout << "  pose_valid_frames: " << poseValidCount << "\n";
    std::cout << "  tracking_ok_frames: " << trackingOkCount << "\n";
    std::cout << "  tracking_lost_frames: " << trackingLostCount << "\n";
    std::cout << "  identity_pose_frames: " << identityPoseCount << "\n";
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
                << "  \"frames_out\": " << outputs.size() << ",\n"
                << "  \"pose_valid_frames\": " << poseValidCount << ",\n"
                << "  \"tracking_ok_frames\": " << trackingOkCount << ",\n"
                << "  \"tracking_lost_frames\": " << trackingLostCount << ",\n"
                << "  \"identity_pose_frames\": " << identityPoseCount << ",\n"
                << "  \"output_csv\": \"" << opts.outputCsv.string() << "\"\n"
                << "}\n";
        summary.flush();
        std::cout << "  summary_json: " << opts.summaryJson << "\n";
    }

    csv.flush();
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
