#include "adapters/camera/gazebo_stereo_config.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <string>
#include <utility>

#include <opencv2/core/persistence.hpp>

#include "common/environment.h"

namespace SmartDrone::Adapters::Camera {
namespace {

std::string ReadString(const cv::FileStorage &storage, const char *key)
{
    std::string value;
    storage[key] >> value;
    return value;
}

std::int64_t ReadInt64(const cv::FileStorage &storage, const char *key,
                       std::int64_t fallback)
{
    const cv::FileNode node = storage[key];
    if (node.empty()) {
        return fallback;
    }
    return static_cast<std::int64_t>(static_cast<double>(node));
}

std::int64_t ReadInt64(const cv::FileNode &parent, const char *key,
                       std::int64_t fallback)
{
    const cv::FileNode node = parent[key];
    if (node.empty()) {
        return fallback;
    }
    return static_cast<std::int64_t>(static_cast<double>(node));
}

double ReadDouble(const cv::FileNode &parent, const char *key,
                  double fallback)
{
    const cv::FileNode node = parent[key];
    if (node.empty()) {
        return fallback;
    }
    const double value = static_cast<double>(node);
    return std::isfinite(value) ? value : fallback;
}

std::string ResolveCalibrationPath(const std::string &configPath,
                                   const std::string &calibrationPath)
{
    const std::filesystem::path calibration(calibrationPath);
    if (calibration.is_absolute()) {
        return calibration.lexically_normal().string();
    }
    const std::filesystem::path parent =
        std::filesystem::path(configPath).parent_path();
    return (parent / calibration).lexically_normal().string();
}

bool LoadCalibrationMetadata(GazeboStereoConfig &config, std::string &error)
{
    cv::FileStorage storage(config.calibrationPath, cv::FileStorage::READ);
    if (!storage.isOpened()) {
        error = "unable to open simulation calibration";
        return false;
    }
    config.cameraWidth = static_cast<int>(ReadInt64(
        storage, "Camera.width", 0));
    config.cameraHeight = static_cast<int>(ReadInt64(
        storage, "Camera.height", 0));
    config.cameraFps = static_cast<int>(ReadInt64(storage, "Camera.fps", 0));
    if (config.cameraWidth <= 0 || config.cameraHeight <= 0 ||
        config.cameraFps <= 0) {
        error = "simulation calibration has invalid image dimensions or fps";
        return false;
    }
    return true;
}

bool ValidateConfig(GazeboStereoConfig &config, std::string &error)
{
    if (config.leftImageTopic.empty() || config.rightImageTopic.empty() ||
        config.clockTopic.empty()) {
        error = "left_image_topic, right_image_topic and clock_topic are required";
        return false;
    }
    if (config.calibrationPath.empty() ||
        !std::filesystem::is_regular_file(config.calibrationPath)) {
        error = "calibration_path does not name a readable file";
        return false;
    }
    return LoadCalibrationMetadata(config, error);
}

void ApplySimulationEnvironmentOverrides(GazeboStereoConfig &config)
{
    const std::string worldName = SmartDrone::Common::EnvStringValue(
        "SMART_DRONE_SIM_WORLD", "");
    if (!worldName.empty()) {
        const std::string worldPrefix = "/world/" + worldName;
        config.clockTopic = worldPrefix + "/clock";
        config.truthPoseTopic = worldPrefix + "/dynamic_pose/info";
    }

    const std::string modelName = SmartDrone::Common::EnvStringValue(
        "SMART_DRONE_SIM_MODEL", "");
    if (!modelName.empty()) {
        config.truthModelName = modelName;
    }
}

GazeboImageFaultConfig ReadFaultValues(
    const cv::FileNode &node, GazeboImageFaultConfig config)
{
    config.blurSigma = std::clamp(
        ReadDouble(node, "blur_sigma", config.blurSigma), 0.0, 20.0);
    config.brightness = std::clamp(
        ReadDouble(node, "brightness", config.brightness), 0.0, 4.0);
    config.noiseStddev = std::clamp(
        ReadDouble(node, "noise_stddev", config.noiseStddev), 0.0, 128.0);
    config.dropRate = std::clamp(
        ReadDouble(node, "drop_rate", config.dropRate), 0.0, 1.0);
    config.delayMs = static_cast<int>(std::clamp<std::int64_t>(
        ReadInt64(node, "delay_ms", config.delayMs), 0, 5000));
    config.blackoutMs = static_cast<int>(std::clamp<std::int64_t>(
        ReadInt64(node, "blackout_ms", config.blackoutMs), 0, 60000));
    return config;
}

GazeboImageFaultConfig ReadFaultDefaults(const cv::FileStorage &storage)
{
    const cv::FileNode node = storage["faults"];
    return node.empty() ? GazeboImageFaultConfig{}
                        : ReadFaultValues(node, {});
}

GazeboImageFaultConfig ReadFaultState(const cv::FileStorage &storage)
{
    const cv::FileNode root = storage.root();
    GazeboImageFaultConfig config = ReadFaultValues(root, {});
    config.generation = static_cast<std::uint64_t>(std::max<std::int64_t>(
        0, ReadInt64(storage, "generation", 0)));
    config.action = ReadString(storage, "action");
    return config;
}

bool ValidFaultState(const cv::FileStorage &storage,
                     const GazeboImageFaultConfig &config,
                     std::uint64_t currentGeneration)
{
    const std::string schema = ReadString(storage, "schema");
    const bool actionValid = config.action == "apply" ||
                             config.action == "clear" ||
                             config.action == "blackout";
    return schema == "smartdrone.sitl.image_fault.v1" && actionValid &&
           config.generation > currentGeneration;
}

} // namespace

GazeboImageFaultConfig LoadGazeboImageFaultConfigFromEnvironment(
    GazeboImageFaultConfig defaults)
{
    GazeboImageFaultConfig config = std::move(defaults);
    config.action = SmartDrone::Common::EnvStringValue(
        "SMART_DRONE_FAULT_ACTION", config.action.c_str());
    if (!SmartDrone::Common::EnvVarIsUnsetOrEmpty(
            "SMART_DRONE_FAULT_BLUR_SIGMA")) {
        config.blurSigma = std::clamp(SmartDrone::Common::EnvDoubleValue(
                                          "SMART_DRONE_FAULT_BLUR_SIGMA",
                                          config.blurSigma),
                                      0.0, 20.0);
        config.blurKernel = 0;
    } else if (!SmartDrone::Common::EnvVarIsUnsetOrEmpty(
                   "SMART_DRONE_FAULT_BLUR")) {
        config.blurKernel = SmartDrone::Common::EnvIntValueClamped(
            "SMART_DRONE_FAULT_BLUR", config.blurKernel, 0, 31);
        config.blurSigma = 0.0;
    }
    config.brightness = std::clamp(SmartDrone::Common::EnvDoubleValue(
                                       "SMART_DRONE_FAULT_BRIGHTNESS",
                                       config.brightness),
                                   0.0, 4.0);
    config.noiseStddev = std::clamp(SmartDrone::Common::EnvDoubleValue(
                                        "SMART_DRONE_FAULT_NOISE",
                                        config.noiseStddev),
                                    0.0, 128.0);
    config.dropRate = std::clamp(
        SmartDrone::Common::EnvDoubleValue(
            "SMART_DRONE_FAULT_DROP", config.dropRate * 100.0) /
            100.0,
        0.0, 1.0);
    config.delayMs = SmartDrone::Common::EnvIntValueClamped(
        "SMART_DRONE_FAULT_DELAY", config.delayMs, 0, 5000);
    config.blackoutMs = SmartDrone::Common::EnvIntValueClamped(
        "SMART_DRONE_FAULT_BLACKOUT_MS", config.blackoutMs, 0, 60000);
    return config;
}

bool TryLoadGazeboImageFaultState(const std::string &statePath,
                                  std::uint64_t currentGeneration,
                                  GazeboImageFaultConfig &out)
{
    if (statePath.empty() || !std::filesystem::is_regular_file(statePath)) {
        return false;
    }
    try {
        cv::FileStorage storage(statePath, cv::FileStorage::READ);
        if (!storage.isOpened()) {
            return false;
        }
        GazeboImageFaultConfig candidate = ReadFaultState(storage);
        if (!ValidFaultState(storage, candidate, currentGeneration)) {
            return false;
        }
        if (candidate.action == "clear") {
            const std::uint64_t generation = candidate.generation;
            candidate = {};
            candidate.generation = generation;
            candidate.action = "clear";
        }
        out = std::move(candidate);
        return true;
    } catch (const std::exception &) {
        return false;
    }
}

GazeboStereoConfigLoadResult LoadGazeboStereoConfig(
    const std::string &configPath)
{
    GazeboStereoConfigLoadResult result;
    cv::FileStorage storage(configPath, cv::FileStorage::READ);
    if (!storage.isOpened()) {
        result.error = "unable to open SMART_DRONE_SIM_CONFIG";
        return result;
    }
    result.config.leftImageTopic = ReadString(storage, "left_image_topic");
    result.config.rightImageTopic = ReadString(storage, "right_image_topic");
    result.config.clockTopic = ReadString(storage, "clock_topic");
    result.config.truthPoseTopic = ReadString(storage, "truth_pose_topic");
    result.config.truthModelName = ReadString(storage, "truth_model_name");
    result.config.calibrationPath = ResolveCalibrationPath(
        configPath, ReadString(storage, "calibration_path"));
    result.config.faultStatePath = SmartDrone::Common::EnvStringValue(
        "SMART_DRONE_SIM_FAULT_FILE", "");
    result.config.pairToleranceNs = std::clamp<std::int64_t>(
        ReadInt64(storage, "pair_tolerance_ns", 5000000), 0, 100000000);
    result.config.queueDepth = static_cast<std::size_t>(std::clamp<std::int64_t>(
        ReadInt64(storage, "queue_depth", 8), 2, 256));
    result.config.fault = LoadGazeboImageFaultConfigFromEnvironment(
        ReadFaultDefaults(storage));
    ApplySimulationEnvironmentOverrides(result.config);
    result.ok = ValidateConfig(result.config, result.error);
    return result;
}

} // namespace SmartDrone::Adapters::Camera
