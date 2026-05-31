#include "core/application/runtime/runtime_config_application.h"
#include "core/application/runtime/runtime_config_message.h"
#include "core/application/runtime/runtime_config_projection.h"
#include "core/application/runtime/runtime_config_update_builder.h"
#include "core/application/runtime/runtime_config_validation.h"
#include "core/application/runtime/runtime_config_value_applier.h"

#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include "core/application/config/config_registry.h"

namespace {

using SmartDrone::Core::Application::ApplyConfigValue;
using SmartDrone::Core::Application::ApplyRemoteRuntimeConfig;
using SmartDrone::Core::Application::BuildRemoteConfig;
using SmartDrone::Core::Application::BuildRuntimeConfigMessage;
using SmartDrone::Core::Application::BuildRuntimeConfigUpdate;
using SmartDrone::Core::Application::ConfigRegistry;
using SmartDrone::Core::Application::ConfigValue;
using SmartDrone::Core::Application::LiveRuntimeTuning;
using SmartDrone::Core::Application::NormalizeRemoteRuntimeConfig;
using SmartDrone::Core::Application::RemoteRuntimeConfig;
using SmartDrone::Core::Application::SyncRuntimeTuning;
using SmartDrone::Core::Application::UnifiedConfig;
using SmartDrone::Core::Application::ValidateRemoteRuntimeConfig;

bool Contains(const std::string &text, const std::string &needle)
{
    return text.find(needle) != std::string::npos;
}

UnifiedConfig MakeBaseConfig()
{
    UnifiedConfig config{};
    config.app.camera.fps = 60;
    config.app.camera.exposureUs = 1234;
    config.app.camera.gain = 2.5f;
    config.app.camera.aeDisable = true;
    config.app.camera.pairMs = 0;
    config.app.camera.uvcDeviceIndex = 2;
    config.app.camera.uvcEyeWidth = 800;
    config.app.camera.uvcEyeHeight = 600;
    config.app.camera.uvcPackedStereo = false;
    config.app.sensorMode = SensorMode::Mono;
    config.app.udp.ip = "10.0.0.8";
    config.app.udp.enable = true;
    config.app.udp.sendImage = false;
    config.app.udp.sendFeature = true;
    config.app.udp.sendMap = true;

    auto &runtime = config.app.runtime;
    runtime.slamInputFps = 44;
    runtime.slamOperationMode =
        SmartDrone::Core::Domain::SlamOperationMode::Localization;
    runtime.slamBackend = SlamBackend::Klt;
    runtime.featureFrontend = FeatureFrontend::LK;
    runtime.useCustomTbc = true;
    runtime.tbcTx = 0.1f;
    runtime.tbcTy = 0.2f;
    runtime.tbcTz = 0.3f;
    runtime.orbNFeatures = 900;
    runtime.orbScaleFactor = 1.2f;
    runtime.orbNLevels = 8;
    runtime.orbIniThFAST = 20;
    runtime.orbMinThFAST = 7;
    runtime.visualFeatureTopK = 1000;
    runtime.visualFeatureMaxPoints = 400;
    runtime.lkPerFrameAcceleration = "auto";
    runtime.orbAcceleration = "cuda";
    runtime.avoidanceEnabled = false;
    runtime.avoidanceHoldOnStaleCloud = true;
    runtime.avoidanceRadiusM = 1.1f;
    runtime.avoidanceLookaheadM = 3.0f;
    runtime.avoidanceSpeedLookaheadS = 1.5f;
    runtime.avoidanceNearFieldRadiusM = 0.4f;
    runtime.avoidanceMaxPointCloudAgeMs = 800;
    runtime.avoidanceMinCloudPoints = 20;
    runtime.avoidanceMinBlockingPoints = 3;
    return config;
}

RemoteRuntimeConfig MakeValidRemote()
{
    RemoteRuntimeConfig remote = BuildRemoteConfig(MakeBaseConfig());
    remote.pairMs = 3;
    remote.visualFeatureInputMaxWidth = 640;
    remote.visualFeatureInputMaxHeight = 400;
    return remote;
}

TEST(RuntimeConfigModulesTest, ProjectsUnifiedConfigToRemoteConfig)
{
    const RemoteRuntimeConfig remote = BuildRemoteConfig(MakeBaseConfig());

    EXPECT_EQ(remote.exposureUs, 1234);
    EXPECT_FLOAT_EQ(remote.gain, 2.5f);
    EXPECT_FALSE(remote.autoExposureEnabled);
    EXPECT_EQ(remote.pairMs, 1);
    EXPECT_EQ(remote.uvcDeviceIndex, 2);
    EXPECT_EQ(remote.sensorMode, SensorMode::Mono);
    EXPECT_EQ(remote.udpIp, "10.0.0.8");
    EXPECT_TRUE(remote.udpEnabled);
    EXPECT_FALSE(remote.sendImage);
    EXPECT_EQ(remote.slamInputFps, 44);
    EXPECT_EQ(remote.slamBackend, SlamBackend::Klt);
    EXPECT_EQ(remote.featureFrontend, FeatureFrontend::LK);
    EXPECT_TRUE(remote.useCustomTbc);
    EXPECT_EQ(remote.orbNFeatures, 900);
    EXPECT_EQ(remote.lkPerFrameAcceleration, "auto");
    EXPECT_FALSE(remote.avoidanceEnabled);
    EXPECT_TRUE(remote.avoidanceHoldOnStaleCloud);
    EXPECT_FLOAT_EQ(remote.avoidanceRadiusM, 1.1f);
    EXPECT_EQ(remote.avoidanceMinCloudPoints, 20);
}

TEST(RuntimeConfigModulesTest, AppliesConfigValueUpdatesAndRejectsBadTypes)
{
    RemoteRuntimeConfig remote = MakeValidRemote();
    auto result = ApplyConfigValue(std::string(ConfigRegistry::CAMERA_GAIN),
                                   ConfigValue{4.25}, remote);
    ASSERT_TRUE(result.ok);
    EXPECT_FLOAT_EQ(remote.gain, 4.25f);

    result = ApplyConfigValue(std::string(ConfigRegistry::SLAM_BACKEND),
                              ConfigValue{std::string("dpvo_tensorrt")},
                              remote);
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(remote.slamBackend, SlamBackend::DpvoTensorRt);

    result = ApplyConfigValue(std::string(ConfigRegistry::SLAM_INPUT_FPS),
                              ConfigValue{int64_t{31}}, remote);
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(remote.slamInputFps, 31);

    result = ApplyConfigValue(std::string(ConfigRegistry::AVOIDANCE_RADIUS_M),
                              ConfigValue{1.25}, remote);
    ASSERT_TRUE(result.ok);
    EXPECT_FLOAT_EQ(remote.avoidanceRadiusM, 1.25f);

    result = ApplyConfigValue(
        std::string(ConfigRegistry::AVOIDANCE_HOLD_ON_STALE_CLOUD),
        ConfigValue{true}, remote);
    ASSERT_TRUE(result.ok);
    EXPECT_TRUE(remote.avoidanceHoldOnStaleCloud);

    result = ApplyConfigValue(std::string(ConfigRegistry::CAMERA_GAIN),
                              ConfigValue{true}, remote);
    EXPECT_FALSE(result.ok);
    EXPECT_TRUE(Contains(result.message, "type mismatch"));

    result = ApplyConfigValue("unsupported.key", ConfigValue{int64_t{1}}, remote);
    EXPECT_FALSE(result.ok);
    EXPECT_TRUE(Contains(result.message, "unsupported config key"));
}

TEST(RuntimeConfigModulesTest, BuildsRuntimeConfigUpdate)
{
    const RemoteRuntimeConfig remote = MakeValidRemote();
    const auto update = BuildRuntimeConfigUpdate(remote);

    const std::string exposureKey(ConfigRegistry::CAMERA_EXPOSURE_US);
    ASSERT_NE(update.values.find(exposureKey), update.values.end());
    EXPECT_EQ(std::get<int64_t>(update.values.at(
                  exposureKey)),
              remote.exposureUs);
    EXPECT_EQ(std::get<std::string>(update.values.at(
                  std::string(ConfigRegistry::SLAM_BACKEND))),
              "klt");
    EXPECT_EQ(std::get<bool>(update.values.at(
                  std::string(ConfigRegistry::STREAM_SEND_MAP))),
              remote.sendMap);
    EXPECT_EQ(std::get<double>(update.values.at(
                  std::string(ConfigRegistry::SLAM_TBC_TX))),
              static_cast<double>(remote.tbcTx));
    EXPECT_EQ(std::get<std::string>(update.values.at(
                  std::string(ConfigRegistry::SLAM_LK_PER_FRAME_ACCELERATION))),
              remote.lkPerFrameAcceleration);
    EXPECT_EQ(std::get<bool>(update.values.at(
                  std::string(ConfigRegistry::AVOIDANCE_ENABLED))),
              remote.avoidanceEnabled);
    EXPECT_EQ(std::get<double>(update.values.at(
                  std::string(ConfigRegistry::AVOIDANCE_RADIUS_M))),
              static_cast<double>(remote.avoidanceRadiusM));
}

TEST(RuntimeConfigModulesTest, NormalizesAndValidatesRemoteConfig)
{
    RemoteRuntimeConfig remote = MakeValidRemote();
    remote.slamBackend = SlamBackend::DpvoTensorRt;
    remote.featureFrontend = FeatureFrontend::Orb;
    remote.lkPerFrameAcceleration = "auto";
    remote.orbAcceleration = "CUDA";
    remote.orbNFeatures = 0;

    NormalizeRemoteRuntimeConfig(remote);
    EXPECT_EQ(remote.featureFrontend, FeatureFrontend::LkGfttPerFrame);
    EXPECT_EQ(remote.lkPerFrameAcceleration, "cpu");
    EXPECT_EQ(remote.orbAcceleration, "cpu");

    std::string err;
    EXPECT_TRUE(ValidateRemoteRuntimeConfig(remote, &err));
    EXPECT_EQ(remote.orbNFeatures, 1200);
}

TEST(RuntimeConfigModulesTest, RejectsInvalidVisualFeatureConfig)
{
    RemoteRuntimeConfig remote = MakeValidRemote();
    remote.visualFeatureTopK = 8;
    remote.visualFeatureMaxPoints = 9;

    std::string err;
    EXPECT_FALSE(ValidateRemoteRuntimeConfig(remote, &err));
    EXPECT_EQ(err, "visual feature config out of range");
}

TEST(RuntimeConfigModulesTest, RejectsInvalidAvoidanceConfig)
{
    RemoteRuntimeConfig remote = MakeValidRemote();
    remote.avoidanceRadiusM = 0.1f;

    std::string err;
    EXPECT_FALSE(ValidateRemoteRuntimeConfig(remote, &err));
    EXPECT_EQ(err, "avoidance distance config out of range");
}

TEST(RuntimeConfigModulesTest, AppliesRemoteConfigAndSyncsRuntimeTuning)
{
    UnifiedConfig config = MakeBaseConfig();
    RemoteRuntimeConfig remote = MakeValidRemote();
    remote.sensorMode = SensorMode::StereoImu;
    remote.uvcEyeWidth = 1024;
    remote.tbcTx = 1.0f;
    remote.tbcTy = 2.0f;
    remote.tbcTz = 3.0f;
    remote.avoidanceEnabled = true;
    remote.avoidanceRadiusM = 1.4f;
    remote.avoidanceMinCloudPoints = 24;

    const auto applied = ApplyRemoteRuntimeConfig(config, remote);
    EXPECT_TRUE(applied.restartNeeded);
    EXPECT_EQ(config.app.sensorMode, SensorMode::StereoImu);
    EXPECT_EQ(config.app.camera.uvcEyeWidth, 1024);
    EXPECT_FLOAT_EQ(config.app.runtime.tbcTx, 1.0f);
    EXPECT_TRUE(config.app.udp.enable);

    LiveRuntimeTuning tuning{};
    SyncRuntimeTuning(tuning, remote, applied.tbc);
    EXPECT_EQ(tuning.slamInputFps.load(), remote.slamInputFps);
    EXPECT_EQ(tuning.sendMap.load(), remote.sendMap);
    EXPECT_FLOAT_EQ(tuning.tbcTy.load(), 2.0f);
    EXPECT_TRUE(tuning.avoidanceEnabled.load());
    EXPECT_FLOAT_EQ(tuning.avoidanceRadiusM.load(), 1.4f);
    EXPECT_EQ(tuning.avoidanceMinCloudPoints.load(), 24);
}

TEST(RuntimeConfigModulesTest, BuildsRuntimeConfigMessage)
{
    UnifiedConfig currentConfig = MakeBaseConfig();
    RemoteRuntimeConfig remote = MakeValidRemote();
    remote.slamInputFps = 120;
    remote.slamOperationMode =
        SmartDrone::Core::Domain::SlamOperationMode::Relocalization;

    const std::string message = BuildRuntimeConfigMessage(remote, currentConfig);
    EXPECT_TRUE(Contains(message, "sensor=mono"));
    EXPECT_TRUE(Contains(message, "backend=klt"));
    EXPECT_TRUE(Contains(message, "slam_mode=relocalization"));
    EXPECT_TRUE(Contains(message, "slam_fps=60"));
    EXPECT_TRUE(Contains(message, "orb_accel=cuda"));
    EXPECT_TRUE(Contains(message, "avoid="));
}

} // namespace
