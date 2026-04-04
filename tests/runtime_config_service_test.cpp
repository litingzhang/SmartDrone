#include <gtest/gtest.h>

#include <filesystem>
#include <mutex>

#include "core/application/config/config_registry.h"
#include "core/application/runtime/runtime_config_service.h"

namespace smartdrone::core::application {
namespace {

UnifiedConfig MakeConfig()
{
    UnifiedConfig cfg{};
    cfg.app.camera.fps = 60;
    cfg.app.camera.exposureUs = 5000;
    cfg.app.camera.gain = 2.0f;
    cfg.app.camera.pairMs = 2;
    cfg.app.runtime.slamInputFps = 20;
    cfg.app.runtime.slamOperationMode = smartdrone::core::domain::SlamOperationMode::Mapping;
    cfg.app.sensorMode = SensorMode::Stereo;
    cfg.app.settings = "config/stereo.yaml";
    cfg.app.udp.enable = false;
    cfg.app.udp.ip = "10.0.0.1";
    cfg.app.udp.sendImage = true;
    cfg.app.udp.sendFeature = false;
    cfg.app.udp.sendMap = false;
    return cfg;
}

TEST(RuntimeConfigServiceTest, RemoteUpdateMutatesConfigAndRequestsRestartForPipelineChanges)
{
    UnifiedConfig config = MakeConfig();
    LiveRuntimeTuning tuning{};
    std::mutex configMutex;
    int restartCount = 0;
    RuntimeConfigService service(config, tuning, configMutex, [&restartCount]() { ++restartCount; });

    RemoteRuntimeConfig remote{};
    remote.exposureUs = 6000;
    remote.gain = 3.5f;
    remote.pairMs = 5;
    remote.slamInputFps = 15;
    remote.slamOperationMode = smartdrone::core::domain::SlamOperationMode::Localization;
    remote.sensorMode = SensorMode::Mono;
    remote.udpEnabled = true;
    remote.udpIp = "10.0.0.9";
    remote.sendImage = false;
    remote.sendFeature = true;
    remote.sendMap = true;

    std::string err;
    ASSERT_TRUE(service.UpdateRemoteConfig(remote, &err)) << err;

    EXPECT_EQ(config.app.camera.exposureUs, 6000);
    EXPECT_FLOAT_EQ(config.app.camera.gain, 3.5f);
    EXPECT_EQ(config.app.camera.pairMs, 5);
    EXPECT_EQ(config.app.runtime.slamInputFps, 15);
    EXPECT_EQ(config.app.runtime.slamOperationMode, smartdrone::core::domain::SlamOperationMode::Localization);
    EXPECT_EQ(config.app.sensorMode, SensorMode::Mono);
    EXPECT_EQ(std::filesystem::path(config.app.settings).filename().string(), "mono_right.yaml");
    EXPECT_TRUE(config.app.udp.enable);
    EXPECT_EQ(config.app.udp.ip, "10.0.0.9");
    EXPECT_FALSE(config.app.udp.sendImage);
    EXPECT_TRUE(config.app.udp.sendFeature);
    EXPECT_TRUE(config.app.udp.sendMap);
    EXPECT_EQ(tuning.slamInputFps.load(std::memory_order_relaxed), 15);
    EXPECT_EQ(tuning.slamOperationMode.load(std::memory_order_relaxed),
              static_cast<uint8_t>(smartdrone::core::domain::SlamOperationMode::Localization));
    EXPECT_EQ(restartCount, 1);
}

TEST(RuntimeConfigServiceTest, ApplyConfigRejectsTypeMismatch)
{
    UnifiedConfig config = MakeConfig();
    LiveRuntimeTuning tuning{};
    std::mutex configMutex;
    RuntimeConfigService service(config, tuning, configMutex, []() {});

    ConfigUpdate update{};
    update.values[std::string(ConfigRegistry::kStreamUdpEnabled)] = std::string("true");

    const CommandResult result = service.ApplyConfig(update, config);

    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.message, "stream.udp_enabled type mismatch");
}

} // namespace
} // namespace smartdrone::core::application
