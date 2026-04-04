#pragma once

#include <atomic>
#include <string>

#include "core/application/app_args.h"
#include "core/domain/runtime_mode.h"

namespace smartdrone::core::application {

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
    int slamInputFps{0};
    domain::SlamOperationMode slamOperationMode{domain::SlamOperationMode::Mapping};
    std::string udpIp;
    bool udpEnabled{false};
    SensorMode sensorMode{SensorMode::Stereo};
    bool sendImage{true};
    bool sendFeature{false};
    bool sendMap{false};
};

struct MainRuntimeAliases {
    SensorMode sensorMode{SensorMode::Stereo};
    domain::SlamOperationMode slamOperationMode{domain::SlamOperationMode::Mapping};
    int width{}, height{}, fps{}, slamInputFps{}, leftCamIndex{}, rightCamIndex{}, exposureUs{}, pairMs{}, keepMs{},
        pairQueue{};
    bool aeDisable{}, requestY8{}, r16Norm{}, udpEnable{}, allowEmptyImu{}, rtImu{}, debugRightOnlyFeatures{},
        slamLowLightEnhance{};
    bool sendImage{true}, sendFeature{false}, sendMap{false};
    float gain{};
    std::string udpIp, spiDev, gpiochip;
    int udpPort{}, cmdPort{}, udpJpegQ{}, udpPayload{}, udpQueue{}, imuHz{}, accelFsG{}, gyroFsDps{},
        rtPrio{};
    uint32_t spiSpeed{};
    uint8_t spiMode{}, spiBits{}, imuStartReg{};
    unsigned drdyLine{};
};

struct LiveRuntimeTuning {
    std::atomic<int> slamInputFps{0};
    std::atomic<uint8_t> slamOperationMode{static_cast<uint8_t>(domain::SlamOperationMode::Mapping)};
    std::atomic<bool> sendImage{true};
    std::atomic<bool> sendFeature{false};
    std::atomic<bool> sendMap{false};
};

}  // namespace smartdrone::core::application
