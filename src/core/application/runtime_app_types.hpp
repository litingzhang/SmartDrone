#pragma once

#include <atomic>
#include <string>

#include "core/application/app_args.hpp"

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
    std::string udpIp;
    bool udpEnabled{false};
    SensorMode sensorMode{SensorMode::Stereo};
    bool sendImage{true};
    bool sendFeature{false};
    bool sendMap{false};
};

struct MainRuntimeAliases {
    SensorMode sensorMode{SensorMode::Stereo};
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
};

}  // namespace smartdrone::core::application
