#pragma once

#include <atomic>
#include <string>

#include "core/application/config/app_args.h"
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
    int uvcDeviceIndex{0};
    int uvcEyeWidth{640};
    int uvcEyeHeight{400};
    bool uvcPackedStereo{true};
    bool autoExposureEnabled{true};
    int slamInputFps{0};
    domain::SlamOperationMode slamOperationMode{domain::SlamOperationMode::Mapping};
    FeatureFrontend featureFrontend{FeatureFrontend::Orb};
    std::string udpIp;
    bool udpEnabled{false};
    SensorMode sensorMode{SensorMode::Stereo};
    bool sendImage{true};
    bool sendFeature{true};
    bool sendMap{false};
    bool useCustomTbc{false};
    float tbcTx{0.0f};
    float tbcTy{0.0f};
    float tbcTz{0.0f};
    float tbcRollDeg{0.0f};
    float tbcPitchDeg{0.0f};
    float tbcYawDeg{0.0f};
    int orbNFeatures{0};
    float orbScaleFactor{0.0f};
    int orbNLevels{0};
    int orbIniThFAST{0};
    int orbMinThFAST{0};
    int xfeatTopK{512};
    int xfeatMaxPoints{320};
    int xfeatInputMaxWidth{640};
    int xfeatInputMaxHeight{400};
};

struct MainRuntimeAliases {
    SensorMode sensorMode{SensorMode::Stereo};
    domain::SlamOperationMode slamOperationMode{domain::SlamOperationMode::Mapping};
    FeatureFrontend featureFrontend{FeatureFrontend::Orb};
    int width{}, height{}, fps{}, slamInputFps{}, leftCamIndex{}, rightCamIndex{}, exposureUs{}, pairMs{}, keepMs{},
        pairQueue{};
    int uvcDeviceIndex{}, uvcEyeWidth{}, uvcEyeHeight{};
    bool aeDisable{}, requestY8{}, r16Norm{}, udpEnable{}, allowEmptyImu{}, rtImu{}, debugRightOnlyFeatures{},
        slamLowLightEnhance{}, jsonDiagnostics{}, uvcPackedStereo{};
    bool sendImage{true}, sendFeature{true}, sendMap{false};
    float gain{};
    std::string udpIp, spiDev, gpiochip;
    int xfeatInputMaxWidth{}, xfeatInputMaxHeight{};
    int udpPort{}, cmdPort{}, udpJpegQ{}, udpPayload{}, udpQueue{}, imuHz{}, accelFsG{}, gyroFsDps{}, rtPrio{};
    uint32_t spiSpeed{};
    uint8_t spiMode{}, spiBits{}, imuStartReg{};
    unsigned drdyLine{};
};

struct LiveRuntimeTuning {
    std::atomic<int> slamInputFps{0};
    std::atomic<uint8_t> slamOperationMode{static_cast<uint8_t>(domain::SlamOperationMode::Mapping)};
    std::atomic<uint8_t> featureFrontend{static_cast<uint8_t>(FeatureFrontend::Orb)};
    std::atomic<bool> sendImage{true};
    std::atomic<bool> sendFeature{true};
    std::atomic<bool> sendMap{false};
    std::atomic<bool> useCustomTbc{false};
    std::atomic<float> tbcTx{0.0f};
    std::atomic<float> tbcTy{0.0f};
    std::atomic<float> tbcTz{0.0f};
    std::atomic<float> tbcRollDeg{0.0f};
    std::atomic<float> tbcPitchDeg{0.0f};
    std::atomic<float> tbcYawDeg{0.0f};
};

} // namespace smartdrone::core::application
