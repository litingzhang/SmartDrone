#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include "core/domain/feature_frontend.h"
#include "core/domain/runtime_mode.h"

namespace fs = std::filesystem;

const char *DefaultSettingsForSensorMode(SensorMode mode);
SensorMode NormalizeSensorModeForSlamBackend(SensorMode mode,
                                             SlamBackend backend);
const char *DefaultSettingsForSlamBackend(SensorMode mode,
                                          SlamBackend backend);
SensorMode ParseSensorModeText(const std::string &text);
const char *ToSensorModeText(SensorMode mode);
FeatureFrontend ParseFeatureFrontendText(const std::string &text);
const char *ToFeatureFrontendText(FeatureFrontend frontend);
bool IsVisualFeatureLightGlueFrontend(FeatureFrontend frontend);
SlamBackend ParseSlamBackendText(const std::string &text);
SlamBackend NormalizeSlamBackendForBuild(SlamBackend backend);
const char *ToSlamBackendText(SlamBackend backend);
SmartDrone::Core::Domain::SlamOperationMode
ParseSlamOperationModeText(const std::string &text);
std::string ResolveRuntimePath(const std::string &path, const char *argv0);
std::string
ResolveSettingsForSensorMode(SensorMode mode,
                             const std::string &currentSettingsPath);
std::string
ResolveSettingsForSlamBackend(SensorMode mode, SlamBackend backend,
                              const std::string &currentSettingsPath);

struct CameraConfig {
    int width{640};
    int height{400};
    int fps{60};
    int leftCamIndex{0};
    int rightCamIndex{1};
    int uvcDeviceIndex{0};
    int uvcEyeWidth{640};
    int uvcEyeHeight{400};
    bool uvcPackedStereo{true};
    bool uvcSwapEyes{false};
    bool aeDisable{false};
    int exposureUs{5000};
    float gain{2.0f};
    bool requestY8{true};
    bool r16Norm{false};
    int pairMs{2};
    int keepMs{120};
    int pairQueue{3};
};

struct UdpConfig {
    bool enable{false};
    std::string ip{"192.168.1.10"};
    int port{5000};
    int cmdPort{14550};
    int jpegQ{80};
    int payload{1200};
    int queue{4};
    bool sendImage{true};
    bool sendFeature{true};
    bool sendMap{false};
};

struct ImuRuntimeConfig {
    std::string spiDev{"/dev/spidev0.0"};
    uint32_t spiSpeed{8000000};
    uint8_t spiMode{0};
    uint8_t spiBits{8};
    std::string gpiochip{"/dev/gpiochip0"};
    unsigned drdyLine{24};
    int imuHz{500};
    int accelFsG{16};
    int gyroFsDps{2000};
    uint8_t imuStartReg{0x1F};
    bool rtImu{false};
    int rtPrio{60};
};

struct RuntimeConfig {
    int64_t offRejectNs{10'000'000};
    bool allowEmptyImu{false};
    int slamInputFps{30};
    SmartDrone::Core::Domain::SlamOperationMode slamOperationMode{
        SmartDrone::Core::Domain::SlamOperationMode::Mapping};
    SlamBackend slamBackend{SlamBackend::Klt};
    FeatureFrontend featureFrontend{FeatureFrontend::LkGfttPerFrame};
    std::string dpvoRepo;
    std::string dpvoPatchEngine;
    std::string dpvoUpdateEngine;
    int dpvoInputWidth{640};
    int dpvoInputHeight{400};
    int dpvoPatchesPerFrame{48};
    int dpvoOptimizationWindow{7};
    std::string visualFeatureRepo;
    std::string visualFeatureDevice{"auto"};
    int visualFeatureTopK{1024};
    int visualFeatureMaxPoints{512};
    int visualFeatureInputMaxWidth{640};
    int visualFeatureInputMaxHeight{409};
    bool lkLoopClosure{false};
    float lkLoopScale{1.20f};
    float lkLoopRelaxation{1.40f};
    std::string lkPerFrameAcceleration{"vpi-cuda"};
    std::string orbAcceleration{"cpu"};
    bool debugRightOnlyFeatures{false};
    bool slamLowLightEnhance{false};
    bool jsonDiagnostics{false};
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
};

struct AppConfig {
    std::string vocab{};
    std::string settings{"config/stereo.yaml"};
    SensorMode sensorMode{SensorMode::Stereo};
    CameraConfig camera;
    UdpConfig udp;
    ImuRuntimeConfig imu;
    RuntimeConfig runtime;
};

class ArgReader {
  public:
    ArgReader(int argc, char **argv);

    std::string GetString(const char *name, const char *defaultValue) const;
    int GetInt(const char *name, int defaultValue) const;
    int64_t GetInt64(const char *name, int64_t defaultValue) const;
    float GetFloat(const char *name, float defaultValue) const;
    bool HasOption(const char *name) const;
    bool HasFlag(const char *name) const;
    uint8_t GetUint8HexOrDec(const char *name, uint8_t defaultValue,
                             const char *defaultText) const;

  private:
    static uint8_t ParseUint8HexOrDec(const std::string &text,
                                      uint8_t defaultValue);

    int m_argc;
    char **m_argv;
};

AppConfig ParseAppConfig(int argc, char **argv);
