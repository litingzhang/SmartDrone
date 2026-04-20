#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include "core/domain/runtime_mode.h"

namespace fs = std::filesystem;

enum class SensorMode {
    Stereo,
    StereoImu,
    Mono,
    MonoImu,
};

enum class FeatureFrontend {
    Orb,
    XFeat,
};

const char *DefaultSettingsForSensorMode(SensorMode mode);
SensorMode ParseSensorModeText(const std::string &text);
const char *ToSensorModeText(SensorMode mode);
FeatureFrontend ParseFeatureFrontendText(const std::string &text);
const char *ToFeatureFrontendText(FeatureFrontend frontend);
smartdrone::core::domain::SlamOperationMode ParseSlamOperationModeText(const std::string &text);
std::string ResolveRuntimePath(const std::string &path, const char *argv0);
std::string ResolveSettingsForSensorMode(SensorMode mode, const std::string &currentSettingsPath);

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
    int slamInputFps{0};
    smartdrone::core::domain::SlamOperationMode slamOperationMode{smartdrone::core::domain::SlamOperationMode::Mapping};
    FeatureFrontend featureFrontend{FeatureFrontend::Orb};
    std::string xfeatPython{"python3"};
    std::string xfeatRepo;
    std::string xfeatWorkerScript{"scripts/xfeat_keypoint_worker.py"};
    std::string xfeatDevice{"auto"};
    int xfeatTopK{512};
    int xfeatMaxPoints{320};
    int xfeatInputMaxWidth{640};
    int xfeatInputMaxHeight{400};
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
    std::string vocab{"ORBvoc.txt"};
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
    bool HasFlag(const char *name) const;
    uint8_t GetUint8HexOrDec(const char *name, uint8_t defaultValue, const char *defaultText) const;

  private:
    static uint8_t ParseUint8HexOrDec(const std::string &text, uint8_t defaultValue);

    int m_argc;
    char **m_argv;
};

AppConfig ParseAppConfig(int argc, char **argv);
