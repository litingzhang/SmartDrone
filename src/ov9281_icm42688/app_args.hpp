#pragma once

#include <cctype>
#include <cstdint>
#include <algorithm>
#include <string>

enum class SensorMode {
    Stereo,
    StereoImu,
};

inline SensorMode ParseSensorModeText(const std::string& text)
{
    std::string normalized = text;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (normalized == "stereo-imu" || normalized == "imu-stereo" || normalized == "stereo_inertial") {
        return SensorMode::StereoImu;
    }
    return SensorMode::Stereo;
}

struct CameraConfig {
    int width{640};
    int height{400};
    int fps{30};
    bool aeDisable{true};
    int exposureUs{5000};
    float gain{8.0f};
    bool requestY8{true};
    bool r16Norm{false};
    int pairMs{2};
    int keepMs{120};
};

struct UdpConfig {
    bool enable{false};
    std::string ip{"192.168.1.10"};
    int port{5000};
    int cmdPort{14550};
    int jpegQ{80};
    int payload{1200};
    int queue{4};
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
};

struct AppConfig {
    std::string vocab{"ORBvoc.txt"};
    std::string settings{"stereo_inertial.yaml"};
    SensorMode sensorMode{SensorMode::Stereo};
    CameraConfig camera;
    UdpConfig udp;
    ImuRuntimeConfig imu;
    RuntimeConfig runtime;
};

class ArgReader {
public:
    ArgReader(int argc, char** argv) : m_argc(argc), m_argv(argv) {}

    std::string GetString(const char* name, const char* defaultValue) const
    {
        for (int i = 1; i + 1 < m_argc; ++i) {
            if (std::string(m_argv[i]) == name) {
                return m_argv[i + 1];
            }
        }
        return defaultValue;
    }

    int GetInt(const char* name, int defaultValue) const
    {
        for (int i = 1; i + 1 < m_argc; ++i) {
            if (std::string(m_argv[i]) == name) {
                return std::stoi(m_argv[i + 1]);
            }
        }
        return defaultValue;
    }

    int64_t GetInt64(const char* name, int64_t defaultValue) const
    {
        for (int i = 1; i + 1 < m_argc; ++i) {
            if (std::string(m_argv[i]) == name) {
                return std::stoll(m_argv[i + 1]);
            }
        }
        return defaultValue;
    }

    float GetFloat(const char* name, float defaultValue) const
    {
        for (int i = 1; i + 1 < m_argc; ++i) {
            if (std::string(m_argv[i]) == name) {
                return std::stof(m_argv[i + 1]);
            }
        }
        return defaultValue;
    }

    bool HasFlag(const char* name) const
    {
        for (int i = 1; i < m_argc; ++i) {
            if (std::string(m_argv[i]) == name) {
                return true;
            }
        }
        return false;
    }

    uint8_t GetUint8HexOrDec(const char* name, uint8_t defaultValue, const char* defaultText) const
    {
        return ParseUint8HexOrDec(GetString(name, defaultText), defaultValue);
    }

private:
    static uint8_t ParseUint8HexOrDec(const std::string& text, uint8_t defaultValue)
    {
        try {
            int base = 10;
            if (text.size() > 2 && text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
                base = 16;
            }
            int value = std::stoi(text, nullptr, base);
            if (value < 0 || value > 255) {
                return defaultValue;
            }
            return static_cast<uint8_t>(value);
        } catch (...) {
            return defaultValue;
        }
    }

    int m_argc;
    char** m_argv;
};

inline AppConfig ParseAppConfig(int argc, char** argv)
{
    ArgReader argReader(argc, argv);
    AppConfig config;

    config.vocab = argReader.GetString("--vocab", "ORBvoc.txt");
    config.settings = argReader.GetString("--settings", "stereo_inertial.yaml");
    config.sensorMode = ParseSensorModeText(argReader.GetString("--sensor-mode", "stereo"));

    config.camera.width = argReader.GetInt("--w", 640);
    config.camera.height = argReader.GetInt("--h", 400);
    config.camera.fps = argReader.GetInt("--fps", 60);
    config.camera.aeDisable = !argReader.HasFlag("--ae");
    config.camera.exposureUs = argReader.GetInt("--exp-us", 5000);
    config.camera.gain = argReader.GetFloat("--gain", 8.0f);
    config.camera.requestY8 = !argReader.HasFlag("--no-y8");
    config.camera.r16Norm = argReader.HasFlag("--r16-norm");
    config.camera.pairMs = argReader.GetInt("--pair-ms", 2);
    config.camera.keepMs = argReader.GetInt("--keep-ms", 120);

    config.udp.enable = argReader.HasFlag("--udp");
    config.udp.ip = argReader.GetString("--udp-ip", "10.42.0.109");
    config.udp.port = argReader.GetInt("--udp-port", 5000);
    config.udp.cmdPort = argReader.GetInt("--cmd-port", 14550);
    config.udp.jpegQ = argReader.GetInt("--udp-jpeg-q", 80);
    config.udp.payload = argReader.GetInt("--udp-payload", 1200);
    config.udp.queue = argReader.GetInt("--udp-queue", 4);

    config.imu.spiDev = argReader.GetString("--spi", "/dev/spidev0.0");
    config.imu.spiSpeed = static_cast<uint32_t>(argReader.GetInt("--speed", 8000000));
    config.imu.spiMode = static_cast<uint8_t>(argReader.GetInt("--mode", 0));
    config.imu.spiBits = static_cast<uint8_t>(argReader.GetInt("--bits", 8));
    config.imu.gpiochip = argReader.GetString("--gpiochip", "/dev/gpiochip0");
    config.imu.drdyLine = static_cast<unsigned>(argReader.GetInt("--drdy", 24));
    config.imu.imuHz = argReader.GetInt("--imu-hz", 500);
    config.imu.accelFsG = argReader.GetInt("--accel-fs", 16);
    config.imu.gyroFsDps = argReader.GetInt("--gyro-fs", 2000);
    config.imu.imuStartReg = argReader.GetUint8HexOrDec("--imu-start-reg", 0x1F, "0x1F");
    config.imu.rtImu = argReader.HasFlag("--rt-imu");
    config.imu.rtPrio = argReader.GetInt("--rt-prio", 60);

    config.runtime.offRejectNs = argReader.GetInt64("--off-reject-ns", 10'000'000);
    config.runtime.allowEmptyImu = argReader.HasFlag("--allow-empty-imu");

    return config;
}
