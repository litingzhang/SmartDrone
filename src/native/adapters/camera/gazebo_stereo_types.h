#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace SmartDrone::Adapters::Camera {

enum class GazeboImagePixelFormat {
    Mono8,
    Rgb8,
    Bgr8,
    Unsupported,
};

enum class GazeboStereoEye {
    Left,
    Right,
};

struct GazeboImageFaultConfig {
    std::uint64_t generation{0};
    std::string action{"none"};
    int blurKernel{0};
    double blurSigma{0.0};
    double brightness{1.0};
    double noiseStddev{0.0};
    double dropRate{0.0};
    int delayMs{0};
    int blackoutMs{0};
};

struct GazeboStereoConfig {
    std::string leftImageTopic;
    std::string rightImageTopic;
    std::string clockTopic;
    std::string truthPoseTopic;
    std::string truthModelName;
    std::string calibrationPath;
    std::string faultStatePath;
    int cameraWidth{0};
    int cameraHeight{0};
    int cameraFps{0};
    std::int64_t pairToleranceNs{5000000};
    std::size_t queueDepth{8};
    GazeboImageFaultConfig fault;
};

struct GazeboRawImage {
    int width{0};
    int height{0};
    std::size_t step{0};
    GazeboImagePixelFormat pixelFormat{GazeboImagePixelFormat::Unsupported};
    std::uint64_t measurementTimestampNs{0};
    std::int64_t captureMonotonicNs{0};
    std::int64_t arrivalMonotonicNs{0};
    std::uint32_t sequence{0};
    std::uint32_t clockResetCounter{0};
    std::shared_ptr<std::vector<std::uint8_t>> payload;
};

} // namespace SmartDrone::Adapters::Camera
