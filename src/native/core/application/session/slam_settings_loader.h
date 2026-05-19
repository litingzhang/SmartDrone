#pragma once

#include <optional>
#include <string>

#include <opencv2/core/persistence.hpp>
#include <sophus/se3.hpp>

namespace smartdrone::core::application {

struct StereoBodyExtrinsics {
    Sophus::SE3f Tbc{Sophus::SE3f()};
    bool loaded{false};
};

struct OrbExtractorSettings {
    int nFeatures{0};
    float scaleFactor{0.0f};
    int nLevels{0};
    int iniThFAST{0};
    int minThFAST{0};
    bool loaded{false};
};

std::optional<Sophus::SE3f> ReadSe3Node(const cv::FileNode &node);
StereoBodyExtrinsics LoadStereoBodyExtrinsics(const std::string &settingsPath);
OrbExtractorSettings LoadOrbExtractorSettings(const std::string &settingsPath);

} // namespace smartdrone::core::application
