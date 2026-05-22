#include "core/application/session/slam/slam_settings_loader.h"

#include <iostream>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace SmartDrone::core::application {

std::optional<Sophus::SE3f> ReadSe3Node(const cv::FileNode &node)
{
    if (node.empty()) {
        return std::nullopt;
    }

    cv::Mat mat;
    node >> mat;
    if (mat.empty() || mat.rows != 4 || mat.cols != 4) {
        return std::nullopt;
    }

    cv::Mat mat32f;
    mat.convertTo(mat32f, CV_32F);
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            transform(r, c) = mat32f.at<float>(r, c);
        }
    }
    return Sophus::SE3f(transform);
}

StereoBodyExtrinsics LoadStereoBodyExtrinsics(const std::string &settingsPath)
{
    StereoBodyExtrinsics extrinsics;
    cv::FileStorage storage(settingsPath, cv::FileStorage::READ);
    if (!storage.isOpened()) {
        std::cerr << "[pose] warning: failed to open settings for stereo body "
                     "extrinsics: "
                  << settingsPath << "\n";
        return extrinsics;
    }

    const auto maybeTbc = ReadSe3Node(storage["T_b_c1"]);
    const auto maybeImuTbc = ReadSe3Node(storage["IMU.T_b_c1"]);
    if (maybeTbc.has_value()) {
        extrinsics.Tbc = *maybeTbc;
        extrinsics.loaded = true;
    } else if (maybeImuTbc.has_value()) {
        extrinsics.Tbc = *maybeImuTbc;
        extrinsics.loaded = true;
    } else {
        std::cerr << "[pose] info: no T_b_c1/IMU.T_b_c1 in settings, pure stereo "
                     "pose stays in camera frame\n";
    }

    if (extrinsics.loaded) {
        const Eigen::Vector3f translation = extrinsics.Tbc.translation();
        std::cerr << "[pose] pure stereo will publish body pose using T_b_c1"
                  << " tx=" << translation.x() << " ty=" << translation.y()
                  << " tz=" << translation.z() << "\n";
    }
    return extrinsics;
}

OrbExtractorSettings LoadOrbExtractorSettings(const std::string &settingsPath)
{
    OrbExtractorSettings settings;
    cv::FileStorage storage(settingsPath, cv::FileStorage::READ);
    if (!storage.isOpened()) {
        std::cerr << "[slam] warning: failed to open settings for ORB extractor: "
                  << settingsPath << "\n";
        return settings;
    }

    const cv::FileNode nFeaturesNode = storage["ORBextractor.nFeatures"];
    const cv::FileNode scaleFactorNode = storage["ORBextractor.scaleFactor"];
    const cv::FileNode nLevelsNode = storage["ORBextractor.nLevels"];
    const cv::FileNode iniThFastNode = storage["ORBextractor.iniThFAST"];
    const cv::FileNode minThFastNode = storage["ORBextractor.minThFAST"];
    if (nFeaturesNode.empty() || scaleFactorNode.empty() || nLevelsNode.empty() ||
        iniThFastNode.empty() || minThFastNode.empty()) {
        std::cerr << "[slam] warning: ORBextractor.* keys missing in settings: "
                  << settingsPath << "\n";
        return settings;
    }

    settings.nFeatures = static_cast<int>(nFeaturesNode);
    settings.scaleFactor = static_cast<float>(scaleFactorNode);
    settings.nLevels = static_cast<int>(nLevelsNode);
    settings.iniThFAST = static_cast<int>(iniThFastNode);
    settings.minThFAST = static_cast<int>(minThFastNode);
    settings.loaded = settings.nFeatures > 0 && settings.scaleFactor > 0.0f &&
                      settings.nLevels > 0 && settings.iniThFAST > 0 &&
                      settings.minThFAST > 0;
    if (!settings.loaded) {
        std::cerr << "[slam] warning: invalid ORBextractor.* values in settings: "
                  << settingsPath << "\n";
    }
    return settings;
}

} // namespace SmartDrone::core::application
