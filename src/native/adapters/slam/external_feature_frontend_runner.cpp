#include "adapters/slam/external_feature_frontend_runner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace smartdrone::adapters::slam {

namespace {

cv::Mat BuildExternalFeatureInputImage(const cv::Mat &gray, int maxWidth, int maxHeight,
                                       float &scaleX, float &scaleY)
{
    scaleX = 1.0f;
    scaleY = 1.0f;
    if (gray.empty()) {
        return gray;
    }

    const int srcWidth = gray.cols;
    const int srcHeight = gray.rows;
    const float widthScale = maxWidth > 0 ? static_cast<float>(maxWidth) / static_cast<float>(std::max(1, srcWidth))
                                          : std::numeric_limits<float>::infinity();
    const float heightScale =
        maxHeight > 0 ? static_cast<float>(maxHeight) / static_cast<float>(std::max(1, srcHeight))
                      : std::numeric_limits<float>::infinity();
    const float resizeScale = std::min(1.0f, std::min(widthScale, heightScale));
    if (resizeScale >= 0.999f) {
        return gray;
    }

    const int targetWidth = std::max(32, static_cast<int>(std::lround(static_cast<float>(srcWidth) * resizeScale)));
    const int targetHeight = std::max(32, static_cast<int>(std::lround(static_cast<float>(srcHeight) * resizeScale)));
    cv::Mat resized;
    cv::resize(gray, resized, cv::Size(targetWidth, targetHeight), 0.0, 0.0, cv::INTER_AREA);
    scaleX = static_cast<float>(srcWidth) / static_cast<float>(targetWidth);
    scaleY = static_cast<float>(srcHeight) / static_cast<float>(targetHeight);
    return resized;
}

void RemapKeypointsToSource(std::vector<cv::Point2f> &keypoints, float scaleX, float scaleY)
{
    if (scaleX == 1.0f && scaleY == 1.0f) {
        return;
    }
    for (cv::Point2f &pt : keypoints) {
        pt.x *= scaleX;
        pt.y *= scaleY;
    }
}

} // namespace

bool RunExternalStereoFeatureFrontend(const ExternalStereoFeatureFrontendRunInput &input,
                                      ExternalStereoFeatureFrontendRunResult &result)
{
    result = {};
    if (input.client == nullptr || input.leftPrepared == nullptr || input.rightPrepared == nullptr ||
        input.leftPrepared->empty() || input.rightPrepared->empty()) {
        result.error = "invalid external feature frontend input";
        return false;
    }

    const auto inputStartTp = std::chrono::steady_clock::now();
    const cv::Mat leftInput = BuildExternalFeatureInputImage(*input.leftPrepared, input.inputMaxWidth,
                                                             input.inputMaxHeight, result.leftScaleX,
                                                             result.leftScaleY);
    const cv::Mat rightInput = BuildExternalFeatureInputImage(*input.rightPrepared, input.inputMaxWidth,
                                                              input.inputMaxHeight, result.rightScaleX,
                                                              result.rightScaleY);
    const auto inputEndTp = std::chrono::steady_clock::now();

    const auto frontendStartTp = std::chrono::steady_clock::now();
    if (!input.client->DetectAndComputeStereo(leftInput, rightInput, result.leftFeatures, result.rightFeatures,
                                              &result.error)) {
        return false;
    }
    const auto frontendEndTp = std::chrono::steady_clock::now();

    result.stats = input.client->LastStats();
    result.inputBuildMs = std::chrono::duration<double, std::milli>(inputEndTp - inputStartTp).count();
    result.frontendCallMs = std::chrono::duration<double, std::milli>(frontendEndTp - frontendStartTp).count();
    RemapKeypointsToSource(result.leftFeatures.keypoints, result.leftScaleX, result.leftScaleY);
    RemapKeypointsToSource(result.rightFeatures.keypoints, result.rightScaleX, result.rightScaleY);
    return true;
}

} // namespace smartdrone::adapters::slam
