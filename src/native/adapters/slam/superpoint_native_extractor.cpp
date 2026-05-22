#include "adapters/slam/superpoint_native_extractor.h"

#include <opencv2/imgproc.hpp>

namespace SmartDrone::Adapters::Slam {

struct SuperPointNativeExtractor::Impl {};

SuperPointNativeExtractor::SuperPointNativeExtractor() = default;
SuperPointNativeExtractor::~SuperPointNativeExtractor() = default;

bool SuperPointNativeExtractor::PrepareGrayImage(const cv::Mat &gray,
                                                 cv::Mat &gray8,
                                                 std::string *err)
{
    if (gray.type() == CV_8UC1 && gray.isContinuous()) {
        gray8 = gray;
        return true;
    }
    if (gray.channels() == 1) {
        gray.convertTo(gray8, CV_8UC1);
    } else {
        cv::cvtColor(gray, gray8, cv::COLOR_BGR2GRAY);
    }
    if (!gray8.isContinuous()) {
        gray8 = gray8.clone();
    }
    if (gray8.empty()) {
        if (err != nullptr) {
            *err = "SuperPoint native gray image preparation failed";
        }
        return false;
    }
    return true;
}

bool SuperPointNativeExtractor::Start(const std::string &, const std::string &,
                                      int topK, int maxPoints, std::string *err)
{
    m_topK = topK;
    m_maxPoints = maxPoints;
    m_lastStats = Stats{};
    m_running = false;
    if (err != nullptr) {
        *err = "native TensorRT SuperPoint backend is unavailable in this target";
    }
    return false;
}

void SuperPointNativeExtractor::Stop()
{
    m_running = false;
    m_lastStats = Stats{};
    m_impl.reset();
}

bool SuperPointNativeExtractor::Running() const
{
    return m_running;
}

SuperPointNativeExtractor::Stats SuperPointNativeExtractor::LastStats() const
{
    return m_lastStats;
}

void SuperPointNativeExtractor::SetLightGlueEveryNOverride(int everyN)
{
    m_lightGlueEveryNOverride = everyN > 0 ? std::clamp(everyN, 1, 120) : 0;
}

bool SuperPointNativeExtractor::Detect(const cv::Mat &gray,
                                       std::vector<cv::Point2f> &outPoints,
                                       std::string *err)
{
    SuperPointFeatureSet features;
    if (!DetectAndCompute(gray, features, err)) {
        return false;
    }
    outPoints = std::move(features.keypoints);
    return true;
}

bool SuperPointNativeExtractor::DetectAndCompute(
    const cv::Mat &, SuperPointFeatureSet &outFeatures, std::string *err)
{
    outFeatures = SuperPointFeatureSet{};
    m_lastStats = Stats{};
    if (err != nullptr) {
        *err = "native TensorRT SuperPoint backend is compiled out";
    }
    return false;
}

bool SuperPointNativeExtractor::DetectAndComputeStereo(
    const cv::Mat &, const cv::Mat &, SuperPointFeatureSet &leftFeatures,
    SuperPointFeatureSet &rightFeatures, std::string *err)
{
    leftFeatures = SuperPointFeatureSet{};
    rightFeatures = SuperPointFeatureSet{};
    m_lastStats = Stats{};
    if (err != nullptr) {
        *err = "native TensorRT SuperPoint backend is compiled out";
    }
    return false;
}

} // namespace SmartDrone::Adapters::Slam
