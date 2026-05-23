#include "adapters/slam/orb/orb_feature_utils.h"

#include <chrono>
#include <utility>

#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "adapters/slam/stereo/descriptor_geometry.h"

namespace SmartDrone::Adapters::Slam {

namespace {

bool ComputeOrbDescriptorsAtPoints(ORB_SLAM3::ORBextractor *extractor,
                                   const cv::Mat &gray,
                                   const std::vector<cv::Point2f> &points,
                                   std::vector<cv::KeyPoint> &keypoints,
                                   cv::Mat &descriptors)
{
    keypoints.clear();
    descriptors.release();
    if (extractor == nullptr || gray.empty() || points.empty()) {
        return false;
    }

    keypoints.reserve(points.size());
    for (const cv::Point2f &pt : points) {
        keypoints.push_back(MakeDescriptorKeyPoint(pt));
    }

    if (!extractor->ComputeDescriptorsAtKeypoints(gray, keypoints, descriptors)) {
        return false;
    }
    return !keypoints.empty() && !descriptors.empty() &&
           descriptors.rows == static_cast<int>(keypoints.size()) &&
           descriptors.type() == CV_8U;
}

bool DetectAndComputeOrbFeatures(ORB_SLAM3::ORBextractor *extractor,
                                 const cv::Mat &gray,
                                 Core::Ports::VisualFeatureSet &features)
{
    features = {};
    if (extractor == nullptr || gray.empty()) {
        return false;
    }

    std::vector<cv::KeyPoint> keypoints;
    std::vector<int> lapping = {0, 0};
    (*extractor)(gray, cv::Mat(), keypoints, features.descriptors, lapping);
    if (keypoints.empty() || features.descriptors.empty() ||
        features.descriptors.rows != static_cast<int>(keypoints.size()) ||
        features.descriptors.type() != CV_8U) {
        features = {};
        return false;
    }

    features.keypoints.reserve(keypoints.size());
    for (const cv::KeyPoint &keypoint : keypoints) {
        features.keypoints.push_back(keypoint.pt);
    }
    return true;
}

} // namespace

OrbDescriptorProvider::OrbDescriptorProvider(ORB_SLAM3::ORBextractor *extractor)
    : m_extractor(extractor)
{
}

void OrbDescriptorProvider::SetExtractor(ORB_SLAM3::ORBextractor *extractor)
{
    m_extractor = extractor;
}

bool OrbDescriptorProvider::Available() const
{
    return m_extractor != nullptr;
}

bool OrbDescriptorProvider::ComputeDescriptorsAtPoints(
    const cv::Mat &gray, const std::vector<cv::Point2f> &points,
    std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors) const
{
    return ComputeOrbDescriptorsAtPoints(m_extractor, gray, points, keypoints,
                                         descriptors);
}

bool OrbDescriptorProvider::DetectAndCompute(
    const cv::Mat &gray, std::vector<cv::KeyPoint> &keypoints,
    cv::Mat &descriptors) const
{
    keypoints.clear();
    descriptors.release();
    if (m_extractor == nullptr || gray.empty()) {
        return false;
    }
    std::vector<int> lapping = {0, 0};
    (*m_extractor)(gray, cv::Mat(), keypoints, descriptors, lapping);
    return !keypoints.empty() && !descriptors.empty() &&
           descriptors.rows == static_cast<int>(keypoints.size()) &&
           descriptors.type() == CV_8U;
}

int OrbDescriptorProvider::DescriptorDistance(
    const cv::Mat &leftDescriptor, const cv::Mat &rightDescriptor) const
{
    return ORB_SLAM3::ORBmatcher::DescriptorDistance(leftDescriptor,
                                                     rightDescriptor);
}

struct DefaultOrbFeatureFrontend::Impl {
    explicit Impl(const OrbFeatureExtractorOptions &options)
        : extractor(std::make_unique<ORB_SLAM3::ORBextractor>(
              options.maxFeatures, options.scaleFactor, options.levels,
              options.initialFastThreshold, options.minimumFastThreshold))
    {
    }

    std::unique_ptr<ORB_SLAM3::ORBextractor> extractor;
    Core::Ports::VisualFeatureFrontendStats lastStats;
};

DefaultOrbFeatureFrontend::DefaultOrbFeatureFrontend(
    const OrbFeatureExtractorOptions &options)
    : m_impl(std::make_unique<Impl>(options))
{
}

DefaultOrbFeatureFrontend::~DefaultOrbFeatureFrontend() = default;

bool DefaultOrbFeatureFrontend::Running() const
{
    return m_impl && m_impl->extractor != nullptr;
}

bool DefaultOrbFeatureFrontend::Detect(
    const Core::Ports::VisualFeatureDetectRequest &request,
    Core::Ports::VisualFeatureDetectResult &result)
{
    result = {};
    Core::Ports::VisualFeatureComputeRequest computeRequest;
    computeRequest.gray = request.gray;
    Core::Ports::VisualFeatureComputeResult computeResult;
    if (!DetectAndCompute(computeRequest, computeResult)) {
        result.error = std::move(computeResult.error);
        return false;
    }
    result.points = std::move(computeResult.features.keypoints);
    return true;
}

bool DefaultOrbFeatureFrontend::DetectAndCompute(
    const Core::Ports::VisualFeatureComputeRequest &request,
    Core::Ports::VisualFeatureComputeResult &result)
{
    result = {};
    if (request.gray == nullptr) {
        result.error = "ORB feature extraction input is null";
        return false;
    }
    const auto start = std::chrono::steady_clock::now();
    const bool ok = DetectAndComputeOrbFeatures(
        m_impl ? m_impl->extractor.get() : nullptr, *request.gray,
        result.features);
    const auto end = std::chrono::steady_clock::now();

    if (m_impl) {
        m_impl->lastStats = {};
        m_impl->lastStats.forwardMs =
            std::chrono::duration<double, std::milli>(end - start).count();
        m_impl->lastStats.totalMs = m_impl->lastStats.forwardMs;
        m_impl->lastStats.rawLeftCount =
            static_cast<int>(result.features.keypoints.size());
        m_impl->lastStats.stereoLeftCount = m_impl->lastStats.rawLeftCount;
    }
    if (!ok) {
        result.error = "ORB feature extraction failed";
    }
    return ok;
}

bool DefaultOrbFeatureFrontend::DetectAndComputeStereo(
    const Core::Ports::StereoVisualFeatureComputeRequest &request,
    Core::Ports::StereoVisualFeatureComputeResult &result)
{
    result = {};
    if (request.leftGray == nullptr || request.rightGray == nullptr) {
        result.error = "ORB stereo feature extraction input is null";
        return false;
    }
    const auto start = std::chrono::steady_clock::now();
    ORB_SLAM3::ORBextractor *extractor =
        m_impl ? m_impl->extractor.get() : nullptr;
    const bool leftOk =
        DetectAndComputeOrbFeatures(extractor, *request.leftGray,
                                    result.leftFeatures);
    const bool rightOk =
        DetectAndComputeOrbFeatures(extractor, *request.rightGray,
                                    result.rightFeatures);
    const auto end = std::chrono::steady_clock::now();

    if (m_impl) {
        m_impl->lastStats = {};
        m_impl->lastStats.forwardMs =
            std::chrono::duration<double, std::milli>(end - start).count();
        m_impl->lastStats.totalMs = m_impl->lastStats.forwardMs;
        m_impl->lastStats.rawLeftCount =
            static_cast<int>(result.leftFeatures.keypoints.size());
        m_impl->lastStats.rawRightCount =
            static_cast<int>(result.rightFeatures.keypoints.size());
        m_impl->lastStats.stereoLeftCount = m_impl->lastStats.rawLeftCount;
        m_impl->lastStats.stereoRightCount = m_impl->lastStats.rawRightCount;
        m_impl->lastStats.imageCount = 2;
    }
    if (!leftOk || !rightOk) {
        result.error = "ORB stereo feature extraction failed";
    }
    return leftOk && rightOk;
}

void DefaultOrbFeatureFrontend::SetLightGlueEveryNOverride(int)
{
}

Core::Ports::VisualFeatureFrontendStats
DefaultOrbFeatureFrontend::LastStats() const
{
    return m_impl ? m_impl->lastStats : Core::Ports::VisualFeatureFrontendStats{};
}

bool DefaultOrbFeatureFrontend::ComputeDescriptorsAtPoints(
    const cv::Mat &gray, const std::vector<cv::Point2f> &points,
    std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors) const
{
    return ComputeOrbDescriptorsAtPoints(m_impl ? m_impl->extractor.get()
                                                : nullptr,
                                         gray, points, keypoints, descriptors);
}

bool DefaultOrbFeatureFrontend::DetectAndCompute(
    const cv::Mat &gray, std::vector<cv::KeyPoint> &keypoints,
    cv::Mat &descriptors) const
{
    keypoints.clear();
    descriptors.release();
    if (!m_impl || m_impl->extractor == nullptr || gray.empty()) {
        return false;
    }
    std::vector<int> lapping = {0, 0};
    (*m_impl->extractor)(gray, cv::Mat(), keypoints, descriptors, lapping);
    return !keypoints.empty() && !descriptors.empty() &&
           descriptors.rows == static_cast<int>(keypoints.size()) &&
           descriptors.type() == CV_8U;
}

int DefaultOrbFeatureFrontend::DescriptorDistance(
    const cv::Mat &leftDescriptor, const cv::Mat &rightDescriptor) const
{
    return ORB_SLAM3::ORBmatcher::DescriptorDistance(leftDescriptor,
                                                     rightDescriptor);
}

} // namespace SmartDrone::Adapters::Slam
