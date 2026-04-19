#include "adapters/slam/orbslam3_engine.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <sophus/se3.hpp>
#include <opencv2/imgproc.hpp>

#include "ImuTypes.h"
#include "ORBmatcher.h"
#include "TrackedVisualData.h"

namespace smartdrone::adapters::slam {

namespace {

struct StereoMatchPair {
    int leftIndex{-1};
    int rightIndex{-1};
    int distance{std::numeric_limits<int>::max()};
};

constexpr int kXFeatTargetMaxWidth = 320;
constexpr int kXFeatTargetMaxHeight = 200;

cv::KeyPoint MakeKeyPoint(const cv::Point2f &pt)
{
    cv::KeyPoint kp;
    kp.pt = pt;
    kp.size = 8.0f;
    kp.angle = -1.0f;
    kp.octave = 0;
    kp.response = 1.0f;
    return kp;
}

std::vector<StereoMatchPair> MatchStereoPairs(const XFeatFeatureSet &left, const XFeatFeatureSet &right)
{
    std::vector<StereoMatchPair> matches;
    if (left.descriptors.empty() || right.descriptors.empty()) {
        return matches;
    }

    std::vector<int> bestRightForLeft(left.descriptors.rows, -1);
    std::vector<int> bestLeftForRight(right.descriptors.rows, -1);
    std::vector<int> bestRightDist(left.descriptors.rows, std::numeric_limits<int>::max());
    std::vector<int> bestLeftDist(right.descriptors.rows, std::numeric_limits<int>::max());

    for (int li = 0; li < left.descriptors.rows; ++li) {
        const cv::Point2f &leftPt = left.keypoints[static_cast<size_t>(li)];
        for (int ri = 0; ri < right.descriptors.rows; ++ri) {
            const cv::Point2f &rightPt = right.keypoints[static_cast<size_t>(ri)];
            const float yDelta = std::fabs(leftPt.y - rightPt.y);
            const float disparity = leftPt.x - rightPt.x;
            if (yDelta > 3.0f || disparity <= 0.5f) {
                continue;
            }
            const int dist =
                ORB_SLAM3::ORBmatcher::DescriptorDistance(left.descriptors.row(li), right.descriptors.row(ri));
            if (dist < bestRightDist[li]) {
                bestRightDist[li] = dist;
                bestRightForLeft[li] = ri;
            }
            if (dist < bestLeftDist[ri]) {
                bestLeftDist[ri] = dist;
                bestLeftForRight[ri] = li;
            }
        }
    }

    matches.reserve(static_cast<size_t>(std::min(left.descriptors.rows, right.descriptors.rows)));
    for (int li = 0; li < left.descriptors.rows; ++li) {
        const int ri = bestRightForLeft[li];
        if (ri < 0 || bestLeftForRight[ri] != li) {
            continue;
        }
        matches.push_back(StereoMatchPair{li, ri, bestRightDist[li]});
    }

    std::sort(matches.begin(), matches.end(), [](const StereoMatchPair &a, const StereoMatchPair &b) {
        return a.distance < b.distance;
    });
    return matches;
}

cv::Mat BuildXFeatInputImage(const cv::Mat &gray, float &scaleX, float &scaleY)
{
    scaleX = 1.0f;
    scaleY = 1.0f;
    if (gray.empty()) {
        return gray;
    }

    const int srcWidth = gray.cols;
    const int srcHeight = gray.rows;
    const float widthScale = static_cast<float>(kXFeatTargetMaxWidth) / static_cast<float>(std::max(1, srcWidth));
    const float heightScale = static_cast<float>(kXFeatTargetMaxHeight) / static_cast<float>(std::max(1, srcHeight));
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

bool IsXFeatTrackingStateSafe(int trackingState)
{
    return trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::OK_KLT;
}

} // namespace

OrbSlam3Engine::OrbSlam3Engine(std::unique_ptr<ORB_SLAM3::System> system, OrbInputMode inputMode, bool useImu)
    : m_system(std::move(system)), m_inputMode(inputMode), m_useImu(useImu)
{
}

bool OrbSlam3Engine::Start() { return static_cast<bool>(m_system); }

void OrbSlam3Engine::SetOperationMode(core::domain::SlamOperationMode mode)
{
    if (!m_system || m_operationMode == mode) {
        return;
    }

    const bool localizationOnly = mode == core::domain::SlamOperationMode::Localization ||
                                  mode == core::domain::SlamOperationMode::Relocalization ||
                                  mode == core::domain::SlamOperationMode::TrackingOnly;

    if (localizationOnly) {
        m_system->ActivateLocalizationMode();
    } else {
        m_system->DeactivateLocalizationMode();
    }
    m_operationMode = mode;
}

void OrbSlam3Engine::SetFeatureFrontend(FeatureFrontend frontend) { m_featureFrontend = frontend; }

void OrbSlam3Engine::SetXFeatFrontendClient(XFeatFrontendClient *client) { m_xfeatFrontendClient = client; }

void OrbSlam3Engine::Stop()
{
    if (m_system) {
        m_system->Shutdown();
    }
}

std::vector<cv::KeyPoint> OrbSlam3Engine::ToKeyPoints(const std::vector<cv::Point2f> &points)
{
    std::vector<cv::KeyPoint> out;
    out.reserve(points.size());
    for (const cv::Point2f &pt : points) {
        cv::KeyPoint kp;
        kp.pt = pt;
        kp.size = 8.0f;
        kp.angle = -1.0f;
        kp.octave = 0;
        kp.response = 1.0f;
        out.push_back(kp);
    }
    return out;
}

bool OrbSlam3Engine::BuildMonoExternalData(const cv::Mat &gray, ORB_SLAM3::ExternalMonoFrameData &outData) const
{
    outData = ORB_SLAM3::ExternalMonoFrameData{};
    m_lastXFeatRawLeftCount = 0;
    m_lastXFeatRawRightCount = 0;
    m_lastXFeatMatchedStereoCount = 0;
    m_lastXFeatInjectedLeftCount = 0;
    m_lastXFeatInjectedRightCount = 0;
    if (m_featureFrontend != FeatureFrontend::XFeat || m_xfeatFrontendClient == nullptr ||
        !m_xfeatFrontendClient->Running()) {
        return false;
    }

    XFeatFeatureSet features;
    std::string err;
    float scaleX = 1.0f;
    float scaleY = 1.0f;
    const cv::Mat xfeatInput = BuildXFeatInputImage(gray, scaleX, scaleY);
    if (!m_xfeatFrontendClient->DetectAndCompute(xfeatInput, features, &err) || features.descriptors.empty()) {
        return false;
    }
    RemapKeypointsToSource(features.keypoints, scaleX, scaleY);
    m_lastXFeatRawLeftCount = static_cast<int>(features.keypoints.size());

    outData.keypoints = ToKeyPoints(features.keypoints);
    outData.descriptors = std::move(features.descriptors);
    m_lastXFeatInjectedLeftCount = static_cast<int>(outData.keypoints.size());
    return !outData.keypoints.empty() && !outData.descriptors.empty();
}

bool OrbSlam3Engine::BuildStereoExternalData(const cv::Mat &leftGray, const cv::Mat &rightGray,
                                             ORB_SLAM3::ExternalStereoFrameData &outData) const
{
    outData = ORB_SLAM3::ExternalStereoFrameData{};
    m_lastXFeatRawLeftCount = 0;
    m_lastXFeatRawRightCount = 0;
    m_lastXFeatMatchedStereoCount = 0;
    m_lastXFeatInjectedLeftCount = 0;
    m_lastXFeatInjectedRightCount = 0;
    if (m_featureFrontend != FeatureFrontend::XFeat || m_xfeatFrontendClient == nullptr ||
        !m_xfeatFrontendClient->Running()) {
        return false;
    }

    XFeatFeatureSet leftFeatures;
    XFeatFeatureSet rightFeatures;
    std::string err;
    float leftScaleX = 1.0f;
    float leftScaleY = 1.0f;
    float rightScaleX = 1.0f;
    float rightScaleY = 1.0f;
    const cv::Mat leftXFeatInput = BuildXFeatInputImage(leftGray, leftScaleX, leftScaleY);
    const cv::Mat rightXFeatInput = BuildXFeatInputImage(rightGray, rightScaleX, rightScaleY);
    if (!m_xfeatFrontendClient->DetectAndCompute(leftXFeatInput, leftFeatures, &err) ||
        !m_xfeatFrontendClient->DetectAndCompute(rightXFeatInput, rightFeatures, &err) || leftFeatures.descriptors.empty() ||
        rightFeatures.descriptors.empty()) {
        return false;
    }
    RemapKeypointsToSource(leftFeatures.keypoints, leftScaleX, leftScaleY);
    RemapKeypointsToSource(rightFeatures.keypoints, rightScaleX, rightScaleY);
    m_lastXFeatRawLeftCount = static_cast<int>(leftFeatures.keypoints.size());
    m_lastXFeatRawRightCount = static_cast<int>(rightFeatures.keypoints.size());

    const std::vector<StereoMatchPair> matches = MatchStereoPairs(leftFeatures, rightFeatures);
    if (matches.empty()) {
        return false;
    }
    m_lastXFeatMatchedStereoCount = static_cast<int>(matches.size());

    outData.leftKeypoints.reserve(matches.size());
    outData.rightKeypoints.reserve(matches.size());
    outData.leftDescriptors = cv::Mat(static_cast<int>(matches.size()), leftFeatures.descriptors.cols, CV_32F);
    outData.rightDescriptors = cv::Mat(static_cast<int>(matches.size()), rightFeatures.descriptors.cols, CV_32F);
    for (size_t i = 0; i < matches.size(); ++i) {
        const StereoMatchPair &match = matches[i];
        outData.leftKeypoints.push_back(MakeKeyPoint(leftFeatures.keypoints[static_cast<size_t>(match.leftIndex)]));
        outData.rightKeypoints.push_back(MakeKeyPoint(rightFeatures.keypoints[static_cast<size_t>(match.rightIndex)]));
        leftFeatures.descriptors.row(match.leftIndex).copyTo(outData.leftDescriptors.row(static_cast<int>(i)));
        rightFeatures.descriptors.row(match.rightIndex).copyTo(outData.rightDescriptors.row(static_cast<int>(i)));
    }
    outData.matchedStereoPairs = true;
    m_lastXFeatInjectedLeftCount = static_cast<int>(outData.leftKeypoints.size());
    m_lastXFeatInjectedRightCount = static_cast<int>(outData.rightKeypoints.size());
    return true;
}

core::ports::SlamOutput OrbSlam3Engine::Process(const core::ports::SlamInputBatch &input, bool extractFeatures,
                                                bool extractPointCloud)
{
    core::ports::SlamOutput out{};
    if (!m_system) {
        return out;
    }
    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    m_lastXFeatRawLeftCount = 0;
    m_lastXFeatRawRightCount = 0;
    m_lastXFeatMatchedStereoCount = 0;
    m_lastXFeatInjectedLeftCount = 0;
    m_lastXFeatInjectedRightCount = 0;

    Sophus::SE3f tcw;
    const bool monoMode = (m_inputMode != OrbInputMode::Stereo);
    const cv::Mat &monoImage =
        (m_inputMode == OrbInputMode::MonoRight) ? input.stereo.right.gray : input.stereo.left.gray;
    const int previousTrackingState = m_system->GetTrackingState();
    const bool tryXFeat =
        m_featureFrontend == FeatureFrontend::XFeat && m_xfeatFrontendClient != nullptr &&
        m_xfeatFrontendClient->Running() && m_system->CanUseExternalFeatureInjection() &&
        IsXFeatTrackingStateSafe(previousTrackingState);

    ORB_SLAM3::ExternalMonoFrameData monoExternal;
    ORB_SLAM3::ExternalStereoFrameData stereoExternal;
    const bool haveMonoExternal = monoMode && tryXFeat && BuildMonoExternalData(monoImage, monoExternal);
    const bool haveStereoExternal =
        !monoMode && tryXFeat && BuildStereoExternalData(input.stereo.left.gray, input.stereo.right.gray, stereoExternal);
    out.usedXFeatFrontend = haveMonoExternal || haveStereoExternal;
    out.xfeatRawLeftCount = m_lastXFeatRawLeftCount;
    out.xfeatRawRightCount = m_lastXFeatRawRightCount;
    out.xfeatMatchedStereoCount = m_lastXFeatMatchedStereoCount;
    out.xfeatInjectedLeftCount = m_lastXFeatInjectedLeftCount;
    out.xfeatInjectedRightCount = m_lastXFeatInjectedRightCount;

    if (m_useImu) {
        if (monoMode) {
            tcw = haveMonoExternal ? m_system->TrackMonocularWithFeatures(monoImage, monoExternal, input.frameTimeSec, input.imu)
                                   : m_system->TrackMonocular(monoImage, input.frameTimeSec, input.imu);
        } else {
            tcw = haveStereoExternal
                      ? m_system->TrackStereoWithFeatures(input.stereo.left.gray, input.stereo.right.gray, stereoExternal,
                                                          input.frameTimeSec, input.imu)
                      : m_system->TrackStereo(input.stereo.left.gray, input.stereo.right.gray, input.frameTimeSec,
                                              input.imu);
        }
    } else {
        if (monoMode) {
            tcw = haveMonoExternal ? m_system->TrackMonocularWithFeatures(monoImage, monoExternal, input.frameTimeSec)
                                   : m_system->TrackMonocular(monoImage, input.frameTimeSec);
        } else {
            tcw = haveStereoExternal
                      ? m_system->TrackStereoWithFeatures(input.stereo.left.gray, input.stereo.right.gray, stereoExternal,
                                                          input.frameTimeSec)
                      : m_system->TrackStereo(input.stereo.left.gray, input.stereo.right.gray, input.frameTimeSec);
        }
    }

    out.trackingState = m_system->GetTrackingState();
    out.mapId = m_system->GetCurrentMapId();

    const Sophus::SE3f twc = tcw.inverse();
    const Eigen::Vector3f t = twc.translation();
    const Eigen::Quaternionf q(twc.so3().unit_quaternion());
    out.poseValid = std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z()) && std::isfinite(q.w()) &&
                    std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z());
    out.pose.valid = out.poseValid;
    out.pose.x = t.x();
    out.pose.y = t.y();
    out.pose.z = t.z();
    out.pose.qw = q.w();
    out.pose.qx = q.x();
    out.pose.qy = q.y();
    out.pose.qz = q.z();

    if (!extractFeatures && !extractPointCloud) {
        return out;
    }

    const int leftWidth = monoMode ? monoImage.cols : input.stereo.left.gray.cols;
    const int leftHeight = monoMode ? monoImage.rows : input.stereo.left.gray.rows;
    ORB_SLAM3::TrackedVisualData visual =
        m_system->ExtractTrackedVisualData(leftWidth, leftHeight, monoMode ? 0 : input.stereo.right.gray.cols,
                                           monoMode ? 0 : input.stereo.right.gray.rows, extractPointCloud, 120);
    if (extractFeatures) {
        if (m_inputMode == OrbInputMode::MonoRight) {
            out.rightFeatures = std::move(visual.leftFeatures);
        } else {
            out.leftFeatures = std::move(visual.leftFeatures);
            out.rightFeatures = std::move(visual.rightFeatures);
        }
    }
    if (extractPointCloud) {
        out.pointCloudXyz = std::move(visual.pointCloudXyz);
    }
    return out;
}

} // namespace smartdrone::adapters::slam
