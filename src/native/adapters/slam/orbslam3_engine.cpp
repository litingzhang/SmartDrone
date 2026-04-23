#include "adapters/slam/orbslam3_engine.h"

#include <algorithm>
#include <chrono>
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

constexpr float kStereoMaxEpipolarDeltaPx = 1.5f;
constexpr float kStereoMinDisparityPx = 0.75f;
constexpr float kStereoMaxDisparityPx = 240.0f;
constexpr float kStereoRatioTest = 0.92f;
constexpr float kStereoMinZnccScore = 0.10f;
constexpr int kStereoPatchRadiusPx = 3;
constexpr int kStereoGridCols = 8;
constexpr int kStereoGridRows = 6;
constexpr int kStereoMaxPairsPerCell = 10;

struct StereoMatchPair {
    int leftIndex{-1};
    int rightIndex{-1};
    int distance{std::numeric_limits<int>::max()};
    float zncc{-1.0f};
    float disparity{0.0f};
};

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

bool ComputePatchZncc(const cv::Mat &leftGray32f, const cv::Point2f &leftPt, const cv::Mat &rightGray32f,
                      const cv::Point2f &rightPt, float &score)
{
    score = -1.0f;
    if (leftGray32f.empty() || rightGray32f.empty()) {
        return false;
    }

    const int patchSize = 2 * kStereoPatchRadiusPx + 1;
    if (leftPt.x < static_cast<float>(kStereoPatchRadiusPx) ||
        leftPt.y < static_cast<float>(kStereoPatchRadiusPx) ||
        rightPt.x < static_cast<float>(kStereoPatchRadiusPx) ||
        rightPt.y < static_cast<float>(kStereoPatchRadiusPx) ||
        leftPt.x >= static_cast<float>(leftGray32f.cols - kStereoPatchRadiusPx) ||
        leftPt.y >= static_cast<float>(leftGray32f.rows - kStereoPatchRadiusPx) ||
        rightPt.x >= static_cast<float>(rightGray32f.cols - kStereoPatchRadiusPx) ||
        rightPt.y >= static_cast<float>(rightGray32f.rows - kStereoPatchRadiusPx)) {
        return false;
    }

    cv::Mat leftPatch;
    cv::Mat rightPatch;
    cv::getRectSubPix(leftGray32f, cv::Size(patchSize, patchSize), leftPt, leftPatch);
    cv::getRectSubPix(rightGray32f, cv::Size(patchSize, patchSize), rightPt, rightPatch);
    if (leftPatch.empty() || rightPatch.empty()) {
        return false;
    }

    cv::Scalar leftMean;
    cv::Scalar leftStd;
    cv::Scalar rightMean;
    cv::Scalar rightStd;
    cv::meanStdDev(leftPatch, leftMean, leftStd);
    cv::meanStdDev(rightPatch, rightMean, rightStd);
    if (leftStd[0] < 1e-3 || rightStd[0] < 1e-3) {
        return false;
    }

    cv::Mat leftNorm = leftPatch - leftMean[0];
    cv::Mat rightNorm = rightPatch - rightMean[0];
    const double denom = static_cast<double>(leftNorm.total()) * leftStd[0] * rightStd[0];
    if (denom <= 1e-9) {
        return false;
    }
    score = static_cast<float>(leftNorm.dot(rightNorm) / denom);
    return std::isfinite(score);
}

bool IsBetterRightCandidate(int candidateDist, float candidateZncc, int currentDist, float currentZncc)
{
    if (currentDist == std::numeric_limits<int>::max()) {
        return true;
    }
    if (candidateDist != currentDist) {
        return candidateDist < currentDist;
    }
    return candidateZncc > currentZncc;
}

std::vector<StereoMatchPair> SelectGridBalancedPairs(const std::vector<StereoMatchPair> &matches,
                                                     const std::vector<cv::Point2f> &leftKeypoints, int imageWidth,
                                                     int imageHeight)
{
    if (matches.empty() || imageWidth <= 0 || imageHeight <= 0) {
        return matches;
    }

    const int cellWidth = std::max(1, (imageWidth + kStereoGridCols - 1) / kStereoGridCols);
    const int cellHeight = std::max(1, (imageHeight + kStereoGridRows - 1) / kStereoGridRows);
    std::vector<int> cellCounts(static_cast<size_t>(kStereoGridCols * kStereoGridRows), 0);
    std::vector<StereoMatchPair> selected;
    selected.reserve(matches.size());

    for (const StereoMatchPair &match : matches) {
        const cv::Point2f &pt = leftKeypoints[static_cast<size_t>(match.leftIndex)];
        const int col = std::clamp(static_cast<int>(pt.x) / cellWidth, 0, kStereoGridCols - 1);
        const int row = std::clamp(static_cast<int>(pt.y) / cellHeight, 0, kStereoGridRows - 1);
        const size_t cellIndex = static_cast<size_t>(row * kStereoGridCols + col);
        if (cellCounts[cellIndex] >= kStereoMaxPairsPerCell) {
            continue;
        }
        ++cellCounts[cellIndex];
        selected.push_back(match);
    }
    return selected;
}

std::vector<StereoMatchPair> MatchStereoPairs(const XFeatFeatureSet &left, const XFeatFeatureSet &right,
                                              const cv::Mat &leftGray, const cv::Mat &rightGray)
{
    std::vector<StereoMatchPair> matches;
    if (left.descriptors.empty() || right.descriptors.empty() || left.keypoints.empty() || right.keypoints.empty()) {
        return matches;
    }

    cv::Mat leftGray32f;
    cv::Mat rightGray32f;
    leftGray.convertTo(leftGray32f, CV_32F);
    rightGray.convertTo(rightGray32f, CV_32F);

    std::vector<StereoMatchPair> bestForLeft(static_cast<size_t>(left.descriptors.rows));
    for (StereoMatchPair &pair : bestForLeft) {
        pair.distance = std::numeric_limits<int>::max();
    }
    std::vector<int> secondBestDist(static_cast<size_t>(left.descriptors.rows), std::numeric_limits<int>::max());
    std::vector<int> bestLeftForRight(static_cast<size_t>(right.descriptors.rows), -1);
    std::vector<int> bestLeftDist(static_cast<size_t>(right.descriptors.rows), std::numeric_limits<int>::max());
    std::vector<float> bestLeftZncc(static_cast<size_t>(right.descriptors.rows), -1.0f);

    for (int li = 0; li < left.descriptors.rows; ++li) {
        const cv::Point2f &leftPt = left.keypoints[static_cast<size_t>(li)];
        int bestRi = -1;
        int bestDist = std::numeric_limits<int>::max();
        int secondDist = std::numeric_limits<int>::max();
        float bestZncc = -1.0f;
        float bestDisparity = 0.0f;

        for (int ri = 0; ri < right.descriptors.rows; ++ri) {
            const cv::Point2f &rightPt = right.keypoints[static_cast<size_t>(ri)];
            const float yDelta = std::fabs(leftPt.y - rightPt.y);
            const float disparity = leftPt.x - rightPt.x;
            if (yDelta > kStereoMaxEpipolarDeltaPx || disparity < kStereoMinDisparityPx ||
                disparity > kStereoMaxDisparityPx) {
                continue;
            }

            const int dist =
                ORB_SLAM3::ORBmatcher::DescriptorDistance(left.descriptors.row(li), right.descriptors.row(ri));
            if (dist < bestDist) {
                secondDist = bestDist;
                bestDist = dist;
                bestRi = ri;
                bestDisparity = disparity;
            } else if (dist < secondDist) {
                secondDist = dist;
            }
        }

        if (bestRi < 0) {
            continue;
        }
        if (secondDist != std::numeric_limits<int>::max() &&
            static_cast<float>(bestDist) >= kStereoRatioTest * static_cast<float>(secondDist)) {
            continue;
        }

        float zncc = -1.0f;
        if (!ComputePatchZncc(leftGray32f, leftPt, rightGray32f, right.keypoints[static_cast<size_t>(bestRi)], zncc) ||
            zncc < kStereoMinZnccScore) {
            continue;
        }

        bestZncc = zncc;
        bestForLeft[static_cast<size_t>(li)] = StereoMatchPair{li, bestRi, bestDist, bestZncc, bestDisparity};
        secondBestDist[static_cast<size_t>(li)] = secondDist;

        if (IsBetterRightCandidate(bestDist, bestZncc, bestLeftDist[static_cast<size_t>(bestRi)],
                                   bestLeftZncc[static_cast<size_t>(bestRi)])) {
            bestLeftDist[static_cast<size_t>(bestRi)] = bestDist;
            bestLeftZncc[static_cast<size_t>(bestRi)] = bestZncc;
            bestLeftForRight[static_cast<size_t>(bestRi)] = li;
        }
    }

    matches.reserve(static_cast<size_t>(std::min(left.descriptors.rows, right.descriptors.rows)));
    for (size_t li = 0; li < bestForLeft.size(); ++li) {
        const StereoMatchPair &pair = bestForLeft[li];
        if (pair.rightIndex < 0) {
            continue;
        }
        if (bestLeftForRight[static_cast<size_t>(pair.rightIndex)] != pair.leftIndex) {
            continue;
        }
        matches.push_back(pair);
    }

    std::sort(matches.begin(), matches.end(), [](const StereoMatchPair &a, const StereoMatchPair &b) {
        if (a.distance != b.distance) {
            return a.distance < b.distance;
        }
        if (a.zncc != b.zncc) {
            return a.zncc > b.zncc;
        }
        return a.disparity < b.disparity;
    });
    return SelectGridBalancedPairs(matches, left.keypoints, leftGray.cols, leftGray.rows);
}

std::vector<cv::Point2f> ToPointList(const std::vector<cv::KeyPoint> &keypoints)
{
    std::vector<cv::Point2f> points;
    points.reserve(keypoints.size());
    for (const cv::KeyPoint &kp : keypoints) {
        points.push_back(kp.pt);
    }
    return points;
}

cv::Mat BuildXFeatInputImage(const cv::Mat &gray, int maxWidth, int maxHeight, float &scaleX, float &scaleY)
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

bool IsXFeatTrackingStateSafe(int trackingState)
{
    return trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::OK_KLT;
}

std::string DescribeTrackingState(int trackingState)
{
    switch (trackingState) {
    case ORB_SLAM3::Tracking::SYSTEM_NOT_READY:
        return "system_not_ready";
    case ORB_SLAM3::Tracking::NO_IMAGES_YET:
        return "no_images_yet";
    case ORB_SLAM3::Tracking::NOT_INITIALIZED:
        return "not_initialized";
    case ORB_SLAM3::Tracking::OK:
        return "ok";
    case ORB_SLAM3::Tracking::RECENTLY_LOST:
        return "recently_lost";
    case ORB_SLAM3::Tracking::LOST:
        return "lost";
    case ORB_SLAM3::Tracking::OK_KLT:
        return "ok_klt";
    default:
        return "unknown";
    }
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

void OrbSlam3Engine::SetXFeatInputSizeLimit(int maxWidth, int maxHeight)
{
    m_xfeatInputMaxWidth = std::max(0, maxWidth);
    m_xfeatInputMaxHeight = std::max(0, maxHeight);
}

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
    const auto totalStartTp = std::chrono::steady_clock::now();
    m_lastXFeatRawLeftCount = 0;
    m_lastXFeatRawRightCount = 0;
    m_lastXFeatMatchedStereoCount = 0;
    m_lastXFeatInjectedLeftCount = 0;
    m_lastXFeatInjectedRightCount = 0;
    m_lastXFeatPrepareMs = 0.0;
    m_lastXFeatWorkerWriteMs = 0.0;
    m_lastXFeatWorkerReadMs = 0.0;
    m_lastXFeatWorkerTotalMs = 0.0;
    m_lastXFeatStereoMatchMs = 0.0;
    m_lastXFeatTotalMs = 0.0;
    m_lastXFeatImageCount = 0;
    m_lastXFeatPayloadBytes = 0;
    if (m_featureFrontend != FeatureFrontend::XFeat || m_xfeatFrontendClient == nullptr ||
        !m_xfeatFrontendClient->Running()) {
        return false;
    }

    XFeatFeatureSet features;
    std::string err;
    float scaleX = 1.0f;
    float scaleY = 1.0f;
    const cv::Mat xfeatInput = BuildXFeatInputImage(gray, m_xfeatInputMaxWidth, m_xfeatInputMaxHeight, scaleX, scaleY);
    if (!m_xfeatFrontendClient->DetectAndCompute(xfeatInput, features, &err) || features.descriptors.empty()) {
        return false;
    }
    const XFeatFrontendClient::Stats stats = m_xfeatFrontendClient->LastStats();
    m_lastXFeatPrepareMs = stats.prepareMs;
    m_lastXFeatWorkerWriteMs = stats.writeMs;
    m_lastXFeatWorkerReadMs = stats.readMs;
    m_lastXFeatWorkerTotalMs = stats.totalMs;
    m_lastXFeatImageCount = stats.imageCount;
    m_lastXFeatPayloadBytes = stats.payloadBytes;
    RemapKeypointsToSource(features.keypoints, scaleX, scaleY);
    m_lastXFeatRawLeftCount = static_cast<int>(features.keypoints.size());

    outData.keypoints = ToKeyPoints(features.keypoints);
    outData.descriptors = std::move(features.descriptors);
    m_lastXFeatInjectedLeftCount = static_cast<int>(outData.keypoints.size());
    m_lastXFeatTotalMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - totalStartTp).count();
    return !outData.keypoints.empty() && !outData.descriptors.empty();
}

bool OrbSlam3Engine::BuildStereoExternalData(const cv::Mat &leftGray, const cv::Mat &rightGray,
                                             ORB_SLAM3::ExternalStereoFrameData &outData,
                                             std::vector<cv::Point2f> *leftRawPoints,
                                             std::vector<cv::Point2f> *rightRawPoints) const
{
    outData = ORB_SLAM3::ExternalStereoFrameData{};
    const auto totalStartTp = std::chrono::steady_clock::now();
    m_lastXFeatRawLeftCount = 0;
    m_lastXFeatRawRightCount = 0;
    m_lastXFeatMatchedStereoCount = 0;
    m_lastXFeatInjectedLeftCount = 0;
    m_lastXFeatInjectedRightCount = 0;
    m_lastXFeatPrepareMs = 0.0;
    m_lastXFeatWorkerWriteMs = 0.0;
    m_lastXFeatWorkerReadMs = 0.0;
    m_lastXFeatWorkerTotalMs = 0.0;
    m_lastXFeatStereoMatchMs = 0.0;
    m_lastXFeatTotalMs = 0.0;
    m_lastXFeatImageCount = 0;
    m_lastXFeatPayloadBytes = 0;
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
    const cv::Mat leftXFeatInput =
        BuildXFeatInputImage(leftGray, m_xfeatInputMaxWidth, m_xfeatInputMaxHeight, leftScaleX, leftScaleY);
    const cv::Mat rightXFeatInput =
        BuildXFeatInputImage(rightGray, m_xfeatInputMaxWidth, m_xfeatInputMaxHeight, rightScaleX, rightScaleY);
    if (!m_xfeatFrontendClient->DetectAndComputeStereo(leftXFeatInput, rightXFeatInput, leftFeatures, rightFeatures,
                                                       &err) ||
        leftFeatures.descriptors.empty() || rightFeatures.descriptors.empty()) {
        return false;
    }
    const XFeatFrontendClient::Stats stats = m_xfeatFrontendClient->LastStats();
    m_lastXFeatPrepareMs = stats.prepareMs;
    m_lastXFeatWorkerWriteMs = stats.writeMs;
    m_lastXFeatWorkerReadMs = stats.readMs;
    m_lastXFeatWorkerTotalMs = stats.totalMs;
    m_lastXFeatImageCount = stats.imageCount;
    m_lastXFeatPayloadBytes = stats.payloadBytes;
    RemapKeypointsToSource(leftFeatures.keypoints, leftScaleX, leftScaleY);
    RemapKeypointsToSource(rightFeatures.keypoints, rightScaleX, rightScaleY);
    if (leftRawPoints != nullptr) {
        *leftRawPoints = leftFeatures.keypoints;
    }
    if (rightRawPoints != nullptr) {
        *rightRawPoints = rightFeatures.keypoints;
    }
    m_lastXFeatRawLeftCount = static_cast<int>(leftFeatures.keypoints.size());
    m_lastXFeatRawRightCount = static_cast<int>(rightFeatures.keypoints.size());

    const auto matchStartTp = std::chrono::steady_clock::now();
    const std::vector<StereoMatchPair> matches = MatchStereoPairs(leftFeatures, rightFeatures, leftGray, rightGray);
    m_lastXFeatStereoMatchMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - matchStartTp).count();
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
    m_lastXFeatTotalMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - totalStartTp).count();
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
    m_lastXFeatPrepareMs = 0.0;
    m_lastXFeatWorkerWriteMs = 0.0;
    m_lastXFeatWorkerReadMs = 0.0;
    m_lastXFeatWorkerTotalMs = 0.0;
    m_lastXFeatStereoMatchMs = 0.0;
    m_lastXFeatTotalMs = 0.0;
    m_lastXFeatImageCount = 0;
    m_lastXFeatPayloadBytes = 0;

    Sophus::SE3f tcw;
    const bool monoMode = (m_inputMode != OrbInputMode::Stereo);
    const cv::Mat &monoImage =
        (m_inputMode == OrbInputMode::MonoRight) ? input.stereo.right.gray : input.stereo.left.gray;
    cv::Mat preparedLeftImage;
    cv::Mat preparedRightImage;
    const int previousTrackingState = m_system->GetTrackingState();
    std::string xfeatStatusReason;
    bool tryXFeat = false;
    if (m_featureFrontend != FeatureFrontend::XFeat) {
        xfeatStatusReason = "frontend_not_xfeat";
    } else if (m_xfeatFrontendClient == nullptr) {
        xfeatStatusReason = "worker_not_configured";
    } else if (!m_xfeatFrontendClient->Running()) {
        xfeatStatusReason = "worker_not_running";
    } else if (!IsXFeatTrackingStateSafe(previousTrackingState)) {
        xfeatStatusReason = "tracking_not_ok:" + DescribeTrackingState(previousTrackingState);
    } else if (monoMode && !m_system->CanUseExternalFeatureInjection()) {
        xfeatStatusReason = "resize_enabled";
    } else if (!monoMode &&
               !m_system->PrepareStereoImagesForTracking(input.stereo.left.gray, input.stereo.right.gray,
                                                        preparedLeftImage, preparedRightImage)) {
        xfeatStatusReason = "prepare_tracking_images_failed";
    } else {
        tryXFeat = true;
        xfeatStatusReason = "enabled";
    }
    if (m_featureFrontend == FeatureFrontend::XFeat && xfeatStatusReason != m_lastXFeatStatusReason) {
        std::cerr << "[slam] xfeat_runtime_status=" << xfeatStatusReason << "\n";
        m_lastXFeatStatusReason = xfeatStatusReason;
    }

    ORB_SLAM3::ExternalMonoFrameData monoExternal;
    ORB_SLAM3::ExternalStereoFrameData stereoExternal;
    std::vector<cv::Point2f> stereoLeftRawPoints;
    std::vector<cv::Point2f> stereoRightRawPoints;
    std::vector<cv::Point2f> stereoStreamLeftPoints;
    std::vector<cv::Point2f> stereoStreamRightPoints;
    const bool haveMonoExternal = monoMode && tryXFeat && BuildMonoExternalData(monoImage, monoExternal);
    const bool haveStereoExternal = !monoMode && tryXFeat &&
                                    BuildStereoExternalData(preparedLeftImage, preparedRightImage,
                                                            stereoExternal, &stereoLeftRawPoints, &stereoRightRawPoints);
    bool haveStereoStreamOnlyXFeat = false;
    if (!monoMode && extractFeatures && m_featureFrontend == FeatureFrontend::XFeat && !haveStereoExternal &&
        m_xfeatFrontendClient != nullptr && m_xfeatFrontendClient->Running()) {
        ORB_SLAM3::ExternalStereoFrameData streamOnlyExternal;
        (void)BuildStereoExternalData(input.stereo.left.gray, input.stereo.right.gray, streamOnlyExternal,
                                      &stereoStreamLeftPoints, &stereoStreamRightPoints);
        haveStereoStreamOnlyXFeat = !stereoStreamLeftPoints.empty() || !stereoStreamRightPoints.empty();
    }
    const bool haveXFeatFeaturesForOutput = haveMonoExternal || haveStereoExternal || haveStereoStreamOnlyXFeat;
    out.usedXFeatFrontend = haveMonoExternal || haveStereoExternal;
    out.xfeatRawLeftCount = m_lastXFeatRawLeftCount;
    out.xfeatRawRightCount = m_lastXFeatRawRightCount;
    out.xfeatMatchedStereoCount = m_lastXFeatMatchedStereoCount;
    out.xfeatInjectedLeftCount = m_lastXFeatInjectedLeftCount;
    out.xfeatInjectedRightCount = m_lastXFeatInjectedRightCount;
    out.xfeatPrepareMs = m_lastXFeatPrepareMs;
    out.xfeatWorkerWriteMs = m_lastXFeatWorkerWriteMs;
    out.xfeatWorkerReadMs = m_lastXFeatWorkerReadMs;
    out.xfeatWorkerTotalMs = m_lastXFeatWorkerTotalMs;
    out.xfeatStereoMatchMs = m_lastXFeatStereoMatchMs;
    out.xfeatTotalMs = m_lastXFeatTotalMs;
    out.xfeatImageCount = m_lastXFeatImageCount;
    out.xfeatPayloadBytes = m_lastXFeatPayloadBytes;

    if (m_useImu) {
        if (monoMode) {
            tcw = haveMonoExternal ? m_system->TrackMonocularWithFeatures(monoImage, monoExternal, input.frameTimeSec, input.imu)
                                   : m_system->TrackMonocular(monoImage, input.frameTimeSec, input.imu);
        } else {
            tcw = haveStereoExternal
                      ? m_system->TrackStereoPreparedWithFeatures(preparedLeftImage, preparedRightImage, stereoExternal,
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
                      ? m_system->TrackStereoPreparedWithFeatures(preparedLeftImage, preparedRightImage, stereoExternal,
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

    if (extractFeatures && haveXFeatFeaturesForOutput) {
        if (monoMode) {
            if (m_inputMode == OrbInputMode::MonoRight) {
                out.rightFeatures = ToPointList(monoExternal.keypoints);
            } else {
                out.leftFeatures = ToPointList(monoExternal.keypoints);
            }
        } else if (haveStereoExternal) {
            out.leftFeatures = std::move(stereoLeftRawPoints);
            out.rightFeatures = std::move(stereoRightRawPoints);
        } else {
            out.leftFeatures = std::move(stereoStreamLeftPoints);
            out.rightFeatures = std::move(stereoStreamRightPoints);
        }
    }

    const bool needVisualExtraction = extractPointCloud || (extractFeatures && !haveXFeatFeaturesForOutput);
    if (!needVisualExtraction) {
        return out;
    }

    const int leftWidth = monoMode ? monoImage.cols : input.stereo.left.gray.cols;
    const int leftHeight = monoMode ? monoImage.rows : input.stereo.left.gray.rows;
    ORB_SLAM3::TrackedVisualData visual =
        m_system->ExtractTrackedVisualData(leftWidth, leftHeight, monoMode ? 0 : input.stereo.right.gray.cols,
                                           monoMode ? 0 : input.stereo.right.gray.rows, extractPointCloud, 120);
    if (extractFeatures && !out.usedXFeatFrontend) {
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
