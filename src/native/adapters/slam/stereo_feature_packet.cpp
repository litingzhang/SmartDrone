#include "adapters/slam/stereo_feature_packet.h"

#include <algorithm>
#include <chrono>
#include <climits>
#include <cstring>

#include "adapters/slam/descriptor_geometry.h"
#include "adapters/slam/slam_env.h"
#include "adapters/slam/stereo_geometry.h"

namespace SmartDrone::adapters::slam {

namespace {

constexpr size_t kStereoFeatureMaxLeftFeatures = 1200;
constexpr size_t kStereoFeatureMaxLeftFeaturesLimit = 2500;
using core::ports::IVisualDescriptorProvider;
using core::ports::StereoFeatureObservationPacket;
using core::ports::StereoFeaturePacket;
using core::ports::StereoFeaturePacketBuildInput;
using core::ports::StereoMatchPair;
using core::ports::VisualFeatureSet;

struct OrbStereoAugmentCandidate {
    int leftIndex{-1};
    int rightIndex{-1};
    int distance{INT_MAX};
    float zncc{-1.0f};
    float disparity{0.0f};
};

struct OrbStereoAugmentOptions {
    int maxHamming{60};
    int maxSecondBest{80};
    float ratio{0.85f};
    float minZncc{kStereoMinZnccScore};
};

struct OrbStereoAugmentFeatures {
    std::vector<cv::KeyPoint> leftKeypoints;
    std::vector<cv::KeyPoint> rightKeypoints;
    cv::Mat leftDescriptors;
    cv::Mat rightDescriptors;
};

struct OrbStereoAugmentMatchState {
    std::vector<int> bestLeftForRight;
    std::vector<int> bestRightDistance;
};

struct OrbStereoAugmentCandidateRequest {
    const IVisualDescriptorProvider *leftProvider{nullptr};
    const OrbStereoAugmentFeatures *features{nullptr};
    const OrbStereoAugmentOptions *options{nullptr};
    const cv::Mat *rightGray{nullptr};
    const cv::Mat *leftGray32f{nullptr};
    const cv::Mat *rightGray32f{nullptr};
    size_t leftIndex{0};
};

uint64_t HashFloatValue(uint64_t hash, float value)
{
    uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "unexpected float size");
    std::memcpy(&bits, &value, sizeof(bits));
    hash ^= static_cast<uint64_t>(bits);
    hash *= 1099511628211ULL;
    return hash;
}

uint64_t HashIntValue(uint64_t hash, int value)
{
    hash ^= static_cast<uint64_t>(static_cast<uint32_t>(value));
    hash *= 1099511628211ULL;
    return hash;
}

uint64_t HashMatSample(uint64_t hash, const cv::Mat &mat)
{
    hash = HashIntValue(hash, mat.rows);
    hash = HashIntValue(hash, mat.cols);
    hash = HashIntValue(hash, mat.type());
    if (mat.empty()) {
        return hash;
    }
    const int rowStride = std::max(1, mat.rows / 16);
    if (mat.type() == CV_32F) {
        const int colStride = std::max(1, mat.cols / 16);
        for (int row = 0; row < mat.rows; row += rowStride) {
            const float *data = mat.ptr<float>(row);
            for (int col = 0; col < mat.cols; col += colStride) {
                hash = HashFloatValue(hash, data[col]);
            }
        }
    } else if (mat.type() == CV_8U) {
        const int colStride = std::max(1, mat.cols / 32);
        for (int row = 0; row < mat.rows; row += rowStride) {
            const uint8_t *data = mat.ptr<uint8_t>(row);
            for (int col = 0; col < mat.cols; col += colStride) {
                hash ^= static_cast<uint64_t>(data[col]);
                hash *= 1099511628211ULL;
            }
        }
    }
    return hash;
}

bool HasInputImages(const StereoFeaturePacketBuildInput &input)
{
    return input.leftPrepared != nullptr && input.rightPrepared != nullptr &&
           !input.leftPrepared->empty() && !input.rightPrepared->empty();
}

bool HasMatchedPoints(const StereoFeaturePacketBuildInput &input)
{
    return input.matchedLeftPoints != nullptr &&
           input.matchedRightPoints != nullptr &&
           !input.matchedLeftPoints->empty() &&
           input.matchedLeftPoints->size() == input.matchedRightPoints->size();
}

std::vector<StereoMatchPair>
BuildAlignedDescriptorMatches(size_t pairedFeatureCount)
{
    std::vector<StereoMatchPair> descriptorMatches;
    descriptorMatches.reserve(pairedFeatureCount);
    for (size_t i = 0; i < pairedFeatureCount; ++i) {
        descriptorMatches.push_back(StereoMatchPair{
            static_cast<int>(i), static_cast<int>(i), 1.0f, 1.0f, 0.0f, 1.0f});
    }
    return descriptorMatches;
}

void CopyPacketFeaturePoints(StereoFeaturePacket &packet)
{
    packet.leftFeaturePoints.clear();
    packet.rightFeaturePoints.clear();
    packet.leftFeaturePoints.reserve(packet.observations.leftKeypoints.size());
    packet.rightFeaturePoints.reserve(packet.observations.rightKeypoints.size());
    for (const cv::KeyPoint &kp : packet.observations.leftKeypoints) {
        packet.leftFeaturePoints.push_back(kp.pt);
    }
    for (const cv::KeyPoint &kp : packet.observations.rightKeypoints) {
        packet.rightFeaturePoints.push_back(kp.pt);
    }
}

bool HasValidFeatureDescriptors(const VisualFeatureSet &features)
{
    return !features.descriptors.empty() &&
           features.descriptors.type() == CV_32F &&
           features.descriptors.rows ==
               static_cast<int>(features.keypoints.size()) &&
           features.descriptors.cols > 0;
}

bool IsStereoMatchInFeatureRange(const StereoMatchPair &match,
                                 const VisualFeatureSet &leftFeatures,
                                 const VisualFeatureSet &rightFeatures)
{
    return match.leftIndex >= 0 && match.rightIndex >= 0 &&
           static_cast<size_t>(match.leftIndex) < leftFeatures.keypoints.size() &&
           static_cast<size_t>(match.rightIndex) < rightFeatures.keypoints.size();
}

bool BuildStereoObservationsFromFeatureMatches(
    const VisualFeatureSet &left, const VisualFeatureSet &right,
    const std::vector<StereoMatchPair> &matches,
    StereoFeatureObservationPacket &outData)
{
    if (!HasValidFeatureDescriptors(left) || !HasValidFeatureDescriptors(right) ||
        matches.empty() || left.descriptors.cols != right.descriptors.cols) {
        return false;
    }

    const int descriptorDim = left.descriptors.cols;
    cv::Mat leftDescriptors(static_cast<int>(matches.size()), descriptorDim,
                            CV_32F);
    cv::Mat rightDescriptors(static_cast<int>(matches.size()), descriptorDim,
                             CV_32F);
    std::vector<cv::KeyPoint> leftKeypoints;
    std::vector<cv::KeyPoint> rightKeypoints;
    leftKeypoints.reserve(matches.size());
    rightKeypoints.reserve(matches.size());

    for (const StereoMatchPair &match : matches) {
        if (match.leftIndex < 0 || match.rightIndex < 0 ||
            static_cast<size_t>(match.leftIndex) >= left.keypoints.size() ||
            static_cast<size_t>(match.rightIndex) >= right.keypoints.size()) {
            continue;
        }
        const int row = static_cast<int>(leftKeypoints.size());
        leftKeypoints.push_back(MakeDescriptorKeyPoint(
            left.keypoints[static_cast<size_t>(match.leftIndex)]));
        rightKeypoints.push_back(MakeDescriptorKeyPoint(
            right.keypoints[static_cast<size_t>(match.rightIndex)]));
        left.descriptors.row(match.leftIndex).copyTo(leftDescriptors.row(row));
        right.descriptors.row(match.rightIndex).copyTo(rightDescriptors.row(row));
    }

    if (leftKeypoints.empty()) {
        return false;
    }

    const int validRows = static_cast<int>(leftKeypoints.size());
    outData.leftKeypoints = std::move(leftKeypoints);
    outData.rightKeypoints = std::move(rightKeypoints);
    outData.leftDescriptors = leftDescriptors.rowRange(0, validRows).clone();
    outData.rightDescriptors = rightDescriptors.rowRange(0, validRows).clone();
    outData.matchedStereoPairs = true;
    return true;
}

bool FinalizeStereoObservationsFromPairs(
    const IVisualDescriptorProvider *leftProvider,
    const IVisualDescriptorProvider *rightProvider, const cv::Mat &leftGray,
    const cv::Mat &rightGray, const std::vector<cv::Point2f> &leftPoints,
    const std::vector<cv::Point2f> &rightPoints,
    StereoFeatureObservationPacket &outData)
{
    if (leftProvider == nullptr || rightProvider == nullptr || leftGray.empty() ||
        rightGray.empty() || leftPoints.empty() ||
        leftPoints.size() != rightPoints.size()) {
        return false;
    }

    std::vector<cv::Point2f> filteredLeft;
    std::vector<cv::Point2f> filteredRight;
    filteredLeft.reserve(leftPoints.size());
    filteredRight.reserve(rightPoints.size());
    for (size_t i = 0; i < leftPoints.size(); ++i) {
        if (!IsPointSafeForDescriptor(leftPoints[i], leftGray) ||
            !IsPointSafeForDescriptor(rightPoints[i], rightGray)) {
            continue;
        }
        filteredLeft.push_back(leftPoints[i]);
        filteredRight.push_back(rightPoints[i]);
    }

    if (filteredLeft.empty() || filteredLeft.size() != filteredRight.size()) {
        return false;
    }

    std::vector<cv::KeyPoint> leftKeypoints;
    std::vector<cv::KeyPoint> rightKeypoints;
    cv::Mat leftDescriptors;
    cv::Mat rightDescriptors;
    if (!leftProvider->ComputeDescriptorsAtPoints(
            leftGray, filteredLeft, leftKeypoints, leftDescriptors) ||
        !rightProvider->ComputeDescriptorsAtPoints(
            rightGray, filteredRight, rightKeypoints, rightDescriptors)) {
        return false;
    }
    if (leftKeypoints.size() != rightKeypoints.size() ||
        leftDescriptors.rows != rightDescriptors.rows ||
        leftDescriptors.rows != static_cast<int>(leftKeypoints.size())) {
        return false;
    }

    outData.leftKeypoints = std::move(leftKeypoints);
    outData.rightKeypoints = std::move(rightKeypoints);
    outData.leftDescriptors = std::move(leftDescriptors);
    outData.rightDescriptors = std::move(rightDescriptors);
    outData.matchedStereoPairs = true;
    outData.leftToRightMatch.resize(outData.leftKeypoints.size());
    for (size_t i = 0; i < outData.leftToRightMatch.size(); ++i) {
        outData.leftToRightMatch[i] = static_cast<int>(i);
    }
    return true;
}

bool LoadOrbStereoAugmentFeatures(
    const IVisualDescriptorProvider *leftProvider,
    const IVisualDescriptorProvider *rightProvider, const cv::Mat &leftGray,
    const cv::Mat &rightGray, OrbStereoAugmentFeatures &features)
{
    if (!leftProvider->DetectAndCompute(leftGray, features.leftKeypoints,
                                        features.leftDescriptors) ||
        !rightProvider->DetectAndCompute(rightGray, features.rightKeypoints,
                                         features.rightDescriptors)) {
        return false;
    }
    return !features.leftKeypoints.empty() && !features.rightKeypoints.empty() &&
           !features.leftDescriptors.empty() && !features.rightDescriptors.empty();
}

bool HasValidOrbStereoAugmentFeatures(
    const OrbStereoAugmentFeatures &features)
{
    return features.leftDescriptors.type() == CV_8U &&
           features.rightDescriptors.type() == CV_8U &&
           features.leftDescriptors.rows ==
               static_cast<int>(features.leftKeypoints.size()) &&
           features.rightDescriptors.rows ==
               static_cast<int>(features.rightKeypoints.size()) &&
           features.leftDescriptors.cols == features.rightDescriptors.cols;
}

bool CanMergeOrbStereoAugmentDescriptors(
    const StereoFeatureObservationPacket &stereoData,
    const OrbStereoAugmentFeatures &features)
{
    if (!stereoData.leftDescriptors.empty() &&
        (stereoData.leftDescriptors.type() != CV_8U ||
         stereoData.leftDescriptors.cols != features.leftDescriptors.cols)) {
        return false;
    }
    return stereoData.rightDescriptors.empty() ||
           (stereoData.rightDescriptors.type() == CV_8U &&
            stereoData.rightDescriptors.cols == features.rightDescriptors.cols);
}

OrbStereoAugmentOptions LoadOrbStereoAugmentOptions()
{
    OrbStereoAugmentOptions options;
    options.maxHamming = EnvIntValueClamped(
        "SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT_MAX_HAMMING", 60, 1, 256);
    options.maxSecondBest = EnvIntValueClamped(
        "SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT_SECOND_BEST", 80, 1, 256);
    options.ratio = EnvFloatValueClamped(
        "SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT_RATIO", 0.85f, 0.1f, 1.0f);
    options.minZncc =
        EnvFloatValueClamped("SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT_MIN_ZNCC",
                             kStereoMinZnccScore, -1.0f, 1.0f);
    return options;
}

bool PassesOrbStereoRatioTest(int bestDistance, int secondDistance,
                              const OrbStereoAugmentOptions &options)
{
    if (bestDistance > options.maxHamming) {
        return false;
    }
    return secondDistance >= options.maxSecondBest ||
           bestDistance <= static_cast<int>(options.ratio * secondDistance);
}

int FindBestOrbStereoRightIndex(
    const IVisualDescriptorProvider &leftProvider,
    const OrbStereoAugmentFeatures &features, const cv::Mat &rightGray,
    size_t leftIndex, int &bestDistance, int &secondDistance)
{
    const cv::Point2f &leftPt = features.leftKeypoints[leftIndex].pt;
    int bestRight = -1;
    bestDistance = INT_MAX;
    secondDistance = INT_MAX;
    for (size_t ri = 0; ri < features.rightKeypoints.size(); ++ri) {
        const cv::Point2f &rightPt = features.rightKeypoints[ri].pt;
        if (!IsPointSafeForDescriptor(rightPt, rightGray) ||
            !IsStereoPairGeometricallyValid(leftPt, rightPt)) {
            continue;
        }
        const int distance = leftProvider.DescriptorDistance(
            features.leftDescriptors.row(static_cast<int>(leftIndex)),
            features.rightDescriptors.row(static_cast<int>(ri)));
        if (distance < bestDistance) {
            secondDistance = bestDistance;
            bestDistance = distance;
            bestRight = static_cast<int>(ri);
        } else if (distance < secondDistance) {
            secondDistance = distance;
        }
    }
    return bestRight;
}

bool BuildOrbStereoAugmentCandidate(
    const OrbStereoAugmentCandidateRequest &request,
    OrbStereoAugmentCandidate &candidate)
{
    const OrbStereoAugmentFeatures &features = *request.features;
    const OrbStereoAugmentOptions &options = *request.options;
    int bestDistance = INT_MAX;
    int secondDistance = INT_MAX;
    const int bestRight = FindBestOrbStereoRightIndex(
        *request.leftProvider, features, *request.rightGray, request.leftIndex,
        bestDistance, secondDistance);
    if (bestRight < 0 ||
        !PassesOrbStereoRatioTest(bestDistance, secondDistance, options)) {
        return false;
    }

    const cv::Point2f &leftPt = features.leftKeypoints[request.leftIndex].pt;
    const cv::Point2f &rightPt =
        features.rightKeypoints[static_cast<size_t>(bestRight)].pt;
    float zncc = -1.0f;
    if (!ComputePatchZncc(*request.leftGray32f, leftPt, *request.rightGray32f,
                          rightPt, zncc) ||
        zncc < request.options->minZncc) {
        return false;
    }
    candidate =
        OrbStereoAugmentCandidate{static_cast<int>(request.leftIndex),
                                  bestRight, bestDistance, zncc,
                                  leftPt.x - rightPt.x};
    return true;
}

void SortOrbStereoAugmentCandidates(
    std::vector<OrbStereoAugmentCandidate> &candidates)
{
    std::sort(candidates.begin(), candidates.end(),
              [](const OrbStereoAugmentCandidate &a,
                 const OrbStereoAugmentCandidate &b) {
                  if (a.distance != b.distance) {
                      return a.distance < b.distance;
                  }
                  if (a.zncc != b.zncc) {
                      return a.zncc > b.zncc;
                  }
                  return a.disparity > b.disparity;
              });
}

std::vector<OrbStereoAugmentCandidate> CollectOrbStereoAugmentCandidates(
    const IVisualDescriptorProvider &leftProvider,
    const OrbStereoAugmentFeatures &features,
    const StereoFeatureObservationPacket &stereoData,
    const OrbStereoAugmentOptions &options, const cv::Mat &leftGray,
    const cv::Mat &rightGray, OrbStereoAugmentMatchState &matchState)
{
    cv::Mat leftGray32f;
    cv::Mat rightGray32f;
    leftGray.convertTo(leftGray32f, CV_32F);
    rightGray.convertTo(rightGray32f, CV_32F);

    std::vector<OrbStereoAugmentCandidate> candidates;
    candidates.reserve(
        std::min(features.leftKeypoints.size(), features.rightKeypoints.size()));
    for (size_t li = 0; li < features.leftKeypoints.size(); ++li) {
        const cv::Point2f &leftPt = features.leftKeypoints[li].pt;
        if (!IsPointSafeForDescriptor(leftPt, leftGray) ||
            IsPointNearExistingKeypoint(leftPt, stereoData.leftKeypoints)) {
            continue;
        }

        OrbStereoAugmentCandidate candidate;
        const OrbStereoAugmentCandidateRequest request{
            &leftProvider, &features, &options, &rightGray, &leftGray32f,
            &rightGray32f, li};
        if (!BuildOrbStereoAugmentCandidate(request, candidate)) {
            continue;
        }
        const size_t rightIndex = static_cast<size_t>(candidate.rightIndex);
        if (candidate.distance >= matchState.bestRightDistance[rightIndex]) {
            continue;
        }
        matchState.bestRightDistance[rightIndex] = candidate.distance;
        matchState.bestLeftForRight[rightIndex] = candidate.leftIndex;
        candidates.push_back(candidate);
    }
    SortOrbStereoAugmentCandidates(candidates);
    return candidates;
}

void EnsureLeftToRightMatchInitialized(StereoFeatureObservationPacket &data)
{
    if (!data.leftToRightMatch.empty()) {
        return;
    }
    data.leftToRightMatch.resize(data.leftKeypoints.size(), -1);
    const size_t alignedCount =
        std::min(data.leftKeypoints.size(), data.rightKeypoints.size());
    for (size_t i = 0; i < alignedCount; ++i) {
        data.leftToRightMatch[i] = static_cast<int>(i);
    }
}

bool IsOrbStereoAugmentCandidateCurrent(
    const OrbStereoAugmentCandidate &candidate,
    const OrbStereoAugmentFeatures &features,
    const OrbStereoAugmentMatchState &matchState)
{
    return candidate.leftIndex >= 0 && candidate.rightIndex >= 0 &&
           static_cast<size_t>(candidate.leftIndex) <
               features.leftKeypoints.size() &&
           static_cast<size_t>(candidate.rightIndex) <
               features.rightKeypoints.size() &&
           matchState.bestLeftForRight[static_cast<size_t>(
               candidate.rightIndex)] == candidate.leftIndex;
}

void AppendOrbStereoAugmentCandidate(
    const OrbStereoAugmentCandidate &candidate,
    const OrbStereoAugmentFeatures &features,
    StereoFeatureObservationPacket &stereoData)
{
    const int rightOutputIndex =
        static_cast<int>(stereoData.rightKeypoints.size());
    stereoData.leftKeypoints.push_back(
        features.leftKeypoints[static_cast<size_t>(candidate.leftIndex)]);
    stereoData.rightKeypoints.push_back(
        features.rightKeypoints[static_cast<size_t>(candidate.rightIndex)]);
    stereoData.leftDescriptors.push_back(
        features.leftDescriptors.row(candidate.leftIndex));
    stereoData.rightDescriptors.push_back(
        features.rightDescriptors.row(candidate.rightIndex));
    stereoData.leftToRightMatch.push_back(rightOutputIndex);
}

size_t AppendOrbStereoAugmentCandidates(
    const std::vector<OrbStereoAugmentCandidate> &candidates,
    const OrbStereoAugmentFeatures &features,
    const OrbStereoAugmentMatchState &matchState,
    StereoFeatureObservationPacket &stereoData, size_t maxExtraPairs)
{
    EnsureLeftToRightMatchInitialized(stereoData);
    size_t appended = 0;
    for (const OrbStereoAugmentCandidate &candidate : candidates) {
        if (appended >= maxExtraPairs) {
            break;
        }
        if (!IsOrbStereoAugmentCandidateCurrent(candidate, features, matchState)) {
            continue;
        }
        const cv::Point2f &leftPt =
            features.leftKeypoints[static_cast<size_t>(candidate.leftIndex)].pt;
        if (IsPointNearExistingKeypoint(leftPt, stereoData.leftKeypoints)) {
            continue;
        }
        AppendOrbStereoAugmentCandidate(candidate, features, stereoData);
        ++appended;
    }
    return appended;
}

bool FinalizeStereoObservationsFromPairsWithAllLeft(
    const IVisualDescriptorProvider *leftProvider,
    const IVisualDescriptorProvider *rightProvider, const cv::Mat &leftGray,
    const cv::Mat &rightGray, const std::vector<cv::Point2f> &allLeftPoints,
    const std::vector<StereoMatchPair> &stereoMatches,
    const VisualFeatureSet &leftFeatures, const VisualFeatureSet &rightFeatures,
    StereoFeatureObservationPacket &outData)
{
    if (leftProvider == nullptr || rightProvider == nullptr || leftGray.empty() ||
        rightGray.empty() || allLeftPoints.empty()) {
        return false;
    }

    std::vector<cv::Point2f> safeLeftPoints;
    safeLeftPoints.reserve(allLeftPoints.size());
    for (const cv::Point2f &point : allLeftPoints) {
        if (IsPointSafeForDescriptor(point, leftGray)) {
            safeLeftPoints.push_back(point);
        }
    }
    if (safeLeftPoints.empty()) {
        return false;
    }

    std::vector<cv::KeyPoint> leftKeypoints;
    cv::Mat leftDescriptors;
    if (!leftProvider->ComputeDescriptorsAtPoints(
            leftGray, safeLeftPoints, leftKeypoints, leftDescriptors)) {
        return false;
    }

    std::vector<cv::Point2f> stereoLeftPoints;
    std::vector<cv::Point2f> stereoRightPoints;
    stereoLeftPoints.reserve(stereoMatches.size());
    stereoRightPoints.reserve(stereoMatches.size());
    for (const StereoMatchPair &match : stereoMatches) {
        if (!IsStereoMatchInFeatureRange(match, leftFeatures, rightFeatures)) {
            continue;
        }
        const cv::Point2f &leftPoint =
            leftFeatures.keypoints[static_cast<size_t>(match.leftIndex)];
        const cv::Point2f &rightPoint =
            rightFeatures.keypoints[static_cast<size_t>(match.rightIndex)];
        if (!IsPointSafeForDescriptor(leftPoint, leftGray) ||
            !IsPointSafeForDescriptor(rightPoint, rightGray) ||
            !IsStereoPairGeometricallyValid(leftPoint, rightPoint)) {
            continue;
        }
        stereoLeftPoints.push_back(leftPoint);
        stereoRightPoints.push_back(rightPoint);
    }
    if (stereoLeftPoints.empty()) {
        return false;
    }

    std::vector<cv::KeyPoint> stereoLeftKeypoints;
    std::vector<cv::KeyPoint> rightKeypoints;
    cv::Mat stereoLeftDescriptors;
    cv::Mat rightDescriptors;
    if (!leftProvider->ComputeDescriptorsAtPoints(leftGray, stereoLeftPoints,
                                                  stereoLeftKeypoints,
                                                  stereoLeftDescriptors) ||
        !rightProvider->ComputeDescriptorsAtPoints(
            rightGray, stereoRightPoints, rightKeypoints, rightDescriptors)) {
        return false;
    }
    if (stereoLeftKeypoints.size() != rightKeypoints.size() ||
        stereoLeftDescriptors.rows != rightDescriptors.rows ||
        stereoLeftDescriptors.rows !=
            static_cast<int>(stereoLeftKeypoints.size())) {
        return false;
    }

    std::vector<int> leftToRight(leftKeypoints.size(), -1);
    cv::Mat mergedRightDescriptors;
    mergedRightDescriptors.create(0, rightDescriptors.cols,
                                  rightDescriptors.type());
    std::vector<cv::KeyPoint> mergedRightKeypoints;
    mergedRightKeypoints.reserve(rightKeypoints.size());
    for (size_t li = 0; li < leftKeypoints.size(); ++li) {
        for (size_t si = 0; si < stereoLeftKeypoints.size(); ++si) {
            const cv::Point2f delta =
                leftKeypoints[li].pt - stereoLeftKeypoints[si].pt;
            if (delta.x * delta.x + delta.y * delta.y > 1.0f) {
                continue;
            }
            leftToRight[li] = static_cast<int>(mergedRightKeypoints.size());
            mergedRightKeypoints.push_back(rightKeypoints[si]);
            mergedRightDescriptors.push_back(
                rightDescriptors.row(static_cast<int>(si)));
            break;
        }
    }

    if (mergedRightKeypoints.empty()) {
        return false;
    }

    outData.leftKeypoints = std::move(leftKeypoints);
    outData.rightKeypoints = std::move(mergedRightKeypoints);
    outData.leftDescriptors = std::move(leftDescriptors);
    outData.rightDescriptors = std::move(mergedRightDescriptors);
    outData.matchedStereoPairs = true;
    outData.leftToRightMatch = std::move(leftToRight);
    return true;
}

void AppendDescriptorLeftOnlyFeatures(
    const IVisualDescriptorProvider *leftProvider, const cv::Mat &leftGray,
    StereoFeatureObservationPacket &stereoData, size_t maxLeftFeatures)
{
    if (leftProvider == nullptr || leftGray.empty() ||
        stereoData.leftKeypoints.size() >= maxLeftFeatures) {
        return;
    }

    std::vector<cv::KeyPoint> orbKeypoints;
    cv::Mat orbDescriptors;
    if (!leftProvider->DetectAndCompute(leftGray, orbKeypoints, orbDescriptors)) {
        return;
    }
    if (orbKeypoints.empty() || orbDescriptors.empty() ||
        orbDescriptors.type() != CV_8U ||
        orbDescriptors.rows != static_cast<int>(orbKeypoints.size())) {
        return;
    }

    const size_t initialLeftCount = stereoData.leftKeypoints.size();
    std::vector<int> selectedRows;
    selectedRows.reserve(
        std::min(orbKeypoints.size(), maxLeftFeatures - initialLeftCount));
    for (size_t i = 0; i < orbKeypoints.size() &&
                       initialLeftCount + selectedRows.size() < maxLeftFeatures;
         ++i) {
        if (!IsPointSafeForDescriptor(orbKeypoints[i].pt, leftGray) ||
            IsPointNearExistingKeypoint(orbKeypoints[i].pt,
                                        stereoData.leftKeypoints)) {
            continue;
        }
        stereoData.leftKeypoints.push_back(orbKeypoints[i]);
        selectedRows.push_back(static_cast<int>(i));
    }
    if (selectedRows.empty()) {
        return;
    }

    cv::Mat mergedDescriptors;
    if (!stereoData.leftDescriptors.empty()) {
        stereoData.leftDescriptors.copyTo(mergedDescriptors);
    } else {
        mergedDescriptors.create(0, orbDescriptors.cols, orbDescriptors.type());
    }
    for (int row : selectedRows) {
        mergedDescriptors.push_back(orbDescriptors.row(row));
    }
    stereoData.leftDescriptors = std::move(mergedDescriptors);
    if (!stereoData.leftToRightMatch.empty()) {
        stereoData.leftToRightMatch.resize(stereoData.leftKeypoints.size(), -1);
    }
}

size_t AppendDescriptorStereoAugmentFeatures(
    const IVisualDescriptorProvider *leftProvider,
    const IVisualDescriptorProvider *rightProvider, const cv::Mat &leftGray,
    const cv::Mat &rightGray, StereoFeatureObservationPacket &stereoData,
    size_t maxExtraPairs)
{
    if (leftProvider == nullptr || rightProvider == nullptr || leftGray.empty() ||
        rightGray.empty() || maxExtraPairs == 0) {
        return 0;
    }

    OrbStereoAugmentFeatures features;
    if (!LoadOrbStereoAugmentFeatures(leftProvider, rightProvider, leftGray,
                                      rightGray, features)) {
        return 0;
    }
    if (!HasValidOrbStereoAugmentFeatures(features) ||
        !CanMergeOrbStereoAugmentDescriptors(stereoData, features)) {
        return 0;
    }

    OrbStereoAugmentMatchState matchState;
    matchState.bestLeftForRight.assign(features.rightKeypoints.size(), -1);
    matchState.bestRightDistance.assign(features.rightKeypoints.size(), INT_MAX);
    const OrbStereoAugmentOptions options = LoadOrbStereoAugmentOptions();
    std::vector<OrbStereoAugmentCandidate> candidates =
        CollectOrbStereoAugmentCandidates(*leftProvider, features, stereoData,
                                          options, leftGray, rightGray,
                                          matchState);
    if (candidates.empty()) {
        return 0;
    }

    const size_t appended = AppendOrbStereoAugmentCandidates(
        candidates, features, matchState, stereoData, maxExtraPairs);
    if (appended > 0) {
        stereoData.matchedStereoPairs = true;
    }
    return appended;
}

} // namespace

uint64_t HashStereoFeatureObservations(
    const core::ports::StereoFeatureObservationPacket &data)
{
    uint64_t hash = 1469598103934665603ULL;
    hash = HashIntValue(hash, static_cast<int>(data.leftKeypoints.size()));
    hash = HashIntValue(hash, static_cast<int>(data.rightKeypoints.size()));
    hash = HashIntValue(hash, data.matchedStereoPairs ? 1 : 0);
    const size_t leftCount = std::min<size_t>(data.leftKeypoints.size(), 512);
    for (size_t i = 0; i < leftCount; ++i) {
        hash = HashFloatValue(hash, data.leftKeypoints[i].pt.x);
        hash = HashFloatValue(hash, data.leftKeypoints[i].pt.y);
    }
    const size_t rightCount = std::min<size_t>(data.rightKeypoints.size(), 512);
    for (size_t i = 0; i < rightCount; ++i) {
        hash = HashFloatValue(hash, data.rightKeypoints[i].pt.x);
        hash = HashFloatValue(hash, data.rightKeypoints[i].pt.y);
    }
    const size_t matchCount = std::min<size_t>(data.leftToRightMatch.size(), 512);
    for (size_t i = 0; i < matchCount; ++i) {
        hash = HashIntValue(hash, data.leftToRightMatch[i]);
    }
    hash = HashMatSample(hash, data.leftDescriptors);
    hash = HashMatSample(hash, data.rightDescriptors);
    return hash;
}

bool BuildStereoFeaturePacket(
    const core::ports::StereoFeaturePacketBuildInput &input,
    core::ports::StereoFeaturePacket &packet)
{
    packet = core::ports::StereoFeaturePacket{};
    if (!HasInputImages(input) || !HasMatchedPoints(input) ||
        input.leftFeatures == nullptr || input.rightFeatures == nullptr) {
        return false;
    }

    const size_t pairedFeatureCount =
        std::min(input.leftFeatures->keypoints.size(),
                 input.rightFeatures->keypoints.size());
    const bool nativeDescriptorInject =
        input.allowNativeDescriptorInject &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_NATIVE_DESCRIPTOR_INJECT", false);
    const bool filteredStereoInject =
        EnvFlagEnabled("SMART_DRONE_SP_LG_FILTERED_STEREO_INJECT", true);
    const bool useDepthFilteredMatches = EnvFlagEnabled(
        "SMART_DRONE_SP_LG_DEPTH_DISPARITY_CONSISTENCY_FILTER", false);
    const bool allLeftGeometricDepth =
        input.allowAllLeftGeometricDepth &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_ALL_LEFT_GEOMETRIC_DEPTH", false);

    if (nativeDescriptorInject) {
        std::vector<StereoMatchPair> descriptorMatches;
        if (filteredStereoInject && input.filteredMatches != nullptr &&
            !input.filteredMatches->empty()) {
            descriptorMatches = *input.filteredMatches;
        } else {
            descriptorMatches = BuildAlignedDescriptorMatches(pairedFeatureCount);
        }
        packet.packed = BuildStereoObservationsFromFeatureMatches(
            *input.leftFeatures, *input.rightFeatures, descriptorMatches,
            packet.observations);
    }

    const std::vector<StereoMatchPair> *depthMatches =
        useDepthFilteredMatches ? input.filteredMatches : input.rawMatches;
    if (!packet.packed && allLeftGeometricDepth && depthMatches != nullptr &&
        !depthMatches->empty()) {
        packet.packed = FinalizeStereoObservationsFromPairsWithAllLeft(
            input.leftDescriptorProvider, input.rightDescriptorProvider,
            *input.leftPrepared, *input.rightPrepared, *input.matchedLeftPoints,
            *depthMatches, *input.leftFeatures, *input.rightFeatures,
            packet.observations);
    }

    if (!packet.packed) {
        packet.packed = FinalizeStereoObservationsFromPairs(
            input.leftDescriptorProvider, input.rightDescriptorProvider,
            *input.leftPrepared, *input.rightPrepared, *input.matchedLeftPoints,
            *input.matchedRightPoints, packet.observations);
    }
    if (!packet.packed) {
        return false;
    }

    const bool initializedForMonoAugmentation =
        input.initializedForMonoAugmentation;
    if (initializedForMonoAugmentation &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT", false)) {
        const auto augmentStartTp = std::chrono::steady_clock::now();
        const size_t maxExtraPairs = EnvSizeValueClamped(
            "SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT_MAX_PAIRS", 96, 1, 512);
        packet.orbStereoAugmentPairs = AppendDescriptorStereoAugmentFeatures(
            input.leftDescriptorProvider, input.rightDescriptorProvider,
            *input.leftPrepared, *input.rightPrepared, packet.observations,
            maxExtraPairs);
        packet.monoAugmentMs +=
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - augmentStartTp)
                .count();
    }

    const int monoAugmentMinOkStreak = EnvIntValueClamped(
        "SMART_DRONE_SP_LG_ORB_LEFT_AUGMENT_MIN_OK_STREAK", 0, 0, 100000);
    if (initializedForMonoAugmentation &&
        input.stableOkStreak >= monoAugmentMinOkStreak &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_ORB_LEFT_AUGMENT", false)) {
        const auto augmentStartTp = std::chrono::steady_clock::now();
        const size_t maxLeftFeatures = EnvSizeValueClamped(
            "SMART_DRONE_STEREO_FEATURE_MAX_LEFT_FEATURES",
            kStereoFeatureMaxLeftFeatures, kStereoFeatureMaxLeftFeatures,
            kStereoFeatureMaxLeftFeaturesLimit);
        AppendDescriptorLeftOnlyFeatures(input.leftDescriptorProvider,
                                         *input.leftPrepared, packet.observations,
                                         maxLeftFeatures);
        packet.monoAugmentMs =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - augmentStartTp)
                .count();
    }

    packet.hash = HashStereoFeatureObservations(packet.observations);
    CopyPacketFeaturePoints(packet);
    return true;
}

bool DefaultStereoFeaturePacketBuilder::BuildPacket(
    const core::ports::StereoFeaturePacketBuildInput &input,
    core::ports::StereoFeaturePacket &packet) const
{
    return BuildStereoFeaturePacket(input, packet);
}

uint64_t DefaultStereoFeaturePacketBuilder::HashStereoData(
    const core::ports::StereoFeatureObservationPacket &data) const
{
    return HashStereoFeatureObservations(data);
}

} // namespace SmartDrone::adapters::slam
