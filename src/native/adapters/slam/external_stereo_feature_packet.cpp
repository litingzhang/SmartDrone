#include "adapters/slam/external_stereo_feature_packet.h"

#include <algorithm>
#include <chrono>
#include <climits>
#include <cstring>

#include "adapters/slam/external_descriptor_geometry.h"
#include "adapters/slam/slam_env.h"
#include "adapters/slam/stereo_geometry.h"

namespace smartdrone::adapters::slam {

namespace {

constexpr size_t kExternalStereoMaxLeftFeatures = 1200;
constexpr size_t kExternalStereoMaxLeftFeaturesLimit = 2500;

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

bool HasInputImages(const ExternalStereoFeaturePacketBuildInput &input)
{
    return input.leftPrepared != nullptr && input.rightPrepared != nullptr &&
           !input.leftPrepared->empty() && !input.rightPrepared->empty();
}

bool HasMatchedPoints(const ExternalStereoFeaturePacketBuildInput &input)
{
    return input.matchedLeftPoints != nullptr && input.matchedRightPoints != nullptr &&
           !input.matchedLeftPoints->empty() && input.matchedLeftPoints->size() == input.matchedRightPoints->size();
}

std::vector<StereoMatchPair> BuildAlignedDescriptorMatches(size_t pairedFeatureCount)
{
    std::vector<StereoMatchPair> descriptorMatches;
    descriptorMatches.reserve(pairedFeatureCount);
    for (size_t i = 0; i < pairedFeatureCount; ++i) {
        descriptorMatches.push_back(StereoMatchPair{static_cast<int>(i), static_cast<int>(i),
                                                    1.0f, 1.0f, 0.0f, 1.0f});
    }
    return descriptorMatches;
}

void CopyPacketFeaturePoints(ExternalStereoFeaturePacket &packet)
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

bool HasValidFeatureDescriptors(const ExternalFeatureSet &features)
{
    return !features.descriptors.empty() && features.descriptors.type() == CV_32F &&
           features.descriptors.rows == static_cast<int>(features.keypoints.size()) && features.descriptors.cols > 0;
}

bool BuildExternalStereoFromFeatureMatches(const ExternalFeatureSet &left, const ExternalFeatureSet &right,
                                           const std::vector<StereoMatchPair> &matches,
                                           ExternalStereoObservationPacket &outData)
{
    if (!HasValidFeatureDescriptors(left) || !HasValidFeatureDescriptors(right) || matches.empty() ||
        left.descriptors.cols != right.descriptors.cols) {
        return false;
    }

    const int descriptorDim = left.descriptors.cols;
    cv::Mat leftDescriptors(static_cast<int>(matches.size()), descriptorDim, CV_32F);
    cv::Mat rightDescriptors(static_cast<int>(matches.size()), descriptorDim, CV_32F);
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
        leftKeypoints.push_back(MakeExternalKeyPoint(left.keypoints[static_cast<size_t>(match.leftIndex)]));
        rightKeypoints.push_back(MakeExternalKeyPoint(right.keypoints[static_cast<size_t>(match.rightIndex)]));
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

bool FinalizeStereoExternalFromPairs(const ExternalDescriptorProvider *leftProvider,
                                     const ExternalDescriptorProvider *rightProvider, const cv::Mat &leftGray,
                                     const cv::Mat &rightGray, const std::vector<cv::Point2f> &leftPoints,
                                     const std::vector<cv::Point2f> &rightPoints,
                                     ExternalStereoObservationPacket &outData)
{
    if (leftProvider == nullptr || rightProvider == nullptr || leftGray.empty() || rightGray.empty() ||
        leftPoints.empty() || leftPoints.size() != rightPoints.size()) {
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
    if (!leftProvider->ComputeDescriptorsAtPoints(leftGray, filteredLeft, leftKeypoints, leftDescriptors) ||
        !rightProvider->ComputeDescriptorsAtPoints(rightGray, filteredRight, rightKeypoints, rightDescriptors)) {
        return false;
    }
    if (leftKeypoints.size() != rightKeypoints.size() || leftDescriptors.rows != rightDescriptors.rows ||
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

bool FinalizeStereoExternalFromPairsWithAllLeft(const ExternalDescriptorProvider *leftProvider,
                                                const ExternalDescriptorProvider *rightProvider, const cv::Mat &leftGray,
                                                const cv::Mat &rightGray,
                                                const std::vector<cv::Point2f> &allLeftPoints,
                                                const std::vector<StereoMatchPair> &stereoMatches,
                                                const ExternalFeatureSet &leftFeatures,
                                                const ExternalFeatureSet &rightFeatures,
                                                ExternalStereoObservationPacket &outData)
{
    if (leftProvider == nullptr || rightProvider == nullptr || leftGray.empty() || rightGray.empty() ||
        allLeftPoints.empty()) {
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
    if (!leftProvider->ComputeDescriptorsAtPoints(leftGray, safeLeftPoints, leftKeypoints, leftDescriptors)) {
        return false;
    }

    std::vector<cv::Point2f> stereoLeftPoints;
    std::vector<cv::Point2f> stereoRightPoints;
    stereoLeftPoints.reserve(stereoMatches.size());
    stereoRightPoints.reserve(stereoMatches.size());
    for (const StereoMatchPair &match : stereoMatches) {
        if (match.leftIndex < 0 || match.rightIndex < 0 ||
            static_cast<size_t>(match.leftIndex) >= leftFeatures.keypoints.size() ||
            static_cast<size_t>(match.rightIndex) >= rightFeatures.keypoints.size()) {
            continue;
        }
        const cv::Point2f &leftPoint = leftFeatures.keypoints[static_cast<size_t>(match.leftIndex)];
        const cv::Point2f &rightPoint = rightFeatures.keypoints[static_cast<size_t>(match.rightIndex)];
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
    if (!leftProvider->ComputeDescriptorsAtPoints(leftGray, stereoLeftPoints, stereoLeftKeypoints,
                                                  stereoLeftDescriptors) ||
        !rightProvider->ComputeDescriptorsAtPoints(rightGray, stereoRightPoints, rightKeypoints, rightDescriptors)) {
        return false;
    }
    if (stereoLeftKeypoints.size() != rightKeypoints.size() ||
        stereoLeftDescriptors.rows != rightDescriptors.rows ||
        stereoLeftDescriptors.rows != static_cast<int>(stereoLeftKeypoints.size())) {
        return false;
    }

    std::vector<int> leftToRight(leftKeypoints.size(), -1);
    cv::Mat mergedRightDescriptors;
    mergedRightDescriptors.create(0, rightDescriptors.cols, rightDescriptors.type());
    std::vector<cv::KeyPoint> mergedRightKeypoints;
    mergedRightKeypoints.reserve(rightKeypoints.size());
    for (size_t li = 0; li < leftKeypoints.size(); ++li) {
        for (size_t si = 0; si < stereoLeftKeypoints.size(); ++si) {
            const cv::Point2f delta = leftKeypoints[li].pt - stereoLeftKeypoints[si].pt;
            if (delta.x * delta.x + delta.y * delta.y > 1.0f) {
                continue;
            }
            leftToRight[li] = static_cast<int>(mergedRightKeypoints.size());
            mergedRightKeypoints.push_back(rightKeypoints[si]);
            mergedRightDescriptors.push_back(rightDescriptors.row(static_cast<int>(si)));
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

void AppendDescriptorLeftOnlyFeatures(const ExternalDescriptorProvider *leftProvider, const cv::Mat &leftGray,
                               ExternalStereoObservationPacket &externalData, size_t maxLeftFeatures)
{
    if (leftProvider == nullptr || leftGray.empty() || externalData.leftKeypoints.size() >= maxLeftFeatures) {
        return;
    }

    std::vector<cv::KeyPoint> orbKeypoints;
    cv::Mat orbDescriptors;
    if (!leftProvider->DetectAndCompute(leftGray, orbKeypoints, orbDescriptors)) {
        return;
    }
    if (orbKeypoints.empty() || orbDescriptors.empty() || orbDescriptors.type() != CV_8U ||
        orbDescriptors.rows != static_cast<int>(orbKeypoints.size())) {
        return;
    }

    const size_t initialLeftCount = externalData.leftKeypoints.size();
    std::vector<int> selectedRows;
    selectedRows.reserve(std::min(orbKeypoints.size(), maxLeftFeatures - initialLeftCount));
    for (size_t i = 0; i < orbKeypoints.size() && initialLeftCount + selectedRows.size() < maxLeftFeatures; ++i) {
        if (!IsPointSafeForDescriptor(orbKeypoints[i].pt, leftGray) ||
            IsPointNearExistingKeypoint(orbKeypoints[i].pt, externalData.leftKeypoints)) {
            continue;
        }
        externalData.leftKeypoints.push_back(orbKeypoints[i]);
        selectedRows.push_back(static_cast<int>(i));
    }
    if (selectedRows.empty()) {
        return;
    }

    cv::Mat mergedDescriptors;
    if (!externalData.leftDescriptors.empty()) {
        externalData.leftDescriptors.copyTo(mergedDescriptors);
    } else {
        mergedDescriptors.create(0, orbDescriptors.cols, orbDescriptors.type());
    }
    for (int row : selectedRows) {
        mergedDescriptors.push_back(orbDescriptors.row(row));
    }
    externalData.leftDescriptors = std::move(mergedDescriptors);
    if (!externalData.leftToRightMatch.empty()) {
        externalData.leftToRightMatch.resize(externalData.leftKeypoints.size(), -1);
    }
}

size_t AppendDescriptorStereoAugmentFeatures(const ExternalDescriptorProvider *leftProvider,
                                      const ExternalDescriptorProvider *rightProvider, const cv::Mat &leftGray,
                                      const cv::Mat &rightGray,
                                      ExternalStereoObservationPacket &externalData, size_t maxExtraPairs)
{
    if (leftProvider == nullptr || rightProvider == nullptr || leftGray.empty() || rightGray.empty() ||
        maxExtraPairs == 0) {
        return 0;
    }

    std::vector<cv::KeyPoint> leftKeypoints;
    std::vector<cv::KeyPoint> rightKeypoints;
    cv::Mat leftDescriptors;
    cv::Mat rightDescriptors;
    if (!leftProvider->DetectAndCompute(leftGray, leftKeypoints, leftDescriptors) ||
        !rightProvider->DetectAndCompute(rightGray, rightKeypoints, rightDescriptors)) {
        return 0;
    }
    if (leftKeypoints.empty() || rightKeypoints.empty() || leftDescriptors.empty() || rightDescriptors.empty() ||
        leftDescriptors.type() != CV_8U || rightDescriptors.type() != CV_8U ||
        leftDescriptors.rows != static_cast<int>(leftKeypoints.size()) ||
        rightDescriptors.rows != static_cast<int>(rightKeypoints.size()) ||
        leftDescriptors.cols != rightDescriptors.cols) {
        return 0;
    }
    if ((!externalData.leftDescriptors.empty() &&
         (externalData.leftDescriptors.type() != CV_8U || externalData.leftDescriptors.cols != leftDescriptors.cols)) ||
        (!externalData.rightDescriptors.empty() &&
         (externalData.rightDescriptors.type() != CV_8U ||
          externalData.rightDescriptors.cols != rightDescriptors.cols))) {
        return 0;
    }

    struct OrbStereoAugmentCandidate {
        int leftIndex{-1};
        int rightIndex{-1};
        int distance{INT_MAX};
        float zncc{-1.0f};
        float disparity{0.0f};
    };

    cv::Mat leftGray32f;
    cv::Mat rightGray32f;
    leftGray.convertTo(leftGray32f, CV_32F);
    rightGray.convertTo(rightGray32f, CV_32F);

    const int maxHamming = EnvIntValueClamped("SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT_MAX_HAMMING", 60, 1, 256);
    const int maxSecondBest = EnvIntValueClamped("SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT_SECOND_BEST", 80, 1, 256);
    const float ratio = EnvFloatValueClamped("SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT_RATIO", 0.85f, 0.1f, 1.0f);
    const float minZncc =
        EnvFloatValueClamped("SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT_MIN_ZNCC", kStereoMinZnccScore, -1.0f, 1.0f);

    std::vector<int> bestLeftForRight(rightKeypoints.size(), -1);
    std::vector<int> bestRightDistance(rightKeypoints.size(), INT_MAX);
    std::vector<OrbStereoAugmentCandidate> candidates;
    candidates.reserve(std::min(leftKeypoints.size(), rightKeypoints.size()));

    for (size_t li = 0; li < leftKeypoints.size(); ++li) {
        const cv::Point2f &leftPt = leftKeypoints[li].pt;
        if (!IsPointSafeForDescriptor(leftPt, leftGray) ||
            IsPointNearExistingKeypoint(leftPt, externalData.leftKeypoints)) {
            continue;
        }

        int bestRight = -1;
        int bestDistance = INT_MAX;
        int secondDistance = INT_MAX;
        for (size_t ri = 0; ri < rightKeypoints.size(); ++ri) {
            const cv::Point2f &rightPt = rightKeypoints[ri].pt;
            if (!IsPointSafeForDescriptor(rightPt, rightGray) ||
                !IsStereoPairGeometricallyValid(leftPt, rightPt)) {
                continue;
            }
            const int distance = leftProvider->DescriptorDistance(
                leftDescriptors.row(static_cast<int>(li)), rightDescriptors.row(static_cast<int>(ri)));
            if (distance < bestDistance) {
                secondDistance = bestDistance;
                bestDistance = distance;
                bestRight = static_cast<int>(ri);
            } else if (distance < secondDistance) {
                secondDistance = distance;
            }
        }
        if (bestRight < 0 || bestDistance > maxHamming) {
            continue;
        }
        if (secondDistance < maxSecondBest && bestDistance > static_cast<int>(ratio * secondDistance)) {
            continue;
        }

        const cv::Point2f &rightPt = rightKeypoints[static_cast<size_t>(bestRight)].pt;
        float zncc = -1.0f;
        if (!ComputePatchZncc(leftGray32f, leftPt, rightGray32f, rightPt, zncc) || zncc < minZncc) {
            continue;
        }
        if (bestDistance >= bestRightDistance[static_cast<size_t>(bestRight)]) {
            continue;
        }
        bestRightDistance[static_cast<size_t>(bestRight)] = bestDistance;
        bestLeftForRight[static_cast<size_t>(bestRight)] = static_cast<int>(li);
        candidates.push_back(
            OrbStereoAugmentCandidate{static_cast<int>(li), bestRight, bestDistance, zncc, leftPt.x - rightPt.x});
    }

    if (candidates.empty()) {
        return 0;
    }

    std::sort(candidates.begin(), candidates.end(), [](const OrbStereoAugmentCandidate &a,
                                                       const OrbStereoAugmentCandidate &b) {
        if (a.distance != b.distance) {
            return a.distance < b.distance;
        }
        if (a.zncc != b.zncc) {
            return a.zncc > b.zncc;
        }
        return a.disparity > b.disparity;
    });

    const bool haveLeftMatch = !externalData.leftToRightMatch.empty();
    if (!haveLeftMatch) {
        externalData.leftToRightMatch.resize(externalData.leftKeypoints.size(), -1);
        const size_t alignedCount = std::min(externalData.leftKeypoints.size(), externalData.rightKeypoints.size());
        for (size_t i = 0; i < alignedCount; ++i) {
            externalData.leftToRightMatch[i] = static_cast<int>(i);
        }
    }

    size_t appended = 0;
    for (const OrbStereoAugmentCandidate &candidate : candidates) {
        if (appended >= maxExtraPairs) {
            break;
        }
        if (candidate.leftIndex < 0 || candidate.rightIndex < 0 ||
            static_cast<size_t>(candidate.leftIndex) >= leftKeypoints.size() ||
            static_cast<size_t>(candidate.rightIndex) >= rightKeypoints.size() ||
            bestLeftForRight[static_cast<size_t>(candidate.rightIndex)] != candidate.leftIndex) {
            continue;
        }

        const cv::Point2f &leftPt = leftKeypoints[static_cast<size_t>(candidate.leftIndex)].pt;
        if (IsPointNearExistingKeypoint(leftPt, externalData.leftKeypoints)) {
            continue;
        }

        const int rightOutputIndex = static_cast<int>(externalData.rightKeypoints.size());
        externalData.leftKeypoints.push_back(leftKeypoints[static_cast<size_t>(candidate.leftIndex)]);
        externalData.rightKeypoints.push_back(rightKeypoints[static_cast<size_t>(candidate.rightIndex)]);
        externalData.leftDescriptors.push_back(leftDescriptors.row(candidate.leftIndex));
        externalData.rightDescriptors.push_back(rightDescriptors.row(candidate.rightIndex));
        externalData.leftToRightMatch.push_back(rightOutputIndex);
        ++appended;
    }

    if (appended > 0) {
        externalData.matchedStereoPairs = true;
    }
    return appended;
}

} // namespace

uint64_t HashExternalStereoData(const ExternalStereoObservationPacket &data)
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

bool BuildExternalStereoFeaturePacket(const ExternalStereoFeaturePacketBuildInput &input,
                                      ExternalStereoFeaturePacket &packet)
{
    packet = ExternalStereoFeaturePacket{};
    if (!HasInputImages(input) || !HasMatchedPoints(input) || input.leftFeatures == nullptr ||
        input.rightFeatures == nullptr) {
        return false;
    }

    const size_t pairedFeatureCount =
        std::min(input.leftFeatures->keypoints.size(), input.rightFeatures->keypoints.size());
    const bool nativeDescriptorInject =
        input.allowNativeDescriptorInject && EnvFlagEnabled("SMART_DRONE_SP_LG_NATIVE_DESCRIPTOR_INJECT", false);
    const bool filteredStereoInject = EnvFlagEnabled("SMART_DRONE_SP_LG_FILTERED_STEREO_INJECT", true);
    const bool useDepthFilteredMatches = EnvFlagEnabled("SMART_DRONE_SP_LG_DEPTH_DISPARITY_CONSISTENCY_FILTER", false);
    const bool allLeftGeometricDepth =
        input.allowAllLeftGeometricDepth && EnvFlagEnabled("SMART_DRONE_SP_LG_ALL_LEFT_GEOMETRIC_DEPTH", false);

    if (nativeDescriptorInject) {
        std::vector<StereoMatchPair> descriptorMatches;
        if (filteredStereoInject && input.filteredMatches != nullptr && !input.filteredMatches->empty()) {
            descriptorMatches = *input.filteredMatches;
        } else {
            descriptorMatches = BuildAlignedDescriptorMatches(pairedFeatureCount);
        }
        packet.packed =
            BuildExternalStereoFromFeatureMatches(*input.leftFeatures, *input.rightFeatures,
                                                  descriptorMatches, packet.observations);
    }

    const std::vector<StereoMatchPair> *depthMatches =
        useDepthFilteredMatches ? input.filteredMatches : input.rawMatches;
    if (!packet.packed && allLeftGeometricDepth && depthMatches != nullptr && !depthMatches->empty()) {
        packet.packed = FinalizeStereoExternalFromPairsWithAllLeft(
            input.leftDescriptorProvider, input.rightDescriptorProvider, *input.leftPrepared, *input.rightPrepared,
            *input.matchedLeftPoints, *depthMatches, *input.leftFeatures, *input.rightFeatures,
            packet.observations);
    }

    if (!packet.packed) {
        packet.packed = FinalizeStereoExternalFromPairs(input.leftDescriptorProvider, input.rightDescriptorProvider,
                                                        *input.leftPrepared, *input.rightPrepared,
                                                        *input.matchedLeftPoints, *input.matchedRightPoints,
                                                        packet.observations);
    }
    if (!packet.packed) {
        return false;
    }

    const bool initializedForMonoAugmentation = input.initializedForMonoAugmentation;
    if (initializedForMonoAugmentation &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT", false)) {
        const auto augmentStartTp = std::chrono::steady_clock::now();
        const size_t maxExtraPairs =
            EnvSizeValueClamped("SMART_DRONE_SP_LG_ORB_STEREO_AUGMENT_MAX_PAIRS", 96, 1, 512);
        packet.orbStereoAugmentPairs =
            AppendDescriptorStereoAugmentFeatures(input.leftDescriptorProvider, input.rightDescriptorProvider,
                                                  *input.leftPrepared, *input.rightPrepared,
                                                  packet.observations, maxExtraPairs);
        packet.monoAugmentMs +=
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - augmentStartTp).count();
    }

    const int monoAugmentMinOkStreak =
        EnvIntValueClamped("SMART_DRONE_SP_LG_ORB_LEFT_AUGMENT_MIN_OK_STREAK", 0, 0, 100000);
    if (initializedForMonoAugmentation && input.stableOkStreak >= monoAugmentMinOkStreak &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_ORB_LEFT_AUGMENT", false)) {
        const auto augmentStartTp = std::chrono::steady_clock::now();
        const size_t maxLeftFeatures =
            EnvSizeValueClamped("SMART_DRONE_EXTERNAL_STEREO_MAX_LEFT_FEATURES", kExternalStereoMaxLeftFeatures,
                                kExternalStereoMaxLeftFeatures, kExternalStereoMaxLeftFeaturesLimit);
        AppendDescriptorLeftOnlyFeatures(input.leftDescriptorProvider, *input.leftPrepared,
                                         packet.observations, maxLeftFeatures);
        packet.monoAugmentMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - augmentStartTp).count();
    }

    packet.hash = HashExternalStereoData(packet.observations);
    CopyPacketFeaturePoints(packet);
    return true;
}

} // namespace smartdrone::adapters::slam
