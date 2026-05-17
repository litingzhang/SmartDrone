#include "adapters/slam/external_stereo_pair_builder.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "adapters/slam/slam_env.h"
#include "adapters/slam/stereo_geometry.h"

namespace smartdrone::adapters::slam {

namespace {

constexpr int kStereoGridCols = 8;
constexpr int kStereoGridRows = 6;
constexpr int kStereoMaxPairsPerCell = 10;
constexpr int kStereoMaxPairsPerCellLimit = 32;
constexpr float kExternalStereoMinCandidateQuality = 0.18f;

bool IsBetterRightCandidate(float candidateScore, float candidateZncc, float currentScore, float currentZncc)
{
    if (!std::isfinite(currentScore)) {
        return true;
    }
    if (candidateScore != currentScore) {
        return candidateScore > currentScore;
    }
    return candidateZncc > currentZncc;
}

bool DescriptorSimilarity(const cv::Mat &lhs, const cv::Mat &rhs, float &score)
{
    score = -std::numeric_limits<float>::infinity();
    if (lhs.empty() || rhs.empty() || lhs.type() != CV_32F || rhs.type() != CV_32F || lhs.cols != rhs.cols ||
        lhs.rows != 1 || rhs.rows != 1) {
        return false;
    }
    score = lhs.dot(rhs);
    return std::isfinite(score);
}

std::vector<StereoMatchPair> SelectGridBalancedPairs(const std::vector<StereoMatchPair> &matches,
                                                     const std::vector<cv::Point2f> &leftKeypoints,
                                                     int imageWidth,
                                                     int imageHeight)
{
    if (matches.empty() || imageWidth <= 0 || imageHeight <= 0) {
        return matches;
    }

    const int cellWidth = std::max(1, (imageWidth + kStereoGridCols - 1) / kStereoGridCols);
    const int cellHeight = std::max(1, (imageHeight + kStereoGridRows - 1) / kStereoGridRows);
    const int maxPairsPerCell = EnvIntValueClamped("SMART_DRONE_EXTERNAL_STEREO_MAX_PAIRS_PER_CELL",
                                                   kStereoMaxPairsPerCell, 1, kStereoMaxPairsPerCellLimit);
    std::vector<int> cellCounts(static_cast<size_t>(kStereoGridCols * kStereoGridRows), 0);
    std::vector<StereoMatchPair> selected;
    selected.reserve(matches.size());

    for (const StereoMatchPair &match : matches) {
        const cv::Point2f &pt = leftKeypoints[static_cast<size_t>(match.leftIndex)];
        const int col = std::clamp(static_cast<int>(pt.x) / cellWidth, 0, kStereoGridCols - 1);
        const int row = std::clamp(static_cast<int>(pt.y) / cellHeight, 0, kStereoGridRows - 1);
        const size_t cellIndex = static_cast<size_t>(row * kStereoGridCols + col);
        if (cellCounts[cellIndex] >= maxPairsPerCell) {
            continue;
        }
        ++cellCounts[cellIndex];
        selected.push_back(match);
    }
    return selected;
}

} // namespace

bool HasValidExternalFeatureDescriptors(const ExternalFeatureSet &features)
{
    return !features.descriptors.empty() && features.descriptors.type() == CV_32F &&
           features.descriptors.rows == static_cast<int>(features.keypoints.size()) && features.descriptors.cols > 0;
}

void LimitStereoPairsInPlace(std::vector<cv::Point2f> &leftPoints, std::vector<cv::Point2f> &rightPoints, size_t maxCount)
{
    const size_t limitedCount = std::min({maxCount, leftPoints.size(), rightPoints.size()});
    leftPoints.resize(limitedCount);
    rightPoints.resize(limitedCount);
}

std::vector<StereoMatchPair> MatchStereoPairs(const ExternalFeatureSet &left,
                                              const ExternalFeatureSet &right,
                                              const cv::Mat &leftGray,
                                              const cv::Mat &rightGray)
{
    std::vector<StereoMatchPair> matches;
    if (!HasValidExternalFeatureDescriptors(left) || !HasValidExternalFeatureDescriptors(right) ||
        left.keypoints.empty() || right.keypoints.empty()) {
        return matches;
    }

    cv::Mat leftGray32f;
    cv::Mat rightGray32f;
    leftGray.convertTo(leftGray32f, CV_32F);
    rightGray.convertTo(rightGray32f, CV_32F);

    std::vector<StereoMatchPair> bestForLeft(static_cast<size_t>(left.descriptors.rows));
    std::vector<int> bestLeftForRight(static_cast<size_t>(right.descriptors.rows), -1);
    std::vector<float> bestLeftScore(static_cast<size_t>(right.descriptors.rows),
                                     -std::numeric_limits<float>::infinity());
    std::vector<float> bestLeftZncc(static_cast<size_t>(right.descriptors.rows), -1.0f);

    for (int li = 0; li < left.descriptors.rows; ++li) {
        const cv::Point2f &leftPt = left.keypoints[static_cast<size_t>(li)];
        int bestRi = -1;
        float bestScore = -std::numeric_limits<float>::infinity();
        float secondScore = -std::numeric_limits<float>::infinity();
        float bestZncc = -1.0f;
        float bestDisparity = 0.0f;

        for (int ri = 0; ri < right.descriptors.rows; ++ri) {
            const cv::Point2f &rightPt = right.keypoints[static_cast<size_t>(ri)];
            const float yDelta = std::fabs(leftPt.y - rightPt.y);
            const float disparity = leftPt.x - rightPt.x;
            if (yDelta > kStereoMaxEpipolarDeltaPx || disparity < ExternalStereoMinDisparityPx() ||
                disparity > kStereoMaxDisparityPx) {
                continue;
            }

            float score = -std::numeric_limits<float>::infinity();
            if (!DescriptorSimilarity(left.descriptors.row(li), right.descriptors.row(ri), score)) {
                continue;
            }
            if (score > bestScore) {
                secondScore = bestScore;
                bestScore = score;
                bestRi = ri;
                bestDisparity = disparity;
            } else if (score > secondScore) {
                secondScore = score;
            }
        }

        if (bestRi < 0 || !std::isfinite(bestScore) || bestScore < kStereoMinDescriptorSimilarity) {
            continue;
        }
        if (std::isfinite(secondScore) && bestScore < secondScore / kStereoSimilarityRatioTest) {
            continue;
        }

        float zncc = -1.0f;
        if (!ComputePatchZncc(leftGray32f, leftPt, rightGray32f, right.keypoints[static_cast<size_t>(bestRi)], zncc) ||
            zncc < kStereoMinZnccScore) {
            continue;
        }

        bestZncc = zncc;
        const float epipolarError = std::fabs(leftPt.y - right.keypoints[static_cast<size_t>(bestRi)].y);
        const float quality = ComputeStereoCandidateQuality(bestScore, bestZncc, epipolarError, bestDisparity);
        if (quality < kExternalStereoMinCandidateQuality) {
            continue;
        }
        bestForLeft[static_cast<size_t>(li)] = StereoMatchPair{li, bestRi, bestScore, bestZncc, bestDisparity, quality};

        if (IsBetterRightCandidate(bestScore, bestZncc, bestLeftScore[static_cast<size_t>(bestRi)],
                                   bestLeftZncc[static_cast<size_t>(bestRi)])) {
            bestLeftScore[static_cast<size_t>(bestRi)] = bestScore;
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
        if (a.descriptorScore != b.descriptorScore) {
            return a.descriptorScore > b.descriptorScore;
        }
        if (a.zncc != b.zncc) {
            return a.zncc > b.zncc;
        }
        if (a.quality != b.quality) {
            return a.quality > b.quality;
        }
        return a.disparity < b.disparity;
    });
    return SelectGridBalancedPairs(matches, left.keypoints, leftGray.cols, leftGray.rows);
}

std::vector<StereoMatchPair> BuildAlignedStereoPairs(const ExternalFeatureSet &left,
                                                     const ExternalFeatureSet &right,
                                                     const cv::Mat &leftGray,
                                                     const cv::Mat &rightGray)
{
    std::vector<StereoMatchPair> matches;
    const size_t pairCount = std::min(left.keypoints.size(), right.keypoints.size());
    if (pairCount == 0 || leftGray.empty() || rightGray.empty()) {
        return matches;
    }

    cv::Mat leftGray32f;
    cv::Mat rightGray32f;
    leftGray.convertTo(leftGray32f, CV_32F);
    rightGray.convertTo(rightGray32f, CV_32F);

    matches.reserve(pairCount);
    for (size_t i = 0; i < pairCount; ++i) {
        const cv::Point2f &leftPt = left.keypoints[i];
        const cv::Point2f &rightPt = right.keypoints[i];
        const float yDelta = std::fabs(leftPt.y - rightPt.y);
        const float disparity = leftPt.x - rightPt.x;
        if (yDelta > kStereoMaxEpipolarDeltaPx || disparity < ExternalStereoMinDisparityPx() ||
            disparity > kStereoMaxDisparityPx) {
            continue;
        }

        float zncc = -1.0f;
        if (!ComputePatchZncc(leftGray32f, leftPt, rightGray32f, rightPt, zncc) || zncc < kStereoMinZnccScore) {
            continue;
        }

        const float quality = ComputeStereoCandidateQuality(1.0f, zncc, yDelta, disparity);
        if (quality < kExternalStereoMinCandidateQuality) {
            continue;
        }
        matches.push_back(
            StereoMatchPair{static_cast<int>(i), static_cast<int>(i), 1.0f, zncc, disparity, quality});
    }

    std::sort(matches.begin(), matches.end(), [](const StereoMatchPair &a, const StereoMatchPair &b) {
        if (a.quality != b.quality) {
            return a.quality > b.quality;
        }
        if (a.zncc != b.zncc) {
            return a.zncc > b.zncc;
        }
        return a.disparity < b.disparity;
    });
    return SelectGridBalancedPairs(matches, left.keypoints, leftGray.cols, leftGray.rows);
}

} // namespace smartdrone::adapters::slam
