#include "adapters/slam/stereo_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

#include "adapters/slam/slam_env.h"

namespace smartdrone::adapters::slam {

namespace {

constexpr float kStereoZnccRefineSearchRadiusPx = 10.0f;
constexpr float kStereoZnccAroundDisparitySearchRadiusPx = 96.0f;

} // namespace

float ExternalStereoMinDisparityPx()
{
    return EnvFloatValueClamped("SMART_DRONE_EXTERNAL_STEREO_MIN_DISPARITY_PX",
                                kStereoMinDisparityPx, 0.05f, kStereoMaxDisparityPx);
}

bool ComputePatchZncc(const cv::Mat &leftGray32f, const cv::Point2f &leftPt,
                      const cv::Mat &rightGray32f, const cv::Point2f &rightPt,
                      float &score)
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

bool IsStereoPairGeometricallyValid(const cv::Point2f &leftPt, const cv::Point2f &rightPt)
{
    const float yDelta = std::fabs(leftPt.y - rightPt.y);
    const float disparity = leftPt.x - rightPt.x;
    return yDelta <= kStereoMaxEpipolarDeltaPx && disparity >= ExternalStereoMinDisparityPx() &&
           disparity <= kStereoMaxDisparityPx;
}

bool RefineRightPointByStereoZncc(const cv::Mat &leftGray32f, const cv::Point2f &leftPt,
                                  const cv::Mat &rightGray32f, const cv::Point2f &predictedRightPt,
                                  cv::Point2f &refinedRightPt, float &bestScore)
{
    bestScore = -1.0f;
    refinedRightPt = predictedRightPt;
    if (leftGray32f.empty() || rightGray32f.empty()) {
        return false;
    }

    const float minRightX = std::max(static_cast<float>(kStereoPatchRadiusPx), leftPt.x - kStereoMaxDisparityPx);
    const float maxRightX = std::min(leftPt.x - ExternalStereoMinDisparityPx(),
                                     static_cast<float>(rightGray32f.cols - kStereoPatchRadiusPx - 1));
    if (minRightX > maxRightX) {
        return false;
    }

    float predictedScore = -1.0f;
    const bool havePredictedScore = IsStereoPairGeometricallyValid(leftPt, predictedRightPt) &&
                                    ComputePatchZncc(leftGray32f, leftPt, rightGray32f, predictedRightPt,
                                                     predictedScore) &&
                                    predictedScore >= kTemporalStereoMinZnccScore;
    float bestRank = havePredictedScore ? predictedScore : -1.0f;

    const int searchStart =
        static_cast<int>(std::floor(std::max(minRightX, predictedRightPt.x - kStereoZnccRefineSearchRadiusPx)));
    const int searchEnd =
        static_cast<int>(std::ceil(std::min(maxRightX, predictedRightPt.x + kStereoZnccRefineSearchRadiusPx)));
    const float rightY = leftPt.y;
    for (int x = searchStart; x <= searchEnd; ++x) {
        const cv::Point2f candidate(static_cast<float>(x), rightY);
        float score = -1.0f;
        if (!ComputePatchZncc(leftGray32f, leftPt, rightGray32f, candidate, score)) {
            continue;
        }
        const float rank = score - 0.015f * std::fabs(candidate.x - predictedRightPt.x);
        if (rank > bestRank + 0.03f) {
            bestRank = rank;
            bestScore = score;
            refinedRightPt = candidate;
        }
    }
    if (bestScore < 0.0f && havePredictedScore) {
        bestScore = predictedScore;
    }
    return bestScore >= kTemporalStereoMinZnccScore && IsStereoPairGeometricallyValid(leftPt, refinedRightPt);
}

bool FindRightPointByStereoZncc(const cv::Mat &leftGray32f, const cv::Point2f &leftPt,
                                const cv::Mat &rightGray32f, cv::Point2f &rightPt,
                                float &bestScore)
{
    bestScore = -1.0f;
    if (leftGray32f.empty() || rightGray32f.empty()) {
        return false;
    }

    const int minRightX =
        static_cast<int>(std::ceil(std::max(static_cast<float>(kStereoPatchRadiusPx), leftPt.x - kStereoMaxDisparityPx)));
    const int maxRightX = static_cast<int>(std::floor(std::min(leftPt.x - ExternalStereoMinDisparityPx(),
                                                               static_cast<float>(rightGray32f.cols -
                                                                                  kStereoPatchRadiusPx - 1))));
    if (minRightX > maxRightX) {
        return false;
    }

    const float rightY = leftPt.y;
    for (int x = minRightX; x <= maxRightX; ++x) {
        const cv::Point2f candidate(static_cast<float>(x), rightY);
        float score = -1.0f;
        if (!ComputePatchZncc(leftGray32f, leftPt, rightGray32f, candidate, score)) {
            continue;
        }
        const float disparity = leftPt.x - candidate.x;
        const float rank = score - 0.0005f * disparity;
        const float bestRank = bestScore < -0.5f ? -1.0f : bestScore - 0.0005f * (leftPt.x - rightPt.x);
        if (rank > bestRank) {
            bestScore = score;
            rightPt = candidate;
        }
    }
    return bestScore >= kTemporalStereoMinZnccScore && IsStereoPairGeometricallyValid(leftPt, rightPt);
}

bool FindRightPointByStereoZnccAroundDisparity(const cv::Mat &leftGray32f,
                                               const cv::Point2f &leftPt,
                                               const cv::Mat &rightGray32f,
                                               float expectedDisparity,
                                               cv::Point2f &rightPt,
                                               float &bestScore)
{
    bestScore = -1.0f;
    if (leftGray32f.empty() || rightGray32f.empty() || !(expectedDisparity > 0.0f) ||
        !std::isfinite(expectedDisparity)) {
        return FindRightPointByStereoZncc(leftGray32f, leftPt, rightGray32f, rightPt, bestScore);
    }

    const float centerRightX = leftPt.x - expectedDisparity;
    const int minRightX = static_cast<int>(std::ceil(std::max(
        {static_cast<float>(kStereoPatchRadiusPx), leftPt.x - kStereoMaxDisparityPx,
         centerRightX - kStereoZnccAroundDisparitySearchRadiusPx})));
    const int maxRightX = static_cast<int>(std::floor(std::min(
        {leftPt.x - ExternalStereoMinDisparityPx(), static_cast<float>(rightGray32f.cols - kStereoPatchRadiusPx - 1),
         centerRightX + kStereoZnccAroundDisparitySearchRadiusPx})));
    if (minRightX > maxRightX) {
        return FindRightPointByStereoZncc(leftGray32f, leftPt, rightGray32f, rightPt, bestScore);
    }

    const float rightY = leftPt.y;
    for (int x = minRightX; x <= maxRightX; ++x) {
        const cv::Point2f candidate(static_cast<float>(x), rightY);
        float score = -1.0f;
        if (!ComputePatchZncc(leftGray32f, leftPt, rightGray32f, candidate, score)) {
            continue;
        }
        const float disparity = leftPt.x - candidate.x;
        const float rank = score - 0.0015f * std::fabs(disparity - expectedDisparity);
        const float bestRank =
            bestScore < -0.5f ? -1.0f : bestScore - 0.0015f * std::fabs((leftPt.x - rightPt.x) - expectedDisparity);
        if (rank > bestRank) {
            bestScore = score;
            rightPt = candidate;
        }
    }
    return bestScore >= kTemporalStereoMinZnccScore && IsStereoPairGeometricallyValid(leftPt, rightPt);
}

float ComputeStereoCandidateQuality(float descriptorScore, float zncc,
                                    float epipolarErrorPx, float disparity)
{
    const float descriptorTerm = std::clamp((descriptorScore - kStereoMinDescriptorSimilarity) /
                                                std::max(1e-3f, 1.0f - kStereoMinDescriptorSimilarity),
                                            0.0f, 1.0f);
    const float znccTerm = std::clamp((zncc + 1.0f) * 0.5f, 0.0f, 1.0f);
    const float epipolarPenalty = std::clamp(epipolarErrorPx / std::max(1e-3f, kStereoMaxEpipolarDeltaPx),
                                             0.0f, 1.0f);
    const float disparityPenalty =
        (disparity < 2.0f || disparity > kStereoMaxDisparityPx * 0.85f) ? 0.25f : 0.0f;
    return std::clamp(0.50f * descriptorTerm + 0.35f * znccTerm + 0.15f * (1.0f - epipolarPenalty) -
                          disparityPenalty,
                      0.0f, 1.0f);
}

std::vector<StereoMatchPair> FilterStereoPairsByDisparityConsistency(
    const std::vector<StereoMatchPair> &matches)
{
    if (matches.size() < 8) {
        return matches;
    }

    std::vector<float> disparities;
    disparities.reserve(matches.size());
    for (const StereoMatchPair &match : matches) {
        disparities.push_back(match.disparity);
    }

    std::vector<float> sortedDisparities = disparities;
    const auto medianIt = sortedDisparities.begin() + static_cast<std::ptrdiff_t>(sortedDisparities.size() / 2);
    std::nth_element(sortedDisparities.begin(), medianIt, sortedDisparities.end());
    const float medianDisparity = *medianIt;

    std::vector<float> absDeviation;
    absDeviation.reserve(disparities.size());
    for (const float disparity : disparities) {
        absDeviation.push_back(std::fabs(disparity - medianDisparity));
    }
    auto madIt = absDeviation.begin() + static_cast<std::ptrdiff_t>(absDeviation.size() / 2);
    std::nth_element(absDeviation.begin(), madIt, absDeviation.end());
    const float mad = *madIt;
    const float tolerance = std::max(kStereoDisparityMinTolerancePx,
                                     kStereoDisparityMadScale * std::max(mad, 1.0f));

    std::vector<StereoMatchPair> filtered;
    filtered.reserve(matches.size());
    for (const StereoMatchPair &match : matches) {
        if (std::fabs(match.disparity - medianDisparity) <= tolerance) {
            filtered.push_back(match);
        }
    }
    if (filtered.size() < (matches.size() / 2)) {
        return matches;
    }
    return filtered;
}

} // namespace smartdrone::adapters::slam
