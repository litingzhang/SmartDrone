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

struct ZnccSearchRange {
  int minRightX{0};
  int maxRightX{-1};
};

struct ZnccSearchRequest {
  const cv::Mat *leftGray32f{nullptr};
  const cv::Point2f *leftPt{nullptr};
  const cv::Mat *rightGray32f{nullptr};
  ZnccSearchRange range;
  float rankPenaltyScale{0.0f};
  float expectedDisparity{0.0f};
  bool rankAroundExpectedDisparity{false};
};

bool HasZnccImages(const cv::Mat &leftGray32f, const cv::Mat &rightGray32f) {
  return !leftGray32f.empty() && !rightGray32f.empty();
}

bool IsPatchCenterInside(const cv::Mat &image, const cv::Point2f &point) {
  return point.x >= static_cast<float>(kStereoPatchRadiusPx) &&
         point.y >= static_cast<float>(kStereoPatchRadiusPx) &&
         point.x < static_cast<float>(image.cols - kStereoPatchRadiusPx) &&
         point.y < static_cast<float>(image.rows - kStereoPatchRadiusPx);
}

int RightImageMaxPatchCenterX(const cv::Mat &rightGray32f) {
  return rightGray32f.cols - kStereoPatchRadiusPx - 1;
}

ZnccSearchRange MakeFullDisparitySearchRange(const cv::Mat &rightGray32f,
                                             const cv::Point2f &leftPt) {
  const int minRightX = static_cast<int>(
      std::ceil(std::max(static_cast<float>(kStereoPatchRadiusPx),
                         leftPt.x - kStereoMaxDisparityPx)));
  const int maxRightX = static_cast<int>(std::floor(std::min(
      leftPt.x - StereoMinDisparityPx(),
      static_cast<float>(RightImageMaxPatchCenterX(rightGray32f)))));
  return {minRightX, maxRightX};
}

ZnccSearchRange MakeRefineSearchRange(const cv::Mat &rightGray32f,
                                      const cv::Point2f &leftPt,
                                      const cv::Point2f &predictedRightPt) {
  const float minRightX = std::max(static_cast<float>(kStereoPatchRadiusPx),
                                   leftPt.x - kStereoMaxDisparityPx);
  const float maxRightX = std::min(
      leftPt.x - StereoMinDisparityPx(),
      static_cast<float>(RightImageMaxPatchCenterX(rightGray32f)));
  if (minRightX > maxRightX) {
    return {1, 0};
  }
  return {static_cast<int>(std::floor(std::max(
              minRightX, predictedRightPt.x - kStereoZnccRefineSearchRadiusPx))),
          static_cast<int>(std::ceil(std::min(
              maxRightX, predictedRightPt.x + kStereoZnccRefineSearchRadiusPx)))};
}

ZnccSearchRange MakeExpectedDisparitySearchRange(
    const cv::Mat &rightGray32f, const cv::Point2f &leftPt,
    float expectedDisparity) {
  const float centerRightX = leftPt.x - expectedDisparity;
  const int minRightX = static_cast<int>(std::ceil(
      std::max({static_cast<float>(kStereoPatchRadiusPx),
                leftPt.x - kStereoMaxDisparityPx,
                centerRightX - kStereoZnccAroundDisparitySearchRadiusPx})));
  const int maxRightX = static_cast<int>(std::floor(std::min(
      {leftPt.x - StereoMinDisparityPx(),
       static_cast<float>(RightImageMaxPatchCenterX(rightGray32f)),
       centerRightX + kStereoZnccAroundDisparitySearchRadiusPx})));
  return {minRightX, maxRightX};
}

float RankStereoZnccCandidate(const ZnccSearchRequest &request,
                              const cv::Point2f &rightPt, float score) {
  const float disparity = request.leftPt->x - rightPt.x;
  if (request.rankAroundExpectedDisparity) {
    return score - request.rankPenaltyScale *
                       std::fabs(disparity - request.expectedDisparity);
  }
  return score - request.rankPenaltyScale * disparity;
}

bool SearchBestZnccRightPoint(const ZnccSearchRequest &request,
                              cv::Point2f &rightPt, float &bestScore,
                              float *selectedRank = nullptr) {
  float bestRank = bestScore < -0.5f
                       ? -1.0f
                       : RankStereoZnccCandidate(request, rightPt, bestScore);
  const float rightY = request.leftPt->y;
  for (int x = request.range.minRightX; x <= request.range.maxRightX; ++x) {
    const cv::Point2f candidate(static_cast<float>(x), rightY);
    float score = -1.0f;
    if (!ComputePatchZncc(*request.leftGray32f, *request.leftPt,
                          *request.rightGray32f, candidate, score)) {
      continue;
    }
    const float rank = RankStereoZnccCandidate(request, candidate, score);
    if (rank > bestRank) {
      bestRank = rank;
      bestScore = score;
      rightPt = candidate;
    }
  }
  if (selectedRank != nullptr) {
    *selectedRank = bestRank;
  }
  return bestScore >= kTemporalStereoMinZnccScore &&
         IsStereoPairGeometricallyValid(*request.leftPt, rightPt);
}

} // namespace

float StereoMinDisparityPx() {
  return EnvFloatValueClamped("SMART_DRONE_STEREO_FEATURE_MIN_DISPARITY_PX",
                              kStereoMinDisparityPx, 0.05f,
                              kStereoMaxDisparityPx);
}

bool ComputePatchZncc(const cv::Mat &leftGray32f, const cv::Point2f &leftPt,
                      const cv::Mat &rightGray32f, const cv::Point2f &rightPt,
                      float &score) {
  score = -1.0f;
  if (!HasZnccImages(leftGray32f, rightGray32f)) {
    return false;
  }

  const int patchSize = 2 * kStereoPatchRadiusPx + 1;
  if (!IsPatchCenterInside(leftGray32f, leftPt) ||
      !IsPatchCenterInside(rightGray32f, rightPt)) {
    return false;
  }

  cv::Mat leftPatch;
  cv::Mat rightPatch;
  cv::getRectSubPix(leftGray32f, cv::Size(patchSize, patchSize), leftPt,
                    leftPatch);
  cv::getRectSubPix(rightGray32f, cv::Size(patchSize, patchSize), rightPt,
                    rightPatch);
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
  const double denom =
      static_cast<double>(leftNorm.total()) * leftStd[0] * rightStd[0];
  if (denom <= 1e-9) {
    return false;
  }
  score = static_cast<float>(leftNorm.dot(rightNorm) / denom);
  return std::isfinite(score);
}

bool IsStereoPairGeometricallyValid(const cv::Point2f &leftPt,
                                    const cv::Point2f &rightPt) {
  const float yDelta = std::fabs(leftPt.y - rightPt.y);
  const float disparity = leftPt.x - rightPt.x;
  return yDelta <= kStereoMaxEpipolarDeltaPx &&
         disparity >= StereoMinDisparityPx() &&
         disparity <= kStereoMaxDisparityPx;
}

bool RefineRightPointByStereoZncc(const cv::Mat &leftGray32f,
                                  const cv::Point2f &leftPt,
                                  const cv::Mat &rightGray32f,
                                  const cv::Point2f &predictedRightPt,
                                  cv::Point2f &refinedRightPt,
                                  float &bestScore) {
  bestScore = -1.0f;
  refinedRightPt = predictedRightPt;
  if (!HasZnccImages(leftGray32f, rightGray32f)) {
    return false;
  }

  const ZnccSearchRange range =
      MakeRefineSearchRange(rightGray32f, leftPt, predictedRightPt);
  if (range.minRightX > range.maxRightX) {
    return false;
  }

  float predictedScore = -1.0f;
  const bool havePredictedScore =
      IsStereoPairGeometricallyValid(leftPt, predictedRightPt) &&
      ComputePatchZncc(leftGray32f, leftPt, rightGray32f, predictedRightPt,
                       predictedScore) &&
      predictedScore >= kTemporalStereoMinZnccScore;

  float searchScore = havePredictedScore ? predictedScore : -1.0f;
  float searchRank = -1.0f;
  cv::Point2f searchRightPt = refinedRightPt;
  ZnccSearchRequest request{&leftGray32f, &leftPt, &rightGray32f, range,
                            0.015f, leftPt.x - predictedRightPt.x, true};
  if (SearchBestZnccRightPoint(request, searchRightPt, searchScore,
                               &searchRank) &&
      (!havePredictedScore || searchRank > predictedScore + 0.03f)) {
    bestScore = searchScore;
    refinedRightPt = searchRightPt;
  } else if (havePredictedScore) {
    bestScore = predictedScore;
  }
  return bestScore >= kTemporalStereoMinZnccScore &&
         IsStereoPairGeometricallyValid(leftPt, refinedRightPt);
}

bool FindRightPointByStereoZncc(const cv::Mat &leftGray32f,
                                const cv::Point2f &leftPt,
                                const cv::Mat &rightGray32f,
                                cv::Point2f &rightPt, float &bestScore) {
  bestScore = -1.0f;
  if (!HasZnccImages(leftGray32f, rightGray32f)) {
    return false;
  }

  const ZnccSearchRange range = MakeFullDisparitySearchRange(rightGray32f, leftPt);
  if (range.minRightX > range.maxRightX) {
    return false;
  }

  const ZnccSearchRequest request{&leftGray32f, &leftPt, &rightGray32f,
                                  range, 0.0005f, 0.0f, false};
  return SearchBestZnccRightPoint(request, rightPt, bestScore);
}

bool FindRightPointByStereoZnccAroundDisparity(const cv::Mat &leftGray32f,
                                               const cv::Point2f &leftPt,
                                               const cv::Mat &rightGray32f,
                                               float expectedDisparity,
                                               cv::Point2f &rightPt,
                                               float &bestScore) {
  bestScore = -1.0f;
  if (!HasZnccImages(leftGray32f, rightGray32f) ||
      !(expectedDisparity > 0.0f) || !std::isfinite(expectedDisparity)) {
    return FindRightPointByStereoZncc(leftGray32f, leftPt, rightGray32f,
                                      rightPt, bestScore);
  }

  const ZnccSearchRange range =
      MakeExpectedDisparitySearchRange(rightGray32f, leftPt, expectedDisparity);
  if (range.minRightX > range.maxRightX) {
    return FindRightPointByStereoZncc(leftGray32f, leftPt, rightGray32f,
                                      rightPt, bestScore);
  }

  const ZnccSearchRequest request{&leftGray32f, &leftPt, &rightGray32f,
                                  range, 0.0015f, expectedDisparity, true};
  return SearchBestZnccRightPoint(request, rightPt, bestScore);
}

float ComputeStereoCandidateQuality(float descriptorScore, float zncc,
                                    float epipolarErrorPx, float disparity) {
  const float descriptorTerm =
      std::clamp((descriptorScore - kStereoMinDescriptorSimilarity) /
                     std::max(1e-3f, 1.0f - kStereoMinDescriptorSimilarity),
                 0.0f, 1.0f);
  const float znccTerm = std::clamp((zncc + 1.0f) * 0.5f, 0.0f, 1.0f);
  const float epipolarPenalty = std::clamp(
      epipolarErrorPx / std::max(1e-3f, kStereoMaxEpipolarDeltaPx), 0.0f, 1.0f);
  const float disparityPenalty =
      (disparity < 2.0f || disparity > kStereoMaxDisparityPx * 0.85f) ? 0.25f
                                                                      : 0.0f;
  return std::clamp(0.50f * descriptorTerm + 0.35f * znccTerm +
                        0.15f * (1.0f - epipolarPenalty) - disparityPenalty,
                    0.0f, 1.0f);
}

std::vector<core::ports::StereoMatchPair>
FilterStereoPairsByDisparityConsistency(
    const std::vector<core::ports::StereoMatchPair> &matches) {
  if (matches.size() < 8) {
    return matches;
  }

  std::vector<float> disparities;
  disparities.reserve(matches.size());
  for (const core::ports::StereoMatchPair &match : matches) {
    disparities.push_back(match.disparity);
  }

  std::vector<float> sortedDisparities = disparities;
  const auto medianIt =
      sortedDisparities.begin() +
      static_cast<std::ptrdiff_t>(sortedDisparities.size() / 2);
  std::nth_element(sortedDisparities.begin(), medianIt,
                   sortedDisparities.end());
  const float medianDisparity = *medianIt;

  std::vector<float> absDeviation;
  absDeviation.reserve(disparities.size());
  for (const float disparity : disparities) {
    absDeviation.push_back(std::fabs(disparity - medianDisparity));
  }
  auto madIt = absDeviation.begin() +
               static_cast<std::ptrdiff_t>(absDeviation.size() / 2);
  std::nth_element(absDeviation.begin(), madIt, absDeviation.end());
  const float mad = *madIt;
  const float tolerance =
      std::max(kStereoDisparityMinTolerancePx,
               kStereoDisparityMadScale * std::max(mad, 1.0f));

  std::vector<core::ports::StereoMatchPair> filtered;
  filtered.reserve(matches.size());
  for (const core::ports::StereoMatchPair &match : matches) {
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
