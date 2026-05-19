#include "adapters/slam/stereo_pair_builder.h"

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
constexpr float kStereoMinCandidateQuality = 0.18f;

struct DescriptorStereoCandidate {
  int rightIndex{-1};
  float bestScore{-std::numeric_limits<float>::infinity()};
  float secondScore{-std::numeric_limits<float>::infinity()};
  float disparity{0.0f};
};

struct DescriptorStereoSelection {
  std::vector<core::ports::StereoMatchPair> bestForLeft;
  std::vector<int> bestLeftForRight;
  std::vector<float> bestLeftScore;
  std::vector<float> bestLeftZncc;
};

struct DescriptorMatchBuildRequest {
  const core::ports::VisualFeatureSet *left{nullptr};
  const core::ports::VisualFeatureSet *right{nullptr};
  const cv::Mat *leftGray32f{nullptr};
  const cv::Mat *rightGray32f{nullptr};
  int leftIndex{0};
};

struct AlignedStereoMatchBuildRequest {
  const cv::Mat *leftGray32f{nullptr};
  const cv::Mat *rightGray32f{nullptr};
  cv::Point2f leftPt;
  cv::Point2f rightPt;
  size_t index{0};
};

bool IsBetterRightCandidate(float candidateScore, float candidateZncc,
                            float currentScore, float currentZncc) {
  if (!std::isfinite(currentScore)) {
    return true;
  }
  if (candidateScore != currentScore) {
    return candidateScore > currentScore;
  }
  return candidateZncc > currentZncc;
}

bool DescriptorSimilarity(const cv::Mat &lhs, const cv::Mat &rhs,
                          float &score) {
  score = -std::numeric_limits<float>::infinity();
  if (lhs.empty() || rhs.empty() || lhs.type() != CV_32F ||
      rhs.type() != CV_32F || lhs.cols != rhs.cols || lhs.rows != 1 ||
      rhs.rows != 1) {
    return false;
  }
  score = lhs.dot(rhs);
  return std::isfinite(score);
}

bool IsStereoCandidateGeometryValid(const cv::Point2f &leftPt,
                                    const cv::Point2f &rightPt,
                                    float &yDelta, float &disparity) {
  yDelta = std::fabs(leftPt.y - rightPt.y);
  disparity = leftPt.x - rightPt.x;
  return yDelta <= kStereoMaxEpipolarDeltaPx &&
         disparity >= StereoMinDisparityPx() &&
         disparity <= kStereoMaxDisparityPx;
}

bool IsDescriptorRatioAccepted(float bestScore, float secondScore) {
  if (!std::isfinite(bestScore) ||
      bestScore < kStereoMinDescriptorSimilarity) {
    return false;
  }
  return !std::isfinite(secondScore) ||
         bestScore >= secondScore / kStereoSimilarityRatioTest;
}

bool IsDescriptorMatchOrderBetter(float score,
                                  const DescriptorStereoCandidate &best) {
  return score > best.bestScore;
}

void UpdateDescriptorCandidate(float score, float disparity, int rightIndex,
                               DescriptorStereoCandidate &best) {
  if (IsDescriptorMatchOrderBetter(score, best)) {
    best.secondScore = best.bestScore;
    best.bestScore = score;
    best.rightIndex = rightIndex;
    best.disparity = disparity;
    return;
  }
  if (score > best.secondScore) {
    best.secondScore = score;
  }
}

DescriptorStereoCandidate FindBestDescriptorCandidate(
    const core::ports::VisualFeatureSet &left,
    const core::ports::VisualFeatureSet &right, int leftIndex) {
  DescriptorStereoCandidate best;
  const cv::Point2f &leftPt = left.keypoints[static_cast<size_t>(leftIndex)];
  for (int ri = 0; ri < right.descriptors.rows; ++ri) {
    float yDelta = 0.0f;
    float disparity = 0.0f;
    const cv::Point2f &rightPt = right.keypoints[static_cast<size_t>(ri)];
    if (!IsStereoCandidateGeometryValid(leftPt, rightPt, yDelta, disparity)) {
      continue;
    }

    float score = -std::numeric_limits<float>::infinity();
    if (DescriptorSimilarity(left.descriptors.row(leftIndex),
                             right.descriptors.row(ri), score)) {
      UpdateDescriptorCandidate(score, disparity, ri, best);
    }
  }
  return best;
}

bool BuildDescriptorMatchForLeft(const DescriptorMatchBuildRequest &request,
                                 core::ports::StereoMatchPair &match) {
  const core::ports::VisualFeatureSet &left = *request.left;
  const core::ports::VisualFeatureSet &right = *request.right;
  const DescriptorStereoCandidate best =
      FindBestDescriptorCandidate(left, right, request.leftIndex);
  if (best.rightIndex < 0 ||
      !IsDescriptorRatioAccepted(best.bestScore, best.secondScore)) {
    return false;
  }

  const cv::Point2f &leftPt =
      left.keypoints[static_cast<size_t>(request.leftIndex)];
  const cv::Point2f &rightPt =
      right.keypoints[static_cast<size_t>(best.rightIndex)];
  float zncc = -1.0f;
  if (!ComputePatchZncc(*request.leftGray32f, leftPt, *request.rightGray32f,
                        rightPt, zncc) ||
      zncc < kStereoMinZnccScore) {
    return false;
  }

  const float epipolarError = std::fabs(leftPt.y - rightPt.y);
  const float quality =
      ComputeStereoCandidateQuality(best.bestScore, zncc, epipolarError,
                                    best.disparity);
  if (quality < kStereoMinCandidateQuality) {
    return false;
  }
  match = core::ports::StereoMatchPair{request.leftIndex, best.rightIndex,
                                       best.bestScore, zncc, best.disparity,
                                       quality};
  return true;
}

DescriptorStereoSelection MakeDescriptorStereoSelection(int leftRows,
                                                        int rightRows) {
  DescriptorStereoSelection selection;
  selection.bestForLeft.resize(static_cast<size_t>(leftRows));
  selection.bestLeftForRight.assign(static_cast<size_t>(rightRows), -1);
  selection.bestLeftScore.assign(
      static_cast<size_t>(rightRows),
      -std::numeric_limits<float>::infinity());
  selection.bestLeftZncc.assign(static_cast<size_t>(rightRows), -1.0f);
  return selection;
}

void UpdateBestRightDescriptorMatch(const core::ports::StereoMatchPair &match,
                                    DescriptorStereoSelection &selection) {
  const size_t rightIndex = static_cast<size_t>(match.rightIndex);
  if (!IsBetterRightCandidate(match.descriptorScore, match.zncc,
                              selection.bestLeftScore[rightIndex],
                              selection.bestLeftZncc[rightIndex])) {
    return;
  }
  selection.bestLeftScore[rightIndex] = match.descriptorScore;
  selection.bestLeftZncc[rightIndex] = match.zncc;
  selection.bestLeftForRight[rightIndex] = match.leftIndex;
}

std::vector<core::ports::StereoMatchPair> CollectMutualDescriptorMatches(
    const DescriptorStereoSelection &selection, int leftRows, int rightRows) {
  std::vector<core::ports::StereoMatchPair> matches;
  matches.reserve(static_cast<size_t>(std::min(leftRows, rightRows)));
  for (const core::ports::StereoMatchPair &pair : selection.bestForLeft) {
    if (pair.rightIndex < 0) {
      continue;
    }
    if (selection.bestLeftForRight[static_cast<size_t>(pair.rightIndex)] ==
        pair.leftIndex) {
      matches.push_back(pair);
    }
  }
  return matches;
}

void SortDescriptorMatches(std::vector<core::ports::StereoMatchPair> &matches) {
  std::sort(matches.begin(), matches.end(),
            [](const core::ports::StereoMatchPair &a,
               const core::ports::StereoMatchPair &b) {
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
}

bool BuildAlignedStereoMatch(const AlignedStereoMatchBuildRequest &request,
                             core::ports::StereoMatchPair &match) {
  float yDelta = 0.0f;
  float disparity = 0.0f;
  if (!IsStereoCandidateGeometryValid(request.leftPt, request.rightPt, yDelta,
                                      disparity)) {
    return false;
  }

  float zncc = -1.0f;
  if (!ComputePatchZncc(*request.leftGray32f, request.leftPt,
                        *request.rightGray32f, request.rightPt, zncc) ||
      zncc < kStereoMinZnccScore) {
    return false;
  }

  const float quality =
      ComputeStereoCandidateQuality(1.0f, zncc, yDelta, disparity);
  if (quality < kStereoMinCandidateQuality) {
    return false;
  }
  match = core::ports::StereoMatchPair{
      static_cast<int>(request.index), static_cast<int>(request.index), 1.0f,
      zncc, disparity, quality};
  return true;
}

void SortAlignedMatches(std::vector<core::ports::StereoMatchPair> &matches) {
  std::sort(matches.begin(), matches.end(),
            [](const core::ports::StereoMatchPair &a,
               const core::ports::StereoMatchPair &b) {
              if (a.quality != b.quality) {
                return a.quality > b.quality;
              }
              if (a.zncc != b.zncc) {
                return a.zncc > b.zncc;
              }
              return a.disparity < b.disparity;
            });
}

std::vector<core::ports::StereoMatchPair> SelectGridBalancedPairs(
    const std::vector<core::ports::StereoMatchPair> &matches,
    const std::vector<cv::Point2f> &leftKeypoints, int imageWidth,
    int imageHeight) {
  if (matches.empty() || imageWidth <= 0 || imageHeight <= 0) {
    return matches;
  }

  const int cellWidth =
      std::max(1, (imageWidth + kStereoGridCols - 1) / kStereoGridCols);
  const int cellHeight =
      std::max(1, (imageHeight + kStereoGridRows - 1) / kStereoGridRows);
  const int maxPairsPerCell = EnvIntValueClamped(
      "SMART_DRONE_STEREO_FEATURE_MAX_PAIRS_PER_CELL", kStereoMaxPairsPerCell,
      1, kStereoMaxPairsPerCellLimit);
  std::vector<int> cellCounts(
      static_cast<size_t>(kStereoGridCols * kStereoGridRows), 0);
  std::vector<core::ports::StereoMatchPair> selected;
  selected.reserve(matches.size());

  for (const core::ports::StereoMatchPair &match : matches) {
    const cv::Point2f &pt = leftKeypoints[static_cast<size_t>(match.leftIndex)];
    const int col =
        std::clamp(static_cast<int>(pt.x) / cellWidth, 0, kStereoGridCols - 1);
    const int row =
        std::clamp(static_cast<int>(pt.y) / cellHeight, 0, kStereoGridRows - 1);
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

bool HasValidVisualFeatureDescriptors(
    const core::ports::VisualFeatureSet &features) {
  return !features.descriptors.empty() &&
         features.descriptors.type() == CV_32F &&
         features.descriptors.rows ==
             static_cast<int>(features.keypoints.size()) &&
         features.descriptors.cols > 0;
}

bool DefaultStereoPairBuilder::BuildPairs(
    const core::ports::StereoPairBuildInput &input,
    core::ports::StereoPairBuildResult &result) const {
  result = core::ports::StereoPairBuildResult{};
  if (input.leftFeatures == nullptr || input.rightFeatures == nullptr ||
      input.leftPrepared == nullptr || input.rightPrepared == nullptr) {
    return false;
  }
  if (input.mode == core::ports::StereoPairBuildMode::DescriptorSearch) {
    result.matches =
        MatchStereoPairs(*input.leftFeatures, *input.rightFeatures,
                         *input.leftPrepared, *input.rightPrepared);
  } else {
    result.matches =
        BuildAlignedStereoPairs(*input.leftFeatures, *input.rightFeatures,
                                *input.leftPrepared, *input.rightPrepared);
  }
  return true;
}

void LimitStereoPairsInPlace(std::vector<cv::Point2f> &leftPoints,
                             std::vector<cv::Point2f> &rightPoints,
                             size_t maxCount) {
  const size_t limitedCount =
      std::min({maxCount, leftPoints.size(), rightPoints.size()});
  leftPoints.resize(limitedCount);
  rightPoints.resize(limitedCount);
}

std::vector<core::ports::StereoMatchPair>
MatchStereoPairs(const core::ports::VisualFeatureSet &left,
                 const core::ports::VisualFeatureSet &right,
                 const cv::Mat &leftGray, const cv::Mat &rightGray) {
  std::vector<core::ports::StereoMatchPair> matches;
  if (!HasValidVisualFeatureDescriptors(left) ||
      !HasValidVisualFeatureDescriptors(right) || left.keypoints.empty() ||
      right.keypoints.empty()) {
    return matches;
  }

  cv::Mat leftGray32f;
  cv::Mat rightGray32f;
  leftGray.convertTo(leftGray32f, CV_32F);
  rightGray.convertTo(rightGray32f, CV_32F);

  DescriptorStereoSelection selection = MakeDescriptorStereoSelection(
      left.descriptors.rows, right.descriptors.rows);
  for (int li = 0; li < left.descriptors.rows; ++li) {
    core::ports::StereoMatchPair match;
    const DescriptorMatchBuildRequest request{&left, &right, &leftGray32f,
                                              &rightGray32f, li};
    if (!BuildDescriptorMatchForLeft(request, match)) {
      continue;
    }
    selection.bestForLeft[static_cast<size_t>(li)] = match;
    UpdateBestRightDescriptorMatch(match, selection);
  }

  matches = CollectMutualDescriptorMatches(selection, left.descriptors.rows,
                                           right.descriptors.rows);
  SortDescriptorMatches(matches);
  return SelectGridBalancedPairs(matches, left.keypoints, leftGray.cols,
                                 leftGray.rows);
}

std::vector<core::ports::StereoMatchPair>
BuildAlignedStereoPairs(const core::ports::VisualFeatureSet &left,
                        const core::ports::VisualFeatureSet &right,
                        const cv::Mat &leftGray, const cv::Mat &rightGray) {
  std::vector<core::ports::StereoMatchPair> matches;
  const size_t pairCount =
      std::min(left.keypoints.size(), right.keypoints.size());
  if (pairCount == 0 || leftGray.empty() || rightGray.empty()) {
    return matches;
  }

  cv::Mat leftGray32f;
  cv::Mat rightGray32f;
  leftGray.convertTo(leftGray32f, CV_32F);
  rightGray.convertTo(rightGray32f, CV_32F);

  matches.reserve(pairCount);
  for (size_t i = 0; i < pairCount; ++i) {
    core::ports::StereoMatchPair match;
    const AlignedStereoMatchBuildRequest request{
        &leftGray32f, &rightGray32f, left.keypoints[i], right.keypoints[i], i};
    if (BuildAlignedStereoMatch(request, match)) {
      matches.push_back(match);
    }
  }

  SortAlignedMatches(matches);
  return SelectGridBalancedPairs(matches, left.keypoints, leftGray.cols,
                                 leftGray.rows);
}

} // namespace smartdrone::adapters::slam
