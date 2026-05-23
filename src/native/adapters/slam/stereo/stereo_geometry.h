#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "core/ports/visual_feature_data.h"

namespace SmartDrone::Adapters::Slam {

constexpr float kStereoMaxEpipolarDeltaPx = 1.5f;
constexpr float kStereoMinDisparityPx = 0.75f;
constexpr float kStereoMaxDisparityPx = 240.0f;
constexpr float kStereoSimilarityRatioTest = 0.98f;
constexpr float kStereoMinDescriptorSimilarity = 0.20f;
constexpr float kStereoMinZnccScore = 0.10f;
constexpr int kStereoPatchRadiusPx = 3;
constexpr float kTemporalStereoMinZnccScore = 0.05f;
constexpr float kStereoDisparityMadScale = 2.5f;
constexpr float kStereoDisparityMinTolerancePx = 6.0f;

float StereoMinDisparityPx();

bool ComputePatchZncc(const cv::Mat &leftGray32f, const cv::Point2f &leftPt,
                      const cv::Mat &rightGray32f, const cv::Point2f &rightPt,
                      float &score);

bool IsStereoPairGeometricallyValid(const cv::Point2f &leftPt,
                                    const cv::Point2f &rightPt);

bool RefineRightPointByStereoZncc(const cv::Mat &leftGray32f,
                                  const cv::Point2f &leftPt,
                                  const cv::Mat &rightGray32f,
                                  const cv::Point2f &predictedRightPt,
                                  cv::Point2f &refinedRightPt,
                                  float &bestScore);

bool FindRightPointByStereoZncc(const cv::Mat &leftGray32f,
                                const cv::Point2f &leftPt,
                                const cv::Mat &rightGray32f,
                                cv::Point2f &rightPt, float &bestScore);

bool FindRightPointByStereoZnccAroundDisparity(const cv::Mat &leftGray32f,
                                               const cv::Point2f &leftPt,
                                               const cv::Mat &rightGray32f,
                                               float expectedDisparity,
                                               cv::Point2f &rightPt,
                                               float &bestScore);

float ComputeStereoCandidateQuality(float descriptorScore, float zncc,
                                    float epipolarErrorPx, float disparity);

std::vector<Core::Ports::StereoMatchPair>
FilterStereoPairsByDisparityConsistency(
    const std::vector<Core::Ports::StereoMatchPair> &matches);

} // namespace SmartDrone::Adapters::Slam
