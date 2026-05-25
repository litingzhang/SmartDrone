#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "core/ports/visual_feature_data.h"

namespace SmartDrone::Adapters::Slam {

constexpr float STEREO_MAX_EPIPOLAR_DELTA_PX = 1.5f;
constexpr float STEREO_MIN_DISPARITY_PX = 0.75f;
constexpr float STEREO_MAX_DISPARITY_PX = 240.0f;
constexpr float STEREO_SIMILARITY_RATIO_TEST = 0.98f;
constexpr float STEREO_MIN_DESCRIPTOR_SIMILARITY = 0.20f;
constexpr float STEREO_MIN_ZNCC_SCORE = 0.10f;
constexpr int STEREO_PATCH_RADIUS_PX = 3;
constexpr float TEMPORAL_STEREO_MIN_ZNCC_SCORE = 0.05f;
constexpr float STEREO_DISPARITY_MAD_SCALE = 2.5f;
constexpr float STEREO_DISPARITY_MIN_TOLERANCE_PX = 6.0f;

float StereoMinDisparityPx();

bool ComputePatchZncc(const cv::Mat &leftGray32f, const cv::Point2f &leftPt,
                      const cv::Mat &rightGray32f, const cv::Point2f &rightPt,
                      float &score);

bool IsStereoPairGeometricallyValid(const cv::Point2f &leftPt,
                                    const cv::Point2f &rightPt);

struct RefineRightPointByStereoZnccRequest {
    const cv::Mat &leftGray32f;
    const cv::Point2f &leftPt;
    const cv::Mat &rightGray32f;
    const cv::Point2f &predictedRightPt;
    cv::Point2f &refinedRightPt;
    float &bestScore;
};

bool RefineRightPointByStereoZncc(
    const RefineRightPointByStereoZnccRequest &request);

bool FindRightPointByStereoZncc(const cv::Mat &leftGray32f,
                                const cv::Point2f &leftPt,
                                const cv::Mat &rightGray32f,
                                cv::Point2f &rightPt, float &bestScore);

struct FindRightPointByStereoZnccAroundDisparityRequest {
    const cv::Mat &leftGray32f;
    const cv::Point2f &leftPt;
    const cv::Mat &rightGray32f;
    float expectedDisparity;
    cv::Point2f &rightPt;
    float &bestScore;
};

bool FindRightPointByStereoZnccAroundDisparity(
    const FindRightPointByStereoZnccAroundDisparityRequest &request);

float ComputeStereoCandidateQuality(float descriptorScore, float zncc,
                                    float epipolarErrorPx, float disparity);

std::vector<Core::Ports::StereoMatchPair>
FilterStereoPairsByDisparityConsistency(
    const std::vector<Core::Ports::StereoMatchPair> &matches);

} // namespace SmartDrone::Adapters::Slam
