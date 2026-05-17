#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

#include "adapters/slam/external_feature_types.h"

namespace smartdrone::adapters::slam {

bool HasValidExternalFeatureDescriptors(const ExternalFeatureSet &features);

std::vector<StereoMatchPair> MatchStereoPairs(const ExternalFeatureSet &left,
                                              const ExternalFeatureSet &right,
                                              const cv::Mat &leftGray,
                                              const cv::Mat &rightGray);

std::vector<StereoMatchPair> BuildAlignedStereoPairs(const ExternalFeatureSet &left,
                                                     const ExternalFeatureSet &right,
                                                     const cv::Mat &leftGray,
                                                     const cv::Mat &rightGray);

void LimitStereoPairsInPlace(std::vector<cv::Point2f> &leftPoints,
                             std::vector<cv::Point2f> &rightPoints,
                             size_t maxCount);

} // namespace smartdrone::adapters::slam
