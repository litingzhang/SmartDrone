#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

#include "core/ports/stereo_processing.h"

namespace SmartDrone::adapters::slam {

class DefaultStereoPairBuilder final : public core::ports::IStereoPairBuilder {
  public:
    bool BuildPairs(const core::ports::StereoPairBuildInput &input,
                    core::ports::StereoPairBuildResult &result) const override;
};

bool HasValidVisualFeatureDescriptors(
    const core::ports::VisualFeatureSet &features);

std::vector<core::ports::StereoMatchPair>
MatchStereoPairs(const core::ports::VisualFeatureSet &left,
                 const core::ports::VisualFeatureSet &right,
                 const cv::Mat &leftGray, const cv::Mat &rightGray);

std::vector<core::ports::StereoMatchPair>
BuildAlignedStereoPairs(const core::ports::VisualFeatureSet &left,
                        const core::ports::VisualFeatureSet &right,
                        const cv::Mat &leftGray, const cv::Mat &rightGray);

void LimitStereoPairsInPlace(std::vector<cv::Point2f> &leftPoints,
                             std::vector<cv::Point2f> &rightPoints,
                             size_t maxCount);

} // namespace SmartDrone::adapters::slam
