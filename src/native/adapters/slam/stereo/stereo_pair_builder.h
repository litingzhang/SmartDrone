#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

#include "core/ports/stereo_processing.h"

namespace SmartDrone::Adapters::Slam {

class DefaultStereoPairBuilder final : public Core::Ports::IStereoPairBuilder {
  public:
    bool BuildPairs(const Core::Ports::StereoPairBuildInput &input,
                    Core::Ports::StereoPairBuildResult &result) const override;
};

bool HasValidVisualFeatureDescriptors(
    const Core::Ports::VisualFeatureSet &features);

std::vector<Core::Ports::StereoMatchPair>
MatchStereoPairs(const Core::Ports::VisualFeatureSet &left,
                 const Core::Ports::VisualFeatureSet &right,
                 const cv::Mat &leftGray, const cv::Mat &rightGray);

std::vector<Core::Ports::StereoMatchPair>
BuildAlignedStereoPairs(const Core::Ports::VisualFeatureSet &left,
                        const Core::Ports::VisualFeatureSet &right,
                        const cv::Mat &leftGray, const cv::Mat &rightGray);

void LimitStereoPairsInPlace(std::vector<cv::Point2f> &leftPoints,
                             std::vector<cv::Point2f> &rightPoints,
                             size_t maxCount);

} // namespace SmartDrone::Adapters::Slam
