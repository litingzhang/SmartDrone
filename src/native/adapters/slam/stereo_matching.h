#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "core/ports/stereo_processing.h"

namespace smartdrone::adapters::slam {

class DefaultStereoMatchSelector final
    : public core::ports::IStereoMatchSelector {
public:
  bool
  SelectMatches(const core::ports::StereoMatchSelectionInput &input,
                core::ports::StereoMatchSelection &selection) const override;
};

void CopyMatchedStereoPointsFromPairs(
    const core::ports::VisualFeatureSet &leftFeatures,
    const core::ports::VisualFeatureSet &rightFeatures,
    const std::vector<core::ports::StereoMatchPair> &matches,
    std::vector<cv::Point2f> &leftPoints,
    std::vector<cv::Point2f> &rightPoints);

bool SelectStereoFeatureMatches(
    const core::ports::StereoMatchSelectionInput &input,
    core::ports::StereoMatchSelection &selection);

} // namespace smartdrone::adapters::slam
