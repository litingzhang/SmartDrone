#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "core/ports/stereo_processing.h"

namespace SmartDrone::Adapters::Slam {

class DefaultStereoMatchSelector final
    : public Core::Ports::IStereoMatchSelector {
  public:
    bool
    SelectMatches(const Core::Ports::StereoMatchSelectionInput &input,
                  Core::Ports::StereoMatchSelection &selection) const override;
};

void CopyMatchedStereoPointsFromPairs(
    const Core::Ports::VisualFeatureSet &leftFeatures,
    const Core::Ports::VisualFeatureSet &rightFeatures,
    const std::vector<Core::Ports::StereoMatchPair> &matches,
    std::vector<cv::Point2f> &leftPoints,
    std::vector<cv::Point2f> &rightPoints);

bool SelectStereoFeatureMatches(
    const Core::Ports::StereoMatchSelectionInput &input,
    Core::Ports::StereoMatchSelection &selection);

} // namespace SmartDrone::Adapters::Slam
