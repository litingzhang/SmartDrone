#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

#include "core/ports/stereo_processing.h"

namespace SmartDrone::Adapters::Slam {

using TemporalStereoStateView = Core::Ports::TemporalStereoStateView;
using TemporalStereoCarryInput = Core::Ports::TemporalStereoCarryInput;
using TemporalStereoCarryResult = Core::Ports::TemporalStereoCarryResult;
using TemporalStereoSourceInput = Core::Ports::TemporalStereoSourceInput;
using TemporalStereoSource = Core::Ports::TemporalStereoSource;

class DefaultTemporalStereoProcessor final
    : public Core::Ports::ITemporalStereoProcessor {
  public:
    bool AppendCarry(const TemporalStereoCarryInput &input,
                     std::vector<cv::Point2f> &matchedLeftPoints,
                     std::vector<cv::Point2f> &matchedRightPoints,
                     TemporalStereoCarryResult &result) const override;
    bool ExtractSource(const TemporalStereoSourceInput &input,
                       TemporalStereoSource &source) const override;
};

size_t AppendTemporalStereoCarry(const TemporalStereoCarryInput &input,
                                 std::vector<cv::Point2f> &matchedLeftPoints,
                                 std::vector<cv::Point2f> &matchedRightPoints);

bool ExtractTemporalStereoSource(
    const cv::Mat &leftPrepared, const cv::Mat &rightPrepared,
    const Core::Ports::StereoFeatureObservationPacket &stereoData,
    cv::Mat &prevLeft, cv::Mat &prevRight,
    std::vector<cv::Point2f> &prevLeftPoints,
    std::vector<cv::Point2f> &prevRightPoints);

} // namespace SmartDrone::Adapters::Slam
