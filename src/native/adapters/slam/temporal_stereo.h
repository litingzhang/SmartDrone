#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

#include "core/ports/stereo_processing.h"

namespace smartdrone::adapters::slam {

using TemporalStereoStateView = core::ports::TemporalStereoStateView;
using TemporalStereoCarryInput = core::ports::TemporalStereoCarryInput;
using TemporalStereoCarryResult = core::ports::TemporalStereoCarryResult;
using TemporalStereoSourceInput = core::ports::TemporalStereoSourceInput;
using TemporalStereoSource = core::ports::TemporalStereoSource;

class DefaultTemporalStereoProcessor final
    : public core::ports::ITemporalStereoProcessor {
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
    const core::ports::StereoFeatureObservationPacket &stereoData,
    cv::Mat &prevLeft, cv::Mat &prevRight,
    std::vector<cv::Point2f> &prevLeftPoints,
    std::vector<cv::Point2f> &prevRightPoints);

} // namespace smartdrone::adapters::slam
