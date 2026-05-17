#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

#include "adapters/slam/external_feature_types.h"

namespace smartdrone::adapters::slam {

struct ExternalTemporalStereoStateView {
    bool havePrevStereo{false};
    const cv::Mat *prevLeft{nullptr};
    const cv::Mat *prevRight{nullptr};
    const std::vector<cv::Point2f> *prevLeftPoints{nullptr};
    const std::vector<cv::Point2f> *prevRightPoints{nullptr};
    bool previousFrameWeak{false};
};

struct ExternalTemporalStereoCarryInput {
    const ExternalTemporalStereoStateView *state{nullptr};
    const cv::Mat *leftPrepared{nullptr};
    const cv::Mat *rightPrepared{nullptr};
    bool initializing{false};
    bool recovering{false};
};

size_t AppendExternalTemporalStereoCarry(const ExternalTemporalStereoCarryInput &input,
                                         std::vector<cv::Point2f> &matchedLeftPoints,
                                         std::vector<cv::Point2f> &matchedRightPoints);

bool ExtractExternalTemporalStereoSource(const cv::Mat &leftPrepared, const cv::Mat &rightPrepared,
                                         const ExternalStereoObservationPacket &externalData,
                                         cv::Mat &prevLeft, cv::Mat &prevRight,
                                         std::vector<cv::Point2f> &prevLeftPoints,
                                         std::vector<cv::Point2f> &prevRightPoints);

} // namespace smartdrone::adapters::slam
