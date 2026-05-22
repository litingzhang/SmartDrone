#pragma once

#include <memory>
#include <vector>

#include <opencv2/core.hpp>

#include "adapters/slam/slam_mode_state.h"

namespace SmartDrone::adapters::slam {

bool StoreVpiPreviousRectified(std::shared_ptr<LkPerFrameVpiState> &state);
bool VpiRemapCurrentStereo(const cv::Mat &leftRaw, const cv::Mat &rightRaw, cv::Mat &leftRect, cv::Mat &rightRect,
                           std::shared_ptr<LkPerFrameVpiState> &state, const cv::Mat &map1x,
                           const cv::Mat &map1y, const cv::Mat &map2x, const cv::Mat &map2y, bool &logged);
bool ComputeVpiCudaDisparity(const cv::Mat &left, const cv::Mat &right, cv::Mat &disp,
                             std::shared_ptr<LkPerFrameVpiState> &state, bool &logged);
bool HasVpiPreviousRectified(const std::shared_ptr<LkPerFrameVpiState> &state);
bool ComputeVpiCudaPreviousRectifiedDisparity(const cv::Size &size, cv::Mat &disp,
                                              std::shared_ptr<LkPerFrameVpiState> &state);
bool ComputeVpiCudaCurrentPyrLk(const cv::Mat &prevLeft, const std::vector<cv::Point2f> &pts0,
                                std::vector<cv::Point2f> &pts1, std::vector<uint8_t> &statusOut,
                                std::shared_ptr<LkPerFrameVpiState> &state);

} // namespace SmartDrone::adapters::slam
