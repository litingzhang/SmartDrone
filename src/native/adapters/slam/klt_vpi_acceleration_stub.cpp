#include "adapters/slam/klt_vpi_acceleration.h"

#include <iostream>

namespace SmartDrone::Adapters::Slam {

struct LkPerFrameVpiState {};

bool StoreVpiPreviousRectified(std::shared_ptr<LkPerFrameVpiState> &)
{
    return false;
}

bool VpiRemapCurrentStereo(const cv::Mat &, const cv::Mat &, cv::Mat &,
                           cv::Mat &, std::shared_ptr<LkPerFrameVpiState> &,
                           const cv::Mat &, const cv::Mat &, const cv::Mat &,
                           const cv::Mat &, bool &)
{
    return false;
}

bool ComputeVpiCudaDisparity(const cv::Mat &, const cv::Mat &, cv::Mat &,
                             std::shared_ptr<LkPerFrameVpiState> &,
                             bool &logged)
{
    if (!logged) {
        std::cerr
            << "[lk_per_frame_accel] VPI support not compiled; fallback=cpu_sgbm\n";
        logged = true;
    }
    return false;
}

bool HasVpiPreviousRectified(const std::shared_ptr<LkPerFrameVpiState> &)
{
    return false;
}

bool ComputeVpiCudaPreviousRectifiedDisparity(
    const cv::Size &, cv::Mat &, std::shared_ptr<LkPerFrameVpiState> &)
{
    return false;
}

bool ComputeVpiCudaCurrentPyrLk(const cv::Mat &,
                                const std::vector<cv::Point2f> &,
                                std::vector<cv::Point2f> &,
                                std::vector<uint8_t> &,
                                std::shared_ptr<LkPerFrameVpiState> &)
{
    return false;
}

} // namespace SmartDrone::Adapters::Slam
