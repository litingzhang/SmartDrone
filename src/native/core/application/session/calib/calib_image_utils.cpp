#include "core/application/session/calib/calib_image_utils.h"

#include <opencv2/imgproc.hpp>

namespace smartdrone::core::application {

cv::Mat EnsureCalibGray8(const cv::Mat &src, bool &convertedOut)
{
    convertedOut = false;
    if (src.empty()) {
        return src;
    }
    if (src.type() == CV_8UC1) {
        return src;
    }
    cv::Mat out;
    if (src.type() == CV_16UC1) {
        src.convertTo(out, CV_8U, 1.0 / 256.0);
    } else {
        src.convertTo(out, CV_8U);
    }
    convertedOut = true;
    return out;
}

} // namespace smartdrone::core::application
