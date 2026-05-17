#include "adapters/slam/slam_image_utils.h"

#include <opencv2/imgproc.hpp>

namespace smartdrone::adapters::slam {

cv::Mat MakeCameraMatrix(float fx, float fy, float cx, float cy)
{
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = fx;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    return K;
}

cv::Mat MakeDistCoeffs(float k1, float k2, float p1, float p2)
{
    cv::Mat D = cv::Mat::zeros(1, 4, CV_64F);
    D.at<double>(0, 0) = k1;
    D.at<double>(0, 1) = k2;
    D.at<double>(0, 2) = p1;
    D.at<double>(0, 3) = p2;
    return D;
}

cv::Mat EnsureGray8(const cv::Mat &image)
{
    if (image.empty()) {
        return {};
    }
    if (image.type() == CV_8UC1) {
        return image;
    }
    cv::Mat gray;
    if (image.channels() == 1) {
        image.convertTo(gray, CV_8U);
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    return gray;
}

} // namespace smartdrone::adapters::slam
