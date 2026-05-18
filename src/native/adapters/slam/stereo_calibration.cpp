#include "adapters/slam/stereo_calibration.h"

#include <cmath>
#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgproc.hpp>

namespace smartdrone::adapters::slam {

namespace {

cv::Mat MakeCameraMatrixLocal(float fx, float fy, float cx, float cy) {
  return (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
}

cv::Mat MakeDistCoeffsLocal(float k1, float k2, float p1, float p2) {
  return (cv::Mat_<double>(4, 1) << k1, k2, p1, p2);
}

bool ValidIntrinsics(const StereoCameraIntrinsics &intrinsics) {
  return intrinsics.fx > 0.0f && intrinsics.fy > 0.0f && !intrinsics.K.empty();
}

} // namespace

bool LoadStereoCalibrationFromSettings(const std::string &settingsPath,
                                       StereoCalibration &calibration) {
  calibration = StereoCalibration{};
  if (settingsPath.empty()) {
    std::cerr << "[stereo_calib] settings path empty; stereo calibration "
                 "unavailable\n";
    return false;
  }

  cv::FileStorage fs(settingsPath, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "[stereo_calib] failed to open settings: " << settingsPath
              << "\n";
    return false;
  }

  calibration.left.fx = static_cast<float>(fs["Camera1.fx"]);
  calibration.left.fy = static_cast<float>(fs["Camera1.fy"]);
  calibration.left.cx = static_cast<float>(fs["Camera1.cx"]);
  calibration.left.cy = static_cast<float>(fs["Camera1.cy"]);
  calibration.right.fx = static_cast<float>(fs["Camera2.fx"]);
  calibration.right.fy = static_cast<float>(fs["Camera2.fy"]);
  calibration.right.cx = static_cast<float>(fs["Camera2.cx"]);
  calibration.right.cy = static_cast<float>(fs["Camera2.cy"]);
  if (!(calibration.left.fx > 0.0f) || !(calibration.left.fy > 0.0f) ||
      !(calibration.right.fx > 0.0f) || !(calibration.right.fy > 0.0f)) {
    std::cerr << "[stereo_calib] invalid camera intrinsics in settings\n";
    return false;
  }

  calibration.left.K =
      MakeCameraMatrixLocal(calibration.left.fx, calibration.left.fy,
                            calibration.left.cx, calibration.left.cy);
  calibration.right.K =
      MakeCameraMatrixLocal(calibration.right.fx, calibration.right.fy,
                            calibration.right.cx, calibration.right.cy);
  calibration.left.D =
      MakeDistCoeffsLocal(static_cast<float>(fs["Camera1.k1"]),
                          static_cast<float>(fs["Camera1.k2"]),
                          static_cast<float>(fs["Camera1.p1"]),
                          static_cast<float>(fs["Camera1.p2"]));
  calibration.right.D =
      MakeDistCoeffsLocal(static_cast<float>(fs["Camera2.k1"]),
                          static_cast<float>(fs["Camera2.k2"]),
                          static_cast<float>(fs["Camera2.p1"]),
                          static_cast<float>(fs["Camera2.p2"]));

  fs["Stereo.T_c1_c2"] >> calibration.T_c1_c2;
  if (calibration.T_c1_c2.empty() || calibration.T_c1_c2.rows != 4 ||
      calibration.T_c1_c2.cols != 4) {
    calibration.T_c1_c2.release();
    std::cerr << "[stereo_calib] Stereo.T_c1_c2 missing; falling back to "
                 "Camera.bf baseline\n";
  }

  const float bf = static_cast<float>(fs["Camera.bf"]);
  calibration.baselineMeters = bf > 0.0f ? bf / calibration.left.fx : 0.0f;
  if (!calibration.T_c1_c2.empty()) {
    cv::Mat T64;
    calibration.T_c1_c2.convertTo(T64, CV_64F);
    calibration.baselineMeters =
        std::abs(static_cast<float>(T64.at<double>(0, 3)));
  }
  if (!(calibration.baselineMeters > 0.005f)) {
    std::cerr << "[stereo_calib] invalid stereo baseline\n";
    return false;
  }

  calibration.loaded =
      ValidIntrinsics(calibration.left) && ValidIntrinsics(calibration.right);
  if (calibration.loaded) {
    std::cerr << "[stereo_calib] loaded fx=" << calibration.left.fx
              << " fy=" << calibration.left.fy
              << " baseline=" << calibration.baselineMeters << "\n";
  }
  return calibration.loaded;
}

bool EnsureStereoRectifier(StereoCalibration &calibration,
                           const cv::Size &inputSize) {
  if (!calibration.loaded || inputSize.area() <= 0) {
    return false;
  }
  if (calibration.rectification.imageSize == inputSize &&
      !calibration.rectification.leftMapX.empty() &&
      !calibration.rectification.rightMapX.empty()) {
    return true;
  }

  cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat t = (cv::Mat_<double>(3, 1)
                   << -static_cast<double>(calibration.baselineMeters),
               0.0, 0.0);
  if (!calibration.T_c1_c2.empty()) {
    cv::Mat T64;
    calibration.T_c1_c2.convertTo(T64, CV_64F);
    cv::Mat Tlr = T64.inv();
    R = Tlr(cv::Rect(0, 0, 3, 3)).clone();
    t = Tlr(cv::Rect(3, 0, 1, 3)).clone();
  }

  cv::Mat R1, R2, P1, P2, Q;
  cv::stereoRectify(calibration.left.K, calibration.left.D, calibration.right.K,
                    calibration.right.D, inputSize, R, t, R1, R2, P1, P2, Q,
                    cv::CALIB_ZERO_DISPARITY, -1.0, inputSize);
  cv::initUndistortRectifyMap(
      calibration.left.K, calibration.left.D, R1, P1, inputSize, CV_32FC1,
      calibration.rectification.leftMapX, calibration.rectification.leftMapY);
  cv::initUndistortRectifyMap(
      calibration.right.K, calibration.right.D, R2, P2, inputSize, CV_32FC1,
      calibration.rectification.rightMapX, calibration.rectification.rightMapY);

  calibration.left.fx = static_cast<float>(P1.at<double>(0, 0));
  calibration.left.fy = static_cast<float>(P1.at<double>(1, 1));
  calibration.left.cx = static_cast<float>(P1.at<double>(0, 2));
  calibration.left.cy = static_cast<float>(P1.at<double>(1, 2));
  calibration.baselineMeters =
      std::abs(static_cast<float>(P2.at<double>(0, 3) / P2.at<double>(0, 0)));
  calibration.rectification.imageSize = inputSize;
  return true;
}

bool ApplyStereoRectification(StereoCalibration &calibration,
                              const cv::Mat &leftGray, const cv::Mat &rightGray,
                              cv::Mat &leftRect, cv::Mat &rightRect) {
  leftRect = leftGray;
  rightRect = rightGray;
  if (!EnsureStereoRectifier(calibration, leftGray.size())) {
    return false;
  }
  if (calibration.rectification.leftMapX.empty() ||
      calibration.rectification.rightMapX.empty()) {
    return false;
  }
  cv::remap(leftGray, leftRect, calibration.rectification.leftMapX,
            calibration.rectification.leftMapY, cv::INTER_LINEAR);
  cv::remap(rightGray, rightRect, calibration.rectification.rightMapX,
            calibration.rectification.rightMapY, cv::INTER_LINEAR);
  return true;
}

bool DefaultStereoCalibrationLoader::LoadFromSettings(
    const std::string &settingsPath, StereoCalibration &calibration) const {
  return LoadStereoCalibrationFromSettings(settingsPath, calibration);
}

bool DefaultStereoRectifier::EnsureRectifier(StereoCalibration &calibration,
                                             const cv::Size &inputSize) const {
  return EnsureStereoRectifier(calibration, inputSize);
}

bool DefaultStereoRectifier::ApplyRectification(StereoCalibration &calibration,
                                                const cv::Mat &leftGray,
                                                const cv::Mat &rightGray,
                                                cv::Mat &leftRect,
                                                cv::Mat &rightRect) const {
  return ApplyStereoRectification(calibration, leftGray, rightGray, leftRect,
                                  rightRect);
}

} // namespace smartdrone::adapters::slam
