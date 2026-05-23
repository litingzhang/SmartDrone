#pragma once

#include <memory>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "Frame.h"

namespace ORB_SLAM3 {

class GeometricCamera;
class ORBextractor;
class ORBVocabulary;

struct OrbFrameSensorContext {
  ORBVocabulary *vocabulary{nullptr};
  ORBextractor *leftExtractor{nullptr};
  ORBextractor *rightExtractor{nullptr};
  ORBextractor *initExtractor{nullptr};
  GeometricCamera *leftCamera{nullptr};
  GeometricCamera *rightCamera{nullptr};
  cv::Mat *calibration{nullptr};
  cv::Mat *distortion{nullptr};
  float baselineFx{0.0f};
  float closeDepthThreshold{0.0f};
  Sophus::SE3f *leftToRightPose{nullptr};
  Frame *previousFrame{nullptr};
  const IMU::Calib *imuCalibration{nullptr};
};

struct OrbStereoFrameCreateRequest {
  const cv::Mat *leftImage{nullptr};
  const cv::Mat *rightImage{nullptr};
  double timestamp{0.0};
  OrbFrameSensorContext sensor;
};

struct OrbStereoFeatureFrameCreateRequest {
  const cv::Mat *leftImage{nullptr};
  const cv::Mat *rightImage{nullptr};
  const StereoFeatureFrameData *features{nullptr};
  double timestamp{0.0};
  OrbFrameSensorContext sensor;
};

struct OrbRgbdFrameCreateRequest {
  const cv::Mat *image{nullptr};
  const cv::Mat *depth{nullptr};
  double timestamp{0.0};
  OrbFrameSensorContext sensor;
};

struct OrbMonocularFrameCreateRequest {
  const cv::Mat *image{nullptr};
  double timestamp{0.0};
  bool useInitExtractor{false};
  OrbFrameSensorContext sensor;
};

struct OrbMonocularFeatureFrameCreateRequest {
  const cv::Mat *image{nullptr};
  const MonoFeatureFrameData *features{nullptr};
  double timestamp{0.0};
  bool useInitExtractor{false};
  OrbFrameSensorContext sensor;
};

class IOrbFrameFactory {
public:
  virtual ~IOrbFrameFactory() = default;

  virtual Frame CreateStereoFrame(
      const OrbStereoFrameCreateRequest &request) const = 0;
  virtual Frame CreateStereoFeatureFrame(
      const OrbStereoFeatureFrameCreateRequest &request) const = 0;
  virtual Frame CreateRgbdFrame(
      const OrbRgbdFrameCreateRequest &request) const = 0;
  virtual Frame CreateMonocularFrame(
      const OrbMonocularFrameCreateRequest &request) const = 0;
  virtual Frame CreateMonocularFeatureFrame(
      const OrbMonocularFeatureFrameCreateRequest &request) const = 0;
};

std::unique_ptr<IOrbFrameFactory> CreateDefaultOrbFrameFactory();

} // namespace ORB_SLAM3
