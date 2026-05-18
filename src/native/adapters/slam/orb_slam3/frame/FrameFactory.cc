#include "FrameFactory.h"

namespace ORB_SLAM3 {

namespace {

const IMU::Calib &ImuCalibrationOrDefault(
    const OrbFrameSensorContext &sensor) {
  static const IMU::Calib kDefaultImuCalibration;
  return sensor.imuCalibration != nullptr ? *sensor.imuCalibration
                                          : kDefaultImuCalibration;
}

ORBextractor *MonoExtractorForRequest(
    const OrbMonocularFrameCreateRequest &request) {
  return request.useInitExtractor && request.sensor.initExtractor != nullptr
             ? request.sensor.initExtractor
             : request.sensor.leftExtractor;
}

ORBextractor *MonoFeatureExtractorForRequest(
    const OrbMonocularFeatureFrameCreateRequest &request) {
  return request.useInitExtractor && request.sensor.initExtractor != nullptr
             ? request.sensor.initExtractor
             : request.sensor.leftExtractor;
}

class DefaultOrbFrameFactory final : public IOrbFrameFactory {
public:
  Frame CreateStereoFrame(
      const OrbStereoFrameCreateRequest &request) const override {
    if (request.leftImage == nullptr || request.rightImage == nullptr ||
        request.sensor.calibration == nullptr ||
        request.sensor.distortion == nullptr ||
        request.sensor.leftCamera == nullptr ||
        request.sensor.leftExtractor == nullptr ||
        request.sensor.rightExtractor == nullptr ||
        request.sensor.vocabulary == nullptr) {
      return Frame();
    }

    if (request.sensor.rightCamera != nullptr &&
        request.sensor.leftToRightPose != nullptr) {
      return Frame(*request.leftImage, *request.rightImage, request.timestamp,
                   request.sensor.leftExtractor, request.sensor.rightExtractor,
                   request.sensor.vocabulary, *request.sensor.calibration,
                   *request.sensor.distortion, request.sensor.baselineFx,
                   request.sensor.closeDepthThreshold,
                   request.sensor.leftCamera, request.sensor.rightCamera,
                   *request.sensor.leftToRightPose,
                   request.sensor.previousFrame,
                   ImuCalibrationOrDefault(request.sensor));
    }

    return Frame(*request.leftImage, *request.rightImage, request.timestamp,
                 request.sensor.leftExtractor, request.sensor.rightExtractor,
                 request.sensor.vocabulary, *request.sensor.calibration,
                 *request.sensor.distortion, request.sensor.baselineFx,
                 request.sensor.closeDepthThreshold,
                 request.sensor.leftCamera, request.sensor.previousFrame,
                 ImuCalibrationOrDefault(request.sensor));
  }

  Frame CreateStereoFeatureFrame(
      const OrbStereoFeatureFrameCreateRequest &request) const override {
    if (request.leftImage == nullptr || request.rightImage == nullptr ||
        request.features == nullptr || request.sensor.calibration == nullptr ||
        request.sensor.distortion == nullptr ||
        request.sensor.leftCamera == nullptr ||
        request.sensor.leftExtractor == nullptr ||
        request.sensor.rightExtractor == nullptr ||
        request.sensor.vocabulary == nullptr) {
      return Frame();
    }

    return Frame(*request.leftImage, *request.rightImage, request.timestamp,
                 request.sensor.leftExtractor, request.sensor.rightExtractor,
                 request.sensor.vocabulary, *request.sensor.calibration,
                 *request.sensor.distortion, request.sensor.baselineFx,
                 request.sensor.closeDepthThreshold,
                 request.sensor.leftCamera, *request.features,
                 request.sensor.previousFrame,
                 ImuCalibrationOrDefault(request.sensor));
  }

  Frame CreateRgbdFrame(
      const OrbRgbdFrameCreateRequest &request) const override {
    if (request.image == nullptr || request.depth == nullptr ||
        request.sensor.calibration == nullptr ||
        request.sensor.distortion == nullptr ||
        request.sensor.leftCamera == nullptr ||
        request.sensor.leftExtractor == nullptr ||
        request.sensor.vocabulary == nullptr) {
      return Frame();
    }

    return Frame(*request.image, *request.depth, request.timestamp,
                 request.sensor.leftExtractor, request.sensor.vocabulary,
                 *request.sensor.calibration, *request.sensor.distortion,
                 request.sensor.baselineFx,
                 request.sensor.closeDepthThreshold,
                 request.sensor.leftCamera, request.sensor.previousFrame,
                 ImuCalibrationOrDefault(request.sensor));
  }

  Frame CreateMonocularFrame(
      const OrbMonocularFrameCreateRequest &request) const override {
    ORBextractor *extractor = MonoExtractorForRequest(request);
    if (request.image == nullptr || extractor == nullptr ||
        request.sensor.distortion == nullptr ||
        request.sensor.leftCamera == nullptr ||
        request.sensor.vocabulary == nullptr) {
      return Frame();
    }

    return Frame(*request.image, request.timestamp, extractor,
                 request.sensor.vocabulary, request.sensor.leftCamera,
                 *request.sensor.distortion, request.sensor.baselineFx,
                 request.sensor.closeDepthThreshold,
                 request.sensor.previousFrame,
                 ImuCalibrationOrDefault(request.sensor));
  }

  Frame CreateMonocularFeatureFrame(
      const OrbMonocularFeatureFrameCreateRequest &request) const override {
    ORBextractor *extractor = MonoFeatureExtractorForRequest(request);
    if (request.image == nullptr || request.features == nullptr ||
        extractor == nullptr || request.sensor.distortion == nullptr ||
        request.sensor.leftCamera == nullptr ||
        request.sensor.vocabulary == nullptr) {
      return Frame();
    }

    return Frame(*request.image, request.timestamp, extractor,
                 request.sensor.vocabulary, request.sensor.leftCamera,
                 *request.sensor.distortion, request.sensor.baselineFx,
                 request.sensor.closeDepthThreshold, *request.features,
                 request.sensor.previousFrame,
                 ImuCalibrationOrDefault(request.sensor));
  }
};

} // namespace

std::unique_ptr<IOrbFrameFactory> CreateDefaultOrbFrameFactory() {
  return std::make_unique<DefaultOrbFrameFactory>();
}

} // namespace ORB_SLAM3
