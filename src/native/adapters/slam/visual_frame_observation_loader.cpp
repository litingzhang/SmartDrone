#include "adapters/slam/visual_frame_observation_loader.h"

#include <algorithm>

namespace smartdrone::adapters::slam {

namespace {

cv::Mat TrimDescriptorRows(const cv::Mat &descriptors, size_t targetRows) {
  if (descriptors.empty() || targetRows == 0) {
    return cv::Mat();
  }
  const int rows =
      std::min<int>(descriptors.rows, static_cast<int>(targetRows));
  return descriptors.rowRange(0, rows).clone();
}

void FitKeypointsToDescriptors(std::vector<cv::KeyPoint> &keypoints,
                               const cv::Mat &descriptors) {
  if (descriptors.rows < static_cast<int>(keypoints.size())) {
    keypoints.resize(static_cast<size_t>(std::max(0, descriptors.rows)));
  }
}

void ApplyOctave(std::vector<cv::KeyPoint> &keypoints, int octave) {
  for (cv::KeyPoint &keypoint : keypoints) {
    keypoint.octave = octave;
  }
}

} // namespace

bool DefaultVisualFrameObservationLoader::LoadMonoObservation(
    const core::ports::MonoFrameObservationLoadRequest &request,
    core::ports::VisualFrameObservationData &outData) const {
  outData = {};
  if (request.features == nullptr) {
    return false;
  }

  outData.leftKeypoints = request.features->keypoints;
  outData.leftDescriptors =
      TrimDescriptorRows(request.features->descriptors,
                         outData.leftKeypoints.size());
  FitKeypointsToDescriptors(outData.leftKeypoints, outData.leftDescriptors);
  outData.featureCount = static_cast<int>(outData.leftKeypoints.size());
  return true;
}

bool DefaultVisualFrameObservationLoader::LoadStereoObservation(
    const core::ports::StereoFrameObservationLoadRequest &request,
    core::ports::VisualFrameObservationData &outData) const {
  outData = {};
  if (request.features == nullptr) {
    return false;
  }

  const core::ports::StereoFeatureObservationPacket &features =
      *request.features;
  outData.leftKeypoints = features.leftKeypoints;
  outData.rightKeypoints = features.rightKeypoints;
  outData.leftDescriptors =
      TrimDescriptorRows(features.leftDescriptors, outData.leftKeypoints.size());
  outData.rightDescriptors = TrimDescriptorRows(features.rightDescriptors,
                                               outData.rightKeypoints.size());
  FitKeypointsToDescriptors(outData.leftKeypoints, outData.leftDescriptors);
  FitKeypointsToDescriptors(outData.rightKeypoints, outData.rightDescriptors);

  const int octave = std::clamp(request.options.injectedKeypointOctave, 0,
                               std::max(0, request.options.maxScaleLevel));
  ApplyOctave(outData.leftKeypoints, octave);
  ApplyOctave(outData.rightKeypoints, octave);

  outData.featureCount = static_cast<int>(outData.leftKeypoints.size());
  outData.rightU = std::vector<float>(
      static_cast<size_t>(outData.featureCount), -1.0f);
  outData.depth = std::vector<float>(
      static_cast<size_t>(outData.featureCount), -1.0f);

  if (!features.matchedStereoPairs) {
    return true;
  }

  const size_t pairCount =
      features.leftToRightMatch.empty()
          ? std::min(outData.leftKeypoints.size(), outData.rightKeypoints.size())
          : outData.leftKeypoints.size();
  for (size_t i = 0; i < pairCount; ++i) {
    int rightIndex = static_cast<int>(i);
    if (!features.leftToRightMatch.empty()) {
      if (i >= features.leftToRightMatch.size()) {
        continue;
      }
      rightIndex = features.leftToRightMatch[i];
    }
    if (rightIndex < 0 ||
        static_cast<size_t>(rightIndex) >= outData.rightKeypoints.size()) {
      continue;
    }
    const float rightX =
        outData.rightKeypoints[static_cast<size_t>(rightIndex)].pt.x;
    const float disparity = outData.leftKeypoints[i].pt.x - rightX;
    if (disparity <= 0.0f) {
      continue;
    }
    outData.rightU[i] = rightX;
    outData.depth[i] =
        (request.options.baselineFx / disparity) *
        request.options.injectedStereoDepthScale;
    if (outData.depth[i] > 0.0f &&
        outData.depth[i] < request.options.closeDepthThreshold) {
      ++outData.closePointCount;
    }
  }

  return true;
}

bool LoadMonoFrameObservation(
    const core::ports::MonoFrameObservationLoadRequest &request,
    core::ports::VisualFrameObservationData &outData) {
  return DefaultVisualFrameObservationLoader().LoadMonoObservation(request,
                                                                  outData);
}

bool LoadStereoFrameObservation(
    const core::ports::StereoFrameObservationLoadRequest &request,
    core::ports::VisualFrameObservationData &outData) {
  return DefaultVisualFrameObservationLoader().LoadStereoObservation(request,
                                                                    outData);
}

} // namespace smartdrone::adapters::slam
