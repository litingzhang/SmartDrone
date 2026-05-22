#include "adapters/slam/visual_frame_observation_loader.h"

#include <algorithm>

namespace SmartDrone::Adapters::Slam {

namespace {

cv::Mat TrimDescriptorRows(const cv::Mat &descriptors, size_t targetRows)
{
    if (descriptors.empty() || targetRows == 0) {
        return cv::Mat();
    }
    const int rows =
        std::min<int>(descriptors.rows, static_cast<int>(targetRows));
    return descriptors.rowRange(0, rows).clone();
}

void FitKeypointsToDescriptors(std::vector<cv::KeyPoint> &keypoints,
                               const cv::Mat &descriptors)
{
    if (descriptors.rows < static_cast<int>(keypoints.size())) {
        keypoints.resize(static_cast<size_t>(std::max(0, descriptors.rows)));
    }
}

void ApplyOctave(std::vector<cv::KeyPoint> &keypoints, int octave)
{
    for (cv::KeyPoint &keypoint : keypoints) {
        keypoint.octave = octave;
    }
}

void LoadStereoKeypointsAndDescriptors(
    const Core::Ports::StereoFeatureObservationPacket &features,
    Core::Ports::VisualFrameObservationData &outData)
{
    outData.leftKeypoints = features.leftKeypoints;
    outData.rightKeypoints = features.rightKeypoints;
    outData.leftDescriptors =
        TrimDescriptorRows(features.leftDescriptors, outData.leftKeypoints.size());
    outData.rightDescriptors = TrimDescriptorRows(features.rightDescriptors,
                                                  outData.rightKeypoints.size());
    FitKeypointsToDescriptors(outData.leftKeypoints, outData.leftDescriptors);
    FitKeypointsToDescriptors(outData.rightKeypoints, outData.rightDescriptors);
}

void ApplyInjectedOctave(
    const Core::Ports::StereoFrameObservationLoadRequest &request,
    Core::Ports::VisualFrameObservationData &outData)
{
    const int octave = std::clamp(request.options.injectedKeypointOctave, 0,
                                  std::max(0, request.options.maxScaleLevel));
    ApplyOctave(outData.leftKeypoints, octave);
    ApplyOctave(outData.rightKeypoints, octave);
}

void InitializeStereoDepthOutputs(
    Core::Ports::VisualFrameObservationData &outData)
{
    outData.featureCount = static_cast<int>(outData.leftKeypoints.size());
    outData.rightU = std::vector<float>(
        static_cast<size_t>(outData.featureCount), -1.0f);
    outData.depth = std::vector<float>(
        static_cast<size_t>(outData.featureCount), -1.0f);
}

size_t StereoPairCount(const Core::Ports::StereoFeatureObservationPacket &features,
                       const Core::Ports::VisualFrameObservationData &outData)
{
    if (features.leftToRightMatch.empty()) {
        return std::min(outData.leftKeypoints.size(), outData.rightKeypoints.size());
    }
    return outData.leftKeypoints.size();
}

int ResolveRightFeatureIndex(
    const Core::Ports::StereoFeatureObservationPacket &features,
    size_t leftIndex)
{
    if (features.leftToRightMatch.empty()) {
        return static_cast<int>(leftIndex);
    }
    if (leftIndex >= features.leftToRightMatch.size()) {
        return -1;
    }
    return features.leftToRightMatch[leftIndex];
}

void ApplyStereoDepthForPair(
    const Core::Ports::StereoFrameObservationLoadRequest &request,
    size_t leftIndex, int rightIndex,
    Core::Ports::VisualFrameObservationData &outData)
{
    if (rightIndex < 0 ||
        static_cast<size_t>(rightIndex) >= outData.rightKeypoints.size()) {
        return;
    }
    const float rightX =
        outData.rightKeypoints[static_cast<size_t>(rightIndex)].pt.x;
    const float disparity = outData.leftKeypoints[leftIndex].pt.x - rightX;
    if (disparity <= 0.0f) {
        return;
    }

    outData.rightU[leftIndex] = rightX;
    outData.depth[leftIndex] =
        (request.options.baselineFx / disparity) *
        request.options.injectedStereoDepthScale;
    if (outData.depth[leftIndex] > 0.0f &&
        outData.depth[leftIndex] < request.options.closeDepthThreshold) {
        ++outData.closePointCount;
    }
}

void FillMatchedStereoDepth(
    const Core::Ports::StereoFrameObservationLoadRequest &request,
    const Core::Ports::StereoFeatureObservationPacket &features,
    Core::Ports::VisualFrameObservationData &outData)
{
    const size_t pairCount = StereoPairCount(features, outData);
    for (size_t i = 0; i < pairCount; ++i) {
        ApplyStereoDepthForPair(request, i, ResolveRightFeatureIndex(features, i),
                                outData);
    }
}

} // namespace

bool DefaultVisualFrameObservationLoader::LoadMonoObservation(
    const Core::Ports::MonoFrameObservationLoadRequest &request,
    Core::Ports::VisualFrameObservationData &outData) const
{
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
    const Core::Ports::StereoFrameObservationLoadRequest &request,
    Core::Ports::VisualFrameObservationData &outData) const
{
    outData = {};
    if (request.features == nullptr) {
        return false;
    }

    const Core::Ports::StereoFeatureObservationPacket &features =
        *request.features;
    LoadStereoKeypointsAndDescriptors(features, outData);
    ApplyInjectedOctave(request, outData);
    InitializeStereoDepthOutputs(outData);

    if (!features.matchedStereoPairs) {
        return true;
    }

    FillMatchedStereoDepth(request, features, outData);
    return true;
}

bool LoadMonoFrameObservation(
    const Core::Ports::MonoFrameObservationLoadRequest &request,
    Core::Ports::VisualFrameObservationData &outData)
{
    return DefaultVisualFrameObservationLoader().LoadMonoObservation(request,
                                                                     outData);
}

bool LoadStereoFrameObservation(
    const Core::Ports::StereoFrameObservationLoadRequest &request,
    Core::Ports::VisualFrameObservationData &outData)
{
    return DefaultVisualFrameObservationLoader().LoadStereoObservation(request,
                                                                       outData);
}

} // namespace SmartDrone::Adapters::Slam
