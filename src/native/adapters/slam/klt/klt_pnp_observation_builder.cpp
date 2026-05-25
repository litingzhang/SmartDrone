#include "adapters/slam/klt/klt_pnp_observation_builder.h"

#include "adapters/slam/klt/klt_mode_utils.h"
#include "adapters/slam/stereo/stereo_geometry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace SmartDrone::Adapters::Slam {
namespace {

struct KltPerFramePnPCandidate {
    cv::Point3f object;
    cv::Point2f image;
    cv::Point2f previousPoint;
    float depth{0.0f};
};

struct AppendTrackedStereoObservationRequest {
    const KltTrackedStereoPnpObservationBuilderOptions &options;
    const LkStereoTrack &previousTrack;
    const cv::Point2f &currentLeft;
    const cv::Point2f &currentRight;
    float zncc;
    KltTrackedStereoPnpObservationSet &result;
};

struct TryAppendTrackedStereoObservationRequest {
    const KltTrackedStereoPnpObservationBuilderOptions &options;
    const cv::Size &previousImageSize;
    const cv::Mat &left32f;
    const cv::Mat &right32f;
    size_t index;
    KltTrackedStereoPnpObservationSet &result;
};

bool IsInsideSampleWindow(const cv::Point2f &pt, int cols, int rows)
{
    return pt.x >= 1.0f && pt.y >= 1.0f && pt.x < static_cast<float>(cols - 1) &&
           pt.y < static_cast<float>(rows - 1);
}

bool HasValidPerFramePnpInput(const KltPerFramePnpObservationBuilderOptions &options)
{
    return options.disparity != nullptr && !options.disparity->empty() && options.previousPoints != nullptr &&
           options.currentPoints != nullptr && options.status != nullptr && options.fx > 0.0f &&
           options.fy > 0.0f && options.baseline > 0.0f;
}

bool HasValidTrackedStereoPnpInput(const KltTrackedStereoPnpObservationBuilderOptions &options)
{
    return options.previousTracks != nullptr && options.currentLeftPoints != nullptr &&
           options.currentRightPoints != nullptr && options.leftStatus != nullptr && options.rightStatus != nullptr &&
           options.currentLeftImage != nullptr && options.currentRightImage != nullptr &&
           !options.currentLeftImage->empty() && !options.currentRightImage->empty() && options.fx > 0.0f &&
           options.fy > 0.0f && options.baseline > 0.0f;
}

cv::Size ResolveImageSize(const cv::Size &configuredSize, const cv::Mat &fallbackImage)
{
    return {configuredSize.width > 0 ? configuredSize.width : fallbackImage.cols,
            configuredSize.height > 0 ? configuredSize.height : fallbackImage.rows};
}

bool IsUsableSampleSize(const cv::Size &size)
{
    return size.width > 2 && size.height > 2;
}

bool PassesPerFrameForwardBackwardCheck(const KltPerFramePnpObservationBuilderOptions &options, size_t index,
                                        const cv::Point2f &previousPoint)
{
    if (!options.useForwardBackwardCheck) {
        return true;
    }
    return options.backwardPoints != nullptr && options.backwardStatus != nullptr &&
           index < options.backwardPoints->size() && index < options.backwardStatus->size() &&
           (*options.backwardStatus)[index] &&
           cv::norm((*options.backwardPoints)[index] - previousPoint) <= LK_PER_FRAME_FORWARD_BACKWARD_MAX_ERR_PX;
}

cv::Point3f MakePnpObjectPoint(const cv::Point2f &point, float depth,
                               const KltPerFramePnpObservationBuilderOptions &options)
{
    return {(point.x - options.cx) * depth / options.fx,
            (point.y - options.cy) * depth / options.fy,
            depth};
}

bool IsDepthUsable(float depth)
{
    return depth >= LK_MIN_DEPTH_METERS && depth <= LK_MAX_DEPTH_METERS && std::isfinite(depth);
}

std::optional<KltPerFramePnPCandidate> BuildPerFrameCandidate(
    const KltPerFramePnpObservationBuilderOptions &options, const cv::Mat &disparity, size_t index,
    const cv::Size &currentImageSize)
{
    if (index >= options.status->size() || !(*options.status)[index]) {
        return std::nullopt;
    }
    const cv::Point2f &previousPoint = (*options.previousPoints)[index];
    const cv::Point2f &currentPoint = (*options.currentPoints)[index];
    if (!IsInsideSampleWindow(previousPoint, disparity.cols, disparity.rows) ||
        !IsInsideSampleWindow(currentPoint, currentImageSize.width, currentImageSize.height)) {
        return std::nullopt;
    }
    if (cv::norm(currentPoint - previousPoint) > LK_MAX_FLOW_PX ||
        !PassesPerFrameForwardBackwardCheck(options, index, previousPoint)) {
        return std::nullopt;
    }

    float sampledDisparity = 0.0f;
    if (!ReadConsistentDisparity(disparity, previousPoint, sampledDisparity)) {
        return std::nullopt;
    }
    const float depth = options.fx * options.baseline / sampledDisparity;
    if (!IsDepthUsable(depth)) {
        return std::nullopt;
    }
    return KltPerFramePnPCandidate{MakePnpObjectPoint(previousPoint, depth, options), currentPoint, previousPoint,
                                   depth};
}

std::vector<KltPerFramePnPCandidate> CollectPerFrameCandidates(
    const KltPerFramePnpObservationBuilderOptions &options, const cv::Mat &disparity,
    const cv::Size &currentImageSize)
{
    std::vector<KltPerFramePnPCandidate> candidates;
    const size_t pointCount = std::min(options.previousPoints->size(), options.currentPoints->size());
    candidates.reserve(pointCount);
    for (size_t i = 0; i < pointCount; ++i) {
        std::optional<KltPerFramePnPCandidate> candidate =
            BuildPerFrameCandidate(options, disparity, i, currentImageSize);
        if (candidate.has_value()) {
            candidates.push_back(*candidate);
        }
    }
    return candidates;
}

int PerFrameCandidateBucket(const KltPerFramePnPCandidate &candidate, const cv::Size &previousImageSize)
{
    const int gridX = std::clamp(static_cast<int>(candidate.previousPoint.x * LK_PER_FRAME_PNP_SELECT_GRID_COLS /
                                                  std::max(1, previousImageSize.width)),
                                 0, LK_PER_FRAME_PNP_SELECT_GRID_COLS - 1);
    const int gridY = std::clamp(static_cast<int>(candidate.previousPoint.y * LK_PER_FRAME_PNP_SELECT_GRID_ROWS /
                                                  std::max(1, previousImageSize.height)),
                                 0, LK_PER_FRAME_PNP_SELECT_GRID_ROWS - 1);
    return ((gridY * LK_PER_FRAME_PNP_SELECT_GRID_COLS) + gridX) * LK_PER_FRAME_PNP_DEPTH_BINS +
           LkPerFrameDepthBin(candidate.depth);
}

void AppendCandidateObservation(const KltPerFramePnPCandidate &candidate, KltPnpObservationSet &observations)
{
    observations.objectPoints.push_back(candidate.object);
    observations.imagePoints.push_back(candidate.image);
}

void AppendDepthBalancedCandidates(const std::vector<KltPerFramePnPCandidate> &candidates,
                                   const cv::Size &previousImageSize, KltPnpObservationSet &observations)
{
    std::array<int, LK_PER_FRAME_PNP_SELECT_GRID_COLS * LK_PER_FRAME_PNP_SELECT_GRID_ROWS * LK_PER_FRAME_PNP_DEPTH_BINS>
        bucketCounts{};
    for (const KltPerFramePnPCandidate &candidate : candidates) {
        const int bucket = PerFrameCandidateBucket(candidate, previousImageSize);
        if (bucketCounts[static_cast<size_t>(bucket)] >= LK_PER_FRAME_PNP_MAX_PER_GRID_DEPTH_BIN) {
            continue;
        }
        ++bucketCounts[static_cast<size_t>(bucket)];
        AppendCandidateObservation(candidate, observations);
    }
}

void AppendAllCandidates(const std::vector<KltPerFramePnPCandidate> &candidates, KltPnpObservationSet &observations)
{
    for (const KltPerFramePnPCandidate &candidate : candidates) {
        AppendCandidateObservation(candidate, observations);
    }
}

cv::Point3f MakeTrackedObjectPoint(const LkStereoTrack &track, float depth,
                                   const KltTrackedStereoPnpObservationBuilderOptions &options)
{
    return {(track.left.x - options.cx) * depth / options.fx,
            (track.left.y - options.cy) * depth / options.fy,
            depth};
}

float BlendTrackedQuality(const LkStereoTrack &previousTrack, float zncc)
{
    const float znccQuality = std::clamp((zncc + 1.0f) * 0.5f, 0.0f, 1.0f);
    return std::clamp(previousTrack.quality * 0.92f + znccQuality * 0.08f, 0.0f, 1.0f);
}

bool HasTrackedStereoStatus(const KltTrackedStereoPnpObservationBuilderOptions &options, size_t index)
{
    return index < options.leftStatus->size() && index < options.rightStatus->size() &&
           (*options.leftStatus)[index] && (*options.rightStatus)[index];
}

bool HasTrackedStereoSampleWindow(const KltTrackedStereoPnpObservationBuilderOptions &options,
                                  const cv::Size &previousImageSize, const LkStereoTrack &previousTrack,
                                  const cv::Point2f &currentLeft, const cv::Point2f &currentRight)
{
    return IsInsideSampleWindow(previousTrack.left, previousImageSize.width, previousImageSize.height) &&
           IsInsideSampleWindow(currentLeft, options.currentLeftImage->cols, options.currentLeftImage->rows) &&
           IsInsideSampleWindow(currentRight, options.currentRightImage->cols, options.currentRightImage->rows);
}

void AppendTrackedStereoObservation(
    const AppendTrackedStereoObservationRequest &request)
{
    const auto &options = request.options;
    const LkStereoTrack &previousTrack = request.previousTrack;
    const float disparity = previousTrack.left.x - previousTrack.right.x;
    const float depth = options.fx * options.baseline / disparity;
    if (!IsDepthUsable(depth)) {
        return;
    }
    request.result.pnp.objectPoints.push_back(
        MakeTrackedObjectPoint(previousTrack, depth, options));
    request.result.pnp.imagePoints.push_back(request.currentLeft);
    request.result.trackedTracks.push_back(
        LkStereoTrack{request.currentLeft, request.currentRight,
                      BlendTrackedQuality(previousTrack, request.zncc),
                      previousTrack.age + 1});
}

void TryAppendTrackedStereoObservation(
    const TryAppendTrackedStereoObservationRequest &request)
{
    const auto &options = request.options;
    if (!HasTrackedStereoStatus(options, request.index)) {
        return;
    }
    const LkStereoTrack &previousTrack =
        (*options.previousTracks)[request.index];
    const cv::Point2f &currentLeft =
        (*options.currentLeftPoints)[request.index];
    cv::Point2f currentRight = (*options.currentRightPoints)[request.index];
    if (!HasTrackedStereoSampleWindow(options, request.previousImageSize,
                                      previousTrack, currentLeft, currentRight) ||
        cv::norm(currentLeft - previousTrack.left) > LK_MAX_FLOW_PX) {
        return;
    }

    float zncc = -1.0f;
    if (!RefineRightPointByStereoZncc(
            {request.left32f, currentLeft, request.right32f, currentRight,
             currentRight, zncc})) {
        return;
    }
    AppendTrackedStereoObservation(
        {options, previousTrack, currentLeft, currentRight, zncc,
         request.result});
}

} // namespace

KltPnpObservationSet BuildKltPerFramePnpObservations(
    const KltPerFramePnpObservationBuilderOptions &options)
{
    KltPnpObservationSet observations;
    if (!HasValidPerFramePnpInput(options)) {
        return observations;
    }

    const cv::Mat &disparity = *options.disparity;
    const cv::Size previousImageSize = ResolveImageSize(options.previousImageSize, disparity);
    const cv::Size currentImageSize = ResolveImageSize(options.currentImageSize, disparity);
    if (!IsUsableSampleSize(previousImageSize) || !IsUsableSampleSize(currentImageSize)) {
        return observations;
    }

    const std::vector<KltPerFramePnPCandidate> candidates =
        CollectPerFrameCandidates(options, disparity, currentImageSize);
    observations.rawCandidateCount = static_cast<int>(candidates.size());
    observations.objectPoints.reserve(candidates.size());
    observations.imagePoints.reserve(candidates.size());

    if (options.useDepthBalancedSelection) {
        AppendDepthBalancedCandidates(candidates, previousImageSize, observations);
        return observations;
    }

    AppendAllCandidates(candidates, observations);
    return observations;
}

KltTrackedStereoPnpObservationSet BuildKltTrackedStereoPnpObservations(
    const KltTrackedStereoPnpObservationBuilderOptions &options)
{
    KltTrackedStereoPnpObservationSet result;
    if (!HasValidTrackedStereoPnpInput(options)) {
        return result;
    }

    const cv::Size previousImageSize = ResolveImageSize(options.previousImageSize, *options.currentLeftImage);
    if (!IsUsableSampleSize(previousImageSize) || options.currentLeftImage->cols <= 2 ||
        options.currentLeftImage->rows <= 2 || options.currentRightImage->cols <= 2 ||
        options.currentRightImage->rows <= 2) {
        return result;
    }

    const size_t pointCount = std::min({options.previousTracks->size(), options.currentLeftPoints->size(),
                                        options.currentRightPoints->size()});
    result.pnp.objectPoints.reserve(pointCount);
    result.pnp.imagePoints.reserve(pointCount);
    result.trackedTracks.reserve(pointCount);

    cv::Mat left32f;
    cv::Mat right32f;
    options.currentLeftImage->convertTo(left32f, CV_32F);
    options.currentRightImage->convertTo(right32f, CV_32F);

    for (size_t i = 0; i < pointCount; ++i) {
        TryAppendTrackedStereoObservation(
            {options, previousImageSize, left32f, right32f, i, result});
    }

    result.pnp.rawCandidateCount = static_cast<int>(result.pnp.objectPoints.size());
    return result;
}

KltPnpObservationSet DefaultVisualPnpObservationBuilder::BuildPerFrameObservations(
    const KltPerFramePnpObservationBuilderOptions &options) const
{
    return BuildKltPerFramePnpObservations(options);
}

KltTrackedStereoPnpObservationSet DefaultVisualPnpObservationBuilder::BuildTrackedStereoObservations(
    const KltTrackedStereoPnpObservationBuilderOptions &options) const
{
    return BuildKltTrackedStereoPnpObservations(options);
}

} // namespace SmartDrone::Adapters::Slam
