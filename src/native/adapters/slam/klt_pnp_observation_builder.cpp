#include "adapters/slam/klt_pnp_observation_builder.h"

#include "adapters/slam/klt_mode_utils.h"
#include "adapters/slam/stereo_geometry.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace smartdrone::adapters::slam {
namespace {

struct KltPerFramePnPCandidate {
    cv::Point3f object;
    cv::Point2f image;
    cv::Point2f previousPoint;
    float depth{0.0f};
};

bool IsInsideSampleWindow(const cv::Point2f &pt, int cols, int rows)
{
    return pt.x >= 1.0f && pt.y >= 1.0f && pt.x < static_cast<float>(cols - 1) &&
           pt.y < static_cast<float>(rows - 1);
}

} // namespace

KltPnpObservationSet BuildKltPerFramePnpObservations(
    const KltPerFramePnpObservationBuilderOptions &options)
{
    KltPnpObservationSet observations;
    if (options.disparity == nullptr || options.disparity->empty() || options.previousPoints == nullptr ||
        options.currentPoints == nullptr || options.status == nullptr || !(options.fx > 0.0f) ||
        !(options.fy > 0.0f) || !(options.baseline > 0.0f)) {
        return observations;
    }

    const cv::Mat &disparity = *options.disparity;
    const int previousCols = options.previousImageSize.width > 0 ? options.previousImageSize.width : disparity.cols;
    const int previousRows = options.previousImageSize.height > 0 ? options.previousImageSize.height : disparity.rows;
    const int currentCols = options.currentImageSize.width > 0 ? options.currentImageSize.width : disparity.cols;
    const int currentRows = options.currentImageSize.height > 0 ? options.currentImageSize.height : disparity.rows;
    if (previousCols <= 2 || previousRows <= 2 || currentCols <= 2 || currentRows <= 2) {
        return observations;
    }

    std::vector<KltPerFramePnPCandidate> candidates;
    const size_t pointCount = std::min(options.previousPoints->size(), options.currentPoints->size());
    candidates.reserve(pointCount);

    for (size_t i = 0; i < pointCount; ++i) {
        if (i >= options.status->size() || !(*options.status)[i]) {
            continue;
        }
        const cv::Point2f &previousPoint = (*options.previousPoints)[i];
        const cv::Point2f &currentPoint = (*options.currentPoints)[i];
        if (!IsInsideSampleWindow(previousPoint, disparity.cols, disparity.rows) ||
            !IsInsideSampleWindow(currentPoint, currentCols, currentRows)) {
            continue;
        }
        if (cv::norm(currentPoint - previousPoint) > kLkMaxFlowPx) {
            continue;
        }
        if (options.useForwardBackwardCheck) {
            if (options.backwardPoints == nullptr || options.backwardStatus == nullptr ||
                i >= options.backwardPoints->size() || i >= options.backwardStatus->size() ||
                !(*options.backwardStatus)[i] ||
                cv::norm((*options.backwardPoints)[i] - previousPoint) > kLkPerFrameForwardBackwardMaxErrPx) {
                continue;
            }
        }

        float sampledDisparity = 0.0f;
        if (!ReadConsistentDisparity(disparity, previousPoint, sampledDisparity)) {
            continue;
        }
        const float z = options.fx * options.baseline / sampledDisparity;
        if (!(z >= kLkMinDepthMeters) || z > kLkMaxDepthMeters || !std::isfinite(z)) {
            continue;
        }
        candidates.push_back({cv::Point3f((previousPoint.x - options.cx) * z / options.fx,
                                          (previousPoint.y - options.cy) * z / options.fy,
                                          z),
                              currentPoint,
                              previousPoint,
                              z});
    }

    observations.rawCandidateCount = static_cast<int>(candidates.size());
    observations.objectPoints.reserve(candidates.size());
    observations.imagePoints.reserve(candidates.size());

    if (options.useDepthBalancedSelection) {
        std::array<int, kLkPerFramePnPSelectGridCols * kLkPerFramePnPSelectGridRows * kLkPerFramePnPDepthBins>
            bucketCounts{};
        for (const KltPerFramePnPCandidate &candidate : candidates) {
            const int gx = std::clamp(static_cast<int>(candidate.previousPoint.x * kLkPerFramePnPSelectGridCols /
                                                       std::max(1, previousCols)),
                                      0, kLkPerFramePnPSelectGridCols - 1);
            const int gy = std::clamp(static_cast<int>(candidate.previousPoint.y * kLkPerFramePnPSelectGridRows /
                                                       std::max(1, previousRows)),
                                      0, kLkPerFramePnPSelectGridRows - 1);
            const int dz = LkPerFrameDepthBin(candidate.depth);
            const int bucket = ((gy * kLkPerFramePnPSelectGridCols) + gx) * kLkPerFramePnPDepthBins + dz;
            if (bucketCounts[static_cast<size_t>(bucket)] >= kLkPerFramePnPMaxPerGridDepthBin) {
                continue;
            }
            ++bucketCounts[static_cast<size_t>(bucket)];
            observations.objectPoints.push_back(candidate.object);
            observations.imagePoints.push_back(candidate.image);
        }
        return observations;
    }

    for (const KltPerFramePnPCandidate &candidate : candidates) {
        observations.objectPoints.push_back(candidate.object);
        observations.imagePoints.push_back(candidate.image);
    }
    return observations;
}

KltTrackedStereoPnpObservationSet BuildKltTrackedStereoPnpObservations(
    const KltTrackedStereoPnpObservationBuilderOptions &options)
{
    KltTrackedStereoPnpObservationSet result;
    if (options.previousTracks == nullptr || options.currentLeftPoints == nullptr ||
        options.currentRightPoints == nullptr || options.leftStatus == nullptr || options.rightStatus == nullptr ||
        options.currentLeftImage == nullptr || options.currentRightImage == nullptr ||
        options.currentLeftImage->empty() || options.currentRightImage->empty() || !(options.fx > 0.0f) ||
        !(options.fy > 0.0f) || !(options.baseline > 0.0f)) {
        return result;
    }

    const int previousCols =
        options.previousImageSize.width > 0 ? options.previousImageSize.width : options.currentLeftImage->cols;
    const int previousRows =
        options.previousImageSize.height > 0 ? options.previousImageSize.height : options.currentLeftImage->rows;
    if (previousCols <= 2 || previousRows <= 2 || options.currentLeftImage->cols <= 2 ||
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
        if (i >= options.leftStatus->size() || i >= options.rightStatus->size() ||
            !(*options.leftStatus)[i] || !(*options.rightStatus)[i]) {
            continue;
        }
        const LkStereoTrack &previousTrack = (*options.previousTracks)[i];
        const cv::Point2f &previousLeft = previousTrack.left;
        const cv::Point2f &previousRight = previousTrack.right;
        const cv::Point2f &currentLeft = (*options.currentLeftPoints)[i];
        cv::Point2f currentRight = (*options.currentRightPoints)[i];

        if (!IsInsideSampleWindow(previousLeft, previousCols, previousRows) ||
            !IsInsideSampleWindow(currentLeft, options.currentLeftImage->cols, options.currentLeftImage->rows) ||
            !IsInsideSampleWindow(currentRight, options.currentRightImage->cols, options.currentRightImage->rows)) {
            continue;
        }
        if (cv::norm(currentLeft - previousLeft) > kLkMaxFlowPx) {
            continue;
        }

        float zncc = -1.0f;
        if (!RefineRightPointByStereoZncc(left32f, currentLeft, right32f, currentRight, currentRight, zncc)) {
            continue;
        }

        const float disparity = previousLeft.x - previousRight.x;
        const float z = options.fx * options.baseline / disparity;
        if (!(z >= kLkMinDepthMeters) || z > kLkMaxDepthMeters || !std::isfinite(z)) {
            continue;
        }

        result.pnp.objectPoints.emplace_back((previousLeft.x - options.cx) * z / options.fx,
                                             (previousLeft.y - options.cy) * z / options.fy,
                                             z);
        result.pnp.imagePoints.push_back(currentLeft);
        const float trackedQuality =
            std::clamp(previousTrack.quality * 0.92f + std::clamp((zncc + 1.0f) * 0.5f, 0.0f, 1.0f) * 0.08f,
                       0.0f, 1.0f);
        result.trackedTracks.push_back(LkStereoTrack{currentLeft, currentRight, trackedQuality, previousTrack.age + 1});
    }

    result.pnp.rawCandidateCount = static_cast<int>(result.pnp.objectPoints.size());
    return result;
}

} // namespace smartdrone::adapters::slam
