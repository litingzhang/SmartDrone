#include "adapters/slam/superpoint/superpoint_native_extractor_postprocess_descriptors.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <vector>

namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal {

namespace {

struct StereoPairCandidate {
    int left{0};
    int right{0};
    float distance{0.0f};
    float disparity{0.0f};
};

struct StereoPairSearchState {
    int leftCount{0};
    int rightCount{0};
    int descriptorDim{0};
    std::vector<int> bestRightForLeft;
    std::vector<float> bestDistForLeft;
    std::vector<float> secondDistForLeft;
    std::vector<int> bestLeftForRight;
    std::vector<float> bestDistForRight;
    std::vector<int> rightOrder;
};

struct StereoPairMatchRequest {
    const SuperPointFeatureSet &leftRaw;
    const SuperPointFeatureSet &rightRaw;
    int maxPoints;
    SuperPointFeatureSet &leftOut;
    SuperPointFeatureSet &rightOut;
};

struct StereoPairDescriptorSearchRequest {
    const SuperPointFeatureSet &leftRaw;
    const SuperPointFeatureSet &rightRaw;
    int leftIndex;
    StereoPairSearchState &state;
};

struct DescriptorSampleWeights {
    int x0{0};
    int y0{0};
    int x1{0};
    int y1{0};
    float w00{0.0f};
    float w01{0.0f};
    float w10{0.0f};
    float w11{0.0f};
};

DescriptorSampleWeights MakeDescriptorSampleWeights(int width, int height,
                                                    float x, float y)
{
    const float gx = std::clamp(x, 0.0f, static_cast<float>(width - 1));
    const float gy = std::clamp(y, 0.0f, static_cast<float>(height - 1));
    const int x0 = static_cast<int>(std::floor(gx));
    const int y0 = static_cast<int>(std::floor(gy));
    const int x1 = std::min(x0 + 1, width - 1);
    const int y1 = std::min(y0 + 1, height - 1);
    const float wx = gx - static_cast<float>(x0);
    const float wy = gy - static_cast<float>(y0);
    return {
        x0,
        y0,
        x1,
        y1,
        (1.0f - wx) * (1.0f - wy),
        wx * (1.0f - wy),
        (1.0f - wx) * wy,
        wx * wy,
    };
}

size_t DescriptorBatchOffset(const TensorBlob &descriptors, int batch)
{
    return static_cast<size_t>(batch) *
           static_cast<size_t>(descriptors.Dim(1)) *
           static_cast<size_t>(descriptors.Dim(2)) *
           static_cast<size_t>(descriptors.Dim(3));
}

size_t DescriptorChannelStride(const TensorBlob &descriptors)
{
    return static_cast<size_t>(descriptors.Dim(2)) *
           static_cast<size_t>(descriptors.Dim(3));
}

void SampleDescriptorBilinearChannel(const TensorBlob &descriptors, int batch,
                                     int channel,
                                     const DescriptorSampleWeights &weights,
                                     float *out)
{
    const int width = descriptors.Dim(3);
    const size_t channelOffset =
        DescriptorBatchOffset(descriptors, batch) +
        static_cast<size_t>(channel) * DescriptorChannelStride(descriptors);
    const float *data = descriptors.FloatData();
    const float v00 =
        data[channelOffset + static_cast<size_t>(weights.y0) * width +
             weights.x0];
    const float v01 =
        data[channelOffset + static_cast<size_t>(weights.y0) * width +
             weights.x1];
    const float v10 =
        data[channelOffset + static_cast<size_t>(weights.y1) * width +
             weights.x0];
    const float v11 =
        data[channelOffset + static_cast<size_t>(weights.y1) * width +
             weights.x1];
    out[channel] = weights.w00 * v00 + weights.w01 * v01 +
                   weights.w10 * v10 + weights.w11 * v11;
}

void BuildDescriptorGridHwcTile(const TensorBlob &descriptors, int batch,
                                int tileStart, int tileEnd,
                                std::vector<float> &hwc)
{
    const int channels = descriptors.Dim(1);
    const size_t spatial = DescriptorChannelStride(descriptors);
    const size_t batchOffset = DescriptorBatchOffset(descriptors, batch);
    const float *data = descriptors.FloatData();
    for (int c = 0; c < channels; ++c) {
        const float *src = data + batchOffset + static_cast<size_t>(c) * spatial +
                           static_cast<size_t>(tileStart);
        for (int index = tileStart; index < tileEnd; ++index) {
            hwc[static_cast<size_t>(index) * static_cast<size_t>(channels) +
                static_cast<size_t>(c)] = src[index - tileStart];
        }
    }
}

size_t DescriptorHwcOffset(int width, int y, int x)
{
    return (static_cast<size_t>(y) * static_cast<size_t>(width) +
            static_cast<size_t>(x)) *
           SUPER_POINT_DESCRIPTOR_DIM;
}

void SampleDescriptorBilinearHwcChannel(
    const DescriptorBilinearHwcSampleRequest &request,
    const DescriptorSampleWeights &weights, int channel)
{
    const size_t base00 =
        DescriptorHwcOffset(request.width, weights.y0, weights.x0);
    const size_t base01 =
        DescriptorHwcOffset(request.width, weights.y0, weights.x1);
    const size_t base10 =
        DescriptorHwcOffset(request.width, weights.y1, weights.x0);
    const size_t base11 =
        DescriptorHwcOffset(request.width, weights.y1, weights.x1);
    const float v00 = request.hwc[base00 + static_cast<size_t>(channel)];
    const float v01 = request.hwc[base01 + static_cast<size_t>(channel)];
    const float v10 = request.hwc[base10 + static_cast<size_t>(channel)];
    const float v11 = request.hwc[base11 + static_cast<size_t>(channel)];
    request.out[channel] = weights.w00 * v00 + weights.w01 * v01 +
                           weights.w10 * v10 + weights.w11 * v11;
}

bool PrepareStereoPairSearchState(const StereoPairMatchRequest &request,
                                  StereoPairSearchState &state)
{
    if (request.leftRaw.descriptors.empty() ||
        request.rightRaw.descriptors.empty() ||
        request.leftRaw.descriptors.type() != CV_32F ||
        request.rightRaw.descriptors.type() != CV_32F ||
        request.leftRaw.descriptors.cols != request.rightRaw.descriptors.cols) {
        return false;
    }
    const int candidateLimit =
        EnvIntClamped("SMART_DRONE_DESCRIPTOR_SUPPLEMENT_CANDIDATES",
                      std::max(1, request.maxPoints), 1, 4096);
    state.leftCount = std::min(
        {static_cast<int>(request.leftRaw.keypoints.size()),
         request.leftRaw.descriptors.rows, candidateLimit});
    state.rightCount = std::min(
        {static_cast<int>(request.rightRaw.keypoints.size()),
         request.rightRaw.descriptors.rows, candidateLimit});
    state.descriptorDim = request.leftRaw.descriptors.cols;
    if (state.leftCount <= 0 || state.rightCount <= 0 ||
        state.descriptorDim <= 0) {
        return false;
    }
    state.bestRightForLeft.assign(static_cast<size_t>(state.leftCount), -1);
    state.bestDistForLeft.assign(static_cast<size_t>(state.leftCount),
                                 std::numeric_limits<float>::infinity());
    state.secondDistForLeft.assign(static_cast<size_t>(state.leftCount),
                                   std::numeric_limits<float>::infinity());
    state.bestLeftForRight.assign(static_cast<size_t>(state.rightCount), -1);
    state.bestDistForRight.assign(static_cast<size_t>(state.rightCount),
                                  std::numeric_limits<float>::infinity());
    return true;
}

void SortStereoRightOrder(const SuperPointFeatureSet &rightRaw,
                          StereoPairSearchState &state)
{
    state.rightOrder.resize(static_cast<size_t>(state.rightCount));
    std::iota(state.rightOrder.begin(), state.rightOrder.end(), 0);
    std::sort(state.rightOrder.begin(), state.rightOrder.end(),
              [&](int lhs, int rhs) {
                  const float ly = rightRaw.keypoints[static_cast<size_t>(lhs)].y;
                  const float ry = rightRaw.keypoints[static_cast<size_t>(rhs)].y;
                  if (std::abs(ly - ry) > 1.0e-6f) {
                      return ly < ry;
                  }
                  return lhs < rhs;
              });
}

float DescriptorDistance(const float *leftDescriptor, const float *rightDescriptor,
                         int descriptorDim)
{
    float dist = 0.0f;
    for (int d = 0; d < descriptorDim; ++d) {
        const float delta = leftDescriptor[d] - rightDescriptor[d];
        dist += delta * delta;
    }
    return dist;
}

void UpdateStereoBestMatches(StereoPairSearchState &state, int li, int ri,
                             float distance)
{
    if (distance < state.bestDistForLeft[static_cast<size_t>(li)]) {
        state.secondDistForLeft[static_cast<size_t>(li)] =
            state.bestDistForLeft[static_cast<size_t>(li)];
        state.bestDistForLeft[static_cast<size_t>(li)] = distance;
        state.bestRightForLeft[static_cast<size_t>(li)] = ri;
    } else if (distance < state.secondDistForLeft[static_cast<size_t>(li)]) {
        state.secondDistForLeft[static_cast<size_t>(li)] = distance;
    }
    if (distance < state.bestDistForRight[static_cast<size_t>(ri)]) {
        state.bestDistForRight[static_cast<size_t>(ri)] = distance;
        state.bestLeftForRight[static_cast<size_t>(ri)] = li;
    }
}

void SearchStereoDescriptorMatches(
    const StereoPairDescriptorSearchRequest &request)
{
    const cv::Point2f &lp =
        request.leftRaw.keypoints[static_cast<size_t>(request.leftIndex)];
    const float minY = lp.y - STEREO_MAX_Y_DIFF_PX;
    const float maxY = lp.y + STEREO_MAX_Y_DIFF_PX;
    const auto firstRight = std::lower_bound(
        request.state.rightOrder.begin(), request.state.rightOrder.end(), minY,
        [&](int index, float value) {
            return request.rightRaw.keypoints[static_cast<size_t>(index)].y <
                   value;
        });
    const auto lastRight = std::upper_bound(
        firstRight, request.state.rightOrder.end(), maxY,
        [&](float value, int index) {
            return value <
                   request.rightRaw.keypoints[static_cast<size_t>(index)].y;
        });
    const float *ld = request.leftRaw.descriptors.ptr<float>(request.leftIndex);
    for (auto it = firstRight; it != lastRight; ++it) {
        const int ri = *it;
        const cv::Point2f &rp = request.rightRaw.keypoints[static_cast<size_t>(ri)];
        const float disparity = lp.x - rp.x;
        if (disparity < StereoMinDisparityPx() ||
            disparity > STEREO_MAX_DISPARITY_PX) {
            continue;
        }
        const float *rd = request.rightRaw.descriptors.ptr<float>(ri);
        const float distance =
            DescriptorDistance(ld, rd, request.state.descriptorDim);
        if (std::isfinite(distance)) {
            UpdateStereoBestMatches(request.state, request.leftIndex, ri,
                                    distance);
        }
    }
}

std::vector<StereoPairCandidate> BuildStereoPairCandidates(
    const SuperPointFeatureSet &leftRaw, const SuperPointFeatureSet &rightRaw,
    const StereoPairSearchState &state)
{
    std::vector<StereoPairCandidate> pairs;
    for (int li = 0; li < state.leftCount; ++li) {
        const int ri = state.bestRightForLeft[static_cast<size_t>(li)];
        if (ri < 0 || state.bestLeftForRight[static_cast<size_t>(ri)] != li) {
            continue;
        }
        const float bestDist = state.bestDistForLeft[static_cast<size_t>(li)];
        const float secondDist =
            state.secondDistForLeft[static_cast<size_t>(li)];
        if (std::isfinite(secondDist) &&
            bestDist >= (STEREO_RATIO * STEREO_RATIO) * secondDist) {
            continue;
        }
        const cv::Point2f &lp = leftRaw.keypoints[static_cast<size_t>(li)];
        const cv::Point2f &rp = rightRaw.keypoints[static_cast<size_t>(ri)];
        const float disparity = lp.x - rp.x;
        pairs.push_back(StereoPairCandidate{li, ri, bestDist, disparity});
    }
    return pairs;
}

void SortAndLimitStereoPairs(std::vector<StereoPairCandidate> &pairs,
                             int maxPoints)
{
    std::sort(pairs.begin(), pairs.end(),
              [](const StereoPairCandidate &lhs,
                 const StereoPairCandidate &rhs) {
                  if (std::abs(lhs.distance - rhs.distance) > 1.0e-6f) {
                      return lhs.distance < rhs.distance;
                  }
                  return lhs.disparity > rhs.disparity;
              });
    if (static_cast<int>(pairs.size()) > std::max(1, maxPoints)) {
        pairs.resize(static_cast<size_t>(std::max(1, maxPoints)));
    }
}

void WriteStereoPairOutputs(const StereoPairMatchRequest &request,
                            const std::vector<StereoPairCandidate> &pairs)
{
    request.leftOut.keypoints.reserve(pairs.size());
    request.rightOut.keypoints.reserve(pairs.size());
    request.leftOut.descriptors = cv::Mat(
        static_cast<int>(pairs.size()), request.leftRaw.descriptors.cols, CV_32F);
    request.rightOut.descriptors =
        cv::Mat(static_cast<int>(pairs.size()), request.rightRaw.descriptors.cols,
                CV_32F);
    for (size_t i = 0; i < pairs.size(); ++i) {
        request.leftOut.keypoints.push_back(
            request.leftRaw.keypoints[static_cast<size_t>(pairs[i].left)]);
        request.rightOut.keypoints.push_back(
            request.rightRaw.keypoints[static_cast<size_t>(pairs[i].right)]);
        request.leftRaw.descriptors.row(pairs[i].left)
            .copyTo(request.leftOut.descriptors.row(static_cast<int>(i)));
        request.rightRaw.descriptors.row(pairs[i].right)
            .copyTo(request.rightOut.descriptors.row(static_cast<int>(i)));
    }
}

} // namespace

void SampleDescriptorBilinear(const TensorBlob &descriptors, int batch, float x,
                              float y, float *out)
{
    const DescriptorSampleWeights weights = MakeDescriptorSampleWeights(
        descriptors.Dim(3), descriptors.Dim(2), x, y);
    for (int c = 0; c < SUPER_POINT_DESCRIPTOR_DIM; ++c) {
        SampleDescriptorBilinearChannel(descriptors, batch, c, weights, out);
    }
    NormalizeVector(out, SUPER_POINT_DESCRIPTOR_DIM);
}

void SampleDescriptorNearest(const TensorBlob &descriptors, int batch, float x,
                             float y, float *out)
{
    const int width = descriptors.Dim(3);
    const int height = descriptors.Dim(2);
    const int ix = std::clamp(static_cast<int>(std::round(x)), 0, width - 1);
    const int iy = std::clamp(static_cast<int>(std::round(y)), 0, height - 1);
    const size_t spatialOffset =
        static_cast<size_t>(iy) * static_cast<size_t>(width) +
        static_cast<size_t>(ix);
    const size_t batchOffset = DescriptorBatchOffset(descriptors, batch);
    const size_t channelStride = DescriptorChannelStride(descriptors);
    const float *data = descriptors.FloatData();
    for (int c = 0; c < SUPER_POINT_DESCRIPTOR_DIM; ++c) {
        out[c] = data[batchOffset + static_cast<size_t>(c) * channelStride +
                      spatialOffset];
    }
    NormalizeVector(out, SUPER_POINT_DESCRIPTOR_DIM);
}

void BuildDescriptorGridHwc(const TensorBlob &descriptors, int batch,
                            std::vector<float> &hwc)
{
    const int channels = descriptors.Dim(1);
    const size_t spatial = DescriptorChannelStride(descriptors);
    hwc.resize(spatial * static_cast<size_t>(channels));

    constexpr int tilePixels = 32;
    for (int tileStart = 0; tileStart < static_cast<int>(spatial);
         tileStart += tilePixels) {
        const int tileEnd =
            std::min(tileStart + tilePixels, static_cast<int>(spatial));
        BuildDescriptorGridHwcTile(descriptors, batch, tileStart, tileEnd, hwc);
    }
}

void SampleDescriptorBilinearHwc(
    const DescriptorBilinearHwcSampleRequest &request)
{
    const DescriptorSampleWeights weights = MakeDescriptorSampleWeights(
        request.width, request.height, request.x, request.y);
    for (int c = 0; c < SUPER_POINT_DESCRIPTOR_DIM; ++c) {
        SampleDescriptorBilinearHwcChannel(request, weights, c);
    }
    NormalizeVector(request.out, SUPER_POINT_DESCRIPTOR_DIM);
}

void MatchStereoPairs(const SuperPointFeatureSet &leftRaw,
                      const SuperPointFeatureSet &rightRaw, int maxPoints,
                      SuperPointFeatureSet &leftOut,
                      SuperPointFeatureSet &rightOut)
{
    leftOut = SuperPointFeatureSet{};
    rightOut = SuperPointFeatureSet{};
    const StereoPairMatchRequest request{
        leftRaw, rightRaw, maxPoints, leftOut, rightOut};
    StereoPairSearchState state;
    if (!PrepareStereoPairSearchState(request, state)) {
        return;
    }
    SortStereoRightOrder(rightRaw, state);
    for (int li = 0; li < state.leftCount; ++li) {
        SearchStereoDescriptorMatches({leftRaw, rightRaw, li, state});
    }
    std::vector<StereoPairCandidate> pairs =
        BuildStereoPairCandidates(leftRaw, rightRaw, state);
    SortAndLimitStereoPairs(pairs, maxPoints);
    WriteStereoPairOutputs(request, pairs);
}

void AppendStereoFeaturePairs(SuperPointFeatureSet &leftOut,
                              SuperPointFeatureSet &rightOut,
                              const SuperPointFeatureSet &leftSupplement,
                              const SuperPointFeatureSet &rightSupplement,
                              int maxPoints)
{
    if (leftSupplement.descriptors.empty() ||
        rightSupplement.descriptors.empty() ||
        leftSupplement.descriptors.type() != CV_32F ||
        rightSupplement.descriptors.type() != CV_32F ||
        leftSupplement.descriptors.cols != rightSupplement.descriptors.cols ||
        leftSupplement.descriptors.rows !=
            static_cast<int>(leftSupplement.keypoints.size()) ||
        rightSupplement.descriptors.rows !=
            static_cast<int>(rightSupplement.keypoints.size())) {
        return;
    }
    if ((!leftOut.descriptors.empty() &&
         leftOut.descriptors.cols != leftSupplement.descriptors.cols) ||
        (!rightOut.descriptors.empty() &&
         rightOut.descriptors.cols != rightSupplement.descriptors.cols)) {
        return;
    }

    const int limit = std::max(1, maxPoints);
    const size_t sourceCount = std::min(leftSupplement.keypoints.size(),
                                        rightSupplement.keypoints.size());
    for (size_t i = 0;
         i < sourceCount && static_cast<int>(leftOut.keypoints.size()) < limit;
         ++i) {
        const cv::Point2f &leftPoint = leftSupplement.keypoints[i];
        const cv::Point2f &rightPoint = rightSupplement.keypoints[i];
        if (IsStereoFeaturePairNearExisting(leftPoint, rightPoint, leftOut,
                                            rightOut)) {
            continue;
        }
        leftOut.keypoints.push_back(leftPoint);
        rightOut.keypoints.push_back(rightPoint);
        leftOut.descriptors.push_back(
            leftSupplement.descriptors.row(static_cast<int>(i)));
        rightOut.descriptors.push_back(
            rightSupplement.descriptors.row(static_cast<int>(i)));
    }
}

bool IsStereoFeaturePairNearExisting(
    const cv::Point2f &leftPoint, const cv::Point2f &rightPoint,
    const SuperPointFeatureSet &existingLeft,
    const SuperPointFeatureSet &existingRight)
{
    const float minDistanceSq = STEREO_MERGE_DISTANCE_PX * STEREO_MERGE_DISTANCE_PX;
    const size_t count =
        std::min(existingLeft.keypoints.size(), existingRight.keypoints.size());
    for (size_t i = 0; i < count; ++i) {
        const cv::Point2f leftDelta = existingLeft.keypoints[i] - leftPoint;
        const cv::Point2f rightDelta = existingRight.keypoints[i] - rightPoint;
        if ((leftDelta.x * leftDelta.x + leftDelta.y * leftDelta.y) <=
                minDistanceSq &&
            (rightDelta.x * rightDelta.x + rightDelta.y * rightDelta.y) <=
                minDistanceSq) {
            return true;
        }
    }
    return false;
}

} // namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal
