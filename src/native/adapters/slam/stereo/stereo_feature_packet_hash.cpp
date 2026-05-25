#include "adapters/slam/stereo/stereo_feature_packet.h"

#include <algorithm>
#include <cstdint>
#include <cstring>

namespace SmartDrone::Adapters::Slam {
namespace {

uint64_t HashFloatValue(uint64_t hash, float value)
{
    uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "unexpected float size");
    std::memcpy(&bits, &value, sizeof(bits));
    hash ^= static_cast<uint64_t>(bits);
    hash *= 1099511628211ULL;
    return hash;
}

uint64_t HashIntValue(uint64_t hash, int value)
{
    hash ^= static_cast<uint64_t>(static_cast<uint32_t>(value));
    hash *= 1099511628211ULL;
    return hash;
}

uint64_t HashMatSample(uint64_t hash, const cv::Mat &mat)
{
    hash = HashIntValue(hash, mat.rows);
    hash = HashIntValue(hash, mat.cols);
    hash = HashIntValue(hash, mat.type());
    if (mat.empty()) {
        return hash;
    }
    const int rowStride = std::max(1, mat.rows / 16);
    if (mat.type() == CV_32F) {
        const int colStride = std::max(1, mat.cols / 16);
        for (int row = 0; row < mat.rows; row += rowStride) {
            const float *data = mat.ptr<float>(row);
            for (int col = 0; col < mat.cols; col += colStride) {
                hash = HashFloatValue(hash, data[col]);
            }
        }
    } else if (mat.type() == CV_8U) {
        const int colStride = std::max(1, mat.cols / 32);
        for (int row = 0; row < mat.rows; row += rowStride) {
            const uint8_t *data = mat.ptr<uint8_t>(row);
            for (int col = 0; col < mat.cols; col += colStride) {
                hash ^= static_cast<uint64_t>(data[col]);
                hash *= 1099511628211ULL;
            }
        }
    }
    return hash;
}

} // namespace

uint64_t HashStereoFeatureObservations(
    const Core::Ports::StereoFeatureObservationPacket &data)
{
    uint64_t hash = 1469598103934665603ULL;
    hash = HashIntValue(hash, static_cast<int>(data.leftKeypoints.size()));
    hash = HashIntValue(hash, static_cast<int>(data.rightKeypoints.size()));
    hash = HashIntValue(hash, data.matchedStereoPairs ? 1 : 0);
    const size_t leftCount = std::min<size_t>(data.leftKeypoints.size(), 512);
    for (size_t i = 0; i < leftCount; ++i) {
        hash = HashFloatValue(hash, data.leftKeypoints[i].pt.x);
        hash = HashFloatValue(hash, data.leftKeypoints[i].pt.y);
    }
    const size_t rightCount = std::min<size_t>(data.rightKeypoints.size(), 512);
    for (size_t i = 0; i < rightCount; ++i) {
        hash = HashFloatValue(hash, data.rightKeypoints[i].pt.x);
        hash = HashFloatValue(hash, data.rightKeypoints[i].pt.y);
    }
    const size_t matchCount = std::min<size_t>(data.leftToRightMatch.size(), 512);
    for (size_t i = 0; i < matchCount; ++i) {
        hash = HashIntValue(hash, data.leftToRightMatch[i]);
    }
    hash = HashMatSample(hash, data.leftDescriptors);
    hash = HashMatSample(hash, data.rightDescriptors);
    return hash;
}

uint64_t DefaultStereoFeaturePacketBuilder::HashStereoData(
    const Core::Ports::StereoFeatureObservationPacket &data) const
{
    return HashStereoFeatureObservations(data);
}

} // namespace SmartDrone::Adapters::Slam
