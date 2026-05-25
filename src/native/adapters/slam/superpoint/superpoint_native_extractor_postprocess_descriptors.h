#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "adapters/slam/superpoint/superpoint_lightglue_frontend_client.h"
#include "adapters/slam/superpoint/superpoint_native_extractor_postprocess_heatmap.h"
#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_common.h"

namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal {

struct DescriptorBilinearHwcSampleRequest {
    const std::vector<float> &hwc;
    int height;
    int width;
    float x;
    float y;
    float *out;
};

void SampleDescriptorBilinear(const TensorBlob &descriptors, int batch, float x,
                              float y, float *out);
void SampleDescriptorNearest(const TensorBlob &descriptors, int batch, float x,
                             float y, float *out);
void BuildDescriptorGridHwc(const TensorBlob &descriptors, int batch,
                            std::vector<float> &hwc);
void SampleDescriptorBilinearHwc(
    const DescriptorBilinearHwcSampleRequest &request);
void MatchStereoPairs(const SuperPointFeatureSet &leftRaw,
                      const SuperPointFeatureSet &rightRaw, int maxPoints,
                      SuperPointFeatureSet &leftOut,
                      SuperPointFeatureSet &rightOut);
void AppendStereoFeaturePairs(SuperPointFeatureSet &leftOut,
                              SuperPointFeatureSet &rightOut,
                              const SuperPointFeatureSet &leftSupplement,
                              const SuperPointFeatureSet &rightSupplement,
                              int maxPoints);
bool IsStereoFeaturePairNearExisting(
    const cv::Point2f &leftPoint, const cv::Point2f &rightPoint,
    const SuperPointFeatureSet &existingLeft,
    const SuperPointFeatureSet &existingRight);

} // namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal
