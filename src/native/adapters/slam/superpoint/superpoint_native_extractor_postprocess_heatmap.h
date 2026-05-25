#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_common.h"

namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal {

struct SuperPointPostStats {
    double heatmapMs{0.0};
    double nmsMs{0.0};
    double scanMs{0.0};
    double sortMs{0.0};
    double descriptorMs{0.0};
    int candidateCount{0};
    int selectedCount{0};
    int descriptorCount{0};

    void Add(const SuperPointPostStats &other);
};

struct Candidate {
    int x{0};
    int y{0};
    float score{0.0f};
};

struct SuperPointPostScratch {
    cv::Mat heatmap;
    cv::Mat localMax;
    std::vector<Candidate> candidates;
    std::vector<Candidate> nmsCandidates;
    std::vector<uint8_t> suppressionMask;
    std::vector<float> descriptorHwc;
};

struct CandidateExtractionRequest {
    const TensorBlob &detector;
    int batch;
    int targetWidth;
    int targetHeight;
    int maxPoints;
    SuperPointPostScratch &scratch;
    SuperPointPostStats *stats;
};

float At4D(const TensorBlob &blob, int b, int c, int y, int x);
void NormalizeVector(float *values, int count);
void BuildInputBatch(const std::vector<cv::Mat> &images, int targetHeight,
                     int targetWidth, std::vector<float> &batch);
void ExtractCandidates(const CandidateExtractionRequest &request);
void ExtractCandidatesFastNms(const CandidateExtractionRequest &request);

} // namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal
