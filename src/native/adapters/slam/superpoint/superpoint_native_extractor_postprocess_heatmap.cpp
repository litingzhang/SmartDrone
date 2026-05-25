#include "adapters/slam/superpoint/superpoint_native_extractor_postprocess_heatmap.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal {

namespace {

struct CandidateExtractionTiming {
    std::chrono::steady_clock::time_point heatmapStartTp{};
    std::chrono::steady_clock::time_point heatmapEndTp{};
    std::chrono::steady_clock::time_point nmsStartTp{};
    std::chrono::steady_clock::time_point nmsEndTp{};
    std::chrono::steady_clock::time_point scanStartTp{};
    std::chrono::steady_clock::time_point scanEndTp{};
    std::chrono::steady_clock::time_point sortStartTp{};
    std::chrono::steady_clock::time_point sortEndTp{};
    int candidateCount{0};
    int selectedCount{0};
};

struct SuperPointTensorView {
    int gridHeight{0};
    int gridWidth{0};
    size_t channelStride{0};
    size_t batchOffset{0};
    const float *data{nullptr};
};

const cv::Mat &SuperPointNmsKernel()
{
    static const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(SUPER_POINT_NMS_RADIUS * 2 + 1, SUPER_POINT_NMS_RADIUS * 2 + 1));
    return kernel;
}

void SortCandidatesByScore(std::vector<Candidate> &candidates)
{
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate &lhs, const Candidate &rhs) {
                  return lhs.score > rhs.score;
              });
}

void RecordCandidateExtractionStats(const CandidateExtractionRequest &request,
                                    const CandidateExtractionTiming &timing)
{
    if (request.stats == nullptr) {
        return;
    }
    request.stats->heatmapMs +=
        DurationMs(timing.heatmapStartTp, timing.heatmapEndTp);
    request.stats->nmsMs += DurationMs(timing.nmsStartTp, timing.nmsEndTp);
    request.stats->scanMs += DurationMs(timing.scanStartTp, timing.scanEndTp);
    request.stats->sortMs += DurationMs(timing.sortStartTp, timing.sortEndTp);
    request.stats->candidateCount += timing.candidateCount;
    request.stats->selectedCount += timing.selectedCount;
}

SuperPointTensorView MakeSuperPointTensorView(const TensorBlob &detector,
                                              int batch)
{
    const int channels = detector.Dim(1);
    const int gridHeight = detector.Dim(2);
    const int gridWidth = detector.Dim(3);
    const size_t imageStride = static_cast<size_t>(channels) *
                               static_cast<size_t>(gridHeight) *
                               static_cast<size_t>(gridWidth);
    const size_t channelStride =
        static_cast<size_t>(gridHeight) * static_cast<size_t>(gridWidth);
    const size_t batchOffset = static_cast<size_t>(batch) * imageStride;
    return {gridHeight, gridWidth, channelStride, batchOffset,
            detector.FloatData()};
}

float MaxSuperPointCellLogit(const SuperPointTensorView &view,
                             size_t spatialOffset)
{
    float maxLogit = view.data[view.batchOffset + spatialOffset];
    for (int c = 1; c < 65; ++c) {
        maxLogit = std::max(
            maxLogit,
            view.data[view.batchOffset +
                      static_cast<size_t>(c) * view.channelStride +
                      spatialOffset]);
    }
    return maxLogit;
}

float BuildSuperPointCellExpValues(const SuperPointTensorView &view,
                                   size_t spatialOffset,
                                   std::array<float, 65> &expValues)
{
    const float maxLogit = MaxSuperPointCellLogit(view, spatialOffset);
    float denom = 0.0f;
    for (int c = 0; c < 65; ++c) {
        expValues[static_cast<size_t>(c)] =
            std::exp(view.data[view.batchOffset +
                               static_cast<size_t>(c) * view.channelStride +
                               spatialOffset] -
                     maxLogit);
        denom += expValues[static_cast<size_t>(c)];
    }
    return denom;
}

void WriteHeatmapCell(const SuperPointTensorView &view, int gy, int gx,
                      cv::Mat &heatmap)
{
    const size_t spatialOffset =
        static_cast<size_t>(gy) * static_cast<size_t>(view.gridWidth) +
        static_cast<size_t>(gx);
    std::array<float, 65> expValues;
    const float denom =
        BuildSuperPointCellExpValues(view, spatialOffset, expValues);
    if (!(denom > 0.0f)) {
        return;
    }
    const float invDenom = 1.0f / denom;
    for (int c = 0; c < 64; ++c) {
        const int y = gy * SUPER_POINT_CELL_SIZE + c / SUPER_POINT_CELL_SIZE;
        const int x = gx * SUPER_POINT_CELL_SIZE + c % SUPER_POINT_CELL_SIZE;
        heatmap.ptr<float>(y)[x] =
            expValues[static_cast<size_t>(c)] * invDenom;
    }
}

void BuildHeatmap(const TensorBlob &detector, int batch, cv::Mat &heatmap)
{
    const SuperPointTensorView view = MakeSuperPointTensorView(detector, batch);
    heatmap.create(view.gridHeight * SUPER_POINT_CELL_SIZE,
                   view.gridWidth * SUPER_POINT_CELL_SIZE, CV_32F);
    heatmap.setTo(cv::Scalar(0.0f));
    for (int gy = 0; gy < view.gridHeight; ++gy) {
        for (int gx = 0; gx < view.gridWidth; ++gx) {
            WriteHeatmapCell(view, gy, gx, heatmap);
        }
    }
}

bool IsInsideFastNmsTarget(const CandidateExtractionRequest &request, int x,
                           int y)
{
    return x >= SUPER_POINT_BORDER && y >= SUPER_POINT_BORDER &&
           x < request.targetWidth - SUPER_POINT_BORDER &&
           y < request.targetHeight - SUPER_POINT_BORDER;
}

void AppendFastNmsCellCandidates(const CandidateExtractionRequest &request,
                                 const SuperPointTensorView &view, int gy,
                                 int gx)
{
    const size_t spatialOffset =
        static_cast<size_t>(gy) * static_cast<size_t>(view.gridWidth) +
        static_cast<size_t>(gx);
    std::array<float, 65> expValues;
    const float denom =
        BuildSuperPointCellExpValues(view, spatialOffset, expValues);
    if (!(denom > 0.0f)) {
        return;
    }
    const float invDenom = 1.0f / denom;
    for (int c = 0; c < 64; ++c) {
        const int y = gy * SUPER_POINT_CELL_SIZE + c / SUPER_POINT_CELL_SIZE;
        const int x = gx * SUPER_POINT_CELL_SIZE + c % SUPER_POINT_CELL_SIZE;
        if (!IsInsideFastNmsTarget(request, x, y)) {
            continue;
        }
        const float score = expValues[static_cast<size_t>(c)] * invDenom;
        if (score > SUPER_POINT_THRESHOLD) {
            request.scratch.candidates.push_back(Candidate{x, y, score});
        }
    }
}

void CollectFastNmsCandidates(const CandidateExtractionRequest &request,
                              const SuperPointTensorView &view)
{
    std::vector<Candidate> &candidates = request.scratch.candidates;
    candidates.clear();
    candidates.reserve(static_cast<size_t>(std::max(1, request.maxPoints)) * 3);
    for (int gy = 0; gy < view.gridHeight; ++gy) {
        for (int gx = 0; gx < view.gridWidth; ++gx) {
            AppendFastNmsCellCandidates(request, view, gy, gx);
        }
    }
}

void PrepareFastNmsSuppression(const CandidateExtractionRequest &request)
{
    request.scratch.suppressionMask.assign(
        static_cast<size_t>(request.targetWidth) *
            static_cast<size_t>(request.targetHeight),
        0);
    request.scratch.nmsCandidates.clear();
    request.scratch.nmsCandidates.reserve(
        static_cast<size_t>(std::max(1, request.maxPoints)));
}

void MarkFastNmsSuppression(const CandidateExtractionRequest &request,
                            const Candidate &candidate)
{
    const int y0 =
        std::max(SUPER_POINT_BORDER, candidate.y - SUPER_POINT_NMS_RADIUS);
    const int y1 = std::min(request.targetHeight - SUPER_POINT_BORDER - 1,
                            candidate.y + SUPER_POINT_NMS_RADIUS);
    const int x0 =
        std::max(SUPER_POINT_BORDER, candidate.x - SUPER_POINT_NMS_RADIUS);
    const int x1 = std::min(request.targetWidth - SUPER_POINT_BORDER - 1,
                            candidate.x + SUPER_POINT_NMS_RADIUS);
    for (int y = y0; y <= y1; ++y) {
        uint8_t *row = request.scratch.suppressionMask.data() +
                       static_cast<size_t>(y) *
                           static_cast<size_t>(request.targetWidth);
        for (int x = x0; x <= x1; ++x) {
            row[x] = 1;
        }
    }
}

bool TryAcceptFastNmsCandidate(const CandidateExtractionRequest &request,
                               const Candidate &candidate)
{
    const size_t center =
        static_cast<size_t>(candidate.y) *
            static_cast<size_t>(request.targetWidth) +
        static_cast<size_t>(candidate.x);
    if (request.scratch.suppressionMask[center] != 0) {
        return false;
    }
    request.scratch.nmsCandidates.push_back(candidate);
    MarkFastNmsSuppression(request, candidate);
    return true;
}

void ApplyFastNmsSuppression(const CandidateExtractionRequest &request)
{
    std::vector<Candidate> &candidates = request.scratch.candidates;
    PrepareFastNmsSuppression(request);
    const int limit = std::max(1, request.maxPoints);
    for (const Candidate &candidate : candidates) {
        if (TryAcceptFastNmsCandidate(request, candidate) &&
            static_cast<int>(request.scratch.nmsCandidates.size()) >= limit) {
            break;
        }
    }
    candidates.swap(request.scratch.nmsCandidates);
}

} // namespace

void SuperPointPostStats::Add(const SuperPointPostStats &other)
{
    heatmapMs += other.heatmapMs;
    nmsMs += other.nmsMs;
    scanMs += other.scanMs;
    sortMs += other.sortMs;
    descriptorMs += other.descriptorMs;
    candidateCount += other.candidateCount;
    selectedCount += other.selectedCount;
    descriptorCount += other.descriptorCount;
}

float At4D(const TensorBlob &blob, int b, int c, int y, int x)
{
    const int channels = blob.Dim(1);
    const int height = blob.Dim(2);
    const int width = blob.Dim(3);
    const size_t index =
        (((static_cast<size_t>(b) * channels + c) * height + y) * width + x);
    return blob.FloatData()[index];
}

void NormalizeVector(float *values, int count)
{
    float normSq = 0.0f;
    for (int i = 0; i < count; ++i) {
        normSq += values[i] * values[i];
    }
    const float invNorm = normSq > 1.0e-12f ? 1.0f / std::sqrt(normSq) : 1.0f;
    for (int i = 0; i < count; ++i) {
        values[i] *= invNorm;
    }
}

void BuildInputBatch(const std::vector<cv::Mat> &images, int targetHeight,
                     int targetWidth, std::vector<float> &batch)
{
    batch.resize(static_cast<size_t>(images.size()) * targetHeight * targetWidth);
    for (size_t i = 0; i < images.size(); ++i) {
        cv::Mat resized;
        if (images[i].rows != targetHeight || images[i].cols != targetWidth) {
            cv::resize(images[i], resized, cv::Size(targetWidth, targetHeight),
                       0.0, 0.0, cv::INTER_LINEAR);
        } else {
            resized = images[i];
        }
        float *dst = batch.data() + i * static_cast<size_t>(targetHeight) *
                                        static_cast<size_t>(targetWidth);
        for (int y = 0; y < targetHeight; ++y) {
            const uint8_t *src = resized.ptr<uint8_t>(y);
            for (int x = 0; x < targetWidth; ++x) {
                dst[static_cast<size_t>(y) * targetWidth + x] =
                    static_cast<float>(src[x]) / 255.0f;
            }
        }
    }
}

void ExtractCandidates(const CandidateExtractionRequest &request)
{
    CandidateExtractionTiming timing;
    timing.heatmapStartTp = std::chrono::steady_clock::now();
    BuildHeatmap(request.detector, request.batch, request.scratch.heatmap);
    timing.heatmapEndTp = std::chrono::steady_clock::now();
    timing.nmsStartTp = timing.heatmapEndTp;
    cv::dilate(request.scratch.heatmap, request.scratch.localMax,
               SuperPointNmsKernel());
    timing.nmsEndTp = std::chrono::steady_clock::now();
    std::vector<Candidate> &candidates = request.scratch.candidates;
    candidates.clear();
    candidates.reserve(static_cast<size_t>(std::max(1, request.maxPoints)) * 2);
    timing.scanStartTp = timing.nmsEndTp;
    for (int y = SUPER_POINT_BORDER; y < request.targetHeight - SUPER_POINT_BORDER;
         ++y) {
        const float *scoreRow = request.scratch.heatmap.ptr<float>(y);
        const float *maxRow = request.scratch.localMax.ptr<float>(y);
        for (int x = SUPER_POINT_BORDER;
             x < request.targetWidth - SUPER_POINT_BORDER; ++x) {
            const float score = scoreRow[x];
            if (score <= SUPER_POINT_THRESHOLD) {
                continue;
            }
            if (score >= maxRow[x]) {
                candidates.push_back(Candidate{x, y, score});
            }
        }
    }
    timing.scanEndTp = std::chrono::steady_clock::now();
    timing.candidateCount = static_cast<int>(candidates.size());
    timing.sortStartTp = timing.scanEndTp;
    SortCandidatesByScore(candidates);
    if (static_cast<int>(candidates.size()) > request.maxPoints) {
        candidates.resize(static_cast<size_t>(std::max(1, request.maxPoints)));
    }
    timing.sortEndTp = std::chrono::steady_clock::now();
    timing.selectedCount = static_cast<int>(candidates.size());
    RecordCandidateExtractionStats(request, timing);
}

void ExtractCandidatesFastNms(const CandidateExtractionRequest &request)
{
    CandidateExtractionTiming timing;
    const SuperPointTensorView view =
        MakeSuperPointTensorView(request.detector, request.batch);
    timing.heatmapStartTp = std::chrono::steady_clock::now();
    CollectFastNmsCandidates(request, view);
    timing.heatmapEndTp = std::chrono::steady_clock::now();
    timing.scanStartTp = timing.heatmapEndTp;
    timing.scanEndTp = timing.heatmapEndTp;
    timing.candidateCount = static_cast<int>(request.scratch.candidates.size());
    timing.sortStartTp = timing.heatmapEndTp;
    SortCandidatesByScore(request.scratch.candidates);
    timing.sortEndTp = std::chrono::steady_clock::now();
    timing.nmsStartTp = timing.sortEndTp;
    ApplyFastNmsSuppression(request);
    timing.nmsEndTp = std::chrono::steady_clock::now();
    timing.selectedCount = static_cast<int>(request.scratch.candidates.size());
    RecordCandidateExtractionStats(request, timing);
}

} // namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal
