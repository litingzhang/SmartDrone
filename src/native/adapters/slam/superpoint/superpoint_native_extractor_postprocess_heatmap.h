struct TensorRtForwardStats {
    double h2dMs{0.0};
    double enqueueMs{0.0};
    double outputMs{0.0};
    double outputConvertMs{0.0};
    double syncMs{0.0};
    double gpuComputeMs{0.0};
    double gpuOutputMs{0.0};
    bool eventTimingEnabled{false};
    size_t h2dBytes{0};
    size_t d2hBytes{0};
    bool pinnedHostOutput{false};
};

struct SuperPointPostStats {
    double heatmapMs{0.0};
    double nmsMs{0.0};
    double scanMs{0.0};
    double sortMs{0.0};
    double descriptorMs{0.0};
    int candidateCount{0};
    int selectedCount{0};
    int descriptorCount{0};

    void Add(const SuperPointPostStats &other)
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

float At4D(const TensorBlob &blob, int b, int c, int y, int x)
{
    const int channels = blob.Dim(1);
    const int height = blob.Dim(2);
    const int width = blob.Dim(3);
    const size_t index =
        (((static_cast<size_t>(b) * channels + c) * height + y) * width + x);
    return blob.FloatData()[index];
}

float HalfToFloat(uint16_t value)
{
    const uint32_t sign = (static_cast<uint32_t>(value & 0x8000u)) << 16u;
    uint32_t exponent = (value >> 10u) & 0x1fu;
    uint32_t mantissa = value & 0x03ffu;
    uint32_t bits = 0;
    if (exponent == 0) {
        if (mantissa == 0) {
            bits = sign;
        } else {
            exponent = 1;
            while ((mantissa & 0x0400u) == 0) {
                mantissa <<= 1u;
                --exponent;
            }
            mantissa &= 0x03ffu;
            bits = sign | ((exponent + 112u) << 23u) | (mantissa << 13u);
        }
    } else if (exponent == 31) {
        bits = sign | 0x7f800000u | (mantissa << 13u);
    } else {
        bits = sign | ((exponent + 112u) << 23u) | (mantissa << 13u);
    }
    float out = 0.0f;
    std::memcpy(&out, &bits, sizeof(out));
    return out;
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
            cv::resize(images[i], resized, cv::Size(targetWidth, targetHeight), 0.0,
                       0.0, cv::INTER_LINEAR);
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

const cv::Mat &SuperPointNmsKernel()
{
    static const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(kSuperPointNmsRadius * 2 + 1, kSuperPointNmsRadius * 2 + 1));
    return kernel;
}

void BuildHeatmap(const TensorBlob &detector, int batch, cv::Mat &heatmap)
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
    const float *data = detector.FloatData();
    heatmap.create(gridHeight * kSuperPointCellSize,
                   gridWidth * kSuperPointCellSize, CV_32F);
    heatmap.setTo(cv::Scalar(0.0f));
    for (int gy = 0; gy < gridHeight; ++gy) {
        for (int gx = 0; gx < gridWidth; ++gx) {
            const size_t spatialOffset =
                static_cast<size_t>(gy) * static_cast<size_t>(gridWidth) +
                static_cast<size_t>(gx);
            float maxLogit = data[batchOffset + spatialOffset];
            for (int c = 1; c < 65; ++c) {
                maxLogit =
                    std::max(maxLogit,
                             data[batchOffset + static_cast<size_t>(c) * channelStride +
                                  spatialOffset]);
            }
            std::array<float, 65> expValues;
            float denom = 0.0f;
            for (int c = 0; c < 65; ++c) {
                expValues[static_cast<size_t>(c)] =
                    std::exp(data[batchOffset + static_cast<size_t>(c) * channelStride +
                                  spatialOffset] -
                             maxLogit);
                denom += expValues[static_cast<size_t>(c)];
            }
            if (!(denom > 0.0f)) {
                continue;
            }
            const float invDenom = 1.0f / denom;
            for (int c = 0; c < 64; ++c) {
                const int y = gy * kSuperPointCellSize + c / kSuperPointCellSize;
                const int x = gx * kSuperPointCellSize + c % kSuperPointCellSize;
                heatmap.ptr<float>(y)[x] = expValues[static_cast<size_t>(c)] * invDenom;
            }
        }
    }
}

void ExtractCandidates(const TensorBlob &detector, int batch, int targetWidth,
                       int targetHeight, int maxPoints,
                       SuperPointPostScratch &scratch,
                       SuperPointPostStats *stats)
{
    const auto heatmapStartTp = std::chrono::steady_clock::now();
    BuildHeatmap(detector, batch, scratch.heatmap);
    const auto heatmapEndTp = std::chrono::steady_clock::now();
    const auto nmsStartTp = heatmapEndTp;
    cv::dilate(scratch.heatmap, scratch.localMax, SuperPointNmsKernel());
    const auto nmsEndTp = std::chrono::steady_clock::now();
    std::vector<Candidate> &candidates = scratch.candidates;
    candidates.clear();
    candidates.reserve(static_cast<size_t>(std::max(1, maxPoints)) * 2);
    const auto scanStartTp = nmsEndTp;
    for (int y = kSuperPointBorder; y < targetHeight - kSuperPointBorder; ++y) {
        const float *scoreRow = scratch.heatmap.ptr<float>(y);
        const float *maxRow = scratch.localMax.ptr<float>(y);
        for (int x = kSuperPointBorder; x < targetWidth - kSuperPointBorder; ++x) {
            const float score = scoreRow[x];
            if (score <= kSuperPointThreshold) {
                continue;
            }
            if (score >= maxRow[x]) {
                candidates.push_back(Candidate{x, y, score});
            }
        }
    }
    const auto scanEndTp = std::chrono::steady_clock::now();
    const int candidateCount = static_cast<int>(candidates.size());
    const auto sortStartTp = scanEndTp;
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate &lhs, const Candidate &rhs) {
                  return lhs.score > rhs.score;
              });
    if (static_cast<int>(candidates.size()) > maxPoints) {
        candidates.resize(static_cast<size_t>(std::max(1, maxPoints)));
    }
    const auto sortEndTp = std::chrono::steady_clock::now();
    if (stats != nullptr) {
        stats->heatmapMs += DurationMs(heatmapStartTp, heatmapEndTp);
        stats->nmsMs += DurationMs(nmsStartTp, nmsEndTp);
        stats->scanMs += DurationMs(scanStartTp, scanEndTp);
        stats->sortMs += DurationMs(sortStartTp, sortEndTp);
        stats->candidateCount += candidateCount;
        stats->selectedCount += static_cast<int>(candidates.size());
    }
}

void ExtractCandidatesFastNms(const TensorBlob &detector, int batch,
                              int targetWidth, int targetHeight, int maxPoints,
                              SuperPointPostScratch &scratch,
                              SuperPointPostStats *stats)
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
    const float *data = detector.FloatData();
    std::vector<Candidate> &candidates = scratch.candidates;
    candidates.clear();
    candidates.reserve(static_cast<size_t>(std::max(1, maxPoints)) * 3);

    const auto heatmapStartTp = std::chrono::steady_clock::now();
    for (int gy = 0; gy < gridHeight; ++gy) {
        for (int gx = 0; gx < gridWidth; ++gx) {
            const size_t spatialOffset =
                static_cast<size_t>(gy) * static_cast<size_t>(gridWidth) +
                static_cast<size_t>(gx);
            float maxLogit = data[batchOffset + spatialOffset];
            for (int c = 1; c < 65; ++c) {
                maxLogit =
                    std::max(maxLogit,
                             data[batchOffset + static_cast<size_t>(c) * channelStride +
                                  spatialOffset]);
            }
            std::array<float, 65> expValues;
            float denom = 0.0f;
            for (int c = 0; c < 65; ++c) {
                expValues[static_cast<size_t>(c)] =
                    std::exp(data[batchOffset + static_cast<size_t>(c) * channelStride +
                                  spatialOffset] -
                             maxLogit);
                denom += expValues[static_cast<size_t>(c)];
            }
            if (!(denom > 0.0f)) {
                continue;
            }
            const float invDenom = 1.0f / denom;
            for (int c = 0; c < 64; ++c) {
                const int y = gy * kSuperPointCellSize + c / kSuperPointCellSize;
                const int x = gx * kSuperPointCellSize + c % kSuperPointCellSize;
                if (x < kSuperPointBorder || y < kSuperPointBorder ||
                    x >= targetWidth - kSuperPointBorder ||
                    y >= targetHeight - kSuperPointBorder) {
                    continue;
                }
                const float score = expValues[static_cast<size_t>(c)] * invDenom;
                if (score > kSuperPointThreshold) {
                    candidates.push_back(Candidate{x, y, score});
                }
            }
        }
    }
    const auto heatmapEndTp = std::chrono::steady_clock::now();
    const int candidateCount = static_cast<int>(candidates.size());

    const auto sortStartTp = heatmapEndTp;
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate &lhs, const Candidate &rhs) {
                  return lhs.score > rhs.score;
              });
    const auto sortEndTp = std::chrono::steady_clock::now();

    const auto nmsStartTp = sortEndTp;
    scratch.suppressionMask.assign(
        static_cast<size_t>(targetWidth) * static_cast<size_t>(targetHeight), 0);
    scratch.nmsCandidates.clear();
    scratch.nmsCandidates.reserve(static_cast<size_t>(std::max(1, maxPoints)));
    for (const Candidate &candidate : candidates) {
        const size_t center =
            static_cast<size_t>(candidate.y) * static_cast<size_t>(targetWidth) +
            static_cast<size_t>(candidate.x);
        if (scratch.suppressionMask[center] != 0) {
            continue;
        }
        scratch.nmsCandidates.push_back(candidate);
        const int y0 =
            std::max(kSuperPointBorder, candidate.y - kSuperPointNmsRadius);
        const int y1 = std::min(targetHeight - kSuperPointBorder - 1,
                                candidate.y + kSuperPointNmsRadius);
        const int x0 =
            std::max(kSuperPointBorder, candidate.x - kSuperPointNmsRadius);
        const int x1 = std::min(targetWidth - kSuperPointBorder - 1,
                                candidate.x + kSuperPointNmsRadius);
        for (int y = y0; y <= y1; ++y) {
            uint8_t *row = scratch.suppressionMask.data() +
                           static_cast<size_t>(y) * static_cast<size_t>(targetWidth);
            for (int x = x0; x <= x1; ++x) {
                row[x] = 1;
            }
        }
        if (static_cast<int>(scratch.nmsCandidates.size()) >=
            std::max(1, maxPoints)) {
            break;
        }
    }
    candidates.swap(scratch.nmsCandidates);
    const auto nmsEndTp = std::chrono::steady_clock::now();

    if (stats != nullptr) {
        stats->heatmapMs += DurationMs(heatmapStartTp, heatmapEndTp);
        stats->nmsMs += DurationMs(nmsStartTp, nmsEndTp);
        stats->scanMs += 0.0;
        stats->sortMs += DurationMs(sortStartTp, sortEndTp);
        stats->candidateCount += candidateCount;
        stats->selectedCount += static_cast<int>(candidates.size());
    }
}
