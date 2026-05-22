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

float BilinearAt(const TensorBlob &blob, int batch, int channel, float x,
                 float y)
{
    const int height = blob.Dim(2);
    const int width = blob.Dim(3);
    const float gx = std::clamp(x, 0.0f, static_cast<float>(width - 1));
    const float gy = std::clamp(y, 0.0f, static_cast<float>(height - 1));
    const int x0 = static_cast<int>(std::floor(gx));
    const int y0 = static_cast<int>(std::floor(gy));
    const int x1 = std::min(x0 + 1, width - 1);
    const int y1 = std::min(y0 + 1, height - 1);
    const float wx = gx - static_cast<float>(x0);
    const float wy = gy - static_cast<float>(y0);
    const float w00 = (1.0f - wx) * (1.0f - wy);
    const float w01 = wx * (1.0f - wy);
    const float w10 = (1.0f - wx) * wy;
    const float w11 = wx * wy;
    const float v00 = At4D(blob, batch, channel, y0, x0);
    const float v01 = At4D(blob, batch, channel, y0, x1);
    const float v10 = At4D(blob, batch, channel, y1, x0);
    const float v11 = At4D(blob, batch, channel, y1, x1);
    return w00 * v00 + w01 * v01 + w10 * v10 + w11 * v11;
}

void SampleDescriptorBilinear(const TensorBlob &descriptors, int batch, float x,
                              float y, float *out)
{
    const int channels = descriptors.Dim(1);
    const int height = descriptors.Dim(2);
    const int width = descriptors.Dim(3);
    const float gx = std::clamp(x, 0.0f, static_cast<float>(width - 1));
    const float gy = std::clamp(y, 0.0f, static_cast<float>(height - 1));
    const int x0 = static_cast<int>(std::floor(gx));
    const int y0 = static_cast<int>(std::floor(gy));
    const int x1 = std::min(x0 + 1, width - 1);
    const int y1 = std::min(y0 + 1, height - 1);
    const float wx = gx - static_cast<float>(x0);
    const float wy = gy - static_cast<float>(y0);
    const float w00 = (1.0f - wx) * (1.0f - wy);
    const float w01 = wx * (1.0f - wy);
    const float w10 = (1.0f - wx) * wy;
    const float w11 = wx * wy;
    const size_t batchOffset =
        static_cast<size_t>(batch) * static_cast<size_t>(channels) *
        static_cast<size_t>(height) * static_cast<size_t>(width);
    const size_t channelStride =
        static_cast<size_t>(height) * static_cast<size_t>(width);
    const float *data = descriptors.FloatData();
    for (int c = 0; c < kSuperPointDescriptorDim; ++c) {
        const size_t channelOffset =
            batchOffset + static_cast<size_t>(c) * channelStride;
        const float v00 =
            data[channelOffset + static_cast<size_t>(y0) * width + x0];
        const float v01 =
            data[channelOffset + static_cast<size_t>(y0) * width + x1];
        const float v10 =
            data[channelOffset + static_cast<size_t>(y1) * width + x0];
        const float v11 =
            data[channelOffset + static_cast<size_t>(y1) * width + x1];
        out[c] = w00 * v00 + w01 * v01 + w10 * v10 + w11 * v11;
    }
    NormalizeVector(out, kSuperPointDescriptorDim);
}

void SampleDescriptorNearest(const TensorBlob &descriptors, int batch, float x,
                             float y, float *out)
{
    const int channels = descriptors.Dim(1);
    const int height = descriptors.Dim(2);
    const int width = descriptors.Dim(3);
    const int ix = std::clamp(static_cast<int>(std::round(x)), 0, width - 1);
    const int iy = std::clamp(static_cast<int>(std::round(y)), 0, height - 1);
    const size_t batchOffset =
        static_cast<size_t>(batch) * static_cast<size_t>(channels) *
        static_cast<size_t>(height) * static_cast<size_t>(width);
    const size_t channelStride =
        static_cast<size_t>(height) * static_cast<size_t>(width);
    const size_t spatialOffset =
        static_cast<size_t>(iy) * static_cast<size_t>(width) +
        static_cast<size_t>(ix);
    const float *data = descriptors.FloatData();
    for (int c = 0; c < kSuperPointDescriptorDim; ++c) {
        out[c] = data[batchOffset + static_cast<size_t>(c) * channelStride +
                      spatialOffset];
    }
    NormalizeVector(out, kSuperPointDescriptorDim);
}

void BuildDescriptorGridHwc(const TensorBlob &descriptors, int batch,
                            std::vector<float> &hwc)
{
    const int channels = descriptors.Dim(1);
    const int height = descriptors.Dim(2);
    const int width = descriptors.Dim(3);
    const size_t spatial =
        static_cast<size_t>(height) * static_cast<size_t>(width);
    const size_t batchOffset =
        static_cast<size_t>(batch) * static_cast<size_t>(channels) * spatial;
    const float *data = descriptors.FloatData();
    hwc.resize(spatial * static_cast<size_t>(channels));

    constexpr int kTilePixels = 32;
    for (int tileStart = 0; tileStart < static_cast<int>(spatial);
         tileStart += kTilePixels) {
        const int tileEnd =
            std::min(tileStart + kTilePixels, static_cast<int>(spatial));
        for (int c = 0; c < channels; ++c) {
            const float *src = data + batchOffset + static_cast<size_t>(c) * spatial +
                               static_cast<size_t>(tileStart);
            for (int index = tileStart; index < tileEnd; ++index) {
                hwc[static_cast<size_t>(index) * static_cast<size_t>(channels) +
                    static_cast<size_t>(c)] = src[index - tileStart];
            }
        }
    }
}

void SampleDescriptorBilinearHwc(const std::vector<float> &hwc, int height,
                                 int width, float x, float y, float *out)
{
    const float gx = std::clamp(x, 0.0f, static_cast<float>(width - 1));
    const float gy = std::clamp(y, 0.0f, static_cast<float>(height - 1));
    const int x0 = static_cast<int>(std::floor(gx));
    const int y0 = static_cast<int>(std::floor(gy));
    const int x1 = std::min(x0 + 1, width - 1);
    const int y1 = std::min(y0 + 1, height - 1);
    const float wx = gx - static_cast<float>(x0);
    const float wy = gy - static_cast<float>(y0);
    const float w00 = (1.0f - wx) * (1.0f - wy);
    const float w01 = wx * (1.0f - wy);
    const float w10 = (1.0f - wx) * wy;
    const float w11 = wx * wy;
    const size_t base00 = (static_cast<size_t>(y0) * static_cast<size_t>(width) +
                           static_cast<size_t>(x0)) *
                          kSuperPointDescriptorDim;
    const size_t base01 = (static_cast<size_t>(y0) * static_cast<size_t>(width) +
                           static_cast<size_t>(x1)) *
                          kSuperPointDescriptorDim;
    const size_t base10 = (static_cast<size_t>(y1) * static_cast<size_t>(width) +
                           static_cast<size_t>(x0)) *
                          kSuperPointDescriptorDim;
    const size_t base11 = (static_cast<size_t>(y1) * static_cast<size_t>(width) +
                           static_cast<size_t>(x1)) *
                          kSuperPointDescriptorDim;
    for (int c = 0; c < kSuperPointDescriptorDim; ++c) {
        const float v00 = hwc[base00 + static_cast<size_t>(c)];
        const float v01 = hwc[base01 + static_cast<size_t>(c)];
        const float v10 = hwc[base10 + static_cast<size_t>(c)];
        const float v11 = hwc[base11 + static_cast<size_t>(c)];
        out[c] = w00 * v00 + w01 * v01 + w10 * v10 + w11 * v11;
    }
    NormalizeVector(out, kSuperPointDescriptorDim);
}

void MatchStereoPairs(const SuperPointFeatureSet &leftRaw,
                      const SuperPointFeatureSet &rightRaw, int maxPoints,
                      SuperPointFeatureSet &leftOut,
                      SuperPointFeatureSet &rightOut)
{
    leftOut = SuperPointFeatureSet{};
    rightOut = SuperPointFeatureSet{};
    if (leftRaw.descriptors.empty() || rightRaw.descriptors.empty() ||
        leftRaw.descriptors.type() != CV_32F ||
        rightRaw.descriptors.type() != CV_32F ||
        leftRaw.descriptors.cols != rightRaw.descriptors.cols) {
        return;
    }

    const int candidateLimit =
        EnvIntClamped("SMART_DRONE_DESCRIPTOR_SUPPLEMENT_CANDIDATES",
                      std::max(1, maxPoints), 1, 4096);
    const int leftCount = std::min({static_cast<int>(leftRaw.keypoints.size()),
                                    leftRaw.descriptors.rows, candidateLimit});
    const int rightCount = std::min({static_cast<int>(rightRaw.keypoints.size()),
                                     rightRaw.descriptors.rows, candidateLimit});
    const int descriptorDim = leftRaw.descriptors.cols;
    if (leftCount <= 0 || rightCount <= 0 || descriptorDim <= 0) {
        return;
    }

    std::vector<int> bestRightForLeft(static_cast<size_t>(leftCount), -1);
    std::vector<float> bestDistForLeft(static_cast<size_t>(leftCount),
                                       std::numeric_limits<float>::infinity());
    std::vector<float> secondDistForLeft(static_cast<size_t>(leftCount),
                                         std::numeric_limits<float>::infinity());
    std::vector<int> bestLeftForRight(static_cast<size_t>(rightCount), -1);
    std::vector<float> bestDistForRight(static_cast<size_t>(rightCount),
                                        std::numeric_limits<float>::infinity());
    std::vector<int> rightOrder(static_cast<size_t>(rightCount));
    std::iota(rightOrder.begin(), rightOrder.end(), 0);
    std::sort(rightOrder.begin(), rightOrder.end(), [&](int lhs, int rhs) {
        const float ly = rightRaw.keypoints[static_cast<size_t>(lhs)].y;
        const float ry = rightRaw.keypoints[static_cast<size_t>(rhs)].y;
        if (std::abs(ly - ry) > 1.0e-6f) {
            return ly < ry;
        }
        return lhs < rhs;
    });

    for (int li = 0; li < leftCount; ++li) {
        const cv::Point2f &lp = leftRaw.keypoints[static_cast<size_t>(li)];
        const float minY = lp.y - kStereoMaxYDiffPx;
        const float maxY = lp.y + kStereoMaxYDiffPx;
        const auto firstRight = std::lower_bound(
            rightOrder.begin(), rightOrder.end(), minY,
            [&](int index, float value) {
                return rightRaw.keypoints[static_cast<size_t>(index)].y < value;
            });
        const auto lastRight = std::upper_bound(
            firstRight, rightOrder.end(), maxY, [&](float value, int index) {
                return value < rightRaw.keypoints[static_cast<size_t>(index)].y;
            });
        const float *ld = leftRaw.descriptors.ptr<float>(li);
        for (auto it = firstRight; it != lastRight; ++it) {
            const int ri = *it;
            const cv::Point2f &rp = rightRaw.keypoints[static_cast<size_t>(ri)];
            const float disparity = lp.x - rp.x;
            if (disparity < StereoMinDisparityPx() ||
                disparity > kStereoMaxDisparityPx) {
                continue;
            }
            const float *rd = rightRaw.descriptors.ptr<float>(ri);
            float dist = 0.0f;
            for (int d = 0; d < descriptorDim; ++d) {
                const float delta = ld[d] - rd[d];
                dist += delta * delta;
            }
            if (!std::isfinite(dist)) {
                continue;
            }
            if (dist < bestDistForLeft[static_cast<size_t>(li)]) {
                secondDistForLeft[static_cast<size_t>(li)] =
                    bestDistForLeft[static_cast<size_t>(li)];
                bestDistForLeft[static_cast<size_t>(li)] = dist;
                bestRightForLeft[static_cast<size_t>(li)] = ri;
            } else if (dist < secondDistForLeft[static_cast<size_t>(li)]) {
                secondDistForLeft[static_cast<size_t>(li)] = dist;
            }
            if (dist < bestDistForRight[static_cast<size_t>(ri)]) {
                bestDistForRight[static_cast<size_t>(ri)] = dist;
                bestLeftForRight[static_cast<size_t>(ri)] = li;
            }
        }
    }

    struct Pair {
        int left{0};
        int right{0};
        float distance{0.0f};
        float disparity{0.0f};
    };
    std::vector<Pair> pairs;
    for (int li = 0; li < leftCount; ++li) {
        const int ri = bestRightForLeft[static_cast<size_t>(li)];
        if (ri < 0 || bestLeftForRight[static_cast<size_t>(ri)] != li) {
            continue;
        }
        const float bestDist = bestDistForLeft[static_cast<size_t>(li)];
        const float secondDist = secondDistForLeft[static_cast<size_t>(li)];
        if (std::isfinite(secondDist) &&
            bestDist >= (kStereoRatio * kStereoRatio) * secondDist) {
            continue;
        }
        const cv::Point2f &lp = leftRaw.keypoints[static_cast<size_t>(li)];
        const cv::Point2f &rp = rightRaw.keypoints[static_cast<size_t>(ri)];
        const float disparity = lp.x - rp.x;
        pairs.push_back(Pair{li, ri, bestDist, disparity});
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair &lhs, const Pair &rhs) {
        if (std::abs(lhs.distance - rhs.distance) > 1.0e-6f) {
            return lhs.distance < rhs.distance;
        }
        return lhs.disparity > rhs.disparity;
    });
    if (static_cast<int>(pairs.size()) > std::max(1, maxPoints)) {
        pairs.resize(static_cast<size_t>(std::max(1, maxPoints)));
    }

    leftOut.keypoints.reserve(pairs.size());
    rightOut.keypoints.reserve(pairs.size());
    leftOut.descriptors =
        cv::Mat(static_cast<int>(pairs.size()), leftRaw.descriptors.cols, CV_32F);
    rightOut.descriptors = cv::Mat(static_cast<int>(pairs.size()),
                                   rightRaw.descriptors.cols, CV_32F);
    for (size_t i = 0; i < pairs.size(); ++i) {
        leftOut.keypoints.push_back(
            leftRaw.keypoints[static_cast<size_t>(pairs[i].left)]);
        rightOut.keypoints.push_back(
            rightRaw.keypoints[static_cast<size_t>(pairs[i].right)]);
        leftRaw.descriptors.row(pairs[i].left)
            .copyTo(leftOut.descriptors.row(static_cast<int>(i)));
        rightRaw.descriptors.row(pairs[i].right)
            .copyTo(rightOut.descriptors.row(static_cast<int>(i)));
    }
}

bool IsStereoFeaturePairNearExisting(
    const cv::Point2f &leftPoint, const cv::Point2f &rightPoint,
    const SuperPointFeatureSet &existingLeft,
    const SuperPointFeatureSet &existingRight)
{
    const float minDistanceSq = kStereoMergeDistancePx * kStereoMergeDistancePx;
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
