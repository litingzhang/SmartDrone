bool SuperPointNativeExtractor::Impl::DetectAndComputeBatch(const std::vector<cv::Mat> &grayImages,
                           int maxPoints, int descriptorLimit,
                           std::vector<SuperPointFeatureSet> &outputs,
                           double *inputMs, double *forwardMs, double *postMs,
                           std::string *err)
{
    outputs.assign(grayImages.size(), SuperPointFeatureSet{});
    if (!trtEngine || grayImages.empty()) {
        if (err != nullptr) {
            *err = "SuperPoint TensorRT backend is not ready";
        }
        return false;
    }
    double inputTotalMs = 0.0;
    double forwardTotalMs = 0.0;
    double postTotalMs = 0.0;
    lastSuperPointBatchedForward = false;
    lastSuperPointForwardStats = TensorRtForwardStats{};
    lastSuperPointPostStats = SuperPointPostStats{};
    const int batchSize = static_cast<int>(grayImages.size());
    const int batchTargetHeight =
        inputHeight > 0 ? inputHeight : grayImages.front().rows;
    const int batchTargetWidth =
        inputWidth > 0 ? inputWidth : grayImages.front().cols;
    if (batchSize > 1 && !superPointBatchDisabled &&
        trtEngine->SupportsBatchSize(batchSize)) {
        const auto inputStartTp = std::chrono::steady_clock::now();
        BuildInputBatch(grayImages, batchTargetHeight, batchTargetWidth,
                        superPointInputBatch);
        const auto inputEndTp = std::chrono::steady_clock::now();
        const auto forwardStartTp = inputEndTp;
        std::string batchErr;
        TensorRtForwardStats batchStats;
        if (trtEngine->Forward(superPointInputBatch, batchSize, batchTargetHeight,
                               batchTargetWidth, detector, descriptors,
                               &batchStats, &batchErr)) {
            const auto forwardEndTp = std::chrono::steady_clock::now();
            const auto postStartTp = forwardEndTp;
            for (size_t batchIndex = 0; batchIndex < grayImages.size();
                 ++batchIndex) {
                if (!PopulateOutputFromTensors(
                        detector, descriptors, static_cast<int>(batchIndex),
                        grayImages[batchIndex], batchTargetHeight, batchTargetWidth,
                        maxPoints, descriptorLimit, outputs[batchIndex],
                        mainPostScratch, &lastSuperPointPostStats, err)) {
                    return false;
                }
            }
            const auto postEndTp = std::chrono::steady_clock::now();
            inputTotalMs += DurationMs(inputStartTp, inputEndTp);
            forwardTotalMs += DurationMs(forwardStartTp, forwardEndTp);
            postTotalMs += DurationMs(postStartTp, postEndTp);
            lastSuperPointForwardStats = batchStats;
            lastSuperPointBatchedForward = true;
            if (inputMs != nullptr) {
                *inputMs = inputTotalMs;
            }
            if (forwardMs != nullptr) {
                *forwardMs = forwardTotalMs;
            }
            if (postMs != nullptr) {
                *postMs = postTotalMs;
            }
            return true;
        }
        superPointBatchDisabled = true;
        std::cerr << "[superpoint_trt] warning: batch_size=" << batchSize
                  << " forward failed; falling back to single-image inference";
        if (!batchErr.empty()) {
            std::cerr << " err=" << batchErr;
        }
        std::cerr << "\n";
    }
    for (size_t batchIndex = 0; batchIndex < grayImages.size(); ++batchIndex) {
        const int targetHeight =
            inputHeight > 0 ? inputHeight : grayImages[batchIndex].rows;
        const int targetWidth =
            inputWidth > 0 ? inputWidth : grayImages[batchIndex].cols;
        const auto inputStartTp = std::chrono::steady_clock::now();
        BuildInputBatch({grayImages[batchIndex]}, targetHeight, targetWidth,
                        superPointInputBatch);
        const auto inputEndTp = std::chrono::steady_clock::now();
        const auto forwardStartTp = inputEndTp;
        TensorRtForwardStats singleStats;
        if (!trtEngine->Forward(superPointInputBatch, 1, targetHeight,
                                targetWidth, detector, descriptors, &singleStats,
                                err)) {
            return false;
        }
        const auto forwardEndTp = std::chrono::steady_clock::now();
        const auto postStartTp = forwardEndTp;
        if (!PopulateOutputFromTensors(
                detector, descriptors, 0, grayImages[batchIndex], targetHeight,
                targetWidth, maxPoints, descriptorLimit, outputs[batchIndex],
                mainPostScratch, &lastSuperPointPostStats, err)) {
            return false;
        }
        const auto postEndTp = std::chrono::steady_clock::now();
        inputTotalMs += DurationMs(inputStartTp, inputEndTp);
        forwardTotalMs += DurationMs(forwardStartTp, forwardEndTp);
        postTotalMs += DurationMs(postStartTp, postEndTp);
        lastSuperPointForwardStats.h2dMs += singleStats.h2dMs;
        lastSuperPointForwardStats.enqueueMs += singleStats.enqueueMs;
        lastSuperPointForwardStats.outputMs += singleStats.outputMs;
        lastSuperPointForwardStats.outputConvertMs += singleStats.outputConvertMs;
        lastSuperPointForwardStats.syncMs += singleStats.syncMs;
        lastSuperPointForwardStats.gpuComputeMs += singleStats.gpuComputeMs;
        lastSuperPointForwardStats.gpuOutputMs += singleStats.gpuOutputMs;
        lastSuperPointForwardStats.eventTimingEnabled =
            lastSuperPointForwardStats.eventTimingEnabled ||
            singleStats.eventTimingEnabled;
        lastSuperPointForwardStats.h2dBytes += singleStats.h2dBytes;
        lastSuperPointForwardStats.d2hBytes += singleStats.d2hBytes;
    }
    if (inputMs != nullptr) {
        *inputMs = inputTotalMs;
    }
    if (forwardMs != nullptr) {
        *forwardMs = forwardTotalMs;
    }
    if (postMs != nullptr) {
        *postMs = postTotalMs;
    }
    return true;
}

bool SuperPointNativeExtractor::Impl::MatchWithLightGlue(const SuperPointFeatureSet &leftRaw,
                        const SuperPointFeatureSet &rightRaw, int maxPoints,
                        int imageWidth, int imageHeight,
                        SuperPointFeatureSet &leftOut,
                        SuperPointFeatureSet &rightOut, double *matchMs,
                        std::string *err)
{
    leftOut = SuperPointFeatureSet{};
    rightOut = SuperPointFeatureSet{};
    lastLightGlueMutualCount = 0;
    lastLightGlueScorePassCount = 0;
    lastLightGlueGeometryPassCount = 0;
    lastLightGlueAcceptedCount = 0;
    lastLightGlueMinScore = 0.0f;
    lastLightGlueMaxScore = 0.0f;
    lastLightGlueDecodeMs = 0.0;
    lastLightGlueOrientation = "none";
    lastLightGlueScoresLookLog = false;
    lastLightGlueForwardStats = TensorRtForwardStats{};
    lastLightGlueRequestedPointCount = 0;
    lastLightGlueInputPointCount = 0;
    lastLightGlueStaticShapeFallback = false;
    if (!lightGlueEngine || lightGluePointCount <= 0 ||
        leftRaw.descriptors.empty() || rightRaw.descriptors.empty() ||
        leftRaw.descriptors.type() != CV_32F ||
        rightRaw.descriptors.type() != CV_32F ||
        leftRaw.descriptors.cols != kSuperPointDescriptorDim ||
        rightRaw.descriptors.cols != kSuperPointDescriptorDim) {
        return false;
    }

    const int leftCount =
        std::min({static_cast<int>(leftRaw.keypoints.size()),
                  leftRaw.descriptors.rows, lightGluePointCount});
    const int rightCount =
        std::min({static_cast<int>(rightRaw.keypoints.size()),
                  rightRaw.descriptors.rows, lightGluePointCount});
    if (leftCount <= 0 || rightCount <= 0) {
        return false;
    }

    auto packInputs = [&](int pointCount) {
        const size_t kptsSize = static_cast<size_t>(pointCount) * 2;
        const size_t descSize =
            static_cast<size_t>(pointCount) * kSuperPointDescriptorDim;
        lightGlueKpts0.assign(kptsSize, -1000.0f);
        lightGlueKpts1.assign(kptsSize, -1000.0f);
        lightGlueDesc0.assign(descSize, 0.0f);
        lightGlueDesc1.assign(descSize, 0.0f);
        const int leftCopyCount = std::min(pointCount, leftCount);
        const int rightCopyCount = std::min(pointCount, rightCount);
        for (int i = 0; i < leftCopyCount; ++i) {
            lightGlueKpts0[static_cast<size_t>(i) * 2] =
                leftRaw.keypoints[static_cast<size_t>(i)].x;
            lightGlueKpts0[static_cast<size_t>(i) * 2 + 1] =
                leftRaw.keypoints[static_cast<size_t>(i)].y;
            const float *src = leftRaw.descriptors.ptr<float>(i);
            std::copy(src, src + kSuperPointDescriptorDim,
                      lightGlueDesc0.data() +
                          static_cast<size_t>(i) * kSuperPointDescriptorDim);
        }
        for (int i = 0; i < rightCopyCount; ++i) {
            lightGlueKpts1[static_cast<size_t>(i) * 2] =
                rightRaw.keypoints[static_cast<size_t>(i)].x;
            lightGlueKpts1[static_cast<size_t>(i) * 2 + 1] =
                rightRaw.keypoints[static_cast<size_t>(i)].y;
            const float *src = rightRaw.descriptors.ptr<float>(i);
            std::copy(src, src + kSuperPointDescriptorDim,
                      lightGlueDesc1.data() +
                          static_cast<size_t>(i) * kSuperPointDescriptorDim);
        }
    };

    const int requestedPointCount = std::min(
        {std::max(leftCount, rightCount), maxPoints, lightGluePointCount});
    int lightGlueInputPointCount = lightGlueDynamicPointCountDisabled
                                       ? lightGluePointCount
                                       : requestedPointCount;
    lastLightGlueRequestedPointCount = requestedPointCount;
    lastLightGlueInputPointCount = lightGlueInputPointCount;
    const auto matchStart = std::chrono::steady_clock::now();
    const std::array<float, 2> imageSize{static_cast<float>(imageWidth),
                                         static_cast<float>(imageHeight)};
    TensorRtForwardStats matchStats;
    packInputs(lightGlueInputPointCount);
    if (!lightGlueEngine->Forward(lightGlueKpts0, lightGlueKpts1,
                                  lightGlueDesc0, lightGlueDesc1, imageSize,
                                  imageSize, lightGlueInputPointCount,
                                  lightGlueScores, &matchStats, err)) {
        if (lightGlueInputPointCount < lightGluePointCount) {
            std::string fallbackErr;
            lightGlueDynamicPointCountDisabled = true;
            lastLightGlueStaticShapeFallback = true;
            lightGlueInputPointCount = lightGluePointCount;
            lastLightGlueInputPointCount = lightGlueInputPointCount;
            packInputs(lightGlueInputPointCount);
            if (!lightGlueEngine->Forward(
                    lightGlueKpts0, lightGlueKpts1, lightGlueDesc0, lightGlueDesc1,
                    imageSize, imageSize, lightGlueInputPointCount, lightGlueScores,
                    &matchStats, &fallbackErr)) {
                if (err != nullptr) {
                    *err = std::move(fallbackErr);
                }
                return false;
            }
        } else {
            return false;
        }
    }
    const auto matchEnd = std::chrono::steady_clock::now();
    lastLightGlueForwardStats = matchStats;
    if (matchMs != nullptr) {
        *matchMs = DurationMs(matchStart, matchEnd);
    }

    const auto decodeStart = std::chrono::steady_clock::now();
    const int matrixRows = lightGlueScores.dims.size() == 3
                               ? lightGlueScores.Dim(1)
                               : lightGlueScores.Dim(0);
    const int matrixCols = lightGlueScores.dims.size() == 3
                               ? lightGlueScores.Dim(2)
                               : lightGlueScores.Dim(1);
    const int outLeftCount = std::min(leftCount, matrixRows);
    const int outRightCount = std::min(rightCount, matrixCols);
    struct Pair {
        int left{0};
        int right{0};
        float score{0.0f};
        float disparity{0.0f};
    };
    float minFiniteScore = std::numeric_limits<float>::infinity();
    float maxFiniteScore = -std::numeric_limits<float>::infinity();
    auto scoreAtDirect = [&](int li, int ri) {
        return lightGlueScores.FloatData()[static_cast<size_t>(li) *
                                               static_cast<size_t>(matrixCols) +
                                           static_cast<size_t>(ri)];
    };
    for (int li = 0; li < outLeftCount; ++li) {
        for (int ri = 0; ri < outRightCount; ++ri) {
            const float score = scoreAtDirect(li, ri);
            if (!std::isfinite(score)) {
                continue;
            }
            minFiniteScore = std::min(minFiniteScore, score);
            maxFiniteScore = std::max(maxFiniteScore, score);
        }
    }
    if (std::isfinite(minFiniteScore) && std::isfinite(maxFiniteScore)) {
        lastLightGlueMinScore = minFiniteScore;
        lastLightGlueMaxScore = maxFiniteScore;
        lastLightGlueScoresLookLog = maxFiniteScore <= 0.0f;
    }
    auto buildPairs = [&](bool transpose, int *mutualCount, int *scorePassCount,
                          int *geometryPassCount) {
        std::vector<Pair> candidatePairs;
        std::vector<int> bestRightForLeft(static_cast<size_t>(outLeftCount), -1);
        std::vector<float> bestScoreForLeft(
            static_cast<size_t>(outLeftCount),
            -std::numeric_limits<float>::infinity());
        std::vector<int> bestLeftForRight(static_cast<size_t>(outRightCount), -1);
        std::vector<float> bestScoreForRight(
            static_cast<size_t>(outRightCount),
            -std::numeric_limits<float>::infinity());
        auto scoreAt = [&](int li, int ri) {
            return transpose ? scoreAtDirect(ri, li) : scoreAtDirect(li, ri);
        };
        for (int li = 0; li < outLeftCount; ++li) {
            for (int ri = 0; ri < outRightCount; ++ri) {
                const float score = scoreAt(li, ri);
                if (!std::isfinite(score)) {
                    continue;
                }
                if (score > bestScoreForLeft[static_cast<size_t>(li)]) {
                    bestScoreForLeft[static_cast<size_t>(li)] = score;
                    bestRightForLeft[static_cast<size_t>(li)] = ri;
                }
                if (score > bestScoreForRight[static_cast<size_t>(ri)]) {
                    bestScoreForRight[static_cast<size_t>(ri)] = score;
                    bestLeftForRight[static_cast<size_t>(ri)] = li;
                }
            }
        }
        candidatePairs.reserve(static_cast<size_t>(outLeftCount));
        for (int li = 0; li < outLeftCount; ++li) {
            const int ri = bestRightForLeft[static_cast<size_t>(li)];
            const float score = bestScoreForLeft[static_cast<size_t>(li)];
            if (ri < 0 || ri >= outRightCount ||
                bestLeftForRight[static_cast<size_t>(ri)] != li) {
                continue;
            }
            ++(*mutualCount);
            const float probability =
                lastLightGlueScoresLookLog ? std::exp(score) : score;
            if (!std::isfinite(probability) || probability < lightGlueMinScore) {
                continue;
            }
            ++(*scorePassCount);
            const cv::Point2f &lp = leftRaw.keypoints[static_cast<size_t>(li)];
            const cv::Point2f &rp = rightRaw.keypoints[static_cast<size_t>(ri)];
            const float disparity = lp.x - rp.x;
            if (std::abs(lp.y - rp.y) > lightGlueMaxYDiffPx ||
                disparity <= lightGlueMinDisparityPx ||
                disparity > kStereoMaxDisparityPx) {
                continue;
            }
            ++(*geometryPassCount);
            candidatePairs.push_back(Pair{li, ri, score, disparity});
        }
        return candidatePairs;
    };
    int directMutualCount = 0;
    int directScorePassCount = 0;
    int directGeometryPassCount = 0;
    const std::string orientation = LowerCopy(
        std::getenv("SMART_DRONE_LIGHTGLUE_SCORE_ORIENTATION") != nullptr
            ? std::getenv("SMART_DRONE_LIGHTGLUE_SCORE_ORIENTATION")
            : "auto");
    const bool useDirect = orientation != "transpose";
    const bool useTranspose = orientation != "direct";
    std::vector<Pair> directPairs;
    if (useDirect) {
        directPairs = buildPairs(false, &directMutualCount, &directScorePassCount,
                                 &directGeometryPassCount);
    }
    int transposeMutualCount = 0;
    int transposeScorePassCount = 0;
    int transposeGeometryPassCount = 0;
    std::vector<Pair> transposePairs;
    if (useTranspose) {
        transposePairs =
            buildPairs(true, &transposeMutualCount, &transposeScorePassCount,
                       &transposeGeometryPassCount);
    }
    std::vector<Pair> pairs = std::move(directPairs);
    lastLightGlueMutualCount = directMutualCount;
    lastLightGlueScorePassCount = directScorePassCount;
    lastLightGlueGeometryPassCount = directGeometryPassCount;
    lastLightGlueOrientation = useDirect ? "direct" : "transpose";
    if (useTranspose &&
        (!useDirect || transposeGeometryPassCount > directGeometryPassCount)) {
        pairs = std::move(transposePairs);
        lastLightGlueMutualCount = transposeMutualCount;
        lastLightGlueScorePassCount = transposeScorePassCount;
        lastLightGlueGeometryPassCount = transposeGeometryPassCount;
        lastLightGlueOrientation = "transpose";
    }
    lastLightGlueAcceptedCount = static_cast<int>(pairs.size());
    std::sort(pairs.begin(), pairs.end(), [](const Pair &lhs, const Pair &rhs) {
        if (std::abs(lhs.score - rhs.score) > 1.0e-6f) {
            return lhs.score > rhs.score;
        }
        return lhs.disparity > rhs.disparity;
    });
    if (static_cast<int>(pairs.size()) > std::max(1, maxPoints)) {
        pairs.resize(static_cast<size_t>(std::max(1, maxPoints)));
    }
    const auto decodeEnd = std::chrono::steady_clock::now();
    lastLightGlueDecodeMs = DurationMs(decodeStart, decodeEnd);

    leftOut.keypoints.reserve(pairs.size());
    rightOut.keypoints.reserve(pairs.size());
    leftOut.descriptors = cv::Mat(static_cast<int>(pairs.size()),
                                  kSuperPointDescriptorDim, CV_32F);
    rightOut.descriptors = cv::Mat(static_cast<int>(pairs.size()),
                                   kSuperPointDescriptorDim, CV_32F);
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
    return true;
}
