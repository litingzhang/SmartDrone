struct SuperPointNativeExtractor::Impl {
    std::unique_ptr<TensorRtSuperPointEngine> trtEngine;
    std::unique_ptr<TensorRtLightGlueEngine> lightGlueEngine;
    TensorBlob detector;
    TensorBlob descriptors;
    TensorBlob lightGlueScores;
    std::vector<float> superPointInputBatch;
    std::vector<float> lightGlueKpts0;
    std::vector<float> lightGlueKpts1;
    std::vector<float> lightGlueDesc0;
    std::vector<float> lightGlueDesc1;
    SuperPointPostScratch mainPostScratch;
    int inputHeight{0};
    int inputWidth{0};
    int lightGluePointCount{0};
    float lightGlueMinScore{0.02f};
    float lightGlueMaxYDiffPx{1.5f};
    float lightGlueMinDisparityPx{0.8f};
    int lightGlueEmptyDisableThreshold{3};
    int lightGlueLowYieldDisableThreshold{3};
    int lightGlueLowYieldMinPairs{8};
    int lightGlueEmptyCooldownFrames{120};
    int lightGlueEmptyCount{0};
    int lightGlueLowYieldCount{0};
    int lightGlueSkipRemaining{0};
    int lightGlueFrameCounter{0};
    int lastLightGlueMutualCount{0};
    int lastLightGlueScorePassCount{0};
    int lastLightGlueGeometryPassCount{0};
    int lastLightGlueAcceptedCount{0};
    int lastLightGlueRequestedPointCount{0};
    int lastLightGlueInputPointCount{0};
    bool lastLightGlueStaticShapeFallback{false};
    float lastLightGlueMinScore{0.0f};
    float lastLightGlueMaxScore{0.0f};
    double lastLightGlueDecodeMs{0.0};
    std::string lastLightGlueOrientation{"none"};
    bool lastLightGlueScoresLookLog{false};
    bool superPointBatchDisabled{false};
    bool lastSuperPointBatchedForward{false};
    TensorRtForwardStats lastSuperPointForwardStats{};
    TensorRtForwardStats lastLightGlueForwardStats{};
    SuperPointPostStats lastSuperPointPostStats{};
    bool lightGlueDynamicPointCountDisabled{false};

    bool Load(const std::string &repoPath, const std::string &deviceText,
              std::string *err)
    {
        const std::string device = LowerCopy(deviceText);
        if (!device.empty() && device != "auto" && device != "cuda") {
            if (err != nullptr) {
                *err = "native TensorRT SuperPoint only supports device=auto|cuda";
            }
            return false;
        }
        const SuperPointTensorRtRuntimeOptions runtimeOptions =
            LoadSuperPointTensorRtRuntimeOptions();
        const int widthHint = runtimeOptions.inputMaxWidth;
        const int heightHint = runtimeOptions.inputMaxHeight;
        const std::filesystem::path enginePath =
            ResolveSuperPointEnginePath(repoPath, widthHint, heightHint);
        if (enginePath.empty()) {
            if (err != nullptr) {
                *err = "SuperPoint TensorRT engine not found under repo: " + repoPath;
            }
            return false;
        }
        auto candidate = std::make_unique<TensorRtSuperPointEngine>();
        if (!candidate->Load(enginePath, err)) {
            return false;
        }
        if (!candidate->PreferredInputSize(inputHeight, inputWidth)) {
            inputHeight = heightHint;
            inputWidth = widthHint;
        }
        trtEngine = std::move(candidate);
        std::cerr << "[superpoint_trt] loaded engine=" << enginePath.string()
                  << " input=" << inputWidth << "x" << inputHeight << "\n";

        const std::filesystem::path lightGluePath =
            ResolveLightGlueEnginePath(repoPath, runtimeOptions.lightGluePoints);
        if (!lightGluePath.empty()) {
            auto matcher = std::make_unique<TensorRtLightGlueEngine>();
            std::string lgErr;
            if (matcher->Load(lightGluePath, &lgErr)) {
                const int fixedPointCount = matcher->FixedPointCount();
                lightGlueEngine = std::move(matcher);
                lightGluePointCount =
                    fixedPointCount > 0 ? fixedPointCount
                                        : runtimeOptions.lightGluePoints;
                lightGlueDynamicPointCountDisabled = fixedPointCount > 0;
                lightGlueMinScore = runtimeOptions.lightGlueMinScore;
                lightGlueMaxYDiffPx = runtimeOptions.lightGlueMaxYDiffPx;
                lightGlueMinDisparityPx = runtimeOptions.lightGlueMinDisparityPx;
                lightGlueEmptyDisableThreshold =
                    runtimeOptions.lightGlueEmptyDisableThreshold;
                lightGlueLowYieldDisableThreshold =
                    runtimeOptions.lightGlueLowYieldDisableThreshold;
                lightGlueLowYieldMinPairs = runtimeOptions.lightGlueLowYieldMinPairs;
                lightGlueEmptyCooldownFrames =
                    runtimeOptions.lightGlueEmptyCooldownFrames;
                std::cerr << "[lightglue_trt] loaded engine=" << lightGluePath.string()
                          << " points=" << lightGluePointCount
                          << " fixed_points=" << (fixedPointCount > 0 ? "Y" : "N")
                          << " min_score=" << lightGlueMinScore
                          << " max_y_diff_px=" << lightGlueMaxYDiffPx
                          << " min_disparity_px=" << lightGlueMinDisparityPx
                          << " empty_disable_threshold="
                          << lightGlueEmptyDisableThreshold
                          << " low_yield_disable_threshold="
                          << lightGlueLowYieldDisableThreshold
                          << " low_yield_min_pairs=" << lightGlueLowYieldMinPairs
                          << " empty_cooldown_frames=" << lightGlueEmptyCooldownFrames
                          << "\n";
            } else {
                std::cerr << "[lightglue_trt] warning: failed to load engine="
                          << lightGluePath.string() << " err=" << lgErr
                          << "; sp_descriptor=primary\n";
            }
        } else {
            std::cerr << "[lightglue_trt] engine not found; sp_descriptor=primary\n";
        }
        return true;
    }

    int maxPointsForLightGlue() const
    {
        return LoadSuperPointTensorRtRuntimeOptions().lightGluePoints;
    }

    bool PopulateOutputFromTensors(
        const TensorBlob &detectorBlob, const TensorBlob &descriptorBlob,
        int tensorBatch, const cv::Mat &sourceImage, int targetHeight,
        int targetWidth, int maxPoints, int descriptorLimit,
        SuperPointFeatureSet &output, SuperPointPostScratch &scratch,
        SuperPointPostStats *postStats, std::string *err)
    {
        output = SuperPointFeatureSet{};
        if (detectorBlob.dims.size() < 4 || descriptorBlob.dims.size() < 4 ||
            detectorBlob.Dim(0) <= tensorBatch ||
            descriptorBlob.Dim(0) <= tensorBatch || detectorBlob.Dim(1) < 65 ||
            descriptorBlob.Dim(1) != kSuperPointDescriptorDim) {
            if (err != nullptr) {
                *err = "TensorRT SuperPoint outputs have unexpected shapes";
            }
            return false;
        }
        const int heatmapWidth = detectorBlob.Dim(3) * kSuperPointCellSize;
        const int heatmapHeight = detectorBlob.Dim(2) * kSuperPointCellSize;
        if (heatmapWidth > targetWidth || heatmapHeight > targetHeight) {
            if (err != nullptr) {
                *err = "TensorRT SuperPoint detector output size does not match input";
            }
            return false;
        }

        const double ratioH = static_cast<double>(sourceImage.rows) /
                              static_cast<double>(targetHeight);
        const double ratioW = static_cast<double>(sourceImage.cols) /
                              static_cast<double>(targetWidth);
        SuperPointPostStats imagePostStats;
        if (EnvFlag("SMART_DRONE_SUPERPOINT_FAST_NMS", false)) {
            ExtractCandidatesFastNms(detectorBlob, tensorBatch, heatmapWidth,
                                     heatmapHeight, maxPoints, scratch,
                                     &imagePostStats);
        } else {
            ExtractCandidates(detectorBlob, tensorBatch, heatmapWidth, heatmapHeight,
                              maxPoints, scratch, &imagePostStats);
        }
        const std::vector<Candidate> &candidates = scratch.candidates;
        output.keypoints.reserve(candidates.size());
        const int descriptorCount =
            std::min(static_cast<int>(candidates.size()),
                     std::clamp(descriptorLimit, 0, maxPoints));
        if (descriptorCount > 0) {
            output.descriptors =
                cv::Mat(descriptorCount, kSuperPointDescriptorDim, CV_32F);
        }
        const auto descriptorStartTp = std::chrono::steady_clock::now();
        const bool useDescriptorHwc =
            EnvFlag("SMART_DRONE_SUPERPOINT_DESCRIPTOR_HWC", false);
        const bool useDescriptorNearest =
            EnvFlag("SMART_DRONE_SUPERPOINT_DESCRIPTOR_NEAREST", false);
        if (useDescriptorHwc && descriptorCount > 0) {
            BuildDescriptorGridHwc(descriptorBlob, tensorBatch,
                                   scratch.descriptorHwc);
        }
        for (size_t i = 0; i < candidates.size(); ++i) {
            const Candidate &candidate = candidates[i];
            output.keypoints.emplace_back(static_cast<float>(candidate.x * ratioW),
                                          static_cast<float>(candidate.y * ratioH));
            if (static_cast<int>(i) >= descriptorCount) {
                continue;
            }
            float *descriptor = output.descriptors.ptr<float>(static_cast<int>(i));
            const float sampleX =
                (static_cast<float>(candidate.x) - kSuperPointCellSize / 2.0f +
                 0.5f) /
                (static_cast<float>(descriptorBlob.Dim(3) * kSuperPointCellSize) -
                 kSuperPointCellSize / 2.0f - 0.5f) *
                static_cast<float>(descriptorBlob.Dim(3) - 1);
            const float sampleY =
                (static_cast<float>(candidate.y) - kSuperPointCellSize / 2.0f +
                 0.5f) /
                (static_cast<float>(descriptorBlob.Dim(2) * kSuperPointCellSize) -
                 kSuperPointCellSize / 2.0f - 0.5f) *
                static_cast<float>(descriptorBlob.Dim(2) - 1);
            if (useDescriptorNearest) {
                SampleDescriptorNearest(descriptorBlob, tensorBatch, sampleX, sampleY,
                                        descriptor);
            } else if (useDescriptorHwc) {
                SampleDescriptorBilinearHwc(
                    scratch.descriptorHwc, descriptorBlob.Dim(2), descriptorBlob.Dim(3),
                    sampleX, sampleY, descriptor);
            } else {
                SampleDescriptorBilinear(descriptorBlob, tensorBatch, sampleX, sampleY,
                                         descriptor);
            }
        }
        const auto descriptorEndTp = std::chrono::steady_clock::now();
        imagePostStats.descriptorMs +=
            DurationMs(descriptorStartTp, descriptorEndTp);
        imagePostStats.descriptorCount += descriptorCount;
        if (postStats != nullptr) {
            postStats->Add(imagePostStats);
        }
        return true;
    }

    bool DetectAndComputeBatch(const std::vector<cv::Mat> &grayImages,
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

    bool MatchWithLightGlue(const SuperPointFeatureSet &leftRaw,
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
};
