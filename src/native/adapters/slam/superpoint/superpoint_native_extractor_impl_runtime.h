void SuperPointNativeExtractor::Impl::RecordLightGlueLowYield(int lightGlueLeftCount)
{
    if (lightGlueLeftCount >= lightGlueLowYieldMinPairs) {
        lightGlueLowYieldCount = 0;
        return;
    }
    ++lightGlueLowYieldCount;
    if (lightGlueLowYieldCount < lightGlueLowYieldDisableThreshold) {
        return;
    }
    lightGlueSkipRemaining = lightGlueEmptyCooldownFrames;
    lightGlueLowYieldCount = 0;
    std::cerr << "[lightglue_trt] low_yield_cooldown pairs="
              << lightGlueLeftCount << " frames="
              << lightGlueSkipRemaining << "\n";
}

void SuperPointNativeExtractor::Impl::RecordLightGlueEmptyOutput()
{
    ++lightGlueEmptyCount;
    ++lightGlueLowYieldCount;
    if (lightGlueEmptyCount < lightGlueEmptyDisableThreshold) {
        return;
    }
    lightGlueSkipRemaining = lightGlueEmptyCooldownFrames;
    lightGlueEmptyCount = 0;
    lightGlueLowYieldCount = 0;
    std::cerr << "[lightglue_trt] empty_output_cooldown frames="
              << lightGlueSkipRemaining << "\n";
}

bool SuperPointNativeExtractor::Impl::Load(const std::string &repoPath, const std::string &deviceText,
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

int SuperPointNativeExtractor::Impl::maxPointsForLightGlue() const
{
    return LoadSuperPointTensorRtRuntimeOptions().lightGluePoints;
}

bool SuperPointNativeExtractor::Impl::PopulateOutputFromTensors(
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
