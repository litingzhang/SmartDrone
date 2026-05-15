#include "adapters/slam/slam_mode_strategy.h"

#include <chrono>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "adapters/slam/slam_engine_access.h"
#include "adapters/slam/slam_mode_common.h"
#include "adapters/slam/slam_tracking_backend.h"

namespace smartdrone::adapters::slam {

namespace {

core::ports::SlamOutput MakeExternalFeatureFailureOutput(SlamEngineAdapter &engine, SlamModeSharedState &state,
                                                         const core::ports::SlamInputBatch &input)
{
    core::ports::SlamOutput out{};
    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    out.trackingState = ORB_SLAM3::Tracking::RECENTLY_LOST;
    if (ORB_SLAM3::System *system = SlamEngineAccess::System(engine)) {
        out.mapId = system->GetCurrentMapId();
        out.matchesInliers = system->GetMatchesInliers();
        out.trackedMapPointCount = static_cast<uint32_t>(system->GetTrackedMapPointCount());
        out.localMapPointCount = static_cast<uint32_t>(system->GetLocalMapPointCount());
        out.localMapPointHash = system->GetLocalMapPointHash();
        out.matchedMapPointHashBeforePoseOptimization = system->GetMatchedMapPointHashBeforePoseOptimization();
        out.trackedMapPointHash = system->GetTrackedMapPointHash();
    }
    state.CopyExternalFeatureStatsToOutput(out);
    if (EnvFlagEnabled("SMART_DRONE_REALTIME_POSE_CONTINUITY", true)) {
        SlamEngineAccess::MaintainRealtimePoseContinuity(engine, out.pose, out.poseValid, input.frameTimeSec,
                                                         out.trackingState);
        out.pose.valid = out.poseValid;
    }
    return out;
}

int ChooseLightGlueCadence(const SlamModeSharedState &state, ORB_SLAM3::System &system)
{
    const int baseEveryN = EnvIntValueClamped("SMART_DRONE_LIGHTGLUE_EVERY_N", 4, 1, 120);
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_ADAPTIVE_CADENCE", false)) {
        return baseEveryN;
    }

    const int stableEveryN =
        EnvIntValueClamped("SMART_DRONE_SP_LG_STABLE_LIGHTGLUE_EVERY_N", std::min(baseEveryN + 1, 120), baseEveryN, 120);
    if (stableEveryN <= baseEveryN) {
        return baseEveryN;
    }

    const int trackingState = system.GetTrackingState();
    const int trackedMapPoints = system.GetTrackedMapPointCount();
    const int okStreakMin = EnvIntValueClamped("SMART_DRONE_SP_LG_STABLE_OK_STREAK", 120, 1, 100000);
    const int trackedMapPointMin = EnvIntValueClamped("SMART_DRONE_SP_LG_STABLE_TRACKED_MPS", 96, 1, 100000);
    if (trackingState == ORB_SLAM3::Tracking::OK && state.m_superPointLightGlueOkStreak >= okStreakMin &&
        trackedMapPoints >= trackedMapPointMin) {
        return stableEveryN;
    }
    return baseEveryN;
}

void UpdateLightGlueCadenceState(SlamModeSharedState &state, const core::ports::SlamOutput &out)
{
    const int trackedMapPointMin = EnvIntValueClamped("SMART_DRONE_SP_LG_STABLE_TRACKED_MPS", 96, 1, 100000);
    const int trustFrontendOkStreak =
        EnvIntValueClamped("SMART_DRONE_SP_LG_TRUST_FRONTEND_PAIRS_OK_STREAK", 0, 0, 100000);
    const int bootstrapTrustHoldTrackedMapMin =
        EnvIntValueClamped("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_HOLD_TRACKED_MPS",
                           trackedMapPointMin, 1, 100000);
    const int trackedMapPoints = static_cast<int>(out.trackedMapPointCount);
    const bool bootstrapTrustAlreadyMature =
        trustFrontendOkStreak > 0 && state.m_superPointLightGlueOkStreak >= trustFrontendOkStreak;
    const bool holdMatureBootstrapTrust =
        bootstrapTrustAlreadyMature && trackedMapPoints >= bootstrapTrustHoldTrackedMapMin;
    if (out.trackingState == ORB_SLAM3::Tracking::OK &&
        (trackedMapPoints >= trackedMapPointMin || holdMatureBootstrapTrust)) {
        ++state.m_superPointLightGlueOkStreak;
    } else {
        state.m_superPointLightGlueOkStreak = 0;
    }
}

void UpdateBootstrapTrustState(SlamModeSharedState &state, bool usedBootstrapTrust, int trustFrontendOkStreak)
{
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_ONCE", false)) {
        return;
    }
    if (trustFrontendOkStreak <= 0) {
        state.m_superPointLightGlueBootstrapTrustClosed = true;
        return;
    }
    if (usedBootstrapTrust) {
        ++state.m_superPointLightGlueBootstrapTrustFrames;
    }
    if (state.m_superPointLightGlueOkStreak >= trustFrontendOkStreak ||
        state.m_superPointLightGlueBootstrapTrustFrames >= trustFrontendOkStreak) {
        state.m_superPointLightGlueBootstrapTrustClosed = true;
    }
}

bool PreviousFrameWasWeak(const SlamModeSharedState &state)
{
    const int minInliers = EnvIntValueClamped("SMART_DRONE_SP_LG_WEAK_FRAME_MIN_INLIERS", 90, 1, 100000);
    const int minTrackedMap = EnvIntValueClamped("SMART_DRONE_SP_LG_WEAK_FRAME_MIN_TRACKED_MAP", 120, 1, 100000);
    return state.m_lastSlamMatchesInliers > 0 &&
           (state.m_lastSlamMatchesInliers < minInliers || state.m_lastSlamTrackedMapPoints < minTrackedMap);
}

void LimitStereoPairsForWeakTracking(std::vector<cv::Point2f> &leftPoints, std::vector<cv::Point2f> &rightPoints,
                                     const SlamModeSharedState &state, bool initializing, bool recovering)
{
    if (initializing || recovering || !EnvFlagEnabled("SMART_DRONE_SP_LG_WEAK_FRAME_PAIR_LIMIT", false) ||
        !PreviousFrameWasWeak(state)) {
        return;
    }

    const size_t maxPairs = EnvSizeValueClamped("SMART_DRONE_SP_LG_WEAK_FRAME_MAX_PAIRS", 192, 48, 1200);
    LimitStereoPairsInPlace(leftPoints, rightPoints, maxPairs);
}

size_t RefineSelectedRightPointsByZncc(const cv::Mat &leftPrepared, const cv::Mat &rightPrepared,
                                       std::vector<cv::Point2f> &leftPoints,
                                       std::vector<cv::Point2f> &rightPoints)
{
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_REFINE_RIGHT_ZNCC", false) || leftPrepared.empty() ||
        rightPrepared.empty() || leftPoints.empty() || leftPoints.size() != rightPoints.size()) {
        return 0;
    }

    cv::Mat left32f;
    cv::Mat right32f;
    leftPrepared.convertTo(left32f, CV_32F);
    rightPrepared.convertTo(right32f, CV_32F);

    const float maxShiftPx =
        EnvFloatValueClamped("SMART_DRONE_SP_LG_REFINE_RIGHT_ZNCC_MAX_SHIFT_PX", 4.0f, 0.25f, 16.0f);
    size_t refinedCount = 0;
    for (size_t i = 0; i < leftPoints.size(); ++i) {
        cv::Point2f refinedRight = rightPoints[i];
        float zncc = -1.0f;
        if (!RefineRightPointByStereoZncc(left32f, leftPoints[i], right32f, rightPoints[i], refinedRight, zncc)) {
            continue;
        }
        if (std::fabs(refinedRight.x - rightPoints[i].x) > maxShiftPx ||
            std::fabs(refinedRight.y - rightPoints[i].y) > 1.0e-3f) {
            continue;
        }
        if (!IsStereoPairGeometricallyValid(leftPoints[i], refinedRight)) {
            continue;
        }
        rightPoints[i] = refinedRight;
        ++refinedCount;
    }
    return refinedCount;
}

std::vector<StereoMatchPair> SelectInitializationTrustedPairs(const SuperPointFeatureSet &leftFeatures,
                                                              const SuperPointFeatureSet &rightFeatures,
                                                              const cv::Mat &leftPrepared,
                                                              const cv::Mat &rightPrepared)
{
    std::vector<StereoMatchPair> matches =
        BuildAlignedStereoPairs(leftFeatures, rightFeatures, leftPrepared, rightPrepared);
    if (matches.empty()) {
        return matches;
    }

    const float closeDisparity =
        EnvFloatValue("SMART_DRONE_SP_LG_INIT_CLOSE_DISPARITY", 4.0f);
    std::stable_sort(matches.begin(), matches.end(),
                     [closeDisparity](const StereoMatchPair &lhs, const StereoMatchPair &rhs) {
                         const bool lhsClose = lhs.disparity >= closeDisparity;
                         const bool rhsClose = rhs.disparity >= closeDisparity;
                         if (lhsClose != rhsClose) {
                             return lhsClose;
                         }
                         if (std::abs(lhs.quality - rhs.quality) > 1.0e-6f) {
                             return lhs.quality > rhs.quality;
                         }
                         if (std::abs(lhs.zncc - rhs.zncc) > 1.0e-6f) {
                             return lhs.zncc > rhs.zncc;
                         }
                         return lhs.disparity > rhs.disparity;
                     });

    const size_t maxPairs = EnvSizeValueClamped("SMART_DRONE_SP_LG_INIT_TRUST_MAX_PAIRS", 256, 48, 1200);
    if (matches.size() > maxPairs) {
        matches.resize(maxPairs);
    }
    return matches;
}

size_t TemporalCarryBudget(const SlamModeSharedState &state)
{
    const size_t stableBudget = EnvSizeValueClamped("SMART_DRONE_SP_LG_TEMPORAL_CARRY_BUDGET",
                                                    kTemporalCarryMinBudget, 0, kTemporalCarryMaxBudget);
    if (!PreviousFrameWasWeak(state)) {
        return stableBudget;
    }
    return EnvSizeValueClamped("SMART_DRONE_SP_LG_WEAK_TEMPORAL_CARRY_BUDGET",
                               kWeakTrackingTemporalCarryBudget, 0, kTemporalCarryMaxBudget);
}

size_t AppendTemporalCarryPairs(SlamModeSharedState &state, const cv::Mat &leftPrepared, const cv::Mat &rightPrepared,
                                std::vector<cv::Point2f> &matchedLeftPoints,
                                std::vector<cv::Point2f> &matchedRightPoints,
                                bool initializing, bool recovering)
{
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_TEMPORAL_CARRY", false) || initializing || recovering ||
        !state.m_spLgHavePrevStereo || state.m_spLgPrevLeft.empty() || state.m_spLgPrevRight.empty() ||
        state.m_spLgPrevLeftPoints.empty() || state.m_spLgPrevLeftPoints.size() != state.m_spLgPrevRightPoints.size()) {
        return 0;
    }

    const size_t budget = TemporalCarryBudget(state);
    if (budget == 0) {
        return 0;
    }

    std::vector<TemporalStereoPair> trackedPairs = TrackStereoPairsTemporally(
        state.m_spLgPrevLeft, state.m_spLgPrevRight, state.m_spLgPrevLeftPoints, state.m_spLgPrevRightPoints,
        leftPrepared, rightPrepared);
    trackedPairs = FilterTemporalPairsWithMotionRansac(trackedPairs, state.m_spLgPrevLeftPoints);
    trackedPairs = LimitTemporalPairs(trackedPairs, budget);

    std::vector<cv::Point2f> carryLeftPoints;
    std::vector<cv::Point2f> carryRightPoints;
    carryLeftPoints.reserve(trackedPairs.size());
    carryRightPoints.reserve(trackedPairs.size());
    for (const TemporalStereoPair &pair : trackedPairs) {
        if (IsStereoPairNearExisting(pair.leftPt, pair.rightPt, matchedLeftPoints, matchedRightPoints)) {
            continue;
        }
        carryLeftPoints.push_back(pair.leftPt);
        carryRightPoints.push_back(pair.rightPt);
    }
    if (carryLeftPoints.empty()) {
        return 0;
    }

    const size_t before = matchedLeftPoints.size();
    size_t inserted = 0;
    std::vector<cv::Point2f> mergedLeftPoints;
    std::vector<cv::Point2f> mergedRightPoints;
    const size_t maxMergedPairs = EnvSizeValueClamped("SMART_DRONE_SP_LG_TEMPORAL_CARRY_MAX_PAIRS",
                                                      std::max(before, kTemporalMaxInjectedPairs),
                                                      kTemporalMaxInjectedPairs, 1200);
    mergedLeftPoints.reserve(std::min(maxMergedPairs, before + carryLeftPoints.size()));
    mergedRightPoints.reserve(std::min(maxMergedPairs, before + carryRightPoints.size()));
    auto appendUnique = [&](const std::vector<cv::Point2f> &sourceLeft, const std::vector<cv::Point2f> &sourceRight,
                            bool countInserted) {
        for (size_t i = 0; i < sourceLeft.size() && i < sourceRight.size(); ++i) {
            if (mergedLeftPoints.size() >= maxMergedPairs) {
                return;
            }
            if (IsStereoPairNearExisting(sourceLeft[i], sourceRight[i], mergedLeftPoints, mergedRightPoints)) {
                continue;
            }
            mergedLeftPoints.push_back(sourceLeft[i]);
            mergedRightPoints.push_back(sourceRight[i]);
            if (countInserted) {
                ++inserted;
            }
        }
    };
    appendUnique(carryLeftPoints, carryRightPoints, true);
    appendUnique(matchedLeftPoints, matchedRightPoints, false);
    matchedLeftPoints = std::move(mergedLeftPoints);
    matchedRightPoints = std::move(mergedRightPoints);
    return inserted;
}

void StoreTemporalCarrySource(SlamModeSharedState &state, const cv::Mat &leftPrepared, const cv::Mat &rightPrepared,
                              const ORB_SLAM3::ExternalStereoFrameData &externalData)
{
    if (!EnvFlagEnabled("SMART_DRONE_SP_LG_TEMPORAL_CARRY", false)) {
        return;
    }

    const size_t pairCount = std::min(externalData.leftKeypoints.size(), externalData.leftToRightMatch.size());
    std::vector<cv::Point2f> leftPoints;
    std::vector<cv::Point2f> rightPoints;
    leftPoints.reserve(pairCount);
    rightPoints.reserve(pairCount);
    for (size_t leftIndex = 0; leftIndex < pairCount; ++leftIndex) {
        const int rightIndex = externalData.leftToRightMatch[leftIndex];
        if (rightIndex < 0 || static_cast<size_t>(rightIndex) >= externalData.rightKeypoints.size()) {
            continue;
        }
        leftPoints.push_back(externalData.leftKeypoints[leftIndex].pt);
        rightPoints.push_back(externalData.rightKeypoints[static_cast<size_t>(rightIndex)].pt);
    }

    if (leftPoints.empty() || leftPoints.size() != rightPoints.size()) {
        state.m_spLgHavePrevStereo = false;
        state.m_spLgPrevLeftPoints.clear();
        state.m_spLgPrevRightPoints.clear();
        return;
    }

    state.m_spLgPrevLeft = leftPrepared.clone();
    state.m_spLgPrevRight = rightPrepared.clone();
    state.m_spLgPrevLeftPoints = std::move(leftPoints);
    state.m_spLgPrevRightPoints = std::move(rightPoints);
    state.m_spLgHavePrevStereo = true;
}

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

uint64_t HashExternalStereoData(const ORB_SLAM3::ExternalStereoFrameData &data)
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

} // namespace

FeatureFrontend SuperPointLightGlueModeStrategy::Frontend() const { return FeatureFrontend::SuperPointLightGlue; }

core::ports::SlamOutput SuperPointLightGlueModeStrategy::Process(SlamEngineAdapter &engine,
                                                                 const core::ports::SlamInputBatch &input,
                                                                 bool extractFeatures, bool extractPointCloud)
{
    ORB_SLAM3::System *system = SlamEngineAccess::System(engine);
    if (system == nullptr) {
        return {};
    }

    SlamModeSharedState &state = SlamEngineAccess::ModeState(engine);
    const bool monoMode = SlamEngineAccess::InputMode(engine) != SlamInputMode::Stereo;
    if (monoMode || state.m_externalFeatureFrontendClient == nullptr || !state.m_externalFeatureFrontendClient->Running()) {
        return MakeExternalFeatureFailureOutput(engine, state, input);
    }

    const int lightGlueEveryN = ChooseLightGlueCadence(state, *system);
    state.m_externalFeatureFrontendClient->SetLightGlueEveryNOverride(lightGlueEveryN);
    state.m_superPointLightGlueLastEveryN = lightGlueEveryN;
    state.ResetExternalFeatureStats();

    const auto externalStartTp = std::chrono::steady_clock::now();
    const auto prepareStartTp = externalStartTp;
    cv::Mat leftPrepared = EnsureGray8(input.stereo.left.gray);
    cv::Mat rightPrepared = EnsureGray8(input.stereo.right.gray);
    if (leftPrepared.empty() || rightPrepared.empty()) {
        return MakeExternalFeatureFailureOutput(engine, state, input);
    }

    if (EnvFlagEnabled("SMART_DRONE_SP_LG_USE_ORB_PREPARED_IMAGES", false)) {
        if (!system->PrepareStereoImagesForTracking(input.stereo.left.gray, input.stereo.right.gray, leftPrepared,
                                                    rightPrepared)) {
            return MakeExternalFeatureFailureOutput(engine, state, input);
        }
        leftPrepared = EnsureGray8(leftPrepared);
        rightPrepared = EnsureGray8(rightPrepared);
        if (leftPrepared.empty() || rightPrepared.empty()) {
            return MakeExternalFeatureFailureOutput(engine, state, input);
        }
    } else if (state.m_lkCalibrationLoaded) {
        state.EnsureStereoRectifier(leftPrepared.size());
        if (!state.m_lkMap1x.empty() && !state.m_lkMap2x.empty()) {
            cv::Mat leftRect;
            cv::Mat rightRect;
            cv::remap(leftPrepared, leftRect, state.m_lkMap1x, state.m_lkMap1y, cv::INTER_LINEAR);
            cv::remap(rightPrepared, rightRect, state.m_lkMap2x, state.m_lkMap2y, cv::INTER_LINEAR);
            leftPrepared = std::move(leftRect);
            rightPrepared = std::move(rightRect);
        }
    }

    float leftScaleX = 1.0f;
    float leftScaleY = 1.0f;
    float rightScaleX = 1.0f;
    float rightScaleY = 1.0f;
    const cv::Mat leftInput = BuildSuperPointInputImage(leftPrepared, state.m_externalFeatureInputMaxWidth,
                                                        state.m_externalFeatureInputMaxHeight, leftScaleX, leftScaleY);
    const cv::Mat rightInput = BuildSuperPointInputImage(rightPrepared, state.m_externalFeatureInputMaxWidth,
                                                         state.m_externalFeatureInputMaxHeight, rightScaleX, rightScaleY);
    const auto inputEndTp = std::chrono::steady_clock::now();

    SuperPointFeatureSet leftFeatures;
    SuperPointFeatureSet rightFeatures;
    std::string featureErr;
    const auto frontendStartTp = std::chrono::steady_clock::now();
    if (!state.m_externalFeatureFrontendClient->DetectAndComputeStereo(leftInput, rightInput, leftFeatures, rightFeatures,
                                                                       &featureErr)) {
        std::cerr << "[superpoint_lightglue] frontend_failed frame_id=" << input.frameId
                  << " err=" << featureErr << "\n";
        return MakeExternalFeatureFailureOutput(engine, state, input);
    }

    const auto frontendEndTp = std::chrono::steady_clock::now();
    const ExternalFeatureFrontendClient::Stats stats = state.m_externalFeatureFrontendClient->LastStats();
    state.m_lastSuperPointPrepareMs = stats.prepareMs;
    state.m_lastSuperPointInputMs = stats.inputMs;
    state.m_lastSuperPointForwardMs = stats.forwardMs;
    state.m_lastSuperPointFrontendMs = stats.totalMs;
    state.m_lastSuperPointImageCount = stats.imageCount;
    state.m_lastSuperPointPayloadBytes = stats.payloadBytes;
    state.m_lastSuperPointRawLeftCount = stats.rawLeftCount;
    state.m_lastSuperPointRawRightCount = stats.rawRightCount;

    RemapKeypointsToSource(leftFeatures.keypoints, leftScaleX, leftScaleY);
    RemapKeypointsToSource(rightFeatures.keypoints, rightScaleX, rightScaleY);

    ORB_SLAM3::Tracking *tracker = system->GetTracker();
    const auto matchStartTp = std::chrono::steady_clock::now();
    const size_t pairedFeatureCount = std::min(leftFeatures.keypoints.size(), rightFeatures.keypoints.size());
    const bool initializingExternalStereo =
        tracker != nullptr && (tracker->mState == ORB_SLAM3::Tracking::NO_IMAGES_YET ||
                               tracker->mState == ORB_SLAM3::Tracking::NOT_INITIALIZED);
    const bool recoveringExternalStereo =
        tracker != nullptr && (tracker->mState == ORB_SLAM3::Tracking::RECENTLY_LOST ||
                               tracker->mState == ORB_SLAM3::Tracking::LOST);
    const bool trustFrontendInitPairs =
        initializingExternalStereo &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_INIT_TRUST_FRONTEND_PAIRS", false);
    const bool trustFrontendRecoveryPairs =
        recoveringExternalStereo &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_RECOVERY_TRUST_FRONTEND_PAIRS", false);
    const int trustFrontendOkStreak =
        EnvIntValueClamped("SMART_DRONE_SP_LG_TRUST_FRONTEND_PAIRS_OK_STREAK", 0, 0, 100000);
    const bool trustFrontendBootstrapPairs =
        trustFrontendOkStreak > 0 &&
        (!EnvFlagEnabled("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_ONCE", false) ||
         !state.m_superPointLightGlueBootstrapTrustClosed) &&
        state.m_superPointLightGlueOkStreak < trustFrontendOkStreak &&
        (!EnvFlagEnabled("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_ONCE", false) ||
         state.m_superPointLightGlueBootstrapTrustFrames < trustFrontendOkStreak) &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_FRONTEND_PAIRS", false);
    const bool trustFrontendPairs =
        pairedFeatureCount > 0 &&
        (trustFrontendInitPairs || trustFrontendRecoveryPairs || trustFrontendBootstrapPairs);
    const bool initializationTrustedPairSelection =
        initializingExternalStereo &&
        trustFrontendPairs &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_INIT_TRUST_SELECT_CLOSE_PAIRS", false);
    std::vector<StereoMatchPair> initializationTrustedMatches;
    std::vector<StereoMatchPair> rawMatches;
    if (initializationTrustedPairSelection) {
        initializationTrustedMatches =
            SelectInitializationTrustedPairs(leftFeatures, rightFeatures, leftPrepared, rightPrepared);
        if (initializationTrustedMatches.empty()) {
            rawMatches = BuildAlignedStereoPairs(leftFeatures, rightFeatures, leftPrepared, rightPrepared);
        }
    } else if (!trustFrontendPairs) {
        rawMatches = BuildAlignedStereoPairs(leftFeatures, rightFeatures, leftPrepared, rightPrepared);
    }
    const bool initializationStereoBias =
        EnvFlagEnabled("SMART_DRONE_SP_LG_INIT_STEREO_BIAS", false) &&
        initializingExternalStereo;
    if (initializationStereoBias && !rawMatches.empty()) {
        const float closeDisparity =
            EnvFloatValue("SMART_DRONE_SP_LG_INIT_CLOSE_DISPARITY", 4.0f);
        std::stable_sort(rawMatches.begin(), rawMatches.end(),
                         [closeDisparity](const StereoMatchPair &lhs, const StereoMatchPair &rhs) {
                             const bool lhsClose = lhs.disparity > closeDisparity;
                             const bool rhsClose = rhs.disparity > closeDisparity;
                             if (lhsClose != rhsClose) {
                                 return lhsClose;
                             }
                             if (std::abs(lhs.quality - rhs.quality) > 1.0e-6f) {
                                 return lhs.quality > rhs.quality;
                             }
                             if (std::abs(lhs.zncc - rhs.zncc) > 1.0e-6f) {
                                 return lhs.zncc > rhs.zncc;
                             }
                             return lhs.disparity > rhs.disparity;
                         });
    }
    const std::vector<StereoMatchPair> matches = FilterStereoPairsByDisparityConsistency(rawMatches);
    std::vector<cv::Point2f> matchedLeftPoints;
    std::vector<cv::Point2f> matchedRightPoints;
    auto appendMatchedPairs = [&](const std::vector<StereoMatchPair> &sourceMatches) {
        matchedLeftPoints.clear();
        matchedRightPoints.clear();
        matchedLeftPoints.reserve(sourceMatches.size());
        matchedRightPoints.reserve(sourceMatches.size());
        for (const StereoMatchPair &match : sourceMatches) {
            if (match.leftIndex < 0 || match.rightIndex < 0 ||
                static_cast<size_t>(match.leftIndex) >= leftFeatures.keypoints.size() ||
                static_cast<size_t>(match.rightIndex) >= rightFeatures.keypoints.size()) {
                continue;
            }
            matchedLeftPoints.push_back(leftFeatures.keypoints[static_cast<size_t>(match.leftIndex)]);
            matchedRightPoints.push_back(rightFeatures.keypoints[static_cast<size_t>(match.rightIndex)]);
        }
    };
    if (initializationTrustedPairSelection && !initializationTrustedMatches.empty()) {
        appendMatchedPairs(initializationTrustedMatches);
    } else if (trustFrontendPairs) {
        matchedLeftPoints.reserve(pairedFeatureCount);
        matchedRightPoints.reserve(pairedFeatureCount);
        for (size_t i = 0; i < pairedFeatureCount; ++i) {
            matchedLeftPoints.push_back(leftFeatures.keypoints[i]);
            matchedRightPoints.push_back(rightFeatures.keypoints[i]);
        }
    } else if (initializationStereoBias && !rawMatches.empty()) {
        appendMatchedPairs(rawMatches);
    } else if (EnvFlagEnabled("SMART_DRONE_SP_LG_FILTERED_STEREO_INJECT", true) && !matches.empty()) {
        appendMatchedPairs(matches);
    } else {
        matchedLeftPoints.reserve(pairedFeatureCount);
        matchedRightPoints.reserve(pairedFeatureCount);
        for (size_t i = 0; i < pairedFeatureCount; ++i) {
            matchedLeftPoints.push_back(leftFeatures.keypoints[i]);
            matchedRightPoints.push_back(rightFeatures.keypoints[i]);
        }
    }
    LimitStereoPairsForWeakTracking(matchedLeftPoints, matchedRightPoints, state, initializingExternalStereo,
                                    recoveringExternalStereo);
    const size_t znccRefinedPairs =
        RefineSelectedRightPointsByZncc(leftPrepared, rightPrepared, matchedLeftPoints, matchedRightPoints);
    const size_t temporalCarryPairs =
        AppendTemporalCarryPairs(state, leftPrepared, rightPrepared, matchedLeftPoints, matchedRightPoints,
                                 initializingExternalStereo, recoveringExternalStereo);
    const double stereoPairMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - matchStartTp).count();
    state.m_lastSuperPointStereoMatchMs = stereoPairMs;
    state.m_lastSuperPointMatchedStereoCount = static_cast<int>(matchedLeftPoints.size());

    ORB_SLAM3::ExternalStereoFrameData externalData;
    const auto packStartTp = std::chrono::steady_clock::now();
    bool packedExternalData = false;
    if (EnvFlagEnabled("SMART_DRONE_SP_LG_NATIVE_DESCRIPTOR_INJECT", false)) {
        std::vector<StereoMatchPair> descriptorMatches;
        if (EnvFlagEnabled("SMART_DRONE_SP_LG_FILTERED_STEREO_INJECT", true) && !matches.empty()) {
            descriptorMatches = matches;
        } else {
            descriptorMatches.reserve(pairedFeatureCount);
            for (size_t i = 0; i < pairedFeatureCount; ++i) {
                descriptorMatches.push_back(StereoMatchPair{static_cast<int>(i), static_cast<int>(i),
                                                            1.0f, 1.0f, 0.0f, 1.0f});
            }
        }
        packedExternalData = BuildExternalStereoFromFeatureMatches(leftFeatures, rightFeatures, descriptorMatches,
                                                                   externalData);
    }
    const std::vector<StereoMatchPair> &depthMatches =
        EnvFlagEnabled("SMART_DRONE_SP_LG_DEPTH_DISPARITY_CONSISTENCY_FILTER", false) ? matches : rawMatches;
    if (!packedExternalData && EnvFlagEnabled("SMART_DRONE_SP_LG_ALL_LEFT_GEOMETRIC_DEPTH", false) &&
        !depthMatches.empty()) {
        packedExternalData = FinalizeStereoExternalFromPairsWithAllLeft(
            tracker != nullptr ? tracker->GetLeftORBExtractor() : nullptr,
            tracker != nullptr ? tracker->GetRightORBExtractor() : nullptr, leftPrepared, rightPrepared,
            matchedLeftPoints, depthMatches, leftFeatures, rightFeatures, externalData);
    }
    if (!packedExternalData &&
        !FinalizeStereoExternalFromPairs(tracker != nullptr ? tracker->GetLeftORBExtractor() : nullptr,
                                         tracker != nullptr ? tracker->GetRightORBExtractor() : nullptr, leftPrepared,
                                         rightPrepared, matchedLeftPoints, matchedRightPoints, externalData)) {
        appendMatchedPairs(matches);
        state.m_lastSuperPointMatchedStereoCount = static_cast<int>(matchedLeftPoints.size());
        if (!FinalizeStereoExternalFromPairs(tracker != nullptr ? tracker->GetLeftORBExtractor() : nullptr,
                                             tracker != nullptr ? tracker->GetRightORBExtractor() : nullptr,
                                             leftPrepared, rightPrepared, matchedLeftPoints, matchedRightPoints,
                                             externalData)) {
            return MakeExternalFeatureFailureOutput(engine, state, input);
        }
    }

    const auto packEndTp = std::chrono::steady_clock::now();
    double monoAugmentMs = 0.0;
    const bool initializedForMonoAugmentation =
        tracker != nullptr && tracker->mState != ORB_SLAM3::Tracking::NO_IMAGES_YET &&
        tracker->mState != ORB_SLAM3::Tracking::NOT_INITIALIZED;
    const int monoAugmentMinOkStreak =
        EnvIntValueClamped("SMART_DRONE_SP_LG_ORB_LEFT_AUGMENT_MIN_OK_STREAK", 0, 0, 100000);
    if (initializedForMonoAugmentation &&
        state.m_superPointLightGlueOkStreak >= monoAugmentMinOkStreak &&
        EnvFlagEnabled("SMART_DRONE_SP_LG_ORB_LEFT_AUGMENT", false)) {
        const auto augmentStartTp = std::chrono::steady_clock::now();
        const size_t maxLeftFeatures =
            EnvSizeValueClamped("SMART_DRONE_EXTERNAL_STEREO_MAX_LEFT_FEATURES", kExternalStereoMaxLeftFeatures,
                                kExternalStereoMaxLeftFeatures, kExternalStereoMaxLeftFeaturesLimit);
        AppendOrbLeftOnlyFeatures(tracker->GetLeftORBExtractor(), leftPrepared, externalData, maxLeftFeatures);
        monoAugmentMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - augmentStartTp).count();
    }

    state.m_lastSuperPointInjectedLeftCount = static_cast<int>(externalData.leftKeypoints.size());
    state.m_lastSuperPointInjectedRightCount = static_cast<int>(externalData.rightKeypoints.size());
    state.m_lastSuperPointExternalHash = HashExternalStereoData(externalData);
    if (EnvFlagEnabled("SMART_DRONE_SP_LG_INJECT_DFX", false)) {
        std::cerr << "[sp_lg_inject_dfx] frame_id=" << input.frameId
                  << " initializing=" << (initializingExternalStereo ? "Y" : "N")
                  << " recovering=" << (recoveringExternalStereo ? "Y" : "N")
                  << " trust_frontend_pairs=" << (trustFrontendPairs ? "Y" : "N")
                  << " init_trust_selected=" << (initializationTrustedPairSelection ? "Y" : "N")
                  << " bootstrap_trust=" << (trustFrontendBootstrapPairs ? "Y" : "N")
                  << " bootstrap_closed=" << (state.m_superPointLightGlueBootstrapTrustClosed ? "Y" : "N")
                  << " bootstrap_frames=" << state.m_superPointLightGlueBootstrapTrustFrames
                  << " ok_streak=" << state.m_superPointLightGlueOkStreak
                  << " prev_inliers=" << state.m_lastSlamMatchesInliers
                  << " prev_tracked=" << state.m_lastSlamTrackedMapPoints
                  << " frontend_pairs=" << pairedFeatureCount
                  << " init_trust_matches=" << initializationTrustedMatches.size()
                  << " raw_matches=" << rawMatches.size()
                  << " filtered_matches=" << matches.size()
                  << " selected_pairs=" << matchedLeftPoints.size()
                  << " zncc_refined=" << znccRefinedPairs
                  << " temporal_carry=" << temporalCarryPairs
                  << " injected=" << externalData.leftKeypoints.size()
                  << "/" << externalData.rightKeypoints.size()
                  << " hash=" << state.m_lastSuperPointExternalHash
                  << " scale=" << leftScaleX << "x" << leftScaleY
                  << "\n";
    }

    ExternalStereoTrackRequest request;
    request.enabled = true;
    request.recordTotalMs = true;
    request.totalStartTp = externalStartTp;
    request.leftPrepared = std::move(leftPrepared);
    request.rightPrepared = std::move(rightPrepared);
    request.externalData = std::move(externalData);
    request.inputPrepareMs = std::chrono::duration<double, std::milli>(inputEndTp - prepareStartTp).count();
    request.frontendMs = std::chrono::duration<double, std::milli>(frontendEndTp - frontendStartTp).count();
    request.stereoPairMs = stereoPairMs;
    request.externalPackMs = std::chrono::duration<double, std::milli>(packEndTp - packStartTp).count();
    request.monoAugmentMs = monoAugmentMs;
    request.externalHash = state.m_lastSuperPointExternalHash;
    request.leftFeaturePoints.reserve(request.externalData.leftKeypoints.size());
    request.rightFeaturePoints.reserve(request.externalData.rightKeypoints.size());
    for (const cv::KeyPoint &kp : request.externalData.leftKeypoints) {
        request.leftFeaturePoints.push_back(kp.pt);
    }
    for (const cv::KeyPoint &kp : request.externalData.rightKeypoints) {
        request.rightFeaturePoints.push_back(kp.pt);
    }
    StoreTemporalCarrySource(state, request.leftPrepared, request.rightPrepared, request.externalData);

    core::ports::SlamOutput out = RunSlamTrackingBackend(engine, input, extractFeatures, extractPointCloud, &request);
    UpdateLightGlueCadenceState(state, out);
    UpdateBootstrapTrustState(state, trustFrontendBootstrapPairs && trustFrontendPairs, trustFrontendOkStreak);
    state.m_lastSlamMatchesInliers = out.matchesInliers;
    state.m_lastSlamTrackedMapPoints = static_cast<int>(out.trackedMapPointCount);
    return out;
}

std::unique_ptr<SlamModeStrategy> CreateSuperPointLightGlueModeStrategy()
{
    return std::make_unique<SuperPointLightGlueModeStrategy>();
}

} // namespace smartdrone::adapters::slam
