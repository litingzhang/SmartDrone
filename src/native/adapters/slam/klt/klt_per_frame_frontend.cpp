#include "adapters/slam/klt/klt_per_frame_frontend.h"

#include "adapters/slam/klt/klt_mode_utils.h"
#include "adapters/slam/klt/klt_vpi_acceleration.h"
#include "adapters/slam/engine/slam_env.h"
#include "adapters/slam/engine/slam_image_utils.h"

#include <algorithm>
#include <chrono>
#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace SmartDrone::Adapters::Slam {
namespace {

struct KltPerFrameAccelerationRequest {
    bool requestVpi{false};
    bool requestVpiRemap{false};
    bool requestVpiDisparity{false};
    bool requestVpiLk{false};
};

struct KltPerFrameTrackingData {
    cv::Mat disparity;
    std::vector<cv::Point2f> previousPoints;
    std::vector<cv::Point2f> currentPoints;
    std::vector<uint8_t> status;
    std::vector<cv::Point2f> backwardPoints;
    std::vector<uint8_t> backwardStatus;
    bool useForwardBackwardCheck{false};
};

KltPerFrameAccelerationRequest
ResolveKltPerFrameAccelerationRequest(const SlamModeSharedState &state)
{
    KltPerFrameAccelerationRequest request;
    request.requestVpi = state.m_lkPerFrameAcceleration == "auto" ||
                         state.m_lkPerFrameAcceleration == "vpi" ||
                         state.m_lkPerFrameAcceleration == "vpi-cuda" ||
                         state.m_lkPerFrameAcceleration == "vpi_cuda" ||
                         state.m_lkPerFrameAcceleration == "gpu";
    request.requestVpiRemap =
        request.requestVpi && EnvFlagEnabled("SMART_DRONE_VPI_REMAP", true);
    request.requestVpiDisparity =
        request.requestVpi && EnvFlagEnabled("SMART_DRONE_VPI_DISPARITY", true);
    request.requestVpiLk =
        request.requestVpi && EnvFlagEnabled("SMART_DRONE_VPI_LK", true);
    return request;
}

void StoreKltPerFramePreviousRectifiedIfNeeded(
    SlamModeSharedState &state, const KltPerFrameFrontendResult &frontend)
{
    if (frontend.usedVpiRemap) {
        StoreVpiPreviousRectified(state.m_lkPerFrameVpi);
    }
}

bool PrepareKltPerFrameInput(SlamModeSharedState &state, const cv::Mat &leftRaw,
                             const cv::Mat &rightRaw,
                             KltPerFrameFrontendResult &result,
                             std::chrono::steady_clock::time_point prepareStart)
{
    cv::Mat leftGray = EnsureGray8(leftRaw);
    cv::Mat rightGray = EnsureGray8(rightRaw);
    if (leftGray.empty() || rightGray.empty() || !state.m_lkCalibrationLoaded) {
        result.inputPrepareMs = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - prepareStart)
                                    .count();
        return false;
    }
    result.leftRect = std::move(leftGray);
    result.rightRect = std::move(rightGray);
    return true;
}

void RectifyKltPerFrameInput(SlamModeSharedState &state,
                             const KltPerFrameAccelerationRequest &request,
                             KltPerFrameFrontendResult &result)
{
    state.EnsureStereoRectifier(result.leftRect.size());
    cv::Mat leftRect = result.leftRect;
    cv::Mat rightRect = result.rightRect;
    if (request.requestVpiRemap && !state.m_lkMap1x.empty() &&
        !state.m_lkMap2x.empty()) {
        result.usedVpiRemap = VpiRemapCurrentStereo(
            result.leftRect, result.rightRect, leftRect, rightRect,
            state.m_lkPerFrameVpi, state.m_lkMap1x, state.m_lkMap1y,
            state.m_lkMap2x, state.m_lkMap2y, state.m_lkPerFrameAccelLogged);
    }
    if (!result.usedVpiRemap && !state.m_lkMap1x.empty() &&
        !state.m_lkMap2x.empty()) {
        cv::remap(result.leftRect, leftRect, state.m_lkMap1x, state.m_lkMap1y,
                  cv::INTER_LINEAR);
        cv::remap(result.rightRect, rightRect, state.m_lkMap2x, state.m_lkMap2y,
                  cv::INTER_LINEAR);
    }
    result.leftRect = std::move(leftRect);
    result.rightRect = std::move(rightRect);
}

void ConfigureKltPerFrameReference(const SlamModeSharedState &state,
                                   const KltPerFrameAccelerationRequest &request,
                                   KltPerFrameFrontendResult &result)
{
    result.preferAcceleratedPnpDefaults = request.requestVpi;
    result.useKeyframeReference =
        EnvFlagEnabled("SMART_DRONE_LK_PER_FRAME_KEYFRAME", false);
    result.keyframeInterval =
        std::max(1, EnvIntValue("SMART_DRONE_LK_PER_FRAME_KEYFRAME_INTERVAL", 6));
    (void)state;
}

bool ComputeKltPerFrameDisparity(
    SlamModeSharedState &state, const KltPerFrameAccelerationRequest &request,
    const KltPerFrameFrontendResult &result, cv::Mat &disparity)
{
    bool usedVpiDisparity = false;
    if (result.usedVpiRemap && request.requestVpiDisparity &&
        HasVpiPreviousRectified(state.m_lkPerFrameVpi)) {
        usedVpiDisparity = ComputeVpiCudaPreviousRectifiedDisparity(
            state.m_lkPrevLeft.size(), disparity, state.m_lkPerFrameVpi);
    }
    if (!usedVpiDisparity && request.requestVpiDisparity) {
        usedVpiDisparity = ComputeVpiCudaDisparity(
            state.m_lkPrevLeft, state.m_lkPrevRight, disparity,
            state.m_lkPerFrameVpi, state.m_lkPerFrameAccelLogged);
    }
    return usedVpiDisparity;
}

void ComputeCpuKltPerFrameDisparity(SlamModeSharedState &state,
                                    const KltPerFrameFrontendResult &result,
                                    cv::Mat &disparity)
{
    if (!state.m_lkPerFrameAccelLogged &&
        state.m_lkPerFrameAcceleration == "cpu") {
        std::cerr << "[lk_per_frame_accel] backend=cpu_sgbm\n";
        state.m_lkPerFrameAccelLogged = true;
    }
    if (!state.m_lkPerFrameSgbm) {
        const int numDisparities =
            std::max(16, ((result.leftRect.cols / 8 + 15) / 16) * 16);
        state.m_lkPerFrameSgbm = cv::StereoSGBM::create(
            0, numDisparities, 5, 8 * 5 * 5, 32 * 5 * 5, 1, 31, 8, 60, 2,
            cv::StereoSGBM::MODE_SGBM_3WAY);
    }
    cv::Mat disparity16;
    state.m_lkPerFrameSgbm->compute(state.m_lkPrevLeft, state.m_lkPrevRight,
                                    disparity16);
    disparity16.convertTo(disparity, CV_32F, 1.0 / 16.0);
}

std::vector<cv::Point2f> SelectKltPerFramePreviousPoints(
    const SlamModeSharedState &state)
{
    std::vector<cv::Point2f> rawPreviousPoints;
    cv::goodFeaturesToTrack(state.m_lkPrevLeft, rawPreviousPoints,
                            kLkGfttPerFrameMaxCorners, kLkGfttQualityLevel,
                            kLkGfttMinDistancePx, cv::Mat(), kLkGfttBlockSize,
                            false, kLkGfttHarrisK);
    return SelectGfttPointsGridBalanced(
        rawPreviousPoints, state.m_lkPrevLeft.size(), kLkGfttPerFrameMaxCorners,
        kLkGfttPerFrameMaxCornersPerCell);
}

bool ComputeKltPerFrameFlow(
    SlamModeSharedState &state, const KltPerFrameAccelerationRequest &request,
    const KltPerFrameFrontendResult &result, KltPerFrameTrackingData &tracking)
{
    bool usedVpiLk = false;
    if (result.usedVpiRemap && request.requestVpiLk &&
        !tracking.previousPoints.empty()) {
        usedVpiLk = ComputeVpiCudaCurrentPyrLk(
            state.m_lkPrevLeft, tracking.previousPoints, tracking.currentPoints,
            tracking.status, state.m_lkPerFrameVpi);
    }
    if (!usedVpiLk && !tracking.previousPoints.empty()) {
        std::vector<float> errors;
        cv::calcOpticalFlowPyrLK(
            state.m_lkPrevLeft, result.leftRect, tracking.previousPoints,
            tracking.currentPoints, tracking.status, errors, cv::Size(21, 21), 3);
    }
    return usedVpiLk;
}

void ComputeKltPerFrameBackwardFlow(const KltPerFrameFrontendResult &result,
                                    const SlamModeSharedState &state,
                                    KltPerFrameTrackingData &tracking)
{
    tracking.useForwardBackwardCheck =
        EnvFlagEnabled("SMART_DRONE_LK_PER_FRAME_FB_CHECK", false);
    if (!tracking.useForwardBackwardCheck || tracking.currentPoints.empty()) {
        return;
    }
    std::vector<float> backwardErrors;
    cv::calcOpticalFlowPyrLK(result.leftRect, state.m_lkPrevLeft,
                             tracking.currentPoints, tracking.backwardPoints,
                             tracking.backwardStatus, backwardErrors,
                             cv::Size(21, 21), 3);
}

void BuildKltPerFrameObservations(SlamModeSharedState &state,
                                  KltPerFrameFrontendResult &result,
                                  KltPerFrameTrackingData &tracking)
{
    KltPerFramePnpObservationBuilderOptions observationOptions;
    observationOptions.disparity = &tracking.disparity;
    observationOptions.previousImageSize = state.m_lkPrevLeft.size();
    observationOptions.currentImageSize = result.leftRect.size();
    observationOptions.previousPoints = &tracking.previousPoints;
    observationOptions.currentPoints = &tracking.currentPoints;
    observationOptions.status = &tracking.status;
    observationOptions.backwardPoints = &tracking.backwardPoints;
    observationOptions.backwardStatus = &tracking.backwardStatus;
    observationOptions.useForwardBackwardCheck = tracking.useForwardBackwardCheck;
    observationOptions.useDepthBalancedSelection =
        EnvFlagEnabled("SMART_DRONE_LK_PER_FRAME_DEPTH_BALANCE", true);
    observationOptions.fx = state.m_lkFx;
    observationOptions.fy = state.m_lkFy;
    observationOptions.cx = state.m_lkCx;
    observationOptions.cy = state.m_lkCy;
    observationOptions.baseline = state.m_lkBaseline;
    result.observations =
        state.VisualPnpObservationBuilder().BuildPerFrameObservations(
            observationOptions);
    result.currentLeftPoints = std::move(tracking.currentPoints);
}

double MsSince(std::chrono::steady_clock::time_point start,
               std::chrono::steady_clock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

} // namespace

KltPerFrameFrontendResult RunKltPerFrameFrontend(SlamModeSharedState &state,
                                                 const cv::Mat &leftRaw,
                                                 const cv::Mat &rightRaw)
{
    KltPerFrameFrontendResult result;
    const auto prepareStart = std::chrono::steady_clock::now();
    if (!PrepareKltPerFrameInput(state, leftRaw, rightRaw, result, prepareStart)) {
        return result;
    }

    const auto rectifyStart = std::chrono::steady_clock::now();
    const KltPerFrameAccelerationRequest request =
        ResolveKltPerFrameAccelerationRequest(state);
    ConfigureKltPerFrameReference(state, request, result);
    RectifyKltPerFrameInput(state, request, result);
    result.valid = true;
    const auto rectifyEnd = std::chrono::steady_clock::now();
    result.rectifyMs = MsSince(rectifyStart, rectifyEnd);
    result.inputPrepareMs = MsSince(prepareStart, rectifyEnd);

    if (!state.m_lkHavePrev) {
        return result;
    }

    const auto disparityStart = std::chrono::steady_clock::now();
    KltPerFrameTrackingData tracking;
    if (!ComputeKltPerFrameDisparity(state, request, result,
                                     tracking.disparity)) {
        ComputeCpuKltPerFrameDisparity(state, result, tracking.disparity);
    }
    const auto disparityEnd = std::chrono::steady_clock::now();
    result.disparityMs = MsSince(disparityStart, disparityEnd);

    const auto gfttStart = std::chrono::steady_clock::now();
    tracking.previousPoints = SelectKltPerFramePreviousPoints(state);
    const auto gfttEnd = std::chrono::steady_clock::now();
    result.gfttMs = MsSince(gfttStart, gfttEnd);

    const auto flowStart = std::chrono::steady_clock::now();
    (void)ComputeKltPerFrameFlow(state, request, result, tracking);
    ComputeKltPerFrameBackwardFlow(result, state, tracking);
    const auto flowEnd = std::chrono::steady_clock::now();
    result.flowMs = MsSince(flowStart, flowEnd);

    const auto candidateStart = std::chrono::steady_clock::now();
    BuildKltPerFrameObservations(state, result, tracking);
    const auto candidateEnd = std::chrono::steady_clock::now();
    result.candidateMs = MsSince(candidateStart, candidateEnd);
    return result;
}

bool ShouldRefreshKltPerFrameReference(
    const SlamModeSharedState &state, const KltPerFrameFrontendResult &frontend,
    int inlierCount)
{
    return !frontend.useKeyframeReference ||
           (state.m_lkFrameCount %
            static_cast<uint32_t>(std::max(1, frontend.keyframeInterval))) == 0 ||
           inlierCount < std::max(kLkMinPnPInliers * 2, 24);
}

void UpdateKltPerFrameReferenceFrame(
    SlamModeSharedState &state, const KltPerFrameFrontendResult &frontend)
{
    state.m_lkPrevLeft = frontend.leftRect.clone();
    state.m_lkPrevRight = frontend.rightRect.clone();
    state.m_lkPerFrameReferenceTwc = state.m_lkTwc;
    StoreKltPerFramePreviousRectifiedIfNeeded(state, frontend);
}

} // namespace SmartDrone::Adapters::Slam
