#include "adapters/slam/klt_per_frame_frontend.h"

#include "adapters/slam/klt_mode_utils.h"
#include "adapters/slam/klt_vpi_acceleration.h"
#include "adapters/slam/slam_env.h"
#include "adapters/slam/slam_image_utils.h"

#include <algorithm>
#include <chrono>
#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace smartdrone::adapters::slam {
namespace {

struct KltPerFrameAccelerationRequest {
  bool requestVpi{false};
  bool requestVpiRemap{false};
  bool requestVpiDisparity{false};
  bool requestVpiLk{false};
};

KltPerFrameAccelerationRequest
ResolveKltPerFrameAccelerationRequest(const SlamModeSharedState &state) {
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
    SlamModeSharedState &state, const KltPerFrameFrontendResult &frontend) {
#if SMART_DRONE_HAS_VPI
  if (frontend.usedVpiRemap) {
    StoreVpiPreviousRectified(state.m_lkPerFrameVpi);
  }
#else
  (void)state;
  (void)frontend;
#endif
}

} // namespace

KltPerFrameFrontendResult RunKltPerFrameFrontend(SlamModeSharedState &state,
                                                 const cv::Mat &leftRaw,
                                                 const cv::Mat &rightRaw) {
  KltPerFrameFrontendResult result;
  const auto prepareStart = std::chrono::steady_clock::now();
  cv::Mat leftGray = EnsureGray8(leftRaw);
  cv::Mat rightGray = EnsureGray8(rightRaw);
  if (leftGray.empty() || rightGray.empty() || !state.m_lkCalibrationLoaded) {
    result.inputPrepareMs = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - prepareStart)
                                .count();
    return result;
  }

  const auto rectifyStart = std::chrono::steady_clock::now();
  state.EnsureStereoRectifier(leftGray.size());
  const KltPerFrameAccelerationRequest request =
      ResolveKltPerFrameAccelerationRequest(state);
  result.preferAcceleratedPnpDefaults = request.requestVpi;
  result.useKeyframeReference =
      EnvFlagEnabled("SMART_DRONE_LK_PER_FRAME_KEYFRAME", false);
  result.keyframeInterval =
      std::max(1, EnvIntValue("SMART_DRONE_LK_PER_FRAME_KEYFRAME_INTERVAL", 6));

  cv::Mat leftRect = leftGray;
  cv::Mat rightRect = rightGray;
  if (request.requestVpiRemap && !state.m_lkMap1x.empty() &&
      !state.m_lkMap2x.empty()) {
    result.usedVpiRemap = VpiRemapCurrentStereo(
        leftGray, rightGray, leftRect, rightRect, state.m_lkPerFrameVpi,
        state.m_lkMap1x, state.m_lkMap1y, state.m_lkMap2x, state.m_lkMap2y,
        state.m_lkPerFrameAccelLogged);
  }
  if (!result.usedVpiRemap && !state.m_lkMap1x.empty() &&
      !state.m_lkMap2x.empty()) {
    cv::remap(leftGray, leftRect, state.m_lkMap1x, state.m_lkMap1y,
              cv::INTER_LINEAR);
    cv::remap(rightGray, rightRect, state.m_lkMap2x, state.m_lkMap2y,
              cv::INTER_LINEAR);
  }
  result.leftRect = leftRect;
  result.rightRect = rightRect;
  result.valid = true;
  const auto rectifyEnd = std::chrono::steady_clock::now();
  result.rectifyMs =
      std::chrono::duration<double, std::milli>(rectifyEnd - rectifyStart)
          .count();
  result.inputPrepareMs =
      std::chrono::duration<double, std::milli>(rectifyEnd - prepareStart)
          .count();

  if (!state.m_lkHavePrev) {
    return result;
  }

  const auto disparityStart = std::chrono::steady_clock::now();
  cv::Mat disparity;
  bool usedVpiDisparity = false;
#if SMART_DRONE_HAS_VPI
  if (result.usedVpiRemap && request.requestVpiDisparity &&
      HasVpiPreviousRectified(state.m_lkPerFrameVpi)) {
    usedVpiDisparity = ComputeVpiCudaPreviousRectifiedDisparity(
        state.m_lkPrevLeft.size(), disparity, state.m_lkPerFrameVpi);
  }
#endif
  if (!usedVpiDisparity && request.requestVpiDisparity) {
    usedVpiDisparity = ComputeVpiCudaDisparity(
        state.m_lkPrevLeft, state.m_lkPrevRight, disparity,
        state.m_lkPerFrameVpi, state.m_lkPerFrameAccelLogged);
  }
  if (!usedVpiDisparity) {
    if (!state.m_lkPerFrameAccelLogged &&
        state.m_lkPerFrameAcceleration == "cpu") {
      std::cerr << "[lk_per_frame_accel] backend=cpu_sgbm\n";
      state.m_lkPerFrameAccelLogged = true;
    }
    if (!state.m_lkPerFrameSgbm) {
      const int numDisparities =
          std::max(16, ((result.leftRect.cols / 8 + 15) / 16) * 16);
      state.m_lkPerFrameSgbm =
          cv::StereoSGBM::create(0, numDisparities, 5, 8 * 5 * 5, 32 * 5 * 5, 1,
                                 31, 8, 60, 2, cv::StereoSGBM::MODE_SGBM_3WAY);
    }
    cv::Mat disparity16;
    state.m_lkPerFrameSgbm->compute(state.m_lkPrevLeft, state.m_lkPrevRight,
                                    disparity16);
    disparity16.convertTo(disparity, CV_32F, 1.0 / 16.0);
  }
  const auto disparityEnd = std::chrono::steady_clock::now();
  result.disparityMs =
      std::chrono::duration<double, std::milli>(disparityEnd - disparityStart)
          .count();

  const auto gfttStart = std::chrono::steady_clock::now();
  std::vector<cv::Point2f> previousPoints;
  std::vector<cv::Point2f> rawPreviousPoints;
  cv::goodFeaturesToTrack(state.m_lkPrevLeft, rawPreviousPoints,
                          kLkGfttPerFrameMaxCorners, kLkGfttQualityLevel,
                          kLkGfttMinDistancePx, cv::Mat(), kLkGfttBlockSize,
                          false, kLkGfttHarrisK);
  previousPoints = SelectGfttPointsGridBalanced(
      rawPreviousPoints, state.m_lkPrevLeft.size(), kLkGfttPerFrameMaxCorners,
      kLkGfttPerFrameMaxCornersPerCell);
  const auto gfttEnd = std::chrono::steady_clock::now();
  result.gfttMs =
      std::chrono::duration<double, std::milli>(gfttEnd - gfttStart).count();

  std::vector<cv::Point2f> currentPoints;
  std::vector<uint8_t> status;
  std::vector<float> errors;
  bool usedVpiLk = false;
  const auto flowStart = std::chrono::steady_clock::now();
#if SMART_DRONE_HAS_VPI
  if (result.usedVpiRemap && request.requestVpiLk && !previousPoints.empty()) {
    usedVpiLk = ComputeVpiCudaCurrentPyrLk(state.m_lkPrevLeft, previousPoints,
                                           currentPoints, status,
                                           state.m_lkPerFrameVpi);
  }
#endif
  if (!usedVpiLk && !previousPoints.empty()) {
    cv::calcOpticalFlowPyrLK(state.m_lkPrevLeft, result.leftRect,
                             previousPoints, currentPoints, status, errors,
                             cv::Size(21, 21), 3);
  }

  std::vector<cv::Point2f> backwardPoints;
  std::vector<uint8_t> backwardStatus;
  std::vector<float> backwardErrors;
  const bool useForwardBackwardCheck =
      EnvFlagEnabled("SMART_DRONE_LK_PER_FRAME_FB_CHECK", false);
  if (useForwardBackwardCheck && !currentPoints.empty()) {
    cv::calcOpticalFlowPyrLK(result.leftRect, state.m_lkPrevLeft, currentPoints,
                             backwardPoints, backwardStatus, backwardErrors,
                             cv::Size(21, 21), 3);
  }
  const auto flowEnd = std::chrono::steady_clock::now();
  result.flowMs =
      std::chrono::duration<double, std::milli>(flowEnd - flowStart).count();

  const auto candidateStart = std::chrono::steady_clock::now();
  KltPerFramePnpObservationBuilderOptions observationOptions;
  observationOptions.disparity = &disparity;
  observationOptions.previousImageSize = state.m_lkPrevLeft.size();
  observationOptions.currentImageSize = result.leftRect.size();
  observationOptions.previousPoints = &previousPoints;
  observationOptions.currentPoints = &currentPoints;
  observationOptions.status = &status;
  observationOptions.backwardPoints = &backwardPoints;
  observationOptions.backwardStatus = &backwardStatus;
  observationOptions.useForwardBackwardCheck = useForwardBackwardCheck;
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
  result.currentLeftPoints = std::move(currentPoints);
  const auto candidateEnd = std::chrono::steady_clock::now();
  result.candidateMs =
      std::chrono::duration<double, std::milli>(candidateEnd - candidateStart)
          .count();
  return result;
}

bool ShouldRefreshKltPerFrameReference(
    const SlamModeSharedState &state, const KltPerFrameFrontendResult &frontend,
    int inlierCount) {
  return !frontend.useKeyframeReference ||
         (state.m_lkFrameCount %
          static_cast<uint32_t>(std::max(1, frontend.keyframeInterval))) == 0 ||
         inlierCount < std::max(kLkMinPnPInliers * 2, 24);
}

void UpdateKltPerFrameReferenceFrame(
    SlamModeSharedState &state, const KltPerFrameFrontendResult &frontend) {
  state.m_lkPrevLeft = frontend.leftRect.clone();
  state.m_lkPrevRight = frontend.rightRect.clone();
  state.m_lkPerFrameReferenceTwc = state.m_lkTwc;
  StoreKltPerFramePreviousRectifiedIfNeeded(state, frontend);
}

} // namespace smartdrone::adapters::slam
