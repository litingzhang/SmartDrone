/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Tracking.h"

#include "FeatureMatcherBackend.h"
// #include "FrameDrawer.h"
#include "Converter.h"
#include "FrameFactory.h"
#include "G2oTypes.h"
#include "GeometricTools.h"
#include "KannalaBrandt8.h"
#include "LocalMapping.h"
#include "LocalMappingBackend.h"
#include "LoopClosingBackend.h"
#include "MLPnPsolver.h"
#include "OptimizationBackend.h"
#include "Pinhole.h"
#include "PlaceRecognitionBackend.h"
#include "TrackedVisualDataExtractor.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>

#include <chrono>
using namespace std;

namespace {

template <typename T> T ClampValue(T value, T minValue, T maxValue) {
  return std::max(minValue, std::min(value, maxValue));
}

const char *EnvValueWithStereoFeatureFallback(const char *name) {
  const char *value = std::getenv(name);
  if (value != nullptr && value[0] != '\0') {
    return value;
  }

  const std::string envName(name ? name : "");
  constexpr const char *kStereoFeaturePrefix = "SMART_DRONE_STEREO_FEATURE_";
  constexpr const char *kLegacyExternalStereoPrefix =
      "SMART_DRONE_EXTERNAL_STEREO_";
  if (envName.rfind(kStereoFeaturePrefix, 0) != 0) {
    return nullptr;
  }

  const std::string legacyName =
      std::string(kLegacyExternalStereoPrefix) +
      envName.substr(std::string(kStereoFeaturePrefix).size());
  value = std::getenv(legacyName.c_str());
  return (value != nullptr && value[0] != '\0') ? value : nullptr;
}

int EnvIntClamped(const char *name, int fallback, int minValue, int maxValue) {
  const char *value = EnvValueWithStereoFeatureFallback(name);
  if (value == nullptr || value[0] == '\0') {
    return ClampValue(fallback, minValue, maxValue);
  }
  char *end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value) {
    return ClampValue(fallback, minValue, maxValue);
  }
  return ClampValue(static_cast<int>(parsed), minValue, maxValue);
}

float EnvFloatClamped(const char *name, float fallback, float minValue,
                      float maxValue) {
  const char *value = EnvValueWithStereoFeatureFallback(name);
  if (value == nullptr || value[0] == '\0') {
    return ClampValue(fallback, minValue, maxValue);
  }
  char *end = nullptr;
  const float parsed = std::strtof(value, &end);
  if (end == value || !std::isfinite(parsed)) {
    return ClampValue(fallback, minValue, maxValue);
  }
  return ClampValue(parsed, minValue, maxValue);
}

bool EnvFlagEnabled(const char *name, bool fallback) {
  const char *value = EnvValueWithStereoFeatureFallback(name);
  if (value == nullptr || value[0] == '\0') {
    return fallback;
  }
  const std::string text(value);
  return !(text == "0" || text == "false" || text == "FALSE" || text == "off" ||
           text == "OFF" || text == "no" || text == "NO");
}

unsigned long EnvUnsignedLongClamped(const char *name, unsigned long fallback,
                                     unsigned long minValue,
                                     unsigned long maxValue) {
  const char *value = EnvValueWithStereoFeatureFallback(name);
  if (value == nullptr || value[0] == '\0') {
    return ClampValue(fallback, minValue, maxValue);
  }
  char *end = nullptr;
  const unsigned long parsed = std::strtoul(value, &end, 10);
  if (end == value) {
    return ClampValue(fallback, minValue, maxValue);
  }
  return ClampValue(parsed, minValue, maxValue);
}

} // namespace

namespace ORB_SLAM3 {
namespace {
uint64_t MixTrackHash(uint64_t hash, uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
  hash ^= value;
  hash *= 1099511628211ULL;
  return hash;
}

uint64_t HashMapPointSequence(const std::vector<MapPoint *> &points,
                              const std::vector<bool> *outliers = nullptr) {
  uint64_t hash = 1469598103934665603ULL;
  for (size_t i = 0; i < points.size(); ++i) {
    MapPoint *point = points[i];
    if (point == nullptr || point->isBad())
      continue;
    if (outliers != nullptr && i < outliers->size() && (*outliers)[i])
      continue;
    hash = MixTrackHash(hash, static_cast<uint64_t>(i));
    hash = MixTrackHash(hash, static_cast<uint64_t>(point->mnId));
  }
  return hash;
}

bool StableTrackingLocalOrderEnabled() {
  return EnvFlagEnabled("SMART_DRONE_ORB_STABLE_TRACKING_LOCAL_ORDER", false);
}

bool KeyFrameIdLess(const KeyFrame *lhs, const KeyFrame *rhs) {
  if (lhs == nullptr)
    return false;
  if (rhs == nullptr)
    return true;
  return lhs->mnId < rhs->mnId;
}

bool MapPointIdLess(const MapPoint *lhs, const MapPoint *rhs) {
  if (lhs == nullptr)
    return false;
  if (rhs == nullptr)
    return true;
  return lhs->mnId < rhs->mnId;
}
} // namespace

TrackedVisualSummary Tracking::GetTrackedVisualSummary() const {
  return BuildTrackedVisualSummary(TrackedVisualSummaryInput{
      GetTrackedMapPointCount(),
      GetLocalMapPointCount(),
      mnMatchesInliers,
      mLocalMapPointHash,
      mMatchedMapPointHashBeforePoseOptimization,
      mTrackedMapPointHash,
      mCurrentFrame.mnCloseMPs,
  });
}

TrackedFeatureSnapshot
Tracking::ExtractTrackedFeatures(int leftImageWidth, int leftImageHeight,
                                 int rightImageWidth,
                                 int rightImageHeight) const {
  return ExtractTrackedFeaturesFromFrame(mCurrentFrame, leftImageWidth,
                                         leftImageHeight, rightImageWidth,
                                         rightImageHeight);
}

TrackedPointCloudSnapshot
Tracking::ExtractTrackedPointCloud(size_t maxPointCloudPoints) const {
  return ExtractTrackedPointCloudFromFrame(mCurrentFrame, maxPointCloudPoints);
}

TrackedVisualData
Tracking::ExtractTrackedVisualData(int leftImageWidth, int leftImageHeight,
                                   int rightImageWidth, int rightImageHeight,
                                   bool includePointCloud,
                                   size_t maxPointCloudPoints) const {
  return ExtractTrackedVisualDataFromFrame(
      mCurrentFrame,
      TrackedVisualSummaryInput{
          GetTrackedMapPointCount(),
          GetLocalMapPointCount(),
          mnMatchesInliers,
          mLocalMapPointHash,
          mMatchedMapPointHashBeforePoseOptimization,
          mTrackedMapPointHash,
          mCurrentFrame.mnCloseMPs,
      },
      leftImageWidth, leftImageHeight, rightImageWidth, rightImageHeight,
      includePointCloud, maxPointCloudPoints);
}

int OptimizeCurrentFramePose(IOrbOptimizationBackend *backend, Frame &frame) {
  if (backend == nullptr) {
    return 0;
  }
  OrbPoseOptimizationResult result;
  backend->OptimizePose(OrbPoseOptimizationRequest{&frame}, result);
  return result.inliers;
}

int OptimizeCurrentFrameInertialLastFrame(IOrbOptimizationBackend *backend,
                                          Frame &frame) {
  if (backend == nullptr) {
    return 0;
  }
  OrbPoseOptimizationResult result;
  backend->OptimizeInertialPoseLastFrame(
      OrbInertialPoseOptimizationRequest{&frame}, result);
  return result.inliers;
}

int OptimizeCurrentFrameInertialLastKeyFrame(IOrbOptimizationBackend *backend,
                                             Frame &frame) {
  if (backend == nullptr) {
    return 0;
  }
  OrbPoseOptimizationResult result;
  backend->OptimizeInertialPoseLastKeyFrame(
      OrbInertialPoseOptimizationRequest{&frame}, result);
  return result.inliers;
}

void RunTrackingGlobalBundleAdjustment(IOrbOptimizationBackend *backend,
                                       Map *map, int iterations) {
  if (backend == nullptr || map == nullptr) {
    return;
  }
  backend->RunGlobalBundleAdjustment(
      OrbGlobalBundleAdjustmentRequest{map, iterations});
}

OrbLocalMappingStatus GetLocalMappingStatus(IOrbLocalMappingBackend *backend) {
  return backend != nullptr ? backend->Status() : OrbLocalMappingStatus{};
}

void InsertLocalMappingKeyFrame(IOrbLocalMappingBackend *backend,
                                KeyFrame *keyFrame) {
  if (backend != nullptr && keyFrame != nullptr) {
    backend->InsertKeyFrame(OrbLocalMappingKeyFrameRequest{keyFrame});
  }
}

void ResetLocalMappingBackend(IOrbLocalMappingBackend *backend, Map *map,
                              bool activeMapOnly) {
  if (backend != nullptr) {
    backend->Reset(OrbLocalMappingResetRequest{map, activeMapOnly});
  }
}

OrbLoopClosingTimingStats
GetLoopClosingTimingStats(IOrbLoopClosingBackend *backend) {
  return backend != nullptr ? backend->TimingStats()
                            : OrbLoopClosingTimingStats{};
}

void ResetLoopClosingBackend(IOrbLoopClosingBackend *backend, Map *map,
                             bool activeMapOnly) {
  if (backend != nullptr) {
    backend->Reset(OrbLoopClosingResetRequest{map, activeMapOnly});
  }
}

namespace {
int CountTrackedMapPoints(const Frame &frame) {
  int tracked = 0;
  for (int i = 0; i < frame.N; ++i) {
    if (frame.mvpMapPoints[i] && !frame.mvbOutlier[i])
      ++tracked;
  }
  return tracked;
}

constexpr int kStereoFeatureStabilizingFrameWindow = 1200;
constexpr unsigned long kStereoFeatureBootstrapKeyframeLimit = 12;
constexpr unsigned long kStereoFeatureStabilizingKeyframeLimit = 120;
constexpr unsigned long kPureStereoBootstrapKeyframeLimit = 6;
constexpr unsigned long kPureStereoStabilizingKeyframeLimit = 24;

int StereoFeatureStabilizingFrameWindow() {
  return EnvIntClamped("SMART_DRONE_STEREO_FEATURE_STABILIZING_FRAME_WINDOW",
                       kStereoFeatureStabilizingFrameWindow, 0, 100000);
}

unsigned long StereoFeatureBootstrapKeyframeLimit() {
  return EnvUnsignedLongClamped("SMART_DRONE_STEREO_FEATURE_BOOTSTRAP_KF_LIMIT",
                                kStereoFeatureBootstrapKeyframeLimit, 0, 1000);
}

unsigned long StereoFeatureStabilizingKeyframeLimit() {
  return EnvUnsignedLongClamped(
      "SMART_DRONE_STEREO_FEATURE_STABILIZING_KF_LIMIT",
      kStereoFeatureStabilizingKeyframeLimit, 0, 1000);
}

int StereoFeatureBootstrapMaxMapPointsPerKeyframe() {
  return EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_BOOTSTRAP_MAX_MAPPOINTS_PER_KF", 240, 1,
      1000);
}

int StereoFeatureStabilizingMaxMapPointsPerKeyframe() {
  return EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_STABILIZING_MAX_MAPPOINTS_PER_KF", 180, 1,
      1000);
}

int StereoFeatureStableMaxMapPointsPerKeyframe() {
  return EnvIntClamped("SMART_DRONE_STEREO_FEATURE_STABLE_MAX_MAPPOINTS_PER_KF",
                       100, 1, 1000);
}

int StereoFeatureStableLocalSearchTh() {
  return EnvIntClamped("SMART_DRONE_STEREO_FEATURE_STABLE_LOCAL_SEARCH_TH", 1,
                       1, 15);
}

int StereoFeatureMinFramesBetweenKeyframes() {
  return EnvIntClamped("SMART_DRONE_STEREO_FEATURE_MIN_FRAMES_BETWEEN_KF", 4, 1,
                       120);
}

bool IsStereoFeatureStabilizing(const Frame &frame, Atlas *pAtlas,
                                int initFrameId) {
  if (!pAtlas || !frame.mbStereoFeatureInjected || initFrameId < 0 ||
      frame.mnId < initFrameId)
    return false;

  const int framesSinceInit = frame.mnId - initFrameId;
  const unsigned long keyFramesInMap = pAtlas->KeyFramesInMap();
  return framesSinceInit <= StereoFeatureStabilizingFrameWindow() &&
         keyFramesInMap <= StereoFeatureStabilizingKeyframeLimit();
}

bool IsStereoFeatureBootstrap(const Frame &frame, Atlas *pAtlas,
                              int initFrameId) {
  return IsStereoFeatureStabilizing(frame, pAtlas, initFrameId) &&
         pAtlas->KeyFramesInMap() <= StereoFeatureBootstrapKeyframeLimit();
}

bool IsStereoFeatureRecoveryHopeless(const Frame &frame, Atlas *pAtlas,
                                     int initFrameId, int matchesInliers) {
  if (!frame.mbStereoFeatureInjected || !pAtlas)
    return false;

  const bool stereoFeatureStabilizing =
      IsStereoFeatureStabilizing(frame, pAtlas, initFrameId);
  if (!stereoFeatureStabilizing)
    return false;

  const int trackedMapPoints = CountTrackedMapPoints(frame);
  return pAtlas->KeyFramesInMap() > 1 && matchesInliers <= 2 &&
         trackedMapPoints <= 2;
}

bool StereoFeatureTrackDfxEnabled() {
  return EnvFlagEnabled("SMART_DRONE_STEREO_FEATURE_TRACK_DFX", false);
}

bool StereoFeaturePoseRescueEnabled() {
  return EnvFlagEnabled("SMART_DRONE_STEREO_FEATURE_POSE_RESCUE", true);
}

int StereoFeatureLeftFeatureCount(const Frame &frame) {
  return frame.Nleft == -1 ? frame.N : frame.Nleft;
}

bool IsStereoFeatureObservationHealthy(const Frame &frame) {
  if (!frame.mbStereoFeatureInjected)
    return false;

  const int featureCount = StereoFeatureLeftFeatureCount(frame);
  const int minFeatures = EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_RESCUE_MIN_FEATURES", 48, 1, 1000);
  const int minClose = EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_RESCUE_MIN_CLOSE_POINTS", 24, 0, 1000);
  const float minCloseRatio = EnvFloatClamped(
      "SMART_DRONE_STEREO_FEATURE_RESCUE_MIN_CLOSE_RATIO", 0.30f, 0.0f, 1.0f);
  return featureCount >= minFeatures && frame.mnCloseMPs >= minClose &&
         static_cast<float>(frame.mnCloseMPs) >=
             minCloseRatio * static_cast<float>(featureCount);
}

bool StereoFeatureStableJumpGuardEnabled() {
  return EnvFlagEnabled("SMART_DRONE_STEREO_FEATURE_STABLE_JUMP_GUARD", false);
}

bool StereoFeatureRequireMapInliers() {
  return EnvFlagEnabled("SMART_DRONE_STEREO_FEATURE_REQUIRE_MAP_INLIERS",
                        false);
}

int StereoFeatureStableJumpGuardMinInliers() {
  return EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_STABLE_JUMP_GUARD_MIN_INLIERS", 110, 1, 2000);
}

int StereoFeatureStableJumpGuardMinTrackedMapPoints() {
  return EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_STABLE_JUMP_GUARD_MIN_TRACKED_MAP", 90, 1,
      2000);
}

float StereoFeatureStableJumpGuardMaxStepMeters() {
  return EnvFloatClamped(
      "SMART_DRONE_STEREO_FEATURE_STABLE_JUMP_GUARD_MAX_STEP_M", 0.09f, 0.01f,
      0.50f);
}

float PoseStepMetersBetweenFrames(const Frame &currentFrame,
                                  const Frame &lastFrame) {
  if (!currentFrame.isSet() || !lastFrame.isSet())
    return 0.0f;

  const Eigen::Vector3f currentTwc =
      currentFrame.GetPose().inverse().translation();
  const Eigen::Vector3f lastTwc = lastFrame.GetPose().inverse().translation();
  return (currentTwc - lastTwc).norm();
}

void LogStereoFeaturePoseRescue(const char *reason, const Frame &frame,
                                Atlas *pAtlas, int state, int initFrameId,
                                int matchesInliers) {
  if (!StereoFeatureTrackDfxEnabled() || !frame.mbStereoFeatureInjected)
    return;

  cerr << "[stereo_feature_pose_rescue] frame=" << frame.mnId
       << " reason=" << reason << " state=" << state
       << " features=" << StereoFeatureLeftFeatureCount(frame)
       << " close=" << frame.mnCloseMPs
       << " tracked=" << CountTrackedMapPoints(frame)
       << " inliers=" << matchesInliers
       << " kfs=" << (pAtlas ? pAtlas->KeyFramesInMap() : 0)
       << " init_frame=" << initFrameId << " stabilizing="
       << (IsStereoFeatureStabilizing(frame, pAtlas, initFrameId) ? "Y" : "N")
       << " bootstrap="
       << (IsStereoFeatureBootstrap(frame, pAtlas, initFrameId) ? "Y" : "N")
       << "\n";
}

bool IsPureStereoStabilizing(int sensor, Atlas *pAtlas) {
  if (sensor != System::STEREO || !pAtlas)
    return false;

  const unsigned long keyFramesInMap = pAtlas->KeyFramesInMap();
  return keyFramesInMap > 0 &&
         keyFramesInMap <= kPureStereoStabilizingKeyframeLimit;
}

bool IsPureStereoBootstrap(int sensor, Atlas *pAtlas) {
  return IsPureStereoStabilizing(sensor, pAtlas) &&
         pAtlas->KeyFramesInMap() <= kPureStereoBootstrapKeyframeLimit;
}

} // namespace

SlamFrameTrackingStats Tracking::GetFrameTrackingStats() const {
  SlamFrameTrackingStats out;
  out.featureExtractMs = mCurrentFrame.mTimeORB_Ext;
  out.stereoMatchMs = mCurrentFrame.mTimeStereoMatch;
  out.featureCount = static_cast<uint32_t>(std::max(0, mCurrentFrame.N));
  out.closeMapPointCount =
      static_cast<uint32_t>(std::max(0, mCurrentFrame.mnCloseMPs));
  out.frameId = static_cast<uint64_t>(mCurrentFrame.mnId);
  out.referenceKeyFrameId =
      mCurrentFrame.mpReferenceKF != nullptr
          ? static_cast<int64_t>(mCurrentFrame.mpReferenceKF->mnId)
          : -1;
  KeyFrame *lastKeyFrame = mpLastKeyFrame;
  out.lastKeyFrameId =
      lastKeyFrame != nullptr ? static_cast<int64_t>(lastKeyFrame->mnId) : -1;
  out.lastKeyFrameFrameId = lastKeyFrame != nullptr
                                ? static_cast<int64_t>(lastKeyFrame->mnFrameId)
                                : -1;

  Map *currentMap = nullptr;
  if (mCurrentFrame.mpReferenceKF != nullptr) {
    currentMap = mCurrentFrame.mpReferenceKF->GetMap();
  } else if (lastKeyFrame != nullptr) {
    currentMap = lastKeyFrame->GetMap();
  }
  out.keyFramesInMap = currentMap != nullptr
                           ? static_cast<uint32_t>(currentMap->KeyFramesInMap())
                           : 0U;
  out.stereoFeatureInitFrameId = mnStereoFeatureInitFrameId;
  out.stereoFeatureInjected = mCurrentFrame.mbStereoFeatureInjected;
  out.stereoFeatureStabilizing = IsStereoFeatureStabilizing(
      mCurrentFrame, mpAtlas, mnStereoFeatureInitFrameId);
  out.stereoFeatureBootstrap = IsStereoFeatureBootstrap(
      mCurrentFrame, mpAtlas, mnStereoFeatureInitFrameId);
  return out;
}

std::vector<OrbFrameTrajectoryEntry>
Tracking::GetFrameTrajectorySnapshot() const {
  std::vector<OrbFrameTrajectoryEntry> entries;
  auto poseIt = mlRelativeFramePoses.begin();
  auto referenceIt = mlpReferences.begin();
  auto timestampIt = mlFrameTimes.begin();
  auto lostIt = mlbLost.begin();
  for (; poseIt != mlRelativeFramePoses.end() &&
         referenceIt != mlpReferences.end() &&
         timestampIt != mlFrameTimes.end() && lostIt != mlbLost.end();
       ++poseIt, ++referenceIt, ++timestampIt, ++lostIt) {
    entries.push_back(OrbFrameTrajectoryEntry{*poseIt, *referenceIt,
                                              *timestampIt, *lostIt});
  }
  return entries;
}

bool Tracking::GetLatestFrameTrajectoryEntry(
    OrbFrameTrajectoryEntry &entry) const {
  if (mlRelativeFramePoses.empty() || mlpReferences.empty() ||
      mlFrameTimes.empty() || mlbLost.empty()) {
    return false;
  }
  entry = OrbFrameTrajectoryEntry{mlRelativeFramePoses.back(),
                                  mlpReferences.back(), mlFrameTimes.back(),
                                  mlbLost.back()};
  return true;
}

Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, Atlas *pAtlas,
                   KeyFrameDatabase *pKFDB, const string &strSettingPath,
                   const int sensor, Settings *settings, const string &_nameSeq)
    : mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFr(0), mbStep(false),
      mbOnlyTracking(false), mbMapUpdated(false), mbVO(false),
      mpFrameFactory(CreateDefaultOrbFrameFactory()),
      mpOptimizationBackend(CreateDefaultOrbOptimizationBackend()),
      mpPlaceRecognitionBackend(
          CreateDefaultOrbPlaceRecognitionBackend(pKFDB)),
      mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB), mbReadyToInitializate(false),
      mpSystem(pSys), bStepByStep(false), mpAtlas(pAtlas),
      mnLastRelocFrameId(0), time_recently_lost(5.0), mnInitialFrameId(0),
      mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr),
      mpLastKeyFrame(static_cast<KeyFrame *>(NULL)) {
  // Load camera parameters from settings file
  if (settings) {
    newParameterLoader(settings);
  } else {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    bool b_parse_cam = ParseCamParamFile(fSettings);
    if (!b_parse_cam) {
      std::cout << "*Error with the camera parameters in the config file*"
                << std::endl;
    }

    // Load ORB parameters
    bool b_parse_orb = ParseORBParamFile(fSettings);
    if (!b_parse_orb) {
      std::cout << "*Error with the ORB parameters in the config file*"
                << std::endl;
    }

    bool b_parse_imu = true;
    if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
        sensor == System::IMU_RGBD) {
      b_parse_imu = ParseIMUParamFile(fSettings);
      if (!b_parse_imu) {
        std::cout << "*Error with the IMU parameters in the config file*"
                  << std::endl;
      }

      mnFramesToResetIMU = mMaxFrames;
    }

    if (!b_parse_cam || !b_parse_orb || !b_parse_imu) {
      std::cerr << "**ERROR in the config file, the format is not correct**"
                << std::endl;
      try {
        throw -1;
      } catch (exception &e) {
      }
    }
  }

  initID = 0;
  lastID = 0;
  mbInitWith3KFs = false;
  mnNumDataset = 0;

  vector<GeometricCamera *> vpCams = mpAtlas->GetAllCameras();
  std::cout << "There are " << vpCams.size() << " cameras in the atlas"
            << std::endl;
  for (GeometricCamera *pCam : vpCams) {
    std::cout << "Camera " << pCam->GetId();
    if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
      std::cout << " is pinhole" << std::endl;
    } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
      std::cout << " is fisheye" << std::endl;
    } else {
      std::cout << " is unknown" << std::endl;
    }
  }

#ifdef REGISTER_TIMES
  vdRectStereo_ms.clear();
  vdResizeImage_ms.clear();
  vdORBExtract_ms.clear();
  vdStereoMatch_ms.clear();
  vdIMUInteg_ms.clear();
  vdPosePred_ms.clear();
  vdLMTrack_ms.clear();
  vdNewKF_ms.clear();
  vdTrackTotal_ms.clear();
#endif
}

#ifdef REGISTER_TIMES
double calcAverage(vector<double> v_times) {
  double accum = 0;
  for (double value : v_times) {
    accum += value;
  }

  return accum / v_times.size();
}

double calcDeviation(vector<double> v_times, double average) {
  double accum = 0;
  for (double value : v_times) {
    accum += pow(value - average, 2);
  }
  return sqrt(accum / v_times.size());
}

double calcAverage(vector<int> v_values) {
  double accum = 0;
  int total = 0;
  for (double value : v_values) {
    if (value == 0)
      continue;
    accum += value;
    total++;
  }

  return accum / total;
}

double calcDeviation(vector<int> v_values, double average) {
  double accum = 0;
  int total = 0;
  for (double value : v_values) {
    if (value == 0)
      continue;
    accum += pow(value - average, 2);
    total++;
  }
  return sqrt(accum / total);
}

void Tracking::LocalMapStats2File() {
  const OrbLocalMappingTimingStats timing =
      mpLocalMappingBackend ? mpLocalMappingBackend->TimingStats()
                            : OrbLocalMappingTimingStats{};
  ofstream f;
  f.open("LocalMapTimeStats.txt");
  f << fixed << setprecision(6);
  f << "#Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF "
       "culling[ms], Total[ms]"
    << endl;
  for (size_t i = 0; i < timing.totalMs.size(); ++i) {
    f << timing.keyframeInsertMs[i] << "," << timing.mapPointCullingMs[i]
      << "," << timing.mapPointCreationMs[i] << ","
      << timing.localBundleAdjustmentSyncMs[i] << ","
      << timing.keyframeCullingSyncMs[i] << "," << timing.totalMs[i] << endl;
  }

  f.close();

  f.open("LBA_Stats.txt");
  f << fixed << setprecision(6);
  f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << endl;
  for (size_t i = 0; i < timing.localBundleAdjustmentSyncMs.size(); ++i) {
    f << timing.localBundleAdjustmentSyncMs[i] << ","
      << timing.localBundleAdjustmentOptimizedKeyframes[i] << ","
      << timing.localBundleAdjustmentFixedKeyframes[i] << ","
      << timing.localBundleAdjustmentMapPoints[i] << ","
      << timing.localBundleAdjustmentEdges[i] << endl;
  }

  f.close();
}

void Tracking::TrackStats2File() {
  ofstream f;
  f.open("SessionInfo.txt");
  f << fixed;
  f << "Number of KFs: " << mpAtlas->GetAllKeyFrames().size() << endl;
  f << "Number of MPs: " << mpAtlas->GetAllMapPoints().size() << endl;

  f << "OpenCV version: " << CV_VERSION << endl;

  f.close();

  f.open("TrackingTimeStats.txt");
  f << fixed << setprecision(6);

  f << "#Image Rect[ms], Image Resize[ms], ORB ext[ms], Stereo match[ms], IMU "
       "preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]"
    << endl;

  for (int i = 0; i < vdTrackTotal_ms.size(); ++i) {
    double stereo_rect = 0.0;
    if (!vdRectStereo_ms.empty()) {
      stereo_rect = vdRectStereo_ms[i];
    }

    double resize_image = 0.0;
    if (!vdResizeImage_ms.empty()) {
      resize_image = vdResizeImage_ms[i];
    }

    double stereo_match = 0.0;
    if (!vdStereoMatch_ms.empty()) {
      stereo_match = vdStereoMatch_ms[i];
    }

    double imu_preint = 0.0;
    if (!vdIMUInteg_ms.empty()) {
      imu_preint = vdIMUInteg_ms[i];
    }

    f << stereo_rect << "," << resize_image << "," << vdORBExtract_ms[i] << ","
      << stereo_match << "," << imu_preint << "," << vdPosePred_ms[i] << ","
      << vdLMTrack_ms[i] << "," << vdNewKF_ms[i] << "," << vdTrackTotal_ms[i]
      << endl;
  }

  f.close();
}

void Tracking::PrintTimeStats() {
  // Save data in files
  TrackStats2File();
  LocalMapStats2File();

  ofstream f;
  f.open("ExecMean.txt");
  f << fixed;
  // Report the mean and std of each one
  std::cout << std::endl << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
  f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
  cout << "OpenCV version: " << CV_VERSION << endl;
  f << "OpenCV version: " << CV_VERSION << endl;
  std::cout << "---------------------------" << std::endl;
  std::cout << "Tracking" << std::setprecision(5) << std::endl << std::endl;
  f << "---------------------------" << std::endl;
  f << "Tracking" << std::setprecision(5) << std::endl << std::endl;
  double average, deviation;
  if (!vdRectStereo_ms.empty()) {
    average = calcAverage(vdRectStereo_ms);
    deviation = calcDeviation(vdRectStereo_ms, average);
    std::cout << "Stereo Rectification: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Stereo Rectification: " << average << "$\\pm$" << deviation
      << std::endl;
  }

  if (!vdResizeImage_ms.empty()) {
    average = calcAverage(vdResizeImage_ms);
    deviation = calcDeviation(vdResizeImage_ms, average);
    std::cout << "Image Resize: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
  }

  average = calcAverage(vdORBExtract_ms);
  deviation = calcDeviation(vdORBExtract_ms, average);
  std::cout << "ORB Extraction: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;

  if (!vdStereoMatch_ms.empty()) {
    average = calcAverage(vdStereoMatch_ms);
    deviation = calcDeviation(vdStereoMatch_ms, average);
    std::cout << "Stereo Matching: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
  }

  if (!vdIMUInteg_ms.empty()) {
    average = calcAverage(vdIMUInteg_ms);
    deviation = calcDeviation(vdIMUInteg_ms, average);
    std::cout << "IMU Preintegration: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "IMU Preintegration: " << average << "$\\pm$" << deviation
      << std::endl;
  }

  average = calcAverage(vdPosePred_ms);
  deviation = calcDeviation(vdPosePred_ms, average);
  std::cout << "Pose Prediction: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(vdLMTrack_ms);
  deviation = calcDeviation(vdLMTrack_ms, average);
  std::cout << "LM Track: " << average << "$\\pm$" << deviation << std::endl;
  f << "LM Track: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(vdNewKF_ms);
  deviation = calcDeviation(vdNewKF_ms, average);
  std::cout << "New KF decision: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(vdTrackTotal_ms);
  deviation = calcDeviation(vdTrackTotal_ms, average);
  std::cout << "Total Tracking: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;

  // Local Mapping time stats
  const OrbLocalMappingTimingStats timing =
      mpLocalMappingBackend ? mpLocalMappingBackend->TimingStats()
                            : OrbLocalMappingTimingStats{};
  std::cout << std::endl << std::endl << std::endl;
  std::cout << "Local Mapping" << std::endl << std::endl;
  f << std::endl << "Local Mapping" << std::endl << std::endl;

  average = calcAverage(timing.keyframeInsertMs);
  deviation = calcDeviation(timing.keyframeInsertMs, average);
  std::cout << "KF Insertion: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(timing.mapPointCullingMs);
  deviation = calcDeviation(timing.mapPointCullingMs, average);
  std::cout << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;
  f << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(timing.mapPointCreationMs);
  deviation = calcDeviation(timing.mapPointCreationMs, average);
  std::cout << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;
  f << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(timing.localBundleAdjustmentMs);
  deviation = calcDeviation(timing.localBundleAdjustmentMs, average);
  std::cout << "LBA: " << average << "$\\pm$" << deviation << std::endl;
  f << "LBA: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(timing.keyframeCullingMs);
  deviation = calcDeviation(timing.keyframeCullingMs, average);
  std::cout << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;
  f << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(timing.totalMs);
  deviation = calcDeviation(timing.totalMs, average);
  std::cout << "Total Local Mapping: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;

  // Local Mapping LBA complexity
  std::cout << "---------------------------" << std::endl;
  std::cout << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;
  f << "---------------------------" << std::endl;
  f << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;

  average = calcAverage(timing.localBundleAdjustmentEdges);
  deviation = calcDeviation(timing.localBundleAdjustmentEdges, average);
  std::cout << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;
  f << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(timing.localBundleAdjustmentOptimizedKeyframes);
  deviation =
      calcDeviation(timing.localBundleAdjustmentOptimizedKeyframes, average);
  std::cout << "LBA KF optimized: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(timing.localBundleAdjustmentFixedKeyframes);
  deviation =
      calcDeviation(timing.localBundleAdjustmentFixedKeyframes, average);
  std::cout << "LBA KF fixed: " << average << "$\\pm$" << deviation
            << std::endl;
  f << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;

  average = calcAverage(timing.localBundleAdjustmentMapPoints);
  deviation = calcDeviation(timing.localBundleAdjustmentMapPoints, average);
  std::cout << "LBA MP: " << average << "$\\pm$" << deviation << std::endl
            << std::endl;
  f << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;

  std::cout << "LBA executions: "
            << timing.localBundleAdjustmentExecutions << std::endl;
  std::cout << "LBA aborts: " << timing.localBundleAdjustmentAborts
            << std::endl;
  f << "LBA executions: " << timing.localBundleAdjustmentExecutions
    << std::endl;
  f << "LBA aborts: " << timing.localBundleAdjustmentAborts << std::endl;

  // Map complexity
  std::cout << "---------------------------" << std::endl;
  std::cout << std::endl << "Map complexity" << std::endl;
  std::cout << "KFs in map: " << mpAtlas->GetAllKeyFrames().size() << std::endl;
  std::cout << "MPs in map: " << mpAtlas->GetAllMapPoints().size() << std::endl;
  f << "---------------------------" << std::endl;
  f << std::endl << "Map complexity" << std::endl;
  vector<Map *> vpMaps = mpAtlas->GetAllMaps();
  Map *pBestMap = vpMaps[0];
  for (int i = 1; i < vpMaps.size(); ++i) {
    if (pBestMap->GetAllKeyFrames().size() <
        vpMaps[i]->GetAllKeyFrames().size()) {
      pBestMap = vpMaps[i];
    }
  }

  f << "KFs in map: " << pBestMap->GetAllKeyFrames().size() << std::endl;
  f << "MPs in map: " << pBestMap->GetAllMapPoints().size() << std::endl;

  const OrbLoopClosingTimingStats loopTiming =
      GetLoopClosingTimingStats(mpLoopClosingBackend.get());

  f << "---------------------------" << std::endl;
  f << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
  std::cout << "---------------------------" << std::endl;
  std::cout << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
  average = calcAverage(loopTiming.databaseQueryMs);
  deviation = calcDeviation(loopTiming.databaseQueryMs, average);
  f << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Database Query: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(loopTiming.sim3EstimationMs);
  deviation = calcDeviation(loopTiming.sim3EstimationMs, average);
  f << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "SE3 estimation: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(loopTiming.placeRecognitionTotalMs);
  deviation = calcDeviation(loopTiming.placeRecognitionTotalMs, average);
  f << "Total Place Recognition: " << average << "$\\pm$" << deviation
    << std::endl
    << std::endl;
  std::cout << "Total Place Recognition: " << average << "$\\pm$" << deviation
            << std::endl
            << std::endl;

  f << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
  std::cout << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
  average = calcAverage(loopTiming.loopFusionMs);
  deviation = calcDeviation(loopTiming.loopFusionMs, average);
  f << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
  average = calcAverage(loopTiming.loopEssentialGraphMs);
  deviation = calcDeviation(loopTiming.loopEssentialGraphMs, average);
  f << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Essential Graph: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(loopTiming.loopTotalMs);
  deviation = calcDeviation(loopTiming.loopTotalMs, average);
  f << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl
    << std::endl;
  std::cout << "Total Loop Closing: " << average << "$\\pm$" << deviation
            << std::endl
            << std::endl;

  f << "Numb exec: " << loopTiming.loopExecutions << std::endl;
  std::cout << "Num exec: " << loopTiming.loopExecutions << std::endl;
  average = calcAverage(loopTiming.loopKeyframes);
  deviation = calcDeviation(loopTiming.loopKeyframes, average);
  f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
            << std::endl;

  f << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
  std::cout << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
  average = calcAverage(loopTiming.mergeMapsMs);
  deviation = calcDeviation(loopTiming.mergeMapsMs, average);
  f << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
  average = calcAverage(loopTiming.weldingBundleAdjustmentMs);
  deviation = calcDeviation(loopTiming.weldingBundleAdjustmentMs, average);
  f << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
  average = calcAverage(loopTiming.mergeEssentialGraphMs);
  deviation = calcDeviation(loopTiming.mergeEssentialGraphMs, average);
  f << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Optimization Ess.: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(loopTiming.mergeTotalMs);
  deviation = calcDeviation(loopTiming.mergeTotalMs, average);
  f << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl
    << std::endl;
  std::cout << "Total Map Merging: " << average << "$\\pm$" << deviation
            << std::endl
            << std::endl;

  f << "Numb exec: " << loopTiming.mergeExecutions << std::endl;
  std::cout << "Num exec: " << loopTiming.mergeExecutions << std::endl;
  average = calcAverage(loopTiming.mergeKeyframes);
  deviation = calcDeviation(loopTiming.mergeKeyframes, average);
  f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(loopTiming.mergeMapPoints);
  deviation = calcDeviation(loopTiming.mergeMapPoints, average);
  f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Number of MPs: " << average << "$\\pm$" << deviation
            << std::endl;

  f << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
  std::cout << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
  average = calcAverage(loopTiming.globalBundleAdjustmentMs);
  deviation = calcDeviation(loopTiming.globalBundleAdjustmentMs, average);
  f << "GBA: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "GBA: " << average << "$\\pm$" << deviation << std::endl;
  average = calcAverage(loopTiming.mapUpdateMs);
  deviation = calcDeviation(loopTiming.mapUpdateMs, average);
  f << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
  average = calcAverage(loopTiming.fullGlobalBundleAdjustmentTotalMs);
  deviation = calcDeviation(loopTiming.fullGlobalBundleAdjustmentTotalMs, average);
  f << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl
    << std::endl;
  std::cout << "Total Full GBA: " << average << "$\\pm$" << deviation
            << std::endl
            << std::endl;

  f << "Numb exec: " << loopTiming.globalBundleAdjustmentExecutions << std::endl;
  std::cout << "Num exec: " << loopTiming.globalBundleAdjustmentExecutions << std::endl;
  f << "Numb abort: " << loopTiming.globalBundleAdjustmentAborts << std::endl;
  std::cout << "Num abort: " << loopTiming.globalBundleAdjustmentAborts << std::endl;
  average = calcAverage(loopTiming.globalBundleAdjustmentKeyframes);
  deviation = calcDeviation(loopTiming.globalBundleAdjustmentKeyframes, average);
  f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
            << std::endl;
  average = calcAverage(loopTiming.globalBundleAdjustmentMapPoints);
  deviation = calcDeviation(loopTiming.globalBundleAdjustmentMapPoints, average);
  f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
  std::cout << "Number of MPs: " << average << "$\\pm$" << deviation
            << std::endl;

  f.close();
}

#endif

Tracking::~Tracking() {
  // f_track_stats.close();
}

void Tracking::newParameterLoader(Settings *settings) {
  mpCamera = settings->camera1();
  mpCamera = mpAtlas->AddCamera(mpCamera);

  if (settings->needToUndistort()) {
    mDistCoef = settings->camera1DistortionCoef();
  } else {
    mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
  }

  // TODO: missing image scaling and rectification
  mImageScale = 1.0f;

  mK = cv::Mat::eye(3, 3, CV_32F);
  mK.at<float>(0, 0) = mpCamera->getParameter(0);
  mK.at<float>(1, 1) = mpCamera->getParameter(1);
  mK.at<float>(0, 2) = mpCamera->getParameter(2);
  mK.at<float>(1, 2) = mpCamera->getParameter(3);

  mK_.setIdentity();
  mK_(0, 0) = mpCamera->getParameter(0);
  mK_(1, 1) = mpCamera->getParameter(1);
  mK_(0, 2) = mpCamera->getParameter(2);
  mK_(1, 2) = mpCamera->getParameter(3);

  if ((mSensor == System::STEREO || mSensor == System::IMU_STEREO ||
       mSensor == System::IMU_RGBD) &&
      settings->cameraType() == Settings::KannalaBrandt) {
    mpCamera2 = settings->camera2();
    mpCamera2 = mpAtlas->AddCamera(mpCamera2);

    mTlr = settings->Tlr();

    // mpFrameDrawer->both = true;
  }

  if (mSensor == System::STEREO || mSensor == System::RGBD ||
      mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
    mbf = settings->bf();
    mThDepth = settings->b() * settings->thDepth();
  }

  if (mSensor == System::RGBD || mSensor == System::IMU_RGBD) {
    mDepthMapFactor = settings->depthMapFactor();
    if (fabs(mDepthMapFactor) < 1e-5)
      mDepthMapFactor = 1;
    else
      mDepthMapFactor = 1.0f / mDepthMapFactor;
  }

  mMinFrames = 0;
  mMaxFrames = settings->fps();
  mbRGB = settings->rgb();

  // ORB parameters
  int nFeatures = settings->nFeatures();
  int nLevels = settings->nLevels();
  int fIniThFAST = settings->initThFAST();
  int fMinThFAST = settings->minThFAST();
  float fScaleFactor = settings->scaleFactor();

  mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels,
                                        fIniThFAST, fMinThFAST);

  if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
    mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels,
                                           fIniThFAST, fMinThFAST);

  if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    mpIniORBextractor = new ORBextractor(5 * nFeatures, fScaleFactor, nLevels,
                                         fIniThFAST, fMinThFAST);

  // IMU parameters
  Sophus::SE3f Tbc = settings->Tbc();
  mInsertKFsLost = settings->insertKFsWhenLost();
  mImuFreq = settings->imuFrequency();
  mImuPer = (mImuFreq > 0.0f) ? (1.0 / static_cast<double>(mImuFreq)) : 0.001;
  float Ng = settings->noiseGyro();
  float Na = settings->noiseAcc();
  float Ngw = settings->gyroWalk();
  float Naw = settings->accWalk();

  const float sf = sqrt(mImuFreq);
  mpImuCalib = new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

  mpImuPreintegratedFromLastKF =
      new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
}

bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings) {
  mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
  cout << endl << "Camera Parameters: " << endl;
  bool b_miss_params = false;

  string sCameraName = fSettings["Camera.type"];
  if (sCameraName == "PinHole") {
    float fx, fy, cx, cy;
    mImageScale = 1.f;

    // Camera calibration parameters
    cv::FileNode node = fSettings["Camera.fx"];
    if (!node.empty() && node.isReal()) {
      fx = node.real();
    } else {
      std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.fy"];
    if (!node.empty() && node.isReal()) {
      fy = node.real();
    } else {
      std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.cx"];
    if (!node.empty() && node.isReal()) {
      cx = node.real();
    } else {
      std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.cy"];
    if (!node.empty() && node.isReal()) {
      cy = node.real();
    } else {
      std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    // Distortion parameters
    node = fSettings["Camera.k1"];
    if (!node.empty() && node.isReal()) {
      mDistCoef.at<float>(0) = node.real();
    } else {
      std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.k2"];
    if (!node.empty() && node.isReal()) {
      mDistCoef.at<float>(1) = node.real();
    } else {
      std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.p1"];
    if (!node.empty() && node.isReal()) {
      mDistCoef.at<float>(2) = node.real();
    } else {
      std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.p2"];
    if (!node.empty() && node.isReal()) {
      mDistCoef.at<float>(3) = node.real();
    } else {
      std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.k3"];
    if (!node.empty() && node.isReal()) {
      mDistCoef.resize(5);
      mDistCoef.at<float>(4) = node.real();
    }

    node = fSettings["Camera.imageScale"];
    if (!node.empty() && node.isReal()) {
      mImageScale = node.real();
    }

    if (b_miss_params) {
      return false;
    }

    if (mImageScale != 1.f) {
      // K matrix parameters must be scaled.
      fx = fx * mImageScale;
      fy = fy * mImageScale;
      cx = cx * mImageScale;
      cy = cy * mImageScale;
    }

    vector<float> vCamCalib{fx, fy, cx, cy};

    mpCamera = new Pinhole(vCamCalib);

    mpCamera = mpAtlas->AddCamera(mpCamera);

    std::cout << "- Camera: Pinhole" << std::endl;
    std::cout << "- Image scale: " << mImageScale << std::endl;
    std::cout << "- fx: " << fx << std::endl;
    std::cout << "- fy: " << fy << std::endl;
    std::cout << "- cx: " << cx << std::endl;
    std::cout << "- cy: " << cy << std::endl;
    std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
    std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;

    std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
    std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

    if (mDistCoef.rows == 5)
      std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

    mK = cv::Mat::eye(3, 3, CV_32F);
    mK.at<float>(0, 0) = fx;
    mK.at<float>(1, 1) = fy;
    mK.at<float>(0, 2) = cx;
    mK.at<float>(1, 2) = cy;

    mK_.setIdentity();
    mK_(0, 0) = fx;
    mK_(1, 1) = fy;
    mK_(0, 2) = cx;
    mK_(1, 2) = cy;
  } else if (sCameraName == "KannalaBrandt8") {
    float fx, fy, cx, cy;
    float k1, k2, k3, k4;
    mImageScale = 1.f;

    // Camera calibration parameters
    cv::FileNode node = fSettings["Camera.fx"];
    if (!node.empty() && node.isReal()) {
      fx = node.real();
    } else {
      std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }
    node = fSettings["Camera.fy"];
    if (!node.empty() && node.isReal()) {
      fy = node.real();
    } else {
      std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.cx"];
    if (!node.empty() && node.isReal()) {
      cx = node.real();
    } else {
      std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.cy"];
    if (!node.empty() && node.isReal()) {
      cy = node.real();
    } else {
      std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    // Distortion parameters
    node = fSettings["Camera.k1"];
    if (!node.empty() && node.isReal()) {
      k1 = node.real();
    } else {
      std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }
    node = fSettings["Camera.k2"];
    if (!node.empty() && node.isReal()) {
      k2 = node.real();
    } else {
      std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.k3"];
    if (!node.empty() && node.isReal()) {
      k3 = node.real();
    } else {
      std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.k4"];
    if (!node.empty() && node.isReal()) {
      k4 = node.real();
    } else {
      std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }

    node = fSettings["Camera.imageScale"];
    if (!node.empty() && node.isReal()) {
      mImageScale = node.real();
    }

    if (!b_miss_params) {
      if (mImageScale != 1.f) {
        // K matrix parameters must be scaled.
        fx = fx * mImageScale;
        fy = fy * mImageScale;
        cx = cx * mImageScale;
        cy = cy * mImageScale;
      }

      vector<float> vCamCalib{fx, fy, cx, cy, k1, k2, k3, k4};
      mpCamera = new KannalaBrandt8(vCamCalib);
      mpCamera = mpAtlas->AddCamera(mpCamera);
      std::cout << "- Camera: Fisheye" << std::endl;
      std::cout << "- Image scale: " << mImageScale << std::endl;
      std::cout << "- fx: " << fx << std::endl;
      std::cout << "- fy: " << fy << std::endl;
      std::cout << "- cx: " << cx << std::endl;
      std::cout << "- cy: " << cy << std::endl;
      std::cout << "- k1: " << k1 << std::endl;
      std::cout << "- k2: " << k2 << std::endl;
      std::cout << "- k3: " << k3 << std::endl;
      std::cout << "- k4: " << k4 << std::endl;

      mK = cv::Mat::eye(3, 3, CV_32F);
      mK.at<float>(0, 0) = fx;
      mK.at<float>(1, 1) = fy;
      mK.at<float>(0, 2) = cx;
      mK.at<float>(1, 2) = cy;

      mK_.setIdentity();
      mK_(0, 0) = fx;
      mK_(1, 1) = fy;
      mK_(0, 2) = cx;
      mK_(1, 2) = cy;
    }

    if (mSensor == System::STEREO || mSensor == System::IMU_STEREO ||
        mSensor == System::IMU_RGBD) {
      // Right camera
      // Camera calibration parameters
      cv::FileNode node = fSettings["Camera2.fx"];
      if (!node.empty() && node.isReal()) {
        fx = node.real();
      } else {
        std::cerr
            << "*Camera2.fx parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
      }
      node = fSettings["Camera2.fy"];
      if (!node.empty() && node.isReal()) {
        fy = node.real();
      } else {
        std::cerr
            << "*Camera2.fy parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
      }

      node = fSettings["Camera2.cx"];
      if (!node.empty() && node.isReal()) {
        cx = node.real();
      } else {
        std::cerr
            << "*Camera2.cx parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
      }

      node = fSettings["Camera2.cy"];
      if (!node.empty() && node.isReal()) {
        cy = node.real();
      } else {
        std::cerr
            << "*Camera2.cy parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
      }

      // Distortion parameters
      node = fSettings["Camera2.k1"];
      if (!node.empty() && node.isReal()) {
        k1 = node.real();
      } else {
        std::cerr
            << "*Camera2.k1 parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
      }
      node = fSettings["Camera2.k2"];
      if (!node.empty() && node.isReal()) {
        k2 = node.real();
      } else {
        std::cerr
            << "*Camera2.k2 parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
      }

      node = fSettings["Camera2.k3"];
      if (!node.empty() && node.isReal()) {
        k3 = node.real();
      } else {
        std::cerr
            << "*Camera2.k3 parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
      }

      node = fSettings["Camera2.k4"];
      if (!node.empty() && node.isReal()) {
        k4 = node.real();
      } else {
        std::cerr
            << "*Camera2.k4 parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
      }

      int leftLappingBegin = -1;
      int leftLappingEnd = -1;

      int rightLappingBegin = -1;
      int rightLappingEnd = -1;

      node = fSettings["Camera.lappingBegin"];
      if (!node.empty() && node.isInt()) {
        leftLappingBegin = node.operator int();
      } else {
        std::cout << "WARNING: Camera.lappingBegin not correctly defined"
                  << std::endl;
      }
      node = fSettings["Camera.lappingEnd"];
      if (!node.empty() && node.isInt()) {
        leftLappingEnd = node.operator int();
      } else {
        std::cout << "WARNING: Camera.lappingEnd not correctly defined"
                  << std::endl;
      }
      node = fSettings["Camera2.lappingBegin"];
      if (!node.empty() && node.isInt()) {
        rightLappingBegin = node.operator int();
      } else {
        std::cout << "WARNING: Camera2.lappingBegin not correctly defined"
                  << std::endl;
      }
      node = fSettings["Camera2.lappingEnd"];
      if (!node.empty() && node.isInt()) {
        rightLappingEnd = node.operator int();
      } else {
        std::cout << "WARNING: Camera2.lappingEnd not correctly defined"
                  << std::endl;
      }

      node = fSettings["Tlr"];
      cv::Mat cvTlr;
      if (!node.empty()) {
        cvTlr = node.mat();
        if (cvTlr.rows != 3 || cvTlr.cols != 4) {
          std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*"
                    << std::endl;
          b_miss_params = true;
        }
      } else {
        std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
        b_miss_params = true;
      }

      if (!b_miss_params) {
        if (mImageScale != 1.f) {
          // K matrix parameters must be scaled.
          fx = fx * mImageScale;
          fy = fy * mImageScale;
          cx = cx * mImageScale;
          cy = cy * mImageScale;

          leftLappingBegin = leftLappingBegin * mImageScale;
          leftLappingEnd = leftLappingEnd * mImageScale;
          rightLappingBegin = rightLappingBegin * mImageScale;
          rightLappingEnd = rightLappingEnd * mImageScale;
        }

        static_cast<KannalaBrandt8 *>(mpCamera)->mvLappingArea[0] =
            leftLappingBegin;
        static_cast<KannalaBrandt8 *>(mpCamera)->mvLappingArea[1] =
            leftLappingEnd;

        // mpFrameDrawer->both = true;

        vector<float> vCamCalib2{fx, fy, cx, cy, k1, k2, k3, k4};
        mpCamera2 = new KannalaBrandt8(vCamCalib2);
        mpCamera2 = mpAtlas->AddCamera(mpCamera2);

        mTlr = Converter::toSophus(cvTlr);

        static_cast<KannalaBrandt8 *>(mpCamera2)->mvLappingArea[0] =
            rightLappingBegin;
        static_cast<KannalaBrandt8 *>(mpCamera2)->mvLappingArea[1] =
            rightLappingEnd;

        std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", "
                  << leftLappingEnd << std::endl;

        std::cout << std::endl << "Camera2 Parameters:" << std::endl;
        std::cout << "- Camera: Fisheye" << std::endl;
        std::cout << "- Image scale: " << mImageScale << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << k1 << std::endl;
        std::cout << "- k2: " << k2 << std::endl;
        std::cout << "- k3: " << k3 << std::endl;
        std::cout << "- k4: " << k4 << std::endl;

        std::cout << "- mTlr: \n" << cvTlr << std::endl;

        std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", "
                  << rightLappingEnd << std::endl;
      }
    }

    if (b_miss_params) {
      return false;
    }

  } else {
    std::cerr << "*Not Supported Camera Sensor*" << std::endl;
    std::cerr << "Check an example configuration file with the desired sensor"
              << std::endl;
  }

  if (mSensor == System::STEREO || mSensor == System::RGBD ||
      mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
    cv::FileNode node = fSettings["Camera.bf"];
    if (!node.empty() && node.isReal()) {
      mbf = node.real();
      if (mImageScale != 1.f) {
        mbf *= mImageScale;
      }
    } else {
      std::cerr << "*Camera.bf parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }
  }

  float fps = fSettings["Camera.fps"];
  if (fps == 0)
    fps = 30;

  // Max/Min Frames to insert keyframes and to check relocalisation
  mMinFrames = 0;
  mMaxFrames = fps;

  cout << "- fps: " << fps << endl;

  int nRGB = fSettings["Camera.RGB"];
  mbRGB = nRGB;

  if (mbRGB)
    cout << "- color order: RGB (ignored if grayscale)" << endl;
  else
    cout << "- color order: BGR (ignored if grayscale)" << endl;

  if (mSensor == System::STEREO || mSensor == System::RGBD ||
      mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
    float fx = mpCamera->getParameter(0);
    cv::FileNode node = fSettings["ThDepth"];
    if (!node.empty() && node.isReal()) {
      mThDepth = node.real();
      mThDepth = mbf * mThDepth / fx;
      cout << endl
           << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    } else {
      std::cerr << "*ThDepth parameter doesn't exist or is not a real number*"
                << std::endl;
      b_miss_params = true;
    }
  }

  if (mSensor == System::RGBD || mSensor == System::IMU_RGBD) {
    cv::FileNode node = fSettings["DepthMapFactor"];
    if (!node.empty() && node.isReal()) {
      mDepthMapFactor = node.real();
      if (fabs(mDepthMapFactor) < 1e-5)
        mDepthMapFactor = 1;
      else
        mDepthMapFactor = 1.0f / mDepthMapFactor;
    } else {
      std::cerr
          << "*DepthMapFactor parameter doesn't exist or is not a real number*"
          << std::endl;
      b_miss_params = true;
    }
  }

  if (b_miss_params) {
    return false;
  }

  return true;
}

bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings) {
  bool b_miss_params = false;
  int nFeatures, nLevels, fIniThFAST, fMinThFAST;
  float fScaleFactor;

  cv::FileNode node = fSettings["ORBextractor.nFeatures"];
  if (!node.empty() && node.isInt()) {
    nFeatures = node.operator int();
  } else {
    std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an "
                 "integer*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["ORBextractor.scaleFactor"];
  if (!node.empty() && node.isReal()) {
    fScaleFactor = node.real();
  } else {
    std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not "
                 "a real number*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["ORBextractor.nLevels"];
  if (!node.empty() && node.isInt()) {
    nLevels = node.operator int();
  } else {
    std::cerr
        << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["ORBextractor.iniThFAST"];
  if (!node.empty() && node.isInt()) {
    fIniThFAST = node.operator int();
  } else {
    std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an "
                 "integer*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["ORBextractor.minThFAST"];
  if (!node.empty() && node.isInt()) {
    fMinThFAST = node.operator int();
  } else {
    std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an "
                 "integer*"
              << std::endl;
    b_miss_params = true;
  }

  if (b_miss_params) {
    return false;
  }

  mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels,
                                        fIniThFAST, fMinThFAST);

  if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
    mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels,
                                           fIniThFAST, fMinThFAST);

  if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    mpIniORBextractor = new ORBextractor(5 * nFeatures, fScaleFactor, nLevels,
                                         fIniThFAST, fMinThFAST);

  cout << endl << "ORB Extractor Parameters: " << endl;
  cout << "- Number of Features: " << nFeatures << endl;
  cout << "- Scale Levels: " << nLevels << endl;
  cout << "- Scale Factor: " << fScaleFactor << endl;
  cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
  cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

  return true;
}

bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings) {
  bool b_miss_params = false;

  cv::Mat cvTbc;
  cv::FileNode node = fSettings["Tbc"];
  if (!node.empty()) {
    cvTbc = node.mat();
    if (cvTbc.rows != 4 || cvTbc.cols != 4) {
      std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*"
                << std::endl;
      b_miss_params = true;
    }
  } else {
    std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
    b_miss_params = true;
  }
  cout << endl;
  cout << "Left camera to Imu Transform (Tbc): " << endl << cvTbc << endl;
  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> eigTbc(cvTbc.ptr<float>(0));
  Sophus::SE3f Tbc(eigTbc);

  node = fSettings["InsertKFsWhenLost"];
  mInsertKFsLost = true;
  if (!node.empty() && node.isInt()) {
    mInsertKFsLost = (bool)node.operator int();
  }

  if (!mInsertKFsLost)
    cout << "Do not insert keyframes when lost visual tracking " << endl;

  float Ng, Na, Ngw, Naw;

  node = fSettings["IMU.Frequency"];
  if (!node.empty() && node.isInt()) {
    mImuFreq = node.operator int();
    mImuPer = (mImuFreq > 0.0f) ? (1.0 / static_cast<double>(mImuFreq)) : 0.001;
  } else {
    std::cerr << "*IMU.Frequency parameter doesn't exist or is not an integer*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["IMU.NoiseGyro"];
  if (!node.empty() && node.isReal()) {
    Ng = node.real();
  } else {
    std::cerr
        << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["IMU.NoiseAcc"];
  if (!node.empty() && node.isReal()) {
    Na = node.real();
  } else {
    std::cerr
        << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["IMU.GyroWalk"];
  if (!node.empty() && node.isReal()) {
    Ngw = node.real();
  } else {
    std::cerr
        << "*IMU.GyroWalk parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["IMU.AccWalk"];
  if (!node.empty() && node.isReal()) {
    Naw = node.real();
  } else {
    std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*"
              << std::endl;
    b_miss_params = true;
  }

  node = fSettings["IMU.fastInit"];
  mFastInit = false;
  if (!node.empty()) {
    mFastInit = static_cast<int>(fSettings["IMU.fastInit"]) != 0;
  }

  if (mFastInit)
    cout << "Fast IMU initialization. Acceleration is not checked \n";

  if (b_miss_params) {
    return false;
  }

  const float sf = sqrt(mImuFreq);
  cout << endl;
  cout << "IMU frequency: " << mImuFreq << " Hz" << endl;
  cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
  cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
  cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
  cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

  mpImuCalib = new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

  mpImuPreintegratedFromLastKF =
      new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);

  return true;
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper) {
  mpLocalMappingBackend = CreateDefaultOrbLocalMappingBackend(pLocalMapper);
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing) {
  mpLoopClosingBackend = CreateDefaultOrbLoopClosingBackend(pLoopClosing);
}

void Tracking::SetStepByStep(bool bSet) { bStepByStep = bSet; }

bool Tracking::GetStepByStep() { return bStepByStep; }

OrbFrameSensorContext Tracking::BuildFrameSensorContext(
    Frame *previousFrame, const IMU::Calib *imuCalibration) const {
  OrbFrameSensorContext sensor;
  sensor.vocabulary = mpORBVocabulary;
  sensor.leftExtractor = mpORBextractorLeft;
  sensor.rightExtractor = mpORBextractorRight;
  sensor.initExtractor = mpIniORBextractor;
  sensor.leftCamera = mpCamera;
  sensor.rightCamera = mpCamera2;
  sensor.calibration = const_cast<cv::Mat *>(&mK);
  sensor.distortion = const_cast<cv::Mat *>(&mDistCoef);
  sensor.baselineFx = mbf;
  sensor.closeDepthThreshold = mThDepth;
  sensor.leftToRightPose = const_cast<Sophus::SE3f *>(&mTlr);
  sensor.previousFrame = previousFrame;
  sensor.imuCalibration = imuCalibration;
  return sensor;
}

bool Tracking::NeedsInitExtractor() const {
  return mState == NOT_INITIALIZED || mState == NO_IMAGES_YET ||
         (lastID - initID) < mMaxFrames;
}

Sophus::SE3f Tracking::GrabImageStereo(const cv::Mat &imRectLeft,
                                       const cv::Mat &imRectRight,
                                       const double &timestamp,
                                       string filename) {
  mImGray = imRectLeft;
  cv::Mat imGrayRight = imRectRight;
  mImRight = imRectRight;

  if (mImGray.channels() == 3) {
    if (mbRGB) {
      cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
      cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
    } else {
      cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
      cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
    }
  } else if (mImGray.channels() == 4) {
    if (mbRGB) {
      cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
      cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGBA2GRAY);
    } else {
      cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
      cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
    }
  }

  Frame *previousFrame =
      mSensor == System::IMU_STEREO ? &mLastFrame : nullptr;
  const IMU::Calib *imuCalibration =
      mSensor == System::IMU_STEREO ? mpImuCalib : nullptr;
  mCurrentFrame = mpFrameFactory->CreateStereoFrame(
      OrbStereoFrameCreateRequest{
          &mImGray, &imGrayRight, timestamp,
          BuildFrameSensorContext(previousFrame, imuCalibration)});

  mCurrentFrame.mNameFile = filename;
  mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
  vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
  vdStereoMatch_ms.push_back(mCurrentFrame.mTimeStereoMatch);
#endif

  Track();

  return mCurrentFrame.GetPose();
}

Sophus::SE3f Tracking::GrabImageStereoWithFeatures(
    const cv::Mat &imRectLeft, const cv::Mat &imRectRight,
    const StereoFeatureFrameData &features, const double &timestamp,
    string filename) {
  mImGray = imRectLeft;
  cv::Mat imGrayRight = imRectRight;
  mImRight = imRectRight;

  if (mImGray.channels() == 3) {
    if (mbRGB) {
      cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
      cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
    } else {
      cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
      cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
    }
  } else if (mImGray.channels() == 4) {
    if (mbRGB) {
      cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
      cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGBA2GRAY);
    } else {
      cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
      cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
    }
  }

  if (mpCamera2) {
    cerr << "ERROR: external stereo feature injection currently supports "
            "rectified stereo without mpCamera2 only."
         << endl;
    return Sophus::SE3f();
  }
  Frame *previousFrame =
      mSensor == System::IMU_STEREO ? &mLastFrame : nullptr;
  const IMU::Calib *imuCalibration =
      mSensor == System::IMU_STEREO ? mpImuCalib : nullptr;
  mCurrentFrame = mpFrameFactory->CreateStereoFeatureFrame(
      OrbStereoFeatureFrameCreateRequest{
          &mImGray, &imGrayRight, &features, timestamp,
          BuildFrameSensorContext(previousFrame, imuCalibration)});

  mCurrentFrame.mNameFile = filename;
  mCurrentFrame.mnDataset = mnNumDataset;

  Track();
  return mCurrentFrame.GetPose();
}

Sophus::SE3f Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD,
                                     const double &timestamp, string filename) {
  mImGray = imRGB;
  cv::Mat imDepth = imD;

  if (mImGray.channels() == 3) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
    else
      cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
  } else if (mImGray.channels() == 4) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
    else
      cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
  }

  if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
    imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

  Frame *previousFrame = mSensor == System::IMU_RGBD ? &mLastFrame : nullptr;
  const IMU::Calib *imuCalibration =
      mSensor == System::IMU_RGBD ? mpImuCalib : nullptr;
  mCurrentFrame = mpFrameFactory->CreateRgbdFrame(
      OrbRgbdFrameCreateRequest{
          &mImGray, &imDepth, timestamp,
          BuildFrameSensorContext(previousFrame, imuCalibration)});

  mCurrentFrame.mNameFile = filename;
  mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
  vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

  Track();

  return mCurrentFrame.GetPose();
}

Sophus::SE3f Tracking::GrabImageMonocular(const cv::Mat &im,
                                          const double &timestamp,
                                          string filename) {
  mImGray = im;
  if (mImGray.channels() == 3) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
    else
      cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
  } else if (mImGray.channels() == 4) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
    else
      cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
  }

  Frame *previousFrame =
      mSensor == System::IMU_MONOCULAR ? &mLastFrame : nullptr;
  const IMU::Calib *imuCalibration =
      mSensor == System::IMU_MONOCULAR ? mpImuCalib : nullptr;
  const bool useInitExtractor =
      mSensor == System::MONOCULAR
          ? NeedsInitExtractor()
          : (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET);
  mCurrentFrame = mpFrameFactory->CreateMonocularFrame(
      OrbMonocularFrameCreateRequest{
          &mImGray, timestamp, useInitExtractor,
          BuildFrameSensorContext(previousFrame, imuCalibration)});

  if (mState == NO_IMAGES_YET)
    t0 = timestamp;

  mCurrentFrame.mNameFile = filename;
  mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
  vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

  lastID = mCurrentFrame.mnId;
  Track();

  return mCurrentFrame.GetPose();
}

Sophus::SE3f Tracking::GrabImageMonocularWithFeatures(
    const cv::Mat &im, const MonoFeatureFrameData &features,
    const double &timestamp, string filename) {
  mImGray = im;
  if (mImGray.channels() == 3) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
    else
      cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
  } else if (mImGray.channels() == 4) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
    else
      cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
  }

  if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR) {
    cerr << "ERROR: external monocular feature injection requires monocular "
            "input sensor."
         << endl;
    return Sophus::SE3f();
  }
  Frame *previousFrame =
      mSensor == System::IMU_MONOCULAR ? &mLastFrame : nullptr;
  const IMU::Calib *imuCalibration =
      mSensor == System::IMU_MONOCULAR ? mpImuCalib : nullptr;
  const bool useInitExtractor =
      mSensor == System::MONOCULAR
          ? NeedsInitExtractor()
          : (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET);
  mCurrentFrame = mpFrameFactory->CreateMonocularFeatureFrame(
      OrbMonocularFeatureFrameCreateRequest{
          &mImGray, &features, timestamp, useInitExtractor,
          BuildFrameSensorContext(previousFrame, imuCalibration)});

  if (mState == NO_IMAGES_YET)
    t0 = timestamp;

  mCurrentFrame.mNameFile = filename;
  mCurrentFrame.mnDataset = mnNumDataset;

  lastID = mCurrentFrame.mnId;
  Track();
  return mCurrentFrame.GetPose();
}

void Tracking::GrabImuData(const IMU::Point &imuMeasurement) {
  mlQueueImuData.push_back(imuMeasurement);
}

void Tracking::PreintegrateIMU() {

  if (!mCurrentFrame.mpPrevFrame) {
    Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
    mCurrentFrame.setIntegrated();
    return;
  }

  mvImuFromLastFrame.clear();
  mvImuFromLastFrame.reserve(mlQueueImuData.size());
  if (mlQueueImuData.size() == 0) {
    Verbose::PrintMess("Not IMU data in mlQueueImuData!!",
                       Verbose::VERBOSITY_NORMAL);
    mCurrentFrame.setIntegrated();
    return;
  }

  while (true) {
    if (mlQueueImuData.empty()) {
      break;
    }
    IMU::Point *m = &mlQueueImuData.front();
    cout.precision(17);
    if (m->t < mCurrentFrame.mpPrevFrame->mTimeStamp - mImuPer) {
      mlQueueImuData.pop_front();
    } else if (m->t < mCurrentFrame.mTimeStamp - mImuPer) {
      mvImuFromLastFrame.push_back(*m);
      mlQueueImuData.pop_front();
    } else {
      mvImuFromLastFrame.push_back(*m);
      break;
    }
  }

  const int n = mvImuFromLastFrame.size() - 1;
  if (n == 0) {
    cout << "Empty IMU measurements vector!!!\n";
    return;
  }

  IMU::Preintegrated *pImuPreintegratedFromLastFrame =
      new IMU::Preintegrated(mLastFrame.mImuBias, mCurrentFrame.mImuCalib);

  for (int i = 0; i < n; i++) {
    float tstep;
    Eigen::Vector3f acc, angVel;
    if ((i == 0) && (i < (n - 1))) {
      float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
      float tini =
          mvImuFromLastFrame[i].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
      acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a -
             (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) *
                 (tini / tab)) *
            0.5f;
      angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w -
                (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) *
                    (tini / tab)) *
               0.5f;
      tstep =
          mvImuFromLastFrame[i + 1].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
    } else if (i < (n - 1)) {
      acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a) * 0.5f;
      angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w) * 0.5f;
      tstep = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
    } else if ((i > 0) && (i == (n - 1))) {
      float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
      float tend = mvImuFromLastFrame[i + 1].t - mCurrentFrame.mTimeStamp;
      acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a -
             (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) *
                 (tend / tab)) *
            0.5f;
      angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w -
                (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) *
                    (tend / tab)) *
               0.5f;
      tstep = mCurrentFrame.mTimeStamp - mvImuFromLastFrame[i].t;
    } else if ((i == 0) && (i == (n - 1))) {
      acc = mvImuFromLastFrame[i].a;
      angVel = mvImuFromLastFrame[i].w;
      tstep = mCurrentFrame.mTimeStamp - mCurrentFrame.mpPrevFrame->mTimeStamp;
    }

    if (!mpImuPreintegratedFromLastKF)
      cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
    mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc, angVel, tstep);
    pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc, angVel, tstep);
  }

  mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
  mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
  mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;

  mCurrentFrame.setIntegrated();

  // Verbose::PrintMess("Preintegration is finished!! ",
  // Verbose::VERBOSITY_DEBUG);
}

bool Tracking::PredictStateIMU() {
  if (!mCurrentFrame.mpPrevFrame) {
    Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
    return false;
  }

  if (mbMapUpdated && mpLastKeyFrame) {
    const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
    const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
    const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

    const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
    const float t12 = mpImuPreintegratedFromLastKF->dT;

    Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
        Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(
                   mpLastKeyFrame->GetImuBias()));
    Eigen::Vector3f twb2 =
        twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
        Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaPosition(
                   mpLastKeyFrame->GetImuBias());
    Eigen::Vector3f Vwb2 =
        Vwb1 + t12 * Gz +
        Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(
                   mpLastKeyFrame->GetImuBias());
    mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

    mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias();
    mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
    return true;
  } else if (!mbMapUpdated) {
    const Eigen::Vector3f twb1 = mLastFrame.GetImuPosition();
    const Eigen::Matrix3f Rwb1 = mLastFrame.GetImuRotation();
    const Eigen::Vector3f Vwb1 = mLastFrame.GetVelocity();
    const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
    const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

    Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
        Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(
                   mLastFrame.mImuBias));
    Eigen::Vector3f twb2 =
        twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
        Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(
                   mLastFrame.mImuBias);
    Eigen::Vector3f Vwb2 =
        Vwb1 + t12 * Gz +
        Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(
                   mLastFrame.mImuBias);

    mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

    mCurrentFrame.mImuBias = mLastFrame.mImuBias;
    mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
    return true;
  } else
    cout << "not IMU prediction!!" << endl;

  return false;
}

void Tracking::ResetFrameIMU() {
  // TODO To implement...
}

void Tracking::Track() {
  if (bStepByStep) {
    if (!mbStep) {
      std::cout << "Tracking: Waiting to the next step" << std::endl;
      return;
    }
    mbStep = false;
  }

  const OrbLocalMappingStatus localMappingStatus =
      GetLocalMappingStatus(mpLocalMappingBackend.get());
  if (localMappingStatus.badImu) {
    cout << "TRACK: Reset map because local mapper set the bad imu flag "
         << endl;
    mpSystem->ResetActiveMap();
    return;
  }

  Map *pCurrentMap = mpAtlas->GetCurrentMap();
  if (!pCurrentMap) {
    cout << "ERROR: There is not an active map in the atlas" << endl;
  }

  if (mState != NO_IMAGES_YET) {
    if (mLastFrame.mTimeStamp > mCurrentFrame.mTimeStamp) {
      cerr
          << "ERROR: Frame with a timestamp older than previous frame detected!"
          << endl;
      mlQueueImuData.clear();
      CreateMapInAtlas();
      return;
    } else if (mCurrentFrame.mTimeStamp > mLastFrame.mTimeStamp + 1.0) {
      // cout << mCurrentFrame.mTimeStamp << ", " << mLastFrame.mTimeStamp <<
      // endl; cout << "id last: " << mLastFrame.mnId << "    id curr: " <<
      // mCurrentFrame.mnId << endl;
      if (mpAtlas->isInertial()) {

        if (mpAtlas->isImuInitialized()) {
          cout << "Timestamp jump detected. State set to LOST. Reseting IMU "
                  "integration..."
               << endl;
          if (!pCurrentMap->GetIniertialBA2()) {
            mpSystem->ResetActiveMap();
          } else {
            CreateMapInAtlas();
          }
        } else {
          cout << "Timestamp jump detected, before IMU initialization. "
                  "Reseting..."
               << endl;
          mpSystem->ResetActiveMap();
        }
        return;
      }
    }
  }

  if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
       mSensor == System::IMU_RGBD) &&
      mpLastKeyFrame)
    mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

  if (mState == NO_IMAGES_YET) {
    mState = NOT_INITIALIZED;
  }

  mLastProcessedState = mState;

  if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
       mSensor == System::IMU_RGBD) &&
      !mbCreatedMap) {
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartPreIMU =
        std::chrono::steady_clock::now();
#endif
    PreintegrateIMU();
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndPreIMU =
        std::chrono::steady_clock::now();

    double timePreImu =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndPreIMU - time_StartPreIMU)
            .count();
    vdIMUInteg_ms.push_back(timePreImu);
#endif
  }
  mbCreatedMap = false;

  // EPG serializes map updates through the slam_backend resource.

  mbMapUpdated = false;

  int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
  int nMapChangeIndex = pCurrentMap->GetLastMapChange();
  if (nCurMapChangeIndex > nMapChangeIndex) {
    pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
    mbMapUpdated = true;
  }

  if (mState == NOT_INITIALIZED) {
    if (mSensor == System::STEREO || mSensor == System::RGBD ||
        mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
      StereoInitialization();
    } else {
      MonocularInitialization();
    }

    // mpFrameDrawer->Update(this);

    if (mState != OK) // If rightly initialized, mState=OK
    {
      mLastFrame = Frame(mCurrentFrame);
      return;
    }

    if (mpAtlas->GetAllMaps().size() == 1) {
      mnFirstFrameId = mCurrentFrame.mnId;
    }
  } else {
    // System is initialized. Track Frame.
    bool bOK;

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartPosePred =
        std::chrono::steady_clock::now();
#endif

    // Initial camera pose estimation using motion model or relocalization (if
    // tracking is lost)
    if (!mbOnlyTracking) {

      // State OK
      // Local Mapping is activated. This is the normal behaviour, unless
      // you explicitly activate the "only tracking" mode.
      if (mState == OK) {

        // Local Mapping might have changed some MapPoints tracked in last frame
        CheckReplacedInLastFrame();

        if ((!mbVelocity && !pCurrentMap->isImuInitialized()) ||
            mCurrentFrame.mnId < mnLastRelocFrameId + 2) {
          Verbose::PrintMess("TRACK: Track with respect to the reference KF ",
                             Verbose::VERBOSITY_DEBUG);
          bOK = TrackReferenceKeyFrame();
        } else {
          Verbose::PrintMess("TRACK: Track with motion model",
                             Verbose::VERBOSITY_DEBUG);
          bOK = TrackWithMotionModel();
          if (!bOK)
            bOK = TrackReferenceKeyFrame();
        }

        if (!bOK) {
          const bool stereoFeatureStabilizing = IsStereoFeatureStabilizing(
              mCurrentFrame, mpAtlas, mnStereoFeatureInitFrameId);
          if (stereoFeatureStabilizing && StereoFeaturePoseRescueEnabled() &&
              IsStereoFeatureObservationHealthy(mCurrentFrame)) {
            mCurrentFrame.SetPose(mLastFrame.GetPose());
            bOK = true;
            LogStereoFeaturePoseRescue(
                "ok_pose_seed_last_frame", mCurrentFrame, mpAtlas, mState,
                mnStereoFeatureInitFrameId, mnMatchesInliers);
          }
        }

        if (!bOK) {
          if (mCurrentFrame.mnId <= (mnLastRelocFrameId + mnFramesToResetIMU) &&
              (mSensor == System::IMU_MONOCULAR ||
               mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)) {
            mState = LOST;
          } else if (mCurrentFrame.mbStereoFeatureInjected &&
                     pCurrentMap->KeyFramesInMap() > 1) {
            mState = RECENTLY_LOST;
            mTimeStampLost = mCurrentFrame.mTimeStamp;
          } else if (pCurrentMap->KeyFramesInMap() > 10) {
            // cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
            mState = RECENTLY_LOST;
            mTimeStampLost = mCurrentFrame.mTimeStamp;
          } else {
            mState = LOST;
          }
        }
      } else {

        if (mState == RECENTLY_LOST) {
          Verbose::PrintMess("Lost for a short time",
                             Verbose::VERBOSITY_NORMAL);
          const bool stereoFeatureStabilizing = IsStereoFeatureStabilizing(
              mCurrentFrame, mpAtlas, mnStereoFeatureInitFrameId);

          bOK = true;
          if ((mSensor == System::IMU_MONOCULAR ||
               mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)) {
            if (pCurrentMap->isImuInitialized())
              PredictStateIMU();
            else
              bOK = false;

            if (mCurrentFrame.mTimeStamp - mTimeStampLost >
                time_recently_lost) {
              mState = LOST;
              Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
              bOK = false;
            }
          } else {
            if (stereoFeatureStabilizing) {
              CheckReplacedInLastFrame();
              if (mbVelocity)
                bOK = TrackWithMotionModel();
              if (!bOK)
                bOK = TrackReferenceKeyFrame();
              if (!bOK && StereoFeaturePoseRescueEnabled() &&
                  IsStereoFeatureObservationHealthy(mCurrentFrame)) {
                mCurrentFrame.SetPose(mLastFrame.GetPose());
                bOK = true;
                LogStereoFeaturePoseRescue("recently_lost_pose_seed_last_frame",
                                           mCurrentFrame, mpAtlas, mState,
                                           mnStereoFeatureInitFrameId,
                                           mnMatchesInliers);
              }
            } else {
              // Relocalization
              bOK = Relocalization();
            }
            // std::cout << "mCurrentFrame.mTimeStamp:" <<
            // to_string(mCurrentFrame.mTimeStamp) << std::endl; std::cout <<
            // "mTimeStampLost:" << to_string(mTimeStampLost) << std::endl;
            if (mCurrentFrame.mTimeStamp - mTimeStampLost > 3.0f && !bOK) {
              mState = LOST;
              Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
              bOK = false;
            }
          }
        } else if (mState == LOST) {

          Verbose::PrintMess("A new map is started...",
                             Verbose::VERBOSITY_NORMAL);

          if (pCurrentMap->KeyFramesInMap() < 10) {
            mpSystem->ResetActiveMap();
            Verbose::PrintMess("Reseting current map...",
                               Verbose::VERBOSITY_NORMAL);
          } else
            CreateMapInAtlas();

          if (mpLastKeyFrame)
            mpLastKeyFrame = static_cast<KeyFrame *>(NULL);

          Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

          return;
        }
      }

    } else {
      // Localization Mode: Local Mapping is deactivated (TODO Not available in
      // inertial mode)
      if (mState == LOST) {
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
            mSensor == System::IMU_RGBD)
          Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
        bOK = Relocalization();
      } else {
        if (!mbVO) {
          // In last frame we tracked enough MapPoints in the map
          if (mbVelocity) {
            bOK = TrackWithMotionModel();
          } else {
            bOK = TrackReferenceKeyFrame();
          }
        } else {
          // In last frame we tracked mainly "visual odometry" points.

          // We compute two camera poses, one from motion model and one doing
          // relocalization. If relocalization is sucessfull we choose that
          // solution, otherwise we retain the "visual odometry" solution.

          bool bOKMM = false;
          bool bOKReloc = false;
          vector<MapPoint *> vpMPsMM;
          vector<bool> vbOutMM;
          Sophus::SE3f TcwMM;
          if (mbVelocity) {
            bOKMM = TrackWithMotionModel();
            vpMPsMM = mCurrentFrame.mvpMapPoints;
            vbOutMM = mCurrentFrame.mvbOutlier;
            TcwMM = mCurrentFrame.GetPose();
          }
          bOKReloc = Relocalization();

          if (bOKMM && !bOKReloc) {
            mCurrentFrame.SetPose(TcwMM);
            mCurrentFrame.mvpMapPoints = vpMPsMM;
            mCurrentFrame.mvbOutlier = vbOutMM;

            if (mbVO) {
              for (int i = 0; i < mCurrentFrame.N; i++) {
                if (mCurrentFrame.mvpMapPoints[i] &&
                    !mCurrentFrame.mvbOutlier[i]) {
                  mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                }
              }
            }
          } else if (bOKReloc) {
            mbVO = false;
          }

          bOK = bOKReloc || bOKMM;
        }
      }
    }

    if (!mCurrentFrame.mpReferenceKF)
      mCurrentFrame.mpReferenceKF = mpReferenceKF;

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndPosePred =
        std::chrono::steady_clock::now();

    double timePosePred =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndPosePred - time_StartPosePred)
            .count();
    vdPosePred_ms.push_back(timePosePred);
#endif

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartLMTrack =
        std::chrono::steady_clock::now();
#endif
    // If we have an initial estimation of the camera pose and matching. Track
    // the local map.
    if (!mbOnlyTracking) {
      const bool posePredictionOk = bOK;
      bool localMapOk = false;
      if (bOK) {
        bOK = TrackLocalMap();
        localMapOk = bOK;
      }
      if (StereoFeatureTrackDfxEnabled() &&
          mCurrentFrame.mbStereoFeatureInjected) {
        cerr << "[stereo_feature_track_state] frame=" << mCurrentFrame.mnId
             << " state=" << mState
             << " pose_ok=" << (posePredictionOk ? "Y" : "N")
             << " local_ok=" << (localMapOk ? "Y" : "N")
             << " final_ok=" << (bOK ? "Y" : "N")
             << " features=" << StereoFeatureLeftFeatureCount(mCurrentFrame)
             << " close=" << mCurrentFrame.mnCloseMPs
             << " tracked=" << CountTrackedMapPoints(mCurrentFrame)
             << " inliers=" << mnMatchesInliers
             << " kfs=" << (mpAtlas ? mpAtlas->KeyFramesInMap() : 0)
             << " init_frame=" << mnStereoFeatureInitFrameId << " stabilizing="
             << (IsStereoFeatureStabilizing(mCurrentFrame, mpAtlas,
                                            mnStereoFeatureInitFrameId)
                     ? "Y"
                     : "N")
             << " bootstrap="
             << (IsStereoFeatureBootstrap(mCurrentFrame, mpAtlas,
                                          mnStereoFeatureInitFrameId)
                     ? "Y"
                     : "N")
             << "\n";
      }
      if (!bOK)
        cout << "Fail to track local map!" << endl;
    } else {
      // mbVO true means that there are few matches to MapPoints in the map. We
      // cannot retrieve a local map and therefore we do not perform
      // TrackLocalMap(). Once the system relocalizes the camera we will use the
      // local map again.
      if (bOK && !mbVO)
        bOK = TrackLocalMap();
    }

    if (bOK) {
      mState = OK;
    } else if (mState == OK) {
      if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
          mSensor == System::IMU_RGBD) {
        Verbose::PrintMess("Track lost for less than one second...",
                           Verbose::VERBOSITY_NORMAL);
        if (!pCurrentMap->isImuInitialized() ||
            !pCurrentMap->GetIniertialBA2()) {
          cout << "IMU is not or recently initialized. Reseting active map..."
               << endl;
          mpSystem->ResetActiveMap();
        }

        mState = RECENTLY_LOST;
      } else
        mState = RECENTLY_LOST; // visual to lost

      mTimeStampLost = mCurrentFrame.mTimeStamp;
    } else if (!bOK && mState == RECENTLY_LOST) {
      if (IsStereoFeatureRecoveryHopeless(mCurrentFrame, mpAtlas,
                                          mnStereoFeatureInitFrameId,
                                          mnMatchesInliers)) {
        mState = LOST;
      }
    }

    // Save frame if recent relocalization, since they are used for IMU reset
    // (as we are making copy, it shluld be once mCurrFrame is completely
    // modified)
    if ((mCurrentFrame.mnId < (mnLastRelocFrameId + mnFramesToResetIMU)) &&
        (mCurrentFrame.mnId > mnFramesToResetIMU) &&
        (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
         mSensor == System::IMU_RGBD) &&
        pCurrentMap->isImuInitialized()) {
      // TODO check this situation
      Verbose::PrintMess("Saving pointer to frame. imu needs reset...",
                         Verbose::VERBOSITY_NORMAL);
      Frame *pF = new Frame(mCurrentFrame);
      pF->mpPrevFrame = new Frame(mLastFrame);

      // Load preintegration
      pF->mpImuPreintegratedFrame =
          new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
    }

    if (pCurrentMap->isImuInitialized()) {
      if (bOK) {
        if (mCurrentFrame.mnId == (mnLastRelocFrameId + mnFramesToResetIMU)) {
          cout << "RESETING FRAME!!!" << endl;
          ResetFrameIMU();
        } else if (mCurrentFrame.mnId > (mnLastRelocFrameId + 30))
          mLastBias = mCurrentFrame.mImuBias;
      }
    }

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndLMTrack =
        std::chrono::steady_clock::now();

    double timeLMTrack =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndLMTrack - time_StartLMTrack)
            .count();
    vdLMTrack_ms.push_back(timeLMTrack);
#endif

    // Update drawer
    // mpFrameDrawer->Update(this);
    // if(mCurrentFrame.isSet())
    //     mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

    if (bOK || mState == RECENTLY_LOST) {
      // Update motion model
      if (mLastFrame.isSet() && mCurrentFrame.isSet()) {
        Sophus::SE3f LastTwc = mLastFrame.GetPose().inverse();
        mVelocity = mCurrentFrame.GetPose() * LastTwc;
        mbVelocity = true;
      } else {
        mbVelocity = false;
      }

      // if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
      // mSensor == System::IMU_RGBD)
      //     mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

      // Clean VO matches
      for (int i = 0; i < mCurrentFrame.N; i++) {
        MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
        if (pMP)
          if (pMP->Observations() < 1) {
            mCurrentFrame.mvbOutlier[i] = false;
            mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
          }
      }

      // Delete temporal MapPoints
      for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(),
                                      lend = mlpTemporalPoints.end();
           lit != lend; lit++) {
        MapPoint *pMP = *lit;
        delete pMP;
      }
      mlpTemporalPoints.clear();

#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point time_StartNewKF =
          std::chrono::steady_clock::now();
#endif
      bool bNeedKF = NeedNewKeyFrame();

      // Check if we need to insert a new keyframe
      // if(bNeedKF && bOK)
      if (bNeedKF && (bOK || (mInsertKFsLost && mState == RECENTLY_LOST &&
                              (mSensor == System::IMU_MONOCULAR ||
                               mSensor == System::IMU_STEREO ||
                               mSensor == System::IMU_RGBD))))
        CreateNewKeyFrame();

#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point time_EndNewKF =
          std::chrono::steady_clock::now();

      double timeNewKF =
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
              time_EndNewKF - time_StartNewKF)
              .count();
      vdNewKF_ms.push_back(timeNewKF);
#endif

      // We allow points with high innovation (considererd outliers by the Huber
      // Function) pass to the new keyframe, so that bundle adjustment will
      // finally decide if they are outliers or not. We don't want next frame to
      // estimate its position with those points so we discard them in the
      // frame. Only has effect if lastframe is tracked
      for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
          mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
      }
    }

    // Reset if the camera get lost soon after initialization
    if (mState == LOST) {
      const bool stereoFeatureStabilizing = IsStereoFeatureStabilizing(
          mCurrentFrame, mpAtlas, mnStereoFeatureInitFrameId);
      const bool stereoFeatureRecoveryHopeless =
          IsStereoFeatureRecoveryHopeless(mCurrentFrame, mpAtlas,
                                          mnStereoFeatureInitFrameId,
                                          mnMatchesInliers);
      if (stereoFeatureStabilizing && !stereoFeatureRecoveryHopeless) {
        mState = RECENTLY_LOST;
        mTimeStampLost = mCurrentFrame.mTimeStamp;
      }
    }

    if (mState == LOST) {
      if (pCurrentMap->KeyFramesInMap() <= 10) {
        mpSystem->ResetActiveMap();
        return;
      }
      if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
          mSensor == System::IMU_RGBD)
        if (!pCurrentMap->isImuInitialized()) {
          Verbose::PrintMess(
              "Track lost before IMU initialisation, reseting...",
              Verbose::VERBOSITY_QUIET);
          mpSystem->ResetActiveMap();
          return;
        }

      CreateMapInAtlas();

      return;
    }

    if (!mCurrentFrame.mpReferenceKF)
      mCurrentFrame.mpReferenceKF = mpReferenceKF;

    mLastFrame = Frame(mCurrentFrame);
  }

  if (mState == OK || mState == RECENTLY_LOST) {
    const bool lostForTrajectoryExport = (mState != OK);

    // Store frame pose information to retrieve the complete camera trajectory
    // afterwards.
    if (mCurrentFrame.isSet()) {
      Sophus::SE3f Tcr_ = mCurrentFrame.GetPose() *
                          mCurrentFrame.mpReferenceKF->GetPoseInverse();
      mlRelativeFramePoses.push_back(Tcr_);
      mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
      mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
      mlbLost.push_back(lostForTrajectoryExport);
    } else {
      // This can happen if tracking is lost
      mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
      mlpReferences.push_back(mlpReferences.back());
      mlFrameTimes.push_back(mlFrameTimes.back());
      mlbLost.push_back(lostForTrajectoryExport);
    }
  }

#ifdef REGISTER_LOOP
  if (Stop()) {

    // Safe area to stop
    return;
  }
#endif
}

void Tracking::StereoInitialization() {
  const bool stereoFeatureInjected = mCurrentFrame.mbStereoFeatureInjected;
  const int stereoInitMinFeatures = EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_INIT_MIN_FEATURES", 56, 1, 1000);
  const int stereoInitMinClosePoints = EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_INIT_MIN_CLOSE_POINTS", 40, 0, 1000);
  const float stereoInitMinCloseRatio = EnvFloatClamped(
      "SMART_DRONE_STEREO_FEATURE_INIT_MIN_CLOSE_RATIO", 0.50f, 0.0f, 1.0f);

  const int stereoFeatureCount =
      (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
  const int stereoClosePointCount = mCurrentFrame.mnCloseMPs;
  const bool stereoInitGeometryHealthy =
      stereoFeatureCount > 0 &&
      static_cast<float>(stereoClosePointCount) >=
          (stereoInitMinCloseRatio * static_cast<float>(stereoFeatureCount));
  const bool stereoInitReady =
      stereoFeatureInjected
          ? (stereoFeatureCount >= stereoInitMinFeatures &&
             stereoClosePointCount >= stereoInitMinClosePoints &&
             stereoInitGeometryHealthy)
          : (mCurrentFrame.N > 500);
  if (stereoFeatureInjected &&
      EnvFlagEnabled("SMART_DRONE_STEREO_FEATURE_INIT_DFX", false)) {
    const float closeRatio = stereoFeatureCount > 0
                                 ? static_cast<float>(stereoClosePointCount) /
                                       static_cast<float>(stereoFeatureCount)
                                 : 0.0f;
    cerr << "[stereo_feature_init_dfx] features=" << stereoFeatureCount
         << " close=" << stereoClosePointCount << " close_ratio=" << closeRatio
         << " min_features=" << stereoInitMinFeatures
         << " min_close=" << stereoInitMinClosePoints
         << " min_close_ratio=" << stereoInitMinCloseRatio
         << " ready=" << (stereoInitReady ? "Y" : "N") << "\n";
  }

  if (stereoInitReady) {
    if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
      if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated) {
        cout << "not IMU meas" << endl;
        return;
      }

      if (!mFastInit && (mCurrentFrame.mpImuPreintegratedFrame->avgA -
                         mLastFrame.mpImuPreintegratedFrame->avgA)
                                .norm() < 0.5) {
        cout << "not enough acceleration" << endl;
        return;
      }

      if (mpImuPreintegratedFromLastKF)
        delete mpImuPreintegratedFromLastKF;

      mpImuPreintegratedFromLastKF =
          new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
      mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    }

    // Set Frame pose to the origin (In case of inertial SLAM to imu)
    if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
      Eigen::Matrix3f Rwb0 = mCurrentFrame.mImuCalib.mTcb.rotationMatrix();
      Eigen::Vector3f twb0 = mCurrentFrame.mImuCalib.mTcb.translation();
      Eigen::Vector3f Vwb0;
      Vwb0.setZero();
      mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, Vwb0);
    } else
      mCurrentFrame.SetPose(Sophus::SE3f());

    // Create KeyFrame
    KeyFrame *pKFini =
        new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

    // Insert KeyFrame in the map
    mpAtlas->AddKeyFrame(pKFini);

    // Create MapPoints and asscoiate to KeyFrame
    if (!mpCamera2) {
      for (int i = 0; i < mCurrentFrame.N; i++) {
        float z = mCurrentFrame.mvDepth[i];
        if (z > 0) {
          Eigen::Vector3f x3D;
          mCurrentFrame.UnprojectStereo(i, x3D);
          MapPoint *pNewMP =
              new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
          pNewMP->AddObservation(pKFini, i);
          pKFini->AddMapPoint(pNewMP, i);
          pNewMP->ComputeDistinctiveDescriptors();
          pNewMP->UpdateNormalAndDepth();
          mpAtlas->AddMapPoint(pNewMP);

          mCurrentFrame.mvpMapPoints[i] = pNewMP;
        }
      }
    } else {
      for (int i = 0; i < mCurrentFrame.Nleft; i++) {
        int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
        if (rightIndex != -1) {
          Eigen::Vector3f x3D = mCurrentFrame.mvStereo3Dpoints[i];

          MapPoint *pNewMP =
              new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());

          pNewMP->AddObservation(pKFini, i);
          pNewMP->AddObservation(pKFini, rightIndex + mCurrentFrame.Nleft);

          pKFini->AddMapPoint(pNewMP, i);
          pKFini->AddMapPoint(pNewMP, rightIndex + mCurrentFrame.Nleft);

          pNewMP->ComputeDistinctiveDescriptors();
          pNewMP->UpdateNormalAndDepth();
          mpAtlas->AddMapPoint(pNewMP);

          mCurrentFrame.mvpMapPoints[i] = pNewMP;
          mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft] = pNewMP;
        }
      }
    }

    Verbose::PrintMess("New Map created with " +
                           to_string(mpAtlas->MapPointsInMap()) + " points",
                       Verbose::VERBOSITY_QUIET);

    // cout << "Active map: " << mpAtlas->GetCurrentMap()->GetId() << endl;

    InsertLocalMappingKeyFrame(mpLocalMappingBackend.get(), pKFini);

    mLastFrame = Frame(mCurrentFrame);
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFini;
    // mnLastRelocFrameId = mCurrentFrame.mnId;

    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
    mpReferenceKF = pKFini;
    mCurrentFrame.mpReferenceKF = pKFini;

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    // mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

    mState = OK;
    if (stereoFeatureInjected)
      mnStereoFeatureInitFrameId = mCurrentFrame.mnId;
  } else if (stereoFeatureInjected) {
    Verbose::PrintMess(
        "Stereo init waiting: features=" + to_string(stereoFeatureCount) +
            " close=" + to_string(stereoClosePointCount) +
            " thresholds=" + to_string(stereoInitMinFeatures) + "/" +
            to_string(stereoInitMinClosePoints) + " ratio=" +
            to_string(stereoFeatureCount > 0
                          ? static_cast<float>(stereoClosePointCount) /
                                static_cast<float>(stereoFeatureCount)
                          : 0.0f),
        Verbose::VERBOSITY_QUIET);
  }
}

void Tracking::MonocularInitialization() {

  if (!mbReadyToInitializate) {
    // Set Reference Frame
    if (mCurrentFrame.mvKeys.size() > 100) {

      mInitialFrame = Frame(mCurrentFrame);
      mLastFrame = Frame(mCurrentFrame);
      mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
      for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
        mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

      fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

      if (mSensor == System::IMU_MONOCULAR) {
        if (mpImuPreintegratedFromLastKF) {
          delete mpImuPreintegratedFromLastKF;
        }
        mpImuPreintegratedFromLastKF =
            new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
        mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
      }

      mbReadyToInitializate = true;

      return;
    }
  } else {
    if (((int)mCurrentFrame.mvKeys.size() <= 100) ||
        ((mSensor == System::IMU_MONOCULAR) &&
         (mLastFrame.mTimeStamp - mInitialFrame.mTimeStamp > 1.0))) {
      mbReadyToInitializate = false;

      return;
    }

    // Find correspondences
    auto matcher = CreateDefaultOrbFeatureMatcher(
        OrbFeatureMatcherOptions{0.9f, true});
    int nmatches = matcher->SearchForInitialization(
        OrbInitializationMatchRequest{&mInitialFrame, &mCurrentFrame,
                                      &mvbPrevMatched, &mvIniMatches, 100});

    // Check if there are enough correspondences
    if (nmatches < 100) {
      mbReadyToInitializate = false;
      return;
    }

    Sophus::SE3f Tcw;
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

    if (mpCamera->ReconstructWithTwoViews(
            TwoViewReconstructionRequest{&mInitialFrame.mvKeysUn,
                                         &mCurrentFrame.mvKeysUn,
                                         &mvIniMatches},
            TwoViewReconstructionResult{&Tcw, &mvIniP3D, &vbTriangulated})) {
      for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
        if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
          mvIniMatches[i] = -1;
          nmatches--;
        }
      }

      // Set Frame Poses
      mInitialFrame.SetPose(Sophus::SE3f());
      mCurrentFrame.SetPose(Tcw);

      CreateInitialMapMonocular();
    }
  }
}

void Tracking::CreateInitialMapMonocular() {
  // Create KeyFrames
  KeyFrame *pKFini =
      new KeyFrame(mInitialFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);
  KeyFrame *pKFcur =
      new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

  if (mSensor == System::IMU_MONOCULAR)
    pKFini->mpImuPreintegrated = (IMU::Preintegrated *)(NULL);

  pKFini->ComputeBoW();
  pKFcur->ComputeBoW();

  // Insert KFs in the map
  mpAtlas->AddKeyFrame(pKFini);
  mpAtlas->AddKeyFrame(pKFcur);

  for (size_t i = 0; i < mvIniMatches.size(); i++) {
    if (mvIniMatches[i] < 0)
      continue;

    // Create MapPoint.
    Eigen::Vector3f worldPos;
    worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
    MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpAtlas->GetCurrentMap());

    pKFini->AddMapPoint(pMP, i);
    pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

    pMP->AddObservation(pKFini, i);
    pMP->AddObservation(pKFcur, mvIniMatches[i]);

    pMP->ComputeDistinctiveDescriptors();
    pMP->UpdateNormalAndDepth();

    // Fill Current Frame structure
    mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
    mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

    // Add to Map
    mpAtlas->AddMapPoint(pMP);
  }

  // Update Connections
  pKFini->UpdateConnections();
  pKFcur->UpdateConnections();

  std::set<MapPoint *> sMPs;
  sMPs = pKFini->GetMapPoints();

  // Bundle Adjustment
  Verbose::PrintMess("New Map created with " +
                         to_string(mpAtlas->MapPointsInMap()) + " points",
                     Verbose::VERBOSITY_QUIET);
  RunTrackingGlobalBundleAdjustment(mpOptimizationBackend.get(),
                                    mpAtlas->GetCurrentMap(), 20);

  float medianDepth = pKFini->ComputeSceneMedianDepth(2);
  float invMedianDepth;
  if (mSensor == System::IMU_MONOCULAR)
    invMedianDepth = 4.0f / medianDepth; // 4.0f
  else
    invMedianDepth = 1.0f / medianDepth;

  if (medianDepth < 0 ||
      pKFcur->TrackedMapPoints(1) < 50) // TODO Check, originally 100 tracks
  {
    Verbose::PrintMess("Wrong initialization, reseting...",
                       Verbose::VERBOSITY_QUIET);
    mpSystem->ResetActiveMap();
    return;
  }

  // Scale initial baseline
  Sophus::SE3f Tc2w = pKFcur->GetPose();
  Tc2w.translation() *= invMedianDepth;
  pKFcur->SetPose(Tc2w);

  // Scale points
  vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
  for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
    if (vpAllMapPoints[iMP]) {
      MapPoint *pMP = vpAllMapPoints[iMP];
      pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
      pMP->UpdateNormalAndDepth();
    }
  }

  if (mSensor == System::IMU_MONOCULAR) {
    pKFcur->mPrevKF = pKFini;
    pKFini->mNextKF = pKFcur;
    pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(
        pKFcur->mpImuPreintegrated->GetUpdatedBias(), pKFcur->mImuCalib);
  }

  InsertLocalMappingKeyFrame(mpLocalMappingBackend.get(), pKFini);
  InsertLocalMappingKeyFrame(mpLocalMappingBackend.get(), pKFcur);
  if (mpLocalMappingBackend != nullptr) {
    mpLocalMappingBackend->SetFirstTimestamp(pKFcur->mTimeStamp);
  }

  mCurrentFrame.SetPose(pKFcur->GetPose());
  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKFcur;
  // mnLastRelocFrameId = mInitialFrame.mnId;

  mvpLocalKeyFrames.push_back(pKFcur);
  mvpLocalKeyFrames.push_back(pKFini);
  mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
  mpReferenceKF = pKFcur;
  mCurrentFrame.mpReferenceKF = pKFcur;

  // Compute here initial velocity
  vector<KeyFrame *> vKFs = mpAtlas->GetAllKeyFrames();

  Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
  mbVelocity = false;
  Eigen::Vector3f phi = deltaT.so3().log();

  double aux = (mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp) /
               (mCurrentFrame.mTimeStamp - mInitialFrame.mTimeStamp);
  phi *= aux;

  mLastFrame = Frame(mCurrentFrame);

  mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

  // mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

  mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

  mState = OK;

  initID = pKFcur->mnId;
}

void Tracking::CreateMapInAtlas() {
  mnLastInitFrameId = mCurrentFrame.mnId;
  mpAtlas->CreateNewMap();
  if (mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR ||
      mSensor == System::IMU_RGBD)
    mpAtlas->SetInertialSensor();
  mbSetInit = false;

  mnInitialFrameId = mCurrentFrame.mnId + 1;
  mState = NO_IMAGES_YET;

  // Restart the variable with information about the last KF
  mbVelocity = false;
  // mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the
  // current id, because it is the new starting point for new map
  Verbose::PrintMess("First frame id in map: " +
                         to_string(mnLastInitFrameId + 1),
                     Verbose::VERBOSITY_NORMAL);
  mbVO =
      false; // Init value for know if there are enough MapPoints in the last KF
  if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR) {
    mbReadyToInitializate = false;
  }

  if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
       mSensor == System::IMU_RGBD) &&
      mpImuPreintegratedFromLastKF) {
    delete mpImuPreintegratedFromLastKF;
    mpImuPreintegratedFromLastKF =
        new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
  }

  if (mpLastKeyFrame)
    mpLastKeyFrame = static_cast<KeyFrame *>(NULL);

  if (mpReferenceKF)
    mpReferenceKF = static_cast<KeyFrame *>(NULL);

  mLastFrame = Frame();
  mCurrentFrame = Frame();
  mvIniMatches.clear();

  mbCreatedMap = true;
}

void Tracking::CheckReplacedInLastFrame() {
  for (int i = 0; i < mLastFrame.N; i++) {
    MapPoint *pMP = mLastFrame.mvpMapPoints[i];

    if (pMP) {
      MapPoint *pRep = pMP->GetReplaced();
      if (pRep) {
        mLastFrame.mvpMapPoints[i] = pRep;
      }
    }
  }
}

bool Tracking::TrackReferenceKeyFrame() {
  const bool stereoFeatureBootstrap = IsStereoFeatureBootstrap(
      mCurrentFrame, mpAtlas, mnStereoFeatureInitFrameId);
  const bool stereoFeatureStabilizing = IsStereoFeatureStabilizing(
      mCurrentFrame, mpAtlas, mnStereoFeatureInitFrameId);
  const bool pureStereoBootstrap = IsPureStereoBootstrap(mSensor, mpAtlas);
  const bool pureStereoStabilizing = IsPureStereoStabilizing(mSensor, mpAtlas);
  const bool stereoBootstrap = stereoFeatureBootstrap || pureStereoBootstrap;
  const bool stereoStabilizing =
      stereoFeatureStabilizing || pureStereoStabilizing;
  const bool stereoFeatureRefProjectionRescue = stereoStabilizing;
  const int minRefKfMatches =
      stereoBootstrap ? 4 : (stereoStabilizing ? 8 : 15);
  const int minRefKfTrackedMapMatches =
      stereoBootstrap ? 2 : (stereoStabilizing ? 5 : 10);
  const float refProjectionSearchTh =
      stereoBootstrap ? 12.0f : (stereoStabilizing ? 8.0f : 5.0f);

  // Compute Bag of Words vector
  mCurrentFrame.ComputeBoW();

  // We perform first an ORB matching with the reference keyframe
  // If enough matches are found we setup a PnP solver
  auto matcher = CreateDefaultOrbFeatureMatcher(
      OrbFeatureMatcherOptions{0.7f, true});
  vector<MapPoint *> vpMapPointMatches;

  auto optimizeReferenceMatches = [&]() -> int {
    OptimizeCurrentFramePose(mpOptimizationBackend.get(), mCurrentFrame);

    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++) {
      if (mCurrentFrame.mvpMapPoints[i]) {
        if (mCurrentFrame.mvbOutlier[i]) {
          MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

          mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
          mCurrentFrame.mvbOutlier[i] = false;
          if (i < mCurrentFrame.Nleft) {
            pMP->mbTrackInView = false;
          } else {
            pMP->mbTrackInViewR = false;
          }
          pMP->mbTrackInView = false;
          pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0) {
          nmatchesMap++;
        }
      }
    }

    return nmatchesMap;
  };

  auto tryReferenceProjectionRescue = [&](const Sophus::SE3f &seedPose,
                                          bool keepExistingMatches) -> bool {
    mCurrentFrame.SetPose(seedPose);

    std::set<MapPoint *> alreadyFound;
    if (!keepExistingMatches) {
      fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(),
           static_cast<MapPoint *>(NULL));
      fill(mCurrentFrame.mvbOutlier.begin(), mCurrentFrame.mvbOutlier.end(),
           false);
    } else {
      for (int i = 0; i < mCurrentFrame.N; ++i) {
        if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i]) {
          alreadyFound.insert(mCurrentFrame.mvpMapPoints[i]);
        }
      }
    }

    const int projectedMatches = matcher->SearchFrameKeyFrameByProjection(
        OrbFrameKeyFrameProjectionRequest{
            &mCurrentFrame, mpReferenceKF, &alreadyFound,
            refProjectionSearchTh, kOrbMatcherHighDistance});
    const int rescuedTracked = CountTrackedMapPoints(mCurrentFrame);

    if (rescuedTracked < minRefKfMatches) {
      cout << "TRACK_REF_KF: Projection rescue only found " << rescuedTracked
           << " matches after adding " << projectedMatches
           << " projected matches!!\n";
      return false;
    }

    const int rescuedMapMatches = optimizeReferenceMatches();
    if (rescuedMapMatches < minRefKfTrackedMapMatches) {
      cout << "TRACK_REF_KF: Projection rescue only kept " << rescuedMapMatches
           << " tracked map matches after PO!!\n";
      return false;
    }

    cout << "TRACK_REF_KF: Projection rescue succeeded with "
         << rescuedMapMatches << " tracked map matches after PO!!\n";
    return true;
  };

  int nmatches = matcher->SearchFrameByBoW(
      OrbFrameBoWMatchRequest{mpReferenceKF, &mCurrentFrame,
                              &vpMapPointMatches});

  if (nmatches < minRefKfMatches) {
    if (stereoFeatureRefProjectionRescue &&
        tryReferenceProjectionRescue(mLastFrame.GetPose(), false)) {
      return true;
    }
    cout << "TRACK_REF_KF: Less than " << minRefKfMatches << " matches!!\n";
    return false;
  }

  mCurrentFrame.mvpMapPoints = vpMapPointMatches;
  mCurrentFrame.SetPose(mLastFrame.GetPose());

  // mCurrentFrame.PrintPointDistribution();
  const int nmatchesMap = optimizeReferenceMatches();

  if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
      mSensor == System::IMU_RGBD)
    return true;
  else {
    if (nmatchesMap < minRefKfTrackedMapMatches) {
      if (stereoFeatureRefProjectionRescue &&
          tryReferenceProjectionRescue(mCurrentFrame.GetPose(), true)) {
        return true;
      }
      cout << "TRACK_REF_KF: Less than " << minRefKfTrackedMapMatches
           << " tracked map matches after PO!!\n";
      return false;
    }
    return true;
  }
}

void Tracking::UpdateLastFrame() {
  // Update pose according to reference keyframe
  KeyFrame *pRef = mLastFrame.mpReferenceKF;
  Sophus::SE3f Tlr = mlRelativeFramePoses.back();
  mLastFrame.SetPose(Tlr * pRef->GetPose());

  if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR ||
      mSensor == System::IMU_MONOCULAR || !mbOnlyTracking)
    return;

  // Create "visual odometry" MapPoints
  // We sort points according to their measured depth by the stereo/RGB-D sensor
  vector<pair<float, int>> vDepthIdx;
  const int Nfeat = mLastFrame.Nleft == -1 ? mLastFrame.N : mLastFrame.Nleft;
  vDepthIdx.reserve(Nfeat);
  for (int i = 0; i < Nfeat; i++) {
    float z = mLastFrame.mvDepth[i];
    if (z > 0) {
      vDepthIdx.push_back(make_pair(z, i));
    }
  }

  if (vDepthIdx.empty())
    return;

  sort(vDepthIdx.begin(), vDepthIdx.end());

  // We insert all close points (depth<mThDepth)
  // If less than 100 close points, we insert the 100 closest ones.
  int nPoints = 0;
  for (size_t j = 0; j < vDepthIdx.size(); j++) {
    int i = vDepthIdx[j].second;

    bool bCreateNew = false;

    MapPoint *pMP = mLastFrame.mvpMapPoints[i];

    if (!pMP)
      bCreateNew = true;
    else if (pMP->Observations() < 1)
      bCreateNew = true;

    if (bCreateNew) {
      Eigen::Vector3f x3D;

      if (mLastFrame.Nleft == -1) {
        mLastFrame.UnprojectStereo(i, x3D);
      } else {
        x3D = mLastFrame.UnprojectStereoFishEye(i);
      }

      MapPoint *pNewMP =
          new MapPoint(x3D, mpAtlas->GetCurrentMap(), &mLastFrame, i);
      mLastFrame.mvpMapPoints[i] = pNewMP;

      mlpTemporalPoints.push_back(pNewMP);
      nPoints++;
    } else {
      nPoints++;
    }

    if (vDepthIdx[j].first > mThDepth && nPoints > 100)
      break;
  }
}

bool Tracking::TrackWithMotionModel() {
  auto matcher = CreateDefaultOrbFeatureMatcher(
      OrbFeatureMatcherOptions{0.9f, true});
  const bool stereoFeatureBootstrap = IsStereoFeatureBootstrap(
      mCurrentFrame, mpAtlas, mnStereoFeatureInitFrameId);
  const bool stereoFeatureStabilizing = IsStereoFeatureStabilizing(
      mCurrentFrame, mpAtlas, mnStereoFeatureInitFrameId);

  // Update last frame pose according to its reference keyframe
  // Create "visual odometry" points if in Localization Mode
  UpdateLastFrame();

  if (mpAtlas->isImuInitialized() &&
      (mCurrentFrame.mnId > mnLastRelocFrameId + mnFramesToResetIMU)) {
    // Predict state with IMU if it is initialized and it doesnt need reset
    PredictStateIMU();
    return true;
  } else {
    mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
  }

  fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(),
       static_cast<MapPoint *>(NULL));

  // Project points seen in previous frame
  int th;

  if (mSensor == System::STEREO)
    th = 7;
  else
    th = 15;

  int nmatches = matcher->SearchFrameFrameByProjection(
      OrbFrameFrameProjectionRequest{
          &mCurrentFrame, &mLastFrame, static_cast<float>(th),
          mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR});
  const int minMotionModelProjectionMatches =
      stereoFeatureBootstrap ? 5 : (stereoFeatureStabilizing ? 8 : 20);
  const int minMotionModelTrackedMapMatches =
      stereoFeatureBootstrap ? 2 : (stereoFeatureStabilizing ? 4 : 10);

  // If few matches, uses a wider window search
  if (nmatches < minMotionModelProjectionMatches) {
    Verbose::PrintMess("Not enough matches, wider window search!!",
                       Verbose::VERBOSITY_NORMAL);
    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(),
         static_cast<MapPoint *>(NULL));

    const int widenedTh = stereoFeatureStabilizing ? 3 * th : 2 * th;
    nmatches = matcher->SearchFrameFrameByProjection(
        OrbFrameFrameProjectionRequest{
            &mCurrentFrame, &mLastFrame, static_cast<float>(widenedTh),
            mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR});
    Verbose::PrintMess("Matches with wider search: " + to_string(nmatches),
                       Verbose::VERBOSITY_NORMAL);
  }

  if (nmatches < minMotionModelProjectionMatches) {
    Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
        mSensor == System::IMU_RGBD)
      return true;
    else
      return false;
  }

  // Optimize frame pose with all matches
  OptimizeCurrentFramePose(mpOptimizationBackend.get(), mCurrentFrame);

  // Discard outliers
  int nmatchesMap = 0;
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
        MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
        mCurrentFrame.mvbOutlier[i] = false;
        if (i < mCurrentFrame.Nleft) {
          pMP->mbTrackInView = false;
        } else {
          pMP->mbTrackInViewR = false;
        }
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        nmatches--;
      } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
        nmatchesMap++;
    }
  }

  if (mbOnlyTracking) {
    mbVO = nmatchesMap < 10;
    return nmatches > 20;
  }

  if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
      mSensor == System::IMU_RGBD)
    return true;
  else {
    if (nmatchesMap < minMotionModelTrackedMapMatches) {
      cout << "TRACK_MM: Less than " << minMotionModelTrackedMapMatches
           << " tracked map matches after PO!!\n";
      return false;
    }
    return true;
  }
}

bool Tracking::TrackLocalMap() {

  // We have an estimation of the camera pose and some map points tracked in the
  // frame. We retrieve the local map and try to find matches to points in the
  // local map.
  mTrackedFr++;

  UpdateLocalMap();
  SearchLocalPoints();

  // TOO check outliers before PO
  int aux1 = 0, aux2 = 0;
  for (int i = 0; i < mCurrentFrame.N; i++)
    if (mCurrentFrame.mvpMapPoints[i]) {
      aux1++;
      if (mCurrentFrame.mvbOutlier[i])
        aux2++;
    }
  mMatchedMapPointHashBeforePoseOptimization =
      HashMapPointSequence(mCurrentFrame.mvpMapPoints);

  int inliers;
  if (!mpAtlas->isImuInitialized())
    OptimizeCurrentFramePose(mpOptimizationBackend.get(), mCurrentFrame);
  else {
    if (mCurrentFrame.mnId <= mnLastRelocFrameId + mnFramesToResetIMU) {
      Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
      OptimizeCurrentFramePose(mpOptimizationBackend.get(), mCurrentFrame);
    } else {
      // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
      if (!mbMapUpdated) //  && (mnMatchesInliers>30))
      {
        Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ",
                           Verbose::VERBOSITY_DEBUG);
        inliers = OptimizeCurrentFrameInertialLastFrame(
            mpOptimizationBackend.get(),
            mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
      } else {
        Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ",
                           Verbose::VERBOSITY_DEBUG);
        inliers = OptimizeCurrentFrameInertialLastKeyFrame(
            mpOptimizationBackend.get(),
            mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
      }
    }
  }

  aux1 = 0, aux2 = 0;
  for (int i = 0; i < mCurrentFrame.N; i++)
    if (mCurrentFrame.mvpMapPoints[i]) {
      aux1++;
      if (mCurrentFrame.mvbOutlier[i])
        aux2++;
    }

  mnMatchesInliers = 0;
  mTrackedMapPointHash = 1469598103934665603ULL;

  // Update MapPoints Statistics
  for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (!mCurrentFrame.mvbOutlier[i]) {
        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
        if (!mbOnlyTracking) {
          if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0) {
            mnMatchesInliers++;
            mTrackedMapPointHash =
                MixTrackHash(mTrackedMapPointHash, static_cast<uint64_t>(i));
            mTrackedMapPointHash = MixTrackHash(
                mTrackedMapPointHash,
                static_cast<uint64_t>(mCurrentFrame.mvpMapPoints[i]->mnId));
          }
        } else {
          mnMatchesInliers++;
          mTrackedMapPointHash =
              MixTrackHash(mTrackedMapPointHash, static_cast<uint64_t>(i));
          mTrackedMapPointHash = MixTrackHash(
              mTrackedMapPointHash,
              static_cast<uint64_t>(mCurrentFrame.mvpMapPoints[i]->mnId));
        }
      } else if (mSensor == System::STEREO)
        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
    }
  }

  // Decide if the tracking was succesful
  // More restrictive if there was a relocalization recently
  const bool stereoFeatureBootstrap = IsStereoFeatureBootstrap(
      mCurrentFrame, mpAtlas, mnStereoFeatureInitFrameId);
  const bool stereoFeatureStabilizing = IsStereoFeatureStabilizing(
      mCurrentFrame, mpAtlas, mnStereoFeatureInitFrameId);
  const bool pureStereoBootstrap = IsPureStereoBootstrap(mSensor, mpAtlas);
  const bool pureStereoStabilizing = IsPureStereoStabilizing(mSensor, mpAtlas);
  const bool stereoBootstrap = stereoFeatureBootstrap || pureStereoBootstrap;
  const bool stereoStabilizing =
      stereoFeatureStabilizing || pureStereoStabilizing;
  const bool stereoFeatureStablePhase = mCurrentFrame.mbStereoFeatureInjected &&
                                        !stereoFeatureBootstrap &&
                                        !stereoFeatureStabilizing;
  const int trackedMapPoints = CountTrackedMapPoints(mCurrentFrame);
  const int effectiveLocalMapMatches =
      stereoStabilizing ? std::max(trackedMapPoints, mnMatchesInliers)
                        : mnMatchesInliers;
  const int bootstrapClosePoints = mCurrentFrame.mnCloseMPs;
  const bool bootstrapVisualOdometryHealthy =
      stereoBootstrap && (trackedMapPoints >= 2 || bootstrapClosePoints >= 24);
  const bool stabilizingVisualOdometryHealthy =
      stereoStabilizing &&
      (trackedMapPoints >= 4 || bootstrapClosePoints >= 32);
  const int stableJumpGuardMinInliers =
      StereoFeatureStableJumpGuardMinInliers();
  const int stableJumpGuardMinTracked =
      StereoFeatureStableJumpGuardMinTrackedMapPoints();
  const float stableJumpGuardMaxStepM =
      StereoFeatureStableJumpGuardMaxStepMeters();
  const float stablePoseStepM =
      stereoFeatureStablePhase
          ? PoseStepMetersBetweenFrames(mCurrentFrame, mLastFrame)
          : 0.0f;
  const bool stableJumpGuardWeakObservation =
      mnMatchesInliers < stableJumpGuardMinInliers ||
      trackedMapPoints < stableJumpGuardMinTracked;
  const bool stableJumpGuardReject = stereoFeatureStablePhase &&
                                     StereoFeatureStableJumpGuardEnabled() &&
                                     stableJumpGuardWeakObservation &&
                                     stablePoseStepM > stableJumpGuardMaxStepM;
  const int minRecentRelocInliers =
      stereoBootstrap ? 4 : (stereoStabilizing ? 8 : 50);
  const int defaultMinStereoLocalMapInliers =
      stereoBootstrap ? 4 : (stereoStabilizing ? 8 : 30);
  const int minStereoLocalMapInliers =
      stereoFeatureBootstrap
          ? EnvIntClamped(
                "SMART_DRONE_STEREO_FEATURE_BOOTSTRAP_MIN_LOCAL_MAP_INLIERS",
                defaultMinStereoLocalMapInliers, 0, 2000)
          : (stereoFeatureStabilizing
                 ? EnvIntClamped("SMART_DRONE_STEREO_FEATURE_STABILIZING_MIN_"
                                 "LOCAL_MAP_INLIERS",
                                 defaultMinStereoLocalMapInliers, 0, 2000)
                 : EnvIntClamped("SMART_DRONE_STEREO_FEATURE_STABLE_MIN_LOCAL_"
                                 "MAP_INLIERS",
                                 defaultMinStereoLocalMapInliers, 0, 2000));
  const bool requireStereoFeatureMapInliers =
      mCurrentFrame.mbStereoFeatureInjected && StereoFeatureRequireMapInliers();
  const int minRecentlyLostInliers =
      stereoBootstrap ? 3 : (stereoStabilizing ? 6 : 10);
  auto logDecision = [&](const char *reason, bool accepted) {
    if (!StereoFeatureTrackDfxEnabled() ||
        !mCurrentFrame.mbStereoFeatureInjected)
      return;

    cerr << "[stereo_feature_local_map] frame=" << mCurrentFrame.mnId
         << " reason=" << reason << " accepted=" << (accepted ? "Y" : "N")
         << " state=" << mState
         << " bootstrap=" << (stereoFeatureBootstrap ? "Y" : "N")
         << " stabilizing=" << (stereoFeatureStabilizing ? "Y" : "N")
         << " features=" << StereoFeatureLeftFeatureCount(mCurrentFrame)
         << " close=" << bootstrapClosePoints << " tracked=" << trackedMapPoints
         << " inliers=" << mnMatchesInliers
         << " effective=" << effectiveLocalMapMatches
         << " kfs=" << (mpAtlas ? mpAtlas->KeyFramesInMap() : 0)
         << " init_frame=" << mnStereoFeatureInitFrameId
         << " step_m=" << stablePoseStepM
         << " stable_guard=" << (stableJumpGuardReject ? "Y" : "N")
         << " guard_max_step=" << stableJumpGuardMaxStepM
         << " guard_min_inliers=" << stableJumpGuardMinInliers
         << " guard_min_tracked=" << stableJumpGuardMinTracked
         << " min_recent=" << minRecentRelocInliers
         << " min_lost=" << minRecentlyLostInliers
         << " min_local=" << minStereoLocalMapInliers << "\n";
  };

  if (mpLocalMappingBackend != nullptr) {
    mpLocalMappingBackend->SetMatchesInliers(mnMatchesInliers);
  }
  if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames &&
      effectiveLocalMapMatches < minRecentRelocInliers) {
    logDecision("recent_reloc_guard", false);
    return false;
  }

  if ((effectiveLocalMapMatches >= minRecentlyLostInliers) &&
      (mState == RECENTLY_LOST)) {
    logDecision("recently_lost_inliers", true);
    return true;
  }

  if (bootstrapVisualOdometryHealthy || stabilizingVisualOdometryHealthy) {
    if (requireStereoFeatureMapInliers &&
        effectiveLocalMapMatches < minStereoLocalMapInliers) {
      logDecision("stereo_vo_require_map_inliers", false);
      return false;
    }
    logDecision("stereo_vo_healthy", true);
    return true;
  }

  if (stableJumpGuardReject) {
    logDecision("stable_jump_guard", false);
    return false;
  }

  if (mSensor == System::IMU_MONOCULAR) {
    if ((mnMatchesInliers < 15 && mpAtlas->isImuInitialized()) ||
        (mnMatchesInliers < 50 && !mpAtlas->isImuInitialized())) {
      logDecision("imu_mono_inliers", false);
      return false;
    } else {
      logDecision("imu_mono_inliers", true);
      return true;
    }
  } else if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
    if (mnMatchesInliers < 15) {
      logDecision("imu_stereo_inliers", false);
      return false;
    } else {
      logDecision("imu_stereo_inliers", true);
      return true;
    }
  } else {
    if (effectiveLocalMapMatches < minStereoLocalMapInliers) {
      logDecision("stereo_inliers", false);
      return false;
    } else {
      logDecision("stereo_inliers", true);
      return true;
    }
  }
}

bool Tracking::NeedNewKeyFrame() {
  if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
       mSensor == System::IMU_RGBD) &&
      !mpAtlas->GetCurrentMap()->isImuInitialized()) {
    if (mSensor == System::IMU_MONOCULAR &&
        (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25)
      return true;
    else if ((mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&
             (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.25)
      return true;
    else
      return false;
  }

  if (mbOnlyTracking)
    return false;

  // If Local Mapping is freezed by a Loop Closure do not insert keyframes
  const OrbLocalMappingStatus localMappingStatus =
      GetLocalMappingStatus(mpLocalMappingBackend.get());
  if (localMappingStatus.stopped || localMappingStatus.stopRequested) {
    /*if(mSensor == System::MONOCULAR)
    {
        std::cout << "NeedNewKeyFrame: localmap stopped" << std::endl;
    }*/
    return false;
  }

  const int nKFs = mpAtlas->KeyFramesInMap();
  const bool stereoFeatureStabilizing = IsStereoFeatureStabilizing(
      mCurrentFrame, mpAtlas, mnStereoFeatureInitFrameId);
  const bool stereoFeatureBootstrap = IsStereoFeatureBootstrap(
      mCurrentFrame, mpAtlas, mnStereoFeatureInitFrameId);
  const int trackedCurrent = CountTrackedMapPoints(mCurrentFrame);

  // Do not insert keyframes if not enough frames have passed from last
  // relocalisation
  if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames &&
      nKFs > mMaxFrames) {
    return false;
  }

  // Tracked MapPoints in the reference keyframe
  int nMinObs = 3;
  if (nKFs <= 2)
    nMinObs = 2;
  int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

  // Local Mapping accept keyframes?
  const bool bLocalMappingIdle = localMappingStatus.acceptingKeyframes;

  // Check how many "close" points are being tracked and how many could be
  // potentially created.
  int nNonTrackedClose = 0;
  int nTrackedClose = 0;

  if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR) {
    int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
    for (int i = 0; i < N; i++) {
      if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth) {
        if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
          nTrackedClose++;
        else
          nNonTrackedClose++;
      }
    }
    // Verbose::PrintMess("[NEEDNEWKF]-> closed points: " +
    // to_string(nTrackedClose) + "; non tracked closed points: " +
    // to_string(nNonTrackedClose), Verbose::VERBOSITY_NORMAL);//
    // Verbose::VERBOSITY_DEBUG);
  }

  const int insertCloseTrackedMax = EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_INSERT_CLOSE_TRACKED_MAX", 100, 0, 1000);
  const int insertCloseNonTrackedMin = EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_INSERT_CLOSE_NONTRACKED_MIN", 70, 0, 1000);
  const int bootstrapInsertCloseTrackedMax = EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_BOOTSTRAP_INSERT_CLOSE_TRACKED_MAX", 40, 0,
      1000);
  const int bootstrapInsertCloseNonTrackedMin = EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_BOOTSTRAP_INSERT_CLOSE_NONTRACKED_MIN", 20, 0,
      1000);
  const int stabilizingInsertCloseTrackedMax = EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_STABILIZING_INSERT_CLOSE_TRACKED_MAX", 80, 0,
      1000);
  const int stabilizingInsertCloseNonTrackedMin = EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_STABILIZING_INSERT_CLOSE_NONTRACKED_MIN", 40,
      0, 1000);

  bool bNeedToInsertClose;
  bNeedToInsertClose = (nTrackedClose < insertCloseTrackedMax) &&
                       (nNonTrackedClose > insertCloseNonTrackedMin);
  if (stereoFeatureBootstrap) {
    bNeedToInsertClose = bNeedToInsertClose ||
                         (nTrackedClose < bootstrapInsertCloseTrackedMax &&
                          nNonTrackedClose > bootstrapInsertCloseNonTrackedMin);
  }
  if (stereoFeatureStabilizing) {
    bNeedToInsertClose =
        bNeedToInsertClose ||
        (nTrackedClose < stabilizingInsertCloseTrackedMax &&
         nNonTrackedClose > stabilizingInsertCloseNonTrackedMin);
  }

  // Thresholds
  float thRefRatio = 0.75f;
  if (nKFs < 2)
    thRefRatio = 0.4f;

  /*int nClosedPoints = nTrackedClose + nNonTrackedClose;
  const int thStereoClosedPoints = 15;
  if(nClosedPoints < thStereoClosedPoints && (mSensor==System::STEREO ||
  mSensor==System::IMU_STEREO))
  {
      //Pseudo-monocular, there are not enough close points to be confident
  about the stereo observations. thRefRatio = 0.9f;
  }*/

  if (mSensor == System::MONOCULAR)
    thRefRatio = 0.9f;

  if (mpCamera2)
    thRefRatio = 0.75f;

  if (mSensor == System::IMU_MONOCULAR) {
    if (mnMatchesInliers > 350) // Points tracked from the local map
      thRefRatio = 0.75f;
    else
      thRefRatio = 0.90f;
  }

  // Condition 1a: More than "MaxFrames" have passed from last keyframe
  // insertion
  const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
  // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
  const bool c1b =
      ((mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames) &&
       bLocalMappingIdle);
  // Condition 1c: tracking is weak
  const bool c1c =
      mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR &&
      mSensor != System::IMU_STEREO && mSensor != System::IMU_RGBD &&
      (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
  // Condition 2: Few tracked points compared to reference keyframe. Lots of
  // visual odometry compared to map matches.
  const bool c2 =
      (((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose)) &&
       mnMatchesInliers > 15);
  const int stereoFeatureGrowMinTrack =
      EnvIntClamped("SMART_DRONE_STEREO_FEATURE_GROW_MIN_TRACK", 6, 0, 1000);
  const int stereoFeatureGrowTrackedCloseMin = EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_GROW_TRACKED_CLOSE_MIN", 20, 0, 1000);
  const int stereoFeatureGrowNonTrackedCloseMin = EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_GROW_NONTRACKED_CLOSE_MIN", 20, 0, 1000);
  const int stereoFeatureGrowCloseMin =
      EnvIntClamped("SMART_DRONE_STEREO_FEATURE_GROW_CLOSE_MIN", 24, 0, 1000);
  const int externalBootstrapGrowTrackedCloseMin = EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_BOOTSTRAP_GROW_TRACKED_CLOSE_MIN", 12, 0,
      1000);
  const int externalBootstrapGrowNonTrackedCloseMin = EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_BOOTSTRAP_GROW_NONTRACKED_CLOSE_MIN", 12, 0,
      1000);
  const int externalBootstrapGrowCloseMin = EnvIntClamped(
      "SMART_DRONE_STEREO_FEATURE_BOOTSTRAP_GROW_CLOSE_MIN", 16, 0, 1000);
  const int stereoFeatureMinFramesBetweenKf =
      StereoFeatureMinFramesBetweenKeyframes();
  const bool stereoFeatureCanInsertByCadence =
      !mCurrentFrame.mbStereoFeatureInjected ||
      mCurrentFrame.mnId >= mnLastKeyFrameId + stereoFeatureMinFramesBetweenKf;
  const bool stereoFeatureGrowMap =
      stereoFeatureStabilizing && stereoFeatureCanInsertByCadence &&
      std::max(mnMatchesInliers, trackedCurrent) >= stereoFeatureGrowMinTrack &&
      (nTrackedClose >= stereoFeatureGrowTrackedCloseMin ||
       nNonTrackedClose >= stereoFeatureGrowNonTrackedCloseMin ||
       mCurrentFrame.mnCloseMPs >= stereoFeatureGrowCloseMin);
  const bool stereoFeatureBootstrapGrowMap =
      stereoFeatureBootstrap && stereoFeatureCanInsertByCadence &&
      (nTrackedClose >= externalBootstrapGrowTrackedCloseMin ||
       nNonTrackedClose >= externalBootstrapGrowNonTrackedCloseMin ||
       mCurrentFrame.mnCloseMPs >= externalBootstrapGrowCloseMin);

  // std::cout << "NeedNewKF: c1a=" << c1a << "; c1b=" << c1b << "; c1c=" << c1c
  // << "; c2=" << c2 << std::endl;
  //  Temporal condition for Inertial cases
  bool c3 = false;
  if (mpLastKeyFrame) {
    if (mSensor == System::IMU_MONOCULAR) {
      if ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
        c3 = true;
    } else if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) {
      if ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= 0.5)
        c3 = true;
    }
  }

  bool c4 = false;
  if ((((mnMatchesInliers < 75) && (mnMatchesInliers > 15)) ||
       mState == RECENTLY_LOST) &&
      (mSensor == System::IMU_MONOCULAR))
    c4 = true;
  else
    c4 = false;

  const bool regularNeedKeyFrame = (((c1a || c1b || c1c) && c2) || c3 || c4);
  const bool shouldInsertKeyFrame =
      (mCurrentFrame.mbStereoFeatureInjected
           ? (regularNeedKeyFrame && stereoFeatureCanInsertByCadence)
           : regularNeedKeyFrame) ||
      stereoFeatureGrowMap || stereoFeatureBootstrapGrowMap;
  if (shouldInsertKeyFrame) {
    // If the mapping accepts keyframes, insert keyframe.
    // Otherwise send a signal to interrupt BA
    if (bLocalMappingIdle || localMappingStatus.initializing) {
      return true;
    } else {
      if (mpLocalMappingBackend != nullptr) {
        mpLocalMappingBackend->InterruptBundleAdjustment();
      }
      if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR) {
        const OrbLocalMappingStatus afterInterruptStatus =
            GetLocalMappingStatus(mpLocalMappingBackend.get());
        if (afterInterruptStatus.keyframesInQueue < 3)
          return true;
        else
          return false;
      } else {
        // std::cout << "NeedNewKeyFrame: localmap is busy" << std::endl;
        return false;
      }
    }
  } else
    return false;
}

void Tracking::CreateNewKeyFrame() {
  const OrbLocalMappingStatus localMappingStatus =
      GetLocalMappingStatus(mpLocalMappingBackend.get());
  if (localMappingStatus.initializing && !mpAtlas->isImuInitialized())
    return;

  if (mpLocalMappingBackend == nullptr ||
      !mpLocalMappingBackend->SetNotStop(true))
    return;

  KeyFrame *pKF =
      new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

  if (mpAtlas->isImuInitialized())
    pKF->bImu = true;

  pKF->SetNewBias(mCurrentFrame.mImuBias);
  mpReferenceKF = pKF;
  mCurrentFrame.mpReferenceKF = pKF;

  if (mpLastKeyFrame) {
    pKF->mPrevKF = mpLastKeyFrame;
    mpLastKeyFrame->mNextKF = pKF;
  } else
    Verbose::PrintMess("No last KF in KF creation!!",
                       Verbose::VERBOSITY_NORMAL);

  // Reset preintegration from last KF (Create new object)
  if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
      mSensor == System::IMU_RGBD) {
    mpImuPreintegratedFromLastKF =
        new IMU::Preintegrated(pKF->GetImuBias(), pKF->mImuCalib);
  }

  if (mSensor != System::MONOCULAR &&
      mSensor != System::IMU_MONOCULAR) // TODO check if incluide imu_stereo
  {
    mCurrentFrame.UpdatePoseMatrices();
    // cout << "create new MPs" << endl;
    // We sort points by the measured depth by the stereo/RGBD sensor.
    // We create all those MapPoints whose depth < mThDepth.
    // If there are less than 100 close points we create the 100 closest.
    int maxPoint = StereoFeatureStableMaxMapPointsPerKeyframe();
    if ((mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&
        !IsStereoFeatureStabilizing(mCurrentFrame, mpAtlas,
                                    mnStereoFeatureInitFrameId)) {
      maxPoint = 100;
    }
    if (IsStereoFeatureBootstrap(mCurrentFrame, mpAtlas,
                                 mnStereoFeatureInitFrameId))
      maxPoint = StereoFeatureBootstrapMaxMapPointsPerKeyframe();
    if (IsStereoFeatureStabilizing(mCurrentFrame, mpAtlas,
                                   mnStereoFeatureInitFrameId))
      maxPoint = StereoFeatureStabilizingMaxMapPointsPerKeyframe();

    vector<pair<float, int>> vDepthIdx;
    int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
    vDepthIdx.reserve(mCurrentFrame.N);
    for (int i = 0; i < N; i++) {
      float z = mCurrentFrame.mvDepth[i];
      if (z > 0) {
        vDepthIdx.push_back(make_pair(z, i));
      }
    }

    int nCreatedPoints = 0;
    int nCountedPoints = 0;
    if (!vDepthIdx.empty()) {
      sort(vDepthIdx.begin(), vDepthIdx.end());

      int nPoints = 0;
      for (size_t j = 0; j < vDepthIdx.size(); j++) {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
        if (!pMP)
          bCreateNew = true;
        else if (pMP->Observations() < 1) {
          bCreateNew = true;
          mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
        }

        if (bCreateNew) {
          Eigen::Vector3f x3D;

          if (mCurrentFrame.Nleft == -1) {
            mCurrentFrame.UnprojectStereo(i, x3D);
          } else {
            x3D = mCurrentFrame.UnprojectStereoFishEye(i);
          }

          MapPoint *pNewMP = new MapPoint(x3D, pKF, mpAtlas->GetCurrentMap());
          pNewMP->AddObservation(pKF, i);

          // Check if it is a stereo observation in order to not
          // duplicate mappoints
          if (mCurrentFrame.Nleft != -1 &&
              mCurrentFrame.mvLeftToRightMatch[i] >= 0) {
            mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft +
                                       mCurrentFrame.mvLeftToRightMatch[i]] =
                pNewMP;
            pNewMP->AddObservation(
                pKF, mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
            pKF->AddMapPoint(pNewMP, mCurrentFrame.Nleft +
                                         mCurrentFrame.mvLeftToRightMatch[i]);
          }

          pKF->AddMapPoint(pNewMP, i);
          pNewMP->ComputeDistinctiveDescriptors();
          pNewMP->UpdateNormalAndDepth();
          mpAtlas->AddMapPoint(pNewMP);

          mCurrentFrame.mvpMapPoints[i] = pNewMP;
          nPoints++;
          nCreatedPoints++;
        } else {
          nPoints++;
        }

        if (vDepthIdx[j].first > mThDepth && nPoints > maxPoint) {
          break;
        }
      }
      nCountedPoints = nPoints;
      // Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints),
      // Verbose::VERBOSITY_NORMAL);
    }
    if (mCurrentFrame.mbStereoFeatureInjected &&
        EnvFlagEnabled("SMART_DRONE_STEREO_FEATURE_KF_DFX", false)) {
      cerr << "[stereo_feature_kf_dfx] frame_id=" << mCurrentFrame.mnId
           << " keyframes=" << mpAtlas->KeyFramesInMap() << " bootstrap="
           << (IsStereoFeatureBootstrap(mCurrentFrame, mpAtlas,
                                        mnStereoFeatureInitFrameId)
                   ? "Y"
                   : "N")
           << " stabilizing="
           << (IsStereoFeatureStabilizing(mCurrentFrame, mpAtlas,
                                          mnStereoFeatureInitFrameId)
                   ? "Y"
                   : "N")
           << " max_point=" << maxPoint << " valid_depth=" << vDepthIdx.size()
           << " counted=" << nCountedPoints << " created=" << nCreatedPoints
           << " map_points=" << mpAtlas->MapPointsInMap() << "\n";
    }
  }

  InsertLocalMappingKeyFrame(mpLocalMappingBackend.get(), pKF);

  mpLocalMappingBackend->SetNotStop(false);

  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints() {
  // Do not search map points already matched
  for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(),
                                    vend = mCurrentFrame.mvpMapPoints.end();
       vit != vend; vit++) {
    MapPoint *pMP = *vit;
    if (pMP) {
      if (pMP->isBad()) {
        *vit = static_cast<MapPoint *>(NULL);
      } else {
        pMP->IncreaseVisible();
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        pMP->mbTrackInView = false;
        pMP->mbTrackInViewR = false;
      }
    }
  }

  int nToMatch = 0;
  // Project points in frame and check its visibility
  for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(),
                                    vend = mvpLocalMapPoints.end();
       vit != vend; vit++) {
    MapPoint *pMP = *vit;

    if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
      continue;
    if (pMP->isBad())
      continue;
    // Project (this fills MapPoint variables for matching)
    if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
      pMP->IncreaseVisible();
      nToMatch++;
    }
    if (pMP->mbTrackInView) {
      mCurrentFrame.mmProjectPoints[pMP->mnId] =
          cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
    }
  }

  if (nToMatch > 0) {
    auto matcher = CreateDefaultOrbFeatureMatcher(
        OrbFeatureMatcherOptions{0.8f, true});
    int th = 1;
    const bool stereoFeatureBootstrap = IsStereoFeatureBootstrap(
        mCurrentFrame, mpAtlas, mnStereoFeatureInitFrameId);
    const bool stereoFeatureStabilizing = IsStereoFeatureStabilizing(
        mCurrentFrame, mpAtlas, mnStereoFeatureInitFrameId);
    const bool stereoFeatureStablePhase =
        mCurrentFrame.mbStereoFeatureInjected && !stereoFeatureBootstrap &&
        !stereoFeatureStabilizing;
    if (mSensor == System::RGBD || mSensor == System::IMU_RGBD)
      th = 3;
    if (mpAtlas->isImuInitialized()) {
      if (mpAtlas->GetCurrentMap()->GetIniertialBA2())
        th = 2;
      else
        th = 6;
    } else if (!mpAtlas->isImuInitialized() &&
               (mSensor == System::IMU_MONOCULAR ||
                mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)) {
      th = 10;
    }

    // If the camera has been relocalised recently, perform a coarser search
    if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
      th = 5;

    if (mState == LOST ||
        mState == RECENTLY_LOST) // Lost for less than 1 second
      th = 15;                   // 15
    else if (stereoFeatureStabilizing)
      th = std::max(th, 7);
    else if (stereoFeatureBootstrap)
      th = std::max(th, 5);
    else if (stereoFeatureStablePhase)
      th = std::max(th, StereoFeatureStableLocalSearchTh());

    const OrbLocalMappingStatus localMappingStatus =
        GetLocalMappingStatus(mpLocalMappingBackend.get());
    matcher->SearchFrameMapByProjection(
        OrbFrameMapProjectionRequest{
            &mCurrentFrame, &mvpLocalMapPoints, static_cast<float>(th),
            localMappingStatus.farPoints,
            localMappingStatus.farPointThreshold});
  }
}

void Tracking::UpdateLocalMap() {
  // This is for visualization
  mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

  // Update
  UpdateLocalKeyFrames();
  UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints() {
  mvpLocalMapPoints.clear();
  mvpLocalMapPoints.reserve(mvpLocalKeyFrames.size() * 200);
  int count_pts = 0;

  auto appendMapPointsFromKeyFrame = [this, &count_pts](KeyFrame *pKF) {
    const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

    for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(),
                                            itEndMP = vpMPs.end();
         itMP != itEndMP; itMP++) {

      MapPoint *pMP = *itMP;
      if (!pMP)
        continue;
      if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
        continue;
      if (!pMP->isBad()) {
        count_pts++;
        mvpLocalMapPoints.push_back(pMP);
        pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
      }
    }
  };

  const bool stableLocalOrder = StableTrackingLocalOrderEnabled();
  if (stableLocalOrder) {
    std::sort(mvpLocalKeyFrames.begin(), mvpLocalKeyFrames.end(),
              KeyFrameIdLess);
    for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(),
                                            itEndKF = mvpLocalKeyFrames.end();
         itKF != itEndKF; ++itKF)
      appendMapPointsFromKeyFrame(*itKF);
  } else {
    for (vector<KeyFrame *>::const_reverse_iterator
             itKF = mvpLocalKeyFrames.rbegin(),
             itEndKF = mvpLocalKeyFrames.rend();
         itKF != itEndKF; ++itKF)
      appendMapPointsFromKeyFrame(*itKF);
  }
  if (stableLocalOrder)
    std::sort(mvpLocalMapPoints.begin(), mvpLocalMapPoints.end(),
              MapPointIdLess);
  mLocalMapPointHash = HashMapPointSequence(mvpLocalMapPoints);
}

void Tracking::UpdateLocalKeyFrames() {
  // Each map point vote for the keyframes in which it has been observed
  map<KeyFrame *, int> keyframeCounter;
  if (!mpAtlas->isImuInitialized() ||
      (mCurrentFrame.mnId < mnLastRelocFrameId + 2)) {
    for (int i = 0; i < mCurrentFrame.N; i++) {
      MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
      if (pMP) {
        if (!pMP->isBad()) {
          const map<KeyFrame *, tuple<int, int>> observations =
              pMP->GetObservations();
          for (map<KeyFrame *, tuple<int, int>>::const_iterator
                   it = observations.begin(),
                   itend = observations.end();
               it != itend; it++)
            keyframeCounter[it->first]++;
        } else {
          mCurrentFrame.mvpMapPoints[i] = NULL;
        }
      }
    }
  } else {
    for (int i = 0; i < mLastFrame.N; i++) {
      // Using lastframe since current frame has not matches yet
      if (mLastFrame.mvpMapPoints[i]) {
        MapPoint *pMP = mLastFrame.mvpMapPoints[i];
        if (!pMP)
          continue;
        if (!pMP->isBad()) {
          const map<KeyFrame *, tuple<int, int>> observations =
              pMP->GetObservations();
          for (map<KeyFrame *, tuple<int, int>>::const_iterator
                   it = observations.begin(),
                   itend = observations.end();
               it != itend; it++)
            keyframeCounter[it->first]++;
        } else {
          mLastFrame.mvpMapPoints[i] = NULL;
        }
      }
    }
  }

  int max = 0;
  KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

  mvpLocalKeyFrames.clear();
  mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

  // All keyframes that observe a map point are included in the local map. Also
  // check which keyframe shares most points
  for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(),
                                            itEnd = keyframeCounter.end();
       it != itEnd; it++) {
    KeyFrame *pKF = it->first;

    if (pKF->isBad())
      continue;

    if (it->second > max ||
        (StableTrackingLocalOrderEnabled() && it->second == max &&
         pKFmax != nullptr && pKF->mnId < pKFmax->mnId)) {
      max = it->second;
      pKFmax = pKF;
    }

    mvpLocalKeyFrames.push_back(pKF);
    pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
  }
  const bool stableLocalOrder = StableTrackingLocalOrderEnabled();
  if (stableLocalOrder)
    std::sort(mvpLocalKeyFrames.begin(), mvpLocalKeyFrames.end(),
              KeyFrameIdLess);

  // Include also some not-already-included keyframes that are neighbors to
  // already-included keyframes
  for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(),
                                          itEndKF = mvpLocalKeyFrames.end();
       itKF != itEndKF; itKF++) {
    // Limit the number of keyframes
    if (mvpLocalKeyFrames.size() > 80) // 80
      break;

    KeyFrame *pKF = *itKF;

    const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

    for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(),
                                            itEndNeighKF = vNeighs.end();
         itNeighKF != itEndNeighKF; itNeighKF++) {
      KeyFrame *pNeighKF = *itNeighKF;
      if (!pNeighKF->isBad()) {
        if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
          mvpLocalKeyFrames.push_back(pNeighKF);
          pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
          break;
        }
      }
    }

    const set<KeyFrame *> spChilds = pKF->GetChilds();
    vector<KeyFrame *> vChilds(spChilds.begin(), spChilds.end());
    if (stableLocalOrder)
      std::sort(vChilds.begin(), vChilds.end(), KeyFrameIdLess);
    for (vector<KeyFrame *>::const_iterator sit = vChilds.begin(),
                                            send = vChilds.end();
         sit != send; sit++) {
      KeyFrame *pChildKF = *sit;
      if (!pChildKF->isBad()) {
        if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
          mvpLocalKeyFrames.push_back(pChildKF);
          pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
          break;
        }
      }
    }

    KeyFrame *pParent = pKF->GetParent();
    if (pParent) {
      if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
        mvpLocalKeyFrames.push_back(pParent);
        pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        break;
      }
    }
  }
  if (stableLocalOrder)
    std::sort(mvpLocalKeyFrames.begin(), mvpLocalKeyFrames.end(),
              KeyFrameIdLess);

  // Add 10 last temporal KFs (mainly for IMU)
  if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ||
       mSensor == System::IMU_RGBD) &&
      mvpLocalKeyFrames.size() < 80) {
    KeyFrame *tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

    const int Nd = 20;
    for (int i = 0; i < Nd; i++) {
      if (!tempKeyFrame)
        break;
      if (tempKeyFrame->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
        mvpLocalKeyFrames.push_back(tempKeyFrame);
        tempKeyFrame->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        tempKeyFrame = tempKeyFrame->mPrevKF;
      }
    }
  }

  if (pKFmax) {
    mpReferenceKF = pKFmax;
    mCurrentFrame.mpReferenceKF = mpReferenceKF;
  }
}

bool Tracking::Relocalization() {
  Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
  // Compute Bag of Words Vector
  mCurrentFrame.ComputeBoW();

  // Relocalization is performed when tracking is lost
  // Track Lost: Query KeyFrame Database for keyframe candidates for
  // relocalisation
  vector<KeyFrame *> vpCandidateKFs =
      mpPlaceRecognitionBackend
          ? mpPlaceRecognitionBackend->DetectRelocalizationCandidates(
                OrbPlaceRecognitionRelocalizationRequest{
                    &mCurrentFrame, mpAtlas->GetCurrentMap()})
          : vector<KeyFrame *>{};

  if (vpCandidateKFs.empty()) {
    Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
    return false;
  }

  const int nKFs = vpCandidateKFs.size();

  // We perform first an ORB matching with each candidate
  // If enough matches are found we setup a PnP solver
  auto matcher = CreateDefaultOrbFeatureMatcher(
      OrbFeatureMatcherOptions{0.75f, true});

  vector<MLPnPsolver *> vpMLPnPsolvers;
  vpMLPnPsolvers.resize(nKFs);

  vector<vector<MapPoint *>> vvpMapPointMatches;
  vvpMapPointMatches.resize(nKFs);

  vector<bool> vbDiscarded;
  vbDiscarded.resize(nKFs);

  int nCandidates = 0;

  for (int i = 0; i < nKFs; i++) {
    KeyFrame *pKF = vpCandidateKFs[i];
    if (pKF->isBad())
      vbDiscarded[i] = true;
    else {
      int nmatches = matcher->SearchFrameByBoW(
          OrbFrameBoWMatchRequest{pKF, &mCurrentFrame,
                                  &vvpMapPointMatches[i]});
      if (nmatches < 15) {
        vbDiscarded[i] = true;
        continue;
      } else {
        MLPnPsolver *pSolver =
            new MLPnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
        pSolver->SetRansacParameters(
            0.99, 10, 300, 6, 0.5,
            5.991); // This solver needs at least 6 points
        vpMLPnPsolvers[i] = pSolver;
        nCandidates++;
      }
    }
  }
  // Alternatively perform some iterations of P4P RANSAC
  // Until we found a camera pose supported by enough inliers
  bool bMatch = false;
  auto projectionMatcher = CreateDefaultOrbFeatureMatcher(
      OrbFeatureMatcherOptions{0.9f, true});

  while (nCandidates > 0 && !bMatch) {
    for (int i = 0; i < nKFs; i++) {
      if (vbDiscarded[i])
        continue;

      // Perform 5 Ransac Iterations
      vector<bool> vbInliers;
      int nInliers;
      bool bNoMore;

      MLPnPsolver *pSolver = vpMLPnPsolvers[i];
      Eigen::Matrix4f eigTcw;
      bool bTcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers, eigTcw);

      // If Ransac reachs max. iterations discard keyframe
      if (bNoMore) {
        vbDiscarded[i] = true;
        nCandidates--;
      }

      // If a Camera Pose is computed, optimize
      if (bTcw) {
        Sophus::SE3f Tcw(eigTcw);
        mCurrentFrame.SetPose(Tcw);

        set<MapPoint *> sFound;

        const int np = vbInliers.size();

        for (int j = 0; j < np; j++) {
          if (vbInliers[j]) {
            mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
            sFound.insert(vvpMapPointMatches[i][j]);
          } else
            mCurrentFrame.mvpMapPoints[j] = NULL;
        }

        int nGood =
            OptimizeCurrentFramePose(mpOptimizationBackend.get(), mCurrentFrame);

        if (nGood < 10)
          continue;

        for (int io = 0; io < mCurrentFrame.N; io++)
          if (mCurrentFrame.mvbOutlier[io])
            mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

        // If few inliers, search by projection in a coarse window and optimize
        // again
        if (nGood < 50) {
          int nadditional =
              projectionMatcher->SearchFrameKeyFrameByProjection(
                  OrbFrameKeyFrameProjectionRequest{
                      &mCurrentFrame, vpCandidateKFs[i], &sFound, 10.0f,
                      100});

          if (nadditional + nGood >= 50) {
            nGood = OptimizeCurrentFramePose(mpOptimizationBackend.get(),
                                             mCurrentFrame);

            // If many inliers but still not enough, search by projection again
            // in a narrower window the camera has been already optimized with
            // many points
            if (nGood > 30 && nGood < 50) {
              sFound.clear();
              for (int ip = 0; ip < mCurrentFrame.N; ip++)
                if (mCurrentFrame.mvpMapPoints[ip])
                  sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
              nadditional =
                  projectionMatcher->SearchFrameKeyFrameByProjection(
                      OrbFrameKeyFrameProjectionRequest{
                          &mCurrentFrame, vpCandidateKFs[i], &sFound, 3.0f,
                          64});

              // Final optimization
              if (nGood + nadditional >= 50) {
                nGood = OptimizeCurrentFramePose(mpOptimizationBackend.get(),
                                                 mCurrentFrame);

                for (int io = 0; io < mCurrentFrame.N; io++)
                  if (mCurrentFrame.mvbOutlier[io])
                    mCurrentFrame.mvpMapPoints[io] = NULL;
              }
            }
          }
        }

        // If the pose is supported by enough inliers stop ransacs and continue
        if (nGood >= 50) {
          bMatch = true;
          break;
        }
      }
    }
  }

  if (!bMatch) {
    for (MLPnPsolver *pSolver : vpMLPnPsolvers)
      delete pSolver;
    return false;
  } else {
    mnLastRelocFrameId = mCurrentFrame.mnId;
    cout << "Relocalized!!" << endl;
    for (MLPnPsolver *pSolver : vpMLPnPsolvers)
      delete pSolver;
    return true;
  }
}

void Tracking::Reset(bool bLocMap) {
  Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

  // Reset Local Mapping
  if (!bLocMap) {
    Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
    ResetLocalMappingBackend(mpLocalMappingBackend.get(), nullptr, false);
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
  }

  // Reset Loop Closing
  Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
  ResetLoopClosingBackend(mpLoopClosingBackend.get(), nullptr, false);
  Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

  // Clear BoW Database
  Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
  if (mpPlaceRecognitionBackend != nullptr) {
    mpPlaceRecognitionBackend->Clear();
  }
  Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

  // Clear Map (this erase MapPoints and KeyFrames)
  mpAtlas->clearAtlas();
  mpAtlas->CreateNewMap();
  if (mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR ||
      mSensor == System::IMU_RGBD)
    mpAtlas->SetInertialSensor();
  mnInitialFrameId = 0;

  KeyFrame::nNextId = 0;
  Frame::nNextId = 0;
  mState = NO_IMAGES_YET;

  mbReadyToInitializate = false;
  mbSetInit = false;

  mlRelativeFramePoses.clear();
  mlpReferences.clear();
  mlFrameTimes.clear();
  mlbLost.clear();
  mCurrentFrame = Frame();
  mnLastRelocFrameId = 0;
  mLastFrame = Frame();
  mpReferenceKF = static_cast<KeyFrame *>(NULL);
  mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
  mvIniMatches.clear();

  Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

void Tracking::ResetActiveMap(bool bLocMap) {
  Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);

  Map *pMap = mpAtlas->GetCurrentMap();

  if (!bLocMap) {
    Verbose::PrintMess("Reseting Local Mapper...",
                       Verbose::VERBOSITY_VERY_VERBOSE);
    ResetLocalMappingBackend(mpLocalMappingBackend.get(), pMap, true);
    Verbose::PrintMess("done", Verbose::VERBOSITY_VERY_VERBOSE);
  }

  // Reset Loop Closing
  Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
  ResetLoopClosingBackend(mpLoopClosingBackend.get(), pMap, true);
  Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

  // Clear BoW Database
  Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
  if (mpPlaceRecognitionBackend != nullptr) {
    // Only clear the active map references.
    mpPlaceRecognitionBackend->ClearMap(OrbPlaceRecognitionResetRequest{pMap});
  }
  Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

  // Clear Map (this erase MapPoints and KeyFrames)
  mpAtlas->clearMap();

  // KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
  // Frame::nNextId = mnLastInitFrameId;
  mnLastInitFrameId = Frame::nNextId;
  // mnLastRelocFrameId = mnLastInitFrameId;
  mState = NO_IMAGES_YET; // NOT_INITIALIZED;

  mbReadyToInitializate = false;

  list<bool> lbLost;
  unsigned int index = mnFirstFrameId;
  cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
  for (Map *pMap : mpAtlas->GetAllMaps()) {
    if (pMap->GetAllKeyFrames().size() > 0) {
      if (index > pMap->GetLowerKFID())
        index = pMap->GetLowerKFID();
    }
  }

  int num_lost = 0;
  cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

  for (list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end();
       ilbL++) {
    if (index < mnInitialFrameId)
      lbLost.push_back(*ilbL);
    else {
      lbLost.push_back(true);
      num_lost += 1;
    }

    index++;
  }
  cout << num_lost << " Frames set to lost" << endl;

  mlbLost = lbLost;

  mnInitialFrameId = mCurrentFrame.mnId;
  mnLastRelocFrameId = mCurrentFrame.mnId;

  mCurrentFrame = Frame();
  mLastFrame = Frame();
  mpReferenceKF = static_cast<KeyFrame *>(NULL);
  mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
  mvIniMatches.clear();

  mbVelocity = false;

  Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

vector<MapPoint *> Tracking::GetLocalMapMPS() { return mvpLocalMapPoints; }

void Tracking::ChangeCalibration(const string &strSettingPath) {
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
  float fx = fSettings["Camera.fx"];
  float fy = fSettings["Camera.fy"];
  float cx = fSettings["Camera.cx"];
  float cy = fSettings["Camera.cy"];

  mK_.setIdentity();
  mK_(0, 0) = fx;
  mK_(1, 1) = fy;
  mK_(0, 2) = cx;
  mK_(1, 2) = cy;

  cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
  K.at<float>(0, 0) = fx;
  K.at<float>(1, 1) = fy;
  K.at<float>(0, 2) = cx;
  K.at<float>(1, 2) = cy;
  K.copyTo(mK);

  cv::Mat DistCoef(4, 1, CV_32F);
  DistCoef.at<float>(0) = fSettings["Camera.k1"];
  DistCoef.at<float>(1) = fSettings["Camera.k2"];
  DistCoef.at<float>(2) = fSettings["Camera.p1"];
  DistCoef.at<float>(3) = fSettings["Camera.p2"];
  const float k3 = fSettings["Camera.k3"];
  if (k3 != 0) {
    DistCoef.resize(5);
    DistCoef.at<float>(4) = k3;
  }
  DistCoef.copyTo(mDistCoef);

  mbf = fSettings["Camera.bf"];

  Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag) { mbOnlyTracking = flag; }

void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b,
                              KeyFrame *pCurrentKeyFrame) {
  Map *pMap = pCurrentKeyFrame->GetMap();
  unsigned int index = mnFirstFrameId;
  list<ORB_SLAM3::KeyFrame *>::iterator lRit = mlpReferences.begin();
  list<bool>::iterator lbL = mlbLost.begin();
  for (auto lit = mlRelativeFramePoses.begin(),
            lend = mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lbL++) {
    if (*lbL)
      continue;

    KeyFrame *pKF = *lRit;

    while (pKF->isBad()) {
      pKF = pKF->GetParent();
    }

    if (pKF->GetMap() == pMap) {
      (*lit).translation() *= s;
    }
  }

  mLastBias = b;

  mpLastKeyFrame = pCurrentKeyFrame;

  mLastFrame.SetNewBias(mLastBias);
  mCurrentFrame.SetNewBias(mLastBias);

  if (!mCurrentFrame.imuIsPreintegrated()) {
    return;
  }

  if (mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId) {
    mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
                                  mLastFrame.mpLastKeyFrame->GetImuPosition(),
                                  mLastFrame.mpLastKeyFrame->GetVelocity());
  } else {
    const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
    const Eigen::Vector3f twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
    const Eigen::Matrix3f Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
    const Eigen::Vector3f Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
    float t12 = mLastFrame.mpImuPreintegrated->dT;

    mLastFrame.SetImuPoseVelocity(
        IMU::NormalizeRotation(
            Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
        twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
        Vwb1 + Gz * t12 +
            Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
  }

  if (mCurrentFrame.mpImuPreintegrated) {
    const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

    const Eigen::Vector3f twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
    const Eigen::Matrix3f Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
    const Eigen::Vector3f Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
    float t12 = mCurrentFrame.mpImuPreintegrated->dT;

    mCurrentFrame.SetImuPoseVelocity(
        IMU::NormalizeRotation(
            Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
        twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
        Vwb1 + Gz * t12 +
            Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
  }

  mnFirstImuFrameId = mCurrentFrame.mnId;
}

void Tracking::NewDataset() { mnNumDataset++; }

int Tracking::GetNumberDataset() { return mnNumDataset; }

int Tracking::GetMatchesInliers() { return mnMatchesInliers; }

size_t Tracking::GetTrackedMapPointCount() const {
  return static_cast<size_t>(CountTrackedMapPoints(mCurrentFrame));
}

size_t Tracking::GetLocalMapPointCount() const {
  size_t count = 0;
  for (MapPoint *pMP : mvpLocalMapPoints) {
    if (pMP && !pMP->isBad())
      ++count;
  }
  return count;
}

uint64_t Tracking::GetLocalMapPointHash() const { return mLocalMapPointHash; }

uint64_t Tracking::GetMatchedMapPointHashBeforePoseOptimization() const {
  return mMatchedMapPointHashBeforePoseOptimization;
}

uint64_t Tracking::GetTrackedMapPointHash() const {
  return mTrackedMapPointHash;
}

void Tracking::SaveSubTrajectory(string strNameFile_frames,
                                 string strNameFile_kf, string strFolder) {
  mpSystem->SaveTrajectoryEuRoC(strFolder + strNameFile_frames);
  // mpSystem->SaveKeyFrameTrajectoryEuRoC(strFolder + strNameFile_kf);
}

void Tracking::SaveSubTrajectory(string strNameFile_frames,
                                 string strNameFile_kf, Map *pMap) {
  mpSystem->SaveTrajectoryEuRoC(strNameFile_frames, pMap);
  if (!strNameFile_kf.empty())
    mpSystem->SaveKeyFrameTrajectoryEuRoC(strNameFile_kf, pMap);
}

float Tracking::GetImageScale() { return mImageScale; }

#ifdef REGISTER_LOOP
void Tracking::RequestStop() {
  mbStopRequested = true;
}

bool Tracking::Stop() {
  if (mbStopRequested && !mbNotStop) {
    mbStopped = true;
    cout << "Tracking STOP" << endl;
    return true;
  }

  return false;
}

bool Tracking::stopRequested() {
  return mbStopRequested;
}

bool Tracking::isStopped() {
  return mbStopped;
}

void Tracking::Release() {
  mbStopped = false;
  mbStopRequested = false;
}
#endif

} // namespace ORB_SLAM3
