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

#ifndef SYSTEM_H
#define SYSTEM_H

#include <mutex>
#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "core/ports/slam_backend_modules.h"
#include "core/ports/slam_backend_state.h"
#include "Verbose.h"
#include "core/ports/tracked_visual_data.h"
#include "core/ports/visual_feature_data.h"
// #include "FrameDrawer.h"
// #include "MapDrawer.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
// #include "Viewer.h"
#include "ImuTypes.h"
#include "Settings.h"

namespace ORB_SLAM3 {

using smartdrone::core::ports::SlamBackendStats;
using smartdrone::core::ports::SlamBackendWaitStats;
using smartdrone::core::ports::SlamMapSummary;
using smartdrone::core::ports::SlamBackendLoopClosureRequest;
using smartdrone::core::ports::SlamBackendLoopClosureResult;
using smartdrone::core::ports::SlamBackendMappingRequest;
using smartdrone::core::ports::SlamBackendMappingResult;
using smartdrone::core::ports::SlamBackendOptimizationRequest;
using smartdrone::core::ports::SlamBackendOptimizationResult;
using smartdrone::core::ports::TrackedFeatureSnapshot;
using smartdrone::core::ports::TrackedPointCloudSnapshot;
using smartdrone::core::ports::TrackedVisualData;
using smartdrone::core::ports::TrackedVisualSummary;
using MonoFeatureFrameData = smartdrone::core::ports::VisualKeypointFeatureSet;
using StereoFeatureFrameData =
    smartdrone::core::ports::StereoFeatureObservationPacket;

// class Viewer;
// class FrameDrawer;
// class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;
class ORBextractor;
class IOrbLocalMappingBackend;
class IOrbLoopClosingBackend;
class IOrbOptimizationBackend;
class IOrbTrackingBackend;

struct LocalMappingWaitStats {
  bool requested = false;
  bool timedOut = false;
  bool acceptingBefore = false;
  bool acceptingAfter = false;
  int queueBefore = 0;
  int queueAfter = 0;
  int timeoutMs = 0;
  double waitMs = 0.0;
};

class System {
public:
  // Input sensor
  enum eSensor {
    MONOCULAR = 0,
    STEREO = 1,
    RGBD = 2,
    IMU_MONOCULAR = 3,
    IMU_STEREO = 4,
    IMU_RGBD = 5,
  };

  // File type
  enum FileType {
    TEXT_FILE = 0,
    BINARY_FILE = 1,
  };

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and
  // Viewer threads.
  System(const string &strVocFile, const string &strSettingsFile,
         const eSensor sensor, const bool bUseViewer = false,
         const int initFr = 0, const string &strSequence = std::string());
  ~System();

  // Proccess the given stereo frame. Images must be synchronized and rectified.
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Returns the camera pose (empty if tracking fails).
  Sophus::SE3f
  TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight,
              const double &timestamp,
              const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
              string filename = "");
  Sophus::SE3f TrackStereoWithFeatures(
      const cv::Mat &imLeft, const cv::Mat &imRight,
      const StereoFeatureFrameData &features, const double &timestamp,
      const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
      string filename = "");
  bool PrepareStereoImagesForTracking(const cv::Mat &imLeft,
                                      const cv::Mat &imRight,
                                      cv::Mat &imLeftPrepared,
                                      cv::Mat &imRightPrepared) const;
  Sophus::SE3f TrackStereoPreparedWithFeatures(
      const cv::Mat &imLeftPrepared, const cv::Mat &imRightPrepared,
      const StereoFeatureFrameData &features, const double &timestamp,
      const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
      string filename = "");

  // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
  // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Input depthmap: Float (CV_32F). Returns the camera pose (empty
  // if tracking fails).
  Sophus::SE3f
  TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp,
            const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
            string filename = "");

  // Proccess the given monocular frame and optionally imu data
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Returns the camera pose (empty if tracking fails).
  Sophus::SE3f
  TrackMonocular(const cv::Mat &im, const double &timestamp,
                 const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
                 string filename = "");
  Sophus::SE3f TrackMonocularWithFeatures(
      const cv::Mat &im, const MonoFeatureFrameData &features,
      const double &timestamp,
      const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
      string filename = "");

  // This stops local mapping thread (map building) and performs only camera
  // tracking.
  void ActivateLocalizationMode();
  // This resumes local mapping thread and performs SLAM again.
  void DeactivateLocalizationMode();

  // Returns true if there have been a big map change (loop closure, global BA)
  // since last call to this function
  bool MapChanged();

  // Reset the system (clear Atlas or the active map)
  void Reset();
  void ResetActiveMap();

  // All threads will be requested to finish.
  // It waits until all threads have finished.
  // This function must be called before saving the trajectory.
  void Shutdown();
  bool isShutDown();

  // Save camera trajectory in the TUM RGB-D dataset format.
  // Only for stereo and RGB-D. This method does not work for monocular.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
  void SaveTrajectoryTUM(const string &filename);

  // Save keyframe poses in the TUM RGB-D dataset format.
  // This method works for all sensor input.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
  void SaveKeyFrameTrajectoryTUM(const string &filename);

  void SaveTrajectoryEuRoC(const string &filename);
  void SaveKeyFrameTrajectoryEuRoC(const string &filename);

  void SaveTrajectoryEuRoC(const string &filename, Map *pMap);
  void SaveKeyFrameTrajectoryEuRoC(const string &filename, Map *pMap);
  bool GetLatestFrameTrajectoryPoseEuRoC(Sophus::SE3f &twc,
                                         double *timestamp = nullptr,
                                         bool *lost = nullptr) const;

  // Save data used for initialization debug
  void SaveDebugData(const int &iniIdx);

  // Save camera trajectory in the KITTI dataset format.
  // Only for stereo and RGB-D. This method does not work for monocular.
  // Call first Shutdown()
  // See format details at:
  // http://www.cvlibs.net/datasets/kitti/eval_odometry.php
  void SaveTrajectoryKITTI(const string &filename);

  // TODO: Save/Load functions
  // SaveMap(const string &filename);
  // LoadMap(const string &filename);

  // Information from most recent processed frame
  // You can call this right after TrackMonocular (or stereo or RGBD)
  int GetTrackingState();
  ORBextractor *GetLeftORBExtractor() const;
  ORBextractor *GetRightORBExtractor() const;
  unsigned long GetCurrentMapId();
  bool CanUseStereoFeatureInjection() const;
  int GetMatchesInliers() const;
  size_t GetTrackedMapPointCount() const;
  size_t GetLocalMapPointCount() const;
  uint64_t GetLocalMapPointHash() const;
  uint64_t GetMatchedMapPointHashBeforePoseOptimization() const;
  uint64_t GetTrackedMapPointHash() const;
  SlamMapSummary GetMapSummary() const;
  SlamBackendStats GetBackendStats() const;
  void LogStereoFeatureDfx(uint64_t frameId,
                           const StereoFeatureFrameData &features) const;
  LocalMappingWaitStats GetLastLocalMappingWaitStats() const;
  TrackedVisualSummary GetTrackedVisualSummary() const;
  TrackedFeatureSnapshot ExtractTrackedFeatures(int leftImageWidth,
                                                int leftImageHeight,
                                                int rightImageWidth,
                                                int rightImageHeight);
  TrackedPointCloudSnapshot
  ExtractTrackedPointCloud(size_t maxPointCloudPoints);
  TrackedVisualData
  ExtractTrackedVisualData(int leftImageWidth, int leftImageHeight,
                           int rightImageWidth, int rightImageHeight,
                           bool includePointCloud, size_t maxPointCloudPoints);
  bool OptimizeBackend(const SlamBackendOptimizationRequest &request,
                       SlamBackendOptimizationResult &result);
  bool ApplyLocalMappingOperation(const SlamBackendMappingRequest &request,
                                  SlamBackendMappingResult &result);
  bool ApplyLoopClosingOperation(const SlamBackendLoopClosureRequest &request,
                                 SlamBackendLoopClosureResult &result);
  void WaitForLocalMappingIdleIfRequested();

  // For debugging
  double GetTimeFromIMUInit();
  bool isLost();
  bool isFinished();

  void ChangeDataset();

  float GetImageScale();

#ifdef REGISTER_TIMES
  void InsertRectTime(double &time);
  void InsertResizeTime(double &time);
  void InsertTrackTime(double &time);
#endif

private:
  void SaveAtlas(int type);
  bool LoadAtlas(int type);
  void StoreLocalMappingWaitStats(const LocalMappingWaitStats &stats);

  string CalculateCheckSum(string filename, int type);

  // Input sensor
  eSensor mSensor;

  // ORB vocabulary used for place recognition and feature matching.
  ORBVocabulary *mpVocabulary;

  // KeyFrame database for place recognition (relocalization and loop
  // detection).
  KeyFrameDatabase *mpKeyFrameDatabase;

  // Map structure that stores the pointers to all KeyFrames and MapPoints.
  // Map* mpMap;
  Atlas *mpAtlas;

  // Tracker. It receives a frame and computes the associated camera pose.
  // It also decides when to insert a new keyframe, create some new MapPoints
  // and performs relocalization if tracking fails.
  Tracking *mpTracker;

  // Local Mapper. It manages the local map and performs local bundle
  // adjustment.
  LocalMapping *mpLocalMapper;
  std::unique_ptr<IOrbLocalMappingBackend> mpLocalMappingBackend;
  std::unique_ptr<IOrbTrackingBackend> mpTrackingBackend;

  // Loop Closer. It searches loops with every new keyframe. If there is a loop
  // it performs a pose graph optimization and full bundle adjustment (in a new
  // thread) afterwards.
  LoopClosing *mpLoopCloser;
  std::unique_ptr<IOrbLoopClosingBackend> mpLoopClosingBackend;
  std::unique_ptr<IOrbOptimizationBackend> mpOptimizationBackend;

  // The viewer draws the map and the current camera pose. It uses Pangolin.
  // Viewer* mpViewer;

  // FrameDrawer* mpFrameDrawer;
  // MapDrawer* mpMapDrawer;

  // System threads: Local Mapping, Loop Closing, Viewer.
  // The Tracking thread "lives" in the main execution thread that creates the
  // System object.
  std::thread *mptLocalMapping;
  std::thread *mptLoopClosing;
  std::thread *mptViewer;

  // Reset flag
  std::mutex mMutexReset;
  bool mbReset;
  bool mbResetActiveMap;

  // Change mode flags
  std::mutex mMutexMode;
  bool mbActivateLocalizationMode;
  bool mbDeactivateLocalizationMode;

  // Shutdown flag
  bool mbShutDown;

  // Tracking state
  int mTrackingState;
  std::mutex mMutexState;
  LocalMappingWaitStats mLastLocalMappingWaitStats;
  mutable std::mutex mMutexLocalMappingWaitStats;

  //
  string mStrLoadAtlasFromFile;
  string mStrSaveAtlasToFile;

  string mStrVocabularyFilePath;

  Settings *settings_;
};

} // namespace ORB_SLAM3

#endif // SYSTEM_H
