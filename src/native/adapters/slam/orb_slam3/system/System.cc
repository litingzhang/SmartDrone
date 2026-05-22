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

#include "System.h"
#include "Atlas.h"
#include "Converter.h"
#include "DUtils/Random.h"
#include "LocalMapping.h"
#include "LocalMappingBackend.h"
#include "LoopClosing.h"
#include "LoopClosingBackend.h"
#include "OptimizationBackend.h"
#include "Tracking.h"
#include "TrackingBackend.h"
#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <sys/stat.h>
#include <sys/types.h>
// #include <pangolin/pangolin.h>
#include "features/orb_vocabulary_bridge.h"
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <iomanip>
#include <iterator>
#include <opencv2/imgcodecs.hpp>
#include <openssl/md5.h>
#include <sstream>
#include <string>
#if SMART_DRONE_HAS_VPI
#include <vpi/Image.h>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/Stream.h>
#include <vpi/WarpMap.h>
#include <vpi/algo/Remap.h>
#endif

#ifdef _WIN32
#include <direct.h>
#endif

namespace ORB_SLAM3 {

namespace {

template <typename T> T ClampValue(T value, T minValue, T maxValue) {
  return std::max(minValue, std::min(value, maxValue));
}

#if SMART_DRONE_HAS_VPI
const char *VpiStatusName(VPIStatus status) {
  const char *name = vpiStatusGetName(status);
  return name != nullptr ? name : "VPI_ERROR_UNKNOWN";
}

bool OrbEnvFlagEnabled(const char *name, bool defaultValue) {
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == '\0')
    return defaultValue;

  std::string text(value);
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return !(text == "0" || text == "false" || text == "off" || text == "no");
}

bool FillVpiWarpMapFromOpenCvMaps(const cv::Mat &mapX, const cv::Mat &mapY,
                                  VPIWarpMap &warp) {
  if (mapX.empty() || mapY.empty() || mapX.size() != mapY.size() ||
      mapX.type() != CV_32FC1 || mapY.type() != CV_32FC1) {
    return false;
  }

  vpiWarpMapFreeData(&warp);
  warp = {};
  warp.grid.numHorizRegions = 1;
  warp.grid.numVertRegions = 1;
  warp.grid.regionWidth[0] = static_cast<int16_t>(mapX.cols);
  warp.grid.regionHeight[0] = static_cast<int16_t>(mapX.rows);
  warp.grid.horizInterval[0] = 1;
  warp.grid.vertInterval[0] = 1;

  VPIStatus status = vpiWarpMapAllocData(&warp);
  if (status != VPI_SUCCESS || warp.keypoints == nullptr) {
    std::cerr << "[orb_vpi_remap] warp map allocation failed: "
              << VpiStatusName(status) << "\n";
    return false;
  }

  for (int y = 0; y < mapX.rows; ++y) {
    auto *row = reinterpret_cast<VPIKeypointF32 *>(
        reinterpret_cast<uint8_t *>(warp.keypoints) +
        static_cast<size_t>(y) * warp.pitchBytes);
    for (int x = 0; x < mapX.cols; ++x) {
      row[x].x = mapX.at<float>(y, x);
      row[x].y = mapY.at<float>(y, x);
    }
  }
  return true;
}

cv::Mat DownloadVpiU8Image(VPIImage image) {
  VPIImageData data{};
  VPIStatus status = vpiImageLockData(
      image, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &data);
  if (status != VPI_SUCCESS ||
      data.bufferType != VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR ||
      data.buffer.pitch.numPlanes < 1 ||
      data.buffer.pitch.planes[0].data == nullptr) {
    std::cerr << "[orb_vpi_remap] image lock failed: " << VpiStatusName(status)
              << "\n";
    return {};
  }

  const auto &plane = data.buffer.pitch.planes[0];
  cv::Mat view(plane.height, plane.width, CV_8UC1, plane.data,
               static_cast<size_t>(plane.pitchBytes));
  cv::Mat out = view.clone();
  vpiImageUnlock(image);
  return out;
}

struct OrbVpiRemapState {
  ~OrbVpiRemapState() {
    if (leftRemapPayload != nullptr)
      vpiPayloadDestroy(leftRemapPayload);
    if (rightRemapPayload != nullptr)
      vpiPayloadDestroy(rightRemapPayload);
    if (leftRect != nullptr)
      vpiImageDestroy(leftRect);
    if (rightRect != nullptr)
      vpiImageDestroy(rightRect);
    if (stream != nullptr)
      vpiStreamDestroy(stream);
    vpiWarpMapFreeData(&leftWarp);
    vpiWarpMapFreeData(&rightWarp);
  }

  int width = 0;
  int height = 0;
  VPIStream stream{nullptr};
  VPIPayload leftRemapPayload{nullptr};
  VPIPayload rightRemapPayload{nullptr};
  VPIImage leftRect{nullptr};
  VPIImage rightRect{nullptr};
  VPIWarpMap leftWarp{};
  VPIWarpMap rightWarp{};
};

bool EnsureOrbVpiRemapState(std::unique_ptr<OrbVpiRemapState> &state,
                            const cv::Size &size, const cv::Mat &map1x,
                            const cv::Mat &map1y, const cv::Mat &map2x,
                            const cv::Mat &map2y) {
  if (state && state->width == size.width && state->height == size.height)
    return true;

  std::unique_ptr<OrbVpiRemapState> next(new OrbVpiRemapState());
  next->width = size.width;
  next->height = size.height;

  VPIStatus status = vpiStreamCreate(VPI_BACKEND_CUDA, &next->stream);
  if (status != VPI_SUCCESS) {
    std::cerr << "[orb_vpi_remap] stream create failed: "
              << VpiStatusName(status) << "\n";
    return false;
  }

  if (!FillVpiWarpMapFromOpenCvMaps(map1x, map1y, next->leftWarp) ||
      !FillVpiWarpMapFromOpenCvMaps(map2x, map2y, next->rightWarp)) {
    return false;
  }

  status = vpiCreateRemap(VPI_BACKEND_CUDA, &next->leftWarp,
                          &next->leftRemapPayload);
  if (status == VPI_SUCCESS)
    status = vpiCreateRemap(VPI_BACKEND_CUDA, &next->rightWarp,
                            &next->rightRemapPayload);
  if (status != VPI_SUCCESS) {
    std::cerr << "[orb_vpi_remap] payload create failed: "
              << VpiStatusName(status) << "\n";
    return false;
  }

  const uint64_t imageBackends = VPI_BACKEND_CUDA | VPI_BACKEND_CPU;
  status = vpiImageCreate(size.width, size.height, VPI_IMAGE_FORMAT_U8,
                          imageBackends, &next->leftRect);
  if (status == VPI_SUCCESS)
    status = vpiImageCreate(size.width, size.height, VPI_IMAGE_FORMAT_U8,
                            imageBackends, &next->rightRect);
  if (status != VPI_SUCCESS) {
    std::cerr << "[orb_vpi_remap] output image create failed: "
              << VpiStatusName(status) << "\n";
    return false;
  }

  std::cerr << "[orb_vpi_remap] backend=vpi_cuda size=" << size.width << "x"
            << size.height << "\n";
  state = std::move(next);
  return true;
}

bool VpiRemapStereoForOrb(const cv::Mat &leftRaw, const cv::Mat &rightRaw,
                          cv::Mat &leftRect, cv::Mat &rightRect,
                          const cv::Mat &map1x, const cv::Mat &map1y,
                          const cv::Mat &map2x, const cv::Mat &map2y) {
  if (!OrbEnvFlagEnabled("SMART_DRONE_ORB_VPI_REMAP", false))
    return false;
  if (leftRaw.empty() || rightRaw.empty() || leftRaw.type() != CV_8UC1 ||
      rightRaw.type() != CV_8UC1 || leftRaw.size() != rightRaw.size())
    return false;

  static std::mutex stateMutex;
  static std::unique_ptr<OrbVpiRemapState> state;
  std::lock_guard<std::mutex> lock(stateMutex);

  if (!EnsureOrbVpiRemapState(state, leftRaw.size(), map1x, map1y, map2x,
                              map2y))
    return false;

  VPIImage leftWrapper = nullptr;
  VPIImage rightWrapper = nullptr;
  VPIStatus status = vpiImageCreateWrapperOpenCVMat(
      leftRaw, VPI_IMAGE_FORMAT_U8, VPI_BACKEND_CUDA, &leftWrapper);
  if (status == VPI_SUCCESS)
    status = vpiImageCreateWrapperOpenCVMat(rightRaw, VPI_IMAGE_FORMAT_U8,
                                            VPI_BACKEND_CUDA, &rightWrapper);
  if (status == VPI_SUCCESS)
    status = vpiSubmitRemap(
        state->stream, VPI_BACKEND_CUDA, state->leftRemapPayload, leftWrapper,
        state->leftRect, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0);
  if (status == VPI_SUCCESS)
    status = vpiSubmitRemap(
        state->stream, VPI_BACKEND_CUDA, state->rightRemapPayload, rightWrapper,
        state->rightRect, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0);
  if (status == VPI_SUCCESS)
    status = vpiStreamSync(state->stream);

  if (leftWrapper != nullptr)
    vpiImageDestroy(leftWrapper);
  if (rightWrapper != nullptr)
    vpiImageDestroy(rightWrapper);
  if (status != VPI_SUCCESS) {
    std::cerr << "[orb_vpi_remap] submit failed: " << VpiStatusName(status)
              << "; fallback=cpu_remap\n";
    state.reset();
    return false;
  }

  leftRect = DownloadVpiU8Image(state->leftRect);
  rightRect = DownloadVpiU8Image(state->rightRect);
  return !leftRect.empty() && !rightRect.empty();
}
#else
bool VpiRemapStereoForOrb(const cv::Mat &, const cv::Mat &, cv::Mat &,
                          cv::Mat &, const cv::Mat &, const cv::Mat &,
                          const cv::Mat &, const cv::Mat &) {
  return false;
}
#endif

bool EnvFlagEnabled(const char *name) {
  const char *value = std::getenv(name);
  return value && value[0] != '\0' && std::string(value) != "0";
}

bool EnvFlagEnabled(const char *name, bool fallback) {
  const char *value = std::getenv(name);
  if (!value || value[0] == '\0')
    return fallback;

  std::string text(value);
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return !(text == "0" || text == "false" || text == "off" || text == "no");
}

int EnvIntClamped(const char *name, int fallback, int minValue, int maxValue) {
  const char *value = std::getenv(name);
  if (!value || value[0] == '\0')
    return fallback;

  char *end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value)
    return fallback;

  return ClampValue(static_cast<int>(parsed), minValue, maxValue);
}

LocalMappingWaitStats WaitForLocalMappingIdle(
    IOrbLocalMappingBackend *localMapper) {
  LocalMappingWaitStats stats;
  if (localMapper == nullptr)
    return stats;

  const int maxSteps = EnvIntClamped(
      "SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE_STEPS", 8, 1, 256);
  const int timeoutMs = maxSteps;
  stats.timeoutMs = timeoutMs;
  OrbLocalMappingStatus status = localMapper->Status();
  stats.queueBefore = status.keyframesInQueue;
  stats.acceptingBefore = status.acceptingKeyframes;
  if (!EnvFlagEnabled("SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE", false)) {
    stats.queueAfter = stats.queueBefore;
    stats.acceptingAfter = stats.acceptingBefore;
    return stats;
  }

  stats.requested = true;
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < maxSteps && (status.keyframesInQueue > 0 || !status.acceptingKeyframes); ++i) {
    localMapper->Step();
    status = localMapper->Status();
  }
  stats.timedOut = status.keyframesInQueue > 0 || !status.acceptingKeyframes;
  const auto end = std::chrono::steady_clock::now();
  stats.waitMs = std::chrono::duration<double, std::milli>(end - start).count();
  status = localMapper->Status();
  stats.queueAfter = status.keyframesInQueue;
  stats.acceptingAfter = status.acceptingKeyframes;
  return stats;
}

void RequestLocalMappingStop(IOrbLocalMappingBackend *localMapper) {
  if (localMapper != nullptr) {
    localMapper->RequestStop();
  }
}

void WaitLocalMappingStopped(IOrbLocalMappingBackend *localMapper) {
  if (localMapper == nullptr) {
    return;
  }
  constexpr int kStopDrainMaxSteps = 256;
  for (int i = 0; i < kStopDrainMaxSteps; ++i) {
    if (localMapper->Status().stopped) {
      return;
    }
    localMapper->Step();
  }
}

void ReleaseLocalMapping(IOrbLocalMappingBackend *localMapper) {
  if (localMapper != nullptr) {
    localMapper->Release();
  }
}

void ApplyLocalizationModeChange(IOrbLocalMappingBackend *localMapper,
                                 IOrbTrackingBackend *tracker,
                                 bool &activateLocalizationMode,
                                 bool &deactivateLocalizationMode) {
  if (tracker == nullptr) {
    return;
  }
  if (activateLocalizationMode) {
    RequestLocalMappingStop(localMapper);
    WaitLocalMappingStopped(localMapper);
    tracker->SetOnlyTracking(true);
    activateLocalizationMode = false;
  }
  if (deactivateLocalizationMode) {
    tracker->SetOnlyTracking(false);
    ReleaseLocalMapping(localMapper);
    deactivateLocalizationMode = false;
  }
}

void ResetTrackingIfRequested(IOrbTrackingBackend *tracker, bool &reset,
                              bool &resetActiveMap) {
  if (tracker == nullptr) {
    reset = false;
    resetActiveMap = false;
    return;
  }
  if (reset) {
    tracker->Reset(false);
    reset = false;
    resetActiveMap = false;
  } else if (resetActiveMap) {
    tracker->Reset(true);
    resetActiveMap = false;
  }
}

void PushImuMeasurements(IOrbTrackingBackend *tracker,
                         const vector<IMU::Point> &measurements) {
  if (tracker == nullptr) {
    return;
  }
  for (const IMU::Point &measurement : measurements) {
    tracker->PushImu(measurement);
  }
}

void UpdateTrackingStateFromBackend(IOrbTrackingBackend *tracker,
                                    int &trackingState) {
  if (tracker != nullptr) {
    trackingState = tracker->Status().state;
  }
}

bool RunSystemGlobalOptimization(
    IOrbOptimizationBackend *backend, Map *map,
    const SlamBackendOptimizationRequest &request) {
  if (backend == nullptr || map == nullptr) {
    return false;
  }
  const int iterations = std::max(1, request.iterationLimit);
  const unsigned long loopKeyframeId =
      static_cast<unsigned long>(request.keyframeId);
  if (request.inertial || map->isImuInitialized()) {
    return backend->RunFullInertialBundleAdjustment(
        OrbFullInertialBundleAdjustmentRequest{
            map, iterations, false, loopKeyframeId});
  }
  return backend->RunGlobalBundleAdjustment(
      OrbGlobalBundleAdjustmentRequest{
          map, iterations, nullptr, loopKeyframeId, request.robust});
}

void EnsureDirectory(const std::string &path) {
#ifdef _WIN32
  _mkdir(path.c_str());
#else
  mkdir(path.c_str(), 0755);
#endif
}

void DumpStereoDebugImages(const cv::Mat &rawLeft, const cv::Mat &rawRight,
                           const cv::Mat &preparedLeft,
                           const cv::Mat &preparedRight, bool rectified) {
  static std::atomic<int> dumpCount{0};
  if (!EnvFlagEnabled("SMART_DRONE_DUMP_STEREO_DEBUG"))
    return;

  const int index = dumpCount.fetch_add(1);
  if (index >= 10)
    return;

  const std::string dir = "/tmp/smartdrone_stereo_debug";
  EnsureDirectory(dir);

  std::ostringstream prefix;
  prefix << dir << "/frame_" << std::setw(3) << std::setfill('0') << index;

  cv::imwrite(prefix.str() + "_raw_left.png", rawLeft);
  cv::imwrite(prefix.str() + "_raw_right.png", rawRight);
  cv::imwrite(prefix.str() + "_prepared_left.png", preparedLeft);
  cv::imwrite(prefix.str() + "_prepared_right.png", preparedRight);

  std::cerr << "[stereo_debug] dumped " << prefix.str()
            << " rectified=" << (rectified ? 1 : 0) << " raw=" << rawLeft.cols
            << "x" << rawLeft.rows << "/" << rawRight.cols << "x"
            << rawRight.rows << " prepared=" << preparedLeft.cols << "x"
            << preparedLeft.rows << "/" << preparedRight.cols << "x"
            << preparedRight.rows << std::endl;
}
} // namespace

System::System(const string &strVocFile, const string &strSettingsFile,
               const eSensor sensor, const bool bUseViewer, const int initFr,
               const string &strSequence)
    : mSensor(sensor), mpVocabulary(nullptr), mpKeyFrameDatabase(nullptr),
      mpAtlas(nullptr), mpTracker(nullptr), mpLocalMapper(nullptr),
      mpLocalMappingBackend(nullptr),
      mpLoopCloser(nullptr), mbReset(false), mbResetActiveMap(false),
      mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false),
      mbShutDown(false), mTrackingState(kOrbTrackingStateSystemNotReady),
      mpOptimizationBackend(CreateDefaultOrbOptimizationBackend()),
      settings_(nullptr) {
  // Output welcome message
  cout << endl
       << "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, "
          "Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of "
          "Zaragoza."
       << endl
       << "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel "
          "and Juan D. Tardós, University of Zaragoza."
       << endl
       << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
       << "This is free software, and you are welcome to redistribute it"
       << endl
       << "under certain conditions. See LICENSE.txt." << endl
       << endl;

  cout << "Input sensor was set to: ";

  if (EnvFlagEnabled("SMART_DRONE_ORB_DETERMINISTIC_RANDOM", false))
    DUtils::Random::SeedRandOnce(0);

  if (mSensor == MONOCULAR)
    cout << "Monocular" << endl;
  else if (mSensor == STEREO)
    cout << "Stereo" << endl;
  else if (mSensor == RGBD)
    cout << "RGB-D" << endl;
  else if (mSensor == IMU_MONOCULAR)
    cout << "Monocular-Inertial" << endl;
  else if (mSensor == IMU_STEREO)
    cout << "Stereo-Inertial" << endl;
  else if (mSensor == IMU_RGBD)
    cout << "RGB-D-Inertial" << endl;

  // Check settings file
  cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    cerr << "Failed to open settings file at: " << strSettingsFile << endl;
    exit(-1);
  }

  cv::FileNode node = fsSettings["File.version"];
  if (!node.empty() && node.isString() && node.string() == "1.0") {
    settings_ = new Settings(strSettingsFile, mSensor);

    mStrLoadAtlasFromFile = settings_->atlasLoadFile();
    mStrSaveAtlasToFile = settings_->atlasSaveFile();

    cout << (*settings_) << endl;
  } else {
    settings_ = nullptr;
    cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
    if (!node.empty() && node.isString()) {
      mStrLoadAtlasFromFile = (string)node;
    }

    node = fsSettings["System.SaveAtlasToFile"];
    if (!node.empty() && node.isString()) {
      mStrSaveAtlasToFile = (string)node;
    }
  }

  node = fsSettings["loopClosing"];
  bool activeLC = true;
  if (!node.empty()) {
    activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
  }

  mStrVocabularyFilePath = strVocFile;

  bool loadedAtlas = false;

  if (mStrLoadAtlasFromFile.empty()) {
    // Load ORB Vocabulary
    cout << endl
         << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = SmartDrone::adapters::slam::LoadOrbVocabularyWithCache(
        strVocFile, *mpVocabulary);
    if (!bVocLoad) {
      cerr << "Wrong path to vocabulary. " << endl;
      cerr << "Falied to open at: " << strVocFile << endl;
      exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    // Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    // Create the Atlas
    cout << "Initialization of Atlas from scratch " << endl;
    mpAtlas = new Atlas(0);
  } else {
    // Load ORB Vocabulary
    cout << endl
         << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = SmartDrone::adapters::slam::LoadOrbVocabularyWithCache(
        strVocFile, *mpVocabulary);
    if (!bVocLoad) {
      cerr << "Wrong path to vocabulary. " << endl;
      cerr << "Falied to open at: " << strVocFile << endl;
      exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    // Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    cout << "Load File" << endl;

    // Load the file with an earlier session
    // clock_t start = clock();
    cout << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile
         << endl;
    bool isRead = LoadAtlas(FileType::BINARY_FILE);

    if (!isRead) {
      cout << "Error to load the file, please try with other session file or "
              "vocabulary file"
           << endl;
      exit(-1);
    }
    // mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    // cout << "KF in DB: " << mpKeyFrameDatabase->mnNumKFs << "; words: " <<
    // mpKeyFrameDatabase->mnNumWords << endl;

    loadedAtlas = true;

    mpAtlas->CreateNewMap();

    // clock_t timeElapsed = clock() - start;
    // unsigned msElapsed = timeElapsed / (CLOCKS_PER_SEC / 1000);
    // cout << "Binary file read in " << msElapsed << " ms" << endl;

  }

  if (mSensor == IMU_STEREO || mSensor == IMU_MONOCULAR || mSensor == IMU_RGBD)
    mpAtlas->SetInertialSensor();

  // Create Drawers. These are used by the Viewer
  //  mpFrameDrawer = new FrameDrawer(mpAtlas);
  //  mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);

  // Initialize tracking. Runtime ticks call it through the EPG session graph.
  cout << "Seq. Name: " << strSequence << endl;
  mpTracker = new Tracking(this, mpVocabulary, mpAtlas, mpKeyFrameDatabase,
                           strSettingsFile, mSensor, settings_, strSequence);

  // Initialize local mapping. The backend is ticked by the EPG session graph.
  mpLocalMapper = new LocalMapping(
      this, mpAtlas, mSensor == MONOCULAR || mSensor == IMU_MONOCULAR,
      mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD,
      strSequence);
  mpLocalMappingBackend = CreateDefaultOrbLocalMappingBackend(mpLocalMapper);
  float farPointThreshold = 0.0f;
  if (settings_)
    farPointThreshold = settings_->thFarPoints();
  else
    farPointThreshold = fsSettings["thFarPoints"];
  mpLocalMappingBackend->ApplyRuntimeConfig(
      OrbLocalMappingRuntimeConfig{initFr, farPointThreshold});
  if (farPointThreshold != 0.0f) {
    cout << "Discard points further than " << farPointThreshold
         << " m from current camera" << endl;
  }

  // Initialize loop closing. The backend is ticked by the EPG session graph.
  //  mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR
  mpLoopCloser =
      new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary,
                      mSensor != MONOCULAR, activeLC); // mSensor!=MONOCULAR);

  // Set runtime peer pointers
  mpTracker->SetLocalMapper(mpLocalMapper);
  mpTracker->SetLoopClosing(mpLoopCloser);

  mpTrackingBackend = CreateDefaultOrbTrackingBackend(mpTracker);
  mpLoopClosingBackend = CreateDefaultOrbLoopClosingBackend(mpLoopCloser);
  mpLocalMappingBackend->AttachRuntimePeers(
      OrbLocalMappingRuntimePeers{mpTrackingBackend.get(), mpLoopCloser});

  mpLoopClosingBackend->AttachRuntimePeers(
      OrbLoopClosingRuntimePeers{mpTrackingBackend.get()});
  mpLoopCloser->SetLocalMapper(mpLocalMapper);
  // Fix verbosity
  Verbose::SetTh(Verbose::VERBOSITY_QUIET);
}

System::~System() = default;

bool System::PrepareStereoImagesForTracking(const cv::Mat &imLeft,
                                            const cv::Mat &imRight,
                                            cv::Mat &imLeftPrepared,
                                            cv::Mat &imRightPrepared) const {
  if (mSensor != STEREO && mSensor != IMU_STEREO) {
    cerr << "ERROR: you called PrepareStereoImagesForTracking but input sensor "
            "was not set to Stereo nor Stereo-Inertial."
         << endl;
    return false;
  }

  imLeftPrepared = imLeft;
  imRightPrepared = imRight;
  const bool rectified = settings_ && settings_->needToRectify();
  if (rectified) {
    cv::Mat M1l = settings_->M1l();
    cv::Mat M2l = settings_->M2l();
    cv::Mat M1r = settings_->M1r();
    cv::Mat M2r = settings_->M2r();

    if (!VpiRemapStereoForOrb(imLeft, imRight, imLeftPrepared, imRightPrepared,
                              M1l, M2l, M1r, M2r)) {
      cv::remap(imLeft, imLeftPrepared, M1l, M2l, cv::INTER_LINEAR);
      cv::remap(imRight, imRightPrepared, M1r, M2r, cv::INTER_LINEAR);
    }
  } else if (settings_ && settings_->needToResize()) {
    cv::resize(imLeft, imLeftPrepared, settings_->newImSize());
    cv::resize(imRight, imRightPrepared, settings_->newImSize());
  } else {
    imLeftPrepared = imLeft;
    imRightPrepared = imRight;
  }
  DumpStereoDebugImages(imLeft, imRight, imLeftPrepared, imRightPrepared,
                        rectified);
  return true;
}

void System::StepBackend() {
  if (mpLocalMappingBackend != nullptr) {
    mpLocalMappingBackend->Step();
  }
  if (mpLoopClosingBackend != nullptr) {
    mpLoopClosingBackend->Step();
  }
}

Sophus::SE3f System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight,
                                 const double &timestamp,
                                 const vector<IMU::Point> &vImuMeas,
                                 string filename) {
  if (mSensor != STEREO && mSensor != IMU_STEREO) {
    cerr << "ERROR: you called TrackStereo but input sensor was not set to "
            "Stereo nor Stereo-Inertial."
         << endl;
    exit(-1);
  }

  cv::Mat imLeftToFeed;
  cv::Mat imRightToFeed;
  PrepareStereoImagesForTracking(imLeft, imRight, imLeftToFeed, imRightToFeed);

  // Check mode change
  {
    unique_lock<mutex> lock(mMutexMode);
    ApplyLocalizationModeChange(mpLocalMappingBackend.get(),
                                mpTrackingBackend.get(),
                                mbActivateLocalizationMode,
                                mbDeactivateLocalizationMode);
  }

  // Check reset
  {
    unique_lock<mutex> lock(mMutexReset);
    ResetTrackingIfRequested(mpTrackingBackend.get(), mbReset,
                             mbResetActiveMap);
  }

  if (mSensor == System::IMU_STEREO)
    PushImuMeasurements(mpTrackingBackend.get(), vImuMeas);

  // std::cout << "start GrabImageStereo" << std::endl;
  Sophus::SE3f Tcw =
      mpTrackingBackend
          ? mpTrackingBackend->TrackStereo(
                OrbStereoTrackRequest{&imLeftToFeed, &imRightToFeed,
                                      timestamp, filename})
          : Sophus::SE3f();

  // std::cout << "out grabber" << std::endl;

  {
    unique_lock<mutex> lock2(mMutexState);
    UpdateTrackingStateFromBackend(mpTrackingBackend.get(), mTrackingState);
  }

  StoreLocalMappingWaitStats(
      WaitForLocalMappingIdle(mpLocalMappingBackend.get()));

  return Tcw;
}

Sophus::SE3f System::TrackStereoWithFeatures(
    const cv::Mat &imLeft, const cv::Mat &imRight,
    const StereoFeatureFrameData &features, const double &timestamp,
    const vector<IMU::Point> &vImuMeas, string filename) {
  if (mSensor != STEREO && mSensor != IMU_STEREO) {
    cerr << "ERROR: you called TrackStereoWithFeatures but input sensor was "
            "not set to Stereo nor Stereo-Inertial."
         << endl;
    exit(-1);
  }

  cv::Mat imLeftToFeed = imLeft;
  cv::Mat imRightToFeed = imRight;
  if (settings_ && settings_->needToRectify()) {
    cv::Mat M1l = settings_->M1l();
    cv::Mat M2l = settings_->M2l();
    cv::Mat M1r = settings_->M1r();
    cv::Mat M2r = settings_->M2r();

    if (!VpiRemapStereoForOrb(imLeft, imRight, imLeftToFeed, imRightToFeed, M1l,
                              M2l, M1r, M2r)) {
      cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
      cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
    }
  } else if (settings_ && settings_->needToResize()) {
    cv::resize(imLeft, imLeftToFeed, settings_->newImSize());
    cv::resize(imRight, imRightToFeed, settings_->newImSize());
  } else {
    imLeftToFeed = imLeft;
    imRightToFeed = imRight;
  }

  {
    unique_lock<mutex> lock(mMutexMode);
    ApplyLocalizationModeChange(mpLocalMappingBackend.get(),
                                mpTrackingBackend.get(),
                                mbActivateLocalizationMode,
                                mbDeactivateLocalizationMode);
  }

  {
    unique_lock<mutex> lock(mMutexReset);
    ResetTrackingIfRequested(mpTrackingBackend.get(), mbReset,
                             mbResetActiveMap);
  }

  if (mSensor == System::IMU_STEREO)
    PushImuMeasurements(mpTrackingBackend.get(), vImuMeas);

  Sophus::SE3f Tcw =
      mpTrackingBackend
          ? mpTrackingBackend->TrackStereoWithFeatures(
                OrbStereoFeatureTrackRequest{
                    &imLeftToFeed, &imRightToFeed, &features, timestamp,
                    filename})
          : Sophus::SE3f();

  {
    unique_lock<mutex> lock2(mMutexState);
    UpdateTrackingStateFromBackend(mpTrackingBackend.get(), mTrackingState);
  }

  StoreLocalMappingWaitStats(
      WaitForLocalMappingIdle(mpLocalMappingBackend.get()));

  return Tcw;
}

Sophus::SE3f System::TrackStereoPreparedWithFeatures(
    const cv::Mat &imLeftPrepared, const cv::Mat &imRightPrepared,
    const StereoFeatureFrameData &features, const double &timestamp,
    const vector<IMU::Point> &vImuMeas, string filename) {
  if (mSensor != STEREO && mSensor != IMU_STEREO) {
    cerr << "ERROR: you called TrackStereoPreparedWithFeatures but input "
            "sensor was not set to Stereo nor Stereo-Inertial."
         << endl;
    exit(-1);
  }

  {
    unique_lock<mutex> lock(mMutexMode);
    ApplyLocalizationModeChange(mpLocalMappingBackend.get(),
                                mpTrackingBackend.get(),
                                mbActivateLocalizationMode,
                                mbDeactivateLocalizationMode);
  }

  {
    unique_lock<mutex> lock(mMutexReset);
    ResetTrackingIfRequested(mpTrackingBackend.get(), mbReset,
                             mbResetActiveMap);
  }

  if (mSensor == System::IMU_STEREO)
    PushImuMeasurements(mpTrackingBackend.get(), vImuMeas);

  Sophus::SE3f Tcw =
      mpTrackingBackend
          ? mpTrackingBackend->TrackStereoWithFeatures(
                OrbStereoFeatureTrackRequest{
                    &imLeftPrepared, &imRightPrepared, &features, timestamp,
                    filename})
          : Sophus::SE3f();

  {
    unique_lock<mutex> lock2(mMutexState);
    UpdateTrackingStateFromBackend(mpTrackingBackend.get(), mTrackingState);
  }

  StoreLocalMappingWaitStats(
      WaitForLocalMappingIdle(mpLocalMappingBackend.get()));

  return Tcw;
}

Sophus::SE3f System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap,
                               const double &timestamp,
                               const vector<IMU::Point> &vImuMeas,
                               string filename) {
  if (mSensor != RGBD && mSensor != IMU_RGBD) {
    cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD."
         << endl;
    exit(-1);
  }

  cv::Mat imToFeed = im;
  cv::Mat imDepthToFeed = depthmap;
  if (settings_ && settings_->needToResize()) {
    cv::Mat resizedIm;
    cv::resize(im, resizedIm, settings_->newImSize());
    imToFeed = resizedIm;

    cv::resize(depthmap, imDepthToFeed, settings_->newImSize());
  }

  // Check mode change
  {
    unique_lock<mutex> lock(mMutexMode);
    ApplyLocalizationModeChange(mpLocalMappingBackend.get(),
                                mpTrackingBackend.get(),
                                mbActivateLocalizationMode,
                                mbDeactivateLocalizationMode);
  }

  // Check reset
  {
    unique_lock<mutex> lock(mMutexReset);
    ResetTrackingIfRequested(mpTrackingBackend.get(), mbReset,
                             mbResetActiveMap);
  }

  if (mSensor == System::IMU_RGBD)
    PushImuMeasurements(mpTrackingBackend.get(), vImuMeas);

  Sophus::SE3f Tcw =
      mpTrackingBackend
          ? mpTrackingBackend->TrackRgbd(
                OrbRgbdTrackRequest{&imToFeed, &imDepthToFeed, timestamp,
                                    filename})
          : Sophus::SE3f();

  unique_lock<mutex> lock2(mMutexState);
  UpdateTrackingStateFromBackend(mpTrackingBackend.get(), mTrackingState);
  return Tcw;
}

Sophus::SE3f System::TrackMonocular(const cv::Mat &im, const double &timestamp,
                                    const vector<IMU::Point> &vImuMeas,
                                    string filename) {

  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbShutDown)
      return Sophus::SE3f();
  }

  if (mSensor != MONOCULAR && mSensor != IMU_MONOCULAR) {
    cerr << "ERROR: you called TrackMonocular but input sensor was not set to "
            "Monocular nor Monocular-Inertial."
         << endl;
    exit(-1);
  }

  cv::Mat imToFeed = im;
  if (settings_ && settings_->needToResize()) {
    cv::Mat resizedIm;
    cv::resize(im, resizedIm, settings_->newImSize());
    imToFeed = resizedIm;
  }

  // Check mode change
  {
    unique_lock<mutex> lock(mMutexMode);
    ApplyLocalizationModeChange(mpLocalMappingBackend.get(),
                                mpTrackingBackend.get(),
                                mbActivateLocalizationMode,
                                mbDeactivateLocalizationMode);
  }

  // Check reset
  {
    unique_lock<mutex> lock(mMutexReset);
    ResetTrackingIfRequested(mpTrackingBackend.get(), mbReset,
                             mbResetActiveMap);
  }

  if (mSensor == System::IMU_MONOCULAR)
    PushImuMeasurements(mpTrackingBackend.get(), vImuMeas);

  Sophus::SE3f Tcw =
      mpTrackingBackend
          ? mpTrackingBackend->TrackMonocular(
                OrbMonocularTrackRequest{&imToFeed, timestamp, filename})
          : Sophus::SE3f();

  unique_lock<mutex> lock2(mMutexState);
  UpdateTrackingStateFromBackend(mpTrackingBackend.get(), mTrackingState);

  return Tcw;
}

Sophus::SE3f System::TrackMonocularWithFeatures(
    const cv::Mat &im, const MonoFeatureFrameData &features,
    const double &timestamp, const vector<IMU::Point> &vImuMeas,
    string filename) {
  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbShutDown)
      return Sophus::SE3f();
  }

  if (mSensor != MONOCULAR && mSensor != IMU_MONOCULAR) {
    cerr << "ERROR: you called TrackMonocularWithFeatures but input sensor was "
            "not set to Monocular nor "
            "Monocular-Inertial."
         << endl;
    exit(-1);
  }

  cv::Mat imToFeed = im;
  if (settings_ && settings_->needToResize()) {
    cv::Mat resizedIm;
    cv::resize(im, resizedIm, settings_->newImSize());
    imToFeed = resizedIm;
  }

  {
    unique_lock<mutex> lock(mMutexMode);
    ApplyLocalizationModeChange(mpLocalMappingBackend.get(),
                                mpTrackingBackend.get(),
                                mbActivateLocalizationMode,
                                mbDeactivateLocalizationMode);
  }

  {
    unique_lock<mutex> lock(mMutexReset);
    ResetTrackingIfRequested(mpTrackingBackend.get(), mbReset,
                             mbResetActiveMap);
  }

  if (mSensor == System::IMU_MONOCULAR)
    PushImuMeasurements(mpTrackingBackend.get(), vImuMeas);

  Sophus::SE3f Tcw =
      mpTrackingBackend
          ? mpTrackingBackend->TrackMonocularWithFeatures(
                OrbMonocularFeatureTrackRequest{&imToFeed, &features,
                                                timestamp, filename})
          : Sophus::SE3f();

  unique_lock<mutex> lock2(mMutexState);
  UpdateTrackingStateFromBackend(mpTrackingBackend.get(), mTrackingState);

  return Tcw;
}

void System::ActivateLocalizationMode() {
  unique_lock<mutex> lock(mMutexMode);
  mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode() {
  unique_lock<mutex> lock(mMutexMode);
  mbDeactivateLocalizationMode = true;
}

bool System::MapChanged() {
  static int n = 0;
  int curn = mpAtlas->GetLastBigChangeIdx();
  if (n < curn) {
    n = curn;
    return true;
  } else
    return false;
}

ORBextractor *System::GetLeftORBExtractor() const {
  return mpTrackingBackend ? mpTrackingBackend->LeftORBExtractor() : nullptr;
}

ORBextractor *System::GetRightORBExtractor() const {
  return mpTrackingBackend ? mpTrackingBackend->RightORBExtractor() : nullptr;
}

unsigned long System::GetCurrentMapId() {
  Map *pCurrentMap = mpAtlas ? mpAtlas->GetCurrentMap() : nullptr;
  return pCurrentMap ? pCurrentMap->GetId()
                     : std::numeric_limits<unsigned long>::max();
}

void System::Reset() {
  unique_lock<mutex> lock(mMutexReset);
  mbReset = true;
}

void System::ResetActiveMap() {
  unique_lock<mutex> lock(mMutexReset);
  mbResetActiveMap = true;
}

void System::Shutdown() {
  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbShutDown)
      return;
    mbShutDown = true;
  }

  cout << "Shutdown" << endl;

  if (mpLocalMappingBackend != nullptr) {
    mpLocalMappingBackend->RequestFinish();
  }
  if (mpLoopClosingBackend != nullptr) {
    mpLoopClosingBackend->RequestFinish();
    mpLoopClosingBackend->AbortGlobalBundleAdjustment();
  }
  constexpr int kShutdownBackendDrainMaxIterations = 1000;
  int drainIterations = 0;
  while (((mpLocalMappingBackend != nullptr &&
           !mpLocalMappingBackend->Status().finished) ||
          (mpLoopClosingBackend != nullptr &&
           (!mpLoopClosingBackend->Status().finished ||
            mpLoopClosingBackend->Status().runningGlobalBundleAdjustment))) &&
         drainIterations < kShutdownBackendDrainMaxIterations) {
    StepBackend();
    ++drainIterations;
  }
  if (drainIterations >= kShutdownBackendDrainMaxIterations) {
    cerr << "ORB backend shutdown drain timed out" << endl;
  }

  if (!mStrSaveAtlasToFile.empty()) {
    Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile,
                       Verbose::VERBOSITY_NORMAL);
    SaveAtlas(FileType::BINARY_FILE);
  }

  /*if(mpViewer)
      pangolin::BindToContext("ORB-SLAM2: Map Viewer");*/

#ifdef REGISTER_TIMES
  if (mpTrackingBackend != nullptr) {
    mpTrackingBackend->PrintTimeStats();
  }
#endif
}

bool System::isShutDown() {
  unique_lock<mutex> lock(mMutexReset);
  return mbShutDown;
}

void System::SaveTrajectoryTUM(const string &filename) {
  cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
  if (mSensor == MONOCULAR) {
    cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
    return;
  }

  vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  const auto trajectory =
      mpTrackingBackend ? mpTrackingBackend->FrameTrajectorySnapshot()
                        : std::vector<OrbFrameTrajectoryEntry>{};
  for (const OrbFrameTrajectoryEntry &entry : trajectory) {
    if (entry.lost)
      continue;

    KeyFrame *pKF = entry.referenceKeyFrame;

    Sophus::SE3f Trw;

    // If the reference keyframe was culled, traverse the spanning tree to get a
    // suitable keyframe.
    if (!pKF)
      continue;
    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Two;

    Sophus::SE3f Tcw = entry.relativePose * Trw;
    Sophus::SE3f Twc = Tcw.inverse();

    Eigen::Vector3f twc = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();

    f << setprecision(6) << entry.timestamp << " " << setprecision(9)
      << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " "
      << q.y() << " " << q.z() << " " << q.w() << endl;
  }
  f.close();
  // cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename) {
  cout << endl
       << "Saving keyframe trajectory to " << filename << " ..." << endl;

  vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame *pKF = vpKFs[i];

    // pKF->SetPose(pKF->GetPose()*Two);

    if (pKF->isBad())
      continue;

    Sophus::SE3f Twc = pKF->GetPoseInverse();
    Eigen::Quaternionf q = Twc.unit_quaternion();
    Eigen::Vector3f t = Twc.translation();
    f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t(0)
      << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " "
      << q.z() << " " << q.w() << endl;
  }

  f.close();
}

void System::SaveTrajectoryEuRoC(const string &filename) {

  cout << endl << "Saving trajectory to " << filename << " ..." << endl;
  /*if(mSensor==MONOCULAR)
  {
      cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." <<
  endl; return;
  }*/

  vector<Map *> vpMaps = mpAtlas->GetAllMaps();
  int numMaxKFs = 0;
  Map *pBiggerMap;
  std::cout << "There are " << std::to_string(vpMaps.size())
            << " maps in the atlas" << std::endl;
  for (Map *pMap : vpMaps) {
    std::cout << "  Map " << std::to_string(pMap->GetId()) << " has "
              << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs"
              << std::endl;
    if (pMap->GetAllKeyFrames().size() > numMaxKFs) {
      numMaxKFs = pMap->GetAllKeyFrames().size();
      pBiggerMap = pMap;
    }
  }

  vector<KeyFrame *> vpKFs = pBiggerMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f
      Twb; // Can be word to cam0 or world to b depending on IMU or not.
  if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
    Twb = vpKFs[0]->GetImuPose();
  else
    Twb = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  // cout << "file open" << endl;
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  const auto trajectory =
      mpTrackingBackend ? mpTrackingBackend->FrameTrajectorySnapshot()
                        : std::vector<OrbFrameTrajectoryEntry>{};
  for (const OrbFrameTrajectoryEntry &entry : trajectory) {
    // cout << "1" << endl;
    if (entry.lost)
      continue;

    KeyFrame *pKF = entry.referenceKeyFrame;
    // cout << "KF: " << pKF->mnId << endl;

    Sophus::SE3f Trw;

    // If the reference keyframe was culled, traverse the spanning tree to get a
    // suitable keyframe.
    if (!pKF)
      continue;

    // cout << "2.5" << endl;

    while (pKF->isBad()) {
      // cout << " 2.bad" << endl;
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
      // cout << "--Parent KF: " << pKF->mnId << endl;
    }

    if (!pKF || pKF->GetMap() != pBiggerMap) {
      // cout << "--Parent KF is from another map" << endl;
      continue;
    }

    // cout << "3" << endl;

    Trw = Trw * pKF->GetPose() *
          Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

    // cout << "4" << endl;

    if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
        mSensor == IMU_RGBD) {
      Sophus::SE3f Twb =
          (pKF->mImuCalib.mTbc * entry.relativePose * Trw).inverse();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << setprecision(6) << 1e9 * entry.timestamp << " " << setprecision(9)
        << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " "
        << q.y() << " " << q.z() << " " << q.w() << endl;
    } else {
      Sophus::SE3f Twc = (entry.relativePose * Trw).inverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f twc = Twc.translation();
      f << setprecision(6) << 1e9 * entry.timestamp << " "
        << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " "
        << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    }

    // cout << "5" << endl;
  }
  // cout << "end saving trajectory" << endl;
  f.close();
  cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}

void System::SaveTrajectoryEuRoC(const string &filename, Map *pMap) {

  cout << endl
       << "Saving trajectory of map " << pMap->GetId() << " to " << filename
       << " ..." << endl;
  /*if(mSensor==MONOCULAR)
  {
      cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." <<
  endl; return;
  }*/

  int numMaxKFs = 0;

  vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f
      Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
  if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
    Twb = vpKFs[0]->GetImuPose();
  else
    Twb = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  // cout << "file open" << endl;
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  const auto trajectory =
      mpTrackingBackend ? mpTrackingBackend->FrameTrajectorySnapshot()
                        : std::vector<OrbFrameTrajectoryEntry>{};
  for (const OrbFrameTrajectoryEntry &entry : trajectory) {
    // cout << "1" << endl;
    if (entry.lost)
      continue;

    KeyFrame *pKF = entry.referenceKeyFrame;
    // cout << "KF: " << pKF->mnId << endl;

    Sophus::SE3f Trw;

    // If the reference keyframe was culled, traverse the spanning tree to get a
    // suitable keyframe.
    if (!pKF)
      continue;

    // cout << "2.5" << endl;

    while (pKF->isBad()) {
      // cout << " 2.bad" << endl;
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
      // cout << "--Parent KF: " << pKF->mnId << endl;
    }

    if (!pKF || pKF->GetMap() != pMap) {
      // cout << "--Parent KF is from another map" << endl;
      continue;
    }

    // cout << "3" << endl;

    Trw = Trw * pKF->GetPose() *
          Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

    // cout << "4" << endl;

    if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
        mSensor == IMU_RGBD) {
      Sophus::SE3f Twb =
          (pKF->mImuCalib.mTbc * entry.relativePose * Trw).inverse();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << setprecision(6) << 1e9 * entry.timestamp << " " << setprecision(9)
        << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " "
        << q.y() << " " << q.z() << " " << q.w() << endl;
    } else {
      Sophus::SE3f Twc = (entry.relativePose * Trw).inverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f twc = Twc.translation();
      f << setprecision(6) << 1e9 * entry.timestamp << " "
        << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " "
        << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    }

    // cout << "5" << endl;
  }
  // cout << "end saving trajectory" << endl;
  f.close();
  cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}

bool System::GetLatestFrameTrajectoryPoseEuRoC(Sophus::SE3f &twc,
                                               double *timestamp,
                                               bool *lost) const {
  if (!mpTrackingBackend || !mpAtlas) {
    return false;
  }
  OrbFrameTrajectoryEntry entry;
  if (!mpTrackingBackend->LatestFrameTrajectoryEntry(entry)) {
    return false;
  }

  if (timestamp) {
    *timestamp = entry.timestamp;
  }
  if (lost) {
    *lost = entry.lost;
  }
  if (entry.lost) {
    return false;
  }

  KeyFrame *pKF = entry.referenceKeyFrame;
  if (!pKF) {
    return false;
  }

  Map *pMap = pKF->GetMap();
  if (!pMap) {
    return false;
  }

  vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
  if (vpKFs.empty()) {
    return false;
  }
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  Sophus::SE3f Twb;
  if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
      mSensor == IMU_RGBD) {
    Twb = vpKFs[0]->GetImuPose();
  } else {
    Twb = vpKFs[0]->GetPoseInverse();
  }

  Sophus::SE3f Trw;
  while (pKF->isBad()) {
    Trw = Trw * pKF->mTcp;
    pKF = pKF->GetParent();
    if (!pKF) {
      return false;
    }
  }
  if (pKF->GetMap() != pMap) {
    return false;
  }

  Trw = Trw * pKF->GetPose() * Twb;
  if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
      mSensor == IMU_RGBD) {
    twc = (pKF->mImuCalib.mTbc * entry.relativePose * Trw).inverse();
  } else {
    twc = (entry.relativePose * Trw).inverse();
  }
  return true;
}

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename) {
  cout << endl
       << "Saving keyframe trajectory to " << filename << " ..." << endl;

  vector<Map *> vpMaps = mpAtlas->GetAllMaps();
  Map *pBiggerMap;
  int numMaxKFs = 0;
  for (Map *pMap : vpMaps) {
    if (pMap && pMap->GetAllKeyFrames().size() > numMaxKFs) {
      numMaxKFs = pMap->GetAllKeyFrames().size();
      pBiggerMap = pMap;
    }
  }

  if (!pBiggerMap) {
    std::cout << "There is not a map!!" << std::endl;
    return;
  }

  vector<KeyFrame *> vpKFs = pBiggerMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame *pKF = vpKFs[i];

    // pKF->SetPose(pKF->GetPose()*Two);

    if (!pKF || pKF->isBad())
      continue;
    if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
        mSensor == IMU_RGBD) {
      Sophus::SE3f Twb = pKF->GetImuPose();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " "
        << q.y() << " " << q.z() << " " << q.w() << endl;

    } else {
      Sophus::SE3f Twc = pKF->GetPoseInverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f t = Twc.translation();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y()
        << " " << q.z() << " " << q.w() << endl;
    }
  }
  f.close();
}

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename, Map *pMap) {
  cout << endl
       << "Saving keyframe trajectory of map " << pMap->GetId() << " to "
       << filename << " ..." << endl;

  vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame *pKF = vpKFs[i];

    if (!pKF || pKF->isBad())
      continue;
    if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
        mSensor == IMU_RGBD) {
      Sophus::SE3f Twb = pKF->GetImuPose();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " "
        << q.y() << " " << q.z() << " " << q.w() << endl;

    } else {
      Sophus::SE3f Twc = pKF->GetPoseInverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f t = Twc.translation();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y()
        << " " << q.z() << " " << q.w() << endl;
    }
  }
  f.close();
}

void System::SaveTrajectoryKITTI(const string &filename) {
  cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
  if (mSensor == MONOCULAR) {
    cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
    return;
  }

  vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  const auto trajectory =
      mpTrackingBackend ? mpTrackingBackend->FrameTrajectorySnapshot()
                        : std::vector<OrbFrameTrajectoryEntry>{};
  for (const OrbFrameTrajectoryEntry &entry : trajectory) {
    ORB_SLAM3::KeyFrame *pKF = entry.referenceKeyFrame;

    Sophus::SE3f Trw;

    if (!pKF)
      continue;

    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Tow;

    Sophus::SE3f Tcw = entry.relativePose * Trw;
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Matrix3f Rwc = Twc.rotationMatrix();
    Eigen::Vector3f twc = Twc.translation();

    f << setprecision(9) << Rwc(0, 0) << " " << Rwc(0, 1) << " " << Rwc(0, 2)
      << " " << twc(0) << " " << Rwc(1, 0) << " " << Rwc(1, 1) << " "
      << Rwc(1, 2) << " " << twc(1) << " " << Rwc(2, 0) << " " << Rwc(2, 1)
      << " " << Rwc(2, 2) << " " << twc(2) << endl;
  }
  f.close();
}

void System::SaveDebugData(const int &initIdx) {
  const OrbLocalMappingDebugSnapshot debug =
      mpLocalMappingBackend ? mpLocalMappingBackend->DebugSnapshot()
                            : OrbLocalMappingDebugSnapshot{};
  if (!debug.available) {
    return;
  }

  // 0. Save initialization trajectory
  SaveTrajectoryEuRoC("init_FrameTrajectoy_" +
                      to_string(debug.initSection) + "_" +
                      to_string(initIdx) + ".txt");

  // 1. Save scale
  ofstream f;
  f.open("init_Scale_" + to_string(debug.initSection) + ".txt",
         ios_base::app);
  f << fixed;
  f << debug.scale << endl;
  f.close();

  // 2. Save gravity direction
  f.open("init_GDir_" + to_string(debug.initSection) + ".txt",
         ios_base::app);
  f << fixed;
  f << debug.gravityRotation(0, 0) << "," << debug.gravityRotation(0, 1) << ","
    << debug.gravityRotation(0, 2) << endl;
  f << debug.gravityRotation(1, 0) << "," << debug.gravityRotation(1, 1) << ","
    << debug.gravityRotation(1, 2) << endl;
  f << debug.gravityRotation(2, 0) << "," << debug.gravityRotation(2, 1) << ","
    << debug.gravityRotation(2, 2) << endl;
  f.close();

  // 3. Save computational cost
  f.open("init_CompCost_" + to_string(debug.initSection) + ".txt",
         ios_base::app);
  f << fixed;
  f << debug.costTime << endl;
  f.close();

  // 4. Save biases
  f.open("init_Biases_" + to_string(debug.initSection) + ".txt",
         ios_base::app);
  f << fixed;
  f << debug.gyroBias(0) << "," << debug.gyroBias(1) << ","
    << debug.gyroBias(2) << endl;
  f << debug.accBias(0) << "," << debug.accBias(1) << ","
    << debug.accBias(2) << endl;
  f.close();

  // 5. Save covariance matrix
  f.open("init_CovMatrix_" + to_string(debug.initSection) + "_" +
             to_string(initIdx) + ".txt",
         ios_base::app);
  f << fixed;
  for (int i = 0; i < debug.inertialCovariance.rows(); i++) {
    for (int j = 0; j < debug.inertialCovariance.cols(); j++) {
      if (j != 0)
        f << ",";
      f << setprecision(15) << debug.inertialCovariance(i, j);
    }
    f << endl;
  }
  f.close();

  // 6. Save initialization time
  f.open("init_Time_" + to_string(debug.initSection) + ".txt",
         ios_base::app);
  f << fixed;
  f << debug.initTime << endl;
  f.close();
}

int System::GetTrackingState() {
  unique_lock<mutex> lock(mMutexState);
  return mTrackingState;
}

bool System::CanUseStereoFeatureInjection() const {
  if (!settings_)
    return true;

  if (settings_->needToResize())
    return false;

  return true;
}

int System::GetMatchesInliers() const {
  return mpTrackingBackend ? mpTrackingBackend->Status().matchesInliers : 0;
}

size_t System::GetTrackedMapPointCount() const {
  return GetMapSummary().trackedMapPointCount;
}

size_t System::GetLocalMapPointCount() const {
  return GetMapSummary().localMapPointCount;
}

uint64_t System::GetLocalMapPointHash() const {
  return GetMapSummary().localMapPointHash;
}

uint64_t System::GetMatchedMapPointHashBeforePoseOptimization() const {
  return GetMapSummary().matchedMapPointHashBeforePoseOptimization;
}

uint64_t System::GetTrackedMapPointHash() const {
  return GetMapSummary().trackedMapPointHash;
}

SlamMapSummary System::GetMapSummary() const {
  const unsigned long mapId = const_cast<System *>(this)->GetCurrentMapId();
  return mpTrackingBackend ? mpTrackingBackend->MapSummary(mapId)
                           : SlamMapSummary{mapId};
}

SlamBackendStats System::GetBackendStats() const {
  SlamBackendStats out;
  out.map = GetMapSummary();
  if (mpTrackingBackend != nullptr) {
    out.frame = mpTrackingBackend->FrameTrackingStats();
  }
  out.frame.trackingState = const_cast<System *>(this)->GetTrackingState();
  const LocalMappingWaitStats wait = GetLastLocalMappingWaitStats();
  out.localMappingWait.requested = wait.requested;
  out.localMappingWait.timedOut = wait.timedOut;
  out.localMappingWait.acceptingBefore = wait.acceptingBefore;
  out.localMappingWait.acceptingAfter = wait.acceptingAfter;
  out.localMappingWait.queueBefore = wait.queueBefore;
  out.localMappingWait.queueAfter = wait.queueAfter;
  out.localMappingWait.timeoutMs = wait.timeoutMs;
  out.localMappingWait.waitMs = wait.waitMs;
  return out;
}

void System::LogStereoFeatureDfx(uint64_t frameId,
                                 const StereoFeatureFrameData &features) const {
  if (!EnvFlagEnabled("SMART_DRONE_SP_LG_TRACK_DFX", false)) {
    return;
  }

  int validDisparityCount = 0;
  double disparitySum = 0.0;
  double minDisparity = std::numeric_limits<double>::infinity();
  double maxDisparity = 0.0;
  const size_t pairCount =
      std::min(features.leftKeypoints.size(), features.rightKeypoints.size());
  for (size_t i = 0; i < pairCount; ++i) {
    const double disparity = static_cast<double>(
        features.leftKeypoints[i].pt.x - features.rightKeypoints[i].pt.x);
    if (!(disparity > 0.0) || !std::isfinite(disparity)) {
      continue;
    }
    ++validDisparityCount;
    disparitySum += disparity;
    minDisparity = std::min(minDisparity, disparity);
    maxDisparity = std::max(maxDisparity, disparity);
  }
  const double meanDisparity =
      validDisparityCount > 0
          ? disparitySum / static_cast<double>(validDisparityCount)
          : 0.0;
  const SlamBackendStats stats = GetBackendStats();
  std::cerr << "[sp_lg_track_dfx] frame_id=" << frameId
            << " injected_left=" << features.leftKeypoints.size()
            << " injected_right=" << features.rightKeypoints.size()
            << " descriptor_rows=" << features.leftDescriptors.rows << "/"
            << features.rightDescriptors.rows
            << " valid_disparity=" << validDisparityCount
            << " disparity_mean=" << meanDisparity << " disparity_min="
            << (validDisparityCount > 0 ? minDisparity : 0.0)
            << " disparity_max=" << maxDisparity
            << " frame_N=" << stats.frame.featureCount
            << " close_mps=" << stats.frame.closeMapPointCount
            << " state=" << stats.frame.trackingState
            << " matches_inliers=" << stats.map.matchesInliers
            << " tracked_map_points=" << stats.map.trackedMapPointCount
            << " local_map_points=" << stats.map.localMapPointCount
            << " map_id=" << stats.map.mapId << "\n";
}

LocalMappingWaitStats System::GetLastLocalMappingWaitStats() const {
  unique_lock<mutex> lock(mMutexLocalMappingWaitStats);
  return mLastLocalMappingWaitStats;
}

void System::WaitForLocalMappingIdleIfRequested() {
  StoreLocalMappingWaitStats(
      WaitForLocalMappingIdle(mpLocalMappingBackend.get()));
}

void System::StoreLocalMappingWaitStats(const LocalMappingWaitStats &stats) {
  unique_lock<mutex> lock(mMutexLocalMappingWaitStats);
  mLastLocalMappingWaitStats = stats;
}

TrackedVisualSummary System::GetTrackedVisualSummary() const {
  return mpTrackingBackend ? mpTrackingBackend->TrackedVisualSummarySnapshot()
                           : TrackedVisualSummary{};
}

TrackedFeatureSnapshot System::ExtractTrackedFeatures(int leftImageWidth,
                                                      int leftImageHeight,
                                                      int rightImageWidth,
                                                      int rightImageHeight) {
  return mpTrackingBackend
             ? mpTrackingBackend->ExtractTrackedFeatures(
                   OrbTrackingVisualDataRequest{
                       leftImageWidth, leftImageHeight, rightImageWidth,
                       rightImageHeight, false, 0})
             : TrackedFeatureSnapshot{};
}

TrackedPointCloudSnapshot
System::ExtractTrackedPointCloud(size_t maxPointCloudPoints) {
  return mpTrackingBackend
             ? mpTrackingBackend->ExtractTrackedPointCloud(maxPointCloudPoints)
             : TrackedPointCloudSnapshot{};
}

TrackedVisualData System::ExtractTrackedVisualData(
    int leftImageWidth, int leftImageHeight, int rightImageWidth,
    int rightImageHeight, bool includePointCloud, size_t maxPointCloudPoints) {
  return mpTrackingBackend
             ? mpTrackingBackend->ExtractTrackedVisualData(
                   OrbTrackingVisualDataRequest{
                       leftImageWidth, leftImageHeight, rightImageWidth,
                       rightImageHeight, includePointCloud,
                       maxPointCloudPoints})
             : TrackedVisualData{};
}

bool System::OptimizeBackend(const SlamBackendOptimizationRequest &request,
                             SlamBackendOptimizationResult &result) {
  result = {};
  if (mpAtlas == nullptr) {
    return false;
  }

  Map *currentMap = mpAtlas->GetCurrentMap();
  if (currentMap == nullptr) {
    return false;
  }

  result.accepted = true;
  switch (request.scope) {
  case SmartDrone::core::ports::SlamBackendOptimizationScope::
      GlobalBundleAdjustment: {
    if (!RunSystemGlobalOptimization(mpOptimizationBackend.get(), currentMap,
                                     request)) {
      result.accepted = false;
      return false;
    }
    result.optimizedKeyframeCount =
        static_cast<int>(currentMap->KeyFramesInMap());
    result.optimizedMapPointCount =
        static_cast<int>(currentMap->MapPointsInMap());
    return true;
  }
  default:
    result.accepted = false;
    return false;
  }
}

bool System::ApplyLocalMappingOperation(const SlamBackendMappingRequest &request,
                                        SlamBackendMappingResult &result) {
  result = {};
  if (mpLocalMappingBackend == nullptr) {
    return false;
  }

  OrbLocalMappingStatus status = mpLocalMappingBackend->Status();
  result.queuedKeyframeCount = status.keyframesInQueue;
  result.paused = status.stopped;
  switch (request.operation) {
  case SmartDrone::core::ports::SlamBackendMappingOperation::Pause:
    mpLocalMappingBackend->RequestStop();
    result.accepted = true;
    return true;
  case SmartDrone::core::ports::SlamBackendMappingOperation::Resume:
    mpLocalMappingBackend->Release();
    result.accepted = true;
    result.paused = false;
    return true;
  case SmartDrone::core::ports::SlamBackendMappingOperation::Reset:
    if (request.mapId != 0 && mpAtlas != nullptr &&
        mpAtlas->GetCurrentMap() != nullptr &&
        mpAtlas->GetCurrentMap()->GetId() == request.mapId) {
      mpLocalMappingBackend->Reset(
          OrbLocalMappingResetRequest{mpAtlas->GetCurrentMap(), true});
    } else {
      mpLocalMappingBackend->Reset(OrbLocalMappingResetRequest{});
    }
    result.accepted = true;
    return true;
  case SmartDrone::core::ports::SlamBackendMappingOperation::ProcessQueue:
    result.accepted = true;
    return true;
  default:
    return false;
  }
}

bool System::ApplyLoopClosingOperation(
    const SlamBackendLoopClosureRequest &request,
    SlamBackendLoopClosureResult &result) {
  result = {};
  if (mpLoopClosingBackend == nullptr) {
    return false;
  }

  switch (request.operation) {
  case SmartDrone::core::ports::SlamBackendLoopClosureOperation::
      AbortGlobalBundleAdjustment:
    mpLoopClosingBackend->AbortGlobalBundleAdjustment();
    result.accepted = true;
    return true;
  case SmartDrone::core::ports::SlamBackendLoopClosureOperation::Reset:
  {
    Map *mapToReset = nullptr;
    if (request.mapId != 0 && mpAtlas != nullptr &&
        mpAtlas->GetCurrentMap() != nullptr &&
        mpAtlas->GetCurrentMap()->GetId() == request.mapId) {
      mapToReset = mpAtlas->GetCurrentMap();
    }
    mpLoopClosingBackend->Reset(
        OrbLoopClosingResetRequest{mapToReset, mapToReset != nullptr});
    result.accepted = true;
    return true;
  }
  default:
    return false;
  }
}

double System::GetTimeFromIMUInit() {
  const OrbLocalMappingStatus status =
      mpLocalMappingBackend ? mpLocalMappingBackend->Status()
                            : OrbLocalMappingStatus{};
  const double aux = status.currentKeyframeTime - status.firstTimestamp;
  if ((aux > 0.) && mpAtlas->isImuInitialized())
    return aux;
  else
    return 0.f;
}

bool System::isLost() {
  if (!mpAtlas->isImuInitialized())
    return false;
  return mpTrackingBackend != nullptr && mpTrackingBackend->Status().lost;
}

bool System::isFinished() { return (GetTimeFromIMUInit() > 0.1); }

void System::ChangeDataset() {
  if (mpTrackingBackend == nullptr || mpAtlas == nullptr ||
      mpAtlas->GetCurrentMap() == nullptr) {
    return;
  }
  mpTrackingBackend->ApplyDatasetChange(OrbTrackingDatasetChangeRequest{
      mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12});
}

float System::GetImageScale() {
  return mpTrackingBackend ? mpTrackingBackend->ImageScale() : 1.0f;
}

#ifdef REGISTER_TIMES
void System::InsertRectTime(double &time) {
  if (mpTrackingBackend != nullptr) {
    mpTrackingBackend->InsertRectTime(OrbTrackingTimingSample{time});
  }
}

void System::InsertResizeTime(double &time) {
  if (mpTrackingBackend != nullptr) {
    mpTrackingBackend->InsertResizeTime(OrbTrackingTimingSample{time});
  }
}

void System::InsertTrackTime(double &time) {
  if (mpTrackingBackend != nullptr) {
    mpTrackingBackend->InsertTrackTime(OrbTrackingTimingSample{time});
  }
}
#endif

void System::SaveAtlas(int type) {
  if (!mStrSaveAtlasToFile.empty()) {
    // clock_t start = clock();

    // Save the current session
    mpAtlas->PreSave();

    string pathSaveFileName = "./";
    pathSaveFileName = pathSaveFileName.append(mStrSaveAtlasToFile);
    pathSaveFileName = pathSaveFileName.append(".osa");

    string strVocabularyChecksum =
        CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);
    std::size_t found = mStrVocabularyFilePath.find_last_of("/\\");
    string strVocabularyName = mStrVocabularyFilePath.substr(found + 1);

    if (type == TEXT_FILE) // File text
    {
      cout << "Starting to write the save text file " << endl;
      std::remove(pathSaveFileName.c_str());
      std::ofstream ofs(pathSaveFileName, std::ios::binary);
      boost::archive::text_oarchive oa(ofs);

      oa << strVocabularyName;
      oa << strVocabularyChecksum;
      oa << mpAtlas;
      cout << "End to write the save text file" << endl;
    } else if (type == BINARY_FILE) // File binary
    {
      cout << "Starting to write the save binary file" << endl;
      std::remove(pathSaveFileName.c_str());
      std::ofstream ofs(pathSaveFileName, std::ios::binary);
      boost::archive::binary_oarchive oa(ofs);
      oa << strVocabularyName;
      oa << strVocabularyChecksum;
      oa << mpAtlas;
      cout << "End to write save binary file" << endl;
    }
  }
}

bool System::LoadAtlas(int type) {
  string strFileVoc, strVocChecksum;
  bool isRead = false;

  string pathLoadFileName = "./";
  pathLoadFileName = pathLoadFileName.append(mStrLoadAtlasFromFile);
  pathLoadFileName = pathLoadFileName.append(".osa");

  if (type == TEXT_FILE) // File text
  {
    cout << "Starting to read the save text file " << endl;
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    if (!ifs.good()) {
      cout << "Load file not found" << endl;
      return false;
    }
    boost::archive::text_iarchive ia(ifs);
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> mpAtlas;
    cout << "End to load the save text file " << endl;
    isRead = true;
  } else if (type == BINARY_FILE) // File binary
  {
    cout << "Starting to read the save binary file" << endl;
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    if (!ifs.good()) {
      cout << "Load file not found" << endl;
      return false;
    }
    boost::archive::binary_iarchive ia(ifs);
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> mpAtlas;
    cout << "End to load the save binary file" << endl;
    isRead = true;
  }

  if (isRead) {
    // Check if the vocabulary is the same
    string strInputVocabularyChecksum =
        CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);

    if (strInputVocabularyChecksum.compare(strVocChecksum) != 0) {
      cout << "The vocabulary load isn't the same which the load session was "
              "created "
           << endl;
      cout << "-Vocabulary name: " << strFileVoc << endl;
      return false; // Both are differents
    }

    mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
    mpAtlas->SetORBVocabulary(mpVocabulary);
    mpAtlas->PostLoad();

    return true;
  }
  return false;
}

string System::CalculateCheckSum(string filename, int type) {
  string checksum = "";

  unsigned char c[MD5_DIGEST_LENGTH];

  std::ios_base::openmode flags = std::ios::in;
  if (type == BINARY_FILE) // Binary file
    flags = std::ios::in | std::ios::binary;

  ifstream f(filename.c_str(), flags);
  if (!f.is_open()) {
    cout << "[E] Unable to open the in file " << filename << " for Md5 hash."
         << endl;
    return checksum;
  }

  MD5_CTX md5Context;
  char buffer[1024];

  MD5_Init(&md5Context);
  while (int count = f.readsome(buffer, sizeof(buffer))) {
    MD5_Update(&md5Context, buffer, count);
  }

  f.close();

  MD5_Final(c, &md5Context);

  for (int i = 0; i < MD5_DIGEST_LENGTH; i++) {
    char aux[10];
    sprintf(aux, "%02x", c[i]);
    checksum = checksum + aux;
  }

  return checksum;
}

} // namespace ORB_SLAM3
