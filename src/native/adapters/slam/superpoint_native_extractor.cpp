#include "adapters/slam/superpoint_native_extractor.h"

#include "adapters/slam/superpoint_lightglue_frontend_client.h"
#include "adapters/slam/superpoint_runtime_options.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "adapters/slam/slam_env.h"

#if defined(SMART_DRONE_SUPERPOINT_TENSORRT_AVAILABLE)
#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_runtime_api.h>
#endif

namespace smartdrone::adapters::slam {

SuperPointNativeExtractor::SuperPointNativeExtractor() = default;

namespace {

constexpr int kSuperPointCellSize = 8;
constexpr int kSuperPointDescriptorDim = 256;
constexpr float kSuperPointThreshold = 0.0005f;
constexpr int kSuperPointBorder = 4;
constexpr int kSuperPointNmsRadius = 4;
constexpr float kStereoMaxYDiffPx = 2.0f;
constexpr float kStereoMinDisparityPx = 0.75f;
constexpr float kStereoMaxDisparityPx = 240.0f;
constexpr float kStereoRatio = 0.92f;
constexpr float kStereoMergeDistancePx = 3.0f;
constexpr int kLightGlueMinStereoPairsForSupplement = 96;
constexpr int kSuperPointStereoExtractionSlack = 96;

double DurationMs(const std::chrono::steady_clock::time_point &start,
                  const std::chrono::steady_clock::time_point &end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

std::string LowerCopy(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return text;
}

bool EnvFlag(const char *name, bool fallback) {
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') {
    return fallback;
  }
  const std::string text = LowerCopy(value);
  return !(text == "0" || text == "false" || text == "off" || text == "no");
}

int EnvIntClamped(const char *name, int fallback, int minValue, int maxValue) {
  const char *value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') {
    return std::clamp(fallback, minValue, maxValue);
  }
  char *end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value) {
    return std::clamp(fallback, minValue, maxValue);
  }
  return std::clamp(static_cast<int>(parsed), minValue, maxValue);
}

float StereoMinDisparityPx() {
  return std::clamp(EnvFloatValue("SMART_DRONE_STEREO_FEATURE_MIN_DISPARITY_PX",
                                  kStereoMinDisparityPx),
                    0.05f, kStereoMaxDisparityPx);
}

std::filesystem::path ResolveSuperPointEnginePath(const std::string &repoPath,
                                                  int widthHint,
                                                  int heightHint) {
  const char *envEngine = std::getenv("SMART_DRONE_SUPERPOINT_TRT_ENGINE");
  if (envEngine == nullptr || envEngine[0] == '\0') {
    envEngine = std::getenv("SUPERPOINT_TRT_ENGINE");
  }
  if (envEngine != nullptr && envEngine[0] != '\0' &&
      std::filesystem::exists(envEngine)) {
    return std::filesystem::path(envEngine);
  }

  const std::filesystem::path repo(repoPath);
  if (!repo.empty()) {
    std::vector<std::string> names;
    if (widthHint > 0 && heightHint > 0) {
      names.push_back("superpoint_dense_" + std::to_string(widthHint) + "x" +
                      std::to_string(heightHint) + "_fp16.engine");
      names.push_back("superpoint_dense_" + std::to_string(widthHint) + "x" +
                      std::to_string(heightHint) + "_fp32.engine");
    }
    names.push_back("superpoint_dense_640x409_fp16.engine");
    names.push_back("superpoint_dense_640x409_fp32.engine");
    names.push_back("superpoint_dense_640x480_fp16.engine");
    names.push_back("superpoint_dense_640x480_fp32.engine");
    names.push_back("superpoint.engine");
    for (const std::string &name : names) {
      const std::filesystem::path candidate = repo / "weights" / name;
      if (std::filesystem::exists(candidate)) {
        return candidate;
      }
    }
  }
  return {};
}

std::filesystem::path ResolveLightGlueEnginePath(const std::string &repoPath,
                                                 int maxPoints) {
  const char *envEngine = std::getenv("SMART_DRONE_LIGHTGLUE_TRT_ENGINE");
  if (envEngine != nullptr && envEngine[0] != '\0' &&
      std::filesystem::exists(envEngine)) {
    return std::filesystem::path(envEngine);
  }

  const std::filesystem::path repo(repoPath);
  if (!repo.empty()) {
    const int n = std::max(1, maxPoints);
    const std::vector<std::string> names = {
        "lightglue_superpoint_" + std::to_string(n) + "_fp16.engine",
        "lightglue_superpoint_" + std::to_string(n) + "_fp32.engine",
        "lightglue_superpoint_fp16.engine",
        "lightglue_superpoint_fp32.engine",
        "lightglue.engine",
    };
    for (const std::string &name : names) {
      const std::filesystem::path candidate = repo / "weights" / name;
      if (std::filesystem::exists(candidate)) {
        return candidate;
      }
    }
  }
  return {};
}

struct TensorBlob {
  enum class HostStorage {
    Float,
    Half,
    Int32,
  };

  std::vector<int> dims;
  std::vector<float> data;
  std::vector<uint16_t> halfData;
  std::vector<int32_t> intData;
  const float *floatData{nullptr};
  size_t floatElementCount{0};
  const void *pendingHostData{nullptr};
  size_t pendingElementCount{0};
  HostStorage hostStorage{HostStorage::Float};
  bool pinnedHostData{false};

  int Dim(size_t index) const { return index < dims.size() ? dims[index] : 0; }
  bool Empty() const {
    return dims.empty() || FloatData() == nullptr || FloatElementCount() == 0;
  }
  const float *FloatData() const {
    return floatData != nullptr ? floatData : data.data();
  }
  size_t FloatElementCount() const {
    return floatData != nullptr ? floatElementCount : data.size();
  }

  void ResetHostData() {
    data.clear();
    halfData.clear();
    intData.clear();
    floatData = nullptr;
    floatElementCount = 0;
    pendingHostData = nullptr;
    pendingElementCount = 0;
    hostStorage = HostStorage::Float;
    pinnedHostData = false;
  }
};

#if defined(SMART_DRONE_SUPERPOINT_TENSORRT_AVAILABLE)
struct CudaPinnedHostBuffer {
  void *ptr{nullptr};
  size_t bytes{0};

  ~CudaPinnedHostBuffer() { Release(); }

  CudaPinnedHostBuffer() = default;
  CudaPinnedHostBuffer(const CudaPinnedHostBuffer &) = delete;
  CudaPinnedHostBuffer &operator=(const CudaPinnedHostBuffer &) = delete;
  CudaPinnedHostBuffer(CudaPinnedHostBuffer &&other) noexcept
      : ptr(other.ptr), bytes(other.bytes) {
    other.ptr = nullptr;
    other.bytes = 0;
  }
  CudaPinnedHostBuffer &operator=(CudaPinnedHostBuffer &&other) noexcept {
    if (this != &other) {
      Release();
      ptr = other.ptr;
      bytes = other.bytes;
      other.ptr = nullptr;
      other.bytes = 0;
    }
    return *this;
  }

  void Release() {
    if (ptr != nullptr) {
      cudaFreeHost(ptr);
      ptr = nullptr;
      bytes = 0;
    }
  }

  bool Ensure(size_t requiredBytes) {
    if (ptr != nullptr && bytes >= requiredBytes) {
      return true;
    }
    Release();
    if (requiredBytes == 0) {
      return true;
    }
    if (cudaMallocHost(&ptr, requiredBytes) != cudaSuccess) {
      ptr = nullptr;
      bytes = 0;
      return false;
    }
    bytes = requiredBytes;
    return true;
  }
};
#endif

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

  void Add(const SuperPointPostStats &other) {
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

float At4D(const TensorBlob &blob, int b, int c, int y, int x) {
  const int channels = blob.Dim(1);
  const int height = blob.Dim(2);
  const int width = blob.Dim(3);
  const size_t index =
      (((static_cast<size_t>(b) * channels + c) * height + y) * width + x);
  return blob.FloatData()[index];
}

float HalfToFloat(uint16_t value) {
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

void NormalizeVector(float *values, int count) {
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
                     int targetWidth, std::vector<float> &batch) {
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

const cv::Mat &SuperPointNmsKernel() {
  static const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(kSuperPointNmsRadius * 2 + 1, kSuperPointNmsRadius * 2 + 1));
  return kernel;
}

void BuildHeatmap(const TensorBlob &detector, int batch, cv::Mat &heatmap) {
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
                       SuperPointPostStats *stats) {
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
                              SuperPointPostStats *stats) {
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
                 float y) {
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
                              float y, float *out) {
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
                             float y, float *out) {
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
                            std::vector<float> &hwc) {
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
                                 int width, float x, float y, float *out) {
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
                      SuperPointFeatureSet &rightOut) {
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
    const SuperPointFeatureSet &existingRight) {
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

#if defined(SMART_DRONE_SUPERPOINT_TENSORRT_AVAILABLE)
bool ScheduleTensorRtOutputCopy(void *devicePtr, nvinfer1::DataType dtype,
                                int64_t volume, cudaStream_t stream,
                                TensorBlob &output,
                                CudaPinnedHostBuffer *pinnedHostBuffer,
                                TensorRtForwardStats *stats,
                                const char *engineName, std::string *err) {
  output.ResetHostData();
  if (volume <= 0) {
    return false;
  }

  const size_t elementCount = static_cast<size_t>(volume);
  const bool usePinnedHost =
      pinnedHostBuffer != nullptr &&
      EnvFlag("SMART_DRONE_TRT_PINNED_HOST_OUTPUT", false);
  auto recordOutputCopy =
      [&](const std::chrono::steady_clock::time_point &start,
          const std::chrono::steady_clock::time_point &end, size_t bytes,
          bool pinned) {
        if (stats != nullptr) {
          stats->outputMs += DurationMs(start, end);
          stats->d2hBytes += bytes;
          stats->pinnedHostOutput = stats->pinnedHostOutput || pinned;
        }
      };

  if (dtype == nvinfer1::DataType::kFLOAT) {
    const size_t bytes = elementCount * sizeof(float);
    output.data.resize(elementCount);
    void *hostPtr = output.data.data();
    bool pinned = false;
    if (usePinnedHost && pinnedHostBuffer->Ensure(bytes)) {
      hostPtr = pinnedHostBuffer->ptr;
      output.pendingHostData = pinnedHostBuffer->ptr;
      output.pendingElementCount = elementCount;
      output.hostStorage = TensorBlob::HostStorage::Float;
      output.pinnedHostData = true;
      pinned = true;
    }
    const auto outputStartTp = std::chrono::steady_clock::now();
    if (cudaMemcpyAsync(hostPtr, devicePtr, bytes, cudaMemcpyDeviceToHost,
                        stream) != cudaSuccess) {
      if (err != nullptr) {
        *err = std::string("TensorRT failed to copy ") + engineName +
               " FP32 output";
      }
      return false;
    }
    const auto outputEndTp = std::chrono::steady_clock::now();
    recordOutputCopy(outputStartTp, outputEndTp, bytes, pinned);
    return true;
  }

  if (dtype == nvinfer1::DataType::kHALF) {
    const size_t bytes = elementCount * sizeof(uint16_t);
    output.data.resize(elementCount);
    output.halfData.resize(elementCount);
    void *hostPtr = output.halfData.data();
    bool pinned = false;
    if (usePinnedHost && pinnedHostBuffer->Ensure(bytes)) {
      hostPtr = pinnedHostBuffer->ptr;
      output.pendingHostData = pinnedHostBuffer->ptr;
      output.pendingElementCount = elementCount;
      output.hostStorage = TensorBlob::HostStorage::Half;
      output.pinnedHostData = true;
      pinned = true;
    } else {
      output.pendingHostData = output.halfData.data();
      output.pendingElementCount = elementCount;
      output.hostStorage = TensorBlob::HostStorage::Half;
    }
    const auto outputStartTp = std::chrono::steady_clock::now();
    if (cudaMemcpyAsync(hostPtr, devicePtr, bytes, cudaMemcpyDeviceToHost,
                        stream) != cudaSuccess) {
      if (err != nullptr) {
        *err = std::string("TensorRT failed to copy ") + engineName +
               " FP16 output";
      }
      return false;
    }
    const auto outputEndTp = std::chrono::steady_clock::now();
    recordOutputCopy(outputStartTp, outputEndTp, bytes, pinned);
    return true;
  }

  if (dtype == nvinfer1::DataType::kINT32) {
    const size_t bytes = elementCount * sizeof(int32_t);
    output.data.resize(elementCount);
    output.intData.resize(elementCount);
    void *hostPtr = output.intData.data();
    bool pinned = false;
    if (usePinnedHost && pinnedHostBuffer->Ensure(bytes)) {
      hostPtr = pinnedHostBuffer->ptr;
      output.pendingHostData = pinnedHostBuffer->ptr;
      output.pendingElementCount = elementCount;
      output.hostStorage = TensorBlob::HostStorage::Int32;
      output.pinnedHostData = true;
      pinned = true;
    } else {
      output.pendingHostData = output.intData.data();
      output.pendingElementCount = elementCount;
      output.hostStorage = TensorBlob::HostStorage::Int32;
    }
    const auto outputStartTp = std::chrono::steady_clock::now();
    if (cudaMemcpyAsync(hostPtr, devicePtr, bytes, cudaMemcpyDeviceToHost,
                        stream) != cudaSuccess) {
      if (err != nullptr) {
        *err = std::string("TensorRT failed to copy ") + engineName +
               " INT32 output";
      }
      return false;
    }
    const auto outputEndTp = std::chrono::steady_clock::now();
    recordOutputCopy(outputStartTp, outputEndTp, bytes, pinned);
    return true;
  }

  if (err != nullptr) {
    *err = std::string("TensorRT ") + engineName +
           " output has unsupported data type";
  }
  return false;
}

bool FinalizeTensorRtOutput(TensorBlob &output, TensorRtForwardStats *stats) {
  if (output.pendingHostData == nullptr || output.pendingElementCount == 0) {
    return true;
  }
  const auto convertStartTp = std::chrono::steady_clock::now();
  if (output.hostStorage == TensorBlob::HostStorage::Float) {
    if (output.pinnedHostData &&
        EnvFlag("SMART_DRONE_TRT_PINNED_HOST_VIEW", false)) {
      output.floatData = static_cast<const float *>(output.pendingHostData);
      output.floatElementCount = output.pendingElementCount;
    } else {
      if (output.pinnedHostData) {
        std::memcpy(output.data.data(), output.pendingHostData,
                    output.pendingElementCount * sizeof(float));
      }
      output.floatData = output.data.data();
      output.floatElementCount = output.data.size();
    }
  } else if (output.hostStorage == TensorBlob::HostStorage::Half) {
    const auto *src = static_cast<const uint16_t *>(output.pendingHostData);
    for (size_t i = 0; i < output.pendingElementCount; ++i) {
      output.data[i] = HalfToFloat(src[i]);
    }
    output.floatData = output.data.data();
    output.floatElementCount = output.data.size();
  } else if (output.hostStorage == TensorBlob::HostStorage::Int32) {
    const auto *src = static_cast<const int32_t *>(output.pendingHostData);
    for (size_t i = 0; i < output.pendingElementCount; ++i) {
      output.data[i] = static_cast<float>(src[i]);
    }
    output.floatData = output.data.data();
    output.floatElementCount = output.data.size();
  }
  const auto convertEndTp = std::chrono::steady_clock::now();
  if (stats != nullptr) {
    stats->outputConvertMs += DurationMs(convertStartTp, convertEndTp);
  }
  output.pendingHostData = nullptr;
  output.pendingElementCount = 0;
  output.hostStorage = TensorBlob::HostStorage::Float;
  return true;
}
#endif

void AppendStereoFeaturePairs(SuperPointFeatureSet &leftOut,
                              SuperPointFeatureSet &rightOut,
                              const SuperPointFeatureSet &leftSupplement,
                              const SuperPointFeatureSet &rightSupplement,
                              int maxPoints) {
  if (leftSupplement.descriptors.empty() ||
      rightSupplement.descriptors.empty() ||
      leftSupplement.descriptors.type() != CV_32F ||
      rightSupplement.descriptors.type() != CV_32F ||
      leftSupplement.descriptors.cols != rightSupplement.descriptors.cols ||
      leftSupplement.descriptors.rows !=
          static_cast<int>(leftSupplement.keypoints.size()) ||
      rightSupplement.descriptors.rows !=
          static_cast<int>(rightSupplement.keypoints.size())) {
    return;
  }
  if ((!leftOut.descriptors.empty() &&
       leftOut.descriptors.cols != leftSupplement.descriptors.cols) ||
      (!rightOut.descriptors.empty() &&
       rightOut.descriptors.cols != rightSupplement.descriptors.cols)) {
    return;
  }

  const int limit = std::max(1, maxPoints);
  const size_t sourceCount = std::min(leftSupplement.keypoints.size(),
                                      rightSupplement.keypoints.size());
  for (size_t i = 0;
       i < sourceCount && static_cast<int>(leftOut.keypoints.size()) < limit;
       ++i) {
    const cv::Point2f &leftPoint = leftSupplement.keypoints[i];
    const cv::Point2f &rightPoint = rightSupplement.keypoints[i];
    if (IsStereoFeaturePairNearExisting(leftPoint, rightPoint, leftOut,
                                        rightOut)) {
      continue;
    }
    leftOut.keypoints.push_back(leftPoint);
    rightOut.keypoints.push_back(rightPoint);
    leftOut.descriptors.push_back(
        leftSupplement.descriptors.row(static_cast<int>(i)));
    rightOut.descriptors.push_back(
        rightSupplement.descriptors.row(static_cast<int>(i)));
  }
}

#if defined(SMART_DRONE_SUPERPOINT_TENSORRT_AVAILABLE)

class TensorRtLogger final : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char *msg) noexcept override {
    if (severity <= Severity::kWARNING) {
      std::cerr << "[superpoint_trt] " << msg << "\n";
    }
  }
};

size_t TensorRtElementSize(nvinfer1::DataType type) {
  switch (type) {
  case nvinfer1::DataType::kFLOAT:
    return sizeof(float);
  case nvinfer1::DataType::kHALF:
    return 2;
  case nvinfer1::DataType::kINT8:
    return 1;
  case nvinfer1::DataType::kINT32:
    return 4;
  case nvinfer1::DataType::kBOOL:
    return 1;
  default:
    return 0;
  }
}

int64_t TensorRtVolume(const nvinfer1::Dims &dims) {
  int64_t volume = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) {
      return 0;
    }
    volume *= dims.d[i];
  }
  return volume;
}

std::vector<int> TensorRtDimsToVector(const nvinfer1::Dims &dims) {
  std::vector<int> out;
  out.reserve(static_cast<size_t>(dims.nbDims));
  for (int i = 0; i < dims.nbDims; ++i) {
    out.push_back(dims.d[i]);
  }
  return out;
}

class TensorRtSuperPointEngine {
public:
  ~TensorRtSuperPointEngine() { Release(); }

  bool Load(const std::filesystem::path &enginePath, std::string *err) {
    Release();
    std::ifstream input(enginePath, std::ios::binary);
    if (!input) {
      if (err != nullptr) {
        *err =
            "failed to open SuperPoint TensorRT engine: " + enginePath.string();
      }
      return false;
    }
    std::vector<char> bytes((std::istreambuf_iterator<char>(input)),
                            std::istreambuf_iterator<char>());
    if (bytes.empty()) {
      if (err != nullptr) {
        *err = "SuperPoint TensorRT engine is empty: " + enginePath.string();
      }
      return false;
    }
    initLibNvInferPlugins(&m_logger, "");
    m_runtime.reset(nvinfer1::createInferRuntime(m_logger));
    if (!m_runtime) {
      if (err != nullptr) {
        *err = "failed to create SuperPoint TensorRT runtime";
      }
      return false;
    }
    m_engine.reset(
        m_runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!m_engine) {
      if (err != nullptr) {
        *err = "failed to deserialize SuperPoint TensorRT engine: " +
               enginePath.string();
      }
      return false;
    }
    m_context.reset(m_engine->createExecutionContext());
    if (!m_context) {
      if (err != nullptr) {
        *err = "failed to create SuperPoint TensorRT execution context";
      }
      return false;
    }
    if (cudaStreamCreate(&m_stream) != cudaSuccess) {
      if (err != nullptr) {
        *err = "failed to create SuperPoint TensorRT CUDA stream";
      }
      return false;
    }
    m_eventTimingEnabled = EnvFlag("SMART_DRONE_TRT_EVENT_TIMING", false);
    if (m_eventTimingEnabled &&
        (cudaEventCreate(&m_computeStartEvent) != cudaSuccess ||
         cudaEventCreate(&m_computeEndEvent) != cudaSuccess ||
         cudaEventCreate(&m_outputStartEvent) != cudaSuccess ||
         cudaEventCreate(&m_outputEndEvent) != cudaSuccess)) {
      if (err != nullptr) {
        *err = "failed to create SuperPoint TensorRT CUDA timing events";
      }
      return false;
    }
    m_inputIndex = FindBinding({"image", "images", "input"}, true);
    m_detectorIndex =
        FindBinding({"detector_logits", "scores", "output0"}, false);
    m_descriptorIndex =
        FindBinding({"dense_descriptors", "descriptors", "output1"}, false);
    if (m_inputIndex < 0 || m_detectorIndex < 0 || m_descriptorIndex < 0) {
      if (err != nullptr) {
        *err = "SuperPoint TensorRT engine bindings are missing expected "
               "input/output names";
      }
      return false;
    }
    return true;
  }

  bool Forward(const std::vector<float> &batch, int batchSize, int height,
               int width, TensorBlob &detector, TensorBlob &descriptors,
               TensorRtForwardStats *stats, std::string *err) {
    if (stats != nullptr) {
      *stats = TensorRtForwardStats{};
      stats->eventTimingEnabled = m_eventTimingEnabled;
    }
    nvinfer1::Dims inputDims{};
    inputDims.nbDims = 4;
    inputDims.d[0] = batchSize;
    inputDims.d[1] = 1;
    inputDims.d[2] = height;
    inputDims.d[3] = width;
    if (!m_context->setBindingDimensions(m_inputIndex, inputDims)) {
      if (err != nullptr) {
        *err = "TensorRT failed to set SuperPoint input dimensions";
      }
      return false;
    }
    if (!m_context->allInputDimensionsSpecified()) {
      if (err != nullptr) {
        *err = "TensorRT SuperPoint input dimensions are incomplete";
      }
      return false;
    }
    if (!EnsureBindingBuffer(m_inputIndex, inputDims,
                             m_engine->getBindingDataType(m_inputIndex), err)) {
      return false;
    }
    const size_t inputBytes = batch.size() * sizeof(float);
    const auto h2dStartTp = std::chrono::steady_clock::now();
    if (cudaMemcpyAsync(m_bindings[static_cast<size_t>(m_inputIndex)],
                        batch.data(), inputBytes, cudaMemcpyHostToDevice,
                        m_stream) != cudaSuccess) {
      if (err != nullptr) {
        *err = "TensorRT failed to copy SuperPoint input";
      }
      return false;
    }
    const auto h2dEndTp = std::chrono::steady_clock::now();
    if (stats != nullptr) {
      stats->h2dMs += DurationMs(h2dStartTp, h2dEndTp);
      stats->h2dBytes += inputBytes;
    }
    for (int index : {m_detectorIndex, m_descriptorIndex}) {
      const nvinfer1::Dims dims = m_context->getBindingDimensions(index);
      if (!EnsureBindingBuffer(index, dims, m_engine->getBindingDataType(index),
                               err)) {
        return false;
      }
    }
    const auto enqueueStartTp = std::chrono::steady_clock::now();
    if (stats != nullptr && m_eventTimingEnabled) {
      cudaEventRecord(m_computeStartEvent, m_stream);
    }
    if (!m_context->enqueueV2(m_bindings.data(), m_stream, nullptr)) {
      if (err != nullptr) {
        *err = "TensorRT SuperPoint enqueue failed";
      }
      return false;
    }
    if (stats != nullptr && m_eventTimingEnabled) {
      cudaEventRecord(m_computeEndEvent, m_stream);
    }
    const auto enqueueEndTp = std::chrono::steady_clock::now();
    if (stats != nullptr) {
      stats->enqueueMs += DurationMs(enqueueStartTp, enqueueEndTp);
    }
    if (stats != nullptr && m_eventTimingEnabled) {
      cudaEventRecord(m_outputStartEvent, m_stream);
    }
    if (!ReadOutput(m_detectorIndex, detector, stats, err) ||
        !ReadOutput(m_descriptorIndex, descriptors, stats, err)) {
      return false;
    }
    if (stats != nullptr && m_eventTimingEnabled) {
      cudaEventRecord(m_outputEndEvent, m_stream);
    }
    const auto syncStartTp = std::chrono::steady_clock::now();
    if (cudaStreamSynchronize(m_stream) != cudaSuccess) {
      if (err != nullptr) {
        *err = "TensorRT SuperPoint stream synchronize failed";
      }
      return false;
    }
    const auto syncEndTp = std::chrono::steady_clock::now();
    const bool outputsReady = FinalizeTensorRtOutput(detector, stats) &&
                              FinalizeTensorRtOutput(descriptors, stats);
    if (stats != nullptr) {
      stats->syncMs += DurationMs(syncStartTp, syncEndTp);
      if (m_eventTimingEnabled) {
        float computeMs = 0.0f;
        float outputMs = 0.0f;
        if (cudaEventElapsedTime(&computeMs, m_computeStartEvent,
                                 m_computeEndEvent) == cudaSuccess) {
          stats->gpuComputeMs += static_cast<double>(computeMs);
        }
        if (cudaEventElapsedTime(&outputMs, m_outputStartEvent,
                                 m_outputEndEvent) == cudaSuccess) {
          stats->gpuOutputMs += static_cast<double>(outputMs);
        }
      }
    }
    if (!outputsReady) {
      return false;
    }
    return !detector.Empty() && !descriptors.Empty();
  }

  bool PreferredInputSize(int &height, int &width) const {
    if (!m_engine || m_inputIndex < 0) {
      return false;
    }
    nvinfer1::Dims dims = m_engine->getBindingDimensions(m_inputIndex);
    if (m_engine->getNbOptimizationProfiles() > 0) {
      dims = m_engine->getProfileDimensions(m_inputIndex, 0,
                                            nvinfer1::OptProfileSelector::kOPT);
    }
    if (dims.nbDims == 4 && dims.d[2] > 0 && dims.d[3] > 0) {
      height = dims.d[2];
      width = dims.d[3];
      return true;
    }
    return false;
  }

  bool SupportsBatchSize(int batchSize) const {
    if (!m_engine || m_inputIndex < 0 || batchSize <= 0) {
      return batchSize == 1;
    }
    const nvinfer1::Dims dims = m_engine->getBindingDimensions(m_inputIndex);
    if (dims.nbDims != 4) {
      return batchSize == 1;
    }
    if (dims.d[0] > 0) {
      return dims.d[0] == batchSize;
    }
    const int profiles = m_engine->getNbOptimizationProfiles();
    for (int profile = 0; profile < profiles; ++profile) {
      const nvinfer1::Dims minDims = m_engine->getProfileDimensions(
          m_inputIndex, profile, nvinfer1::OptProfileSelector::kMIN);
      const nvinfer1::Dims maxDims = m_engine->getProfileDimensions(
          m_inputIndex, profile, nvinfer1::OptProfileSelector::kMAX);
      if (minDims.nbDims == 4 && maxDims.nbDims == 4 &&
          minDims.d[0] <= batchSize && batchSize <= maxDims.d[0]) {
        return true;
      }
    }
    return batchSize == 1;
  }

private:
  struct DestroyRuntime {
    void operator()(nvinfer1::IRuntime *ptr) const {
      if (ptr != nullptr) {
        ptr->destroy();
      }
    }
  };
  struct DestroyEngine {
    void operator()(nvinfer1::ICudaEngine *ptr) const {
      if (ptr != nullptr) {
        ptr->destroy();
      }
    }
  };
  struct DestroyContext {
    void operator()(nvinfer1::IExecutionContext *ptr) const {
      if (ptr != nullptr) {
        ptr->destroy();
      }
    }
  };

  int FindBinding(std::initializer_list<const char *> names, bool input) const {
    for (const char *name : names) {
      const int index = m_engine->getBindingIndex(name);
      if (index >= 0 && m_engine->bindingIsInput(index) == input) {
        return index;
      }
    }
    if (input) {
      for (int i = 0; i < m_engine->getNbBindings(); ++i) {
        if (m_engine->bindingIsInput(i)) {
          return i;
        }
      }
      return -1;
    }
    for (int i = 0; i < m_engine->getNbBindings(); ++i) {
      if (!m_engine->bindingIsInput(i)) {
        return i;
      }
    }
    return -1;
  }

  bool EnsureBindingBuffer(int index, const nvinfer1::Dims &dims,
                           nvinfer1::DataType type, std::string *err) {
    const size_t bindingIndex = static_cast<size_t>(index);
    if (m_bindings.size() < static_cast<size_t>(m_engine->getNbBindings())) {
      m_bindings.assign(static_cast<size_t>(m_engine->getNbBindings()),
                        nullptr);
      m_bindingBytes.assign(static_cast<size_t>(m_engine->getNbBindings()), 0);
      m_pinnedHostOutputs.clear();
      m_pinnedHostOutputs.resize(
          static_cast<size_t>(m_engine->getNbBindings()));
    }
    const int64_t volume = TensorRtVolume(dims);
    const size_t elementSize = TensorRtElementSize(type);
    if (volume <= 0 || elementSize == 0) {
      if (err != nullptr) {
        *err = "invalid SuperPoint TensorRT binding dimensions";
      }
      return false;
    }
    const size_t bytes = static_cast<size_t>(volume) * elementSize;
    if (m_bindingBytes[bindingIndex] >= bytes &&
        m_bindings[bindingIndex] != nullptr) {
      return true;
    }
    if (m_bindings[bindingIndex] != nullptr) {
      cudaFree(m_bindings[bindingIndex]);
      m_bindings[bindingIndex] = nullptr;
    }
    if (cudaMalloc(&m_bindings[bindingIndex], bytes) != cudaSuccess) {
      if (err != nullptr) {
        *err = "failed to allocate SuperPoint TensorRT binding buffer";
      }
      return false;
    }
    m_bindingBytes[bindingIndex] = bytes;
    return true;
  }

  bool ReadOutput(int index, TensorBlob &output, TensorRtForwardStats *stats,
                  std::string *err) {
    const nvinfer1::Dims dims = m_context->getBindingDimensions(index);
    output.dims = TensorRtDimsToVector(dims);
    const int64_t volume = TensorRtVolume(dims);
    if (volume <= 0) {
      return false;
    }
    const nvinfer1::DataType dtype = m_engine->getBindingDataType(index);
    return ScheduleTensorRtOutputCopy(
        m_bindings[static_cast<size_t>(index)], dtype, volume, m_stream, output,
        &m_pinnedHostOutputs[static_cast<size_t>(index)], stats, "SuperPoint",
        err);
  }

  void Release() {
    if (m_stream != nullptr) {
      cudaStreamSynchronize(m_stream);
    }
    m_context.reset();
    if (m_computeStartEvent != nullptr) {
      cudaEventDestroy(m_computeStartEvent);
      m_computeStartEvent = nullptr;
    }
    if (m_computeEndEvent != nullptr) {
      cudaEventDestroy(m_computeEndEvent);
      m_computeEndEvent = nullptr;
    }
    if (m_outputStartEvent != nullptr) {
      cudaEventDestroy(m_outputStartEvent);
      m_outputStartEvent = nullptr;
    }
    if (m_outputEndEvent != nullptr) {
      cudaEventDestroy(m_outputEndEvent);
      m_outputEndEvent = nullptr;
    }
    for (void *ptr : m_bindings) {
      if (ptr != nullptr) {
        cudaFree(ptr);
      }
    }
    m_bindings.clear();
    m_bindingBytes.clear();
    m_pinnedHostOutputs.clear();
    if (m_stream != nullptr) {
      cudaStreamDestroy(m_stream);
      m_stream = nullptr;
    }
    m_engine.reset();
    m_runtime.reset();
  }

  TensorRtLogger m_logger;
  std::unique_ptr<nvinfer1::IRuntime, DestroyRuntime> m_runtime;
  std::unique_ptr<nvinfer1::ICudaEngine, DestroyEngine> m_engine;
  std::unique_ptr<nvinfer1::IExecutionContext, DestroyContext> m_context;
  cudaStream_t m_stream{nullptr};
  cudaEvent_t m_computeStartEvent{nullptr};
  cudaEvent_t m_computeEndEvent{nullptr};
  cudaEvent_t m_outputStartEvent{nullptr};
  cudaEvent_t m_outputEndEvent{nullptr};
  bool m_eventTimingEnabled{false};
  std::vector<void *> m_bindings;
  std::vector<size_t> m_bindingBytes;
  std::vector<CudaPinnedHostBuffer> m_pinnedHostOutputs;
  int m_inputIndex{-1};
  int m_detectorIndex{-1};
  int m_descriptorIndex{-1};
};

class TensorRtLightGlueEngine {
public:
  ~TensorRtLightGlueEngine() { Release(); }

  bool Load(const std::filesystem::path &enginePath, std::string *err) {
    Release();
    std::ifstream input(enginePath, std::ios::binary);
    if (!input) {
      if (err != nullptr) {
        *err =
            "failed to open LightGlue TensorRT engine: " + enginePath.string();
      }
      return false;
    }
    std::vector<char> bytes((std::istreambuf_iterator<char>(input)),
                            std::istreambuf_iterator<char>());
    if (bytes.empty()) {
      if (err != nullptr) {
        *err = "LightGlue TensorRT engine is empty: " + enginePath.string();
      }
      return false;
    }
    initLibNvInferPlugins(&m_logger, "");
    m_runtime.reset(nvinfer1::createInferRuntime(m_logger));
    if (!m_runtime) {
      if (err != nullptr) {
        *err = "failed to create LightGlue TensorRT runtime";
      }
      return false;
    }
    m_engine.reset(
        m_runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!m_engine) {
      if (err != nullptr) {
        *err = "failed to deserialize LightGlue TensorRT engine: " +
               enginePath.string();
      }
      return false;
    }
    m_context.reset(m_engine->createExecutionContext());
    if (!m_context) {
      if (err != nullptr) {
        *err = "failed to create LightGlue TensorRT execution context";
      }
      return false;
    }
    if (cudaStreamCreate(&m_stream) != cudaSuccess) {
      if (err != nullptr) {
        *err = "failed to create LightGlue TensorRT CUDA stream";
      }
      return false;
    }
    m_eventTimingEnabled = EnvFlag("SMART_DRONE_TRT_EVENT_TIMING", false);
    if (m_eventTimingEnabled &&
        (cudaEventCreate(&m_computeStartEvent) != cudaSuccess ||
         cudaEventCreate(&m_computeEndEvent) != cudaSuccess ||
         cudaEventCreate(&m_outputStartEvent) != cudaSuccess ||
         cudaEventCreate(&m_outputEndEvent) != cudaSuccess)) {
      if (err != nullptr) {
        *err = "failed to create LightGlue TensorRT CUDA timing events";
      }
      return false;
    }
    m_kpts0Index = FindBinding({"keypoints0"}, true);
    m_kpts1Index = FindBinding({"keypoints1"}, true);
    m_desc0Index = FindBinding({"descriptors0"}, true);
    m_desc1Index = FindBinding({"descriptors1"}, true);
    m_size0Index = FindBinding({"image_size0"}, true);
    m_size1Index = FindBinding({"image_size1"}, true);
    m_scoresIndex =
        FindBinding({"assignment_scores", "scores", "output0"}, false);
    if (m_kpts0Index < 0 || m_kpts1Index < 0 || m_desc0Index < 0 ||
        m_desc1Index < 0 || m_size0Index < 0 || m_size1Index < 0 ||
        m_scoresIndex < 0) {
      if (err != nullptr) {
        *err = "LightGlue TensorRT engine bindings are missing expected "
               "input/output names";
      }
      return false;
    }
    return true;
  }

  int FixedPointCount() const {
    if (!m_engine || m_kpts0Index < 0) {
      return 0;
    }
    nvinfer1::Dims dims = m_engine->getBindingDimensions(m_kpts0Index);
    if (dims.nbDims == 3 && dims.d[1] > 0) {
      return dims.d[1];
    }
    const int profiles = m_engine->getNbOptimizationProfiles();
    for (int profile = 0; profile < profiles; ++profile) {
      const nvinfer1::Dims minDims = m_engine->getProfileDimensions(
          m_kpts0Index, profile, nvinfer1::OptProfileSelector::kMIN);
      const nvinfer1::Dims maxDims = m_engine->getProfileDimensions(
          m_kpts0Index, profile, nvinfer1::OptProfileSelector::kMAX);
      if (minDims.nbDims == 3 && maxDims.nbDims == 3 && minDims.d[1] > 0 &&
          minDims.d[1] == maxDims.d[1]) {
        return minDims.d[1];
      }
    }
    return 0;
  }

  bool Forward(const std::vector<float> &keypoints0,
               const std::vector<float> &keypoints1,
               const std::vector<float> &descriptors0,
               const std::vector<float> &descriptors1,
               const std::array<float, 2> &imageSize0,
               const std::array<float, 2> &imageSize1, int pointCount,
               TensorBlob &scores, TensorRtForwardStats *stats,
               std::string *err) {
    if (stats != nullptr) {
      *stats = TensorRtForwardStats{};
      stats->eventTimingEnabled = m_eventTimingEnabled;
    }
    if (pointCount <= 0) {
      return false;
    }
    if (!SetInput(m_kpts0Index, {1, pointCount, 2}, keypoints0, stats, err) ||
        !SetInput(m_kpts1Index, {1, pointCount, 2}, keypoints1, stats, err) ||
        !SetInput(m_desc0Index, {1, pointCount, kSuperPointDescriptorDim},
                  descriptors0, stats, err) ||
        !SetInput(m_desc1Index, {1, pointCount, kSuperPointDescriptorDim},
                  descriptors1, stats, err)) {
      return false;
    }
    const std::vector<float> size0{imageSize0[0], imageSize0[1]};
    const std::vector<float> size1{imageSize1[0], imageSize1[1]};
    if (!SetInput(m_size0Index, {1, 2}, size0, stats, err) ||
        !SetInput(m_size1Index, {1, 2}, size1, stats, err)) {
      return false;
    }
    if (!m_context->allInputDimensionsSpecified()) {
      if (err != nullptr) {
        *err = "TensorRT LightGlue input dimensions are incomplete";
      }
      return false;
    }
    for (int index : {m_scoresIndex}) {
      const nvinfer1::Dims dims = m_context->getBindingDimensions(index);
      if (!EnsureBindingBuffer(index, dims, m_engine->getBindingDataType(index),
                               err)) {
        return false;
      }
    }
    const auto enqueueStartTp = std::chrono::steady_clock::now();
    if (stats != nullptr && m_eventTimingEnabled) {
      cudaEventRecord(m_computeStartEvent, m_stream);
    }
    if (!m_context->enqueueV2(m_bindings.data(), m_stream, nullptr)) {
      if (err != nullptr) {
        *err = "TensorRT LightGlue enqueue failed";
      }
      return false;
    }
    if (stats != nullptr && m_eventTimingEnabled) {
      cudaEventRecord(m_computeEndEvent, m_stream);
    }
    const auto enqueueEndTp = std::chrono::steady_clock::now();
    if (stats != nullptr) {
      stats->enqueueMs += DurationMs(enqueueStartTp, enqueueEndTp);
    }
    if (stats != nullptr && m_eventTimingEnabled) {
      cudaEventRecord(m_outputStartEvent, m_stream);
    }
    if (!ReadOutput(m_scoresIndex, scores, stats, err)) {
      return false;
    }
    if (stats != nullptr && m_eventTimingEnabled) {
      cudaEventRecord(m_outputEndEvent, m_stream);
    }
    const auto syncStartTp = std::chrono::steady_clock::now();
    if (cudaStreamSynchronize(m_stream) != cudaSuccess) {
      if (err != nullptr) {
        *err = "TensorRT LightGlue stream synchronize failed";
      }
      return false;
    }
    const auto syncEndTp = std::chrono::steady_clock::now();
    const bool outputReady = FinalizeTensorRtOutput(scores, stats);
    if (stats != nullptr) {
      stats->syncMs += DurationMs(syncStartTp, syncEndTp);
      if (m_eventTimingEnabled) {
        float computeMs = 0.0f;
        float outputMs = 0.0f;
        if (cudaEventElapsedTime(&computeMs, m_computeStartEvent,
                                 m_computeEndEvent) == cudaSuccess) {
          stats->gpuComputeMs += static_cast<double>(computeMs);
        }
        if (cudaEventElapsedTime(&outputMs, m_outputStartEvent,
                                 m_outputEndEvent) == cudaSuccess) {
          stats->gpuOutputMs += static_cast<double>(outputMs);
        }
      }
    }
    if (!outputReady) {
      return false;
    }
    return !scores.Empty();
  }

private:
  struct DestroyRuntime {
    void operator()(nvinfer1::IRuntime *ptr) const {
      if (ptr != nullptr) {
        ptr->destroy();
      }
    }
  };
  struct DestroyEngine {
    void operator()(nvinfer1::ICudaEngine *ptr) const {
      if (ptr != nullptr) {
        ptr->destroy();
      }
    }
  };
  struct DestroyContext {
    void operator()(nvinfer1::IExecutionContext *ptr) const {
      if (ptr != nullptr) {
        ptr->destroy();
      }
    }
  };

  int FindBinding(std::initializer_list<const char *> names, bool input) const {
    for (const char *name : names) {
      const int index = m_engine->getBindingIndex(name);
      if (index >= 0 && m_engine->bindingIsInput(index) == input) {
        return index;
      }
    }
    return -1;
  }

  static nvinfer1::Dims MakeDims(const std::vector<int> &dims) {
    nvinfer1::Dims out{};
    out.nbDims = static_cast<int>(std::min<size_t>(dims.size(), 8));
    for (int i = 0; i < out.nbDims; ++i) {
      out.d[i] = dims[static_cast<size_t>(i)];
    }
    return out;
  }

  bool SetInput(int index, const std::vector<int> &dims,
                const std::vector<float> &data, TensorRtForwardStats *stats,
                std::string *err) {
    const nvinfer1::Dims trtDims = MakeDims(dims);
    if (!m_context->setBindingDimensions(index, trtDims)) {
      if (err != nullptr) {
        *err = "TensorRT failed to set LightGlue input dimensions";
      }
      return false;
    }
    if (!EnsureBindingBuffer(index, trtDims,
                             m_engine->getBindingDataType(index), err)) {
      return false;
    }
    const size_t bytes = data.size() * sizeof(float);
    const auto h2dStartTp = std::chrono::steady_clock::now();
    if (cudaMemcpyAsync(m_bindings[static_cast<size_t>(index)], data.data(),
                        bytes, cudaMemcpyHostToDevice,
                        m_stream) != cudaSuccess) {
      if (err != nullptr) {
        *err = "TensorRT failed to copy LightGlue input";
      }
      return false;
    }
    const auto h2dEndTp = std::chrono::steady_clock::now();
    if (stats != nullptr) {
      stats->h2dMs += DurationMs(h2dStartTp, h2dEndTp);
      stats->h2dBytes += bytes;
    }
    return true;
  }

  bool EnsureBindingBuffer(int index, const nvinfer1::Dims &dims,
                           nvinfer1::DataType type, std::string *err) {
    const size_t bindingIndex = static_cast<size_t>(index);
    if (m_bindings.size() < static_cast<size_t>(m_engine->getNbBindings())) {
      m_bindings.assign(static_cast<size_t>(m_engine->getNbBindings()),
                        nullptr);
      m_bindingBytes.assign(static_cast<size_t>(m_engine->getNbBindings()), 0);
      m_pinnedHostOutputs.clear();
      m_pinnedHostOutputs.resize(
          static_cast<size_t>(m_engine->getNbBindings()));
    }
    const int64_t volume = TensorRtVolume(dims);
    const size_t elementSize = TensorRtElementSize(type);
    if (volume <= 0 || elementSize == 0) {
      if (err != nullptr) {
        *err = "invalid LightGlue TensorRT binding dimensions";
      }
      return false;
    }
    const size_t bytes = static_cast<size_t>(volume) * elementSize;
    if (m_bindingBytes[bindingIndex] >= bytes &&
        m_bindings[bindingIndex] != nullptr) {
      return true;
    }
    if (m_bindings[bindingIndex] != nullptr) {
      cudaFree(m_bindings[bindingIndex]);
      m_bindings[bindingIndex] = nullptr;
    }
    if (cudaMalloc(&m_bindings[bindingIndex], bytes) != cudaSuccess) {
      if (err != nullptr) {
        *err = "failed to allocate LightGlue TensorRT binding buffer";
      }
      return false;
    }
    m_bindingBytes[bindingIndex] = bytes;
    return true;
  }

  bool ReadOutput(int index, TensorBlob &output, TensorRtForwardStats *stats,
                  std::string *err) {
    const nvinfer1::Dims dims = m_context->getBindingDimensions(index);
    output.dims = TensorRtDimsToVector(dims);
    const int64_t volume = TensorRtVolume(dims);
    if (volume <= 0) {
      return false;
    }
    const nvinfer1::DataType dtype = m_engine->getBindingDataType(index);
    return ScheduleTensorRtOutputCopy(
        m_bindings[static_cast<size_t>(index)], dtype, volume, m_stream, output,
        &m_pinnedHostOutputs[static_cast<size_t>(index)], stats, "LightGlue",
        err);
  }

  void Release() {
    if (m_stream != nullptr) {
      cudaStreamSynchronize(m_stream);
    }
    m_context.reset();
    if (m_computeStartEvent != nullptr) {
      cudaEventDestroy(m_computeStartEvent);
      m_computeStartEvent = nullptr;
    }
    if (m_computeEndEvent != nullptr) {
      cudaEventDestroy(m_computeEndEvent);
      m_computeEndEvent = nullptr;
    }
    if (m_outputStartEvent != nullptr) {
      cudaEventDestroy(m_outputStartEvent);
      m_outputStartEvent = nullptr;
    }
    if (m_outputEndEvent != nullptr) {
      cudaEventDestroy(m_outputEndEvent);
      m_outputEndEvent = nullptr;
    }
    for (void *ptr : m_bindings) {
      if (ptr != nullptr) {
        cudaFree(ptr);
      }
    }
    m_bindings.clear();
    m_bindingBytes.clear();
    m_pinnedHostOutputs.clear();
    if (m_stream != nullptr) {
      cudaStreamDestroy(m_stream);
      m_stream = nullptr;
    }
    m_engine.reset();
    m_runtime.reset();
  }

  TensorRtLogger m_logger;
  std::unique_ptr<nvinfer1::IRuntime, DestroyRuntime> m_runtime;
  std::unique_ptr<nvinfer1::ICudaEngine, DestroyEngine> m_engine;
  std::unique_ptr<nvinfer1::IExecutionContext, DestroyContext> m_context;
  cudaStream_t m_stream{nullptr};
  cudaEvent_t m_computeStartEvent{nullptr};
  cudaEvent_t m_computeEndEvent{nullptr};
  cudaEvent_t m_outputStartEvent{nullptr};
  cudaEvent_t m_outputEndEvent{nullptr};
  bool m_eventTimingEnabled{false};
  std::vector<void *> m_bindings;
  std::vector<size_t> m_bindingBytes;
  std::vector<CudaPinnedHostBuffer> m_pinnedHostOutputs;
  int m_kpts0Index{-1};
  int m_kpts1Index{-1};
  int m_desc0Index{-1};
  int m_desc1Index{-1};
  int m_size0Index{-1};
  int m_size1Index{-1};
  int m_scoresIndex{-1};
};

#endif

} // namespace

struct SuperPointNativeExtractor::Impl {
#if defined(SMART_DRONE_SUPERPOINT_TENSORRT_AVAILABLE)
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
            std::string *err) {
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

  int maxPointsForLightGlue() const {
    return LoadSuperPointTensorRtRuntimeOptions().lightGluePoints;
  }

  bool PopulateOutputFromTensors(
      const TensorBlob &detectorBlob, const TensorBlob &descriptorBlob,
      int tensorBatch, const cv::Mat &sourceImage, int targetHeight,
      int targetWidth, int maxPoints, int descriptorLimit,
      SuperPointFeatureSet &output, SuperPointPostScratch &scratch,
      SuperPointPostStats *postStats, std::string *err) {
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
                             std::string *err) {
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
                          std::string *err) {
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
#endif
};

SuperPointNativeExtractor::~SuperPointNativeExtractor() = default;

bool SuperPointNativeExtractor::PrepareGrayImage(const cv::Mat &gray,
                                                 cv::Mat &gray8,
                                                 std::string *err) {
  if (gray.type() == CV_8UC1 && gray.isContinuous()) {
    gray8 = gray;
    return true;
  }
  if (gray.channels() == 1) {
    gray.convertTo(gray8, CV_8UC1);
  } else {
    cv::cvtColor(gray, gray8, cv::COLOR_BGR2GRAY);
  }
  if (!gray8.isContinuous()) {
    gray8 = gray8.clone();
  }
  if (gray8.empty()) {
    if (err != nullptr) {
      *err = "SuperPoint native gray image preparation failed";
    }
    return false;
  }
  return true;
}

bool SuperPointNativeExtractor::Start(const std::string &repoPath,
                                      const std::string &device, int topK,
                                      int maxPoints, std::string *err) {
  m_topK = topK;
  m_maxPoints = maxPoints;
  m_lastStats = Stats{};
#if defined(SMART_DRONE_SUPERPOINT_TENSORRT_AVAILABLE)
  m_impl = std::make_unique<Impl>();
  if (!m_impl->Load(repoPath, device, err)) {
    m_impl.reset();
    m_running = false;
    return false;
  }
  m_running = true;
  return true;
#else
  m_running = false;
  if (err != nullptr) {
    *err = "native TensorRT SuperPoint backend is unavailable in this target";
  }
  return false;
#endif
}

void SuperPointNativeExtractor::Stop() {
  m_running = false;
  m_lastStats = Stats{};
  m_impl.reset();
}

bool SuperPointNativeExtractor::Running() const { return m_running; }

SuperPointNativeExtractor::Stats SuperPointNativeExtractor::LastStats() const {
  return m_lastStats;
}

void SuperPointNativeExtractor::SetLightGlueEveryNOverride(int everyN) {
  m_lightGlueEveryNOverride = everyN > 0 ? std::clamp(everyN, 1, 120) : 0;
}

bool SuperPointNativeExtractor::Detect(const cv::Mat &gray,
                                       std::vector<cv::Point2f> &outPoints,
                                       std::string *err) {
  SuperPointFeatureSet features;
  if (!DetectAndCompute(gray, features, err)) {
    return false;
  }
  outPoints = std::move(features.keypoints);
  return true;
}

bool SuperPointNativeExtractor::DetectAndCompute(
    const cv::Mat &gray, SuperPointFeatureSet &outFeatures, std::string *err) {
  outFeatures = SuperPointFeatureSet{};
  m_lastStats = Stats{};
#if defined(SMART_DRONE_SUPERPOINT_TENSORRT_AVAILABLE)
  if (!m_running || !m_impl) {
    if (err != nullptr) {
      *err = "SuperPoint TensorRT backend not running";
    }
    return false;
  }
  const auto totalStartTp = std::chrono::steady_clock::now();
  cv::Mat gray8;
  const auto prepareStartTp = totalStartTp;
  if (!PrepareGrayImage(gray, gray8, err)) {
    return false;
  }
  const auto prepareEndTp = std::chrono::steady_clock::now();
  std::vector<SuperPointFeatureSet> outputs;
  const auto inferStartTp = prepareEndTp;
  double inputMs = 0.0;
  double forwardMs = 0.0;
  double postMs = 0.0;
  const int detectMaxPoints = std::max(m_topK, m_maxPoints);
  if (!m_impl->DetectAndComputeBatch({gray8}, detectMaxPoints, detectMaxPoints,
                                     outputs, &inputMs, &forwardMs, &postMs,
                                     err) ||
      outputs.empty()) {
    return false;
  }
  const auto inferEndTp = std::chrono::steady_clock::now();
  outFeatures = std::move(outputs.front());
  m_lastStats.prepareMs = DurationMs(prepareStartTp, prepareEndTp);
  m_lastStats.inputMs = inputMs;
  m_lastStats.forwardMs = forwardMs;
  m_lastStats.postMs = postMs;
  m_lastStats.inferMs = DurationMs(inferStartTp, inferEndTp);
  m_lastStats.totalMs = DurationMs(totalStartTp, inferEndTp);
  m_lastStats.imageCount = 1;
  m_lastStats.payloadBytes = static_cast<uint32_t>(gray8.total());
  return true;
#else
  if (err != nullptr) {
    *err = "native TensorRT SuperPoint backend is compiled out";
  }
  (void)gray;
  return false;
#endif
}

bool SuperPointNativeExtractor::DetectAndComputeStereo(
    const cv::Mat &leftGray, const cv::Mat &rightGray,
    SuperPointFeatureSet &leftFeatures, SuperPointFeatureSet &rightFeatures,
    std::string *err) {
  leftFeatures = SuperPointFeatureSet{};
  rightFeatures = SuperPointFeatureSet{};
  m_lastStats = Stats{};
#if defined(SMART_DRONE_SUPERPOINT_TENSORRT_AVAILABLE)
  if (!m_running || !m_impl) {
    if (err != nullptr) {
      *err = "SuperPoint TensorRT backend not running";
    }
    return false;
  }
  const auto totalStartTp = std::chrono::steady_clock::now();
  cv::Mat leftGray8;
  cv::Mat rightGray8;
  const auto prepareStartTp = totalStartTp;
  if (!PrepareGrayImage(leftGray, leftGray8, err) ||
      !PrepareGrayImage(rightGray, rightGray8, err)) {
    return false;
  }
  const auto prepareEndTp = std::chrono::steady_clock::now();
  std::vector<SuperPointFeatureSet> rawOutputs;
  const auto inferStartTp = prepareEndTp;
  double inputMs = 0.0;
  double forwardMs = 0.0;
  double postMs = 0.0;
  const int requestedMaxPoints = std::max(1, m_maxPoints);
  const int lightGluePoints = m_impl->maxPointsForLightGlue();
  const int descriptorCandidates =
      EnvIntClamped("SMART_DRONE_DESCRIPTOR_SUPPLEMENT_CANDIDATES",
                    requestedMaxPoints, 1, 4096);
  const int requiredFeatureBudget =
      std::max({requestedMaxPoints, lightGluePoints, descriptorCandidates}) +
      kSuperPointStereoExtractionSlack;
  const int extractionBudget = EnvIntClamped(
      "SMART_DRONE_SUPERPOINT_STEREO_EXTRACTION_BUDGET",
      std::min(std::max(m_topK, requestedMaxPoints), requiredFeatureBudget),
      requestedMaxPoints, 4096);
  const int descriptorLimit = EnvIntClamped(
      "SMART_DRONE_SUPERPOINT_DESCRIPTOR_LIMIT",
      std::max({requestedMaxPoints, lightGluePoints, descriptorCandidates}), 1,
      extractionBudget);
  if (!m_impl->DetectAndComputeBatch({leftGray8, rightGray8}, extractionBudget,
                                     descriptorLimit, rawOutputs, &inputMs,
                                     &forwardMs, &postMs, err) ||
      rawOutputs.size() != 2) {
    return false;
  }
  double lightGlueMatchMs = 0.0;
  bool usedLightGlue = false;
  bool usedDescriptorPrimary = false;
  bool usedDescriptorSupplement = false;
  bool skippedLightGlue = false;
  SuperPointFeatureSet lightGlueLeftFeatures;
  SuperPointFeatureSet lightGlueRightFeatures;
  SuperPointFeatureSet descriptorLeftFeatures;
  SuperPointFeatureSet descriptorRightFeatures;
  int lightGlueLeftCount = 0;
  int lightGlueRightCount = 0;
  int descriptorLeftCount = 0;
  int descriptorRightCount = 0;
  double descriptorMatchMs = 0.0;
  int descriptorMatchCalls = 0;
  const int maxStereoPairs = std::max(1, m_maxPoints);
  const int lightGlueEveryN =
      m_lightGlueEveryNOverride > 0
          ? std::clamp(m_lightGlueEveryNOverride, 1, 120)
          : EnvIntClamped("SMART_DRONE_LIGHTGLUE_EVERY_N", 4, 1, 120);
  const int lightGlueFrameIndex = m_impl->lightGlueFrameCounter++;
  const bool lightGlueCadenceSkip =
      lightGlueEveryN > 1 && (lightGlueFrameIndex % lightGlueEveryN) != 0;
  const char *lightGlueSkipReason = "none";
  const int lightGlueSupplementMinPairs =
      EnvIntClamped("SMART_DRONE_LIGHTGLUE_SUPPLEMENT_MIN_PAIRS",
                    kLightGlueMinStereoPairsForSupplement, 0, maxStereoPairs);
  auto buildDescriptorMatches = [&]() {
    if (!descriptorLeftFeatures.keypoints.empty() ||
        !descriptorRightFeatures.keypoints.empty()) {
      return;
    }
    const auto descriptorMatchStartTp = std::chrono::steady_clock::now();
    MatchStereoPairs(rawOutputs[0], rawOutputs[1], maxStereoPairs,
                     descriptorLeftFeatures, descriptorRightFeatures);
    const auto descriptorMatchEndTp = std::chrono::steady_clock::now();
    descriptorMatchMs +=
        DurationMs(descriptorMatchStartTp, descriptorMatchEndTp);
    ++descriptorMatchCalls;
    descriptorLeftCount =
        static_cast<int>(descriptorLeftFeatures.keypoints.size());
    descriptorRightCount =
        static_cast<int>(descriptorRightFeatures.keypoints.size());
  };
  m_impl->lastLightGlueMutualCount = 0;
  m_impl->lastLightGlueScorePassCount = 0;
  m_impl->lastLightGlueGeometryPassCount = 0;
  m_impl->lastLightGlueAcceptedCount = 0;
  m_impl->lastLightGlueMinScore = 0.0f;
  m_impl->lastLightGlueMaxScore = 0.0f;
  m_impl->lastLightGlueDecodeMs = 0.0;
  m_impl->lastLightGlueOrientation = "none";
  m_impl->lastLightGlueScoresLookLog = false;
  m_impl->lastLightGlueForwardStats = TensorRtForwardStats{};
  m_impl->lastLightGlueRequestedPointCount = 0;
  m_impl->lastLightGlueInputPointCount = 0;
  m_impl->lastLightGlueStaticShapeFallback = false;
  if (lightGlueCadenceSkip) {
    skippedLightGlue = true;
    lightGlueSkipReason = "cadence";
  } else if (m_impl->lightGlueSkipRemaining > 0) {
    --m_impl->lightGlueSkipRemaining;
    skippedLightGlue = true;
    lightGlueSkipReason = "cooldown";
  }
  if (!skippedLightGlue &&
      m_impl->MatchWithLightGlue(rawOutputs[0], rawOutputs[1], maxStereoPairs,
                                 leftGray8.cols, leftGray8.rows,
                                 lightGlueLeftFeatures, lightGlueRightFeatures,
                                 &lightGlueMatchMs, err)) {
    usedLightGlue = true;
    if (!lightGlueLeftFeatures.keypoints.empty() &&
        !lightGlueRightFeatures.keypoints.empty()) {
      lightGlueLeftCount =
          static_cast<int>(lightGlueLeftFeatures.keypoints.size());
      lightGlueRightCount =
          static_cast<int>(lightGlueRightFeatures.keypoints.size());
      leftFeatures = std::move(lightGlueLeftFeatures);
      rightFeatures = std::move(lightGlueRightFeatures);
      m_impl->lightGlueEmptyCount = 0;
      if (lightGlueLeftCount < m_impl->lightGlueLowYieldMinPairs) {
        ++m_impl->lightGlueLowYieldCount;
        if (m_impl->lightGlueLowYieldCount >=
            m_impl->lightGlueLowYieldDisableThreshold) {
          m_impl->lightGlueSkipRemaining = m_impl->lightGlueEmptyCooldownFrames;
          m_impl->lightGlueLowYieldCount = 0;
          std::cerr << "[lightglue_trt] low_yield_cooldown pairs="
                    << lightGlueLeftCount
                    << " frames=" << m_impl->lightGlueSkipRemaining << "\n";
        }
      } else {
        m_impl->lightGlueLowYieldCount = 0;
      }
      if (lightGlueSupplementMinPairs > 0 &&
          static_cast<int>(std::min(leftFeatures.keypoints.size(),
                                    rightFeatures.keypoints.size())) <
              lightGlueSupplementMinPairs) {
        const size_t beforeCount = std::min(leftFeatures.keypoints.size(),
                                            rightFeatures.keypoints.size());
        buildDescriptorMatches();
        AppendStereoFeaturePairs(leftFeatures, rightFeatures,
                                 descriptorLeftFeatures,
                                 descriptorRightFeatures, maxStereoPairs);
        usedDescriptorSupplement =
            std::min(leftFeatures.keypoints.size(),
                     rightFeatures.keypoints.size()) > beforeCount;
      }
    } else {
      ++m_impl->lightGlueEmptyCount;
      ++m_impl->lightGlueLowYieldCount;
      if (m_impl->lightGlueEmptyCount >=
          m_impl->lightGlueEmptyDisableThreshold) {
        m_impl->lightGlueSkipRemaining = m_impl->lightGlueEmptyCooldownFrames;
        m_impl->lightGlueEmptyCount = 0;
        m_impl->lightGlueLowYieldCount = 0;
        std::cerr << "[lightglue_trt] empty_output_cooldown frames="
                  << m_impl->lightGlueSkipRemaining << "\n";
      }
      buildDescriptorMatches();
      leftFeatures = std::move(descriptorLeftFeatures);
      rightFeatures = std::move(descriptorRightFeatures);
      usedDescriptorPrimary = true;
    }
  } else {
    buildDescriptorMatches();
    leftFeatures = std::move(descriptorLeftFeatures);
    rightFeatures = std::move(descriptorRightFeatures);
    usedDescriptorPrimary = true;
  }
  const auto inferEndTp = std::chrono::steady_clock::now();
  m_lastStats.prepareMs = DurationMs(prepareStartTp, prepareEndTp);
  m_lastStats.inputMs = inputMs;
  m_lastStats.forwardMs = forwardMs;
  m_lastStats.postMs = postMs;
  m_lastStats.inferMs = DurationMs(inferStartTp, inferEndTp);
  m_lastStats.totalMs = DurationMs(totalStartTp, inferEndTp);
  m_lastStats.rawLeftCount = static_cast<int>(rawOutputs[0].keypoints.size());
  m_lastStats.rawRightCount = static_cast<int>(rawOutputs[1].keypoints.size());
  m_lastStats.stereoLeftCount = static_cast<int>(leftFeatures.keypoints.size());
  m_lastStats.stereoRightCount =
      static_cast<int>(rightFeatures.keypoints.size());
  m_lastStats.lightGlueUsed = usedLightGlue;
  m_lastStats.descriptorFallbackUsed =
      usedDescriptorPrimary || usedDescriptorSupplement;
  m_lastStats.imageCount = 2;
  m_lastStats.payloadBytes =
      static_cast<uint32_t>(leftGray8.total() + rightGray8.total());
  const TensorRtForwardStats &spStats = m_impl->lastSuperPointForwardStats;
  const TensorRtForwardStats &lgStats = m_impl->lastLightGlueForwardStats;
  const SuperPointPostStats &spPostStats = m_impl->lastSuperPointPostStats;
  std::cerr << "[superpoint_trt_perf] batch=2 input_ms=" << inputMs
            << " gpu_forward_ms=" << forwardMs << " cpu_post_ms=" << postMs
            << " sp_heatmap_ms=" << spPostStats.heatmapMs
            << " sp_nms_ms=" << spPostStats.nmsMs
            << " sp_scan_ms=" << spPostStats.scanMs
            << " sp_sort_ms=" << spPostStats.sortMs
            << " sp_desc_sample_ms=" << spPostStats.descriptorMs
            << " sp_candidates=" << spPostStats.candidateCount
            << " sp_selected=" << spPostStats.selectedCount
            << " sp_descriptor_rows=" << spPostStats.descriptorCount
            << " sp_h2d_ms=" << spStats.h2dMs
            << " sp_enqueue_ms=" << spStats.enqueueMs
            << " sp_d2h_ms=" << spStats.outputMs
            << " sp_convert_ms=" << spStats.outputConvertMs
            << " sp_sync_ms=" << spStats.syncMs
            << " sp_gpu_compute_ms=" << spStats.gpuComputeMs
            << " sp_gpu_d2h_ms=" << spStats.gpuOutputMs
            << " sp_event_timing=" << (spStats.eventTimingEnabled ? "Y" : "N")
            << " sp_pinned_host=" << (spStats.pinnedHostOutput ? "Y" : "N")
            << " sp_h2d_bytes=" << spStats.h2dBytes
            << " sp_d2h_bytes=" << spStats.d2hBytes << " sp_batched="
            << (m_impl->lastSuperPointBatchedForward ? "Y" : "N")
            << " lightglue=" << (usedLightGlue ? "Y" : "N")
            << " skipped_lightglue=" << (skippedLightGlue ? "Y" : "N")
            << " lg_every_n=" << lightGlueEveryN
            << " lg_frame_index=" << lightGlueFrameIndex
            << " lg_skip_reason=" << lightGlueSkipReason
            << " lg_h2d_ms=" << lgStats.h2dMs
            << " lg_enqueue_ms=" << lgStats.enqueueMs
            << " lg_d2h_ms=" << lgStats.outputMs
            << " lg_convert_ms=" << lgStats.outputConvertMs
            << " lg_sync_ms=" << lgStats.syncMs
            << " lg_gpu_compute_ms=" << lgStats.gpuComputeMs
            << " lg_gpu_d2h_ms=" << lgStats.gpuOutputMs
            << " lg_event_timing=" << (lgStats.eventTimingEnabled ? "Y" : "N")
            << " lg_pinned_host=" << (lgStats.pinnedHostOutput ? "Y" : "N")
            << " lg_h2d_bytes=" << lgStats.h2dBytes
            << " lg_d2h_bytes=" << lgStats.d2hBytes
            << " lg_requested_pts=" << m_impl->lastLightGlueRequestedPointCount
            << " lg_input_pts=" << m_impl->lastLightGlueInputPointCount
            << " lg_static_shape_fallback="
            << (m_impl->lastLightGlueStaticShapeFallback ? "Y" : "N")
            << " sp_descriptor="
            << (usedDescriptorPrimary
                    ? "primary"
                    : (usedDescriptorSupplement ? "supplement" : "none"))
            << " descriptor_match_ms=" << descriptorMatchMs
            << " descriptor_match_calls=" << descriptorMatchCalls
            << " descriptor_candidates=" << descriptorCandidates
            << " extraction_budget=" << extractionBudget
            << " descriptor_limit=" << descriptorLimit
            << " lg_score_range=" << m_impl->lastLightGlueMinScore << "/"
            << m_impl->lastLightGlueMaxScore << " lg_score_space="
            << (m_impl->lastLightGlueScoresLookLog ? "log" : "prob")
            << " lg_decode_ms=" << m_impl->lastLightGlueDecodeMs
            << " lg_orientation=" << m_impl->lastLightGlueOrientation
            << " lg_filter=" << m_impl->lastLightGlueMutualCount << "/"
            << m_impl->lastLightGlueScorePassCount << "/"
            << m_impl->lastLightGlueGeometryPassCount << "/"
            << m_impl->lastLightGlueAcceptedCount
            << " supplement_min=" << lightGlueSupplementMinPairs
            << " lightglue_ms=" << lightGlueMatchMs
            << " total_ms=" << m_lastStats.totalMs
            << " raw_pts=" << rawOutputs[0].keypoints.size() << "/"
            << rawOutputs[1].keypoints.size()
            << " lg_pts=" << lightGlueLeftCount << "/" << lightGlueRightCount
            << " desc_pts=" << descriptorLeftCount << "/"
            << descriptorRightCount
            << " stereo_pts=" << leftFeatures.keypoints.size() << "/"
            << rightFeatures.keypoints.size() << "\n";
  return true;
#else
  if (err != nullptr) {
    *err = "native TensorRT SuperPoint backend is compiled out";
  }
  (void)leftGray;
  (void)rightGray;
  return false;
#endif
}

} // namespace smartdrone::adapters::slam
