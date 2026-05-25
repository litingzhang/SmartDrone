#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal {

constexpr int SUPER_POINT_CELL_SIZE = 8;
constexpr int SUPER_POINT_DESCRIPTOR_DIM = 256;
constexpr float SUPER_POINT_THRESHOLD = 0.0005f;
constexpr int SUPER_POINT_BORDER = 4;
constexpr int SUPER_POINT_NMS_RADIUS = 4;
constexpr float STEREO_MAX_Y_DIFF_PX = 2.0f;
constexpr float STEREO_MIN_DISPARITY_PX = 0.75f;
constexpr float STEREO_MAX_DISPARITY_PX = 240.0f;
constexpr float STEREO_RATIO = 0.92f;
constexpr float STEREO_MERGE_DISTANCE_PX = 3.0f;
constexpr int LIGHT_GLUE_MIN_STEREO_PAIRS_FOR_SUPPLEMENT = 96;
constexpr int SUPER_POINT_STEREO_EXTRACTION_SLACK = 96;

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

    int Dim(size_t index) const;
    bool Empty() const;
    const float *FloatData() const;
    size_t FloatElementCount() const;
    void ResetHostData();
};

struct CudaPinnedHostBuffer {
    void *ptr{nullptr};
    size_t bytes{0};

    ~CudaPinnedHostBuffer();
    CudaPinnedHostBuffer() = default;
    CudaPinnedHostBuffer(const CudaPinnedHostBuffer &) = delete;
    CudaPinnedHostBuffer &operator=(const CudaPinnedHostBuffer &) = delete;
    CudaPinnedHostBuffer(CudaPinnedHostBuffer &&other) noexcept;
    CudaPinnedHostBuffer &operator=(CudaPinnedHostBuffer &&other) noexcept;

    void Release();
    bool Ensure(size_t requiredBytes);
};

double DurationMs(const std::chrono::steady_clock::time_point &start,
                  const std::chrono::steady_clock::time_point &end);
std::string LowerCopy(std::string text);
bool EnvFlag(const char *name, bool fallback);
int EnvIntClamped(const char *name, int fallback, int minValue, int maxValue);
float StereoMinDisparityPx();
float HalfToFloat(uint16_t value);
std::filesystem::path ResolveSuperPointEnginePath(const std::string &repoPath,
                                                  int widthHint,
                                                  int heightHint);
std::filesystem::path ResolveLightGlueEnginePath(const std::string &repoPath,
                                                 int maxPoints);

} // namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal
