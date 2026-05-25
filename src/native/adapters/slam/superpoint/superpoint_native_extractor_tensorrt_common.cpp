#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_common.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include <cuda_runtime_api.h>

#include "adapters/slam/engine/slam_env.h"
#include "common/environment.h"

namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal {

double DurationMs(const std::chrono::steady_clock::time_point &start,
                  const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

std::string LowerCopy(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return text;
}

bool EnvFlag(const char *name, bool fallback)
{
    return SmartDrone::Common::EnvFlagEnabled(name, fallback);
}

int EnvIntClamped(const char *name, int fallback, int minValue, int maxValue)
{
    return SmartDrone::Common::EnvIntValueClamped(name, fallback, minValue,
                                                  maxValue);
}

namespace {

std::filesystem::path ExistingEnvPath(const char *name)
{
    const std::filesystem::path path(
        SmartDrone::Common::EnvStringValue(name, ""));
    return !path.empty() && std::filesystem::exists(path) ? path
                                                          : std::filesystem::path{};
}

} // namespace

float StereoMinDisparityPx()
{
    return std::clamp(
        EnvFloatValue("SMART_DRONE_STEREO_FEATURE_MIN_DISPARITY_PX",
                      STEREO_MIN_DISPARITY_PX),
        0.05f, STEREO_MAX_DISPARITY_PX);
}

float HalfToFloat(uint16_t value)
{
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

std::filesystem::path ResolveSuperPointEnginePath(const std::string &repoPath,
                                                  int widthHint,
                                                  int heightHint)
{
    std::filesystem::path envEngine =
        ExistingEnvPath("SMART_DRONE_SUPERPOINT_TRT_ENGINE");
    if (envEngine.empty()) {
        envEngine = ExistingEnvPath("SUPERPOINT_TRT_ENGINE");
    }
    if (!envEngine.empty()) {
        return envEngine;
    }

    const std::filesystem::path repo(repoPath);
    if (!repo.empty()) {
        std::vector<std::string> names;
        if (widthHint > 0 && heightHint > 0) {
            names.push_back("superpoint_dense_" + std::to_string(widthHint) +
                            "x" + std::to_string(heightHint) +
                            "_fp16.engine");
            names.push_back("superpoint_dense_" + std::to_string(widthHint) +
                            "x" + std::to_string(heightHint) +
                            "_fp32.engine");
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
                                                 int maxPoints)
{
    const std::filesystem::path envEngine =
        ExistingEnvPath("SMART_DRONE_LIGHTGLUE_TRT_ENGINE");
    if (!envEngine.empty()) {
        return envEngine;
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

int TensorBlob::Dim(size_t index) const
{
    return index < dims.size() ? dims[index] : 0;
}

bool TensorBlob::Empty() const
{
    return dims.empty() || FloatData() == nullptr || FloatElementCount() == 0;
}

const float *TensorBlob::FloatData() const
{
    return floatData != nullptr ? floatData : data.data();
}

size_t TensorBlob::FloatElementCount() const
{
    return floatData != nullptr ? floatElementCount : data.size();
}

void TensorBlob::ResetHostData()
{
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

CudaPinnedHostBuffer::~CudaPinnedHostBuffer()
{
    Release();
}

CudaPinnedHostBuffer::CudaPinnedHostBuffer(
    CudaPinnedHostBuffer &&other) noexcept
    : ptr(other.ptr), bytes(other.bytes)
{
    other.ptr = nullptr;
    other.bytes = 0;
}

CudaPinnedHostBuffer &CudaPinnedHostBuffer::operator=(
    CudaPinnedHostBuffer &&other) noexcept
{
    if (this != &other) {
        Release();
        ptr = other.ptr;
        bytes = other.bytes;
        other.ptr = nullptr;
        other.bytes = 0;
    }
    return *this;
}

void CudaPinnedHostBuffer::Release()
{
    if (ptr != nullptr) {
        cudaFreeHost(ptr);
        ptr = nullptr;
        bytes = 0;
    }
}

bool CudaPinnedHostBuffer::Ensure(size_t requiredBytes)
{
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

} // namespace SmartDrone::Adapters::Slam::SuperPointTensorRtInternal
