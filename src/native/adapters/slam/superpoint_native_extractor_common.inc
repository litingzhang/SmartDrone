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
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    const std::string text = LowerCopy(value);
    return !(text == "0" || text == "false" || text == "off" || text == "no");
}

int EnvIntClamped(const char *name, int fallback, int minValue, int maxValue)
{
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

float StereoMinDisparityPx()
{
    return std::clamp(EnvFloatValue("SMART_DRONE_STEREO_FEATURE_MIN_DISPARITY_PX",
                                    kStereoMinDisparityPx),
                      0.05f, kStereoMaxDisparityPx);
}

std::filesystem::path ResolveSuperPointEnginePath(const std::string &repoPath,
                                                  int widthHint,
                                                  int heightHint)
{
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
                                                 int maxPoints)
{
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

    int Dim(size_t index) const
    {
        return index < dims.size() ? dims[index] : 0;
    }
    bool Empty() const
    {
        return dims.empty() || FloatData() == nullptr || FloatElementCount() == 0;
    }
    const float *FloatData() const
    {
        return floatData != nullptr ? floatData : data.data();
    }
    size_t FloatElementCount() const
    {
        return floatData != nullptr ? floatElementCount : data.size();
    }

    void ResetHostData()
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
};

struct CudaPinnedHostBuffer {
    void *ptr{nullptr};
    size_t bytes{0};

    ~CudaPinnedHostBuffer()
    {
        Release();
    }

    CudaPinnedHostBuffer() = default;
    CudaPinnedHostBuffer(const CudaPinnedHostBuffer &) = delete;
    CudaPinnedHostBuffer &operator=(const CudaPinnedHostBuffer &) = delete;
    CudaPinnedHostBuffer(CudaPinnedHostBuffer &&other) noexcept
        : ptr(other.ptr), bytes(other.bytes)
    {
        other.ptr = nullptr;
        other.bytes = 0;
    }
    CudaPinnedHostBuffer &operator=(CudaPinnedHostBuffer &&other) noexcept
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

    void Release()
    {
        if (ptr != nullptr) {
            cudaFreeHost(ptr);
            ptr = nullptr;
            bytes = 0;
        }
    }

    bool Ensure(size_t requiredBytes)
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
};
