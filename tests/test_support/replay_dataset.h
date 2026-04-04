#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "core/ports/camera_provider.h"
#include "core/ports/imu_provider.h"

namespace smartdrone::tests {

struct ReplayImageSample {
    uint64_t timestampNs{0};
    std::filesystem::path path;
};

struct ReplayImuSample {
    int64_t timestampNs{0};
    float gx{0.0f};
    float gy{0.0f};
    float gz{0.0f};
    float ax{0.0f};
    float ay{0.0f};
    float az{0.0f};
};

class ReplayDataset {
  public:
    static ReplayDataset Load(const std::filesystem::path &rootDir, size_t maxFrames = 0);

    const std::vector<ReplayImageSample> &LeftFrames() const { return m_leftFrames; }
    const std::vector<ReplayImageSample> &RightFrames() const { return m_rightFrames; }
    const std::vector<ReplayImuSample> &ImuSamples() const { return m_imuSamples; }
    bool Empty() const { return m_leftFrames.empty() || m_rightFrames.empty() || m_imuSamples.empty(); }

  private:
    std::vector<ReplayImageSample> m_leftFrames;
    std::vector<ReplayImageSample> m_rightFrames;
    std::vector<ReplayImuSample> m_imuSamples;
};

class ReplayCameraProvider final : public smartdrone::core::ports::ICameraProvider {
  public:
    explicit ReplayCameraProvider(const ReplayDataset &dataset);

    bool Start() override;
    void Stop() override;
    bool GrabStereo(smartdrone::core::ports::StereoFrame &out, int timeoutMs, bool preferLatest,
                    uint64_t minTimestampNs) override;
    smartdrone::core::ports::CameraHealth GetHealth() const override;

  private:
    const ReplayDataset &m_dataset;
    size_t m_nextIndex{0};
    bool m_started{false};
};

class ReplayImuProvider final : public smartdrone::core::ports::IImuProvider {
  public:
    explicit ReplayImuProvider(const ReplayDataset &dataset);

    bool Start() override;
    void Stop() override;
    bool Ready() const override;
    std::vector<smartdrone::core::ports::ImuReading> PopWindow(int64_t fromNs, int64_t toNs) override;

  private:
    const ReplayDataset &m_dataset;
    bool m_started{false};
    size_t m_cursor{0};
};

} // namespace smartdrone::tests
