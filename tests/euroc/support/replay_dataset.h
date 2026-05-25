#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "core/ports/camera_provider.h"
#include "core/ports/imu_provider.h"

namespace SmartDrone::Tests {

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

    const std::vector<ReplayImageSample> &LeftFrames() const
    {
        return m_leftFrames;
    }
    const std::vector<ReplayImageSample> &RightFrames() const
    {
        return m_rightFrames;
    }
    const std::vector<ReplayImuSample> &ImuSamples() const
    {
        return m_imuSamples;
    }
    bool Empty() const
    {
        return m_leftFrames.empty() || m_rightFrames.empty();
    }

  private:
    std::vector<ReplayImageSample> m_leftFrames;
    std::vector<ReplayImageSample> m_rightFrames;
    std::vector<ReplayImuSample> m_imuSamples;
};

struct ReplayCameraProgress {
    std::atomic<size_t> framesConsumed{0};
    std::atomic<bool> finished{false};
};

class ReplayCameraProvider final : public SmartDrone::Core::Ports::ICameraProvider {
  public:
    explicit ReplayCameraProvider(
        const ReplayDataset &dataset,
        std::shared_ptr<ReplayCameraProgress> progress = {});

    bool Open(const SmartDrone::Core::Ports::CameraOpenConfig &config) override;
    void Close() override;
    bool Start() override;
    void Stop() override;
    bool GrabStereo(SmartDrone::Core::Ports::StereoFrame &out, bool preferLatest,
                    uint64_t minTimestampNs) override;
    SmartDrone::Core::Ports::CameraHealth GetHealth() const override;
    SmartDrone::Core::Ports::CameraDiagnostics GetDiagnostics() const override;
    SmartDrone::Core::Ports::CameraProviderSemantics Semantics() const override;
    bool Finished() const;
    size_t FramesConsumed() const;

  private:
    const ReplayDataset &m_dataset;
    std::shared_ptr<ReplayCameraProgress> m_progress;
    std::atomic<size_t> m_nextIndex{0};
    bool m_started{false};
};

class ReplayImuProvider final : public SmartDrone::Core::Ports::IImuProvider {
  public:
    explicit ReplayImuProvider(const ReplayDataset &dataset);

    bool Start() override;
    void Stop() override;
    bool Ready() const override;
    std::vector<SmartDrone::Core::Ports::ImuReading> PopWindow(int64_t fromNs, int64_t toNs) override;

  private:
    const ReplayDataset &m_dataset;
    bool m_started{false};
    size_t m_cursor{0};
};

} // namespace SmartDrone::Tests
