#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

#include "core/application/session/runtime_session_common.h"
#include "core/ports/camera_provider.h"

namespace smartdrone::adapters::camera {

class UvcStereoCamera final : public core::ports::ICameraProvider {
  public:
    UvcStereoCamera() = default;
    ~UvcStereoCamera() override;

    bool Open(const core::application::MainRuntimeAliases &aliases) override;
    void Close() override;
    bool Start() override;
    void Stop() override;
    bool GrabStereo(core::ports::StereoFrame &out, int timeoutMs, bool preferLatest,
                    uint64_t minTimestampNs = 0) override;
    core::ports::CameraHealth GetHealth() const override;
    core::ports::CameraDiagnostics GetDiagnostics() const override;
    core::ports::CameraProviderSemantics Semantics() const override;

  private:
    struct StereoFrameItem {
        core::ports::StereoFrame frame;
        uint64_t captureTimestampNs{0};
    };

    void CaptureLoop();
    bool OpenDevice(int deviceIndex, int width, int height, int fps, bool aeDisable, int exposureUs, float gain);
    void PushFrame(core::ports::StereoFrame &&frame, uint64_t captureTimestampNs);

    mutable std::mutex m_mutex;
    std::condition_variable m_cv;
    cv::VideoCapture m_cap;
    std::deque<StereoFrameItem> m_queue;
    std::thread m_thread;
    bool m_open{false};
    bool m_running{false};
    int m_deviceIndex{0};
    int m_width{0};
    int m_height{0};
    int m_fps{0};
    size_t m_maxQueue{1};
    uint32_t m_sequence{0};
    uint64_t m_lastFrameTimestampNs{0};
    uint64_t m_lastPairTimestampNs{0};
    core::ports::CameraDiagnostics m_diag{};
};

} // namespace smartdrone::adapters::camera
