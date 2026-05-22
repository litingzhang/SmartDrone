#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "core/ports/camera_provider.h"

struct v4l2_buffer;
struct v4l2_capability;
struct v4l2_format;

namespace SmartDrone::Adapters::Camera {

class UvcStereoCamera final : public Core::Ports::ICameraProvider {
  public:
    UvcStereoCamera() = default;
    ~UvcStereoCamera() override;

    bool Open(const Core::Ports::CameraOpenConfig &config) override;
    void Close() override;
    bool Start() override;
    void Stop() override;
    bool GrabStereo(Core::Ports::StereoFrame &out, int timeoutMs, bool preferLatest,
                    uint64_t minTimestampNs = 0) override;
    Core::Ports::CameraHealth GetHealth() const override;
    Core::Ports::CameraDiagnostics GetDiagnostics() const override;
    Core::Ports::CameraProviderSemantics Semantics() const override;

  private:
    struct StereoFrameItem {
        Core::Ports::StereoFrame frame;
        uint64_t captureTimestampNs{0};
    };

    struct MappedBuffer {
        void *start{nullptr};
        size_t length{0};
    };

    struct CapturedPackedFrame {
        cv::Mat packed;
        uint64_t captureTimestampNs{0};
        uint64_t arriveNs{0};
    };

    struct DeviceOpenParams {
        int deviceIndex{0};
        int width{0};
        int height{0};
        int fps{0};
        bool aeDisable{false};
        int exposureUs{0};
        float gain{0.0F};
    };

    enum class CaptureStatus {
        Frame,
        NoFrame,
        Timeout,
        Stopped,
        Fatal,
    };

    bool ApplyOpenConfig(const Core::Ports::CameraOpenConfig &config);
    void ResetOpenState(const Core::Ports::CameraOpenConfig &config);
    void MarkOpenHealthy();
    CaptureStatus CaptureOnce(int timeoutMs);
    CaptureStatus PollCaptureReady(int fd, int timeoutMs);
    CaptureStatus DequeueCaptureBuffer(int fd, v4l2_buffer &buffer);
    CaptureStatus DecodeCaptureBuffer(int fd, v4l2_buffer &buffer, CapturedPackedFrame &frame);
    bool TryPopOrStop(Core::Ports::StereoFrame &out, bool preferLatest, uint64_t minTimestampNs);
    void DrainReadyFrames();
    bool OpenDevice(const DeviceOpenParams &params);
    bool ValidateDeviceCapabilities(int fd, const std::string &devicePath, v4l2_capability &caps);
    bool ApplyDeviceFormat(int fd, const std::string &devicePath, const DeviceOpenParams &params,
                           v4l2_format &format);
    bool ConfigureDeviceFps(int fd, const std::string &devicePath, int fps);
    bool RequestAndMapBuffers(int fd, const std::string &devicePath, uint32_t &bufferCount,
                              std::vector<MappedBuffer> &buffers);
    bool QueueBuffersAndStart(int fd, const std::string &devicePath, uint32_t bufferCount,
                              std::vector<MappedBuffer> &buffers);
    void ReleaseMappedBuffers(std::vector<MappedBuffer> &buffers);
    void StoreOpenedDevice(int fd, const v4l2_format &format, std::vector<MappedBuffer> &&buffers);
    void LogOpenedDevice(const DeviceOpenParams &params, const v4l2_capability &caps, const v4l2_format &format,
                         uint32_t bufferCount);
    void CloseDevice();
    bool PopCandidateLocked(Core::Ports::StereoFrame &out, bool preferLatest, uint64_t minTimestampNs);
    Core::Ports::StereoFrame BuildStereoFrame(const cv::Mat &packed, uint64_t captureTimestampNs, uint64_t arriveNs);
    void PushFrame(Core::Ports::StereoFrame &&frame, uint64_t captureTimestampNs);
    void MarkCaptureFault(bool acceptingFrames);

    mutable std::mutex m_mutex;
    std::mutex m_captureMu;
    std::deque<StereoFrameItem> m_queue;
    bool m_open{false};
    bool m_running{false};
    bool m_streaming{false};
    int m_deviceIndex{0};
    int m_width{0};
    int m_height{0};
    int m_fps{0};
    bool m_swapEyes{false};
    int m_fd{-1};
    uint32_t m_pixelFormat{0};
    uint32_t m_bytesPerLine{0};
    std::vector<MappedBuffer> m_buffers;
    size_t m_maxQueue{1};
    uint32_t m_sequence{0};
    uint64_t m_lastFrameTimestampNs{0};
    uint64_t m_lastPairTimestampNs{0};
    Core::Ports::CameraDiagnostics m_diag{};
};

} // namespace SmartDrone::Adapters::Camera
