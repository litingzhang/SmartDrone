#include "adapters/camera/uvc_stereo_camera.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <opencv2/imgproc.hpp>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>

#include "common/time_utils.h"

namespace SmartDrone::Adapters::Camera {

namespace {

constexpr size_t DEFAULT_BUFFER_COUNT = 4;
constexpr int32_t FALLBACK_EXPOSURE_ABSOLUTE_MAX = 10000;
constexpr int32_t AUTO_EXPOSURE_GAIN_FLOOR = 32;

uint32_t YuyvFourcc()
{
    return v4l2_fourcc('Y', 'U', 'Y', 'V');
}

std::string FourccToString(uint32_t fourcc)
{
    if (fourcc == 0) {
        return "unknown";
    }
    std::string text(4, ' ');
    text[0] = static_cast<char>(fourcc & 0xFFu);
    text[1] = static_cast<char>((fourcc >> 8) & 0xFFu);
    text[2] = static_cast<char>((fourcc >> 16) & 0xFFu);
    text[3] = static_cast<char>((fourcc >> 24) & 0xFFu);
    return text;
}

uint64_t TimevalToNs(const timeval &tv)
{
    if (tv.tv_sec < 0 || tv.tv_usec < 0) {
        return 0;
    }
    const uint64_t secNs = static_cast<uint64_t>(tv.tv_sec) * 1000000000ULL;
    const uint64_t usecNs = static_cast<uint64_t>(tv.tv_usec) * 1000ULL;
    return secNs + usecNs;
}

const char *TimestampFlagToString(uint32_t flags)
{
    switch (flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) {
    case V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC:
        return "monotonic";
    case V4L2_BUF_FLAG_TIMESTAMP_COPY:
        return "copy";
    default:
        return "unknown";
    }
}

struct UvcControlInfo {
    bool supported{false};
    int32_t min{0};
    int32_t max{0};
    int32_t step{1};
    int32_t def{0};
};

bool QueryUvcControl(int fd, uint32_t id, UvcControlInfo &out)
{
    v4l2_queryctrl query{};
    query.id = id;
    if (::ioctl(fd, VIDIOC_QUERYCTRL, &query) != 0) {
        return false;
    }
    if ((query.flags & V4L2_CTRL_FLAG_DISABLED) != 0) {
        return false;
    }
    out.supported = true;
    out.min = query.minimum;
    out.max = query.maximum;
    out.step = std::max<int32_t>(1, query.step);
    out.def = query.default_value;
    if (out.max < out.min) {
        if (id == V4L2_CID_EXPOSURE_ABSOLUTE) {
            out.max = std::max<int32_t>(out.min, FALLBACK_EXPOSURE_ABSOLUTE_MAX);
            std::cerr << "[uvc] warning: exposure_absolute reports invalid range min=" << query.minimum
                      << " max=" << query.maximum << "; using fallback max=" << out.max << "\n";
        } else {
            out.max = out.min;
        }
    }
    return true;
}

bool SetUvcControl(int fd, uint32_t id, int32_t value, const char *name)
{
    v4l2_control ctrl{};
    ctrl.id = id;
    ctrl.value = value;
    if (::ioctl(fd, VIDIOC_S_CTRL, &ctrl) != 0) {
        std::cerr << "[uvc] warning: failed to set " << name << " value=" << value << " errno=" << errno << "\n";
        return false;
    }
    return true;
}

void ConfigureUvcControls(int fd, int deviceIndex, bool aeDisable, int exposureUs, float gain)
{
    UvcControlInfo exposureAuto{};
    UvcControlInfo exposureAbs{};
    UvcControlInfo gainInfo{};
    const bool haveExposureAuto = QueryUvcControl(fd, V4L2_CID_EXPOSURE_AUTO, exposureAuto);
    const bool haveExposureAbs = QueryUvcControl(fd, V4L2_CID_EXPOSURE_ABSOLUTE, exposureAbs);
    const bool haveGain = QueryUvcControl(fd, V4L2_CID_GAIN, gainInfo);

    const int32_t autoMode = aeDisable ? static_cast<int32_t>(V4L2_EXPOSURE_MANUAL)
                                       : static_cast<int32_t>(V4L2_EXPOSURE_APERTURE_PRIORITY);
    if (haveExposureAuto) {
        SetUvcControl(fd, V4L2_CID_EXPOSURE_AUTO, autoMode, "exposure_auto");
    }

    if (aeDisable && haveExposureAbs) {
        const int requestedExposure100Us = std::max(1, static_cast<int>(std::lround(exposureUs / 100.0)));
        const int32_t clampedExposure = std::clamp<int32_t>(requestedExposure100Us, exposureAbs.min, exposureAbs.max);
        SetUvcControl(fd, V4L2_CID_EXPOSURE_ABSOLUTE, clampedExposure, "exposure_absolute");
        std::cerr << "[uvc] control device=" << deviceIndex << " exposure mode=manual exposure_us=" << exposureUs
                  << " exposure_absolute=" << clampedExposure << "\n";
    } else if (!aeDisable) {
        std::cerr << "[uvc] control device=" << deviceIndex << " exposure mode=auto\n";
    }

    if (haveGain) {
        const int requestedGain = std::max(0, static_cast<int>(std::lround(gain)));
        const int effectiveGain = aeDisable ? requestedGain : std::max(requestedGain, AUTO_EXPOSURE_GAIN_FLOOR);
        const int32_t clampedGain = std::clamp<int32_t>(effectiveGain, gainInfo.min, gainInfo.max);
        SetUvcControl(fd, V4L2_CID_GAIN, clampedGain, "gain");
        std::cerr << "[uvc] control device=" << deviceIndex << " gain=" << clampedGain << "\n";
    }
}

bool IoctlRetry(int fd, unsigned long request, void *arg)
{
    while (true) {
        if (::ioctl(fd, request, arg) == 0) {
            return true;
        }
        if (errno != EINTR) {
            return false;
        }
    }
}

enum class DecodePackedStatus {
    Frame,
    NoFrame,
    Fatal,
};

struct PackedDecodeInput {
    int width{0};
    int height{0};
    uint32_t bytesPerLine{0};
    void *start{nullptr};
    size_t usedBytes{0};
};

bool HasFrameBytes(size_t usedBytes, size_t rowBytes, int height)
{
    return usedBytes >= rowBytes * static_cast<size_t>(std::max(0, height));
}

DecodePackedStatus DecodeYuyvPacked(const PackedDecodeInput &input, cv::Mat &packed)
{
    const size_t rowBytes = static_cast<size_t>(input.width) * 2U;
    if (!HasFrameBytes(input.usedBytes, rowBytes, input.height)) {
        return DecodePackedStatus::NoFrame;
    }
    const size_t step = std::max<size_t>(rowBytes, static_cast<size_t>(input.bytesPerLine));
    cv::Mat packedYuyv(input.height, input.width, CV_8UC2, input.start, step);
    cv::cvtColor(packedYuyv, packed, cv::COLOR_YUV2GRAY_YUY2);
    return DecodePackedStatus::Frame;
}

DecodePackedStatus DecodeGrayPacked(const PackedDecodeInput &input, cv::Mat &packed)
{
    const size_t rowBytes = static_cast<size_t>(input.width);
    if (!HasFrameBytes(input.usedBytes, rowBytes, input.height)) {
        return DecodePackedStatus::NoFrame;
    }
    const size_t step = std::max<size_t>(rowBytes, static_cast<size_t>(input.bytesPerLine));
    cv::Mat packedGray(input.height, input.width, CV_8UC1, input.start, step);
    packed = packedGray.clone();
    return DecodePackedStatus::Frame;
}

DecodePackedStatus DecodeColorPacked(const PackedDecodeInput &input, int conversion, cv::Mat &packed)
{
    const size_t rowBytes = static_cast<size_t>(input.width) * 3U;
    if (!HasFrameBytes(input.usedBytes, rowBytes, input.height)) {
        return DecodePackedStatus::NoFrame;
    }
    const size_t step = std::max<size_t>(rowBytes, static_cast<size_t>(input.bytesPerLine));
    cv::Mat packedColor(input.height, input.width, CV_8UC3, input.start, step);
    cv::cvtColor(packedColor, packed, conversion);
    return DecodePackedStatus::Frame;
}

DecodePackedStatus DecodePackedFrame(uint32_t pixelFormat, const PackedDecodeInput &input, cv::Mat &packed)
{
    if (pixelFormat == V4L2_PIX_FMT_YUYV) {
        return DecodeYuyvPacked(input, packed);
    }
    if (pixelFormat == V4L2_PIX_FMT_GREY) {
        return DecodeGrayPacked(input, packed);
    }
    if (pixelFormat == V4L2_PIX_FMT_BGR24) {
        return DecodeColorPacked(input, cv::COLOR_BGR2GRAY, packed);
    }
    if (pixelFormat == V4L2_PIX_FMT_RGB24) {
        return DecodeColorPacked(input, cv::COLOR_RGB2GRAY, packed);
    }

    std::cerr << "[uvc] unsupported pixel format for packed stereo: " << FourccToString(pixelFormat) << "\n";
    return DecodePackedStatus::Fatal;
}

} // namespace

UvcStereoCamera::~UvcStereoCamera()
{
    Close();
}

bool UvcStereoCamera::ApplyOpenConfig(
    const Core::Ports::CameraOpenConfig &config)
{
    if (!config.uvcPackedStereo) {
        std::cerr << "[uvc] unsupported configuration: uvc_stereo_opencv "
                     "requires camera.uvc_packed_stereo=true\n";
        return false;
    }
    m_deviceIndex = config.uvcDeviceIndex;
    m_width = config.uvcEyeWidth;
    m_height = config.uvcEyeHeight;
    m_swapEyes = config.uvcSwapEyes;
    if (m_deviceIndex >= 0 && m_width > 0 && m_height > 0) {
        return true;
    }
    std::cerr << "[uvc] invalid configuration: device_index=" << m_deviceIndex
              << " eye=" << m_width << "x" << m_height << "\n";
    return false;
}

void UvcStereoCamera::ResetOpenState(
    const Core::Ports::CameraOpenConfig &config)
{
    m_fps = config.fps;
    m_maxQueue = 1;
    if (config.pairQueue > 1) {
        std::cerr << "[uvc] forcing packed-stereo frame_queue=1 for "
                     "lowest-latency capture (requested="
                  << config.pairQueue << ")\n";
    }
    m_sequence = 0;
    m_lastFrameTimestampNs = 0;
    m_lastPairTimestampNs = 0;
}

void UvcStereoCamera::MarkOpenHealthy()
{
    m_open.store(true, std::memory_order_release);
    m_running.store(true, std::memory_order_release);
    m_diag = {};
    m_diag.healthy = true;
    m_diag.acceptFrames = true;
    m_diag.pairTolNs = 0;
    PublishDiagnostics();
}

bool UvcStereoCamera::Open(const Core::Ports::CameraOpenConfig &config)
{
    Close();

    if (!ApplyOpenConfig(config)) {
        return false;
    }
    ResetOpenState(config);

    const int packedWidth = (m_width > 0) ? (m_width * 2) : 0;
    const DeviceOpenParams params{
        m_deviceIndex,
        packedWidth,
        m_height,
        m_fps,
        config.autoExposureDisabled,
        config.exposureUs,
        config.gain,
    };
    if (!OpenDevice(params)) {
        CloseDevice();
        return false;
    }

    MarkOpenHealthy();
    return true;
}

void UvcStereoCamera::Close()
{
    m_running.store(false, std::memory_order_release);
    m_open.store(false, std::memory_order_release);
    m_diag.acceptFrames = false;
    PublishDiagnostics();
    CloseDevice();

    m_queue.clear();
    m_lastFrameTimestampNs = 0;
    m_lastPairTimestampNs = 0;
    m_diag.healthy = false;
    m_diag.lastFrameAgeMsL = -1;
    m_diag.lastFrameAgeMsR = -1;
    m_diag.lastPairAgeMs = -1;
    m_diag.pairedQueue = 0;
    PublishDiagnostics();
}

bool UvcStereoCamera::Start()
{
    return m_open.load(std::memory_order_acquire);
}

void UvcStereoCamera::Stop()
{
    Close();
}

bool UvcStereoCamera::GrabStereo(Core::Ports::StereoFrame &out, bool preferLatest, uint64_t minTimestampNs)
{
    if (TryPopOrStop(out, preferLatest, minTimestampNs)) {
        return true;
    }

    const auto status = CaptureOnce();
    if (status == CaptureStatus::Frame && preferLatest) {
        DrainReadyFrames();
    }
    if (status == CaptureStatus::Stopped || status == CaptureStatus::Fatal) {
        return false;
    }
    return TryPopOrStop(out, preferLatest, minTimestampNs);
}

bool UvcStereoCamera::TryPopOrStop(Core::Ports::StereoFrame &out, bool preferLatest, uint64_t minTimestampNs)
{
    if (PopCandidate(out, preferLatest, minTimestampNs)) {
        return true;
    }
    return false;
}

void UvcStereoCamera::DrainReadyFrames()
{
    for (size_t i = 0; i < m_buffers.size(); ++i) {
        if (CaptureOnce() != CaptureStatus::Frame) {
            return;
        }
    }
}

bool UvcStereoCamera::PopCandidate(Core::Ports::StereoFrame &out, bool preferLatest, uint64_t minTimestampNs)
{
    auto candidate = m_queue.end();
    if (preferLatest) {
        for (auto it = m_queue.rbegin(); it != m_queue.rend(); ++it) {
            if (it->captureTimestampNs >= minTimestampNs) {
                candidate = std::prev(it.base());
                break;
            }
        }
    } else {
        for (auto it = m_queue.begin(); it != m_queue.end(); ++it) {
            if (it->captureTimestampNs >= minTimestampNs) {
                candidate = it;
                break;
            }
        }
    }

    if (candidate == m_queue.end()) {
        return false;
    }
    out = std::move(candidate->frame);
    m_queue.erase(m_queue.begin(), std::next(candidate));
    m_diag.pairedQueue = m_queue.size();
    m_diag.lastPairAgeMs = 0;
    PublishDiagnostics();
    return true;
}

UvcStereoCamera::CaptureStatus UvcStereoCamera::CaptureOnce()
{
    if (!m_running.load(std::memory_order_acquire)) {
        return CaptureStatus::Stopped;
    }
    const int fd = m_fd.load(std::memory_order_acquire);
    if (fd < 0) {
        return CaptureStatus::Stopped;
    }

    const CaptureStatus ready = PollCaptureReady(fd);
    if (ready != CaptureStatus::Frame) {
        return ready;
    }

    v4l2_buffer buffer{};
    const CaptureStatus dequeued = DequeueCaptureBuffer(fd, buffer);
    if (dequeued != CaptureStatus::Frame) {
        return dequeued;
    }

    CapturedPackedFrame frame{};
    const CaptureStatus decoded = DecodeCaptureBuffer(fd, buffer, frame);
    if (decoded != CaptureStatus::Frame) {
        return decoded;
    }
    PushFrame(BuildStereoFrame(frame.packed, frame.captureTimestampNs,
                               frame.captureMonotonicNs, frame.arriveNs),
              frame.captureTimestampNs);
    return CaptureStatus::Frame;
}

UvcStereoCamera::CaptureStatus UvcStereoCamera::PollCaptureReady(int fd)
{
    pollfd pfd{};
    pfd.fd = fd;
    pfd.events = POLLIN;
    const int pollRc = ::poll(&pfd, 1, 0);
    if (pollRc < 0) {
        if (errno == EINTR) {
            return CaptureStatus::NoFrame;
        }
        MarkCaptureFault(false);
        return CaptureStatus::Fatal;
    }
    if (pollRc == 0) {
        return CaptureStatus::NoFrame;
    }
    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        MarkCaptureFault(false);
        return CaptureStatus::Fatal;
    }
    if ((pfd.revents & POLLIN) == 0) {
        return CaptureStatus::NoFrame;
    }
    return CaptureStatus::Frame;
}

UvcStereoCamera::CaptureStatus UvcStereoCamera::DequeueCaptureBuffer(int fd, v4l2_buffer &buffer)
{
    buffer = {};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (!IoctlRetry(fd, VIDIOC_DQBUF, &buffer)) {
        const int savedErrno = errno;
        if (savedErrno == EAGAIN) {
            return CaptureStatus::NoFrame;
        }
        if (savedErrno != EPIPE && savedErrno != ENODEV && savedErrno != EBADF) {
            MarkCaptureFault(false);
        }
        return CaptureStatus::Fatal;
    }

    if (buffer.index >= m_buffers.size()) {
        MarkCaptureFault(false);
        return CaptureStatus::Fatal;
    }
    return CaptureStatus::Frame;
}

UvcStereoCamera::CaptureStatus UvcStereoCamera::DecodeCaptureBuffer(int fd, v4l2_buffer &buffer,
                                                                    CapturedPackedFrame &frame)
{
    const MappedBuffer mapped = m_buffers[buffer.index];
    const size_t usedBytes = (buffer.bytesused > 0 && buffer.bytesused <= mapped.length)
                                 ? static_cast<size_t>(buffer.bytesused)
                                 : mapped.length;

    const int packedWidth = m_width * 2;
    const PackedDecodeInput input{packedWidth, m_height, m_bytesPerLine, mapped.start, usedBytes};
    const auto decoded = DecodePackedFrame(m_pixelFormat, input, frame.packed);

    if (!IoctlRetry(fd, VIDIOC_QBUF, &buffer)) {
        MarkCaptureFault(false);
        return CaptureStatus::Fatal;
    }
    if (decoded != DecodePackedStatus::Frame) {
        MarkCaptureFault(false);
        return decoded == DecodePackedStatus::Fatal ? CaptureStatus::Fatal : CaptureStatus::NoFrame;
    }
    if (frame.packed.empty() || frame.packed.cols != packedWidth || frame.packed.rows != m_height) {
        MarkCaptureFault(false);
        return CaptureStatus::NoFrame;
    }

    frame.captureTimestampNs = TimevalToNs(buffer.timestamp);
    frame.arriveNs = MonoTimeUs() * 1000ULL;
    frame.captureMonotonicNs =
        (buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
                V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC
            ? frame.captureTimestampNs
            : frame.arriveNs;
    return CaptureStatus::Frame;
}

Core::Ports::StereoFrame UvcStereoCamera::BuildStereoFrame(
    const cv::Mat &packed, uint64_t captureTimestampNs,
    uint64_t captureMonotonicNs, uint64_t arriveNs)
{
    const int halfWidth = packed.cols / 2;
    const int packedHeight = packed.rows;
    Core::Ports::StereoFrame stereo{};
    stereo.left.cameraId = 0;
    stereo.right.cameraId = 1;
    stereo.left.timestampNs = captureTimestampNs;
    stereo.right.timestampNs = captureTimestampNs;
    stereo.left.captureMonotonicNs = static_cast<int64_t>(captureMonotonicNs);
    stereo.right.captureMonotonicNs = static_cast<int64_t>(captureMonotonicNs);
    stereo.left.arriveNs = static_cast<int64_t>(arriveNs);
    stereo.right.arriveNs = static_cast<int64_t>(arriveNs);
    stereo.left.sequence = ++m_sequence;
    stereo.right.sequence = m_sequence;
    if (m_swapEyes) {
        stereo.left.gray = packed(cv::Rect(halfWidth, 0, halfWidth, packedHeight)).clone();
        stereo.right.gray = packed(cv::Rect(0, 0, halfWidth, packedHeight)).clone();
    } else {
        stereo.left.gray = packed(cv::Rect(0, 0, halfWidth, packedHeight)).clone();
        stereo.right.gray = packed(cv::Rect(halfWidth, 0, halfWidth, packedHeight)).clone();
    }
    return stereo;
}

void UvcStereoCamera::MarkCaptureFault(bool acceptingFrames)
{
    if (m_running.load(std::memory_order_acquire)) {
        m_diag.healthy = false;
        m_diag.acceptFrames = acceptingFrames;
        ++m_diag.droppedPairs;
        PublishDiagnostics();
    }
}

Core::Ports::CameraHealth UvcStereoCamera::GetHealth() const
{
    const std::shared_ptr<const DiagnosticsSnapshot> snapshot =
        LoadDiagnosticsSnapshot();
    if (!snapshot) {
        return {};
    }
    return {snapshot->diagnostics.healthy,
            snapshot->diagnostics.droppedPairs};
}

Core::Ports::CameraDiagnostics UvcStereoCamera::GetDiagnostics() const
{
    const std::shared_ptr<const DiagnosticsSnapshot> snapshot =
        LoadDiagnosticsSnapshot();
    if (!snapshot) {
        return {};
    }
    Core::Ports::CameraDiagnostics out = snapshot->diagnostics;
    const uint64_t nowNs = MonoTimeUs() * 1000ULL;
    out.lastFrameAgeMsL =
        snapshot->lastFrameTimestampNs > 0
            ? static_cast<int64_t>(
                  (nowNs - snapshot->lastFrameTimestampNs) / 1000000ULL)
            : -1;
    out.lastFrameAgeMsR = out.lastFrameAgeMsL;
    out.lastPairAgeMs =
        snapshot->lastPairTimestampNs > 0
            ? static_cast<int64_t>(
                  (nowNs - snapshot->lastPairTimestampNs) / 1000000ULL)
            : -1;
    return out;
}

Core::Ports::CameraProviderSemantics UvcStereoCamera::Semantics() const
{
    return Core::Ports::CameraProviderSemantics::PackedStereoSingleDevice;
}

bool UvcStereoCamera::OpenDevice(const DeviceOpenParams &params)
{
    const std::string devicePath = "/dev/video" + std::to_string(params.deviceIndex);
    const int fd = ::open(devicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "[uvc] failed to open " << devicePath << " errno=" << errno << "\n";
        return false;
    }

    v4l2_capability caps{};
    if (!ValidateDeviceCapabilities(fd, devicePath, caps)) {
        ::close(fd);
        return false;
    }

    v4l2_format format{};
    if (!ApplyDeviceFormat(fd, devicePath, params, format)) {
        ::close(fd);
        return false;
    }

    ConfigureDeviceFps(fd, devicePath, params.fps);

    std::vector<MappedBuffer> buffers;
    uint32_t bufferCount = 0;
    if (!RequestAndMapBuffers(fd, devicePath, bufferCount, buffers)) {
        ::close(fd);
        return false;
    }

    if (!QueueBuffersAndStart(fd, devicePath, bufferCount, buffers)) {
        ::close(fd);
        return false;
    }

    ConfigureUvcControls(fd, params.deviceIndex, params.aeDisable, params.exposureUs, params.gain);
    StoreOpenedDevice(fd, format, std::move(buffers));
    LogOpenedDevice(params, caps, format, bufferCount);
    return true;
}

bool UvcStereoCamera::ValidateDeviceCapabilities(int fd, const std::string &devicePath, v4l2_capability &caps)
{
    if (!IoctlRetry(fd, VIDIOC_QUERYCAP, &caps)) {
        std::cerr << "[uvc] VIDIOC_QUERYCAP failed on " << devicePath << " errno=" << errno << "\n";
        return false;
    }
    if ((caps.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0U || (caps.capabilities & V4L2_CAP_STREAMING) == 0U) {
        std::cerr << "[uvc] device does not support capture+streaming: " << devicePath << "\n";
        return false;
    }
    return true;
}

bool UvcStereoCamera::ApplyDeviceFormat(int fd, const std::string &devicePath, const DeviceOpenParams &params,
                                        v4l2_format &format)
{
    format = {};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = static_cast<uint32_t>(std::max(0, params.width));
    format.fmt.pix.height = static_cast<uint32_t>(std::max(0, params.height));
    format.fmt.pix.pixelformat = YuyvFourcc();
    format.fmt.pix.field = V4L2_FIELD_NONE;
    if (!IoctlRetry(fd, VIDIOC_S_FMT, &format)) {
        std::cerr << "[uvc] VIDIOC_S_FMT failed on " << devicePath << " errno=" << errno << "\n";
        return false;
    }

    const int actualWidth = static_cast<int>(format.fmt.pix.width);
    const int actualHeight = static_cast<int>(format.fmt.pix.height);
    if (actualWidth == params.width && actualHeight == params.height) {
        return true;
    }
    std::cerr << "[uvc] configured packed frame " << params.width << "x" << params.height << " but device negotiated "
              << actualWidth << "x" << actualHeight << "\n";
    return false;
}

bool UvcStereoCamera::ConfigureDeviceFps(int fd, const std::string &devicePath, int fps)
{
    v4l2_streamparm streamParm{};
    streamParm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (fps <= 0 || !IoctlRetry(fd, VIDIOC_G_PARM, &streamParm)) {
        return true;
    }
    if ((streamParm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) == 0U) {
        return true;
    }
    streamParm.parm.capture.timeperframe.numerator = 1;
    streamParm.parm.capture.timeperframe.denominator = static_cast<uint32_t>(fps);
    if (!IoctlRetry(fd, VIDIOC_S_PARM, &streamParm)) {
        std::cerr << "[uvc] warning: VIDIOC_S_PARM failed on " << devicePath << " errno=" << errno << "\n";
    }
    return true;
}

bool UvcStereoCamera::RequestAndMapBuffers(int fd, const std::string &devicePath, uint32_t &bufferCount,
                                           std::vector<MappedBuffer> &buffers)
{
    v4l2_requestbuffers request{};
    request.count = static_cast<uint32_t>(DEFAULT_BUFFER_COUNT);
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (!IoctlRetry(fd, VIDIOC_REQBUFS, &request) || request.count == 0) {
        std::cerr << "[uvc] VIDIOC_REQBUFS failed on " << devicePath << " errno=" << errno << "\n";
        return false;
    }

    buffers.reserve(request.count);
    for (uint32_t i = 0; i < request.count; ++i) {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        if (!IoctlRetry(fd, VIDIOC_QUERYBUF, &buffer)) {
            std::cerr << "[uvc] VIDIOC_QUERYBUF failed index=" << i << " errno=" << errno << "\n";
            ReleaseMappedBuffers(buffers);
            return false;
        }
        void *start = ::mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buffer.m.offset);
        if (start == MAP_FAILED) {
            std::cerr << "[uvc] mmap failed index=" << i << " errno=" << errno << "\n";
            ReleaseMappedBuffers(buffers);
            return false;
        }
        buffers.push_back({start, static_cast<size_t>(buffer.length)});
    }
    bufferCount = request.count;
    return true;
}

bool UvcStereoCamera::QueueBuffersAndStart(int fd, const std::string &devicePath, uint32_t bufferCount,
                                           std::vector<MappedBuffer> &buffers)
{
    for (uint32_t i = 0; i < bufferCount; ++i) {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        if (!IoctlRetry(fd, VIDIOC_QBUF, &buffer)) {
            std::cerr << "[uvc] VIDIOC_QBUF failed index=" << i << " errno=" << errno << "\n";
            ReleaseMappedBuffers(buffers);
            return false;
        }
    }

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (IoctlRetry(fd, VIDIOC_STREAMON, &type)) {
        return true;
    }
    std::cerr << "[uvc] VIDIOC_STREAMON failed on " << devicePath << " errno=" << errno << "\n";
    ReleaseMappedBuffers(buffers);
    return false;
}

void UvcStereoCamera::ReleaseMappedBuffers(std::vector<MappedBuffer> &buffers)
{
    for (const auto &mapped : buffers) {
        if (mapped.start != nullptr && mapped.start != MAP_FAILED) {
            ::munmap(mapped.start, mapped.length);
        }
    }
    buffers.clear();
}

void UvcStereoCamera::StoreOpenedDevice(int fd, const v4l2_format &format, std::vector<MappedBuffer> &&buffers)
{
    m_streaming = true;
    m_pixelFormat = format.fmt.pix.pixelformat;
    m_bytesPerLine = format.fmt.pix.bytesperline;
    m_buffers = std::move(buffers);
    m_fd.store(fd, std::memory_order_release);
}

void UvcStereoCamera::LogOpenedDevice(const DeviceOpenParams &params, const v4l2_capability &caps,
                                      const v4l2_format &format, uint32_t bufferCount)
{
    double actualFps = 0.0;
    v4l2_streamparm actualParm{};
    actualParm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    const int fd = m_fd.load(std::memory_order_acquire);
    if (fd >= 0 && IoctlRetry(fd, VIDIOC_G_PARM, &actualParm) &&
        actualParm.parm.capture.timeperframe.numerator != 0) {
        const auto &tpf = actualParm.parm.capture.timeperframe;
        actualFps = static_cast<double>(tpf.denominator) / static_cast<double>(tpf.numerator);
    }

    const int actualWidth = static_cast<int>(format.fmt.pix.width);
    const int actualHeight = static_cast<int>(format.fmt.pix.height);
    std::cerr << "[uvc] opened device=" << params.deviceIndex << " driver="
              << reinterpret_cast<const char *>(caps.driver) << " card=" << reinterpret_cast<const char *>(caps.card)
              << " packed=" << actualWidth << "x" << actualHeight << " eye=" << (actualWidth / 2) << "x"
              << actualHeight << " fourcc=" << FourccToString(format.fmt.pix.pixelformat)
              << " bytesperline=" << format.fmt.pix.bytesperline << " buffers=" << bufferCount;
    if (actualFps > 0.0) {
        std::cerr << " fps=" << actualFps;
    }
    std::cerr << "\n";

    v4l2_buffer firstProbe{};
    firstProbe.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    firstProbe.memory = V4L2_MEMORY_MMAP;
    firstProbe.index = 0;
    if (fd >= 0 && IoctlRetry(fd, VIDIOC_QUERYBUF, &firstProbe)) {
        std::cerr << "[uvc] buffer timestamps=" << TimestampFlagToString(firstProbe.flags) << "\n";
    }
    if (format.fmt.pix.pixelformat != YuyvFourcc()) {
        std::cerr << "[uvc] warning: requested fourcc=YUYV but device negotiated "
                  << FourccToString(format.fmt.pix.pixelformat) << "\n";
    }
}

void UvcStereoCamera::CloseDevice()
{
    const int fd = m_fd.exchange(-1, std::memory_order_acq_rel);
    const bool streaming = m_streaming;
    m_streaming = false;
    m_pixelFormat = 0;
    m_bytesPerLine = 0;
    std::vector<MappedBuffer> buffers;
    buffers.swap(m_buffers);

    if (fd >= 0 && streaming) {
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (::ioctl(fd, VIDIOC_STREAMOFF, &type) != 0 && errno != EINVAL && errno != ENODEV && errno != EBADF) {
            std::cerr << "[uvc] warning: VIDIOC_STREAMOFF failed errno=" << errno << "\n";
        }
    }

    for (const auto &buffer : buffers) {
        if (buffer.start != nullptr && buffer.start != MAP_FAILED) {
            ::munmap(buffer.start, buffer.length);
        }
    }

    if (fd >= 0) {
        ::close(fd);
    }
}

void UvcStereoCamera::PushFrame(Core::Ports::StereoFrame &&frame, uint64_t captureTimestampNs)
{
    if (!m_running.load(std::memory_order_acquire)) {
        return;
    }

    StereoFrameItem item{};
    item.frame = std::move(frame);
    item.captureTimestampNs = captureTimestampNs;
    m_queue.push_back(std::move(item));
    while (m_queue.size() > m_maxQueue) {
        m_queue.pop_front();
        ++m_diag.droppedPairs;
    }

    m_diag.healthy = true;
    m_diag.acceptFrames = true;
    ++m_diag.rawCountL;
    ++m_diag.rawCountR;
    m_diag.lastRawSeqL = m_sequence;
    m_diag.lastRawSeqR = m_sequence;
    m_diag.lastPairDtMs = 0;
    m_diag.lastRejectDtUs = 0;
    m_diag.pendingL = 0;
    m_diag.pendingR = 0;
    m_diag.pairedQueue = m_queue.size();
    m_lastFrameTimestampNs = captureTimestampNs;
    m_lastPairTimestampNs = captureTimestampNs;
    PublishDiagnostics();
}

void UvcStereoCamera::PublishDiagnostics()
{
    DiagnosticsSnapshot snapshot{};
    snapshot.diagnostics = m_diag;
    snapshot.lastFrameTimestampNs = m_lastFrameTimestampNs;
    snapshot.lastPairTimestampNs = m_lastPairTimestampNs;
    std::atomic_store_explicit(
        &m_diagSnapshot,
        std::make_shared<const DiagnosticsSnapshot>(std::move(snapshot)),
        std::memory_order_release);
}

std::shared_ptr<const UvcStereoCamera::DiagnosticsSnapshot>
UvcStereoCamera::LoadDiagnosticsSnapshot() const
{
    return std::atomic_load_explicit(&m_diagSnapshot,
                                     std::memory_order_acquire);
}

} // namespace SmartDrone::Adapters::Camera
