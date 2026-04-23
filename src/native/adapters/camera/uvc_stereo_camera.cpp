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

#include <fcntl.h>
#include <linux/videodev2.h>
#include <opencv2/imgproc.hpp>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>

#include "common/time_utils.h"

namespace smartdrone::adapters::camera {

namespace {

constexpr size_t kDefaultBufferCount = 4;
constexpr int kCapturePollTimeoutMs = 250;

uint32_t YuyvFourcc() { return v4l2_fourcc('Y', 'U', 'Y', 'V'); }

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
        out.max = out.min;
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
        const int32_t clampedGain = std::clamp<int32_t>(requestedGain, gainInfo.min, gainInfo.max);
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

} // namespace

UvcStereoCamera::~UvcStereoCamera() { Close(); }

bool UvcStereoCamera::Open(const core::application::MainRuntimeAliases &aliases)
{
    Close();

    if (!aliases.uvcPackedStereo) {
        std::cerr << "[uvc] unsupported configuration: uvc_stereo_opencv requires camera.uvc_packed_stereo=true\n";
        return false;
    }

    m_deviceIndex = aliases.uvcDeviceIndex;
    m_width = aliases.uvcEyeWidth;
    m_height = aliases.uvcEyeHeight;
    m_swapEyes = aliases.uvcSwapEyes;
    if (m_deviceIndex < 0 || m_width <= 0 || m_height <= 0) {
        std::cerr << "[uvc] invalid configuration: device_index=" << m_deviceIndex << " eye=" << m_width << "x"
                  << m_height << "\n";
        return false;
    }

    m_fps = aliases.fps;
    m_maxQueue = 1;
    if (aliases.pairQueue > 1) {
        std::cerr << "[uvc] forcing packed-stereo frame_queue=1 for lowest-latency capture (requested="
                  << aliases.pairQueue << ")\n";
    }
    m_sequence = 0;
    m_lastFrameTimestampNs = 0;
    m_lastPairTimestampNs = 0;

    const int packedWidth = (m_width > 0) ? (m_width * 2) : 0;
    if (!OpenDevice(m_deviceIndex, packedWidth, m_height, m_fps, aliases.aeDisable, aliases.exposureUs, aliases.gain)) {
        CloseDevice();
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_open = true;
        m_running = true;
        m_diag = {};
        m_diag.healthy = true;
        m_diag.acceptFrames = true;
        m_diag.pairTolNs = 0;
    }

    m_thread = std::thread(&UvcStereoCamera::CaptureLoop, this);
    return true;
}

void UvcStereoCamera::Close()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_running = false;
        m_open = false;
        m_diag.acceptFrames = false;
    }
    m_cv.notify_all();

    if (m_thread.joinable()) {
        m_thread.join();
    }

    CloseDevice();

    std::lock_guard<std::mutex> lock(m_mutex);
    m_queue.clear();
    m_lastFrameTimestampNs = 0;
    m_lastPairTimestampNs = 0;
    m_diag.healthy = false;
    m_diag.lastFrameAgeMsL = -1;
    m_diag.lastFrameAgeMsR = -1;
    m_diag.lastPairAgeMs = -1;
    m_diag.pairedQueue = 0;
}

bool UvcStereoCamera::Start() { return m_open; }

void UvcStereoCamera::Stop() { Close(); }

bool UvcStereoCamera::GrabStereo(core::ports::StereoFrame &out, int timeoutMs, bool preferLatest, uint64_t minTimestampNs)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(std::max(1, timeoutMs));

    while (m_running || !m_queue.empty()) {
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

        if (candidate != m_queue.end()) {
            out = std::move(candidate->frame);
            m_queue.erase(m_queue.begin(), std::next(candidate));
            m_diag.pairedQueue = m_queue.size();
            m_diag.lastPairAgeMs = 0;
            return true;
        }

        if (m_cv.wait_until(lock, deadline) == std::cv_status::timeout) {
            break;
        }
    }

    return false;
}

core::ports::CameraHealth UvcStereoCamera::GetHealth() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return {m_diag.healthy, m_diag.droppedPairs};
}

core::ports::CameraDiagnostics UvcStereoCamera::GetDiagnostics() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    core::ports::CameraDiagnostics out = m_diag;
    const uint64_t nowNs = MonoTimeUs() * 1000ULL;
    out.lastFrameAgeMsL =
        m_lastFrameTimestampNs > 0 ? static_cast<int64_t>((nowNs - m_lastFrameTimestampNs) / 1000000ULL) : -1;
    out.lastFrameAgeMsR = out.lastFrameAgeMsL;
    out.lastPairAgeMs =
        m_lastPairTimestampNs > 0 ? static_cast<int64_t>((nowNs - m_lastPairTimestampNs) / 1000000ULL) : -1;
    return out;
}

core::ports::CameraProviderSemantics UvcStereoCamera::Semantics() const
{
    return core::ports::CameraProviderSemantics::PackedStereoSingleDevice;
}

void UvcStereoCamera::CaptureLoop()
{
    while (true) {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (!m_running) {
                break;
            }
        }

        int fd = -1;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            fd = m_fd;
        }
        if (fd < 0) {
            break;
        }

        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        const int pollRc = ::poll(&pfd, 1, kCapturePollTimeoutMs);
        if (pollRc < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_running) {
                m_diag.healthy = false;
                m_diag.acceptFrames = false;
                ++m_diag.droppedPairs;
            }
            break;
        }
        if (pollRc == 0) {
            continue;
        }
        if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_running) {
                m_diag.healthy = false;
                m_diag.acceptFrames = false;
                ++m_diag.droppedPairs;
            }
            break;
        }
        if ((pfd.revents & POLLIN) == 0) {
            continue;
        }

        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        if (!IoctlRetry(fd, VIDIOC_DQBUF, &buffer)) {
            const int savedErrno = errno;
            if (savedErrno == EAGAIN) {
                continue;
            }
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_running && savedErrno != EPIPE && savedErrno != ENODEV && savedErrno != EBADF) {
                m_diag.healthy = false;
                m_diag.acceptFrames = false;
                ++m_diag.droppedPairs;
            }
            break;
        }

        if (buffer.index >= m_buffers.size()) {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_diag.healthy = false;
            m_diag.acceptFrames = false;
            ++m_diag.droppedPairs;
            break;
        }

        const uint64_t captureTimestampNs = TimevalToNs(buffer.timestamp);
        const uint64_t arriveNs = MonoTimeUs() * 1000ULL;
        const MappedBuffer mapped = m_buffers[buffer.index];
        const size_t usedBytes = (buffer.bytesused > 0 && buffer.bytesused <= mapped.length)
                                     ? static_cast<size_t>(buffer.bytesused)
                                     : mapped.length;

        cv::Mat packed;
        const int packedWidth = m_width * 2;
        const int packedHeight = m_height;
        if (m_pixelFormat == V4L2_PIX_FMT_YUYV) {
            const size_t rowBytes = static_cast<size_t>(packedWidth) * 2U;
            if (usedBytes < rowBytes * static_cast<size_t>(packedHeight)) {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_diag.healthy = false;
                m_diag.acceptFrames = false;
                ++m_diag.droppedPairs;
                if (IoctlRetry(fd, VIDIOC_QBUF, &buffer)) {
                    continue;
                }
                break;
            }
            const size_t step = std::max<size_t>(rowBytes, static_cast<size_t>(m_bytesPerLine));
            cv::Mat packedYuyv(packedHeight, packedWidth, CV_8UC2, mapped.start, step);
            cv::cvtColor(packedYuyv, packed, cv::COLOR_YUV2GRAY_YUY2);
        } else if (m_pixelFormat == V4L2_PIX_FMT_GREY) {
            const size_t rowBytes = static_cast<size_t>(packedWidth);
            if (usedBytes < rowBytes * static_cast<size_t>(packedHeight)) {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_diag.healthy = false;
                m_diag.acceptFrames = false;
                ++m_diag.droppedPairs;
                if (IoctlRetry(fd, VIDIOC_QBUF, &buffer)) {
                    continue;
                }
                break;
            }
            const size_t step = std::max<size_t>(rowBytes, static_cast<size_t>(m_bytesPerLine));
            cv::Mat packedGray(packedHeight, packedWidth, CV_8UC1, mapped.start, step);
            packed = packedGray.clone();
        } else if (m_pixelFormat == V4L2_PIX_FMT_BGR24) {
            const size_t rowBytes = static_cast<size_t>(packedWidth) * 3U;
            if (usedBytes < rowBytes * static_cast<size_t>(packedHeight)) {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_diag.healthy = false;
                m_diag.acceptFrames = false;
                ++m_diag.droppedPairs;
                if (IoctlRetry(fd, VIDIOC_QBUF, &buffer)) {
                    continue;
                }
                break;
            }
            const size_t step = std::max<size_t>(rowBytes, static_cast<size_t>(m_bytesPerLine));
            cv::Mat packedBgr(packedHeight, packedWidth, CV_8UC3, mapped.start, step);
            cv::cvtColor(packedBgr, packed, cv::COLOR_BGR2GRAY);
        } else if (m_pixelFormat == V4L2_PIX_FMT_RGB24) {
            const size_t rowBytes = static_cast<size_t>(packedWidth) * 3U;
            if (usedBytes < rowBytes * static_cast<size_t>(packedHeight)) {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_diag.healthy = false;
                m_diag.acceptFrames = false;
                ++m_diag.droppedPairs;
                if (IoctlRetry(fd, VIDIOC_QBUF, &buffer)) {
                    continue;
                }
                break;
            }
            const size_t step = std::max<size_t>(rowBytes, static_cast<size_t>(m_bytesPerLine));
            cv::Mat packedRgb(packedHeight, packedWidth, CV_8UC3, mapped.start, step);
            cv::cvtColor(packedRgb, packed, cv::COLOR_RGB2GRAY);
        } else {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_diag.healthy = false;
            m_diag.acceptFrames = false;
            ++m_diag.droppedPairs;
            std::cerr << "[uvc] unsupported pixel format for packed stereo: " << FourccToString(m_pixelFormat) << "\n";
            if (IoctlRetry(fd, VIDIOC_QBUF, &buffer)) {
                continue;
            }
            break;
        }

        if (!IoctlRetry(fd, VIDIOC_QBUF, &buffer)) {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_running) {
                m_diag.healthy = false;
                m_diag.acceptFrames = false;
                ++m_diag.droppedPairs;
            }
            break;
        }

        if (packed.empty() || packed.cols != packedWidth || packed.rows != packedHeight) {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_diag.healthy = false;
            m_diag.acceptFrames = false;
            ++m_diag.droppedPairs;
            continue;
        }

        const int halfWidth = packedWidth / 2;
        core::ports::StereoFrame stereo{};
        stereo.left.cameraId = 0;
        stereo.right.cameraId = 1;
        stereo.left.timestampNs = captureTimestampNs;
        stereo.right.timestampNs = captureTimestampNs;
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

        PushFrame(std::move(stereo), captureTimestampNs);
    }
}

bool UvcStereoCamera::OpenDevice(int deviceIndex, int width, int height, int fps, bool aeDisable, int exposureUs,
                                 float gain)
{
    const std::string devicePath = "/dev/video" + std::to_string(deviceIndex);
    const int fd = ::open(devicePath.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "[uvc] failed to open " << devicePath << " errno=" << errno << "\n";
        return false;
    }

    v4l2_capability caps{};
    if (!IoctlRetry(fd, VIDIOC_QUERYCAP, &caps)) {
        std::cerr << "[uvc] VIDIOC_QUERYCAP failed on " << devicePath << " errno=" << errno << "\n";
        ::close(fd);
        return false;
    }
    if ((caps.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0U || (caps.capabilities & V4L2_CAP_STREAMING) == 0U) {
        std::cerr << "[uvc] device does not support capture+streaming: " << devicePath << "\n";
        ::close(fd);
        return false;
    }

    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = static_cast<uint32_t>(std::max(0, width));
    format.fmt.pix.height = static_cast<uint32_t>(std::max(0, height));
    format.fmt.pix.pixelformat = YuyvFourcc();
    format.fmt.pix.field = V4L2_FIELD_NONE;
    format.fmt.pix.bytesperline = 0;
    format.fmt.pix.sizeimage = 0;
    if (!IoctlRetry(fd, VIDIOC_S_FMT, &format)) {
        std::cerr << "[uvc] VIDIOC_S_FMT failed on " << devicePath << " errno=" << errno << "\n";
        ::close(fd);
        return false;
    }

    const int actualWidth = static_cast<int>(format.fmt.pix.width);
    const int actualHeight = static_cast<int>(format.fmt.pix.height);
    if (actualWidth != width || actualHeight != height) {
        std::cerr << "[uvc] configured packed frame " << width << "x" << height
                  << " but device negotiated " << actualWidth << "x" << actualHeight << "\n";
        ::close(fd);
        return false;
    }

    v4l2_streamparm streamParm{};
    streamParm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (fps > 0 && IoctlRetry(fd, VIDIOC_G_PARM, &streamParm)) {
        if ((streamParm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) != 0U) {
            streamParm.parm.capture.timeperframe.numerator = 1;
            streamParm.parm.capture.timeperframe.denominator = static_cast<uint32_t>(fps);
            if (!IoctlRetry(fd, VIDIOC_S_PARM, &streamParm)) {
                std::cerr << "[uvc] warning: VIDIOC_S_PARM failed on " << devicePath << " errno=" << errno << "\n";
            }
        }
    }

    v4l2_requestbuffers request{};
    request.count = static_cast<uint32_t>(kDefaultBufferCount);
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (!IoctlRetry(fd, VIDIOC_REQBUFS, &request)) {
        std::cerr << "[uvc] VIDIOC_REQBUFS failed on " << devicePath << " errno=" << errno << "\n";
        ::close(fd);
        return false;
    }
    if (request.count == 0) {
        std::cerr << "[uvc] VIDIOC_REQBUFS returned zero buffers on " << devicePath << "\n";
        ::close(fd);
        return false;
    }

    std::vector<MappedBuffer> buffers;
    buffers.reserve(request.count);
    bool setupOk = true;
    for (uint32_t i = 0; i < request.count; ++i) {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        if (!IoctlRetry(fd, VIDIOC_QUERYBUF, &buffer)) {
            std::cerr << "[uvc] VIDIOC_QUERYBUF failed index=" << i << " errno=" << errno << "\n";
            setupOk = false;
            break;
        }

        void *start = ::mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buffer.m.offset);
        if (start == MAP_FAILED) {
            std::cerr << "[uvc] mmap failed index=" << i << " errno=" << errno << "\n";
            setupOk = false;
            break;
        }

        buffers.push_back({start, static_cast<size_t>(buffer.length)});
    }

    if (!setupOk) {
        for (const auto &mapped : buffers) {
            if (mapped.start != nullptr && mapped.start != MAP_FAILED) {
                ::munmap(mapped.start, mapped.length);
            }
        }
        ::close(fd);
        return false;
    }

    for (uint32_t i = 0; i < request.count; ++i) {
        v4l2_buffer buffer{};
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        if (!IoctlRetry(fd, VIDIOC_QBUF, &buffer)) {
            std::cerr << "[uvc] VIDIOC_QBUF failed index=" << i << " errno=" << errno << "\n";
            for (const auto &mapped : buffers) {
                if (mapped.start != nullptr && mapped.start != MAP_FAILED) {
                    ::munmap(mapped.start, mapped.length);
                }
            }
            ::close(fd);
            return false;
        }
    }

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (!IoctlRetry(fd, VIDIOC_STREAMON, &type)) {
        std::cerr << "[uvc] VIDIOC_STREAMON failed on " << devicePath << " errno=" << errno << "\n";
        for (const auto &mapped : buffers) {
            if (mapped.start != nullptr && mapped.start != MAP_FAILED) {
                ::munmap(mapped.start, mapped.length);
            }
        }
        ::close(fd);
        return false;
    }

    ConfigureUvcControls(fd, deviceIndex, aeDisable, exposureUs, gain);

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_fd = fd;
        m_streaming = true;
        m_pixelFormat = format.fmt.pix.pixelformat;
        m_bytesPerLine = format.fmt.pix.bytesperline;
        m_buffers = std::move(buffers);
    }

    double actualFps = 0.0;
    v4l2_streamparm actualParm{};
    actualParm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (IoctlRetry(fd, VIDIOC_G_PARM, &actualParm)) {
        const auto &tpf = actualParm.parm.capture.timeperframe;
        if (tpf.numerator != 0) {
            actualFps = static_cast<double>(tpf.denominator) / static_cast<double>(tpf.numerator);
        }
    }

    std::cerr << "[uvc] opened device=" << deviceIndex << " driver=" << reinterpret_cast<const char *>(caps.driver)
              << " card=" << reinterpret_cast<const char *>(caps.card) << " packed=" << actualWidth << "x"
              << actualHeight << " eye=" << (actualWidth / 2) << "x" << actualHeight
              << " fourcc=" << FourccToString(format.fmt.pix.pixelformat)
              << " bytesperline=" << format.fmt.pix.bytesperline << " buffers=" << request.count;
    if (actualFps > 0.0) {
        std::cerr << " fps=" << actualFps;
    }
    std::cerr << "\n";

    v4l2_buffer firstProbe{};
    firstProbe.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    firstProbe.memory = V4L2_MEMORY_MMAP;
    firstProbe.index = 0;
    if (IoctlRetry(fd, VIDIOC_QUERYBUF, &firstProbe)) {
        std::cerr << "[uvc] buffer timestamps=" << TimestampFlagToString(firstProbe.flags) << "\n";
    }

    if (format.fmt.pix.pixelformat != YuyvFourcc()) {
        std::cerr << "[uvc] warning: requested fourcc=YUYV but device negotiated "
                  << FourccToString(format.fmt.pix.pixelformat) << "\n";
    }

    return true;
}

void UvcStereoCamera::CloseDevice()
{
    int fd = -1;
    bool streaming = false;
    std::vector<MappedBuffer> buffers;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        fd = m_fd;
        streaming = m_streaming;
        m_fd = -1;
        m_streaming = false;
        m_pixelFormat = 0;
        m_bytesPerLine = 0;
        buffers.swap(m_buffers);
    }

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

void UvcStereoCamera::PushFrame(core::ports::StereoFrame &&frame, uint64_t captureTimestampNs)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_running) {
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
    m_cv.notify_all();
}

} // namespace smartdrone::adapters::camera
