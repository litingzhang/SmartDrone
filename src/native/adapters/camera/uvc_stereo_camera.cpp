#include "adapters/camera/uvc_stereo_camera.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <iostream>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <opencv2/imgproc.hpp>
#include <sys/ioctl.h>
#include <unistd.h>

#include "common/time_utils.h"

namespace smartdrone::adapters::camera {

namespace {

constexpr int kDefaultBackend = cv::CAP_V4L2;

int YuyvFourcc() { return cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'); }

std::string FourccToString(int fourcc)
{
    if (fourcc <= 0) {
        return "unknown";
    }
    std::string text(4, ' ');
    text[0] = static_cast<char>(fourcc & 0xFF);
    text[1] = static_cast<char>((fourcc >> 8) & 0xFF);
    text[2] = static_cast<char>((fourcc >> 16) & 0xFF);
    text[3] = static_cast<char>((fourcc >> 24) & 0xFF);
    return text;
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

void ConfigureUvcControls(int deviceIndex, bool aeDisable, int exposureUs, float gain)
{
    const std::string devicePath = "/dev/video" + std::to_string(deviceIndex);
    const int fd = ::open(devicePath.c_str(), O_RDWR);
    if (fd < 0) {
        std::cerr << "[uvc] warning: failed to open controls on " << devicePath << " errno=" << errno << "\n";
        return;
    }

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
        std::cerr << "[uvc] control exposure mode=manual exposure_us=" << exposureUs
                  << " exposure_absolute=" << clampedExposure << "\n";
    } else if (!aeDisable) {
        std::cerr << "[uvc] control exposure mode=auto\n";
    }

    if (haveGain) {
        const int requestedGain = std::max(0, static_cast<int>(std::lround(gain)));
        const int32_t clampedGain = std::clamp<int32_t>(requestedGain, gainInfo.min, gainInfo.max);
        SetUvcControl(fd, V4L2_CID_GAIN, clampedGain, "gain");
        std::cerr << "[uvc] control gain=" << clampedGain << "\n";
    }

    ::close(fd);
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
    if (m_deviceIndex < 0 || m_width <= 0 || m_height <= 0) {
        std::cerr << "[uvc] invalid configuration: device_index=" << m_deviceIndex << " eye=" << m_width << "x"
                  << m_height << "\n";
        return false;
    }
    m_fps = aliases.fps;
    // Packed-stereo UVC is used for live teleoperation/SLAM preview, so keep
    // only the newest frame to avoid queueing stale images behind slow
    // processing stages.
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

    if (m_cap.isOpened()) {
        m_cap.release();
    }

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

        if (!m_cap.grab()) {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_diag.healthy = false;
            m_diag.acceptFrames = false;
            ++m_diag.droppedPairs;
            continue;
        }

        const uint64_t captureUs = MonoTimeUs();
        cv::Mat packed;
        if (!m_cap.retrieve(packed) || packed.empty()) {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_diag.healthy = false;
            m_diag.acceptFrames = false;
            ++m_diag.droppedPairs;
            continue;
        }
        const uint64_t arriveNs = MonoTimeUs() * 1000ULL;

        const int packedWidth = packed.cols;
        const int packedHeight = packed.rows;
        if (packedWidth < 2 || (packedWidth % 2) != 0 || packedHeight <= 0) {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_diag.healthy = false;
            m_diag.acceptFrames = false;
            ++m_diag.droppedPairs;
            continue;
        }
        if (packedWidth != (m_width * 2) || packedHeight != m_height) {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_diag.healthy = false;
            m_diag.acceptFrames = false;
            ++m_diag.droppedPairs;
            std::cerr << "[uvc] packed frame dimension mismatch expected=" << (m_width * 2) << "x" << m_height
                      << " actual=" << packedWidth << "x" << packedHeight << "\n";
            continue;
        }

        const int halfWidth = packedWidth / 2;
        cv::Mat packedGray;
        if (packed.channels() == 1) {
            packedGray = packed;
        } else if (packed.type() == CV_8UC2) {
            cv::cvtColor(packed, packedGray, cv::COLOR_YUV2GRAY_YUY2);
        } else {
            cv::cvtColor(packed, packedGray, cv::COLOR_BGR2GRAY);
        }

        core::ports::StereoFrame stereo{};
        stereo.left.cameraId = 0;
        stereo.right.cameraId = 1;
        stereo.left.timestampNs = captureUs * 1000ULL;
        stereo.right.timestampNs = captureUs * 1000ULL;
        stereo.left.arriveNs = static_cast<int64_t>(arriveNs);
        stereo.right.arriveNs = static_cast<int64_t>(arriveNs);
        stereo.left.sequence = ++m_sequence;
        stereo.right.sequence = m_sequence;
        // The current UVC device exposes the physical right-eye image on the
        // left half of the packed frame and the physical left-eye image on the
        // right half, so the logical stereo definition is intentionally
        // reversed here.
        stereo.left.gray = packedGray(cv::Rect(halfWidth, 0, halfWidth, packedHeight)).clone();
        stereo.right.gray = packedGray(cv::Rect(0, 0, halfWidth, packedHeight)).clone();

        PushFrame(std::move(stereo), captureUs * 1000ULL);
    }
}

bool UvcStereoCamera::OpenDevice(int deviceIndex, int width, int height, int fps, bool aeDisable, int exposureUs,
                                 float gain)
{
    if (!m_cap.open(deviceIndex, kDefaultBackend)) {
        return false;
    }
    m_cap.set(cv::CAP_PROP_BUFFERSIZE, 1.0);
    const int requestedFourcc = YuyvFourcc();
    m_cap.set(cv::CAP_PROP_FOURCC, static_cast<double>(requestedFourcc));
    if (width > 0) {
        m_cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
    }
    if (height > 0) {
        m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
    }
    if (fps > 0) {
        m_cap.set(cv::CAP_PROP_FPS, static_cast<double>(fps));
    }
    if (!m_cap.isOpened()) {
        return false;
    }

    const int actualWidth = static_cast<int>(m_cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int actualHeight = static_cast<int>(m_cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double actualFps = m_cap.get(cv::CAP_PROP_FPS);
    const int actualFourcc = static_cast<int>(m_cap.get(cv::CAP_PROP_FOURCC));
    if (actualWidth != width || actualHeight != height) {
        std::cerr << "[uvc] configured packed frame " << width << "x" << height
                  << " but device negotiated " << actualWidth << "x" << actualHeight << "\n";
        m_cap.release();
        return false;
    }

    std::cerr << "[uvc] opened device=" << deviceIndex << " backend=" << kDefaultBackend
              << " packed=" << actualWidth << "x" << actualHeight << " eye=" << (actualWidth / 2) << "x"
              << actualHeight << " fourcc=" << FourccToString(actualFourcc);
    if (actualFps > 0.0) {
        std::cerr << " fps=" << actualFps;
    }
    std::cerr << "\n";
    if (actualFourcc != requestedFourcc) {
        std::cerr << "[uvc] warning: requested fourcc=YUYV but device negotiated " << FourccToString(actualFourcc)
                  << "\n";
    }
    ConfigureUvcControls(deviceIndex, aeDisable, exposureUs, gain);
    return true;
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
