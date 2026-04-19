#include "adapters/camera/uvc_stereo_camera.h"

#include <algorithm>
#include <chrono>
#include <iostream>

#include <opencv2/imgproc.hpp>

#include "common/time_utils.h"

namespace smartdrone::adapters::camera {

namespace {

constexpr int kDefaultBackend = cv::CAP_V4L2;

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
    m_maxQueue = static_cast<size_t>(std::max(1, aliases.pairQueue));
    m_sequence = 0;
    m_lastFrameTimestampNs = 0;
    m_lastPairTimestampNs = 0;

    const int packedWidth = (m_width > 0) ? (m_width * 2) : 0;
    if (!OpenDevice(m_deviceIndex, packedWidth, m_height, m_fps)) {
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
        stereo.left.gray = packedGray(cv::Rect(0, 0, halfWidth, packedHeight)).clone();
        stereo.right.gray = packedGray(cv::Rect(halfWidth, 0, halfWidth, packedHeight)).clone();

        PushFrame(std::move(stereo), captureUs * 1000ULL);
    }
}

bool UvcStereoCamera::OpenDevice(int deviceIndex, int width, int height, int fps)
{
    if (!m_cap.open(deviceIndex, kDefaultBackend)) {
        return false;
    }
    m_cap.set(cv::CAP_PROP_BUFFERSIZE, 1.0);
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
    if (actualWidth != width || actualHeight != height) {
        std::cerr << "[uvc] configured packed frame " << width << "x" << height
                  << " but device negotiated " << actualWidth << "x" << actualHeight << "\n";
        m_cap.release();
        return false;
    }

    std::cerr << "[uvc] opened device=" << deviceIndex << " backend=" << kDefaultBackend
              << " packed=" << actualWidth << "x" << actualHeight << " eye=" << (actualWidth / 2) << "x"
              << actualHeight;
    if (actualFps > 0.0) {
        std::cerr << " fps=" << actualFps;
    }
    std::cerr << "\n";
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
