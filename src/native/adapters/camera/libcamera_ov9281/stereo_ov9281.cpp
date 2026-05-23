#include "adapters/camera/libcamera_ov9281/stereo_ov9281.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <optional>
#include <string_view>

#include <sys/mman.h>
#include <time.h>

#include "common/runtime_state.h"

namespace {

constexpr int32_t kSlamAeMaxExposureUs = 7000;
constexpr float kSlamAeMaxGain = 6.0f;

struct R16CompressionState {
    bool initialized{false};
    double lo{0.0};
    double hi{4095.0};
    std::array<uint8_t, 65536> lut{};
};

void *MMapFD(int fd, size_t len, off_t off = 0)
{
    void *p = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, off);
    if (p == MAP_FAILED) {
        return nullptr;
    }
    return p;
}

void MUnmap(void *p, size_t len)
{
    if (p && p != MAP_FAILED) {
        munmap(p, len);
    }
}

int64_t Abs64(int64_t x)
{
    return x < 0 ? -x : x;
}

int64_t NowNs()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

bool SetControlIfSupported(libcamera::Camera *camera, libcamera::ControlList &controls, const char *name,
                           const libcamera::ControlValue &value)
{
    if (!camera || !name || name[0] == '\0') {
        return false;
    }
    const libcamera::ControlInfoMap &controlMap = camera->controls();
    for (const auto &[id, info] : controlMap) {
        (void)info;
        if (!id) {
            continue;
        }
        if (std::string_view(id->name()) == name) {
            controls.set(id->id(), value);
            return true;
        }
    }
    return false;
}

using R16Histogram = std::array<uint32_t, 4096>;

R16CompressionState &R16StateForCamera(int camIndex)
{
    static std::array<R16CompressionState, 2> states{};
    const size_t stateIdx = static_cast<size_t>((camIndex == 1) ? 1 : 0);
    return states[stateIdx];
}

uint32_t BuildR16Histogram(const cv::Mat &src16, R16Histogram &hist)
{
    uint32_t sampleCount = 0;
    for (int y = 0; y < src16.rows; y += 2) {
        const uint16_t *row = src16.ptr<uint16_t>(y);
        for (int x = 0; x < src16.cols; x += 2) {
            const uint16_t value = row[x];
            const uint16_t bin = static_cast<uint16_t>(value >> 4);
            hist[std::min<size_t>(bin, hist.size() - 1)]++;
            ++sampleCount;
        }
    }
    return sampleCount;
}

void EstimateR16Range(const R16Histogram &hist, uint32_t sampleCount, double &targetLo, double &targetHi)
{
    const uint32_t lowTargetCount = static_cast<uint32_t>(sampleCount * 0.01);
    const uint32_t highTargetCount = static_cast<uint32_t>(sampleCount * 0.99);
    uint32_t cumulative = 0;
    int lowBin = 0;
    int highBin = static_cast<int>(hist.size() - 1);
    bool lowFound = false;
    for (size_t i = 0; i < hist.size(); ++i) {
        cumulative += hist[i];
        if (!lowFound && cumulative >= lowTargetCount) {
            lowBin = static_cast<int>(i);
            lowFound = true;
        }
        if (cumulative >= highTargetCount) {
            highBin = static_cast<int>(i);
            break;
        }
    }

    targetLo = static_cast<double>(lowBin << 4);
    targetHi = static_cast<double>(highBin << 4);
    if (targetHi <= targetLo + 64.0) {
        targetHi = targetLo + 64.0;
    }
}

void UpdateR16Range(R16CompressionState &state, double targetLo, double targetHi)
{
    if (!state.initialized) {
        state.lo = targetLo;
        state.hi = targetHi;
        state.initialized = true;
    } else {
        state.lo = state.lo * 0.85 + targetLo * 0.15;
        state.hi = state.hi * 0.85 + targetHi * 0.15;
    }
    if (state.hi <= state.lo + 64.0) {
        state.hi = state.lo + 64.0;
    }
}

void RebuildR16Lut(R16CompressionState &state)
{
    const double invRange = 255.0 / (state.hi - state.lo);
    for (int i = 0; i < 65536; ++i) {
        const double v = (static_cast<double>(i) - state.lo) * invRange;
        const int out = (v <= 0.0) ? 0 : (v >= 255.0) ? 255
                                                      : static_cast<int>(v + 0.5);
        state.lut[static_cast<size_t>(i)] = static_cast<uint8_t>(out);
    }
}

void ApplyR16Lut(const cv::Mat &src16, cv::Mat &dst8, const R16CompressionState &state)
{
    dst8.create(src16.rows, src16.cols, CV_8UC1);
    for (int y = 0; y < src16.rows; ++y) {
        const uint16_t *srcRow = src16.ptr<uint16_t>(y);
        uint8_t *dstRow = dst8.ptr<uint8_t>(y);
        for (int x = 0; x < src16.cols; ++x) {
            dstRow[x] = state.lut[srcRow[x]];
        }
    }
}

void LogR16Compression(int camIndex, uint32_t seq, const R16CompressionState &state, uint32_t sampleCount)
{
    if ((seq % 120u) == 0u) {
        std::cerr << "[cam_r16] cam=" << camIndex << " seq=" << seq << " lo=" << state.lo << " hi=" << state.hi
                  << " samples=" << sampleCount << "\n";
    }
}

void CompressR16AdaptiveForSlam(const cv::Mat &src16, cv::Mat &dst8, int camIndex, uint32_t seq)
{
    R16CompressionState &state = R16StateForCamera(camIndex);
    R16Histogram hist{};
    const uint32_t sampleCount = BuildR16Histogram(src16, hist);
    if (sampleCount == 0) {
        dst8.create(src16.rows, src16.cols, CV_8UC1);
        dst8.setTo(0);
        return;
    }

    double targetLo = 0.0;
    double targetHi = 0.0;
    EstimateR16Range(hist, sampleCount, targetLo, targetHi);
    UpdateR16Range(state, targetLo, targetHi);
    RebuildR16Lut(state);
    ApplyR16Lut(src16, dst8, state);
    LogR16Compression(camIndex, seq, state, sampleCount);
}

} // namespace

bool LibcameraMonoCam::Open(const MonoCameraOpenParams &params)
{
    if (!PrepareOpen(params)) {
        return false;
    }
    if (!ConfigureStream(params)) {
        return false;
    }

    const int64_t frameDurationUs = std::max<int64_t>(1, 1000000LL / std::max(1, params.fps));
    m_controls.set(libcamera::controls::FrameDurationLimits,
                   libcamera::Span<const int64_t, 2>({frameDurationUs, frameDurationUs}));
    const int32_t exposureCapUs =
        static_cast<int32_t>(std::max<int64_t>(1, std::min<int64_t>(kSlamAeMaxExposureUs, frameDurationUs - 500)));
    ConfigureControls(params, frameDurationUs, exposureCapUs);

    if (!ConfigureCamera()) {
        return false;
    }

    m_allocator = std::make_unique<libcamera::FrameBufferAllocator>(m_cam);
    if (m_allocator->allocate(m_stream) < 0) {
        std::cerr << "Failed to allocate buffers\n";
        ResetOpenState();
        return false;
    }

    const auto &buffers = m_allocator->buffers(m_stream);
    if (buffers.empty()) {
        std::cerr << "No buffers allocated\n";
        ResetOpenState();
        return false;
    }

    if (!MapBuffers(buffers) || !CreateRequests(buffers)) {
        return false;
    }
    CreateFramePool(buffers.size() + 2);

    m_cam->requestCompleted.connect(this, &LibcameraMonoCam::OnRequestComplete);
    return true;
}

bool LibcameraMonoCam::PrepareOpen(const MonoCameraOpenParams &params)
{
    ResetOpenState();
    m_cam = params.camera;
    m_camIndex = params.camIndex;
    m_active.store(true, std::memory_order_relaxed);
    m_streamFault.store(false, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(m_callbackMu);
        m_closing = false;
        m_callbacksInFlight = 0;
    }
    if (!m_cam) {
        return false;
    }
    if (!m_cam->acquire()) {
        return true;
    }
    std::cerr << "Failed to acquire camera " << m_cam->id() << "\n";
    ResetOpenState();
    return false;
}

bool LibcameraMonoCam::ConfigureStream(const MonoCameraOpenParams &params)
{
    m_config = m_cam->generateConfiguration({libcamera::StreamRole::Viewfinder});
    if (!m_config || m_config->size() < 1) {
        std::cerr << "Failed to generate config\n";
        ResetOpenState();
        return false;
    }
    libcamera::StreamConfiguration &sc = m_config->at(0);
    sc.size.width = params.width;
    sc.size.height = params.height;
    if (params.requestY8) {
        sc.pixelFormat = libcamera::formats::R8;
    }
    return true;
}

void LibcameraMonoCam::ConfigureControls(const MonoCameraOpenParams &params, int64_t frameDurationUs,
                                         int32_t exposureCapUs)
{
    const float gainCap = kSlamAeMaxGain;
    m_aeConfiguredAuto = !params.aeDisable;
    if (params.aeDisable) {
        const int32_t manualExposureUs = std::max<int32_t>(1, std::min<int32_t>(params.exposureUs, exposureCapUs));
        const float manualGain = std::max(1.0f, std::min(params.gain, gainCap));
        m_controls.set(libcamera::controls::AeEnable, false);
        m_controls.set(libcamera::controls::ExposureTime, manualExposureUs);
        m_controls.set(libcamera::controls::AnalogueGain, manualGain);
    } else {
        m_controls.set(libcamera::controls::AeEnable, true);
        m_controls.set(libcamera::controls::ExposureTime, exposureCapUs);
        m_controls.set(libcamera::controls::AnalogueGain, std::max(1.0f, std::min(params.gain, gainCap)));
    }

    const bool nrSet = SetControlIfSupported(m_cam.get(), m_controls, "NoiseReductionMode", libcamera::ControlValue(1));
    const bool sharpSet = SetControlIfSupported(m_cam.get(), m_controls, "Sharpness", libcamera::ControlValue(0.0f));
    const bool tonemapSet = SetControlIfSupported(m_cam.get(), m_controls, "TonemapMode", libcamera::ControlValue(0));
    std::cerr << "[cam] isp_hint cam=" << m_camIndex << " ae=" << (params.aeDisable ? "manual" : "auto")
              << " frame_us=" << frameDurationUs << " exp_cap_us=" << exposureCapUs << " gain_cap=" << gainCap
              << " nr=" << (nrSet ? "set" : "n/a") << " sharp=" << (sharpSet ? "set" : "n/a")
              << " tonemap=" << (tonemapSet ? "set" : "n/a") << "\n";
}

bool LibcameraMonoCam::ConfigureCamera()
{
    if (m_config->validate() == libcamera::CameraConfiguration::Invalid) {
        std::cerr << "Invalid camera configuration\n";
        ResetOpenState();
        return false;
    }
    if (m_cam->configure(m_config.get())) {
        std::cerr << "Failed to configure camera\n";
        ResetOpenState();
        return false;
    }
    m_stream = m_config->at(0).stream();
    if (m_stream) {
        return true;
    }
    std::cerr << "No stream after configure\n";
    ResetOpenState();
    return false;
}

bool LibcameraMonoCam::Start()
{
    if (m_cam->start(&m_controls)) {
        std::cerr << "camera start failed\n";
        ResetOpenState();
        return false;
    }
    for (auto &r : m_requests) {
        if (m_cam->queueRequest(r.get()) < 0) {
            m_streamFault.store(true, std::memory_order_relaxed);
            std::cerr << "[cam] initial queueRequest failed cam=" << m_camIndex << "\n";
            ResetOpenState();
            return false;
        }
    }
    return true;
}

bool LibcameraMonoCam::MapBuffers(const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers)
{
    m_bufferMaps.clear();
    for (auto &buf : buffers) {
        std::vector<PlaneMap> planes;
        planes.reserve(buf->planes().size());
        for (const libcamera::FrameBuffer::Plane &p : buf->planes()) {
            const int fd = p.fd.get();
            const size_t len = p.length;
            const off_t off = static_cast<off_t>(p.offset);
            void *addr = MMapFD(fd, len, off);
            if (!addr) {
                std::cerr << "mmap failed\n";
                ResetOpenState();
                return false;
            }
            planes.push_back({addr, len, off});
        }
        m_bufferMaps[buf.get()] = std::move(planes);
    }
    return true;
}

bool LibcameraMonoCam::CreateRequests(const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers)
{
    m_requests.clear();
    for (auto &buf : buffers) {
        std::unique_ptr<libcamera::Request> req = m_cam->createRequest();
        if (!req || req->addBuffer(m_stream, buf.get()) < 0) {
            ResetOpenState();
            return false;
        }
        m_requests.push_back(std::move(req));
    }
    return true;
}

void LibcameraMonoCam::CreateFramePool(size_t slotCount)
{
    std::lock_guard<std::mutex> lock(m_framePoolMu);
    m_framePool = std::make_shared<FramePoolState>();
    m_framePool->active.store(true, std::memory_order_relaxed);
    m_framePool->slots.reserve(slotCount);
    for (size_t i = 0; i < slotCount; ++i) {
        m_framePool->slots.push_back(std::make_unique<FrameSlot>());
        m_framePool->freeSlots.push_back(m_framePool->slots.back().get());
    }
}

void LibcameraMonoCam::Stop()
{
    m_active.store(false, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(m_framePoolMu);
        if (m_framePool) {
            m_framePool->active.store(false, std::memory_order_relaxed);
        }
    }
    if (m_cam) {
        m_cam->stop();
    }
}

void LibcameraMonoCam::Close()
{
    m_active.store(false, std::memory_order_relaxed);
    SetSink(nullptr);
    {
        std::lock_guard<std::mutex> lock(m_callbackMu);
        m_closing = true;
    }

    if (m_cam) {
        m_cam->requestCompleted.disconnect(this, &LibcameraMonoCam::OnRequestComplete);
    }

    if (m_cam) {
        m_cam->stop();
    }

    WaitForCallbacks();

    for (auto &kv : m_bufferMaps) {
        for (auto &pm : kv.second) {
            MUnmap(pm.addr, pm.len);
        }
    }
    m_bufferMaps.clear();

    m_requests.clear();
    if (m_allocator && m_stream) {
        m_allocator->free(m_stream);
    }
    m_allocator.reset();
    m_config.reset();
    m_stream = nullptr;
    {
        std::lock_guard<std::mutex> lock(m_framePoolMu);
        if (m_framePool) {
            m_framePool->active.store(false, std::memory_order_relaxed);
            m_framePool->freeSlots.clear();
        }
        m_framePool.reset();
    }

    if (m_cam) {
        m_cam->release();
        m_cam.reset();
    }
}

void LibcameraMonoCam::SetSink(std::function<void(FrameItem &&)> sink)
{
    std::lock_guard<std::mutex> lock(m_sinkMu);
    m_sink = std::move(sink);
}

libcamera::PixelFormat LibcameraMonoCam::PixelFmt() const
{
    return m_config->at(0).pixelFormat;
}

libcamera::Size LibcameraMonoCam::SizeWH() const
{
    return m_config->at(0).size;
}

int LibcameraMonoCam::Stride() const
{
    return m_config->at(0).stride;
}

void LibcameraMonoCam::SetR16Normalize(bool on)
{
    m_r16Normalize = on;
}

bool LibcameraMonoCam::Healthy() const
{
    return !m_streamFault.load(std::memory_order_relaxed);
}

LibcameraMonoCam::CallbackScope::CallbackScope(LibcameraMonoCam *owner)
    : self(owner)
{
    if (!self) {
        return;
    }
    std::lock_guard<std::mutex> lock(self->m_callbackMu);
    if (self->m_closing) {
        return;
    }
    ++self->m_callbacksInFlight;
    armed = true;
}

LibcameraMonoCam::CallbackScope::~CallbackScope()
{
    if (!self || !armed) {
        return;
    }
    std::lock_guard<std::mutex> lock(self->m_callbackMu);
    --self->m_callbacksInFlight;
    if (self->m_callbacksInFlight == 0) {
        self->m_callbackCv.notify_all();
    }
}

bool LibcameraMonoCam::CallbackScope::Active() const
{
    return armed;
}

void LibcameraMonoCam::WaitForCallbacks()
{
    std::unique_lock<std::mutex> lock(m_callbackMu);
    while (m_callbacksInFlight != 0) {
        if (m_callbackCv.wait_for(lock, std::chrono::seconds(1), [this] { return m_callbacksInFlight == 0; })) {
            break;
        }
        std::cerr << "[cam] waiting callbacks cam=" << m_camIndex << " in_flight=" << m_callbacksInFlight << "\n";
    }
}

std::function<void(FrameItem &&)> LibcameraMonoCam::CopySink() const
{
    std::lock_guard<std::mutex> lock(m_sinkMu);
    return m_sink;
}

bool LibcameraMonoCam::RequeueRequest(libcamera::Request *req)
{
    if (!req || !m_cam) {
        return false;
    }
    req->reuse(libcamera::Request::ReuseBuffers);
    const int rc = m_cam->queueRequest(req);
    if (rc < 0) {
        const bool firstFault = !m_streamFault.exchange(true, std::memory_order_relaxed);
        m_active.store(false, std::memory_order_relaxed);
        if (firstFault) {
            std::cerr << "[cam] queueRequest failed cam=" << m_camIndex << " rc=" << rc << "\n";
        }
        return false;
    }
    return true;
}

void LibcameraMonoCam::OnRequestComplete(libcamera::Request *req)
{
    CallbackScope callbackScope(this);
    if (!callbackScope.Active() || !m_active.load(std::memory_order_relaxed) || !req ||
        req->status() == libcamera::Request::RequestCancelled) {
        return;
    }

    libcamera::FrameBuffer *buf = RequestBuffer(req);
    if (!buf) {
        RequeueIfActive(req);
        return;
    }
    const libcamera::FrameMetadata &md = buf->metadata();

    FrameItem item;
    item.camIndex = m_camIndex;
    item.arriveNs = NowNs();
    item.tsNs = md.timestamp;
    item.seq = md.sequence;

    LogIspMetadata(md, req->metadata());
    uint8_t *data = MappedBufferData(buf);
    if (!data) {
        RequeueIfActive(req);
        return;
    }

    auto frameSlot = AcquireFrameSlot();
    if (!frameSlot) {
        RequeueIfActive(req);
        return;
    }

    ConvertFrameToGray(m_config->at(0), data, md.sequence, frameSlot->gray);
    item.gray = frameSlot->gray;
    PublishFrame(std::move(item), std::move(frameSlot));
    RequeueIfActive(req);
}

void LibcameraMonoCam::LogIspMetadata(const libcamera::FrameMetadata &metadata,
                                      const libcamera::ControlList &meta) const
{
    if ((metadata.sequence % 120u) != 0u) {
        return;
    }
    const std::optional<int32_t> metaExposureUs = meta.get(libcamera::controls::ExposureTime);
    const std::optional<float> metaGain = meta.get(libcamera::controls::AnalogueGain);
    const std::optional<bool> metaAe = meta.get(libcamera::controls::AeEnable);
    const char *metaAeText = metaAe.has_value() ? (*metaAe ? "1" : "0") : "na";
    std::cerr << "[cam_isp] cam=" << m_camIndex << " seq=" << metadata.sequence
              << " ae_cfg=" << (m_aeConfiguredAuto ? "auto" : "manual") << " ae_meta=" << metaAeText
              << " exp_us=" << (metaExposureUs.has_value() ? *metaExposureUs : -1)
              << " gain=" << (metaGain.has_value() ? *metaGain : -1.0f) << "\n";
    if (metaExposureUs.has_value() && *metaExposureUs > kSlamAeMaxExposureUs) {
        std::cerr << "[cam_isp_warn] cam=" << m_camIndex << " seq=" << metadata.sequence
                  << " exposure high for slam exp_us=" << *metaExposureUs << " cap_us=" << kSlamAeMaxExposureUs
                  << "\n";
    }
    if (metaGain.has_value() && *metaGain > kSlamAeMaxGain) {
        std::cerr << "[cam_isp_warn] cam=" << m_camIndex << " seq=" << metadata.sequence
                  << " gain high for slam gain=" << *metaGain << " cap=" << kSlamAeMaxGain << "\n";
    }
}

libcamera::FrameBuffer *LibcameraMonoCam::RequestBuffer(libcamera::Request *req)
{
    auto it = req->buffers().find(m_stream);
    return it == req->buffers().end() ? nullptr : it->second;
}

uint8_t *LibcameraMonoCam::MappedBufferData(libcamera::FrameBuffer *buffer)
{
    auto mit = m_bufferMaps.find(buffer);
    if (mit == m_bufferMaps.end() || mit->second.empty() || !mit->second[0].addr) {
        return nullptr;
    }
    return reinterpret_cast<uint8_t *>(mit->second[0].addr);
}

void LibcameraMonoCam::ConvertFrameToGray(const libcamera::StreamConfiguration &config, uint8_t *data,
                                          uint32_t sequence, cv::Mat &gray8)
{
    const int w = config.size.width;
    const int h = config.size.height;
    const int stride = config.stride;
    if (config.pixelFormat == libcamera::formats::R8) {
        cv::Mat g(h, w, CV_8UC1, static_cast<void *>(data), static_cast<size_t>(stride));
        gray8.create(h, w, CV_8UC1);
        g.copyTo(gray8);
    } else if (config.pixelFormat == libcamera::formats::XRGB8888) {
        cv::Mat bgra(h, w, CV_8UC4, static_cast<void *>(data), static_cast<size_t>(stride));
        gray8.create(h, w, CV_8UC1);
        cv::cvtColor(bgra, gray8, cv::COLOR_BGRA2GRAY);
    } else if (config.pixelFormat == libcamera::formats::RGB888) {
        cv::Mat rgb(h, w, CV_8UC3, static_cast<void *>(data), static_cast<size_t>(stride));
        gray8.create(h, w, CV_8UC1);
        cv::cvtColor(rgb, gray8, cv::COLOR_RGB2GRAY);
    } else if (config.pixelFormat == libcamera::formats::R16) {
        cv::Mat m16(h, w, CV_16UC1, static_cast<void *>(data), static_cast<size_t>(stride));
        gray8.create(h, w, CV_8UC1);
        if (!m_r16Normalize) {
            m16.convertTo(gray8, CV_8U, 1.0 / 256.0);
        } else {
            CompressR16AdaptiveForSlam(m16, gray8, m_camIndex, sequence);
        }
    } else {
        cv::Mat g(h, w, CV_8UC1, static_cast<void *>(data), static_cast<size_t>(stride));
        gray8.create(h, w, CV_8UC1);
        g.copyTo(gray8);
    }
}

void LibcameraMonoCam::PublishFrame(FrameItem &&item, std::shared_ptr<FrameSlot> frameSlot)
{
    item.owner = std::shared_ptr<void>(std::move(frameSlot));
    auto sink = CopySink();
    if (sink) {
        sink(std::move(item));
    }
}

void LibcameraMonoCam::RequeueIfActive(libcamera::Request *req)
{
    if (m_active.load(std::memory_order_relaxed)) {
        RequeueRequest(req);
    }
}

std::shared_ptr<LibcameraMonoCam::FrameSlot> LibcameraMonoCam::AcquireFrameSlot()
{
    std::lock_guard<std::mutex> lock(m_framePoolMu);
    if (!m_framePool) {
        return {};
    }
    std::lock_guard<std::mutex> poolLock(m_framePool->mu);
    if (m_framePool->freeSlots.empty()) {
        return {};
    }
    FrameSlot *slot = m_framePool->freeSlots.front();
    m_framePool->freeSlots.pop_front();
    std::shared_ptr<FramePoolState> framePool = m_framePool;
    return std::shared_ptr<FrameSlot>(slot, [framePool](FrameSlot *s) {
        if (!framePool || !s || !framePool->active.load(std::memory_order_relaxed)) {
            return;
        }
        std::lock_guard<std::mutex> lock(framePool->mu);
        if (!framePool->active.load(std::memory_order_relaxed)) {
            return;
        }
        framePool->freeSlots.push_back(s);
    });
}

#include "stereo_ov9281_pairing.h"
