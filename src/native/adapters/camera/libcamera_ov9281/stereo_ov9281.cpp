#include "adapters/camera/libcamera_ov9281/stereo_ov9281.h"

#include <algorithm>
#include <chrono>
#include <iostream>

#include <sys/mman.h>
#include <time.h>

namespace {

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

int64_t Abs64(int64_t x) { return x < 0 ? -x : x; }

int64_t NowNs()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

} // namespace

std::atomic<bool> g_runningFlag{true};

void SigIntHandler(int) { g_runningFlag.store(false); }

bool LibcameraMonoCam::Open(std::shared_ptr<libcamera::Camera> cam, int camIndex, int w, int h, int fps, bool aeDisable,
                            int exposureUs, float gain, bool requestY8)
{
    m_cam = std::move(cam);
    m_camIndex = camIndex;
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
    if (m_cam->acquire()) {
        std::cerr << "Failed to acquire camera " << m_cam->id() << "\n";
        return false;
    }

    m_config = m_cam->generateConfiguration({libcamera::StreamRole::Viewfinder});
    if (!m_config || m_config->size() < 1) {
        std::cerr << "Failed to generate config\n";
        return false;
    }

    libcamera::StreamConfiguration &sc = m_config->at(0);
    sc.size.width = w;
    sc.size.height = h;
    if (requestY8) {
        sc.pixelFormat = libcamera::formats::R8;
    }

    const int64_t us = std::max<int64_t>(1, 1000000LL / std::max(1, fps));
    m_controls.set(libcamera::controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>({us, us}));

    if (aeDisable) {
        m_controls.set(libcamera::controls::AeEnable, false);
        m_controls.set(libcamera::controls::ExposureTime, exposureUs);
        m_controls.set(libcamera::controls::AnalogueGain, gain);
    }

    if (m_config->validate() == libcamera::CameraConfiguration::Invalid) {
        std::cerr << "Invalid camera configuration\n";
        return false;
    }

    if (m_cam->configure(m_config.get())) {
        std::cerr << "Failed to configure camera\n";
        return false;
    }

    m_stream = sc.stream();
    if (!m_stream) {
        std::cerr << "No stream after configure\n";
        return false;
    }

    m_allocator = std::make_unique<libcamera::FrameBufferAllocator>(m_cam);
    if (m_allocator->allocate(m_stream) < 0) {
        std::cerr << "Failed to allocate buffers\n";
        return false;
    }

    const auto &buffers = m_allocator->buffers(m_stream);
    if (buffers.empty()) {
        std::cerr << "No buffers allocated\n";
        return false;
    }

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
                return false;
            }
            planes.push_back({addr, len, off});
        }
        m_bufferMaps[buf.get()] = std::move(planes);
    }

    m_requests.clear();
    for (auto &buf : buffers) {
        std::unique_ptr<libcamera::Request> req = m_cam->createRequest();
        if (!req) {
            return false;
        }
        if (req->addBuffer(m_stream, buf.get()) < 0) {
            return false;
        }
        m_requests.push_back(std::move(req));
    }

    {
        std::lock_guard<std::mutex> lock(m_framePoolMu);
        m_framePool = std::make_shared<FramePoolState>();
        m_framePool->active.store(true, std::memory_order_relaxed);
        const size_t slotCount = buffers.size() + 2;
        m_framePool->slots.reserve(slotCount);
        for (size_t i = 0; i < slotCount; ++i) {
            m_framePool->slots.push_back(std::make_unique<FrameSlot>());
            m_framePool->freeSlots.push_back(m_framePool->slots.back().get());
        }
    }

    m_cam->requestCompleted.connect(this, &LibcameraMonoCam::OnRequestComplete);
    return true;
}

bool LibcameraMonoCam::Start()
{
    if (m_cam->start(&m_controls)) {
        std::cerr << "camera start failed\n";
        return false;
    }
    for (auto &r : m_requests) {
        if (m_cam->queueRequest(r.get()) < 0) {
            m_streamFault.store(true, std::memory_order_relaxed);
            std::cerr << "[cam] initial queueRequest failed cam=" << m_camIndex << "\n";
            return false;
        }
    }
    return true;
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

libcamera::PixelFormat LibcameraMonoCam::PixelFmt() const { return m_config->at(0).pixelFormat; }

libcamera::Size LibcameraMonoCam::SizeWH() const { return m_config->at(0).size; }

int LibcameraMonoCam::Stride() const { return m_config->at(0).stride; }

void LibcameraMonoCam::SetR16Normalize(bool on) { m_r16Normalize = on; }

bool LibcameraMonoCam::Healthy() const { return !m_streamFault.load(std::memory_order_relaxed); }

LibcameraMonoCam::CallbackScope::CallbackScope(LibcameraMonoCam *owner) : self(owner)
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

bool LibcameraMonoCam::CallbackScope::Active() const { return armed; }

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
    if (!callbackScope.Active()) {
        return;
    }
    if (!m_active.load(std::memory_order_relaxed)) {
        return;
    }
    if (!req || req->status() == libcamera::Request::RequestCancelled) {
        return;
    }

    auto it = req->buffers().find(m_stream);
    if (it == req->buffers().end()) {
        if (m_active.load(std::memory_order_relaxed)) {
            RequeueRequest(req);
        }
        return;
    }
    libcamera::FrameBuffer *buf = it->second;
    const libcamera::FrameMetadata &md = buf->metadata();

    FrameItem item;
    item.camIndex = m_camIndex;
    item.arriveNs = NowNs();
    item.tsNs = md.timestamp;
    item.seq = md.sequence;

    const libcamera::StreamConfiguration &sc = m_config->at(0);
    const int w = sc.size.width;
    const int h = sc.size.height;
    const int stride = sc.stride;

    auto mit = m_bufferMaps.find(buf);
    if (mit == m_bufferMaps.end() || mit->second.empty() || !mit->second[0].addr) {
        if (m_active.load(std::memory_order_relaxed)) {
            RequeueRequest(req);
        }
        return;
    }
    uint8_t *p0 = reinterpret_cast<uint8_t *>(mit->second[0].addr);

    auto frameSlot = AcquireFrameSlot();
    if (!frameSlot) {
        if (m_active.load(std::memory_order_relaxed)) {
            RequeueRequest(req);
        }
        return;
    }
    cv::Mat &gray8 = frameSlot->gray;
    if (sc.pixelFormat == libcamera::formats::R8) {
        cv::Mat g(h, w, CV_8UC1, static_cast<void *>(p0), static_cast<size_t>(stride));
        gray8.create(h, w, CV_8UC1);
        g.copyTo(gray8);
    } else if (sc.pixelFormat == libcamera::formats::XRGB8888) {
        cv::Mat bgra(h, w, CV_8UC4, static_cast<void *>(p0), static_cast<size_t>(stride));
        gray8.create(h, w, CV_8UC1);
        cv::cvtColor(bgra, gray8, cv::COLOR_BGRA2GRAY);
    } else if (sc.pixelFormat == libcamera::formats::RGB888) {
        cv::Mat rgb(h, w, CV_8UC3, static_cast<void *>(p0), static_cast<size_t>(stride));
        gray8.create(h, w, CV_8UC1);
        cv::cvtColor(rgb, gray8, cv::COLOR_RGB2GRAY);
    } else if (sc.pixelFormat == libcamera::formats::R16) {
        cv::Mat m16(h, w, CV_16UC1, static_cast<void *>(p0), static_cast<size_t>(stride));
        gray8.create(h, w, CV_8UC1);
        if (!m_r16Normalize) {
            m16.convertTo(gray8, CV_8U, 1.0 / 256.0);
        } else {
            double minv = 0;
            double maxv = 0;
            cv::minMaxLoc(m16, &minv, &maxv);
            const double scale = (maxv > minv) ? (255.0 / (maxv - minv)) : 1.0;
            const double shift = -minv * scale;
            m16.convertTo(gray8, CV_8U, scale, shift);
        }
    } else {
        cv::Mat g(h, w, CV_8UC1, static_cast<void *>(p0), static_cast<size_t>(stride));
        gray8.create(h, w, CV_8UC1);
        g.copyTo(gray8);
    }

    item.gray = gray8;
    item.owner = std::shared_ptr<void>(std::move(frameSlot));
    auto sink = CopySink();
    if (sink) {
        sink(std::move(item));
    }

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

bool LibcameraStereoOV9281_TsPair::Open(int w, int h, int fps, bool aeDisable, int exposureUs, float gain,
                                        bool requestY8, int64_t pairThreshNs, int64_t keepWindowNs, int maxPairQueue,
                                        bool r16Normalize, int leftCamIndex, int rightCamIndex)
{
    m_w = w;
    m_h = h;
    m_fps = fps;
    m_maxPairQueue = maxPairQueue;
    m_pairThreshNs = pairThreshNs;
    m_keepWindowNs = keepWindowNs;
    m_acceptFrames.store(true, std::memory_order_relaxed);

    m_cm = std::make_unique<libcamera::CameraManager>();
    if (m_cm->start()) {
        std::cerr << "CameraManager start failed\n";
        return false;
    }

    const auto &cams = m_cm->cameras();
    if (cams.size() < 2) {
        std::cerr << "Need 2 cameras, but found " << cams.size() << "\n";
        return false;
    }

    const int camCount = static_cast<int>(cams.size());
    if (leftCamIndex < 0 || leftCamIndex >= camCount || rightCamIndex < 0 || rightCamIndex >= camCount) {
        std::cerr << "[cam] invalid camera index left=" << leftCamIndex << " right=" << rightCamIndex
                  << " available=" << camCount << "\n";
        for (int i = 0; i < camCount; ++i) {
            std::cerr << "[cam] available index=" << i << " id=" << cams[static_cast<size_t>(i)]->id() << "\n";
        }
        return false;
    }
    if (leftCamIndex == rightCamIndex) {
        std::cerr << "[cam] left and right camera index must differ, got " << leftCamIndex << "\n";
        return false;
    }

    m_camL = cams[static_cast<size_t>(leftCamIndex)];
    m_camR = cams[static_cast<size_t>(rightCamIndex)];

    auto sink = [&](FrameItem &&fi) { PushFrame(std::move(fi)); };

    if (!m_left.Open(m_camL, 0, w, h, fps, aeDisable, exposureUs, gain, requestY8)) {
        return false;
    }
    if (!m_right.Open(m_camR, 1, w, h, fps, aeDisable, exposureUs, gain, requestY8)) {
        return false;
    }

    m_left.SetR16Normalize(r16Normalize);
    m_right.SetR16Normalize(r16Normalize);
    m_left.SetSink(sink);
    m_right.SetSink(sink);

    if (!m_left.Start()) {
        return false;
    }
    if (!m_right.Start()) {
        return false;
    }

    std::cerr << "Left fmt=" << m_left.PixelFmt().toString() << " size=" << m_left.SizeWH().toString()
              << " stride=" << m_left.Stride() << "\n";
    std::cerr << "Right fmt=" << m_right.PixelFmt().toString() << " size=" << m_right.SizeWH().toString()
              << " stride=" << m_right.Stride() << "\n";
    return true;
}

void LibcameraStereoOV9281_TsPair::Close()
{
    m_acceptFrames.store(false, std::memory_order_relaxed);
    m_left.SetSink(nullptr);
    m_right.SetSink(nullptr);
    m_left.Stop();
    m_right.Stop();

    {
        std::lock_guard<std::mutex> lk(m_muPair);
        m_qL.clear();
        m_qR.clear();
        m_paired.clear();
    }
    m_cvPair.notify_all();

    m_left.Close();
    m_right.Close();
    m_camL.reset();
    m_camR.reset();
    if (m_cm) {
        m_cm->stop();
    }
    m_cm.reset();
}

bool LibcameraStereoOV9281_TsPair::GrabPair(FrameItem &L, FrameItem &R, int timeoutMs, bool preferLatest,
                                            uint64_t minTimestampNs)
{
    std::unique_lock<std::mutex> lk(m_muPair);
    if (!m_cvPair.wait_for(lk, std::chrono::milliseconds(timeoutMs), [&] {
            return HasEligiblePairLocked(minTimestampNs) || !g_runningFlag.load() ||
                   !m_acceptFrames.load(std::memory_order_relaxed);
        })) {
        return false;
    }
    if (!HasEligiblePairLocked(minTimestampNs)) {
        return false;
    }

    size_t selectedIndex = 0;
    if (minTimestampNs > 0) {
        selectedIndex = m_paired.size();
        for (size_t i = 0; i < m_paired.size(); ++i) {
            if (PairTimestampNs(m_paired[i]) >= minTimestampNs) {
                selectedIndex = i;
                if (!preferLatest) {
                    break;
                }
            }
        }
        if (selectedIndex >= m_paired.size()) {
            return false;
        }
    } else if (preferLatest && m_paired.size() > 1) {
        selectedIndex = m_paired.size() - 1;
    }

    if (selectedIndex > 0) {
        m_droppedPaired.fetch_add(selectedIndex, std::memory_order_relaxed);
        for (size_t i = 0; i < selectedIndex; ++i) {
            m_paired.pop_front();
        }
    }

    auto &p = m_paired.front();
    L = std::move(p.first);
    R = std::move(p.second);
    m_paired.pop_front();
    return true;
}

int64_t LibcameraStereoOV9281_TsPair::LastDtMs() const { return m_lastDtMs.load(); }

uint32_t LibcameraStereoOV9281_TsPair::LastSeq() const { return m_lastSeq.load(); }

uint32_t LibcameraStereoOV9281_TsPair::LastRawSeqL() const { return m_lastRawSeq[0].load(std::memory_order_relaxed); }

uint32_t LibcameraStereoOV9281_TsPair::LastRawSeqR() const { return m_lastRawSeq[1].load(std::memory_order_relaxed); }

uint64_t LibcameraStereoOV9281_TsPair::RawCountL() const { return m_rawFrameCount[0].load(std::memory_order_relaxed); }

uint64_t LibcameraStereoOV9281_TsPair::RawCountR() const { return m_rawFrameCount[1].load(std::memory_order_relaxed); }

int64_t LibcameraStereoOV9281_TsPair::LastRejectDtUs() const
{
    return m_lastRejectDtNs.load(std::memory_order_relaxed) / 1000;
}

uint64_t LibcameraStereoOV9281_TsPair::DroppedUnpairedL() const
{
    return m_droppedUnpaired[0].load(std::memory_order_relaxed);
}

uint64_t LibcameraStereoOV9281_TsPair::DroppedUnpairedR() const
{
    return m_droppedUnpaired[1].load(std::memory_order_relaxed);
}

size_t LibcameraStereoOV9281_TsPair::PendL() const
{
    std::lock_guard<std::mutex> lk(m_muPair);
    return m_qL.size();
}

size_t LibcameraStereoOV9281_TsPair::PendR() const
{
    std::lock_guard<std::mutex> lk(m_muPair);
    return m_qR.size();
}

int64_t LibcameraStereoOV9281_TsPair::PairTolNs() const { return m_pairThreshNs; }

uint64_t LibcameraStereoOV9281_TsPair::DroppedPaired() const { return m_droppedPaired.load(std::memory_order_relaxed); }

bool LibcameraStereoOV9281_TsPair::Healthy() const { return m_left.Healthy() && m_right.Healthy(); }

uint64_t LibcameraStereoOV9281_TsPair::PairTimestampNs(const std::pair<FrameItem, FrameItem> &pair) const
{
    return (pair.first.tsNs + pair.second.tsNs) / 2ULL;
}

bool LibcameraStereoOV9281_TsPair::HasEligiblePairLocked(uint64_t minTimestampNs) const
{
    if (m_paired.empty()) {
        return false;
    }
    if (minTimestampNs == 0) {
        return true;
    }
    for (const auto &pair : m_paired) {
        if (PairTimestampNs(pair) >= minTimestampNs) {
            return true;
        }
    }
    return false;
}

void LibcameraStereoOV9281_TsPair::PushFrame(FrameItem &&fi)
{
    if (!m_acceptFrames.load(std::memory_order_relaxed)) {
        return;
    }
    std::lock_guard<std::mutex> lk(m_muPair);
    if (!m_acceptFrames.load(std::memory_order_relaxed)) {
        return;
    }
    OnFrameLocked(std::move(fi));
}

bool LibcameraStereoOV9281_TsPair::TryPairLocked()
{
    if (m_qL.empty() || m_qR.empty()) {
        return false;
    }

    const size_t bestRightForLeft = FindBestMatchIndex(m_qR, m_qL.front().tsNs);
    const int64_t bestLeftDt =
        (bestRightForLeft < m_qR.size())
            ? Abs64(static_cast<int64_t>(m_qR[bestRightForLeft].tsNs) - static_cast<int64_t>(m_qL.front().tsNs))
            : INT64_MAX;
    const size_t bestLeftForRight = FindBestMatchIndex(m_qL, m_qR.front().tsNs);
    const int64_t bestRightDt =
        (bestLeftForRight < m_qL.size())
            ? Abs64(static_cast<int64_t>(m_qL[bestLeftForRight].tsNs) - static_cast<int64_t>(m_qR.front().tsNs))
            : INT64_MAX;

    const bool pairFromLeft = bestLeftDt <= bestRightDt;
    const int64_t bestDt = pairFromLeft ? bestLeftDt : bestRightDt;
    if (bestDt > m_pairThreshNs) {
        m_lastRejectDtNs.store(bestDt == INT64_MAX ? -1 : bestDt, std::memory_order_relaxed);
        if (m_qL.front().tsNs <= m_qR.front().tsNs) {
            m_droppedUnpaired[0].fetch_add(1, std::memory_order_relaxed);
            m_qL.pop_front();
        } else {
            m_droppedUnpaired[1].fetch_add(1, std::memory_order_relaxed);
            m_qR.pop_front();
        }
        return true;
    }

    FrameItem L;
    FrameItem R;
    if (pairFromLeft) {
        L = std::move(m_qL.front());
        R = std::move(m_qR[bestRightForLeft]);
        m_qL.pop_front();
        m_qR.erase(m_qR.begin() + static_cast<std::ptrdiff_t>(bestRightForLeft));
    } else {
        L = std::move(m_qL[bestLeftForRight]);
        R = std::move(m_qR.front());
        m_qL.erase(m_qL.begin() + static_cast<std::ptrdiff_t>(bestLeftForRight));
        m_qR.pop_front();
    }

    m_lastDtMs.store((static_cast<int64_t>(R.tsNs) - static_cast<int64_t>(L.tsNs)) / 1'000'000);
    m_lastSeq.store(L.seq);

    m_paired.push_back({std::move(L), std::move(R)});
    while (static_cast<int>(m_paired.size()) > m_maxPairQueue) {
        m_paired.pop_front();
        m_droppedPaired.fetch_add(1, std::memory_order_relaxed);
    }
    m_cvPair.notify_one();
    return true;
}

size_t LibcameraStereoOV9281_TsPair::FindBestMatchIndex(const std::deque<FrameItem> &q, uint64_t targetTs) const
{
    const size_t limit = std::min(kPairLookahead, q.size());
    size_t bestIdx = q.size();
    int64_t bestDt = INT64_MAX;
    for (size_t i = 0; i < limit; ++i) {
        const int64_t dt = Abs64(static_cast<int64_t>(q[i].tsNs) - static_cast<int64_t>(targetTs));
        if (dt < bestDt) {
            bestDt = dt;
            bestIdx = i;
        }
    }
    return bestIdx;
}

void LibcameraStereoOV9281_TsPair::PurgeOldLocked()
{
    uint64_t newest = 0;
    if (!m_qL.empty()) {
        newest = std::max<uint64_t>(newest, m_qL.back().tsNs);
    }
    if (!m_qR.empty()) {
        newest = std::max<uint64_t>(newest, m_qR.back().tsNs);
    }

    auto purge = [&](std::deque<FrameItem> &q) {
        while (!q.empty() && static_cast<int64_t>(newest - q.front().tsNs) > m_keepWindowNs) {
            q.pop_front();
        }
    };
    purge(m_qL);
    purge(m_qR);
}

void LibcameraStereoOV9281_TsPair::OnFrameLocked(FrameItem &&fi)
{
    if (fi.camIndex >= 0 && fi.camIndex < 2) {
        m_lastRawSeq[fi.camIndex].store(fi.seq, std::memory_order_relaxed);
        m_rawFrameCount[fi.camIndex].fetch_add(1, std::memory_order_relaxed);
    }
    if (fi.camIndex == 0) {
        m_qL.push_back(std::move(fi));
    } else {
        m_qR.push_back(std::move(fi));
    }

    PurgeOldLocked();
    while (TryPairLocked()) {
    }
}
