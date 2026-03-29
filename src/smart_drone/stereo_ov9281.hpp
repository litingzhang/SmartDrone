#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <sys/mman.h>
#include <time.h>

#include <opencv2/opencv.hpp>

#include <libcamera/framebuffer_allocator.h>
#include <libcamera/libcamera.h>

using namespace libcamera;

static std::atomic<bool> g_runningFlag{true};
static void SigIntHandler(int) { g_runningFlag.store(false); }

struct FrameItem {
    int camIndex{-1};
    uint64_t tsNs{0};
    uint32_t seq{0};
    int64_t arriveNs{0};
    cv::Mat gray;
    std::shared_ptr<void> owner;
};

struct PlaneMap {
    void *addr{nullptr};
    size_t len{0};
    off_t off{0};
};

static void *MMapFD(int fd, size_t len, off_t off = 0)
{
    void *p = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, off);
    if (p == MAP_FAILED)
        return nullptr;
    return p;
}

static void MUnmap(void *p, size_t len)
{
    if (p && p != MAP_FAILED)
        munmap(p, len);
}

static inline int64_t Abs64(int64_t x) { return x < 0 ? -x : x; }

static int64_t Median(std::vector<int64_t> v)
{
    if (v.empty())
        return 0;
    const size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    return v[mid];
}

static int64_t NowNs()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

class LibcameraMonoCam {
  public:
    bool Open(std::shared_ptr<Camera> cam, int camIndex, int w, int h, int fps, bool aeDisable,
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

        if (!m_cam)
            return false;
        if (m_cam->acquire()) {
            std::cerr << "Failed to acquire camera " << m_cam->id() << "\n";
            return false;
        }

        m_config = m_cam->generateConfiguration({StreamRole::Viewfinder});
        if (!m_config || m_config->size() < 1) {
            std::cerr << "Failed to generate config\n";
            return false;
        }

        StreamConfiguration &sc = m_config->at(0);
        sc.size.width = w;
        sc.size.height = h;
        if (requestY8)
            sc.pixelFormat = formats::R8;

        const int64_t us = std::max<int64_t>(1, 1000000LL / std::max(1, fps));
        m_controls.set(controls::FrameDurationLimits, Span<const int64_t, 2>({us, us}));

        if (aeDisable) {
            m_controls.set(controls::AeEnable, false);
            m_controls.set(controls::ExposureTime, exposureUs);
            m_controls.set(controls::AnalogueGain, gain);
        }

        if (m_config->validate() == CameraConfiguration::Invalid) {
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

        m_allocator = std::make_unique<FrameBufferAllocator>(m_cam);
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
            for (const FrameBuffer::Plane &p : buf->planes()) {
                const int fd = p.fd.get();
                const size_t len = p.length;
                const off_t off = (off_t)p.offset;
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
            std::unique_ptr<Request> req = m_cam->createRequest();
            if (!req)
                return false;
            if (req->addBuffer(m_stream, buf.get()) < 0)
                return false;
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

    bool Start()
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

    void Stop()
    {
        m_active.store(false, std::memory_order_relaxed);
        {
            std::lock_guard<std::mutex> lock(m_framePoolMu);
            if (m_framePool)
                m_framePool->active.store(false, std::memory_order_relaxed);
        }
        if (m_cam)
            m_cam->stop();
    }

    void Close()
    {
        m_active.store(false, std::memory_order_relaxed);
        SetSink(nullptr);
        {
            std::lock_guard<std::mutex> lock(m_callbackMu);
            m_closing = true;
        }

        if (m_cam)
            m_cam->requestCompleted.disconnect(this, &LibcameraMonoCam::OnRequestComplete);

        if (m_cam)
            m_cam->stop();

        WaitForCallbacks();

        for (auto &kv : m_bufferMaps)
            for (auto &pm : kv.second)
                MUnmap(pm.addr, pm.len);
        m_bufferMaps.clear();

        m_requests.clear();
        if (m_allocator && m_stream)
            m_allocator->free(m_stream);
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

    void SetSink(std::function<void(FrameItem &&)> sink)
    {
        std::lock_guard<std::mutex> lock(m_sinkMu);
        m_sink = std::move(sink);
    }
    PixelFormat PixelFmt() const { return m_config->at(0).pixelFormat; }
    Size SizeWH() const { return m_config->at(0).size; }
    int Stride() const { return m_config->at(0).stride; }
    void SetR16Normalize(bool on) { m_r16Normalize = on; }
    bool Healthy() const { return !m_streamFault.load(std::memory_order_relaxed); }

  private:
    struct CallbackScope {
        LibcameraMonoCam* self{nullptr};
        bool armed{false};

        explicit CallbackScope(LibcameraMonoCam* owner) : self(owner)
        {
            if (!self)
                return;
            std::lock_guard<std::mutex> lock(self->m_callbackMu);
            if (self->m_closing)
                return;
            ++self->m_callbacksInFlight;
            armed = true;
        }

        ~CallbackScope()
        {
            if (!self || !armed)
                return;
            std::lock_guard<std::mutex> lock(self->m_callbackMu);
            --self->m_callbacksInFlight;
            if (self->m_callbacksInFlight == 0)
                self->m_callbackCv.notify_all();
        }

        bool Active() const { return armed; }
    };

    void WaitForCallbacks()
    {
        std::unique_lock<std::mutex> lock(m_callbackMu);
        while (m_callbacksInFlight != 0) {
            if (m_callbackCv.wait_for(lock, std::chrono::seconds(1),
                                      [this] { return m_callbacksInFlight == 0; })) {
                break;
            }
            std::cerr << "[cam] waiting callbacks cam=" << m_camIndex
                      << " in_flight=" << m_callbacksInFlight << "\n";
        }
    }

    std::function<void(FrameItem &&)> CopySink() const
    {
        std::lock_guard<std::mutex> lock(m_sinkMu);
        return m_sink;
    }

    bool RequeueRequest(Request *req)
    {
        if (!req || !m_cam)
            return false;
        req->reuse(Request::ReuseBuffers);
        const int rc = m_cam->queueRequest(req);
        if (rc < 0) {
            const bool firstFault = !m_streamFault.exchange(true, std::memory_order_relaxed);
            m_active.store(false, std::memory_order_relaxed);
            if (firstFault) {
                std::cerr << "[cam] queueRequest failed cam=" << m_camIndex
                          << " rc=" << rc << "\n";
            }
            return false;
        }
        return true;
    }

    void OnRequestComplete(Request *req)
    {
        CallbackScope callbackScope(this);
        if (!callbackScope.Active())
            return;
        if (!m_active.load(std::memory_order_relaxed))
            return;
        if (!req || req->status() == Request::RequestCancelled)
            return;

        auto it = req->buffers().find(m_stream);
        if (it == req->buffers().end()) {
            if (m_active.load(std::memory_order_relaxed))
                RequeueRequest(req);
            return;
        }
        FrameBuffer *buf = it->second;
        const FrameMetadata &md = buf->metadata();

        FrameItem item;
        item.camIndex = m_camIndex;
        item.arriveNs = NowNs();
        item.tsNs = md.timestamp;
        item.seq = md.sequence;

        const StreamConfiguration &sc = m_config->at(0);
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
        cv::Mat& gray8 = frameSlot->gray;
        if (sc.pixelFormat == formats::R8) {
            cv::Mat g(h, w, CV_8UC1, (void *)p0, (size_t)stride);
            gray8.create(h, w, CV_8UC1);
            g.copyTo(gray8);
        } else if (sc.pixelFormat == formats::XRGB8888) {
            cv::Mat bgra(h, w, CV_8UC4, (void *)p0, (size_t)stride);
            gray8.create(h, w, CV_8UC1);
            cv::cvtColor(bgra, gray8, cv::COLOR_BGRA2GRAY);
        } else if (sc.pixelFormat == formats::RGB888) {
            cv::Mat rgb(h, w, CV_8UC3, (void *)p0, (size_t)stride);
            gray8.create(h, w, CV_8UC1);
            cv::cvtColor(rgb, gray8, cv::COLOR_RGB2GRAY);
        } else if (sc.pixelFormat == formats::R16) {
            cv::Mat m16(h, w, CV_16UC1, (void *)p0, (size_t)stride);
            gray8.create(h, w, CV_8UC1);
            if (!m_r16Normalize) {
                m16.convertTo(gray8, CV_8U, 1.0 / 256.0);
            } else {
                double minv = 0, maxv = 0;
                cv::minMaxLoc(m16, &minv, &maxv);
                const double scale = (maxv > minv) ? (255.0 / (maxv - minv)) : 1.0;
                const double shift = -minv * scale;
                m16.convertTo(gray8, CV_8U, scale, shift);
            }
        } else {
            cv::Mat g(h, w, CV_8UC1, (void *)p0, (size_t)stride);
            gray8.create(h, w, CV_8UC1);
            g.copyTo(gray8);
        }

        item.gray = gray8;
        item.owner = std::shared_ptr<void>(std::move(frameSlot));
        auto sink = CopySink();
        if (sink)
            sink(std::move(item));

        if (m_active.load(std::memory_order_relaxed)) {
            RequeueRequest(req);
        }
    }

    struct FrameSlot {
        cv::Mat gray;
    };

    struct FramePoolState {
        std::atomic<bool> active{false};
        std::vector<std::unique_ptr<FrameSlot>> slots;
        std::deque<FrameSlot*> freeSlots;
        std::mutex mu;
    };

    std::shared_ptr<FrameSlot> AcquireFrameSlot()
    {
        std::lock_guard<std::mutex> lock(m_framePoolMu);
        if (!m_framePool) {
            return {};
        }
        std::lock_guard<std::mutex> poolLock(m_framePool->mu);
        if (m_framePool->freeSlots.empty()) {
            return {};
        }
        FrameSlot* slot = m_framePool->freeSlots.front();
        m_framePool->freeSlots.pop_front();
        std::shared_ptr<FramePoolState> framePool = m_framePool;
        return std::shared_ptr<FrameSlot>(slot, [framePool](FrameSlot* s) {
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

    std::shared_ptr<Camera> m_cam;
    int m_camIndex{-1};
    std::unique_ptr<CameraConfiguration> m_config;
    Stream *m_stream{nullptr};
    ControlList m_controls{controls::controls};
    std::unique_ptr<FrameBufferAllocator> m_allocator;
    std::vector<std::unique_ptr<Request>> m_requests;
    std::map<FrameBuffer *, std::vector<PlaneMap>> m_bufferMaps;
    std::function<void(FrameItem &&)> m_sink;
    mutable std::mutex m_sinkMu;
    bool m_r16Normalize{false};
    std::atomic<bool> m_active{false};
    std::atomic<bool> m_streamFault{false};
    std::mutex m_framePoolMu;
    std::shared_ptr<FramePoolState> m_framePool;
    std::mutex m_callbackMu;
    std::condition_variable m_callbackCv;
    size_t m_callbacksInFlight{0};
    bool m_closing{false};
};

class LibcameraStereoOV9281_TsPair {
  public:
    static constexpr size_t kPairLookahead = 3;

    bool Open(int w, int h, int fps, bool aeDisable, int exposureUs, float gain, bool requestY8,
              int64_t pair_thresh_ns, int64_t keepWindowNs, int maxPairQueue = 8,
              bool r16_normalize = false, int leftCamIndex = 0, int rightCamIndex = 1)
    {
        m_w = w;
        m_h = h;
        m_fps = fps;
        m_maxPairQueue = maxPairQueue;
        m_pairThreshNs = pair_thresh_ns;
        m_keepWindowNs = keepWindowNs;
        m_acceptFrames.store(true, std::memory_order_relaxed);

        m_cm = std::make_unique<CameraManager>();
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
        if (leftCamIndex < 0 || leftCamIndex >= camCount ||
            rightCamIndex < 0 || rightCamIndex >= camCount) {
            std::cerr << "[cam] invalid camera index left=" << leftCamIndex
                      << " right=" << rightCamIndex
                      << " available=" << camCount << "\n";
            for (int i = 0; i < camCount; ++i) {
                std::cerr << "[cam] available index=" << i
                          << " id=" << cams[static_cast<size_t>(i)]->id() << "\n";
            }
            return false;
        }
        if (leftCamIndex == rightCamIndex) {
            std::cerr << "[cam] left and right camera index must differ, got "
                      << leftCamIndex << "\n";
            return false;
        }

        m_camL = cams[static_cast<size_t>(leftCamIndex)];
        m_camR = cams[static_cast<size_t>(rightCamIndex)];

        auto sink = [&](FrameItem &&fi) { PushFrame(std::move(fi)); };

        if (!m_left.Open(m_camL, 0, w, h, fps, aeDisable, exposureUs, gain, requestY8))
            return false;
        if (!m_right.Open(m_camR, 1, w, h, fps, aeDisable, exposureUs, gain, requestY8))
            return false;

        m_left.SetR16Normalize(r16_normalize);
        m_right.SetR16Normalize(r16_normalize);
        m_left.SetSink(sink);
        m_right.SetSink(sink);

        if (!m_left.Start())
            return false;
        if (!m_right.Start())
            return false;

        std::cerr << "Left fmt=" << m_left.PixelFmt().toString()
                  << " size=" << m_left.SizeWH().toString() << " stride=" << m_left.Stride() << "\n";
        std::cerr << "Right fmt=" << m_right.PixelFmt().toString()
                  << " size=" << m_right.SizeWH().toString() << " stride=" << m_right.Stride()
                  << "\n";
        return true;
    }

    void Close()
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
        if (m_cm)
            m_cm->stop();
        m_cm.reset();
    }

    bool GrabPair(FrameItem &L, FrameItem &R, int timeoutMs = 1000, bool preferLatest = false)
    {
        std::unique_lock<std::mutex> lk(m_muPair);
        if (!m_cvPair.wait_for(lk, std::chrono::milliseconds(timeoutMs),
                               [&] {
                                   return !m_paired.empty() || !g_runningFlag.load() ||
                                          !m_acceptFrames.load(std::memory_order_relaxed);
                               })) {
            return false;
        }
        if (m_paired.empty())
            return false;
        if (preferLatest && m_paired.size() > 1) {
            const size_t staleCount = m_paired.size() - 1;
            for (size_t i = 0; i < staleCount; ++i) {
                m_paired.pop_front();
            }
            m_droppedPaired.fetch_add(staleCount, std::memory_order_relaxed);
        }
        auto &p = m_paired.front();
        L = std::move(p.first);
        R = std::move(p.second);
        m_paired.pop_front();
        return true;
    }

    int64_t LastDtMs() const { return m_lastDtMs.load(); }
    uint32_t LastSeq() const { return m_lastSeq.load(); }
    uint32_t LastRawSeqL() const { return m_lastRawSeq[0].load(std::memory_order_relaxed); }
    uint32_t LastRawSeqR() const { return m_lastRawSeq[1].load(std::memory_order_relaxed); }
    uint64_t RawCountL() const { return m_rawFrameCount[0].load(std::memory_order_relaxed); }
    uint64_t RawCountR() const { return m_rawFrameCount[1].load(std::memory_order_relaxed); }
    int64_t LastRejectDtUs() const { return m_lastRejectDtNs.load(std::memory_order_relaxed) / 1000; }
    uint64_t DroppedUnpairedL() const { return m_droppedUnpaired[0].load(std::memory_order_relaxed); }
    uint64_t DroppedUnpairedR() const { return m_droppedUnpaired[1].load(std::memory_order_relaxed); }
    size_t PendL() const
    {
        std::lock_guard<std::mutex> lk(m_muPair);
        return m_qL.size();
    }
    size_t PendR() const
    {
        std::lock_guard<std::mutex> lk(m_muPair);
        return m_qR.size();
    }
    int64_t PairTolNs() const { return m_pairThreshNs; }
    uint64_t DroppedPaired() const { return m_droppedPaired.load(std::memory_order_relaxed); }
    bool Healthy() const { return m_left.Healthy() && m_right.Healthy(); }

  private:
    void PushFrame(FrameItem &&fi)
    {
        if (!m_acceptFrames.load(std::memory_order_relaxed))
            return;
        std::lock_guard<std::mutex> lk(m_muPair);
        if (!m_acceptFrames.load(std::memory_order_relaxed))
            return;
        OnFrameLocked(std::move(fi));
    }

    bool TryPairLocked()
    {
        if (m_qL.empty() || m_qR.empty())
            return false;

        const size_t bestRightForLeft = FindBestMatchIndex(m_qR, m_qL.front().tsNs);
        const int64_t bestLeftDt = (bestRightForLeft < m_qR.size())
            ? Abs64(static_cast<int64_t>(m_qR[bestRightForLeft].tsNs) - static_cast<int64_t>(m_qL.front().tsNs))
            : INT64_MAX;
        const size_t bestLeftForRight = FindBestMatchIndex(m_qL, m_qR.front().tsNs);
        const int64_t bestRightDt = (bestLeftForRight < m_qL.size())
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

        m_lastDtMs.store(((int64_t)R.tsNs - (int64_t)L.tsNs) / 1'000'000);
        m_lastSeq.store(L.seq);

        m_paired.push_back({std::move(L), std::move(R)});
        while ((int)m_paired.size() > m_maxPairQueue) {
            m_paired.pop_front();
            m_droppedPaired.fetch_add(1, std::memory_order_relaxed);
        }
        m_cvPair.notify_one();
        return true;
    }

    size_t FindBestMatchIndex(const std::deque<FrameItem>& q, uint64_t targetTs) const
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

    void PurgeOldLocked()
    {
        uint64_t newest = 0;
        if (!m_qL.empty())
            newest = std::max<uint64_t>(newest, m_qL.back().tsNs);
        if (!m_qR.empty())
            newest = std::max<uint64_t>(newest, m_qR.back().tsNs);

        auto purge = [&](std::deque<FrameItem> &q) {
            while (!q.empty() && (int64_t)(newest - q.front().tsNs) > m_keepWindowNs)
                q.pop_front();
        };
        purge(m_qL);
        purge(m_qR);
    }

    void OnFrameLocked(FrameItem &&fi)
    {
        if (fi.camIndex >= 0 && fi.camIndex < 2) {
            m_lastRawSeq[fi.camIndex].store(fi.seq, std::memory_order_relaxed);
            m_rawFrameCount[fi.camIndex].fetch_add(1, std::memory_order_relaxed);
        }
        if (fi.camIndex == 0)
            m_qL.push_back(std::move(fi));
        else
            m_qR.push_back(std::move(fi));

        PurgeOldLocked();
        while (TryPairLocked()) {
        }
    }

    int m_w{640}, m_h{400}, m_fps{60};
    int m_maxPairQueue{8};
    int64_t m_pairThreshNs{2'000'000};
    int64_t m_keepWindowNs{120'000'000};

    std::unique_ptr<CameraManager> m_cm;
    std::shared_ptr<Camera> m_camL, m_camR;
    LibcameraMonoCam m_left, m_right;

    mutable std::mutex m_muPair;
    std::condition_variable m_cvPair;
    std::deque<FrameItem> m_qL, m_qR;
    std::deque<std::pair<FrameItem, FrameItem>> m_paired;
    std::atomic<bool> m_acceptFrames{false};

    std::atomic<int64_t> m_lastDtMs{0};
    std::atomic<uint32_t> m_lastSeq{0};
    std::atomic<uint64_t> m_droppedPaired{0};
    std::atomic<uint32_t> m_lastRawSeq[2]{{0}, {0}};
    std::atomic<uint64_t> m_rawFrameCount[2]{{0}, {0}};
    std::atomic<int64_t> m_lastRejectDtNs{0};
    std::atomic<uint64_t> m_droppedUnpaired[2]{{0}, {0}};
};
