
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/libcamera.h>

using namespace libcamera;


// ---------------- globals ----------------
static std::atomic<bool> g_runningFlag{true};
static void SigIntHandler(int) { g_runningFlag.store(false); }
// ---------------- frames ----------------
struct FrameItem {
    int camIndex{-1};    // 0 left, 1 right
    uint64_t tsNs{0};    // libcamera metadata timestamp
    uint32_t seq{0};
    int64_t arriveNs{0}; // NowNs() at callback
    cv::Mat gray;         // CV_8UC1
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
// ---------------- helpers ----------------
static int64_t Median(std::vector<int64_t> v)
{
    if (v.empty())
        return 0;
    size_t mid = v.size() / 2;
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

        if (requestY8) {
            sc.pixelFormat = formats::R8; // mono 8-bit
        }

        std::cerr << "[cam" << m_camIndex << "] request fmt=" << sc.pixelFormat.toString()
                  << " size=" << sc.size.toString() << "\n";

        const int64_t us = std::max<int64_t>(1, 1000000LL / std::max(1, fps));
        m_controls.set(controls::FrameDurationLimits, Span<const int64_t, 2>({us, us}));

        if (aeDisable) {
            m_controls.set(controls::AeEnable, false);
            m_controls.set(controls::ExposureTime, exposureUs);
            m_controls.set(controls::AnalogueGain, gain);
        }

        CameraConfiguration::Status status = m_config->validate();
        if (status == CameraConfiguration::Invalid) {
            std::cerr << "Invalid camera configuration\n";
            return false;
        }

        std::cerr << "[cam" << m_camIndex << "] validate status="
                  << (status == CameraConfiguration::Adjusted ? "Adjusted" : "Valid")
                  << " fmt=" << m_config->at(0).pixelFormat.toString()
                  << " size=" << m_config->at(0).size.toString()
                  << " stride=" << m_config->at(0).stride << "\n";

        if (m_cam->configure(m_config.get())) {
            std::cerr << "Failed to configure camera\n";
            return false;
        }

        std::cerr << "[cam" << m_camIndex << "] configured fmt="
                  << m_config->at(0).pixelFormat.toString()
                  << " size=" << m_config->at(0).size.toString()
                  << " stride=" << m_config->at(0).stride << "\n";

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
                int fd = p.fd.get();
                size_t len = p.length;
                off_t off = (off_t)p.offset;
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
            if (m_cam->queueRequest(r.get()) < 0)
                return false;
        }
        return true;
    }

    void Stop()
    {
        if (m_cam)
            m_cam->stop();
    }

    void Close()
    {
        for (auto &kv : m_bufferMaps)
            for (auto &pm : kv.second)
                MUnmap(pm.addr, pm.len);
        m_bufferMaps.clear();

        if (m_cam) {
            m_cam->requestCompleted.disconnect(this, &LibcameraMonoCam::OnRequestComplete);
            m_cam->release();
            m_cam.reset();
        }
    }

    void SetSink(std::function<void(FrameItem &&)> sink) { m_sink = std::move(sink); }

    PixelFormat PixelFmt() const { return m_config->at(0).pixelFormat; }
    Size SizeWH() const { return m_config->at(0).size; }
    int Stride() const { return m_config->at(0).stride; }

    void SetR16Normalize(bool on) { m_r16Normalize = on; }

  private:
    void OnRequestComplete(Request *req)
    {
        if (!req || req->status() == Request::RequestCancelled)
            return;

        auto it = req->buffers().find(m_stream);
        if (it == req->buffers().end()) {
            m_cam->queueRequest(req);
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
            req->reuse(Request::ReuseBuffers);
            m_cam->queueRequest(req);
            return;
        }
        uint8_t *p0 = reinterpret_cast<uint8_t *>(mit->second[0].addr);

        cv::Mat gray8;
        if (sc.pixelFormat == formats::R8) {
            cv::Mat g(h, w, CV_8UC1, (void *)p0, (size_t)stride);
            gray8 = g.clone();
        } else if (sc.pixelFormat == formats::XRGB8888) {
            cv::Mat bgra(h, w, CV_8UC4, (void *)p0, (size_t)stride);
            cv::Mat gray;
            cv::cvtColor(bgra, gray, cv::COLOR_BGRA2GRAY);
            gray8 = gray;
        } else if (sc.pixelFormat == formats::RGB888) {
            cv::Mat rgb(h, w, CV_8UC3, (void *)p0, (size_t)stride);
            cv::Mat gray;
            cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
            gray8 = gray;
        } else if (sc.pixelFormat == formats::R16) {
            cv::Mat m16(h, w, CV_16UC1, (void *)p0, (size_t)stride);
            cv::Mat g8(h, w, CV_8UC1);
            if (!m_r16Normalize) {
                m16.convertTo(g8, CV_8U, 1.0 / 256.0);
            } else {
                double minv = 0, maxv = 0;
                cv::minMaxLoc(m16, &minv, &maxv);
                double scale = (maxv > minv) ? (255.0 / (maxv - minv)) : 1.0;
                double shift = -minv * scale;
                m16.convertTo(g8, CV_8U, scale, shift);
            }
            gray8 = g8;
        } else {
            cv::Mat g(h, w, CV_8UC1, (void *)p0, (size_t)stride);
            gray8 = g.clone();
        }

        item.gray = std::move(gray8);

        if (m_sink)
            m_sink(std::move(item));

        req->reuse(Request::ReuseBuffers);
        m_cam->queueRequest(req);
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
    bool m_r16Normalize{false};
};

// ---------------- Stereo pairing with SOFTWARE CAM OFFSET ----------------
class LibcameraStereoOV9281_TsPair {
  public:
    bool Open(int w, int h, int fps, bool aeDisable, int exposureUs, float gain, bool requestY8,
              int64_t pair_thresh_ns, int64_t keepWindowNs, int maxPairQueue = 8,
              bool r16_normalize = false)
    {
        m_w = w;
        m_h = h;
        m_fps = fps;
        m_maxPairQueue = maxPairQueue;
        m_pairThreshNs = pair_thresh_ns;
        m_keepWindowNs = keepWindowNs;

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

        m_camL = cams[0];
        m_camR = cams[1];

        auto sinkL = [&](FrameItem &&fi) { PushInbox(0, std::move(fi)); };
        auto sinkR = [&](FrameItem &&fi) { PushInbox(1, std::move(fi)); };

        if (!m_left.Open(m_camL, 0, w, h, fps, aeDisable, exposureUs, gain, requestY8))
            return false;
        if (!m_right.Open(m_camR, 1, w, h, fps, aeDisable, exposureUs, gain, requestY8))
            return false;

        m_left.SetR16Normalize(r16_normalize);
        m_right.SetR16Normalize(r16_normalize);

        m_left.SetSink(sinkL);
        m_right.SetSink(sinkR);

        if (!m_left.Start())
            return false;
        if (!m_right.Start())
            return false;

        std::cerr << "Left fmt=" << m_left.PixelFmt().toString()
                  << " size=" << m_left.SizeWH().toString() << " stride=" << m_left.Stride() << "\n";
        std::cerr << "Right fmt=" << m_right.PixelFmt().toString()
                  << " size=" << m_right.SizeWH().toString() << " stride=" << m_right.Stride()
                  << "\n";

        m_running.store(true);
        m_thL = std::thread([&] { ConsumeLoop(0); });
        m_thR = std::thread([&] { ConsumeLoop(1); });

        return true;
    }

    void Close()
    {
        m_running.store(false);
        m_cvIn.notify_all();
        m_cvEst.notify_all();
        if (m_thL.joinable())
            m_thL.join();
        if (m_thR.joinable())
            m_thR.join();

        {
            std::lock_guard<std::mutex> lk(m_muIn);
            m_inboxL.clear();
            m_inboxR.clear();
        }

        m_left.Stop();
        m_right.Stop();
        m_left.Close();
        m_right.Close();
        if (m_cm)
            m_cm->stop();
        m_cm.reset();
    }

    // manual set (ns)
    void SetCam1OffsetNs(int64_t ns) { m_cam1TsOffsetNs.store(ns, std::memory_order_relaxed); }
    int64_t Cam1OffsetNs() const { return m_cam1TsOffsetNs.load(std::memory_order_relaxed); }

    // Auto estimate cam1 offset without requiring paired frames.
    // Returns true if estimated, false if timeout/insufficient samples.
    bool AutoEstimateAndSetOffset(int samples, int timeoutMs)
    {
        if (samples <= 10)
            samples = 10;

        // enable estimate mode: consumers will only record timestamps and NOT feed pairing
        {
            std::lock_guard<std::mutex> lk(m_muEst);
            m_estimating.store(true);
            m_estReady.store(false);
            m_tsL.clear();
            m_tsR.clear();
            m_needSamples = samples;
        }
        m_cvEst.notify_all();

        const int64_t t_deadline = NowNs() + (int64_t)timeoutMs * 1'000'000LL;

        while (g_runningFlag.load()) {
            if (m_estReady.load())
                break;
            if (NowNs() > t_deadline)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        bool ok = m_estReady.load();
        m_estimating.store(false);

        if (ok) {
            const int64_t off = m_cam1TsOffsetNs.load();
            std::cerr << "[camoff] estimated cam1TsOffsetNs=" << off << " (" << (off / 1e6)
                      << " ms)\n";
        } else {
            std::cerr << "[camoff] estimate failed/timeout. cam1TsOffsetNs stays "
                      << m_cam1TsOffsetNs.load() << "\n";
        }
        return ok;
    }

    bool GrabPair(FrameItem &L, FrameItem &R, int timeoutMs = 1000)
    {
        std::unique_lock<std::mutex> lk(m_muPair);
        if (!m_cvPair.wait_for(lk, std::chrono::milliseconds(timeoutMs),
                               [&] { return !m_paired.empty() || !g_runningFlag.load(); })) {
            return false;
        }
        if (m_paired.empty())
            return false;
        auto &p = m_paired.front();
        L = std::move(p.first);
        R = std::move(p.second);
        m_paired.pop_front();
        return true;
    }

    int64_t LastDtMs() const { return m_lastDtMs.load(); }
    uint32_t LastSeq() const { return m_lastSeq.load(); }

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

  private:
    void PushInbox(int camIdx, FrameItem &&fi)
    {
        {
            std::lock_guard<std::mutex> lk(m_muIn);
            if (camIdx == 0)
                m_inboxL.push_back(std::move(fi));
            else
                m_inboxR.push_back(std::move(fi));

            const size_t kMaxInbox = 12;
            auto &inb = (camIdx == 0) ? m_inboxL : m_inboxR;
            while (inb.size() > kMaxInbox)
                inb.pop_front();
        }
        m_cvIn.notify_all();
    }

    bool PopInbox(int camIdx, FrameItem &out)
    {
        std::unique_lock<std::mutex> lk(m_muIn);
        m_cvIn.wait(lk, [&] {
            if (!m_running.load() || !g_runningFlag.load())
                return true;
            return camIdx == 0 ? !m_inboxL.empty() : !m_inboxR.empty();
        });

        if (!m_running.load() || !g_runningFlag.load())
            return false;

        auto &inb = (camIdx == 0) ? m_inboxL : m_inboxR;
        if (inb.empty())
            return false;

        out = std::move(inb.front());
        inb.pop_front();
        return true;
    }

    void RecordForEstimateLocked(int camIdx, uint64_t ts)
    {
        // store monotonic timestamps
        if (camIdx == 0)
            m_tsL.push_back((int64_t)ts);
        else
            m_tsR.push_back((int64_t)ts);

        // keep bounded
        const size_t kMax = (size_t)m_needSamples + 50;
        if (m_tsL.size() > kMax)
            m_tsL.erase(m_tsL.begin(), m_tsL.begin() + (m_tsL.size() - kMax));
        if (m_tsR.size() > kMax)
            m_tsR.erase(m_tsR.begin(), m_tsR.begin() + (m_tsR.size() - kMax));

        // if enough, compute offset
        if ((int)m_tsL.size() >= m_needSamples && (int)m_tsR.size() >= m_needSamples) {
            // compute nearest-neighbor dt: (R - L)
            std::vector<int64_t> dts;
            dts.reserve(std::min(m_tsL.size(), m_tsR.size()));

            // two-pointer nearest neighbor (both are increasing)
            size_t j = 0;
            for (size_t i = 0; i < m_tsL.size(); ++i) {
                int64_t tL = m_tsL[i];
                while (j + 1 < m_tsR.size() && Abs64(m_tsR[j + 1] - tL) <= Abs64(m_tsR[j] - tL))
                    j++;
                dts.push_back(m_tsR[j] - tL);
            }

            int64_t dt_med = Median(dts);
            // we want to make (R + off) align with L => off = -median(R-L)
            int64_t off = -dt_med;

            m_cam1TsOffsetNs.store(off, std::memory_order_relaxed);
            m_estReady.store(true);
            m_cvEst.notify_all();
        }
    }

    void ConsumeLoop(int camIdx)
    {
        while (m_running.load() && g_runningFlag.load()) {
            FrameItem fi;
            if (!PopInbox(camIdx, fi))
                continue;

            // estimation mode: just record raw timestamps and drop frames (no pairing yet)
            if (m_estimating.load(std::memory_order_relaxed)) {
                std::lock_guard<std::mutex> lk(m_muEst);
                if (!m_estReady.load()) {
                    RecordForEstimateLocked(camIdx, fi.tsNs);
                }
                continue;
            }

            // apply software offset to RIGHT camera timestamp
            if (camIdx == 1) {
                int64_t off = m_cam1TsOffsetNs.load(std::memory_order_relaxed);
                fi.tsNs = (uint64_t)((int64_t)fi.tsNs + off);
            }

            // feed pairing
            {
                std::lock_guard<std::mutex> lk(m_muPair);
                OnFrameLocked(std::move(fi));
            }
        }
    }

    bool TryPairLocked()
    {
        if (m_qL.empty() || m_qR.empty())
            return false;

        bool anchor_left = (m_qL.front().tsNs <= m_qR.front().tsNs);
        auto &Aq = anchor_left ? m_qL : m_qR;
        auto &Bq = anchor_left ? m_qR : m_qL;

        const uint64_t ats = Aq.front().tsNs;

        size_t best = 0;
        int64_t bestDt = INT64_MAX;
        for (size_t i = 0; i < Bq.size(); ++i) {
            int64_t dt = (int64_t)Bq[i].tsNs - (int64_t)ats;
            int64_t adt = Abs64(dt);
            if (adt < bestDt) {
                bestDt = adt;
                best = i;
            }
            if ((int64_t)Bq[i].tsNs > (int64_t)ats && adt > bestDt)
                break;
        }

        if (bestDt > m_pairThreshNs) {
            int64_t dtLr = (int64_t)m_qR.front().tsNs - (int64_t)m_qL.front().tsNs;
            if (dtLr > 0)
                m_qL.pop_front();
            else
                m_qR.pop_front();
            return true;
        }

        FrameItem A = std::move(Aq.front());
        FrameItem B = std::move(Bq[best]);
        Aq.pop_front();
        Bq.erase(Bq.begin() + best);

        FrameItem L, R;
        if (anchor_left) {
            L = std::move(A);
            R = std::move(B);
        } else {
            L = std::move(B);
            R = std::move(A);
        }

        m_lastDtMs.store(((int64_t)R.tsNs - (int64_t)L.tsNs) / 1'000'000);
        m_lastSeq.store(L.seq);

        m_paired.push_back({std::move(L), std::move(R)});
        while ((int)m_paired.size() > m_maxPairQueue)
            m_paired.pop_front();
        m_cvPair.notify_one();
        return true;
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
        if (fi.camIndex == 0)
            m_qL.push_back(std::move(fi));
        else
            m_qR.push_back(std::move(fi));

        PurgeOldLocked();
        while (TryPairLocked()) {
        }
    }

  private:
    int m_w{640}, m_h{400}, m_fps{30};
    int m_maxPairQueue{8};

    int64_t m_pairThreshNs{2'000'000};
    int64_t m_keepWindowNs{120'000'000};

    std::unique_ptr<CameraManager> m_cm;
    std::shared_ptr<Camera> m_camL, m_camR;
    LibcameraMonoCam m_left, m_right;

    // inbox (producer: libcamera callbacks; consumer: 2 threads)
    std::atomic<bool> m_running{false};
    std::thread m_thL, m_thR;
    std::mutex m_muIn;
    std::condition_variable m_cvIn;
    std::deque<FrameItem> m_inboxL, m_inboxR;

    // pairing
    mutable std::mutex m_muPair;
    std::condition_variable m_cvPair;
    std::deque<FrameItem> m_qL, m_qR;
    std::deque<std::pair<FrameItem, FrameItem>> m_paired;

    std::atomic<int64_t> m_lastDtMs{0};
    std::atomic<uint32_t> m_lastSeq{0};

    // software offset
    std::atomic<int64_t> m_cam1TsOffsetNs{0};

    // estimation state
    std::atomic<bool> m_estimating{false};
    std::atomic<bool> m_estReady{false};
    std::mutex m_muEst;
    std::condition_variable m_cvEst;
    std::vector<int64_t> m_tsL, m_tsR;
    int m_needSamples{120};
};
