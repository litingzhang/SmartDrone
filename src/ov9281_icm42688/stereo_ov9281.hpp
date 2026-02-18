
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/libcamera.h>

using namespace libcamera;


// ---------------- globals ----------------
static std::atomic<bool> g_running{true};
static void SigIntHandler(int) { g_running.store(false); }
// ---------------- frames ----------------
struct FrameItem {
    int cam_index{-1};    // 0 left, 1 right
    uint64_t ts_ns{0};    // libcamera metadata timestamp
    uint32_t seq{0};      // libcamera metadata sequence（diagnostic only）
    int64_t arrive_ns{0}; // NowNs() at callback
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
    bool Open(std::shared_ptr<Camera> cam, int cam_index, int w, int h, int fps, bool ae_disable,
              int exposure_us, float gain, bool request_y8)
    {
        cam_ = std::move(cam);
        cam_index_ = cam_index;

        if (!cam_)
            return false;
        if (cam_->acquire()) {
            std::cerr << "Failed to acquire camera " << cam_->id() << "\n";
            return false;
        }

        config_ = cam_->generateConfiguration({StreamRole::Viewfinder});
        if (!config_ || config_->size() < 1) {
            std::cerr << "Failed to generate config\n";
            return false;
        }

        StreamConfiguration &sc = config_->at(0);
        sc.size.width = w;
        sc.size.height = h;

        if (request_y8) {
            sc.pixelFormat = formats::R8; // mono 8-bit
        }

        const int64_t us = std::max<int64_t>(1, 1000000LL / std::max(1, fps));
        controls_.set(controls::FrameDurationLimits, Span<const int64_t, 2>({us, us}));

        if (ae_disable) {
            controls_.set(controls::AeEnable, false);
            controls_.set(controls::ExposureTime, exposure_us);
            controls_.set(controls::AnalogueGain, gain);
        }

        CameraConfiguration::Status status = config_->validate();
        if (status == CameraConfiguration::Invalid) {
            std::cerr << "Invalid camera configuration\n";
            return false;
        }

        if (cam_->configure(config_.get())) {
            std::cerr << "Failed to configure camera\n";
            return false;
        }

        stream_ = sc.stream();
        if (!stream_) {
            std::cerr << "No stream after configure\n";
            return false;
        }

        allocator_ = std::make_unique<FrameBufferAllocator>(cam_);
        if (allocator_->allocate(stream_) < 0) {
            std::cerr << "Failed to allocate buffers\n";
            return false;
        }

        const auto &buffers = allocator_->buffers(stream_);
        if (buffers.empty()) {
            std::cerr << "No buffers allocated\n";
            return false;
        }

        bufferMaps_.clear();
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
            bufferMaps_[buf.get()] = std::move(planes);
        }

        requests_.clear();
        for (auto &buf : buffers) {
            std::unique_ptr<Request> req = cam_->createRequest();
            if (!req)
                return false;
            if (req->addBuffer(stream_, buf.get()) < 0)
                return false;
            requests_.push_back(std::move(req));
        }

        cam_->requestCompleted.connect(this, &LibcameraMonoCam::OnRequestComplete);
        return true;
    }

    bool Start()
    {
        if (cam_->start(&controls_)) {
            std::cerr << "camera start failed\n";
            return false;
        }
        for (auto &r : requests_) {
            if (cam_->queueRequest(r.get()) < 0)
                return false;
        }
        return true;
    }

    void Stop()
    {
        if (cam_)
            cam_->stop();
    }

    void Close()
    {
        for (auto &kv : bufferMaps_)
            for (auto &pm : kv.second)
                MUnmap(pm.addr, pm.len);
        bufferMaps_.clear();

        if (cam_) {
            cam_->requestCompleted.disconnect(this, &LibcameraMonoCam::OnRequestComplete);
            cam_->release();
            cam_.reset();
        }
    }

    void SetSink(std::function<void(FrameItem &&)> sink) { sink_ = std::move(sink); }

    PixelFormat PixelFmt() const { return config_->at(0).pixelFormat; }
    Size SizeWH() const { return config_->at(0).size; }
    int Stride() const { return config_->at(0).stride; }

    void SetR16Normalize(bool on) { r16_normalize_ = on; }

  private:
    void OnRequestComplete(Request *req)
    {
        if (!req || req->status() == Request::RequestCancelled)
            return;

        auto it = req->buffers().find(stream_);
        if (it == req->buffers().end()) {
            cam_->queueRequest(req);
            return;
        }
        FrameBuffer *buf = it->second;

        const FrameMetadata &md = buf->metadata();

        FrameItem item;
        item.cam_index = cam_index_;
        item.arrive_ns = NowNs();
        item.ts_ns = md.timestamp;
        item.seq = md.sequence;

        const StreamConfiguration &sc = config_->at(0);
        const int w = sc.size.width;
        const int h = sc.size.height;
        const int stride = sc.stride;

        auto mit = bufferMaps_.find(buf);
        if (mit == bufferMaps_.end() || mit->second.empty() || !mit->second[0].addr) {
            req->reuse(Request::ReuseBuffers);
            cam_->queueRequest(req);
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
            if (!r16_normalize_) {
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

        if (sink_)
            sink_(std::move(item));

        req->reuse(Request::ReuseBuffers);
        cam_->queueRequest(req);
    }

    std::shared_ptr<Camera> cam_;
    int cam_index_{-1};

    std::unique_ptr<CameraConfiguration> config_;
    Stream *stream_{nullptr};

    ControlList controls_{controls::controls};

    std::unique_ptr<FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<Request>> requests_;
    std::map<FrameBuffer *, std::vector<PlaneMap>> bufferMaps_;
    std::function<void(FrameItem &&)> sink_;
    bool r16_normalize_{false};
};

// ---------------- Stereo pairing with SOFTWARE CAM OFFSET ----------------
class LibcameraStereoOV9281_TsPair {
  public:
    bool Open(int w, int h, int fps, bool ae_disable, int exposure_us, float gain, bool request_y8,
              int64_t pair_thresh_ns, int64_t keep_window_ns, int max_pair_queue = 8,
              bool r16_normalize = false)
    {
        w_ = w;
        h_ = h;
        fps_ = fps;
        max_pair_queue_ = max_pair_queue;
        pair_thresh_ns_ = pair_thresh_ns;
        keep_window_ns_ = keep_window_ns;

        cm_ = std::make_unique<CameraManager>();
        if (cm_->start()) {
            std::cerr << "CameraManager start failed\n";
            return false;
        }

        const auto &cams = cm_->cameras();
        if (cams.size() < 2) {
            std::cerr << "Need 2 cameras, but found " << cams.size() << "\n";
            return false;
        }

        camL_ = cams[0];
        camR_ = cams[1];

        auto sinkL = [&](FrameItem &&fi) { PushInbox(0, std::move(fi)); };
        auto sinkR = [&](FrameItem &&fi) { PushInbox(1, std::move(fi)); };

        if (!left_.Open(camL_, 0, w, h, fps, ae_disable, exposure_us, gain, request_y8))
            return false;
        if (!right_.Open(camR_, 1, w, h, fps, ae_disable, exposure_us, gain, request_y8))
            return false;

        left_.SetR16Normalize(r16_normalize);
        right_.SetR16Normalize(r16_normalize);

        left_.SetSink(sinkL);
        right_.SetSink(sinkR);

        if (!left_.Start())
            return false;
        if (!right_.Start())
            return false;

        std::cerr << "Left fmt=" << left_.PixelFmt().toString()
                  << " size=" << left_.SizeWH().toString() << " stride=" << left_.Stride() << "\n";
        std::cerr << "Right fmt=" << right_.PixelFmt().toString()
                  << " size=" << right_.SizeWH().toString() << " stride=" << right_.Stride()
                  << "\n";

        running_.store(true);
        thL_ = std::thread([&] { ConsumeLoop(0); });
        thR_ = std::thread([&] { ConsumeLoop(1); });

        return true;
    }

    void Close()
    {
        running_.store(false);
        cv_in_.notify_all();
        cv_est_.notify_all();
        if (thL_.joinable())
            thL_.join();
        if (thR_.joinable())
            thR_.join();

        {
            std::lock_guard<std::mutex> lk(mu_in_);
            inboxL_.clear();
            inboxR_.clear();
        }

        left_.Stop();
        right_.Stop();
        left_.Close();
        right_.Close();
        if (cm_)
            cm_->stop();
        cm_.reset();
    }

    // manual set (ns)
    void SetCam1OffsetNs(int64_t ns) { cam1_ts_offset_ns_.store(ns, std::memory_order_relaxed); }
    int64_t Cam1OffsetNs() const { return cam1_ts_offset_ns_.load(std::memory_order_relaxed); }

    // Auto estimate cam1 offset without requiring paired frames.
    // Returns true if estimated, false if timeout/insufficient samples.
    bool AutoEstimateAndSetOffset(int samples, int timeout_ms)
    {
        if (samples <= 10)
            samples = 10;

        // enable estimate mode: consumers will only record timestamps and NOT feed pairing
        {
            std::lock_guard<std::mutex> lk(mu_est_);
            estimating_.store(true);
            est_ready_.store(false);
            tsL_.clear();
            tsR_.clear();
            need_samples_ = samples;
        }
        cv_est_.notify_all();

        const int64_t t_deadline = NowNs() + (int64_t)timeout_ms * 1'000'000LL;

        while (g_running.load()) {
            if (est_ready_.load())
                break;
            if (NowNs() > t_deadline)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        bool ok = est_ready_.load();
        estimating_.store(false);

        if (ok) {
            const int64_t off = cam1_ts_offset_ns_.load();
            std::cerr << "[camoff] estimated cam1_ts_offset_ns=" << off << " (" << (off / 1e6)
                      << " ms)\n";
        } else {
            std::cerr << "[camoff] estimate failed/timeout. cam1_ts_offset_ns stays "
                      << cam1_ts_offset_ns_.load() << "\n";
        }
        return ok;
    }

    bool GrabPair(FrameItem &L, FrameItem &R, int timeout_ms = 1000)
    {
        std::unique_lock<std::mutex> lk(mu_pair_);
        if (!cv_pair_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                               [&] { return !paired_.empty() || !g_running.load(); })) {
            return false;
        }
        if (paired_.empty())
            return false;
        auto &p = paired_.front();
        L = std::move(p.first);
        R = std::move(p.second);
        paired_.pop_front();
        return true;
    }

    int64_t LastDtMs() const { return last_dt_ms_.load(); }
    uint32_t LastSeq() const { return last_seq_.load(); }

    size_t PendL() const
    {
        std::lock_guard<std::mutex> lk(mu_pair_);
        return qL_.size();
    }
    size_t PendR() const
    {
        std::lock_guard<std::mutex> lk(mu_pair_);
        return qR_.size();
    }

  private:
    void PushInbox(int cam_idx, FrameItem &&fi)
    {
        {
            std::lock_guard<std::mutex> lk(mu_in_);
            if (cam_idx == 0)
                inboxL_.push_back(std::move(fi));
            else
                inboxR_.push_back(std::move(fi));

            const size_t kMaxInbox = 12;
            auto &inb = (cam_idx == 0) ? inboxL_ : inboxR_;
            while (inb.size() > kMaxInbox)
                inb.pop_front();
        }
        cv_in_.notify_all();
    }

    bool PopInbox(int cam_idx, FrameItem &out)
    {
        std::unique_lock<std::mutex> lk(mu_in_);
        cv_in_.wait(lk, [&] {
            if (!running_.load() || !g_running.load())
                return true;
            return cam_idx == 0 ? !inboxL_.empty() : !inboxR_.empty();
        });

        if (!running_.load() || !g_running.load())
            return false;

        auto &inb = (cam_idx == 0) ? inboxL_ : inboxR_;
        if (inb.empty())
            return false;

        out = std::move(inb.front());
        inb.pop_front();
        return true;
    }

    void RecordForEstimateLocked(int cam_idx, uint64_t ts)
    {
        // store monotonic timestamps
        if (cam_idx == 0)
            tsL_.push_back((int64_t)ts);
        else
            tsR_.push_back((int64_t)ts);

        // keep bounded
        const size_t kMax = (size_t)need_samples_ + 50;
        if (tsL_.size() > kMax)
            tsL_.erase(tsL_.begin(), tsL_.begin() + (tsL_.size() - kMax));
        if (tsR_.size() > kMax)
            tsR_.erase(tsR_.begin(), tsR_.begin() + (tsR_.size() - kMax));

        // if enough, compute offset
        if ((int)tsL_.size() >= need_samples_ && (int)tsR_.size() >= need_samples_) {
            // compute nearest-neighbor dt: (R - L)
            std::vector<int64_t> dts;
            dts.reserve(std::min(tsL_.size(), tsR_.size()));

            // two-pointer nearest neighbor (both are increasing)
            size_t j = 0;
            for (size_t i = 0; i < tsL_.size(); ++i) {
                int64_t tL = tsL_[i];
                while (j + 1 < tsR_.size() && Abs64(tsR_[j + 1] - tL) <= Abs64(tsR_[j] - tL))
                    j++;
                dts.push_back(tsR_[j] - tL);
            }

            int64_t dt_med = Median(dts);
            // we want to make (R + off) align with L => off = -median(R-L)
            int64_t off = -dt_med;

            cam1_ts_offset_ns_.store(off, std::memory_order_relaxed);
            est_ready_.store(true);
            cv_est_.notify_all();
        }
    }

    void ConsumeLoop(int cam_idx)
    {
        while (running_.load() && g_running.load()) {
            FrameItem fi;
            if (!PopInbox(cam_idx, fi))
                continue;

            // estimation mode: just record raw timestamps and drop frames (no pairing yet)
            if (estimating_.load(std::memory_order_relaxed)) {
                std::lock_guard<std::mutex> lk(mu_est_);
                if (!est_ready_.load()) {
                    RecordForEstimateLocked(cam_idx, fi.ts_ns);
                }
                continue;
            }

            // apply software offset to RIGHT camera timestamp
            if (cam_idx == 1) {
                int64_t off = cam1_ts_offset_ns_.load(std::memory_order_relaxed);
                fi.ts_ns = (uint64_t)((int64_t)fi.ts_ns + off);
            }

            // feed pairing
            {
                std::lock_guard<std::mutex> lk(mu_pair_);
                OnFrameLocked(std::move(fi));
            }
        }
    }

    bool TryPairLocked()
    {
        if (qL_.empty() || qR_.empty())
            return false;

        bool anchor_left = (qL_.front().ts_ns <= qR_.front().ts_ns);
        auto &Aq = anchor_left ? qL_ : qR_;
        auto &Bq = anchor_left ? qR_ : qL_;

        const uint64_t ats = Aq.front().ts_ns;

        size_t best = 0;
        int64_t best_dt = INT64_MAX;
        for (size_t i = 0; i < Bq.size(); ++i) {
            int64_t dt = (int64_t)Bq[i].ts_ns - (int64_t)ats;
            int64_t adt = Abs64(dt);
            if (adt < best_dt) {
                best_dt = adt;
                best = i;
            }
            if ((int64_t)Bq[i].ts_ns > (int64_t)ats && adt > best_dt)
                break;
        }

        if (best_dt > pair_thresh_ns_) {
            int64_t dt_lr = (int64_t)qR_.front().ts_ns - (int64_t)qL_.front().ts_ns;
            if (dt_lr > 0)
                qL_.pop_front();
            else
                qR_.pop_front();
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

        last_dt_ms_.store(((int64_t)R.ts_ns - (int64_t)L.ts_ns) / 1'000'000);
        last_seq_.store(L.seq);

        paired_.push_back({std::move(L), std::move(R)});
        while ((int)paired_.size() > max_pair_queue_)
            paired_.pop_front();
        cv_pair_.notify_one();
        return true;
    }

    void PurgeOldLocked()
    {
        uint64_t newest = 0;
        if (!qL_.empty())
            newest = std::max<uint64_t>(newest, qL_.back().ts_ns);
        if (!qR_.empty())
            newest = std::max<uint64_t>(newest, qR_.back().ts_ns);

        auto purge = [&](std::deque<FrameItem> &q) {
            while (!q.empty() && (int64_t)(newest - q.front().ts_ns) > keep_window_ns_)
                q.pop_front();
        };
        purge(qL_);
        purge(qR_);
    }

    void OnFrameLocked(FrameItem &&fi)
    {
        if (fi.cam_index == 0)
            qL_.push_back(std::move(fi));
        else
            qR_.push_back(std::move(fi));

        PurgeOldLocked();
        while (TryPairLocked()) {
        }
    }

  private:
    int w_{1280}, h_{800}, fps_{30};
    int max_pair_queue_{8};

    int64_t pair_thresh_ns_{2'000'000};
    int64_t keep_window_ns_{120'000'000};

    std::unique_ptr<CameraManager> cm_;
    std::shared_ptr<Camera> camL_, camR_;
    LibcameraMonoCam left_, right_;

    // inbox (producer: libcamera callbacks; consumer: 2 threads)
    std::atomic<bool> running_{false};
    std::thread thL_, thR_;
    std::mutex mu_in_;
    std::condition_variable cv_in_;
    std::deque<FrameItem> inboxL_, inboxR_;

    // pairing
    mutable std::mutex mu_pair_;
    std::condition_variable cv_pair_;
    std::deque<FrameItem> qL_, qR_;
    std::deque<std::pair<FrameItem, FrameItem>> paired_;

    std::atomic<int64_t> last_dt_ms_{0};
    std::atomic<uint32_t> last_seq_{0};

    // software offset
    std::atomic<int64_t> cam1_ts_offset_ns_{0};

    // estimation state
    std::atomic<bool> estimating_{false};
    std::atomic<bool> est_ready_{false};
    std::mutex mu_est_;
    std::condition_variable cv_est_;
    std::vector<int64_t> tsL_, tsR_;
    int need_samples_{120};
};
