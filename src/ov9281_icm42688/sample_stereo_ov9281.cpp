// libcamera_stereo_ov9281.cpp
#include <libcamera/libcamera.h>
#include <libcamera/framebuffer.h> 
#include <opencv2/opencv.hpp>
#include <csignal>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

using namespace libcamera;

static std::atomic<bool> g_running{true};

struct StereoFrame {
  double t;       // seconds
  cv::Mat left;   // CV_8UC1
  cv::Mat right;  // CV_8UC1
};

static void SigIntHandler(int) { g_running.store(false); }

static void *MMapFD(int fd, size_t len, off_t off = 0) {
  void *p = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, off);
  if (p == MAP_FAILED) return nullptr;
  return p;
}

static void MUnmap(void *p, size_t len) {
  if (p && p != MAP_FAILED) munmap(p, len);
}

struct PlaneMap {
  void *addr{nullptr};
  size_t len{0};
};

struct FrameItem {
  int cam_index{-1};      // 0 left, 1 right
  uint64_t ts_ns{0};      // libcamera timestamp (ns)
  cv::Mat gray;           // CV_8UC1
};

class LibcameraMonoCam {
public:
  bool Open(std::shared_ptr<Camera> cam, int cam_index, int w, int h, int fps) {
    cam_ = std::move(cam);
    cam_index_ = cam_index;

    if (!cam_) return false;
    if (cam_->acquire()) {
      std::cerr << "Failed to acquire camera " << cam_->id() << "\n";
      return false;
    }

    // Generate configuration: one stream (viewfinder is fine)
    config_ = cam_->generateConfiguration({StreamRole::Viewfinder});
    if (!config_ || config_->size() < 1) {
      std::cerr << "Failed to generate config\n";
      return false;
    }

    StreamConfiguration &sc = config_->at(0);
    sc.size.width = w;
    sc.size.height = h;

    // Prefer GREY/Y8; if pipeline changes it, we'll handle Y plane later.
    sc.pixelFormat = formats::R8;

    // Ask for higher fps via controls (pipeline may clamp)
    controls_.set(controls::FrameDurationLimits,
                  Span<const int64_t, 2>({1000000LL / fps, 1000000LL / fps}));
    controls_.set(controls::AeEnable, false);

    controls_.set(controls::ExposureTime, 3000);     // 3ms 例子
    controls_.set(controls::AnalogueGain, 4.0f);     // 4x 例子
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

    // mmap each buffer plane once
    bufferMaps_.clear();
    for (auto &buf : buffers) {
      std::vector<PlaneMap> planes;
      planes.reserve(buf->planes().size());
      for (const FrameBuffer::Plane &p : buf->planes()) {
        int fd = p.fd.get();
        size_t len = p.length;
        void *addr = MMapFD(fd, len);
        if (!addr) {
          std::cerr << "mmap failed\n";
          return false;
        }
        planes.push_back({addr, len});
      }
      bufferMaps_[buf.get()] = std::move(planes);
    }

    // Create requests and attach buffers
    requests_.clear();
    for (auto &buf : buffers) {
      std::unique_ptr<Request> req = cam_->createRequest();
      if (!req) {
        std::cerr << "createRequest failed\n";
        return false;
      }
      if (req->addBuffer(stream_, buf.get()) < 0) {
        std::cerr << "addBuffer failed\n";
        return false;
      }
      requests_.push_back(std::move(req));
    }

    cam_->requestCompleted.connect(this, &LibcameraMonoCam::OnRequestComplete);

    return true;
  }

  bool Start() {
    if (cam_->start(&controls_)) {
      std::cerr << "camera start failed\n";
      return false;
    }
    // queue all requests
    for (auto &r : requests_) {
      if (cam_->queueRequest(r.get()) < 0) {
        std::cerr << "queueRequest failed\n";
        return false;
      }
    }
    return true;
  }

  void Stop() {
    if (cam_) cam_->stop();
  }

  void Close() {
    // unmap
    for (auto &kv : bufferMaps_) {
      for (auto &pm : kv.second) MUnmap(pm.addr, pm.len);
    }
    bufferMaps_.clear();

    if (cam_) {
      cam_->requestCompleted.disconnect(this, &LibcameraMonoCam::OnRequestComplete);
      cam_->release();
      cam_.reset();
    }
  }

  // 提供一个线程安全的“取帧回调”
  void SetSink(std::function<void(FrameItem &&)> sink) {
    sink_ = std::move(sink);
  }

  // Debug info
  PixelFormat PixelFmt() const { return config_->at(0).pixelFormat; }
  Size SizeWH() const { return config_->at(0).size; }
  int Stride() const { return config_->at(0).stride; }

private:
  void OnRequestComplete(Request *req) {
    if (!req || req->status() == Request::RequestCancelled) return;

    // timestamp (ns): libcamera::FrameMetadata::timestamp
    auto *buffer = req->findBuffer(stream_);
    const libcamera::FrameMetadata &fbmd = buffer->metadata();
    uint64_t ts = fbmd.timestamp;      // ns (libcamera 里通常是 ns)
    unsigned seq = fbmd.sequence;

    // Find buffer
    auto it = req->buffers().find(stream_);
    if (it == req->buffers().end()) {
      cam_->queueRequest(req);
      return;
    }
    FrameBuffer *buf = it->second;

    // Convert to grayscale cv::Mat
    FrameItem item;
    item.cam_index = cam_index_;
    item.ts_ns = ts;

    const StreamConfiguration &sc = config_->at(0);
    const int w = sc.size.width;
    const int h = sc.size.height;
    const int stride = sc.stride; // bytes per row (for plane 0)

    // Plane 0 address
    auto mit = bufferMaps_.find(buf);
    if (mit == bufferMaps_.end() || mit->second.empty() || !mit->second[0].addr) {
      cam_->queueRequest(req);
      return;
    }
    uint8_t *p0 = reinterpret_cast<uint8_t *>(mit->second[0].addr);

const PixelFormat fmt = sc.pixelFormat;

if (fmt == formats::R16) {
  const uint16_t *src = reinterpret_cast<const uint16_t *>(p0);

  // 注意：stride 是字节数；R16 每像素2字节，所以每行 uint16_t 个数是 stride/2
  const int src_step = stride / 2;

  cv::Mat m8(h, w, CV_8UC1);
  for (int y = 0; y < h; ++y) {
    const uint16_t *row = src + y * src_step;
    uint8_t *dst = m8.ptr<uint8_t>(y);
    for (int x = 0; x < w; ++x) {
      // 10-bit in low bits -> 8-bit
      dst[x] = static_cast<uint8_t>(row[x] >> 2);
    }
  }
  item.gray = std::move(m8);
} else {
    cv::Mat img16(h, w, CV_16UC1, p0, stride);

    cv::Mat img8;
    img16.convertTo(img8, CV_8U, 1.0 / 256.0);

    item.gray = img8.clone();
}


    if (sink_) sink_(std::move(item));

    // Requeue
    req->reuse(Request::ReuseBuffers);
    cam_->queueRequest(req);
  }

  std::shared_ptr<libcamera::Camera> cam_;
  int cam_index_{-1};

  std::unique_ptr<CameraConfiguration> config_;
  Stream *stream_{nullptr};

  ControlList controls_{controls::controls};

  std::unique_ptr<FrameBufferAllocator> allocator_;
  std::vector<std::unique_ptr<Request>> requests_;

  std::map<FrameBuffer *, std::vector<PlaneMap>> bufferMaps_;

  std::function<void(FrameItem &&)> sink_;
};

class LibcameraStereoOV9281 {
public:
  bool Open(int w, int h, int fps, int max_pair_queue = 8, uint64_t pair_tol_ns = 1500000ULL) {
    w_ = w; h_ = h; fps_ = fps;
    max_pair_queue_ = max_pair_queue;
    pair_tol_ns_ = pair_tol_ns;

    cm_ = std::make_unique<CameraManager>();
    if (cm_->start()) {
      std::cerr << "CameraManager start failed\n";
      return false;
    }

    const std::vector<std::shared_ptr<Camera>> &cams = cm_->cameras();
    if (cams.size() < 2) {
      std::cerr << "Need 2 cameras, but found " << cams.size() << "\n";
      return false;
    }

    // Pick first two cameras as left/right
    camL_ = cams[0];
    camR_ = cams[1];

    // sink callback: push into per-cam queue, then try pair
    auto sink = [&](FrameItem &&fi) {
      std::lock_guard<std::mutex> lk(mu_);
      if (fi.cam_index == 0) qL_.push_back(std::move(fi));
      else qR_.push_back(std::move(fi));
      TryPairLocked();
    };

    if (!left_.Open(camL_, 0, w, h, fps)) return false;
    if (!right_.Open(camR_, 1, w, h, fps)) return false;
    left_.SetSink(sink);
    right_.SetSink(sink);

    // Start both
    if (!left_.Start()) return false;
    if (!right_.Start()) return false;

    // Print actual config
    std::cerr << "Left fmt=" << left_.PixelFmt().toString()
              << " size=" << left_.SizeWH().toString()
              << " stride=" << left_.Stride() << "\n";
    std::cerr << "Right fmt=" << right_.PixelFmt().toString()
              << " size=" << right_.SizeWH().toString()
              << " stride=" << right_.Stride() << "\n";

    return true;
  }

  void Close() {
    left_.Stop();
    right_.Stop();
    left_.Close();
    right_.Close();

    if (cm_) cm_->stop();
    cm_.reset();
  }

  // 阻塞等一对同步帧（按 timestamp 配对）
  bool Grab(StereoFrame &out, int timeout_ms = 1000) {
    std::unique_lock<std::mutex> lk(mu_);
    if (!cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), [&]{ return !paired_.empty() || !g_running.load(); })) {
      return false;
    }
    if (paired_.empty()) return false;

    out = std::move(paired_.front());
    paired_.pop_front();
    return true;
  }

private:
  void TryPairLocked() {
    // 简单策略：取队头，按 ts 最接近配对；超出容忍则丢旧的
    while (!qL_.empty() && !qR_.empty()) {
      uint64_t tL = qL_.front().ts_ns;
      uint64_t tR = qR_.front().ts_ns;

      int64_t dt = (int64_t)tL - (int64_t)tR;
      uint64_t adt = (dt < 0) ? (uint64_t)(-dt) : (uint64_t)dt;

      if (adt <= pair_tol_ns_) {
        StereoFrame sf;
        uint64_t t = (tL + tR) / 2;
        sf.t = double(t) * 1e-9;  // seconds
        sf.left = std::move(qL_.front().gray);
        sf.right = std::move(qR_.front().gray);
        qL_.pop_front();
        qR_.pop_front();

        paired_.push_back(std::move(sf));
        while ((int)paired_.size() > max_pair_queue_) paired_.pop_front();
        cv_.notify_one();
      } else {
        // drop older one to catch up
        if (tL < tR) qL_.pop_front();
        else qR_.pop_front();
      }
    }
  }

  int w_{1280}, h_{800}, fps_{200};
  int max_pair_queue_{8};
  uint64_t pair_tol_ns_{1500000ULL}; // 1.5ms tolerance

  std::unique_ptr<CameraManager> cm_;
    std::shared_ptr<Camera> camL_;
    std::shared_ptr<Camera> camR_;

  LibcameraMonoCam left_;
  LibcameraMonoCam right_;

  std::mutex mu_;
  std::condition_variable cv_;
  std::deque<FrameItem> qL_;
  std::deque<FrameItem> qR_;
  std::deque<StereoFrame> paired_;
};

static int GetArgI(int argc, char **argv, const char *name, int def) {
  for (int i = 1; i + 1 < argc; i++) {
    if (std::string(argv[i]) == name) return std::stoi(argv[i + 1]);
  }
  return def;
}

int main(int argc, char **argv) {
  signal(SIGINT, SigIntHandler);
  signal(SIGTERM, SigIntHandler);

  int w = GetArgI(argc, argv, "--w", 1280);
  int h = GetArgI(argc, argv, "--h", 800);
  int fps = GetArgI(argc, argv, "--fps", 200);

  LibcameraStereoOV9281 stereo;
  if (!stereo.Open(w, h, fps)) return 1;

  std::cerr << "t_s,left_mean,right_mean\n";
  while (g_running.load()) {
    StereoFrame f;
    if (!stereo.Grab(f, 1000)) continue;

    // quick sanity: print mean gray values
    double ml = cv::mean(f.left)[0];
    double mr = cv::mean(f.right)[0];
    std::cout.setf(std::ios::fixed);
    std::cout << std::setprecision(6) << f.t
            << "," << ml << "," << mr << "\n";
  }

  stereo.Close();
  return 0;
}
