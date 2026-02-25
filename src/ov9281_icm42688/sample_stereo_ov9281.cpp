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

static std::atomic<bool> g_runningFlag{true};

struct StereoFrame {
  double t;       // seconds
  cv::Mat left;   // CV_8UC1
  cv::Mat right;  // CV_8UC1
};

static void SigIntHandler(int) { g_runningFlag.store(false); }

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
  int camIndex{-1};      // 0 left, 1 right
  uint64_t tsNs{0};      // libcamera timestamp (ns)
  cv::Mat gray;           // CV_8UC1
};

class LibcameraMonoCam {
public:
  bool Open(std::shared_ptr<Camera> cam, int camIndex, int w, int h, int fps) {
    m_cam = std::move(cam);
    m_camIndex = camIndex;

    if (!m_cam) return false;
    if (m_cam->acquire()) {
      std::cerr << "Failed to acquire camera " << m_cam->id() << "\n";
      return false;
    }

    // Generate configuration: one stream (viewfinder is fine)
    m_config = m_cam->generateConfiguration({StreamRole::Viewfinder});
    if (!m_config || m_config->size() < 1) {
      std::cerr << "Failed to generate config\n";
      return false;
    }

    StreamConfiguration &sc = m_config->at(0);
    sc.size.width = w;
    sc.size.height = h;

    // Prefer GREY/Y8; if pipeline changes it, we'll handle Y plane later.
    sc.pixelFormat = formats::R8;

    // Ask for higher fps via controls (pipeline may clamp)
    m_controls.set(controls::FrameDurationLimits,
                  Span<const int64_t, 2>({1000000LL / fps, 1000000LL / fps}));
    m_controls.set(controls::AeEnable, false);

    m_controls.set(controls::ExposureTime, 3000);     // 3ms 例子
    m_controls.set(controls::AnalogueGain, 4.0f);     // 4x 例子
    CameraConfiguration::Status status = m_config->validate();
    if (status == CameraConfiguration::Invalid) {
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

    // mmap each buffer plane once
    m_bufferMaps.clear();
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
      m_bufferMaps[buf.get()] = std::move(planes);
    }

    // Create requests and attach buffers
    m_requests.clear();
    for (auto &buf : buffers) {
      std::unique_ptr<Request> req = m_cam->createRequest();
      if (!req) {
        std::cerr << "createRequest failed\n";
        return false;
      }
      if (req->addBuffer(m_stream, buf.get()) < 0) {
        std::cerr << "addBuffer failed\n";
        return false;
      }
      m_requests.push_back(std::move(req));
    }

    m_cam->requestCompleted.connect(this, &LibcameraMonoCam::OnRequestComplete);

    return true;
  }

  bool Start() {
    if (m_cam->start(&m_controls)) {
      std::cerr << "camera start failed\n";
      return false;
    }
    // queue all requests
    for (auto &r : m_requests) {
      if (m_cam->queueRequest(r.get()) < 0) {
        std::cerr << "queueRequest failed\n";
        return false;
      }
    }
    return true;
  }

  void Stop() {
    if (m_cam) m_cam->stop();
  }

  void Close() {
    // unmap
    for (auto &kv : m_bufferMaps) {
      for (auto &pm : kv.second) MUnmap(pm.addr, pm.len);
    }
    m_bufferMaps.clear();

    if (m_cam) {
      m_cam->requestCompleted.disconnect(this, &LibcameraMonoCam::OnRequestComplete);
      m_cam->release();
      m_cam.reset();
    }
  }

  // 提供一个线程安全的“取帧回调”
  void SetSink(std::function<void(FrameItem &&)> sink) {
    m_sink = std::move(sink);
  }

  // Debug info
  PixelFormat PixelFmt() const { return m_config->at(0).pixelFormat; }
  Size SizeWH() const { return m_config->at(0).size; }
  int Stride() const { return m_config->at(0).stride; }

private:
  void OnRequestComplete(Request *req) {
    if (!req || req->status() == Request::RequestCancelled) return;

    // timestamp (ns): libcamera::FrameMetadata::timestamp
    auto *buffer = req->findBuffer(m_stream);
    const libcamera::FrameMetadata &fbmd = buffer->metadata();
    uint64_t ts = fbmd.timestamp;      // ns (libcamera 里通常是 ns)
    unsigned seq = fbmd.sequence;

    // Find buffer
    auto it = req->buffers().find(m_stream);
    if (it == req->buffers().end()) {
      m_cam->queueRequest(req);
      return;
    }
    FrameBuffer *buf = it->second;

    // Convert to grayscale cv::Mat
    FrameItem item;
    item.camIndex = m_camIndex;
    item.tsNs = ts;

    const StreamConfiguration &sc = m_config->at(0);
    const int w = sc.size.width;
    const int h = sc.size.height;
    const int stride = sc.stride; // bytes per row (for plane 0)

    // Plane 0 address
    auto mit = m_bufferMaps.find(buf);
    if (mit == m_bufferMaps.end() || mit->second.empty() || !mit->second[0].addr) {
      m_cam->queueRequest(req);
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


    if (m_sink) m_sink(std::move(item));

    // Requeue
    req->reuse(Request::ReuseBuffers);
    m_cam->queueRequest(req);
  }

  std::shared_ptr<libcamera::Camera> m_cam;
  int m_camIndex{-1};

  std::unique_ptr<CameraConfiguration> m_config;
  Stream *m_stream{nullptr};

  ControlList m_controls{controls::controls};

  std::unique_ptr<FrameBufferAllocator> m_allocator;
  std::vector<std::unique_ptr<Request>> m_requests;

  std::map<FrameBuffer *, std::vector<PlaneMap>> m_bufferMaps;

  std::function<void(FrameItem &&)> m_sink;
};

class LibcameraStereoOV9281 {
public:
  bool Open(int w, int h, int fps, int maxPairQueue = 8, uint64_t pairTolNs = 1500000ULL) {
    m_w = w; m_h = h; m_fps = fps;
    m_maxPairQueue = maxPairQueue;
    m_pairTolNs = pairTolNs;

    m_cm = std::make_unique<CameraManager>();
    if (m_cm->start()) {
      std::cerr << "CameraManager start failed\n";
      return false;
    }

    const std::vector<std::shared_ptr<Camera>> &cams = m_cm->cameras();
    if (cams.size() < 2) {
      std::cerr << "Need 2 cameras, but found " << cams.size() << "\n";
      return false;
    }

    // Pick first two cameras as left/right
    m_camL = cams[0];
    m_camR = cams[1];

    // sink callback: push into per-cam queue, then try pair
    auto sink = [&](FrameItem &&fi) {
      std::lock_guard<std::mutex> lk(m_mu);
      if (fi.camIndex == 0) m_qL.push_back(std::move(fi));
      else m_qR.push_back(std::move(fi));
      TryPairLocked();
    };

    if (!m_left.Open(m_camL, 0, w, h, fps)) return false;
    if (!m_right.Open(m_camR, 1, w, h, fps)) return false;
    m_left.SetSink(sink);
    m_right.SetSink(sink);

    // Start both
    if (!m_left.Start()) return false;
    if (!m_right.Start()) return false;

    // Print actual config
    std::cerr << "Left fmt=" << m_left.PixelFmt().toString()
              << " size=" << m_left.SizeWH().toString()
              << " stride=" << m_left.Stride() << "\n";
    std::cerr << "Right fmt=" << m_right.PixelFmt().toString()
              << " size=" << m_right.SizeWH().toString()
              << " stride=" << m_right.Stride() << "\n";

    return true;
  }

  void Close() {
    m_left.Stop();
    m_right.Stop();
    m_left.Close();
    m_right.Close();

    if (m_cm) m_cm->stop();
    m_cm.reset();
  }

  // 阻塞等一对同步帧（按 timestamp 配对）
  bool Grab(StereoFrame &out, int timeoutMs = 1000) {
    std::unique_lock<std::mutex> lk(m_mu);
    if (!m_cv.wait_for(lk, std::chrono::milliseconds(timeoutMs), [&]{ return !m_paired.empty() || !g_runningFlag.load(); })) {
      return false;
    }
    if (m_paired.empty()) return false;

    out = std::move(m_paired.front());
    m_paired.pop_front();
    return true;
  }

private:
  void TryPairLocked() {
    // 简单策略：取队头，按 ts 最接近配对；超出容忍则丢旧的
    while (!m_qL.empty() && !m_qR.empty()) {
      uint64_t tL = m_qL.front().tsNs;
      uint64_t tR = m_qR.front().tsNs;

      int64_t dt = (int64_t)tL - (int64_t)tR;
      uint64_t adt = (dt < 0) ? (uint64_t)(-dt) : (uint64_t)dt;

      if (adt <= m_pairTolNs) {
        StereoFrame sf;
        uint64_t t = (tL + tR) / 2;
        sf.t = double(t) * 1e-9;  // seconds
        sf.left = std::move(m_qL.front().gray);
        sf.right = std::move(m_qR.front().gray);
        m_qL.pop_front();
        m_qR.pop_front();

        m_paired.push_back(std::move(sf));
        while ((int)m_paired.size() > m_maxPairQueue) m_paired.pop_front();
        m_cv.notify_one();
      } else {
        // drop older one to catch up
        if (tL < tR) m_qL.pop_front();
        else m_qR.pop_front();
      }
    }
  }

  int m_w{1280}, m_h{800}, m_fps{200};
  int m_maxPairQueue{8};
  uint64_t m_pairTolNs{1500000ULL}; // 1.5ms tolerance

  std::unique_ptr<CameraManager> m_cm;
    std::shared_ptr<Camera> m_camL;
    std::shared_ptr<Camera> m_camR;

  LibcameraMonoCam m_left;
  LibcameraMonoCam m_right;

  std::mutex m_mu;
  std::condition_variable m_cv;
  std::deque<FrameItem> m_qL;
  std::deque<FrameItem> m_qR;
  std::deque<StereoFrame> m_paired;
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
  while (g_runningFlag.load()) {
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
