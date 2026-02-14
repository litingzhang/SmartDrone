#include <libcamera/libcamera.h>
#include <libcamera/framebuffer_allocator.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <opencv2/opencv.hpp>

#include <gpiod.h>
#include <linux/spi/spidev.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <pthread.h>
#include <sched.h>
#include "mavlink_pose_sender.hpp"
// ORB-SLAM3
#include "System.h"
#include "ImuTypes.h"
#include <sophus/se3.hpp>

using namespace libcamera;

// ---------------- globals ----------------
static std::atomic<bool> g_running{true};
static void SigIntHandler(int) { g_running.store(false); }

// ---------------- time ----------------
static int64_t NowNs() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
static inline int64_t Abs64(int64_t x) { return x < 0 ? -x : x; }

// ---------------- helpers ----------------
static int64_t Median(std::vector<int64_t> v) {
  if (v.empty()) return 0;
  size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  return v[mid];
}

// ---------------- mmap helpers ----------------
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
  off_t  off{0};
};

// ---------------- IMU (ICM42688) ----------------
static inline int16_t Be16ToI16(uint8_t hi, uint8_t lo) {
  return (int16_t)((uint16_t(hi) << 8) | uint16_t(lo));
}

// Bank0 registers (subset)
static constexpr uint8_t REG_DEVICE_CONFIG   = 0x11;
static constexpr uint8_t REG_INT_CONFIG      = 0x14;
static constexpr uint8_t REG_INT_STATUS      = 0x2D;
static constexpr uint8_t REG_PWR_MGMT0       = 0x4E;
static constexpr uint8_t REG_GYRO_CONFIG0    = 0x4F;
static constexpr uint8_t REG_ACCEL_CONFIG0   = 0x50;
static constexpr uint8_t REG_INT_CONFIG1     = 0x64;
static constexpr uint8_t REG_INT_SOURCE0     = 0x65;

static constexpr uint8_t SPI_READ_MASK  = 0x80;

struct ImuSample {
  int64_t t_ns{};
  float ax{}, ay{}, az{}; // m/s^2
  float gx{}, gy{}, gz{}; // rad/s
};
struct ImuScale {
  float accel_lsb_per_g{2048.0f};   // default for 16g
  float gyro_lsb_per_dps{16.4f};    // default for 2000 dps
};

static bool SetThreadRealtime(int prio) {
  sched_param sp{};
  sp.sched_priority = prio;
  return pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0;
}

class SpiDev {
public:
  explicit SpiDev(std::string dev) : dev_(std::move(dev)) {}

  bool Open(uint32_t speed_hz, uint8_t mode, uint8_t bits_per_word) {
    fd_ = ::open(dev_.c_str(), O_RDWR);
    if (fd_ < 0) {
      std::cerr << "open " << dev_ << " failed: " << strerror(errno) << "\n";
      return false;
    }
    if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(fd_, SPI_IOC_RD_MODE, &mode) < 0) {
      std::cerr << "SPI set mode failed: " << strerror(errno) << "\n";
      return false;
    }
    if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0 ||
        ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word) < 0) {
      std::cerr << "SPI set bits failed: " << strerror(errno) << "\n";
      return false;
    }
    if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0 ||
        ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz) < 0) {
      std::cerr << "SPI set speed failed: " << strerror(errno) << "\n";
      return false;
    }
    speed_hz_ = speed_hz;
    mode_ = mode;
    bits_ = bits_per_word;
    return true;
  }

  ~SpiDev() { if (fd_ >= 0) ::close(fd_); }

  bool WriteReg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { uint8_t(reg & 0x7F), val };
    uint8_t rx[2] = {0,0};
    return Transfer(tx, rx, sizeof(tx));
  }

  bool ReadReg(uint8_t reg, uint8_t &val) {
    uint8_t tx[2] = { uint8_t(SPI_READ_MASK | (reg & 0x7F)), 0x00 };
    uint8_t rx[2] = {0,0};
    if (!Transfer(tx, rx, sizeof(tx))) return false;
    val = rx[1];
    return true;
  }

  bool ReadRegs(uint8_t start_reg, uint8_t *out, size_t len) {
    std::vector<uint8_t> tx(len + 1, 0x00);
    std::vector<uint8_t> rx(len + 1, 0x00);
    tx[0] = uint8_t(SPI_READ_MASK | (start_reg & 0x7F));
    if (!Transfer(tx.data(), rx.data(), rx.size())) return false;
    memcpy(out, rx.data() + 1, len);
    return true;
  }

private:
  bool Transfer(const uint8_t *tx, uint8_t *rx, size_t len) {
    spi_ioc_transfer tr{};
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = (uint32_t)len;
    tr.speed_hz = speed_hz_;
    tr.bits_per_word = bits_;
    tr.delay_usecs = 0;
    if (ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
      std::cerr << "SPI transfer failed: " << strerror(errno) << "\n";
      return false;
    }
    return true;
  }

  std::string dev_;
  int fd_{-1};
  uint32_t speed_hz_{8000000};
  uint8_t mode_{SPI_MODE_0};
  uint8_t bits_{8};
};

class DrdyGpio {
public:
  bool Open(const std::string &chip_path, unsigned line_offset) {
    chip_ = gpiod_chip_open(chip_path.c_str());
    if (!chip_) {
      std::cerr << "gpiod_chip_open(" << chip_path << ") failed: " << strerror(errno) << "\n";
      return false;
    }

    gpiod_line_settings *settings = gpiod_line_settings_new();
    if (!settings) return false;
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING);
    gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);

    gpiod_line_config *line_cfg = gpiod_line_config_new();
    if (!line_cfg) { gpiod_line_settings_free(settings); return false; }

    unsigned offsets[1] = { line_offset };
    int rc = gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings);
    gpiod_line_settings_free(settings);
    if (rc < 0) { gpiod_line_config_free(line_cfg); return false; }

    gpiod_request_config *req_cfg = gpiod_request_config_new();
    if (!req_cfg) { gpiod_line_config_free(line_cfg); return false; }
    gpiod_request_config_set_consumer(req_cfg, "icm42688_drdy");

    request_ = gpiod_chip_request_lines(chip_, req_cfg, line_cfg);
    gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(line_cfg);

    if (!request_) {
      std::cerr << "gpiod_chip_request_lines failed: " << strerror(errno) << "\n";
      return false;
    }

    evbuf_ = gpiod_edge_event_buffer_new(256);
    return evbuf_ != nullptr;
  }

  ~DrdyGpio() {
    if (evbuf_) gpiod_edge_event_buffer_free(evbuf_);
    if (request_) gpiod_line_request_release(request_);
    if (chip_) gpiod_chip_close(chip_);
  }

  bool WaitTs(int timeout_ms, int64_t &ts_ns_out) {
    int64_t timeout_ns = (timeout_ms < 0) ? -1 : (int64_t)timeout_ms * 1000000LL;
    int ret = gpiod_line_request_wait_edge_events(request_, timeout_ns);
    if (ret <= 0) return false;

    int n = gpiod_line_request_read_edge_events(request_, evbuf_, 1);
    if (n <= 0) return false;

    struct gpiod_edge_event *ev = gpiod_edge_event_buffer_get_event(evbuf_, 0);
    if (!ev) return false;

    ts_ns_out = (int64_t)gpiod_edge_event_get_timestamp_ns(ev);
    return true;
  }

private:
  gpiod_chip *chip_{nullptr};
  gpiod_line_request *request_{nullptr};
  gpiod_edge_event_buffer *evbuf_{nullptr};
};

static uint8_t OdrCodeFromHz(int hz) {
  switch (hz) {
    case 8000: return 0x03;
    case 4000: return 0x04;
    case 2000: return 0x05;
    case 1000: return 0x06;
    case 200:  return 0x07;
    case 100:  return 0x08;
    case 50:   return 0x09;
    case 25:   return 0x0A;
    default:   return 0x0F;
  }
}

static bool BuildFsBitsAndScale(int accel_fs_g, int gyro_fs_dps,
                               uint8_t &accel_fs_bits, uint8_t &gyro_fs_bits,
                               ImuScale &scale) {
  switch (accel_fs_g) {
    case 2:  scale.accel_lsb_per_g = 16384.0f; accel_fs_bits = 0x03; break;
    case 4:  scale.accel_lsb_per_g = 8192.0f;  accel_fs_bits = 0x02; break;
    case 8:  scale.accel_lsb_per_g = 4096.0f;  accel_fs_bits = 0x01; break;
    case 16: scale.accel_lsb_per_g = 2048.0f;  accel_fs_bits = 0x00; break;
    default:
      std::cerr << "Unsupported accel-fs " << accel_fs_g << " (use 2/4/8/16)\n";
      return false;
  }

  switch (gyro_fs_dps) {
    case 125:  scale.gyro_lsb_per_dps = 262.4f; gyro_fs_bits = 0x04; break;
    case 250:  scale.gyro_lsb_per_dps = 131.0f; gyro_fs_bits = 0x03; break;
    case 500:  scale.gyro_lsb_per_dps = 65.5f;  gyro_fs_bits = 0x02; break;
    case 1000: scale.gyro_lsb_per_dps = 32.8f;  gyro_fs_bits = 0x01; break;
    case 2000: scale.gyro_lsb_per_dps = 16.4f;  gyro_fs_bits = 0x00; break;
    default:
      std::cerr << "Unsupported gyro-fs " << gyro_fs_dps << " (use 125/250/500/1000/2000)\n";
      return false;
  }

  return true;
}

static bool IcmResetAndConfig(SpiDev &spi, int imu_hz, int accel_fs_g, int gyro_fs_dps,
                              ImuScale &scale_out) {
  uint8_t accel_fs_bits = 0, gyro_fs_bits = 0;
  ImuScale sc{};
  if (!BuildFsBitsAndScale(accel_fs_g, gyro_fs_dps, accel_fs_bits, gyro_fs_bits, sc)) return false;

  if (!spi.WriteReg(REG_DEVICE_CONFIG, 0x01)) return false;
  usleep(100000);

  spi.WriteReg(REG_INT_CONFIG, 0x30);
  spi.WriteReg(REG_INT_SOURCE0, 0x08);  // DRDY
  spi.WriteReg(REG_INT_CONFIG1, 0x00);

  spi.WriteReg(REG_PWR_MGMT0, 0x0F);
  usleep(20000);

  uint8_t odr = OdrCodeFromHz(imu_hz);

  uint8_t gyro_cfg0  = uint8_t((gyro_fs_bits  << 5) | (odr & 0x0F));
  uint8_t accel_cfg0 = uint8_t((accel_fs_bits << 5) | (odr & 0x0F));

  if (!spi.WriteReg(REG_GYRO_CONFIG0, gyro_cfg0)) return false;
  if (!spi.WriteReg(REG_ACCEL_CONFIG0, accel_cfg0)) return false;
  usleep(20000);

  scale_out = sc;
  return true;
}

static void ConvertRaw12_AccelGyro_ToSI(const uint8_t raw12[12], const ImuScale &sc, ImuSample &s) {
  int16_t ax = Be16ToI16(raw12[0], raw12[1]);
  int16_t ay = Be16ToI16(raw12[2], raw12[3]);
  int16_t az = Be16ToI16(raw12[4], raw12[5]);
  int16_t gx = Be16ToI16(raw12[6], raw12[7]);
  int16_t gy = Be16ToI16(raw12[8], raw12[9]);
  int16_t gz = Be16ToI16(raw12[10], raw12[11]);

  constexpr float kG = 9.80665f;
  s.ax = (float(ax) / sc.accel_lsb_per_g) * kG;
  s.ay = (float(ay) / sc.accel_lsb_per_g) * kG;
  s.az = (float(az) / sc.accel_lsb_per_g) * kG;

  constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
  s.gx = (float(gx) / sc.gyro_lsb_per_dps) * kDeg2Rad;
  s.gy = (float(gy) / sc.gyro_lsb_per_dps) * kDeg2Rad;
  s.gz = (float(gz) / sc.gyro_lsb_per_dps) * kDeg2Rad;
}

// ---------------- frames ----------------
struct FrameItem {
  int cam_index{-1};     // 0 left, 1 right
  uint64_t ts_ns{0};     // libcamera metadata timestamp
  uint32_t seq{0};       // libcamera metadata sequence（diagnostic only）
  int64_t  arrive_ns{0}; // NowNs() at callback
  cv::Mat gray;          // CV_8UC1
};

// ---------------- camera (libcamera) ----------------
class LibcameraMonoCam {
public:
  bool Open(std::shared_ptr<Camera> cam, int cam_index, int w, int h, int fps,
            bool ae_disable, int exposure_us, float gain,
            bool request_y8) {
    cam_ = std::move(cam);
    cam_index_ = cam_index;

    if (!cam_) return false;
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
      sc.pixelFormat = formats::R8;  // mono 8-bit
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
      if (!req) return false;
      if (req->addBuffer(stream_, buf.get()) < 0) return false;
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
    for (auto &r : requests_) {
      if (cam_->queueRequest(r.get()) < 0) return false;
    }
    return true;
  }

  void Stop() { if (cam_) cam_->stop(); }

  void Close() {
    for (auto &kv : bufferMaps_) for (auto &pm : kv.second) MUnmap(pm.addr, pm.len);
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
  void OnRequestComplete(Request *req) {
    if (!req || req->status() == Request::RequestCancelled) return;

    auto it = req->buffers().find(stream_);
    if (it == req->buffers().end()) { cam_->queueRequest(req); return; }
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
      cv::Mat g(h, w, CV_8UC1, (void*)p0, (size_t)stride);
      gray8 = g.clone();
    } else if (sc.pixelFormat == formats::XRGB8888) {
      cv::Mat bgra(h, w, CV_8UC4, (void*)p0, (size_t)stride);
      cv::Mat gray;
      cv::cvtColor(bgra, gray, cv::COLOR_BGRA2GRAY);
      gray8 = gray;
    } else if (sc.pixelFormat == formats::RGB888) {
      cv::Mat rgb(h, w, CV_8UC3, (void*)p0, (size_t)stride);
      cv::Mat gray;
      cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
      gray8 = gray;
    } else if (sc.pixelFormat == formats::R16) {
      cv::Mat m16(h, w, CV_16UC1, (void*)p0, (size_t)stride);
      cv::Mat g8(h, w, CV_8UC1);
      if (!r16_normalize_) {
        m16.convertTo(g8, CV_8U, 1.0 / 256.0);
      } else {
        double minv=0, maxv=0;
        cv::minMaxLoc(m16, &minv, &maxv);
        double scale = (maxv > minv) ? (255.0 / (maxv - minv)) : 1.0;
        double shift = -minv * scale;
        m16.convertTo(g8, CV_8U, scale, shift);
      }
      gray8 = g8;
    } else {
      cv::Mat g(h, w, CV_8UC1, (void*)p0, (size_t)stride);
      gray8 = g.clone();
    }

    item.gray = std::move(gray8);

    if (sink_) sink_(std::move(item));

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
  bool Open(int w, int h, int fps,
            bool ae_disable, int exposure_us, float gain,
            bool request_y8,
            int64_t pair_thresh_ns,
            int64_t keep_window_ns,
            int max_pair_queue = 8,
            bool r16_normalize = false) {
    w_ = w; h_ = h; fps_ = fps;
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

    if (!left_.Open(camL_, 0, w, h, fps, ae_disable, exposure_us, gain, request_y8)) return false;
    if (!right_.Open(camR_, 1, w, h, fps, ae_disable, exposure_us, gain, request_y8)) return false;

    left_.SetR16Normalize(r16_normalize);
    right_.SetR16Normalize(r16_normalize);

    left_.SetSink(sinkL);
    right_.SetSink(sinkR);

    if (!left_.Start()) return false;
    if (!right_.Start()) return false;

    std::cerr << "Left fmt=" << left_.PixelFmt().toString()
              << " size=" << left_.SizeWH().toString()
              << " stride=" << left_.Stride() << "\n";
    std::cerr << "Right fmt=" << right_.PixelFmt().toString()
              << " size=" << right_.SizeWH().toString()
              << " stride=" << right_.Stride() << "\n";

    running_.store(true);
    thL_ = std::thread([&]{ ConsumeLoop(0); });
    thR_ = std::thread([&]{ ConsumeLoop(1); });

    return true;
  }

  void Close() {
    running_.store(false);
    cv_in_.notify_all();
    cv_est_.notify_all();
    if (thL_.joinable()) thL_.join();
    if (thR_.joinable()) thR_.join();

    {
      std::lock_guard<std::mutex> lk(mu_in_);
      inboxL_.clear();
      inboxR_.clear();
    }

    left_.Stop(); right_.Stop();
    left_.Close(); right_.Close();
    if (cm_) cm_->stop();
    cm_.reset();
  }

  // manual set (ns)
  void SetCam1OffsetNs(int64_t ns) {
    cam1_ts_offset_ns_.store(ns, std::memory_order_relaxed);
  }
  int64_t Cam1OffsetNs() const {
    return cam1_ts_offset_ns_.load(std::memory_order_relaxed);
  }

  // Auto estimate cam1 offset without requiring paired frames.
  // Returns true if estimated, false if timeout/insufficient samples.
  bool AutoEstimateAndSetOffset(int samples, int timeout_ms) {
    if (samples <= 10) samples = 10;

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
      if (est_ready_.load()) break;
      if (NowNs() > t_deadline) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    bool ok = est_ready_.load();
    estimating_.store(false);

    if (ok) {
      const int64_t off = cam1_ts_offset_ns_.load();
      std::cerr << "[camoff] estimated cam1_ts_offset_ns=" << off
                << " (" << (off / 1e6) << " ms)\n";
    } else {
      std::cerr << "[camoff] estimate failed/timeout. cam1_ts_offset_ns stays "
                << cam1_ts_offset_ns_.load() << "\n";
    }
    return ok;
  }

  bool GrabPair(FrameItem &L, FrameItem &R, int timeout_ms = 1000) {
    std::unique_lock<std::mutex> lk(mu_pair_);
    if (!cv_pair_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                           [&]{ return !paired_.empty() || !g_running.load(); })) {
      return false;
    }
    if (paired_.empty()) return false;
    auto &p = paired_.front();
    L = std::move(p.first);
    R = std::move(p.second);
    paired_.pop_front();
    return true;
  }

  int64_t LastDtMs() const { return last_dt_ms_.load(); }
  uint32_t LastSeq() const { return last_seq_.load(); }

  size_t PendL() const { std::lock_guard<std::mutex> lk(mu_pair_); return qL_.size(); }
  size_t PendR() const { std::lock_guard<std::mutex> lk(mu_pair_); return qR_.size(); }

private:
  void PushInbox(int cam_idx, FrameItem &&fi) {
    {
      std::lock_guard<std::mutex> lk(mu_in_);
      if (cam_idx == 0) inboxL_.push_back(std::move(fi));
      else inboxR_.push_back(std::move(fi));

      const size_t kMaxInbox = 12;
      auto &inb = (cam_idx == 0) ? inboxL_ : inboxR_;
      while (inb.size() > kMaxInbox) inb.pop_front();
    }
    cv_in_.notify_all();
  }

  bool PopInbox(int cam_idx, FrameItem &out) {
    std::unique_lock<std::mutex> lk(mu_in_);
    cv_in_.wait(lk, [&]{
      if (!running_.load() || !g_running.load()) return true;
      return cam_idx == 0 ? !inboxL_.empty() : !inboxR_.empty();
    });

    if (!running_.load() || !g_running.load()) return false;

    auto &inb = (cam_idx == 0) ? inboxL_ : inboxR_;
    if (inb.empty()) return false;

    out = std::move(inb.front());
    inb.pop_front();
    return true;
  }

  void RecordForEstimateLocked(int cam_idx, uint64_t ts) {
    // store monotonic timestamps
    if (cam_idx == 0) tsL_.push_back((int64_t)ts);
    else tsR_.push_back((int64_t)ts);

    // keep bounded
    const size_t kMax = (size_t)need_samples_ + 50;
    if (tsL_.size() > kMax) tsL_.erase(tsL_.begin(), tsL_.begin() + (tsL_.size() - kMax));
    if (tsR_.size() > kMax) tsR_.erase(tsR_.begin(), tsR_.begin() + (tsR_.size() - kMax));

    // if enough, compute offset
    if ((int)tsL_.size() >= need_samples_ && (int)tsR_.size() >= need_samples_) {
      // compute nearest-neighbor dt: (R - L)
      std::vector<int64_t> dts;
      dts.reserve(std::min(tsL_.size(), tsR_.size()));

      // two-pointer nearest neighbor (both are increasing)
      size_t j = 0;
      for (size_t i = 0; i < tsL_.size(); ++i) {
        int64_t tL = tsL_[i];
        while (j + 1 < tsR_.size() && Abs64(tsR_[j + 1] - tL) <= Abs64(tsR_[j] - tL)) j++;
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

  void ConsumeLoop(int cam_idx) {
    while (running_.load() && g_running.load()) {
      FrameItem fi;
      if (!PopInbox(cam_idx, fi)) continue;

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

  bool TryPairLocked() {
    if (qL_.empty() || qR_.empty()) return false;

    bool anchor_left = (qL_.front().ts_ns <= qR_.front().ts_ns);
    auto &Aq = anchor_left ? qL_ : qR_;
    auto &Bq = anchor_left ? qR_ : qL_;

    const uint64_t ats = Aq.front().ts_ns;

    size_t best = 0;
    int64_t best_dt = INT64_MAX;
    for (size_t i = 0; i < Bq.size(); ++i) {
      int64_t dt = (int64_t)Bq[i].ts_ns - (int64_t)ats;
      int64_t adt = Abs64(dt);
      if (adt < best_dt) { best_dt = adt; best = i; }
      if ((int64_t)Bq[i].ts_ns > (int64_t)ats && adt > best_dt) break;
    }

    if (best_dt > pair_thresh_ns_) {
      int64_t dt_lr = (int64_t)qR_.front().ts_ns - (int64_t)qL_.front().ts_ns;
      if (dt_lr > 0) qL_.pop_front();
      else qR_.pop_front();
      return true;
    }

    FrameItem A = std::move(Aq.front());
    FrameItem B = std::move(Bq[best]);
    Aq.pop_front();
    Bq.erase(Bq.begin() + best);

    FrameItem L, R;
    if (anchor_left) { L = std::move(A); R = std::move(B); }
    else { L = std::move(B); R = std::move(A); }

    last_dt_ms_.store(((int64_t)R.ts_ns - (int64_t)L.ts_ns) / 1'000'000);
    last_seq_.store(L.seq);

    paired_.push_back({std::move(L), std::move(R)});
    while ((int)paired_.size() > max_pair_queue_) paired_.pop_front();
    cv_pair_.notify_one();
    return true;
  }

  void PurgeOldLocked() {
    uint64_t newest = 0;
    if (!qL_.empty()) newest = std::max<uint64_t>(newest, qL_.back().ts_ns);
    if (!qR_.empty()) newest = std::max<uint64_t>(newest, qR_.back().ts_ns);

    auto purge = [&](std::deque<FrameItem> &q) {
      while (!q.empty() && (int64_t)(newest - q.front().ts_ns) > keep_window_ns_) q.pop_front();
    };
    purge(qL_);
    purge(qR_);
  }

  void OnFrameLocked(FrameItem &&fi) {
    if (fi.cam_index == 0) qL_.push_back(std::move(fi));
    else qR_.push_back(std::move(fi));

    PurgeOldLocked();
    while (TryPairLocked()) {}
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

// ---------------- IMU buffer ----------------
class ImuBuffer {
public:
  void Push(const ImuSample &s) {
    std::lock_guard<std::mutex> lk(mu_);
    q_.push_back(s);

    const int64_t keep_ns = keep_sec_ * 1000000000LL;
    while (!q_.empty() && (q_.back().t_ns - q_.front().t_ns) > keep_ns) {
      q_.pop_front();
      if (last_used_idx_ > 0) last_used_idx_--;
    }
  }

  std::vector<ORB_SLAM3::IMU::Point> PopBetweenNs(int64_t t0_ns, int64_t t1_ns,
                                                  int64_t slack_before_ns,
                                                  int64_t slack_after_ns) {
    std::vector<ORB_SLAM3::IMU::Point> out;
    std::lock_guard<std::mutex> lk(mu_);

    if (q_.empty()) return out;
    if (t1_ns < t0_ns) return out;

    const int64_t a0 = t0_ns - slack_before_ns;
    const int64_t a1 = t1_ns + slack_after_ns;

    size_t i = std::min(last_used_idx_, q_.size());
    while (i < q_.size() && q_[i].t_ns <= a0) i++;

    size_t j = i;
    while (j < q_.size() && q_[j].t_ns <= a1) {
      const auto &s = q_[j];
      if (s.t_ns > t0_ns && s.t_ns <= t1_ns) {
        const double ts = (double)s.t_ns * 1e-9;
        out.emplace_back(cv::Point3f(s.ax,s.ay,s.az),
                         cv::Point3f(s.gx,s.gy,s.gz),
                         ts);
      }
      j++;
    }

    if (!out.empty()) {
      size_t k = i;
      while (k < q_.size() && q_[k].t_ns <= t1_ns) k++;
      last_used_idx_ = k;
    }

    const int64_t purge_before = t0_ns - purge_margin_ns_;
    while (!q_.empty() && q_.front().t_ns < purge_before) {
      q_.pop_front();
      if (last_used_idx_ > 0) last_used_idx_--;
    }

    return out;
  }

  size_t Size() const {
    std::lock_guard<std::mutex> lk(mu_);
    return q_.size();
  }

  bool PeekFirstLast(int64_t &t_first, int64_t &t_last) const {
    std::lock_guard<std::mutex> lk(mu_);
    if (q_.empty()) return false;
    t_first = q_.front().t_ns;
    t_last  = q_.back().t_ns;
    return true;
  }

private:
  mutable std::mutex mu_;
  std::deque<ImuSample> q_;
  size_t last_used_idx_{0};

  int keep_sec_{5};
  int64_t purge_margin_ns_{20000000};  // 20ms
};


// ---------------- UDP image sender ----------------
class UdpImageSender {
public:
  struct Job {
    int cam_index;          // 0=L, 1=R
    uint32_t seq;           // sequence for debug
    double frame_t;         // seconds
    cv::Mat gray;           // CV_8UC1
  };

  bool Open(const std::string& ip, int port,
            int jpeg_quality = 80,
            int max_payload = 1200,      // safe under typical MTU (1500)
            int max_queue = 4) {
    jpeg_quality_ = std::max(10, std::min(95, jpeg_quality));
    max_payload_ = std::max(400, max_payload);
    max_queue_ = std::max(1, max_queue);

    sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
      std::cerr << "[udp] socket() failed: " << strerror(errno) << "\n";
      return false;
    }

    memset(&dst_, 0, sizeof(dst_));
    dst_.sin_family = AF_INET;
    dst_.sin_port = htons((uint16_t)port);
    if (::inet_pton(AF_INET, ip.c_str(), &dst_.sin_addr) != 1) {
      std::cerr << "[udp] inet_pton failed for " << ip << "\n";
      ::close(sock_); sock_ = -1;
      return false;
    }

    running_.store(true);
    th_ = std::thread([&]{ Loop(); });
    return true;
  }

  void Close() {
    running_.store(false);
    cv_.notify_all();
    if (th_.joinable()) th_.join();
    if (sock_ >= 0) { ::close(sock_); sock_ = -1; }
    {
      std::lock_guard<std::mutex> lk(mu_);
      q_.clear();
    }
  }

  // called from SLAM thread (non-blocking-ish)
  void Enqueue(int cam_index, uint32_t seq, double frame_t, const cv::Mat& gray) {
    if (sock_ < 0) return;
    Job job;
    job.cam_index = cam_index;
    job.seq = seq;
    job.frame_t = frame_t;
    job.gray = gray.clone();  // IMPORTANT: own data (libcamera buffer will be reused)

    {
      std::lock_guard<std::mutex> lk(mu_);
      q_.push_back(std::move(job));
      while ((int)q_.size() > max_queue_) q_.pop_front(); // drop old to keep real-time
    }
    cv_.notify_one();
  }

private:
#pragma pack(push, 1)
  struct PacketHeader {
    uint32_t magic;       // 'VSIM' 0x5643494D (or any)
    uint16_t version;     // 1
    uint8_t  cam_index;   // 0/1
    uint8_t  flags;       // reserved
    uint32_t seq;         // camera sequence
    double   frame_t;     // seconds
    uint32_t frame_id;    // incremental id for this sender
    uint16_t chunk_idx;   // 0..chunk_cnt-1
    uint16_t chunk_cnt;   // total chunks
    uint32_t total_size;  // jpeg bytes total
    uint32_t chunk_size;  // bytes in this packet payload
  };
#pragma pack(pop)

  void Loop() {
    std::vector<uchar> jpeg;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};

    while (running_.load()) {
      Job job;
      {
        std::unique_lock<std::mutex> lk(mu_);
        cv_.wait(lk, [&]{ return !running_.load() || !q_.empty(); });
        if (!running_.load()) break;
        job = std::move(q_.front());
        q_.pop_front();
      }

      // encode
      jpeg.clear();
      try {
        cv::imencode(".jpg", job.gray, jpeg, params);
      } catch (const std::exception& e) {
        std::cerr << "[udp] imencode exception: " << e.what() << "\n";
        continue;
      }
      if (jpeg.empty()) continue;

      const uint32_t total = (uint32_t)jpeg.size();
      const uint16_t chunks = (uint16_t)((total + max_payload_ - 1) / max_payload_);
      const uint32_t fid = frame_id_.fetch_add(1, std::memory_order_relaxed);

      for (uint16_t ci = 0; ci < chunks; ++ci) {
        const uint32_t off = (uint32_t)ci * (uint32_t)max_payload_;
        const uint32_t left = total - off;
        const uint32_t pay = (left > (uint32_t)max_payload_) ? (uint32_t)max_payload_ : left;

        PacketHeader h{};
        h.magic = 0x5643494D; // 'VCIM' just a tag
        h.version = 1;
        h.cam_index = (uint8_t)job.cam_index;
        h.flags = 0;
        h.seq = job.seq;
        h.frame_t = job.frame_t;
        h.frame_id = fid;
        h.chunk_idx = ci;
        h.chunk_cnt = chunks;
        h.total_size = total;
        h.chunk_size = pay;

        // build packet buffer: header + payload
        std::vector<uint8_t> pkt(sizeof(PacketHeader) + pay);
        memcpy(pkt.data(), &h, sizeof(PacketHeader));
        memcpy(pkt.data() + sizeof(PacketHeader), jpeg.data() + off, pay);

        ssize_t sent = ::sendto(sock_, pkt.data(), pkt.size(), 0,
                                (sockaddr*)&dst_, sizeof(dst_));
        (void)sent; // ignore drop; UDP is best-effort
      }
    }
  }

  int sock_{-1};
  sockaddr_in dst_{};

  int jpeg_quality_{80};
  int max_payload_{1200};
  int max_queue_{4};

  std::atomic<bool> running_{false};
  std::thread th_;
  std::mutex mu_;
  std::condition_variable cv_;
  std::deque<Job> q_;

  std::atomic<uint32_t> frame_id_{1};
};


// ---------------- args ----------------
static std::string GetArgS(int argc, char **argv, const char *name, const char *def) {
  for (int i = 1; i + 1 < argc; i++) if (std::string(argv[i]) == name) return argv[i + 1];
  return def;
}
static int GetArgI(int argc, char **argv, const char *name, int def) {
  for (int i = 1; i + 1 < argc; i++) if (std::string(argv[i]) == name) return std::stoi(argv[i + 1]);
  return def;
}
static int64_t GetArgI64(int argc, char **argv, const char *name, int64_t def) {
  for (int i = 1; i + 1 < argc; i++) if (std::string(argv[i]) == name) return std::stoll(argv[i + 1]);
  return def;
}
static float GetArgF(int argc, char **argv, const char *name, float def) {
  for (int i = 1; i + 1 < argc; i++) if (std::string(argv[i]) == name) return std::stof(argv[i + 1]);
  return def;
}
static bool HasArg(int argc, char **argv, const char *name) {
  for (int i = 1; i < argc; i++) if (std::string(argv[i]) == name) return true;
  return false;
}
static uint8_t ParseU8HexOrDec(const std::string &s, uint8_t def) {
  try {
    int base = 10;
    std::string t = s;
    if (t.size() > 2 && t[0] == '0' && (t[1] == 'x' || t[1] == 'X')) base = 16;
    int v = std::stoi(t, nullptr, base);
    if (v < 0 || v > 255) return def;
    return (uint8_t)v;
  } catch (...) {
    return def;
  }
}

int main(int argc, char **argv) {
  signal(SIGINT, SigIntHandler);
  signal(SIGTERM, SigIntHandler);

  const std::string vocab = GetArgS(argc, argv, "--vocab", "ORBvoc.txt");
  const std::string settings = GetArgS(argc, argv, "--settings", "stereo_inertial.yaml");

  // camera
  const int w = GetArgI(argc, argv, "--w", 1280);
  const int h = GetArgI(argc, argv, "--h", 800);
  const int fps = GetArgI(argc, argv, "--fps", 30);
  const bool ae_disable = !HasArg(argc, argv, "--ae");
  const int exposure_us = GetArgI(argc, argv, "--exp-us", 5000);
  const float gain = GetArgF(argc, argv, "--gain", 8.0f);

  const bool request_y8 = !HasArg(argc, argv, "--no-y8");
  const bool r16_norm = HasArg(argc, argv, "--r16-norm");

  // stereo pairing
  const int pair_ms = GetArgI(argc, argv, "--pair-ms", 2);
  const int keep_ms = GetArgI(argc, argv, "--keep-ms", 120);
  // udp
  const bool udp_enable = HasArg(argc, argv, "--udp");
  const std::string udp_ip = GetArgS(argc, argv, "--udp-ip", "192.168.1.10");
  const int udp_port = GetArgI(argc, argv, "--udp-port", 5000);
  const int udp_jpeg_q = GetArgI(argc, argv, "--udp-jpeg-q", 80);
  const int udp_payload = GetArgI(argc, argv, "--udp-payload", 1200);
  const int udp_q = GetArgI(argc, argv, "--udp-queue", 4);

  // --- NEW: camera timestamp compensation ---
  const int cam1_ts_offset_ms = GetArgI(argc, argv, "--cam1-ts-offset-ms", 0);
  const bool auto_cam_offset = HasArg(argc, argv, "--auto-cam-offset") || (cam1_ts_offset_ms == 0);
  const int auto_offset_samples = GetArgI(argc, argv, "--auto-offset-samples", 120);
  const int auto_offset_timeout_ms = GetArgI(argc, argv, "--auto-offset-timeout-ms", 3000);

  // imu
  const std::string spi_dev = GetArgS(argc, argv, "--spi", "/dev/spidev0.0");
  const uint32_t spi_speed = (uint32_t)GetArgI(argc, argv, "--speed", 8000000);
  const uint8_t spi_mode = (uint8_t)GetArgI(argc, argv, "--mode", 0);
  const uint8_t spi_bits = (uint8_t)GetArgI(argc, argv, "--bits", 8);

  const std::string gpiochip = GetArgS(argc, argv, "--gpiochip", "/dev/gpiochip0");
  const unsigned drdy_line = (unsigned)GetArgI(argc, argv, "--drdy", 24);
  const int imu_hz = GetArgI(argc, argv, "--imu-hz", 200);

  const int accel_fs_g = GetArgI(argc, argv, "--accel-fs", 16);
  const int gyro_fs_dps = GetArgI(argc, argv, "--gyro-fs", 2000);

  uint8_t imu_start_reg = 0x1F;
  {
    std::string s = GetArgS(argc, argv, "--imu-start-reg", "0x1F");
    imu_start_reg = ParseU8HexOrDec(s, 0x1F);
  }

  // timing / robust
  const int64_t off_reject_ns = GetArgI64(argc, argv, "--off-reject-ns", 10'000'000); // 10ms
  const bool allow_empty_imu = HasArg(argc, argv, "--allow-empty-imu");

  // realtime (optional)
  const bool rt_imu = HasArg(argc, argv, "--rt-imu");
  const int rt_prio = GetArgI(argc, argv, "--rt-prio", 60);

  std::cerr << "ORB vocab=" << vocab << "\nsettings=" << settings << "\n";
  std::cerr << "cam " << w << "x" << h << " @" << fps
            << " ae_disable=" << (ae_disable ? "true":"false")
            << " exp_us=" << exposure_us << " gain=" << gain
            << " request_y8=" << (request_y8 ? "Y":"N")
            << " r16_norm=" << (r16_norm ? "Y":"N")
            << "\n";
  std::cerr << "pair_thresh=" << pair_ms << "ms keep_window=" << keep_ms << "ms\n";
  std::cerr << "cam1_ts_offset_ms=" << cam1_ts_offset_ms
            << " auto_cam_offset=" << (auto_cam_offset ? "Y":"N")
            << " auto_samples=" << auto_offset_samples
            << " auto_timeout_ms=" << auto_offset_timeout_ms << "\n";

  std::cerr << "imu spi=" << spi_dev << " speed=" << spi_speed
            << " mode=" << int(spi_mode) << " bits=" << int(spi_bits)
            << " drdy=" << gpiochip << ":" << drdy_line
            << " imu_hz=" << imu_hz
            << " imu_start_reg=0x" << std::hex << int(imu_start_reg) << std::dec
            << " accel_fs=" << accel_fs_g << "g"
            << " gyro_fs=" << gyro_fs_dps << "dps"
            << "\n";
std::cerr << "udp_enable=" << (udp_enable ? "Y":"N")
          << " udp_ip=" << udp_ip
          << " udp_port=" << udp_port
          << " jpeg_q=" << udp_jpeg_q
          << " payload=" << udp_payload
          << " queue=" << udp_q
          << "\n";

  // ORB-SLAM3 init
  ORB_SLAM3::System SLAM(vocab, settings, ORB_SLAM3::System::STEREO, false);
  MavlinkSerial mav("/dev/ttyAMA0", 921600);
  // UDP sender (optional)
  UdpImageSender udp;
  if (udp_enable) {
    if (!udp.Open(udp_ip, udp_port, udp_jpeg_q, udp_payload, udp_q)) {
      std::cerr << "[udp] open failed, continue without udp.\n";
    } else {
      std::cerr << "[udp] sending to " << udp_ip << ":" << udp_port << "\n";
    }
  }

  // IMU thread
  ImuBuffer imu_buf;
  std::atomic<bool> imu_ok{false};
  std::atomic<uint64_t> imu_cnt{0};
  std::atomic<uint64_t> imu_drop{0};

  ImuScale imu_scale{};
  std::atomic<float> accel_lsb_per_g{imu_scale.accel_lsb_per_g};
  std::atomic<float> gyro_lsb_per_dps{imu_scale.gyro_lsb_per_dps};

  std::thread imu_th([&](){
    if (rt_imu) {
      if (!SetThreadRealtime(rt_prio)) {
        std::cerr << "[imu] SetThreadRealtime failed (need CAP_SYS_NICE/root). continue.\n";
      } else {
        std::cerr << "[imu] realtime SCHED_FIFO prio=" << rt_prio << "\n";
      }
    }

    SpiDev spi(spi_dev);
    if (!spi.Open(spi_speed, spi_mode, spi_bits)) return;

    ImuScale sc{};
    if (!IcmResetAndConfig(spi, imu_hz, accel_fs_g, gyro_fs_dps, sc)) return;

    accel_lsb_per_g.store(sc.accel_lsb_per_g);
    gyro_lsb_per_dps.store(sc.gyro_lsb_per_dps);
    std::cerr << "[imu] scale accel_lsb/g=" << sc.accel_lsb_per_g
              << " gyro_lsb/dps=" << sc.gyro_lsb_per_dps << "\n";

    DrdyGpio drdy;
    if (!drdy.Open(gpiochip, drdy_line)) return;

    imu_ok.store(true);

    uint8_t raw12[12]{};
    uint8_t st = 0;
    spi.ReadReg(REG_INT_STATUS, st);

    while (g_running.load()) {
      int64_t t_irq_ns = 0;
      if (!drdy.WaitTs(1000, t_irq_ns)) continue;

      ImuSample s{};
      s.t_ns = t_irq_ns;

      spi.ReadReg(REG_INT_STATUS, st);

      if (!spi.ReadRegs(imu_start_reg, raw12, sizeof(raw12))) {
        imu_drop.fetch_add(1, std::memory_order_relaxed);
        continue;
      }

      ImuScale sc2{};
      sc2.accel_lsb_per_g = accel_lsb_per_g.load();
      sc2.gyro_lsb_per_dps = gyro_lsb_per_dps.load();
      ConvertRaw12_AccelGyro_ToSI(raw12, sc2, s);

      // static uint64_t imuCnt = 0;
      // if (imuCnt % 200 == 0) {

      //   std::cerr.setf(std::ios::fixed);
      //   std::cerr << std::setprecision(6);

      //   std::cerr << "[IMU] a(m/s^2)=[" << s.ax << "," << s.ay << "," << s.az << "]"
      //             << " g(rad/s)=[" << s.gx << "," << s.gy << "," << s.gz << "]"
      //             << "\n";
      // }
      // imuCnt++;

      imu_buf.Push(s);
      imu_cnt.fetch_add(1, std::memory_order_relaxed);
    }
  });

  // Stereo camera
  LibcameraStereoOV9281_TsPair cam;
  if (!cam.Open(
        w, h, fps,
        ae_disable, exposure_us, gain,
        request_y8,
        (int64_t)pair_ms * 1'000'000LL,
        (int64_t)keep_ms * 1'000'000LL,
        8,
        r16_norm)) {
    g_running.store(false);
    if (imu_th.joinable()) imu_th.join();
    return 1;
  }

  // Apply manual or auto cam1 offset
  if (cam1_ts_offset_ms != 0) {
    cam.SetCam1OffsetNs((int64_t)cam1_ts_offset_ms * 1'000'000LL);
    std::cerr << "[camoff] manual cam1_ts_offset_ms=" << cam1_ts_offset_ms << "\n";
  } else if (auto_cam_offset) {
    std::cerr << "[camoff] auto estimating cam1 offset... samples=" << auto_offset_samples
              << " timeout_ms=" << auto_offset_timeout_ms << "\n";
    cam.AutoEstimateAndSetOffset(auto_offset_samples, auto_offset_timeout_ms);
  }

  // Warmup: estimate ts_offset_ns (libcamera timestamp domain -> NowNs domain)
  // Now that we can pair, this should work.
  std::vector<int64_t> offs;
  offs.reserve(120);

  int warm_pairs = 0;
  int64_t last_print = NowNs();
  while (g_running.load() && warm_pairs < 60) {
    FrameItem L, R;
    if (!cam.GrabPair(L, R, 1000)) {
      const int64_t now = NowNs();
      if (now - last_print > 1'000'000'000LL) {
        last_print = now;
        std::cerr << "[warmup] waiting pairs..."
                  << " last_seq=" << cam.LastSeq()
                  << " dt_ms=" << cam.LastDtMs()
                  << " pendL=" << cam.PendL()
                  << " pendR=" << cam.PendR()
                  << " imu_ok=" << (imu_ok.load() ? "Y":"N")
                  << " imu_buf=" << imu_buf.Size()
                  << "\n";
      }
      continue;
    }
    int64_t cam_ts = (int64_t)((L.ts_ns + R.ts_ns) / 2);
    int64_t arrive = (L.arrive_ns && R.arrive_ns) ? (L.arrive_ns + R.arrive_ns) / 2 : NowNs();
    offs.push_back(arrive - cam_ts);
    warm_pairs++;
  }

  int64_t ts_offset_ns = Median(offs);
  if (!offs.empty()) {
    int64_t mino = *std::min_element(offs.begin(), offs.end());
    int64_t maxo = *std::max_element(offs.begin(), offs.end());
    std::cerr << "Init ts_offset_ns(median)=" << ts_offset_ns
              << " range=[" << mino << "," << maxo << "] ns\n";
  } else {
    std::cerr << "Init ts_offset_ns failed (no warmup pairs). Using 0.\n";
    ts_offset_ns = 0;
  }

  // Main loop
  int64_t last_frame_ns = 0;
  uint64_t frame_cnt_1s = 0;
  uint64_t last_imu_cnt = imu_cnt.load();
  uint64_t last_imu_drop = imu_drop.load();
  int64_t last_stat_ns = NowNs();

  const int64_t frame_step_ns = (int64_t)(1000000000LL / std::max(1, fps));
  const int64_t imu_dt_ns = (int64_t)(1000000000LL / std::max(1, imu_hz));
  const int64_t slack_before_ns = std::max<int64_t>(2 * imu_dt_ns, 5'000'000);
  const int64_t slack_after_ns  = std::max<int64_t>(2 * imu_dt_ns, 5'000'000);

  std::cerr << "t_s, tx,ty,tz, qw,qx,qy,qz\n";
  std::ios::sync_with_stdio(false);
  std::cout.tie(nullptr);
  std::cout.setf(std::ios::fixed);
  std::cout << std::setprecision(6);

  while (g_running.load()) {
    FrameItem L, R;
    if (!cam.GrabPair(L, R, 1000)) {
      const int64_t now = NowNs();
      if (now - last_stat_ns > 1'000'000'000LL) {
        last_stat_ns = now;
        const uint64_t ic = imu_cnt.load();
        const uint64_t id = imu_drop.load();
        std::cerr << "[wd] no pair 1s"
                  << " last_seq=" << cam.LastSeq()
                  << " dt_ms=" << cam.LastDtMs()
                  << " pendL=" << cam.PendL()
                  << " pendR=" << cam.PendR()
                  << " imu_hz~=" << (ic - last_imu_cnt)
                  << " imu_drop~=" << (id - last_imu_drop)
                  << " imu_buf=" << imu_buf.Size()
                  << "\n";
        last_imu_cnt = ic;
        last_imu_drop = id;
      }
      continue;
    }

    frame_cnt_1s++;

    const int64_t cam_ts = (int64_t)((L.ts_ns + R.ts_ns) / 2);
    const int64_t arrive = (L.arrive_ns && R.arrive_ns) ? (L.arrive_ns + R.arrive_ns) / 2 : NowNs();
    const int64_t off_meas = arrive - cam_ts;

    const int64_t err = off_meas - ts_offset_ns;
    if (Abs64(err) < off_reject_ns) {
      ts_offset_ns = ts_offset_ns + (err >> 6); // alpha=1/64
    }

    int64_t frame_ns = cam_ts + ts_offset_ns;

    if (last_frame_ns != 0 && frame_ns <= last_frame_ns) {
      frame_ns = last_frame_ns + frame_step_ns;
    }

    std::vector<ORB_SLAM3::IMU::Point> vImu;
    if (last_frame_ns != 0) {
      vImu = imu_buf.PopBetweenNs(last_frame_ns, frame_ns, slack_before_ns, slack_after_ns);
    }
    const int64_t prev_frame_ns = last_frame_ns;
    last_frame_ns = frame_ns;

    const double frame_t = (double)frame_ns * 1e-9;

    const int64_t now = NowNs();
    if (now - last_stat_ns > 1'000'000'000LL) {
      last_stat_ns = now;
      const uint64_t ic = imu_cnt.load();
      const uint64_t id = imu_drop.load();
      // std::cerr << "[STAT] fps~=" << frame_cnt_1s
      //           << " dt_ms=" << cam.LastDtMs()
      //           << " imu_hz~=" << (ic - last_imu_cnt)
      //           << " imu_drop~=" << (id - last_imu_drop)
      //           << " imu_buf=" << imu_buf.Size()
      //           << " last_vImu=" << vImu.size()
      //           << " off_ns=" << ts_offset_ns
      //           << "\n";
      frame_cnt_1s = 0;
      last_imu_cnt = ic;
      last_imu_drop = id;
    }

    if (prev_frame_ns != 0 && vImu.empty()) {
      int64_t t_first=0, t_last=0;
      bool ok = imu_buf.PeekFirstLast(t_first, t_last);
      std::cerr << "[imu] EMPTY vImu"
                << " t0=" << (double)prev_frame_ns * 1e-9
                << " t1=" << (double)frame_ns * 1e-9
                << " dt_ms=" << (frame_ns - prev_frame_ns) / 1e6
                << " buf=" << imu_buf.Size();
      if (ok) {
        std::cerr << " imu_first=" << (double)t_first * 1e-9
                  << " imu_last="  << (double)t_last  * 1e-9;
      }
      std::cerr << "\n";

      if (!allow_empty_imu) continue;
    }

    if (udp_enable) {
      udp.Enqueue(0, L.seq, frame_t, L.gray);
      udp.Enqueue(1, R.seq, frame_t, R.gray);
    }

    Sophus::SE3f Tcw = SLAM.TrackStereo(L.gray, R.gray, frame_t, vImu);
    int state = SLAM.GetTrackingState();   // ORB-SLAM3 有这个接口
    if (state != ORB_SLAM3::Tracking::OK) {
        std::cout << "[TRACK] state=" << state
                  << " (not OK), skip pose\n";
    }
    Sophus::SE3f Twc = Tcw.inverse();

    const Eigen::Vector3f t = Twc.translation();
    const Eigen::Quaternionf q(Twc.so3().unit_quaternion());
    MavlinkSerial::Pose p_ned;
    p_ned.x = t.x();
    p_ned.y = t.y();
    p_ned.z = t.z();
    p_ned.qw = q.w();
    p_ned.qx = q.x();
    p_ned.qy = q.y();
    p_ned.qz = q.z();
    MavlinkSerial::normalizeQuat(p_ned.qw,p_ned.qx,p_ned.qy,p_ned.qz);

    // auto p_ned = MavlinkSerial::enuToNed(p_enu);

    uint64_t t_us = mono_time_us();
    mav.sendOdometry(t_us, p_ned, MAV_FRAME_LOCAL_NED, MAV_FRAME_BODY_FRD);

    static uint64_t posCnt = 0;
    if (posCnt % 30 == 0) {
      std::cout << "[POSE]" << frame_t << ",\033[32m"
                << p_ned.x << "," << p_ned.y << "," << p_ned.z << "\033[0m,"
                << p_ned.qw << "," << p_ned.qx << "," << p_ned.qy << "," << p_ned.qz
                << std::endl;
    }
    posCnt++;
  }

  cam.Close();
  g_running.store(false);
  if (imu_th.joinable()) imu_th.join();
  SLAM.Shutdown();
  return 0;
}
