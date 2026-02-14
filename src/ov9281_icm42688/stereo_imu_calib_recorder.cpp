// stereo_imu_calib_recorder.cpp
//
// 功能：双目(OV9281/libcamera PiSP, 强制 ISP 输出 YUV420 仅取Y平面) + ICM42688(SPI+DRDY) 标定数据录制
// 输出：EuRoC 风格目录：
//   <out>/mav0/cam0/data/*.png + <out>/mav0/cam0/data.csv
//   <out>/mav0/cam1/data/*.png + <out>/mav0/cam1/data.csv
//   <out>/mav0/imu0/data.csv
//
// 编译（示例）:
// g++ stereo_imu_calib_recorder.cpp -O3 -std=c++17 \
//   `pkg-config --cflags --libs libcamera opencv4 libgpiod` \
//   -lpthread -o stereo_imu_calib_recorder
//
// 运行（示例）:
// ./stereo_imu_calib_recorder \
//   --out /home/ltz/calib_01 \
//   --w 1280 --h 800 --fps 30 \
//   --exp-us 3000 --gain 2 --ae-disable \
//   --spi /dev/spidev0.0 --speed 8000000 --mode 0 --bits 8 \
//   --gpiochip /dev/gpiochip0 --drdy 24 --imu-hz 400
//
// 改进点（400Hz稳定版）：
// 1) setvbuf() 增大 csv 缓冲，降低写盘抖动
// 2) DRDY 一次读多条 edge event，避免 event 堆积导致 IMU 时间戳落后
// 3) 支持 --imu-flush-every N：每 N 行 flush 一次（默认 800）

#include <libcamera/libcamera.h>
#include <libcamera/framebuffer_allocator.h>

#include <opencv2/opencv.hpp>

#include <gpiod.h>
#include <linux/spi/spidev.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <filesystem>
#include <algorithm>

using namespace libcamera;
namespace fs = std::filesystem;

static std::atomic<bool> g_running{true};
static void SigIntHandler(int) { g_running.store(false); }

// ---------------- time util ----------------
static int64_t NowNs() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
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
static float GetArgF(int argc, char **argv, const char *name, float def) {
  for (int i = 1; i + 1 < argc; i++) if (std::string(argv[i]) == name) return std::stof(argv[i + 1]);
  return def;
}
static bool HasArg(int argc, char **argv, const char *name) {
  for (int i = 1; i < argc; i++) if (std::string(argv[i]) == name) return true;
  return false;
}

// ---------------- IMU (ICM42688) ----------------
static inline int16_t Be16ToI16(uint8_t hi, uint8_t lo) {
  return (int16_t)((uint16_t(hi) << 8) | uint16_t(lo));
}

// Bank0 registers (subset)
static constexpr uint8_t REG_DEVICE_CONFIG   = 0x11;
static constexpr uint8_t REG_INT_CONFIG      = 0x14;
static constexpr uint8_t REG_TEMP_DATA1      = 0x1D;
static constexpr uint8_t REG_INT_STATUS      = 0x2D;
static constexpr uint8_t REG_PWR_MGMT0       = 0x4E;
static constexpr uint8_t REG_GYRO_CONFIG0    = 0x4F;
static constexpr uint8_t REG_ACCEL_CONFIG0   = 0x50;
static constexpr uint8_t REG_INT_CONFIG1     = 0x64;
static constexpr uint8_t REG_INT_SOURCE0     = 0x65;

static constexpr uint8_t SPI_READ_MASK  = 0x80;

struct ImuSample {
  int64_t t_ns{};          // DRDY timestamp (CLOCK_MONOTONIC domain)
  float ax{}, ay{}, az{};  // m/s^2
  float gx{}, gy{}, gz{};  // rad/s
};

class SpiDev {
public:
  explicit SpiDev(std::string dev) : dev_(std::move(dev)) {}
  ~SpiDev() { if (fd_ >= 0) ::close(fd_); }

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
  bool Open(const std::string &chip_path, unsigned line_offset, int max_burst = 256) {
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

    max_burst_ = std::max(1, max_burst);
    evbuf_ = gpiod_edge_event_buffer_new((size_t)max_burst_);
    return evbuf_ != nullptr;
  }

  ~DrdyGpio() {
    if (evbuf_) gpiod_edge_event_buffer_free(evbuf_);
    if (request_) gpiod_line_request_release(request_);
    if (chip_) gpiod_chip_close(chip_);
  }

  // 关键：一次尽量读多条，返回“最后一条事件”的时间戳，避免堆积
  bool WaitLastTs(int timeout_ms, int64_t &ts_ns_out) {
    int64_t timeout_ns = (timeout_ms < 0) ? -1 : (int64_t)timeout_ms * 1000000LL;
    int ret = gpiod_line_request_wait_edge_events(request_, timeout_ns);
    if (ret <= 0) return false;

    int n = gpiod_line_request_read_edge_events(request_, evbuf_, (size_t)max_burst_);
    if (n <= 0) return false;

    // 取最后一个 event 的时间戳
    struct gpiod_edge_event *ev_last = gpiod_edge_event_buffer_get_event(evbuf_, (size_t)(n - 1));
    if (!ev_last) return false;

    ts_ns_out = (int64_t)gpiod_edge_event_get_timestamp_ns(ev_last);
    return true;
  }

private:
  gpiod_chip *chip_{nullptr};
  gpiod_line_request *request_{nullptr};
  gpiod_edge_event_buffer *evbuf_{nullptr};
  int max_burst_{256};
};

static bool IcmResetAndConfig(SpiDev &spi, int imu_hz) {
  auto odr_code = [&](int hz) -> uint8_t {
    switch (hz) {
      case 8000: return 0x03;
      case 4000: return 0x04;
      case 2000: return 0x05;
      case 1000: return 0x06;
      case 200: return 0x07;
      case 100:  return 0x08;
      case 50:  return 0x09;
      case 25:  return 0x0A;
      default:   return 0x0F; // 500Hz
    }
  };

  spi.WriteReg(REG_DEVICE_CONFIG, 0x01);
  usleep(100000);

  spi.WriteReg(REG_INT_CONFIG, 0x30);
  spi.WriteReg(REG_INT_SOURCE0, 0x08);
  spi.WriteReg(REG_INT_CONFIG1, 0x00);

  spi.WriteReg(REG_PWR_MGMT0, 0x0F);
  usleep(20000);

  uint8_t gyro_fs = 0x00;   // 示例：2000 dps
  uint8_t accel_fs = 0x00;  // 示例：16 g

  uint8_t odr = odr_code(imu_hz);
  uint8_t gyro_cfg0  = uint8_t((gyro_fs << 5)  | (odr & 0x0F));
  uint8_t accel_cfg0 = uint8_t((accel_fs << 5) | (odr & 0x0F));

  if (!spi.WriteReg(REG_GYRO_CONFIG0, gyro_cfg0)) return false;
  if (!spi.WriteReg(REG_ACCEL_CONFIG0, accel_cfg0)) return false;
  usleep(20000);

  return true;
}

static void ConvertRawToSI(const uint8_t raw[14], ImuSample &s) {
  int16_t ax   = Be16ToI16(raw[2], raw[3]);
  int16_t ay   = Be16ToI16(raw[4], raw[5]);
  int16_t az   = Be16ToI16(raw[6], raw[7]);
  int16_t gx   = Be16ToI16(raw[8], raw[9]);
  int16_t gy   = Be16ToI16(raw[10], raw[11]);
  int16_t gz   = Be16ToI16(raw[12], raw[13]);

  constexpr float kG = 9.80665f;
  constexpr float accel_lsb_per_g  = 2048.0f; // TODO: 按你的量程修
  constexpr float gyro_lsb_per_dps = 16.4f;   // TODO: 按你的量程修

  s.ax = (float(ax) / accel_lsb_per_g) * kG;
  s.ay = (float(ay) / accel_lsb_per_g) * kG;
  s.az = (float(az) / accel_lsb_per_g) * kG;

  constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
  s.gx = (float(gx) / gyro_lsb_per_dps) * kDeg2Rad;
  s.gy = (float(gy) / gyro_lsb_per_dps) * kDeg2Rad;
  s.gz = (float(gz) / gyro_lsb_per_dps) * kDeg2Rad;
}

// ---------------- Stereo camera (libcamera) ----------------
struct FrameItem {
  int cam_index{-1};
  uint64_t ts_ns{0};
  int64_t  arrive_ns{0};
  cv::Mat gray; // CV_8UC1
};

class LibcameraMonoCam {
public:
  bool Open(std::shared_ptr<Camera> cam, int cam_index, int w, int h, int fps,
            bool ae_disable, int exposure_us, float gain) {
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

    // 强制 ISP 输出
    sc.pixelFormat = formats::YUV420;

    const int64_t maxFrameUs = 1000000LL / std::max(1, fps);
    const int64_t minFrameUs = 1000000LL / 120;
    controls_.set(controls::FrameDurationLimits,
                  Span<const int64_t, 2>({minFrameUs, maxFrameUs}));

    // if (ae_disable) {
      controls_.set(controls::AeEnable, false);
      controls_.set(controls::ExposureTime, exposure_us);
      controls_.set(controls::AnalogueGain, gain);
    // }

    CameraConfiguration::Status status = config_->validate();
    if (status == CameraConfiguration::Invalid) {
      std::cerr << "Invalid camera configuration\n";
      return false;
    }

    if (config_->at(0).pixelFormat != formats::YUV420) {
      std::cerr << "ERROR: validate() changed pixelFormat to "
                << config_->at(0).pixelFormat.toString()
                << " (expected YUV420). Refuse to run.\n";
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
        void *addr = MMapFD(fd, len);
        if (!addr) {
          std::cerr << "mmap failed\n";
          return false;
        }
        planes.push_back({addr, len});
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

private:
  void OnRequestComplete(Request *req) {
    if (!req || req->status() == Request::RequestCancelled) return;

    auto it = req->buffers().find(stream_);
    if (it == req->buffers().end()) {
      req->reuse(Request::ReuseBuffers);
      cam_->queueRequest(req);
      return;
    }
    FrameBuffer *buf = it->second;

    auto *buffer = req->findBuffer(stream_);
    const FrameMetadata &fbmd = buffer->metadata();

    FrameItem item;
    item.cam_index = cam_index_;
    item.arrive_ns = NowNs();
    item.ts_ns = fbmd.timestamp;

    const StreamConfiguration &sc = config_->at(0);
    const int w = sc.size.width;
    const int h = sc.size.height;
    const int strideY = sc.stride;

    auto mit = bufferMaps_.find(buf);
    if (mit == bufferMaps_.end() || mit->second.empty() || !mit->second[0].addr) {
      req->reuse(Request::ReuseBuffers);
      cam_->queueRequest(req);
      return;
    }

    if (sc.pixelFormat != formats::YUV420) {
      std::cerr << "Unexpected fmt in callback: " << sc.pixelFormat.toString() << "\n";
      req->reuse(Request::ReuseBuffers);
      cam_->queueRequest(req);
      return;
    }

    uint8_t *pY = reinterpret_cast<uint8_t *>(mit->second[0].addr);
    cv::Mat yview(h, w, CV_8UC1, (void*)pY, (size_t)strideY);
    item.gray = yview.clone(); // buffer复用，必须clone

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
};

class LibcameraStereoOV9281 {
public:
  bool Open(int w, int h, int fps,
            bool ae_disable, int exposure_us, float gain,
            int max_pair_queue = 8, uint64_t pair_tol_ns = 15000000) {
    w_ = w; h_ = h; fps_ = fps;
    max_pair_queue_ = max_pair_queue;
    pair_tol_ns_ = pair_tol_ns;

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

    auto sink = [&](FrameItem &&fi) {
      std::lock_guard<std::mutex> lk(mu_);
      if (fi.cam_index == 0) qL_.push_back(std::move(fi));
      else qR_.push_back(std::move(fi));
      TryPairLocked();
    };

    if (!left_.Open(camL_, 0, w, h, fps, ae_disable, exposure_us, gain)) return false;
    if (!right_.Open(camR_, 1, w, h, fps, ae_disable, exposure_us, gain)) return false;
    left_.SetSink(sink);
    right_.SetSink(sink);

    if (!left_.Start()) return false;
    if (!right_.Start()) return false;

    std::cerr << "Left  fmt=" << left_.PixelFmt().toString()
              << " size=" << left_.SizeWH().toString()
              << " stride=" << left_.Stride() << "\n";
    std::cerr << "Right fmt=" << right_.PixelFmt().toString()
              << " size=" << right_.SizeWH().toString()
              << " stride=" << right_.Stride() << "\n";
    return true;
  }

  void Close() {
    left_.Stop(); right_.Stop();
    left_.Close(); right_.Close();
    if (cm_) cm_->stop();
    cm_.reset();
  }

  bool GrabPair(FrameItem &L, FrameItem &R, int timeout_ms = 1000) {
    std::unique_lock<std::mutex> lk(mu_);
    if (!cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                      [&]{ return !paired_raw_.empty() || !g_running.load(); })) {
      return false;
    }
    if (paired_raw_.empty()) return false;
    auto &p = paired_raw_.front();
    L = std::move(p.first);
    R = std::move(p.second);
    paired_raw_.pop_front();
    return true;
  }

  uint64_t PairTolNs() const { return pair_tol_ns_; }

private:
  void TryPairLocked() {
    while (!qL_.empty() && !qR_.empty()) {
      uint64_t tL = qL_.front().ts_ns;
      uint64_t tR = qR_.front().ts_ns;

      int64_t dt = (int64_t)tL - (int64_t)tR;
      uint64_t adt = (dt < 0) ? (uint64_t)(-dt) : (uint64_t)dt;

      if (adt <= pair_tol_ns_) {
        auto L = std::move(qL_.front()); qL_.pop_front();
        auto R = std::move(qR_.front()); qR_.pop_front();
        paired_raw_.push_back({std::move(L), std::move(R)});
        while ((int)paired_raw_.size() > max_pair_queue_) paired_raw_.pop_front();
        cv_.notify_one();
      } else {
        if (tL < tR) qL_.pop_front();
        else qR_.pop_front();
      }
    }
  }

  int w_{1280}, h_{800}, fps_{60};
  int max_pair_queue_{8};
  uint64_t pair_tol_ns_{1000000};

  std::unique_ptr<CameraManager> cm_;
  std::shared_ptr<Camera> camL_, camR_;
  LibcameraMonoCam left_, right_;

  std::mutex mu_;
  std::condition_variable cv_;
  std::deque<FrameItem> qL_, qR_;
  std::deque<std::pair<FrameItem, FrameItem>> paired_raw_;
};

// ---------------- recorder helpers ----------------
static void EnsureDir(const fs::path &p) {
  std::error_code ec;
  fs::create_directories(p, ec);
  if (ec) {
    std::cerr << "create_directories failed: " << p << " : " << ec.message() << "\n";
  }
}

static std::string TsToName(int64_t t_ns) {
  std::ostringstream oss;
  oss << t_ns << ".png";
  return oss.str();
}

struct MedianEstimator {
  static int64_t Median(std::vector<int64_t> v) {
    if (v.empty()) return 0;
    size_t mid = v.size()/2;
    std::nth_element(v.begin(), v.begin()+mid, v.end());
    return v[mid];
  }
};

static void SetupFileBuffer(FILE *f, size_t bytes) {
  if (!f) return;
  // _IOFBF: fully buffered
  setvbuf(f, nullptr, _IOFBF, bytes);
}

int main(int argc, char **argv) {
  signal(SIGINT, SigIntHandler);
  signal(SIGTERM, SigIntHandler);

  const std::string out_root = GetArgS(argc, argv, "--out", "./calib_out");

  // camera
  const int w = GetArgI(argc, argv, "--w", 1280);
  const int h = GetArgI(argc, argv, "--h", 800);
  const int fps = GetArgI(argc, argv, "--fps", 30);
  const bool ae_disable = HasArg(argc, argv, "--ae-disable");
  const int exposure_us = GetArgI(argc, argv, "--exp-us", 3000);
  const float gain = GetArgF(argc, argv, "--gain", 2.0f);

  // imu
  const std::string spi_dev = GetArgS(argc, argv, "--spi", "/dev/spidev0.0");
  const uint32_t spi_speed = (uint32_t)GetArgI(argc, argv, "--speed", 8000000);
  const uint8_t spi_mode = (uint8_t)GetArgI(argc, argv, "--mode", 0);
  const uint8_t spi_bits = (uint8_t)GetArgI(argc, argv, "--bits", 8);

  const std::string gpiochip = GetArgS(argc, argv, "--gpiochip", "/dev/gpiochip0");
  const unsigned drdy_line = (unsigned)GetArgI(argc, argv, "--drdy", 24);
  const int imu_hz = GetArgI(argc, argv, "--imu-hz", 400);

  const int imu_flush_every = GetArgI(argc, argv, "--imu-flush-every", 800); // 400Hz: 2秒flush一次
  const int drdy_burst = GetArgI(argc, argv, "--drdy-burst", 256);

  // optional
  const int max_frames = GetArgI(argc, argv, "--max-frames", -1);
  const int warmup_pairs = GetArgI(argc, argv, "--warmup", 60);

  std::cerr << "out=" << out_root << "\n";
  std::cerr << "cam " << w << "x" << h << " @" << fps
            << " ae_disable=" << (ae_disable ? "true":"false")
            << " exp_us=" << exposure_us << " gain=" << gain
            << " (FORCE ISP YUV420->Y)\n";
  std::cerr << "imu spi=" << spi_dev << " speed=" << spi_speed
            << " mode=" << int(spi_mode) << " bits=" << int(spi_bits)
            << " drdy=" << gpiochip << ":" << drdy_line
            << " imu_hz=" << imu_hz
            << " imu_flush_every=" << imu_flush_every
            << " drdy_burst=" << drdy_burst << "\n";

  // dirs
  fs::path root(out_root);
  fs::path cam0_data = root / "cam0";
  fs::path cam1_data = root / "cam1";
  EnsureDir(cam0_data);
  EnsureDir(cam1_data);

  FILE *f_cam0 = std::fopen((root / "cam0" / "data.csv").c_str(), "w");
  FILE *f_cam1 = std::fopen((root / "cam1" / "data.csv").c_str(), "w");
  FILE *f_imu  = std::fopen((root / "imu.csv").c_str(), "w");
  if (!f_cam0 || !f_cam1 || !f_imu) {
    std::cerr << "Failed to open output csv files.\n";
    return 1;
  }

  // 给 csv 大缓冲：减少 400Hz 下的写盘抖动
  SetupFileBuffer(f_cam0, 1 << 20);
  SetupFileBuffer(f_cam1, 1 << 20);
  SetupFileBuffer(f_imu,  4 << 20); // imu 行数更多，给大点

  std::fprintf(f_cam0, "#timestamp [ns],filename\n");
  std::fprintf(f_cam1, "#timestamp [ns],filename\n");
  std::fprintf(f_imu,  "#timestamp [ns],w_x [rad/s],w_y [rad/s],w_z [rad/s],a_x [m/s^2],a_y [m/s^2],a_z [m/s^2]\n");

  // IMU thread
  std::atomic<bool> imu_ok{false};
  std::thread imu_th([&](){
    SpiDev spi(spi_dev);
    if (!spi.Open(spi_speed, spi_mode, spi_bits)) return;
    if (!IcmResetAndConfig(spi, imu_hz)) return;

    DrdyGpio drdy;
    if (!drdy.Open(gpiochip, drdy_line, drdy_burst)) return;

    imu_ok.store(true);

    uint8_t raw[14]{};
    uint8_t st = 0;
    spi.ReadReg(REG_INT_STATUS, st);

    int imu_lines = 0;

    while (g_running.load()) {
      int64_t t_irq_ns = 0;
      if (!drdy.WaitLastTs(1000, t_irq_ns)) continue;

      ImuSample s{};
      s.t_ns = t_irq_ns;

      // 读取一帧 IMU（注意：如果你想“每个 DRDY 事件都对应一帧”，需要循环读 n 次；
      // 这里策略是：取最后一个 DRDY 时间戳，读当前寄存器快照——防止 event 堆积时延迟扩大）
      spi.ReadReg(REG_INT_STATUS, st);
      if (!spi.ReadRegs(REG_TEMP_DATA1, raw, sizeof(raw))) continue;
      ConvertRawToSI(raw, s);

      std::fprintf(f_imu, "%lld,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                   (long long)s.t_ns,
                   (double)s.gx, (double)s.gy, (double)s.gz,
                   (double)s.ax, (double)s.ay, (double)s.az);

      imu_lines++;
      if (imu_flush_every > 0 && (imu_lines % imu_flush_every) == 0) {
        std::fflush(f_imu);
      }
    }

    std::fflush(f_imu);
  });

  // Camera
  LibcameraStereoOV9281 cam;
  if (!cam.Open(w, h, fps, ae_disable, exposure_us, gain)) {
    g_running.store(false);
    if (imu_th.joinable()) imu_th.join();
    return 1;
  }

  // warmup estimate ts_offset_ns (NowNs domain - camera ts domain)
  std::vector<int64_t> offs;
  offs.reserve(std::max(10, warmup_pairs));

  std::cerr << "Warmup for ts_offset... pairs=" << warmup_pairs << "\n";
  for (int i = 0; i < warmup_pairs && g_running.load(); ) {
    FrameItem L, R;
    if (!cam.GrabPair(L, R, 1000)) continue;

    int64_t cam_ts = (int64_t)((L.ts_ns + R.ts_ns)/2);
    int64_t arrive = (L.arrive_ns && R.arrive_ns) ? (L.arrive_ns + R.arrive_ns)/2 : NowNs();
    offs.push_back(arrive - cam_ts);
    ++i;
  }

  int64_t ts_offset_ns = MedianEstimator::Median(offs);
  if (!offs.empty()) {
    int64_t mino = *std::min_element(offs.begin(), offs.end());
    int64_t maxo = *std::max_element(offs.begin(), offs.end());
    std::cerr << "Init ts_offset_ns(median)=" << ts_offset_ns
              << " range=[" << mino << "," << maxo << "] ns\n";
  } else {
    std::cerr << "Warmup failed: no frames, ts_offset=0\n";
  }

  // record loop
  int saved = 0;
  int64_t last_pair_ns = 0;

  std::cerr << "Recording... press Ctrl+C to stop\n";
  while (g_running.load()) {
    if (max_frames > 0 && saved >= max_frames) break;

    FrameItem L, R;
    if (!cam.GrabPair(L, R, 1000)) continue;

    int64_t dt_lr = (int64_t)L.ts_ns - (int64_t)R.ts_ns;
    if ((saved % 30) == 0) {
      std::cerr << "[pair] dt_lr_us=" << (std::llabs(dt_lr) / 1000.0)
                << " tol_us=" << (cam.PairTolNs()/1000.0) << "\n";
      std::cerr << "[imu] ok=" << (imu_ok.load() ? "true" : "false") << "\n";
    }

    int64_t cam_ts = (int64_t)((L.ts_ns + R.ts_ns)/2);
    int64_t pair_ns = cam_ts + ts_offset_ns;

    if (last_pair_ns != 0 && pair_ns <= last_pair_ns) pair_ns = last_pair_ns + 1;
    last_pair_ns = pair_ns;

    const std::string name = TsToName(pair_ns);
    const fs::path fnL = cam0_data / name;
    const fs::path fnR = cam1_data / name;

    if (!cv::imwrite(fnL.string(), L.gray)) {
      std::cerr << "imwrite failed: " << fnL << "\n";
      continue;
    }
    if (!cv::imwrite(fnR.string(), R.gray)) {
      std::cerr << "imwrite failed: " << fnR << "\n";
      continue;
    }

    std::fprintf(f_cam0, "%lld,%s\n", (long long)pair_ns, name.c_str());
    std::fprintf(f_cam1, "%lld,%s\n", (long long)pair_ns, name.c_str());

    if ((saved % 50) == 0) {
      std::fflush(f_cam0); std::fflush(f_cam1);
      std::cerr << "saved pairs=" << saved << "\n";
    }
    saved++;
  }

  std::cerr << "Stopping...\n";
  cam.Close();
  g_running.store(false);
  if (imu_th.joinable()) imu_th.join();

  std::fflush(f_cam0); std::fflush(f_cam1); std::fflush(f_imu);
  std::fclose(f_cam0); std::fclose(f_cam1); std::fclose(f_imu);

  std::cerr << "Done. Saved pairs=" << saved << "\n";
  return 0;
}
