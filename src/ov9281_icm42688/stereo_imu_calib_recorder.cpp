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
#include "udp_image_sender.hpp"
using namespace libcamera;
namespace fs = std::filesystem;

static std::atomic<bool> g_runningFlag{true};
static void SigIntHandler(int) { g_runningFlag.store(false); }
static uint32_t g_seq = 0;

static int64_t NowNs() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

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

static inline int16_t Be16ToI16(uint8_t hi, uint8_t lo) {
  return (int16_t)((uint16_t(hi) << 8) | uint16_t(lo));
}

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
  int64_t tNs{};          // DRDY timestamp (CLOCK_MONOTONIC domain)
  float ax{}, ay{}, az{};  // m/s^2
  float gx{}, gy{}, gz{};  // rad/s
};

class SpiDev {
public:
  explicit SpiDev(std::string dev) : m_dev(std::move(dev)) {}
  ~SpiDev() { if (m_fd >= 0) ::close(m_fd); }

  bool Open(uint32_t speed_hz, uint8_t mode, uint8_t bitsPerWord) {
    m_fd = ::open(m_dev.c_str(), O_RDWR);
    if (m_fd < 0) {
      std::cerr << "open " << m_dev << " failed: " << strerror(errno) << "\n";
      return false;
    }
    if (ioctl(m_fd, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(m_fd, SPI_IOC_RD_MODE, &mode) < 0) {
      std::cerr << "SPI set mode failed: " << strerror(errno) << "\n";
      return false;
    }
    if (ioctl(m_fd, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord) < 0 ||
        ioctl(m_fd, SPI_IOC_RD_BITS_PER_WORD, &bitsPerWord) < 0) {
      std::cerr << "SPI set bits failed: " << strerror(errno) << "\n";
      return false;
    }
    if (ioctl(m_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0 ||
        ioctl(m_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz) < 0) {
      std::cerr << "SPI set speed failed: " << strerror(errno) << "\n";
      return false;
    }
    m_speedHz = speed_hz;
    m_mode = mode;
    m_bits = bitsPerWord;
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

  bool ReadRegs(uint8_t startReg, uint8_t *out, size_t len) {
    std::vector<uint8_t> tx(len + 1, 0x00);
    std::vector<uint8_t> rx(len + 1, 0x00);
    tx[0] = uint8_t(SPI_READ_MASK | (startReg & 0x7F));
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
    tr.speed_hz = m_speedHz;
    tr.bits_per_word = m_bits;
    tr.delay_usecs = 0;
    if (ioctl(m_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
      std::cerr << "SPI transfer failed: " << strerror(errno) << "\n";
      return false;
    }
    return true;
  }

  std::string m_dev;
  int m_fd{-1};
  uint32_t m_speedHz{8000000};
  uint8_t m_mode{SPI_MODE_0};
  uint8_t m_bits{8};
};

class DrdyGpio {
public:
  bool Open(const std::string &chipPath, unsigned lineOffset, int maxBurst = 256) {
    m_chip = gpiod_chip_open(chipPath.c_str());
    if (!m_chip) {
      std::cerr << "gpiod_chip_open(" << chipPath << ") failed: " << strerror(errno) << "\n";
      return false;
    }

    gpiod_line_settings *settings = gpiod_line_settings_new();
    if (!settings) return false;
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING);
    gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);

    gpiod_line_config *lineConfig = gpiod_line_config_new();
    if (!lineConfig) { gpiod_line_settings_free(settings); return false; }

    unsigned offsets[1] = { lineOffset };
    int rc = gpiod_line_config_add_line_settings(lineConfig, offsets, 1, settings);
    gpiod_line_settings_free(settings);
    if (rc < 0) { gpiod_line_config_free(lineConfig); return false; }

    gpiod_request_config *requestConfig = gpiod_request_config_new();
    if (!requestConfig) { gpiod_line_config_free(lineConfig); return false; }
    gpiod_request_config_set_consumer(requestConfig, "icm42688_drdy");

    m_request = gpiod_chip_request_lines(m_chip, requestConfig, lineConfig);
    gpiod_request_config_free(requestConfig);
    gpiod_line_config_free(lineConfig);

    if (!m_request) {
      std::cerr << "gpiod_chip_request_lines failed: " << strerror(errno) << "\n";
      return false;
    }

    m_maxBurst = std::max(1, maxBurst);
    m_evbuf = gpiod_edge_event_buffer_new((size_t)m_maxBurst);
    return m_evbuf != nullptr;
  }

  ~DrdyGpio() {
    if (m_evbuf) gpiod_edge_event_buffer_free(m_evbuf);
    if (m_request) gpiod_line_request_release(m_request);
    if (m_chip) gpiod_chip_close(m_chip);
  }

  bool WaitLastTs(int timeoutMs, int64_t &tsNsOut) {
    int64_t timeoutNs = (timeoutMs < 0) ? -1 : (int64_t)timeoutMs * 1000000LL;
    int ret = gpiod_line_request_wait_edge_events(m_request, timeoutNs);
    if (ret <= 0) return false;

    int n = gpiod_line_request_read_edge_events(m_request, m_evbuf, (size_t)m_maxBurst);
    if (n <= 0) return false;

    struct gpiod_edge_event *evLast = gpiod_edge_event_buffer_get_event(m_evbuf, (size_t)(n - 1));
    if (!evLast) return false;

    tsNsOut = (int64_t)gpiod_edge_event_get_timestamp_ns(evLast);
    return true;
  }

private:
  gpiod_chip *m_chip{nullptr};
  gpiod_line_request *m_request{nullptr};
  gpiod_edge_event_buffer *m_evbuf{nullptr};
  int m_maxBurst{256};
};

static bool IcmResetAndConfig(SpiDev &spi, int imuHz) {
  auto odrCode = [&](int hz) -> uint8_t {
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

  uint8_t gyroFs = 0x00;
  uint8_t accelFs = 0x00;

  uint8_t odr = odrCode(imuHz);
  uint8_t gyro_cfg0  = uint8_t((gyroFs << 5)  | (odr & 0x0F));
  uint8_t accel_cfg0 = uint8_t((accelFs << 5) | (odr & 0x0F));

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
  constexpr float accelLsbPerG  = 2048.0f;
  constexpr float gyroLsbPerDps = 16.4f;

  s.ax = (float(ax) / accelLsbPerG) * kG;
  s.ay = (float(ay) / accelLsbPerG) * kG;
  s.az = (float(az) / accelLsbPerG) * kG;

  constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
  s.gx = (float(gx) / gyroLsbPerDps) * kDeg2Rad;
  s.gy = (float(gy) / gyroLsbPerDps) * kDeg2Rad;
  s.gz = (float(gz) / gyroLsbPerDps) * kDeg2Rad;
}

struct FrameItem {
  int camIndex{-1};
  uint64_t tsNs{0};
  int64_t  arriveNs{0};
  cv::Mat gray; // CV_8UC1
};

class LibcameraMonoCam {
public:
  bool Open(std::shared_ptr<Camera> cam, int camIndex, int w, int h, int fps,
            bool aeDisable, int exposureUs, float gain) {
    m_cam = std::move(cam);
    m_camIndex = camIndex;

    if (!m_cam) return false;
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

    sc.pixelFormat = formats::YUV420;

    const int64_t maxFrameUs = 1000000LL / std::max(1, fps);
    const int64_t minFrameUs = 1000000LL / 120;
    m_controls.set(controls::FrameDurationLimits,
                  Span<const int64_t, 2>({minFrameUs, maxFrameUs}));

      m_controls.set(controls::AeEnable, false);
      m_controls.set(controls::ExposureTime, exposureUs);
      m_controls.set(controls::AnalogueGain, gain);


    CameraConfiguration::Status status = m_config->validate();
    if (status == CameraConfiguration::Invalid) {
      std::cerr << "Invalid camera configuration\n";
      return false;
    }

    if (m_config->at(0).pixelFormat != formats::YUV420) {
      std::cerr << "ERROR: validate() changed pixelFormat to "
                << m_config->at(0).pixelFormat.toString()
                << " (expected YUV420). Refuse to run.\n";
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

    m_requests.clear();
    for (auto &buf : buffers) {
      std::unique_ptr<Request> req = m_cam->createRequest();
      if (!req) return false;
      if (req->addBuffer(m_stream, buf.get()) < 0) return false;
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
    for (auto &r : m_requests) {
      if (m_cam->queueRequest(r.get()) < 0) return false;
    }
    return true;
  }

  void Stop() { if (m_cam) m_cam->stop(); }

  void Close() {
    for (auto &kv : m_bufferMaps) for (auto &pm : kv.second) MUnmap(pm.addr, pm.len);
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

private:
  void OnRequestComplete(Request *req) {
    if (!req || req->status() == Request::RequestCancelled) return;

    auto it = req->buffers().find(m_stream);
    if (it == req->buffers().end()) {
      req->reuse(Request::ReuseBuffers);
      m_cam->queueRequest(req);
      return;
    }
    FrameBuffer *buf = it->second;

    auto *buffer = req->findBuffer(m_stream);
    const FrameMetadata &fbmd = buffer->metadata();

    FrameItem item;
    item.camIndex = m_camIndex;
    item.arriveNs = NowNs();
    item.tsNs = fbmd.timestamp;

    const StreamConfiguration &sc = m_config->at(0);
    const int w = sc.size.width;
    const int h = sc.size.height;
    const int strideY = sc.stride;

    auto mit = m_bufferMaps.find(buf);
    if (mit == m_bufferMaps.end() || mit->second.empty() || !mit->second[0].addr) {
      req->reuse(Request::ReuseBuffers);
      m_cam->queueRequest(req);
      return;
    }

    if (sc.pixelFormat != formats::YUV420) {
      std::cerr << "Unexpected fmt in callback: " << sc.pixelFormat.toString() << "\n";
      req->reuse(Request::ReuseBuffers);
      m_cam->queueRequest(req);
      return;
    }

    uint8_t *pY = reinterpret_cast<uint8_t *>(mit->second[0].addr);
    cv::Mat yview(h, w, CV_8UC1, (void*)pY, (size_t)strideY);
    item.gray = yview.clone();

    if (m_sink) m_sink(std::move(item));

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
};

class LibcameraStereoOV9281 {
public:
  bool Open(int w, int h, int fps,
            bool aeDisable, int exposureUs, float gain,
            int maxPairQueue = 8, uint64_t pairTolNs = 15000000) {
    m_w = w; m_h = h; m_fps = fps;
    m_maxPairQueue = maxPairQueue;
    m_pairTolNs = pairTolNs;

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

    auto sink = [&](FrameItem &&fi) {
      std::lock_guard<std::mutex> lk(m_mu);
      if (fi.camIndex == 0) m_qL.push_back(std::move(fi));
      else m_qR.push_back(std::move(fi));
      TryPairLocked();
    };

    if (!m_left.Open(m_camL, 0, w, h, fps, aeDisable, exposureUs, gain)) return false;
    if (!m_right.Open(m_camR, 1, w, h, fps, aeDisable, exposureUs, gain)) return false;
    m_left.SetSink(sink);
    m_right.SetSink(sink);

    if (!m_left.Start()) return false;
    if (!m_right.Start()) return false;

    std::cerr << "Left  fmt=" << m_left.PixelFmt().toString()
              << " size=" << m_left.SizeWH().toString()
              << " stride=" << m_left.Stride() << "\n";
    std::cerr << "Right fmt=" << m_right.PixelFmt().toString()
              << " size=" << m_right.SizeWH().toString()
              << " stride=" << m_right.Stride() << "\n";
    return true;
  }

  void Close() {
    m_left.Stop(); m_right.Stop();
    m_left.Close(); m_right.Close();
    if (m_cm) m_cm->stop();
    m_cm.reset();
  }

  bool GrabPair(FrameItem &L, FrameItem &R, int timeoutMs = 1000) {
    std::unique_lock<std::mutex> lk(m_mu);
    if (!m_cv.wait_for(lk, std::chrono::milliseconds(timeoutMs),
                      [&]{ return !m_pairedRaw.empty() || !g_runningFlag.load(); })) {
      return false;
    }
    if (m_pairedRaw.empty()) return false;
    auto &p = m_pairedRaw.front();
    L = std::move(p.first);
    R = std::move(p.second);
    m_pairedRaw.pop_front();
    return true;
  }

  uint64_t PairTolNs() const { return m_pairTolNs; }

private:
  void TryPairLocked() {
    while (!m_qL.empty() && !m_qR.empty()) {
      uint64_t tL = m_qL.front().tsNs;
      uint64_t tR = m_qR.front().tsNs;

      int64_t dt = (int64_t)tL - (int64_t)tR;
      uint64_t adt = (dt < 0) ? (uint64_t)(-dt) : (uint64_t)dt;

      if (adt <= m_pairTolNs) {
        auto L = std::move(m_qL.front()); m_qL.pop_front();
        auto R = std::move(m_qR.front()); m_qR.pop_front();
        m_pairedRaw.push_back({std::move(L), std::move(R)});
        while ((int)m_pairedRaw.size() > m_maxPairQueue) m_pairedRaw.pop_front();
        m_cv.notify_one();
      } else {
        if (tL < tR) m_qL.pop_front();
        else m_qR.pop_front();
      }
    }
  }

  int m_w{1280}, m_h{800}, m_fps{60};
  int m_maxPairQueue{8};
  uint64_t m_pairTolNs{1000000};

  std::unique_ptr<CameraManager> m_cm;
  std::shared_ptr<Camera> m_camL, m_camR;
  LibcameraMonoCam m_left, m_right;

  std::mutex m_mu;
  std::condition_variable m_cv;
  std::deque<FrameItem> m_qL, m_qR;
  std::deque<std::pair<FrameItem, FrameItem>> m_pairedRaw;
};

static void EnsureDir(const fs::path &p) {
  std::error_code ec;
  fs::create_directories(p, ec);
  if (ec) {
    std::cerr << "create_directories failed: " << p << " : " << ec.message() << "\n";
  }
}

static std::string TsToName(int64_t tNs) {
  std::ostringstream oss;
  oss << tNs << ".png";
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

  setvbuf(f, nullptr, _IOFBF, bytes);
}

int main(int argc, char **argv) {
  signal(SIGINT, SigIntHandler);
  signal(SIGTERM, SigIntHandler);

  const std::string outRoot = GetArgS(argc, argv, "--out", "./calib_out");
  const std::string udpIp = GetArgS(argc, argv, "--udp-ip", "10.42.0.109");
  const int udpPort = GetArgI(argc, argv, "--udp-port", 14550);

  const int w = GetArgI(argc, argv, "--w", 1280);
  const int h = GetArgI(argc, argv, "--h", 800);
  const int fps = GetArgI(argc, argv, "--fps", 30);
  const bool aeDisable = HasArg(argc, argv, "--ae-disable");
  const int exposureUs = GetArgI(argc, argv, "--exp-us", 3000);
  const float gain = GetArgF(argc, argv, "--gain", 2.0f);

  const std::string spiDev = GetArgS(argc, argv, "--spi", "/dev/spidev0.0");
  const uint32_t spi_speed = (uint32_t)GetArgI(argc, argv, "--speed", 8000000);
  const uint8_t spiMode = (uint8_t)GetArgI(argc, argv, "--mode", 0);
  const uint8_t spiBits = (uint8_t)GetArgI(argc, argv, "--bits", 8);

  const std::string gpiochip = GetArgS(argc, argv, "--gpiochip", "/dev/gpiochip0");
  const unsigned drdyLine = (unsigned)GetArgI(argc, argv, "--drdy", 24);
  const int imuHz = GetArgI(argc, argv, "--imu-hz", 400);
  const int pairTolUs = GetArgI(argc, argv, "--pair-tol-us", 2000);
  const int64_t offRejectNs = GetArgI64(argc, argv, "--off-reject-ns", 5'000'000);

  const int imuFlushEvery = GetArgI(argc, argv, "--imu-flush-every", 800);
  const int drdyBurst = GetArgI(argc, argv, "--drdy-burst", 256);

  const int max_frames = GetArgI(argc, argv, "--max-frames", -1);
  const int warmupPairs = GetArgI(argc, argv, "--warmup", 60);

  std::cerr << "out=" << outRoot << "\n";
  std::cerr << "cam " << w << "x" << h << " @" << fps
            << " aeDisable=" << (aeDisable ? "true":"false")
            << " exp_us=" << exposureUs << " gain=" << gain << "\n";
  std::cerr << "imu spi=" << spiDev << " speed=" << spi_speed
            << " mode=" << int(spiMode) << " bits=" << int(spiBits)
            << " drdy=" << gpiochip << ":" << drdyLine
            << " imuHz=" << imuHz
            << " imuFlushEvery=" << imuFlushEvery
            << " drdyBurst=" << drdyBurst << "\n";
  std::cerr << "pairTolUs=" << pairTolUs
            << " offRejectNs=" << offRejectNs << "\n";

  fs::path root(outRoot);
  fs::path cam0_data = root / "cam0";
  fs::path cam1_data = root / "cam1";
  EnsureDir(cam0_data);
  EnsureDir(cam1_data);

  UdpImageSender udp;
  if (!udp.Open(udpIp, udpPort, 45)) {
      std::cerr << "[udp] open failed, continue without udp.\n";
  } else {
      std::cerr << "[udp] sending to " << udpIp << ":" << udpPort << "\n";
  }

  FILE *fCam0 = std::fopen((root / "cam0" / "data.csv").c_str(), "w");
  FILE *fCam1 = std::fopen((root / "cam1" / "data.csv").c_str(), "w");
  FILE *fImu  = std::fopen((root / "imu.csv").c_str(), "w");
  if (!fCam0 || !fCam1 || !fImu) {
    std::cerr << "Failed to open output csv files.\n";
    return 1;
  }

  SetupFileBuffer(fCam0, 1 << 20);
  SetupFileBuffer(fCam1, 1 << 20);
  SetupFileBuffer(fImu,  4 << 20); // imu 行数更多，给大点

  std::fprintf(fCam0, "#timestamp [ns],filename\n");
  std::fprintf(fCam1, "#timestamp [ns],filename\n");
  std::fprintf(fImu,  "#timestamp [ns],wX [rad/s],wY [rad/s],wZ [rad/s],aX [m/s^2],aY [m/s^2],aZ [m/s^2]\n");

  std::atomic<bool> imuOk{false};
  std::thread imuThread([&](){
    SpiDev spi(spiDev);
    if (!spi.Open(spi_speed, spiMode, spiBits)) return;
    if (!IcmResetAndConfig(spi, imuHz)) return;

    DrdyGpio drdy;
    if (!drdy.Open(gpiochip, drdyLine, drdyBurst)) return;

    imuOk.store(true);

    uint8_t raw[14]{};
    uint8_t st = 0;
    spi.ReadReg(REG_INT_STATUS, st);

    int imu_lines = 0;

    while (g_runningFlag.load()) {
      int64_t tIrqNs = 0;
      if (!drdy.WaitLastTs(1000, tIrqNs)) continue;

      ImuSample s{};
      s.tNs = tIrqNs;


      spi.ReadReg(REG_INT_STATUS, st);
      if (!spi.ReadRegs(REG_TEMP_DATA1, raw, sizeof(raw))) continue;
      ConvertRawToSI(raw, s);

      std::fprintf(fImu, "%lld,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                   (long long)s.tNs,
                   (double)s.gx, (double)s.gy, (double)s.gz,
                   (double)s.ax, (double)s.ay, (double)s.az);

      imu_lines++;
      if (imuFlushEvery > 0 && (imu_lines % imuFlushEvery) == 0) {
        std::fflush(fImu);
      }
    }

    std::fflush(fImu);
  });

  LibcameraStereoOV9281 cam;
  if (!cam.Open(
          w,
          h,
          fps,
          aeDisable,
          exposureUs,
          gain,
          8,
          static_cast<uint64_t>(std::max(1, pairTolUs)) * 1000ULL)) {
    g_runningFlag.store(false);
    if (imuThread.joinable()) imuThread.join();
    return 1;
  }

  std::vector<int64_t> offs;
  offs.reserve(std::max(10, warmupPairs));

  std::cerr << "Warmup for ts_offset... pairs=" << warmupPairs << "\n";
  for (int i = 0; i < warmupPairs && g_runningFlag.load(); ) {
    FrameItem L, R;
    if (!cam.GrabPair(L, R, 1000)) continue;

    int64_t camTs = (int64_t)((L.tsNs + R.tsNs)/2);
    int64_t arrive = (L.arriveNs && R.arriveNs) ? (L.arriveNs + R.arriveNs)/2 : NowNs();
    offs.push_back(arrive - camTs);
    ++i;
  }

  int64_t tsOffsetNs = MedianEstimator::Median(offs);
  if (!offs.empty()) {
    int64_t mino = *std::min_element(offs.begin(), offs.end());
    int64_t maxo = *std::max_element(offs.begin(), offs.end());
    std::cerr << "Init tsOffsetNs(median)=" << tsOffsetNs
              << " range=[" << mino << "," << maxo << "] ns\n";
  } else {
    std::cerr << "Warmup failed: no frames, ts_offset=0\n";
  }

  int saved = 0;
  int64_t lastLeftNs = 0;
  int64_t lastRightNs = 0;

  std::cerr << "Recording... press Ctrl+C to stop\n";
  while (g_runningFlag.load()) {
    if (max_frames > 0 && saved >= max_frames) break;

    FrameItem L, R;
    if (!cam.GrabPair(L, R, 1000)) continue;

    int64_t dtLr = (int64_t)L.tsNs - (int64_t)R.tsNs;
    if ((saved % 30) == 0) {
      std::cerr << "[pair] dt_lr_us=" << ((dtLr < 0 ? -dtLr : dtLr) / 1000.0)
                << " tol_us=" << (cam.PairTolNs()/1000.0) << "\n";
      std::cerr << "[imu] ok=" << (imuOk.load() ? "true" : "false") << "\n";
    }

    int64_t camTs = (int64_t)((L.tsNs + R.tsNs)/2);
    int64_t arrive = (L.arriveNs && R.arriveNs) ? (L.arriveNs + R.arriveNs)/2 : NowNs();
    int64_t off_meas = arrive - camTs;
    int64_t err = off_meas - tsOffsetNs;
    if ((err < 0 ? -err : err) < offRejectNs) {
      tsOffsetNs += (err >> 6);
    }

    int64_t leftNs = (int64_t)L.tsNs + tsOffsetNs;
    int64_t rightNs = (int64_t)R.tsNs + tsOffsetNs;

    if (lastLeftNs != 0 && leftNs <= lastLeftNs) leftNs = lastLeftNs + 1;
    if (lastRightNs != 0 && rightNs <= lastRightNs) rightNs = lastRightNs + 1;
    lastLeftNs = leftNs;
    lastRightNs = rightNs;

    const std::string nameL = TsToName(leftNs);
    const std::string nameR = TsToName(rightNs);
    const fs::path fnL = cam0_data / nameL;
    const fs::path fnR = cam1_data / nameR;

    if (!cv::imwrite(fnL.string(), L.gray)) {
      std::cerr << "imwrite failed: " << fnL << "\n";
      continue;
    }
    if (!cv::imwrite(fnR.string(), R.gray)) {
      std::cerr << "imwrite failed: " << fnR << "\n";
      continue;
    }

    std::fprintf(fCam0, "%lld,%s\n", (long long)leftNs, nameL.c_str());
    std::fprintf(fCam1, "%lld,%s\n", (long long)rightNs, nameR.c_str());

    udp.Enqueue(0, g_seq, leftNs, L.gray);
    udp.Enqueue(1, g_seq++, rightNs, R.gray);
    if ((saved % 50) == 0) {
      std::fflush(fCam0); std::fflush(fCam1);
      std::cerr << "saved pairs=" << saved << "\n";
    }
    saved++;
  }

  std::cerr << "Stopping...\n";
  cam.Close();
  g_runningFlag.store(false);
  if (imuThread.joinable()) imuThread.join();

  std::fflush(fCam0); std::fflush(fCam1); std::fflush(fImu);
  std::fclose(fCam0); std::fclose(fCam1); std::fclose(fImu);

  std::cerr << "Done. Saved pairs=" << saved << "\n";
  return 0;
}
