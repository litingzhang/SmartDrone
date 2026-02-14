
#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <gpiod.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

static std::atomic<bool> g_running{true};

static void HandleSigint(int) { g_running.store(false); }

static int64_t NowNs() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

static inline int16_t Be16ToI16(uint8_t hi, uint8_t lo) {
  return (int16_t)((uint16_t(hi) << 8) | uint16_t(lo));
}

// ---------------- ICM42688 Registers (common subset) ----------------
// Bank 0 register addresses (ICM-42688 / ICM-42688-P)
static constexpr uint8_t REG_DEVICE_CONFIG   = 0x11;
static constexpr uint8_t REG_DRIVE_CONFIG    = 0x13;
static constexpr uint8_t REG_INT_CONFIG      = 0x14;
static constexpr uint8_t REG_FIFO_CONFIG     = 0x16;
static constexpr uint8_t REG_TEMP_DATA1      = 0x1D;  // TEMP_DATA1, TEMP_DATA0
static constexpr uint8_t REG_ACCEL_DATA_X1   = 0x1F;  // ACCEL_X1..Z0
static constexpr uint8_t REG_GYRO_DATA_X1    = 0x25;  // GYRO_X1..Z0
static constexpr uint8_t REG_INT_STATUS      = 0x2D;
static constexpr uint8_t REG_PWR_MGMT0       = 0x4E;
static constexpr uint8_t REG_GYRO_CONFIG0    = 0x4F;
static constexpr uint8_t REG_ACCEL_CONFIG0   = 0x50;
static constexpr uint8_t REG_GYRO_ACCEL_CONFIG0 = 0x52;
static constexpr uint8_t REG_INT_CONFIG1     = 0x64;
static constexpr uint8_t REG_INT_SOURCE0     = 0x65;
static constexpr uint8_t REG_WHO_AM_I        = 0x75;

// Read/write bit convention (SPI):
// - For InvenSense IMUs: MSB=1 indicates READ, MSB=0 indicates WRITE.
// - Address in lower 7 bits.
static constexpr uint8_t SPI_READ_MASK  = 0x80;
static constexpr uint8_t SPI_WRITE_MASK = 0x00;

struct ImuSample {
  int64_t t_ns{};
  float ax{}, ay{}, az{}; // m/s^2
  float gx{}, gy{}, gz{}; // rad/s
  float temp_c{};         // Celsius (approx)
};

class SpiDev {
public:
  explicit SpiDev(std::string dev) : dev_(std::move(dev)) {}

  bool Open(uint32_t speed_hz, uint8_t mode, uint8_t bits_per_word) {
    fd_ = ::open(dev_.c_str(), O_RDWR);
    if (fd_ < 0) {
      std::cerr << "open " << dev_ << " failed: " << strerror(errno) << "\n";
      return false;
    }

    if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(fd_, SPI_IOC_RD_MODE, &mode) < 0) {
      std::cerr << "SPI set mode failed: " << strerror(errno) << "\n";
      return false;
    }

    if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0 ||
        ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word) < 0) {
      std::cerr << "SPI set bits_per_word failed: " << strerror(errno) << "\n";
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

  ~SpiDev() {
    if (fd_ >= 0) ::close(fd_);
  }

  bool WriteReg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { uint8_t(SPI_WRITE_MASK | (reg & 0x7F)), val };
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
    // We need to send 1 addr byte + dummy bytes, read back same count
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
  uint8_t mode_{SPI_MODE_3};
  uint8_t bits_{8};
};
class DrdyGpio {
public:
  bool Open(const std::string &chip_path, unsigned line_offset) {
    chip_ = gpiod_chip_open(chip_path.c_str());
    if (!chip_) {
      std::cerr << "gpiod_chip_open(" << chip_path << ") failed: "
                << strerror(errno) << "\n";
      return false;
    }

    // line settings: input + rising edge
    gpiod_line_settings *settings = gpiod_line_settings_new();
    if (!settings) {
      std::cerr << "gpiod_line_settings_new failed\n";
      return false;
    }
gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING);
gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);

    // line config: apply settings to the given offset
    gpiod_line_config *line_cfg = gpiod_line_config_new();
    if (!line_cfg) {
      std::cerr << "gpiod_line_config_new failed\n";
      gpiod_line_settings_free(settings);
      return false;
    }
    unsigned offsets[1] = { line_offset };
    int rc = gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings);
    gpiod_line_settings_free(settings);
    if (rc < 0) {
      std::cerr << "gpiod_line_config_add_line_settings failed: "
                << strerror(errno) << "\n";
      gpiod_line_config_free(line_cfg);
      return false;
    }

    // request config
    gpiod_request_config *req_cfg = gpiod_request_config_new();
    if (!req_cfg) {
      std::cerr << "gpiod_request_config_new failed\n";
      gpiod_line_config_free(line_cfg);
      return false;
    }
    gpiod_request_config_set_consumer(req_cfg, "icm42688_drdy");

    request_ = gpiod_chip_request_lines(chip_, req_cfg, line_cfg);

    gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(line_cfg);

    if (!request_) {
      std::cerr << "gpiod_chip_request_lines failed: "
                << strerror(errno) << "\n";
      return false;
    }

    evbuf_ = gpiod_edge_event_buffer_new(256);
    if (!evbuf_) {
      std::cerr << "gpiod_edge_event_buffer_new failed\n";
      return false;
    }

    return true;
  }

  ~DrdyGpio() {
    if (evbuf_) gpiod_edge_event_buffer_free(evbuf_);
    if (request_) gpiod_line_request_release(request_);
    if (chip_) gpiod_chip_close(chip_);
  }

bool Wait(int timeout_ms) {
  int64_t timeout_ns = (timeout_ms < 0) ? -1 : (int64_t)timeout_ms * 1000000LL;
  int ret = gpiod_line_request_wait_edge_events(request_, timeout_ns);
  if (ret <= 0) return false; // 0 timeout, <0 error

  // Drain all pending events (最多读 64 个)
  int n = gpiod_line_request_read_edge_events(request_, evbuf_, 256);
  return n > 0;
}

private:
  gpiod_chip *chip_{nullptr};
  gpiod_line_request *request_{nullptr};
  gpiod_edge_event_buffer *evbuf_{nullptr};
};

struct Config {
  std::string spi_dev = "/dev/spidev0.0";
  uint32_t spi_speed_hz = 8000000;
  uint8_t spi_mode = SPI_MODE_0;
  uint8_t spi_bits = 8;

  std::string gpiochip = "/dev/gpiochip0";
  unsigned drdy_line = 24;

  int imu_hz = 200;
  bool print_csv = true;
};

static void PrintUsage() {
  std::cerr <<
    "Usage: icm42688 [--spi /dev/spidevX.Y] [--speed hz] [--mode 0..3] [--bits 8]\n"
    "               [--gpiochip /dev/gpiochipN] [--drdy line] [--hz imu_hz] [--no-csv]\n";
}

static std::optional<Config> ParseArgs(int argc, char **argv) {
  Config c;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto need = [&](const char *name) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << name << "\n";
        return nullptr;
      }
      return argv[++i];
    };

    if (a == "--spi") {
      const char *v = need("--spi"); if (!v) return std::nullopt;
      c.spi_dev = v;
    } else if (a == "--speed") {
      const char *v = need("--speed"); if (!v) return std::nullopt;
      c.spi_speed_hz = (uint32_t)strtoul(v, nullptr, 10);
    } else if (a == "--mode") {
      const char *v = need("--mode"); if (!v) return std::nullopt;
      c.spi_mode = (uint8_t)strtoul(v, nullptr, 10);
    } else if (a == "--bits") {
      const char *v = need("--bits"); if (!v) return std::nullopt;
      c.spi_bits = (uint8_t)strtoul(v, nullptr, 10);
    } else if (a == "--gpiochip") {
      const char *v = need("--gpiochip"); if (!v) return std::nullopt;
      c.gpiochip = v;
    } else if (a == "--drdy") {
      const char *v = need("--drdy"); if (!v) return std::nullopt;
      c.drdy_line = (unsigned)strtoul(v, nullptr, 10);
    } else if (a == "--hz") {
      const char *v = need("--hz"); if (!v) return std::nullopt;
      c.imu_hz = (int)strtol(v, nullptr, 10);
    } else if (a == "--no-csv") {
      c.print_csv = false;
    } else if (a == "-h" || a == "--help") {
      PrintUsage();
      return std::nullopt;
    } else {
      std::cerr << "Unknown arg: " << a << "\n";
      PrintUsage();
      return std::nullopt;
    }
  }
  return c;
}

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
  uint8_t gyro_fs = 0x00;   // assume 2000 dps
  uint8_t accel_fs = 0x00;  // assume 16 g

  uint8_t gyro_odr = odr_code(imu_hz);
  uint8_t accel_odr = odr_code(imu_hz);

  uint8_t gyro_cfg0  = uint8_t((gyro_fs << 5)  | (gyro_odr & 0x0F));
  uint8_t accel_cfg0 = uint8_t((accel_fs << 5) | (accel_odr & 0x0F));
  spi.WriteReg(REG_PWR_MGMT0, 0x0F);
  usleep(20000);
  if (!spi.WriteReg(REG_GYRO_CONFIG0, gyro_cfg0)) return false;
  if (!spi.WriteReg(REG_ACCEL_CONFIG0, accel_cfg0)) return false;
uint8_t st=0;

spi.ReadReg(REG_INT_STATUS, st);
  usleep(20000);
  return true;
}

static void ConvertRawToSI(const uint8_t raw[14], ImuSample &s) {
  // raw layout if starting from TEMP_DATA1 (0x1D):
  // TEMP_DATA1, TEMP_DATA0,
  // ACCEL_X1,X0, Y1,Y0, Z1,Z0,
  // GYRO_X1,X0,  Y1,Y0, Z1,Z0
  int16_t temp = Be16ToI16(raw[0], raw[1]);
  int16_t ax   = Be16ToI16(raw[2], raw[3]);
  int16_t ay   = Be16ToI16(raw[4], raw[5]);
  int16_t az   = Be16ToI16(raw[6], raw[7]);
  int16_t gx   = Be16ToI16(raw[8], raw[9]);
  int16_t gy   = Be16ToI16(raw[10], raw[11]);
  int16_t gz   = Be16ToI16(raw[12], raw[13]);

  // Scaling based on chosen FS (assumed):
  // accel 16g => 2048 LSB/g (common for some InvenSense), OR 4096 depending on chip.
  // gyro 2000 dps => 16.4 LSB/(dps) (classic MPU), but ICM42688 uses different (e.g., 16.4 still common).
  //
  // You MUST verify these for ICM42688; adjust if values look wrong.
  constexpr float kG = 9.80665f;

  // Conservative starting point (often close):
  constexpr float accel_lsb_per_g = 2048.0f;      // adjust if needed
  constexpr float gyro_lsb_per_dps = 16.4f;       // adjust if needed

  float ax_g = float(ax) / accel_lsb_per_g;
  float ay_g = float(ay) / accel_lsb_per_g;
  float az_g = float(az) / accel_lsb_per_g;

  float gx_dps = float(gx) / gyro_lsb_per_dps;
  float gy_dps = float(gy) / gyro_lsb_per_dps;
  float gz_dps = float(gz) / gyro_lsb_per_dps;

  s.ax = ax_g * kG;
  s.ay = ay_g * kG;
  s.az = az_g * kG;

  constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
  s.gx = gx_dps * kDeg2Rad;
  s.gy = gy_dps * kDeg2Rad;
  s.gz = gz_dps * kDeg2Rad;

  // Temperature formula is chip-specific. Placeholder linear approx.
  // If you care, replace with datasheet formula.
  s.temp_c = float(temp) / 132.48f + 25.0f;
}

int main(int argc, char **argv) {
  signal(SIGINT, HandleSigint);
  signal(SIGTERM, HandleSigint);

  auto cfg_opt = ParseArgs(argc, argv);
  if (!cfg_opt) return 1;
  const Config cfg = *cfg_opt;

  SpiDev spi(cfg.spi_dev);
  if (!spi.Open(cfg.spi_speed_hz, cfg.spi_mode, cfg.spi_bits)) return 1;

  if (!IcmResetAndConfig(spi, cfg.imu_hz)) {
    std::cerr << "ICM config failed.\n";
    return 1;
  }
auto Dump = [&](uint8_t r, const char* name){
  uint8_t v = 0;
  if (!spi.ReadReg(r, v)) {
    std::cerr << name << "=<read fail>\n";
    return;
  }
  std::cerr << name << "=0x" << std::hex << int(v) << std::dec << "\n";
};

Dump(REG_INT_CONFIG,   "INT_CONFIG");    // 0x14
Dump(REG_INT_CONFIG1,  "INT_CONFIG1");   // 0x64
Dump(REG_INT_SOURCE0,  "INT_SOURCE0");   // 0x65
Dump(REG_INT_STATUS,   "INT_STATUS");    // 0x2D

  DrdyGpio drdy;
  if (!drdy.Open(cfg.gpiochip, cfg.drdy_line)) return 1;

  if (cfg.print_csv) {
    std::cout << "t_ns,ax,ay,az,gx,gy,gz,temp_c\n";
  }

  // Main loop: wait DRDY -> timestamp -> read burst -> convert -> print
  uint8_t raw[14]{};
  int64_t last_t = 0;
  uint8_t st = 0;
  spi.ReadReg(REG_INT_STATUS, st);
  while (g_running.load()) {
    // Wait up to 1s; if timeout, continue so SIGINT can break.
    if (!drdy.Wait(1000)) continue;
  const int64_t t_ns = NowNs();

  spi.ReadReg(REG_INT_STATUS, st);       // ack/clear latch
  if (!spi.ReadRegs(REG_TEMP_DATA1, raw, sizeof(raw))) continue;

  ImuSample s{};
  s.t_ns = t_ns;
  ConvertRawToSI(raw, s);

  std::cout << s.t_ns << "," << s.ax << "," << s.ay << "," << s.az << ","
            << s.gx << "," << s.gy << "," << s.gz << "," << s.temp_c << "\n";
  }

  return 0;
}
