#pragma once

#include <atomic>
#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <mutex>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <vector>

// ORB-SLAM3 IMU type
#include "ImuTypes.h"   // ORB_SLAM3::IMU::Point

namespace ORB_SLAM3 {

class ICM20948Poller {
public:
  struct Options {
    std::string i2c_dev = "/dev/i2c-1";
    uint8_t i2c_addr = 0x68;

    double rate_hz = 200.0;     // poll rate
    double calib_seconds = 2.0; // gyro bias calib duration

    size_t ring_seconds = 5;    // ring buffer length in seconds
    bool use_realtime_fifo = true;
    int fifo_priority = 80;     // 1..99 (needs sudo / CAP_SYS_NICE)
  };

  explicit ICM20948Poller(const Options& opt)
  : opt_(opt),
    ring_cap_(static_cast<size_t>(std::ceil(opt.rate_hz * opt.ring_seconds)) + 64),
    ring_(ring_cap_)
  {
    open_i2c_();
    whoami_check_();
    init_basic_();
  }

  ~ICM20948Poller() {
    Stop();
    if (fd_ >= 0) close(fd_);
  }

  void Start() {
    if (running_.exchange(true)) return;
    stop_.store(false);

    int rc = pthread_create(&th_, nullptr, &ICM20948Poller::thread_entry_, this);
    if (rc != 0) {
      running_.store(false);
      throw std::runtime_error(std::string("pthread_create failed: ") + std::strerror(rc));
    }
  }

  void Stop() {
    if (!running_.load()) return;
    stop_.store(true);
    pthread_join(th_, nullptr);
    running_.store(false);
  }

  // Return IMU measurements in (t0, t1], timestamps in seconds (monotonic_raw).
  std::vector<IMU::Point> GetImuBetween(double t0, double t1) {
    std::lock_guard<std::mutex> lk(m_);
    std::vector<IMU::Point> out;
    out.reserve(256);

    // iterate from oldest to newest
    for (size_t i = 0; i < size_; i++) {
      size_t idx = (head_ + ring_.size() - size_ + i) % ring_.size();
      const auto& s = ring_[idx];
      if (s.t > t0 && s.t <= t1) out.push_back(s.meas);
    }
    return out;
  }

  // Use this to timestamp camera frames consistently if you don't have libcamera monotonic timestamps.
  static double NowMonotonicRawSec() {
    return (double)now_ns_() * 1e-9;
  }

private:
  struct Sample {
    double t;
    IMU::Point meas;
  };

  // ---------- time ----------
  static uint64_t now_ns_() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
  }

  // ---------- i2c low-level ----------
  void open_i2c_() {
    fd_ = ::open(opt_.i2c_dev.c_str(), O_RDWR);
    if (fd_ < 0) {
      throw std::runtime_error("open(" + opt_.i2c_dev + ") failed: " + std::string(std::strerror(errno)));
    }
  }

  int i2c_write_byte_(uint8_t reg, uint8_t val) {
    if (ioctl(fd_, I2C_SLAVE, opt_.i2c_addr) < 0) return -1;
    uint8_t buf[2] = {reg, val};
    if (::write(fd_, buf, 2) != 2) return -1;
    return 0;
  }

  int i2c_read_byte_(uint8_t reg, uint8_t* out) {
    if (ioctl(fd_, I2C_SLAVE, opt_.i2c_addr) < 0) return -1;
    if (::write(fd_, &reg, 1) != 1) return -1;
    if (::read(fd_, out, 1) != 1) return -1;
    return 0;
  }

  int i2c_read_bytes_rdwr_(uint8_t reg, uint8_t* data, uint16_t len) {
    struct i2c_rdwr_ioctl_data ioctl_data;
    struct i2c_msg msgs[2];

    msgs[0].addr  = opt_.i2c_addr;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;

    msgs[1].addr  = opt_.i2c_addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = len;
    msgs[1].buf   = data;

    ioctl_data.msgs  = msgs;
    ioctl_data.nmsgs = 2;

    if (ioctl(fd_, I2C_RDWR, &ioctl_data) < 0) return -1;
    return 0;
  }

  // ---------- ICM20948 registers (match your header) ----------
  static constexpr uint8_t REG_ADD_WIA = 0x00;
  static constexpr uint8_t REG_VAL_WIA = 0xEA;

  static constexpr uint8_t REG_ADD_REG_BANK_SEL = 0x7F;
  static constexpr uint8_t REG_VAL_REG_BANK_0   = 0x00;
  static constexpr uint8_t REG_VAL_REG_BANK_2   = 0x20;

  static constexpr uint8_t REG_ADD_PWR_MIGMT_1   = 0x06;
  static constexpr uint8_t REG_VAL_ALL_RGE_RESET = 0x80;
  static constexpr uint8_t REG_VAL_RUN_MODE      = 0x01;

  static constexpr uint8_t REG_ADD_ACCEL_XOUT_H  = 0x2D; // per your .h
  static constexpr uint8_t REG_ADD_GYRO_XOUT_H   = 0x33; // per your .h

  static constexpr uint8_t REG_ADD_GYRO_SMPLRT_DIV = 0x00; // bank2
  static constexpr uint8_t REG_ADD_GYRO_CONFIG_1   = 0x01;
  static constexpr uint8_t REG_VAL_BIT_GYRO_DLPCFG_6   = 0x30;
  static constexpr uint8_t REG_VAL_BIT_GYRO_FS_1000DPS = 0x04;
  static constexpr uint8_t REG_VAL_BIT_GYRO_DLPF       = 0x01;

  static constexpr uint8_t REG_ADD_ACCEL_SMPLRT_DIV_2 = 0x11;
  static constexpr uint8_t REG_ADD_ACCEL_CONFIG       = 0x14;
  static constexpr uint8_t REG_VAL_BIT_ACCEL_DLPCFG_6 = 0x30;
  static constexpr uint8_t REG_VAL_BIT_ACCEL_FS_2g    = 0x00;
  static constexpr uint8_t REG_VAL_BIT_ACCEL_DLPF     = 0x01;

  static constexpr float GYRO_SSF_AT_FS_1000DPS = 32.8f;
  static constexpr float ACCEL_SSF_AT_FS_2g     = 16384.0f;

  int select_bank_(uint8_t bank) {
    return i2c_write_byte_(REG_ADD_REG_BANK_SEL, bank);
  }

  void whoami_check_() {
    select_bank_(REG_VAL_REG_BANK_0);
    uint8_t v = 0;
    if (i2c_read_byte_(REG_ADD_WIA, &v) != 0 || v != REG_VAL_WIA) {
      throw std::runtime_error("ICM20948 WHOAMI mismatch (bus ok, but device not found or addr wrong)");
    }
  }

  void init_basic_() {
    select_bank_(REG_VAL_REG_BANK_0);
    // reset
    i2c_write_byte_(REG_ADD_PWR_MIGMT_1, REG_VAL_ALL_RGE_RESET);
    usleep(10 * 1000);
    // run mode
    i2c_write_byte_(REG_ADD_PWR_MIGMT_1, REG_VAL_RUN_MODE);
    usleep(10 * 1000);

    // bank2 config (match your C init)
    select_bank_(REG_VAL_REG_BANK_2);
    i2c_write_byte_(REG_ADD_GYRO_SMPLRT_DIV, 0x07);
    i2c_write_byte_(REG_ADD_GYRO_CONFIG_1, (uint8_t)(REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_1000DPS | REG_VAL_BIT_GYRO_DLPF));
    i2c_write_byte_(REG_ADD_ACCEL_SMPLRT_DIV_2, 0x07);
    i2c_write_byte_(REG_ADD_ACCEL_CONFIG, (uint8_t)(REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF));

    select_bank_(REG_VAL_REG_BANK_0);
    usleep(10 * 1000);
  }

  // read raw accel & gyro by two 6-byte bursts, safe with your register map
  bool read_accel_gyro_raw_(int16_t acc[3], int16_t gyr[3]) {
    uint8_t a[6], g[6];
    if (select_bank_(REG_VAL_REG_BANK_0) != 0) return false;
    if (i2c_read_bytes_rdwr_(REG_ADD_ACCEL_XOUT_H, a, 6) != 0) return false;
    if (i2c_read_bytes_rdwr_(REG_ADD_GYRO_XOUT_H,  g, 6) != 0) return false;

    acc[0] = (int16_t)((a[0] << 8) | a[1]);
    acc[1] = (int16_t)((a[2] << 8) | a[3]);
    acc[2] = (int16_t)((a[4] << 8) | a[5]);

    gyr[0] = (int16_t)((g[0] << 8) | g[1]);
    gyr[1] = (int16_t)((g[2] << 8) | g[3]);
    gyr[2] = (int16_t)((g[4] << 8) | g[5]);
    return true;
  }

  void push_(double t, const Eigen::Vector3f& acc_ms2, const Eigen::Vector3f& gyr_rads) {
    std::lock_guard<std::mutex> lk(m_);
    IMU::Point p(acc_ms2, gyr_rads, t);   // ORB-SLAM3 ImuTypes.h
    ring_[head_] = Sample{t, p};
    head_ = (head_ + 1) % ring_.size();
    if (size_ < ring_.size()) size_++;
  }

  static void* thread_entry_(void* arg) {
    static_cast<ICM20948Poller*>(arg)->thread_main_();
    return nullptr;
  }

  void thread_main_() {
    // Optional: raise scheduling priority for lower jitter
    if (opt_.use_realtime_fifo) {
      sched_param sp{};
      sp.sched_priority = opt_.fifo_priority;
      pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
      // ignore failure; still works as normal thread
    }

    // ----- gyro bias calibration -----
    const double dt = 1.0 / opt_.rate_hz;
    const uint64_t period_ns = (uint64_t)llround(1e9 / opt_.rate_hz);

    const uint64_t calib_ns = (uint64_t)llround(opt_.calib_seconds * 1e9);
    const uint64_t t_start = now_ns_();
    uint64_t next_t = t_start;

    double sum_gx=0, sum_gy=0, sum_gz=0;
    size_t cnt=0;

    while (!stop_.load()) {
      uint64_t t_ns = now_ns_();
      if (t_ns - t_start >= calib_ns) break;

      int16_t acc_raw[3], gyr_raw[3];
      if (read_accel_gyro_raw_(acc_raw, gyr_raw)) {
        const float deg2rad = 0.017453292519943295f;
        float gx = ((float)gyr_raw[0] / GYRO_SSF_AT_FS_1000DPS) * deg2rad;
        float gy = ((float)gyr_raw[1] / GYRO_SSF_AT_FS_1000DPS) * deg2rad;
        float gz = ((float)gyr_raw[2] / GYRO_SSF_AT_FS_1000DPS) * deg2rad;
        sum_gx += gx; sum_gy += gy; sum_gz += gz;
        cnt++;
      }

      next_t += period_ns;
      int64_t sleep_ns = (int64_t)next_t - (int64_t)now_ns_();
      if (sleep_ns > 0) {
        timespec ts{ sleep_ns / 1000000000LL, sleep_ns % 1000000000LL };
        nanosleep(&ts, nullptr);
      } else {
        next_t = now_ns_();
      }
    }

    if (cnt > 10) {
      gyro_bias_[0] = (float)(sum_gx / (double)cnt);
      gyro_bias_[1] = (float)(sum_gy / (double)cnt);
      gyro_bias_[2] = (float)(sum_gz / (double)cnt);
    }

    // ----- main poll loop -----
    next_t = now_ns_();
    while (!stop_.load()) {
      next_t += period_ns;

      int16_t acc_raw[3], gyr_raw[3];
      uint64_t t_ns = now_ns_();
      double t = (double)t_ns * 1e-9;

      if (read_accel_gyro_raw_(acc_raw, gyr_raw)) {
        const float deg2rad = 0.017453292519943295f;
        const float g_to_ms2 = 9.80665f;

        Eigen::Vector3f acc_ms2;
        acc_ms2[0] = ((float)acc_raw[0] / ACCEL_SSF_AT_FS_2g) * g_to_ms2;
        acc_ms2[1] = ((float)acc_raw[1] / ACCEL_SSF_AT_FS_2g) * g_to_ms2;
        acc_ms2[2] = ((float)acc_raw[2] / ACCEL_SSF_AT_FS_2g) * g_to_ms2;

        Eigen::Vector3f gyr_rads;
        gyr_rads[0] = ((float)gyr_raw[0] / GYRO_SSF_AT_FS_1000DPS) * deg2rad - gyro_bias_[0];
        gyr_rads[1] = ((float)gyr_raw[1] / GYRO_SSF_AT_FS_1000DPS) * deg2rad - gyro_bias_[1];
        gyr_rads[2] = ((float)gyr_raw[2] / GYRO_SSF_AT_FS_1000DPS) * deg2rad - gyro_bias_[2];

        push_(t, acc_ms2, gyr_rads);
      }

      int64_t sleep_ns = (int64_t)next_t - (int64_t)now_ns_();
      if (sleep_ns > 0) {
        timespec ts{ sleep_ns / 1000000000LL, sleep_ns % 1000000000LL };
        nanosleep(&ts, nullptr);
      } else {
        next_t = now_ns_();
      }
    }
  }

private:
  Options opt_;
  int fd_ = -1;

  std::atomic<bool> running_{false};
  std::atomic<bool> stop_{false};
  pthread_t th_{};

  // ring buffer
  std::mutex m_;
  const size_t ring_cap_;
  std::vector<Sample> ring_;
  size_t head_ = 0;
  size_t size_ = 0;

  // gyro bias (rad/s)
  float gyro_bias_[3] = {0,0,0};
};

} // namespace ORB_SLAM3
