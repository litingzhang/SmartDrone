#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <pthread.h>
#include <string>
#include <vector>

#include "ImuTypes.h"

class Icm20948Poller {
  public:
    struct Options {
        std::string m_i2cDev;
        uint8_t m_addr;
        double m_rateHz;
        double m_calibSeconds;
        size_t m_ringSeconds;
        bool m_realtimeFifo;
        int m_fifoPrio;

        Options()
            : m_i2cDev("/dev/i2c-1"), m_addr(0x68), m_rateHz(200.0), m_calibSeconds(2.0),
              m_ringSeconds(5), m_realtimeFifo(true), m_fifoPrio(80)
        {
        }
    };

    explicit Icm20948Poller(const Options &opt);
    ~Icm20948Poller();

    void Start();
    void Stop();

    std::vector<ORB_SLAM3::IMU::Point> GetBetween(double t0, double t1);

  private:
    struct RingItem {
        double m_t;
        ORB_SLAM3::IMU::Point m_p;

        RingItem();
        RingItem(double t, const ORB_SLAM3::IMU::Point &p);
    };

    static void *ThreadEntry(void *arg);
    void ThreadMain();

    void OpenI2c();
    void WhoAmICheck();
    void InitBasic();

    int I2cWrite(uint8_t reg, uint8_t val);
    int I2cRead(uint8_t reg, uint8_t *out);
    int I2cReadRdwr(uint8_t reg, uint8_t *data, uint16_t len);
    int SelectBank(uint8_t bank);

    bool ReadAccelGyroRaw(int16_t acc[3], int16_t gyr[3]);
    void Push(double t, const Eigen::Vector3f &acc, const Eigen::Vector3f &gyr);

  private:
    Options m_opt;
    int m_fd;

    std::atomic<bool> m_running;
    std::atomic<bool> m_stop;
    pthread_t m_thread;

    std::mutex m_mutex;
    size_t m_ringCap;
    std::vector<RingItem> m_ring;
    size_t m_head;
    size_t m_size;

    float m_gyroBias[3];

  private:
    static constexpr uint8_t REG_ADD_WIA = 0x00;
    static constexpr uint8_t REG_VAL_WIA = 0xEA;

    static constexpr uint8_t REG_ADD_REG_BANK_SEL = 0x7F;
    static constexpr uint8_t REG_VAL_REG_BANK_0 = 0x00;
    static constexpr uint8_t REG_VAL_REG_BANK_2 = 0x20;

    static constexpr uint8_t REG_ADD_PWR_MIGMT_1 = 0x06;
    static constexpr uint8_t REG_VAL_ALL_RGE_RESET = 0x80;
    static constexpr uint8_t REG_VAL_RUN_MODE = 0x01;

    static constexpr uint8_t REG_ADD_ACCEL_XOUT_H = 0x2D;
    static constexpr uint8_t REG_ADD_GYRO_XOUT_H = 0x33;

    static constexpr uint8_t REG_ADD_GYRO_SMPLRT_DIV = 0x00;
    static constexpr uint8_t REG_ADD_GYRO_CONFIG_1 = 0x01;
    static constexpr uint8_t REG_VAL_BIT_GYRO_DLPCFG_6 = 0x30;
    static constexpr uint8_t REG_VAL_BIT_GYRO_FS_1000DPS = 0x04;
    static constexpr uint8_t REG_VAL_BIT_GYRO_DLPF = 0x01;

    static constexpr uint8_t REG_ADD_ACCEL_SMPLRT_DIV_2 = 0x11;
    static constexpr uint8_t REG_ADD_ACCEL_CONFIG = 0x14;
    static constexpr uint8_t REG_VAL_BIT_ACCEL_DLPCFG_6 = 0x30;
    static constexpr uint8_t REG_VAL_BIT_ACCEL_FS_2g = 0x00;
    static constexpr uint8_t REG_VAL_BIT_ACCEL_DLPF = 0x01;

    static constexpr float GYRO_SSF_AT_FS_1000DPS = 32.8f;
    static constexpr float ACCEL_SSF_AT_FS_2g = 16384.0f;
};
