#include "Icm20948Poller.h"

#include <cmath>
#include <cstring>
#include <errno.h>
#include <iostream>
#include <stdexcept>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <pthread.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "TimeUtils.h"

Icm20948Poller::RingItem::RingItem() : m_t(0.0), m_p(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.0) {}

Icm20948Poller::RingItem::RingItem(double t, const ORB_SLAM3::IMU::Point &p) : m_t(t), m_p(p) {}

Icm20948Poller::Icm20948Poller(const Options &opt)
    : m_opt(opt), m_fd(-1), m_running(false), m_stop(false), m_thread(), m_mutex(), m_ringCap(0),
      m_ring(), m_head(0), m_size(0), m_gyroBias{0, 0, 0}
{
    m_ringCap = static_cast<size_t>(std::ceil(m_opt.m_rateHz * m_opt.m_ringSeconds)) + 64;
    m_ring.resize(m_ringCap);

    OpenI2c();
    WhoAmICheck();
    InitBasic();
}

Icm20948Poller::~Icm20948Poller()
{
    Stop();
    if (m_fd >= 0)
        ::close(m_fd);
}

void Icm20948Poller::Start()
{
    if (m_running.exchange(true))
        return;

    m_stop.store(false);

    int rc = pthread_create(&m_thread, nullptr, &Icm20948Poller::ThreadEntry, this);
    if (rc != 0) {
        m_running.store(false);
        throw std::runtime_error(std::string("pthread_create failed: ") + std::strerror(rc));
    }
}

void Icm20948Poller::Stop()
{
    if (!m_running.load())
        return;

    m_stop.store(true);
    pthread_join(m_thread, nullptr);
    m_running.store(false);
}

std::vector<ORB_SLAM3::IMU::Point> Icm20948Poller::GetBetween(double t0, double t1)
{
    std::lock_guard<std::mutex> lk(m_mutex);

    std::vector<ORB_SLAM3::IMU::Point> out;
    out.reserve(256);

    for (size_t i = 0; i < m_size; i++) {
        size_t idx = (m_head + m_ring.size() - m_size + i) % m_ring.size();
        const auto &s = m_ring[idx];

        if (s.m_t > t0 && s.m_t <= t1)
            out.push_back(s.m_p);
    }

    return out;
}

void *Icm20948Poller::ThreadEntry(void *arg)
{
    static_cast<Icm20948Poller *>(arg)->ThreadMain();
    return nullptr;
}

void Icm20948Poller::ThreadMain()
{
    if (m_opt.m_realtimeFifo) {
        sched_param sp;
        std::memset(&sp, 0, sizeof(sp));
        sp.sched_priority = m_opt.m_fifoPrio;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    }

    const uint64_t periodNs = static_cast<uint64_t>(llround(1e9 / m_opt.m_rateHz));

    uint64_t t0 = TimeUtils::NowNsMonotonic();
    uint64_t nextT = t0;
    const uint64_t calibNs = static_cast<uint64_t>(llround(m_opt.m_calibSeconds * 1e9));

    double sumGx = 0.0;
    double sumGy = 0.0;
    double sumGz = 0.0;
    size_t cnt = 0;

    while (!m_stop.load()) {
        uint64_t tn = TimeUtils::NowNsMonotonic();
        if (tn - t0 >= calibNs)
            break;

        int16_t accRaw[3];
        int16_t gyrRaw[3];
        if (ReadAccelGyroRaw(accRaw, gyrRaw)) {
            const float deg2rad = 0.017453292519943295f;

            float gx = (static_cast<float>(gyrRaw[0]) / GYRO_SSF_AT_FS_1000DPS) * deg2rad;
            float gy = (static_cast<float>(gyrRaw[1]) / GYRO_SSF_AT_FS_1000DPS) * deg2rad;
            float gz = (static_cast<float>(gyrRaw[2]) / GYRO_SSF_AT_FS_1000DPS) * deg2rad;

            sumGx += gx;
            sumGy += gy;
            sumGz += gz;
            cnt++;
        }

        nextT += periodNs;
        int64_t sleepNs =
            static_cast<int64_t>(nextT) - static_cast<int64_t>(TimeUtils::NowNsMonotonic());

        if (sleepNs > 0) {
            timespec ts;
            ts.tv_sec = sleepNs / 1000000000LL;
            ts.tv_nsec = sleepNs % 1000000000LL;
            nanosleep(&ts, nullptr);
        } else {
            nextT = TimeUtils::NowNsMonotonic();
        }
    }

    if (cnt > 10) {
        m_gyroBias[0] = static_cast<float>(sumGx / static_cast<double>(cnt));
        m_gyroBias[1] = static_cast<float>(sumGy / static_cast<double>(cnt));
        m_gyroBias[2] = static_cast<float>(sumGz / static_cast<double>(cnt));
    }

    std::cerr << "[IMU] gyro bias(rad/s): " << m_gyroBias[0] << " " << m_gyroBias[1] << " "
              << m_gyroBias[2] << "\n";

    nextT = TimeUtils::NowNsMonotonic();

    while (!m_stop.load()) {
        nextT += periodNs;

        int16_t accRaw[3];
        int16_t gyrRaw[3];

        uint64_t tn = TimeUtils::NowNsMonotonic();
        double t = static_cast<double>(tn) * 1e-9;

        if (ReadAccelGyroRaw(accRaw, gyrRaw)) {
            const float deg2rad = 0.017453292519943295f;
            const float gToMs2 = 9.80665f;

            Eigen::Vector3f acc;
            acc[0] = (static_cast<float>(accRaw[0]) / ACCEL_SSF_AT_FS_2g) * gToMs2;
            acc[1] = (static_cast<float>(accRaw[1]) / ACCEL_SSF_AT_FS_2g) * gToMs2;
            acc[2] = (static_cast<float>(accRaw[2]) / ACCEL_SSF_AT_FS_2g) * gToMs2;

            Eigen::Vector3f gyr;
            gyr[0] =
                (static_cast<float>(gyrRaw[0]) / GYRO_SSF_AT_FS_1000DPS) * deg2rad - m_gyroBias[0];
            gyr[1] =
                (static_cast<float>(gyrRaw[1]) / GYRO_SSF_AT_FS_1000DPS) * deg2rad - m_gyroBias[1];
            gyr[2] =
                (static_cast<float>(gyrRaw[2]) / GYRO_SSF_AT_FS_1000DPS) * deg2rad - m_gyroBias[2];

            Push(t, acc, gyr);
        }

        int64_t sleepNs =
            static_cast<int64_t>(nextT) - static_cast<int64_t>(TimeUtils::NowNsMonotonic());
        if (sleepNs > 0) {
            timespec ts;
            ts.tv_sec = sleepNs / 1000000000LL;
            ts.tv_nsec = sleepNs % 1000000000LL;
            nanosleep(&ts, nullptr);
        } else {
            nextT = TimeUtils::NowNsMonotonic();
        }
    }
}

void Icm20948Poller::OpenI2c()
{
    m_fd = ::open(m_opt.m_i2cDev.c_str(), O_RDWR);
    if (m_fd < 0) {
        throw std::runtime_error("open(" + m_opt.m_i2cDev +
                                 ") failed: " + std::string(std::strerror(errno)));
    }
}

int Icm20948Poller::I2cWrite(uint8_t reg, uint8_t val)
{
    if (ioctl(m_fd, I2C_SLAVE, m_opt.m_addr) < 0)
        return -1;

    uint8_t buf[2] = {reg, val};
    return (::write(m_fd, buf, 2) == 2) ? 0 : -1;
}

int Icm20948Poller::I2cRead(uint8_t reg, uint8_t *out)
{
    if (ioctl(m_fd, I2C_SLAVE, m_opt.m_addr) < 0)
        return -1;

    if (::write(m_fd, &reg, 1) != 1)
        return -1;

    if (::read(m_fd, out, 1) != 1)
        return -1;

    return 0;
}

int Icm20948Poller::I2cReadRdwr(uint8_t reg, uint8_t *data, uint16_t len)
{
    struct i2c_rdwr_ioctl_data ioctlData;
    struct i2c_msg msgs[2];

    msgs[0].addr = m_opt.m_addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &reg;

    msgs[1].addr = m_opt.m_addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = data;

    ioctlData.msgs = msgs;
    ioctlData.nmsgs = 2;

    if (ioctl(m_fd, I2C_RDWR, &ioctlData) < 0)
        return -1;

    return 0;
}

int Icm20948Poller::SelectBank(uint8_t bank) { return I2cWrite(REG_ADD_REG_BANK_SEL, bank); }

void Icm20948Poller::WhoAmICheck()
{
    SelectBank(REG_VAL_REG_BANK_0);

    uint8_t v = 0;
    if (I2cRead(REG_ADD_WIA, &v) != 0 || v != REG_VAL_WIA)
        throw std::runtime_error("ICM20948 WHOAMI mismatch (addr/bus wrong?)");
}

void Icm20948Poller::InitBasic()
{
    SelectBank(REG_VAL_REG_BANK_0);
    I2cWrite(REG_ADD_PWR_MIGMT_1, REG_VAL_ALL_RGE_RESET);
    usleep(10 * 1000);
    I2cWrite(REG_ADD_PWR_MIGMT_1, REG_VAL_RUN_MODE);
    usleep(10 * 1000);

    SelectBank(REG_VAL_REG_BANK_2);
    I2cWrite(REG_ADD_GYRO_SMPLRT_DIV, 0x07);
    I2cWrite(REG_ADD_GYRO_CONFIG_1,
             static_cast<uint8_t>(REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_1000DPS |
                                  REG_VAL_BIT_GYRO_DLPF));
    I2cWrite(REG_ADD_ACCEL_SMPLRT_DIV_2, 0x07);
    I2cWrite(REG_ADD_ACCEL_CONFIG,
             static_cast<uint8_t>(REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g |
                                  REG_VAL_BIT_ACCEL_DLPF));
    SelectBank(REG_VAL_REG_BANK_0);
    usleep(10 * 1000);
}

bool Icm20948Poller::ReadAccelGyroRaw(int16_t acc[3], int16_t gyr[3])
{
    uint8_t a[6];
    uint8_t g[6];

    if (SelectBank(REG_VAL_REG_BANK_0) != 0)
        return false;

    if (I2cReadRdwr(REG_ADD_ACCEL_XOUT_H, a, 6) != 0)
        return false;

    if (I2cReadRdwr(REG_ADD_GYRO_XOUT_H, g, 6) != 0)
        return false;

    acc[0] = static_cast<int16_t>((a[0] << 8) | a[1]);
    acc[1] = static_cast<int16_t>((a[2] << 8) | a[3]);
    acc[2] = static_cast<int16_t>((a[4] << 8) | a[5]);

    gyr[0] = static_cast<int16_t>((g[0] << 8) | g[1]);
    gyr[1] = static_cast<int16_t>((g[2] << 8) | g[3]);
    gyr[2] = static_cast<int16_t>((g[4] << 8) | g[5]);

    return true;
}

void Icm20948Poller::Push(double t, const Eigen::Vector3f &acc, const Eigen::Vector3f &gyr)
{
    std::lock_guard<std::mutex> lk(m_mutex);

    ORB_SLAM3::IMU::Point p(acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2], t);

    m_ring[m_head] = RingItem(t, p);
    m_head = (m_head + 1) % m_ring.size();

    if (m_size < m_ring.size())
        m_size++;
}
