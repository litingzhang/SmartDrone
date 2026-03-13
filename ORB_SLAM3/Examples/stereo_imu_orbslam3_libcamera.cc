// stereo_imu_orbslam3_libcamera.cc
// Build & run inside ORB_SLAM3-master (see commands below)
//
// Usage:
//   sudo ./stereo_imu_orbslam3_libcamera Vocabulary/ORBvoc.txt path/to/settings.yaml
//
// Notes:
// - Camera: libcamera two cameras, YUV420, take Y plane as grayscale cv::Mat.
// - Timestamp: use FrameMetadata::timestamp (ns) -> seconds.
// - IMU: ICM20948 via /dev/i2c-1, poll 200Hz, gyro bias calibrated at startup.
// - Timebase: IMU uses CLOCK_MONOTONIC to better match libcamera timestamps.

#include <atomic>
#include <csignal>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <cmath>
#include <errno.h>
#include <cstring>
#include <type_traits>

#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <time.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <pthread.h>
#include <sched.h>

#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/formats.h>

#include <opencv2/core.hpp>

// ORB-SLAM3
#include "System.h"
#include "ImuTypes.h"

using namespace libcamera;
using namespace ORB_SLAM3;

static std::atomic<bool> g_run{true};
static void on_sigint(int) { g_run.store(false); }

// -------------------- Helpers: time --------------------
static inline uint64_t now_ns_monotonic()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}
static inline double ns_to_sec(uint64_t ns) { return (double)ns * 1e-9; }

// -------------------- FrameItem --------------------
struct FrameItem {
    int cam_index = -1;
    uint64_t ts_ns = 0;

    unsigned int width = 0;
    unsigned int height = 0;
    unsigned int stride = 0;
    libcamera::PixelFormat fmt;

    // packed planes copied sequentially (plane0 then plane1 then plane2...)
    std::shared_ptr<std::vector<uint8_t>> data;
};

static std::mutex g_mtx;
static std::deque<FrameItem> g_left_q;
static std::deque<FrameItem> g_right_q;

static bool copy_framebuffer_planes(libcamera::FrameBuffer *fb,
                                    std::shared_ptr<std::vector<uint8_t>> &out_blob)
{
    if (!fb) return false;
    const auto &planes = fb->planes();
    if (planes.empty()) return false;

    size_t total = 0;
    for (const auto &p : planes) total += p.length;

    auto blob = std::make_shared<std::vector<uint8_t>>();
    blob->resize(total);

    const long page = sysconf(_SC_PAGESIZE);
    const size_t pageMask = (size_t)page - 1;

    size_t outOff = 0;

    for (size_t i = 0; i < planes.size(); i++)
    {
        const auto &p = planes[i];
        int fd = p.fd.get();
        size_t len = p.length;
        size_t off = p.offset;

        size_t offAligned = off & ~pageMask;
        size_t delta = off - offAligned;
        size_t mapLen = len + delta;

        void *addr = mmap(nullptr, mapLen, PROT_READ, MAP_SHARED, fd, offAligned);
        if (addr == MAP_FAILED)
        {
            std::cerr << "mmap failed: plane=" << i
                      << " fd=" << fd
                      << " off=" << off
                      << " offAligned=" << offAligned
                      << " len=" << len
                      << " mapLen=" << mapLen
                      << " errno=" << errno
                      << " (" << strerror(errno) << ")\n";
            return false;
        }

        std::memcpy(blob->data() + outOff, (uint8_t *)addr + delta, len);
        outOff += len;

        munmap(addr, mapLen);
    }

    out_blob = std::move(blob);
    return true;
}

static void push_frame(const FrameItem &it)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    auto &q = (it.cam_index == 0) ? g_left_q : g_right_q;
    q.push_back(it);

    constexpr size_t kMax = 30;
    while (q.size() > kMax) q.pop_front();
}

static bool pop_synced_pair(FrameItem &L, FrameItem &R, uint64_t max_dt_ns)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    if (g_left_q.empty() || g_right_q.empty())
        return false;

    for (auto itL = g_left_q.begin(); itL != g_left_q.end(); ++itL)
    {
        uint64_t tL = itL->ts_ns;

        auto bestItR = g_right_q.end();
        uint64_t bestAbs = std::numeric_limits<uint64_t>::max();

        for (auto itR = g_right_q.begin(); itR != g_right_q.end(); ++itR)
        {
            uint64_t tR = itR->ts_ns;
            uint64_t absd = (tR > tL) ? (tR - tL) : (tL - tR);
            if (absd < bestAbs)
            {
                bestAbs = absd;
                bestItR = itR;
            }
        }

        if (bestItR != g_right_q.end() && bestAbs <= max_dt_ns)
        {
            L = *itL;
            R = *bestItR;

            g_left_q.erase(g_left_q.begin(), std::next(itL));
            g_right_q.erase(g_right_q.begin(), std::next(bestItR));
            return true;
        }
    }

    if (g_left_q.front().ts_ns < g_right_q.front().ts_ns)
        g_left_q.pop_front();
    else
        g_right_q.pop_front();

    return false;
}

// -------------------- Camera context --------------------
struct CamCtx
{
    int index = -1;
    std::shared_ptr<Camera> cam;
    std::unique_ptr<CameraConfiguration> config;
    std::unique_ptr<FrameBufferAllocator> alloc;
    Stream *stream = nullptr;
    std::vector<std::unique_ptr<Request>> requests;

    unsigned int width = 0;
    unsigned int height = 0;
    unsigned int stride = 0;
    PixelFormat fmt;

    void requestComplete(Request *req)
    {
        if (!req || req->status() == Request::RequestCancelled)
            return;

        auto &bufMap = req->buffers();
        auto it = bufMap.find(stream);
        if (it == bufMap.end())
            return;

        FrameBuffer *fb = it->second;
        const FrameMetadata &md = fb->metadata();

        std::shared_ptr<std::vector<uint8_t>> blob;
        if (!copy_framebuffer_planes(fb, blob))
        {
            std::cerr << "cam" << index << " copy_framebuffer_planes FAILED\n";
            req->reuse(Request::ReuseBuffers);
            cam->queueRequest(req);
            return;
        }

        FrameItem item;
        item.cam_index = index;
        item.ts_ns = md.timestamp;   // ns (libcamera)
        item.stride = stride;
        item.fmt = fmt;
        item.width = width;
        item.height = height;
        item.data = blob;

        push_frame(item);

        req->reuse(Request::ReuseBuffers);
        cam->queueRequest(req);
    }
};

static int setup_camera(CamCtx &ctx,
                        const std::shared_ptr<Camera> &camera,
                        unsigned int width,
                        unsigned int height)
{
    ctx.cam = camera;

    if (ctx.cam->acquire())
    {
        std::cerr << "Failed to acquire camera " << ctx.index << "\n";
        return -1;
    }

    ctx.config = ctx.cam->generateConfiguration({StreamRole::Viewfinder});
    if (!ctx.config)
    {
        std::cerr << "Failed to generate configuration (cam " << ctx.index << ")\n";
        return -1;
    }

    StreamConfiguration &sc = ctx.config->at(0);
    sc.size.width = width;
    sc.size.height = height;
    sc.pixelFormat = formats::YUV420;

    ctx.config->validate();
    if (ctx.cam->configure(ctx.config.get()) < 0)
    {
        std::cerr << "Failed to configure camera " << ctx.index << "\n";
        return -1;
    }

    ctx.stream = sc.stream();
    ctx.width = sc.size.width;
    ctx.height = sc.size.height;
    ctx.stride = sc.stride;
    ctx.fmt = sc.pixelFormat;

    std::cerr << "cam" << ctx.index
              << " configured: " << sc.size.toString()
              << " stride=" << sc.stride
              << " pixelFormat=" << sc.pixelFormat.toString()
              << "\n";

    ctx.alloc = std::make_unique<FrameBufferAllocator>(ctx.cam);
    if (ctx.alloc->allocate(ctx.stream) < 0)
    {
        std::cerr << "Failed to allocate buffers (cam " << ctx.index << ")\n";
        return -1;
    }

    ctx.cam->requestCompleted.connect(&ctx, &CamCtx::requestComplete);

    const auto &buffers = ctx.alloc->buffers(ctx.stream);
    for (const std::unique_ptr<FrameBuffer> &buffer : buffers)
    {
        std::unique_ptr<Request> req = ctx.cam->createRequest();
        if (!req)
        {
            std::cerr << "Failed to create request (cam " << ctx.index << ")\n";
            return -1;
        }
        if (req->addBuffer(ctx.stream, buffer.get()) < 0)
        {
            std::cerr << "Failed to add buffer to request (cam " << ctx.index << ")\n";
            return -1;
        }
        ctx.requests.push_back(std::move(req));
    }

    if (ctx.cam->start() < 0)
    {
        std::cerr << "Failed to start camera " << ctx.index << "\n";
        return -1;
    }

    for (auto &req : ctx.requests)
    {
        if (ctx.cam->queueRequest(req.get()) < 0)
        {
            std::cerr << "Failed to queue request (cam " << ctx.index << ")\n";
            return -1;
        }
    }

    return 0;
}

static void stop_camera(CamCtx &ctx)
{
    if (!ctx.cam) return;
    ctx.cam->stop();
    ctx.cam->release();
}

// -------------------- IMU Poller (ICM20948) --------------------
class ICM20948Poller {
public:
    struct Options {
        std::string i2c_dev = "/dev/i2c-1";
        uint8_t addr = 0x68;
        double rate_hz = 200.0;
        double calib_seconds = 2.0;
        size_t ring_seconds = 5;
        bool realtime_fifo = true;
        int fifo_prio = 80;
    };

    explicit ICM20948Poller(const Options& opt)
    : opt_(opt)
    {
        ring_cap_ = (size_t)std::ceil(opt_.rate_hz * opt_.ring_seconds) + 64;
        ring_.resize(ring_cap_);
        open_i2c_();
        whoami_check_();
        init_basic_();
    }

    ~ICM20948Poller() {
        Stop();
        if (fd_ >= 0) ::close(fd_);
    }

    void Start() {
        if (running_.exchange(true)) return;
        stop_.store(false);
        int rc = pthread_create(&th_, nullptr, &ICM20948Poller::thread_entry, this);
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

    std::vector<IMU::Point> GetBetween(double t0, double t1) {
        std::lock_guard<std::mutex> lk(m_);
        std::vector<IMU::Point> out;
        out.reserve(256);

        for (size_t i = 0; i < size_; i++) {
            size_t idx = (head_ + ring_.size() - size_ + i) % ring_.size();
            const auto& s = ring_[idx];
            if (s.t > t0 && s.t <= t1) out.push_back(s.p);
        }
        return out;
    }

private:
    // Register map: match your ICM20948.h
    static constexpr uint8_t REG_ADD_WIA = 0x00;
    static constexpr uint8_t REG_VAL_WIA = 0xEA;

    static constexpr uint8_t REG_ADD_REG_BANK_SEL = 0x7F;
    static constexpr uint8_t REG_VAL_REG_BANK_0   = 0x00;
    static constexpr uint8_t REG_VAL_REG_BANK_2   = 0x20;

    static constexpr uint8_t REG_ADD_PWR_MIGMT_1   = 0x06;
    static constexpr uint8_t REG_VAL_ALL_RGE_RESET = 0x80;
    static constexpr uint8_t REG_VAL_RUN_MODE      = 0x01;

    static constexpr uint8_t REG_ADD_ACCEL_XOUT_H  = 0x2D;
    static constexpr uint8_t REG_ADD_GYRO_XOUT_H   = 0x33;

    static constexpr uint8_t REG_ADD_GYRO_SMPLRT_DIV = 0x00;
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

    struct RingItem {
        double t = 0.0;
        ORB_SLAM3::IMU::Point p;

        // 必须显式构造 p（IMU::Point 没有默认构造）
        RingItem()
            : t(0.0),
            p(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.0) {}

        RingItem(double tt, const ORB_SLAM3::IMU::Point& pp)
            : t(tt), p(pp) {}
    };

    static void* thread_entry(void* arg) {
        ((ICM20948Poller*)arg)->thread_main_();
        return nullptr;
    }

    void thread_main_() {
        if (opt_.realtime_fifo) {
            sched_param sp{};
            sp.sched_priority = opt_.fifo_prio;
            pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        }

        const uint64_t period_ns = (uint64_t)llround(1e9 / opt_.rate_hz);

        // ---- gyro bias calib ----
        uint64_t t0 = now_ns_monotonic();
        uint64_t next_t = t0;
        const uint64_t calib_ns = (uint64_t)llround(opt_.calib_seconds * 1e9);

        double sum_gx=0, sum_gy=0, sum_gz=0;
        size_t cnt=0;

        while (!stop_.load()) {
            uint64_t tn = now_ns_monotonic();
            if (tn - t0 >= calib_ns) break;

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
            int64_t sleep_ns = (int64_t)next_t - (int64_t)now_ns_monotonic();
            if (sleep_ns > 0) {
                timespec ts{ sleep_ns / 1000000000LL, sleep_ns % 1000000000LL };
                nanosleep(&ts, nullptr);
            } else {
                next_t = now_ns_monotonic();
            }
        }

        if (cnt > 10) {
            gyro_bias_[0] = (float)(sum_gx / (double)cnt);
            gyro_bias_[1] = (float)(sum_gy / (double)cnt);
            gyro_bias_[2] = (float)(sum_gz / (double)cnt);
        }
        std::cerr << "[IMU] gyro bias(rad/s): "
                  << gyro_bias_[0] << " " << gyro_bias_[1] << " " << gyro_bias_[2] << "\n";

        // ---- main loop ----
        next_t = now_ns_monotonic();
        while (!stop_.load()) {
            next_t += period_ns;

            int16_t acc_raw[3], gyr_raw[3];
            uint64_t tn = now_ns_monotonic();
            double t = (double)tn * 1e-9;

            if (read_accel_gyro_raw_(acc_raw, gyr_raw)) {
                const float deg2rad = 0.017453292519943295f;
                const float g_to_ms2 = 9.80665f;

                Eigen::Vector3f acc;
                acc[0] = ((float)acc_raw[0] / ACCEL_SSF_AT_FS_2g) * g_to_ms2;
                acc[1] = ((float)acc_raw[1] / ACCEL_SSF_AT_FS_2g) * g_to_ms2;
                acc[2] = ((float)acc_raw[2] / ACCEL_SSF_AT_FS_2g) * g_to_ms2;

                Eigen::Vector3f gyr;
                gyr[0] = ((float)gyr_raw[0] / GYRO_SSF_AT_FS_1000DPS) * deg2rad - gyro_bias_[0];
                gyr[1] = ((float)gyr_raw[1] / GYRO_SSF_AT_FS_1000DPS) * deg2rad - gyro_bias_[1];
                gyr[2] = ((float)gyr_raw[2] / GYRO_SSF_AT_FS_1000DPS) * deg2rad - gyro_bias_[2];

                push_(t, acc, gyr);
            }

            int64_t sleep_ns = (int64_t)next_t - (int64_t)now_ns_monotonic();
            if (sleep_ns > 0) {
                timespec ts{ sleep_ns / 1000000000LL, sleep_ns % 1000000000LL };
                nanosleep(&ts, nullptr);
            } else {
                next_t = now_ns_monotonic();
            }
        }
    }

    void open_i2c_() {
        fd_ = ::open(opt_.i2c_dev.c_str(), O_RDWR);
        if (fd_ < 0)
            throw std::runtime_error("open(" + opt_.i2c_dev + ") failed: " + std::string(std::strerror(errno)));
    }

    int i2c_write_(uint8_t reg, uint8_t val) {
        if (ioctl(fd_, I2C_SLAVE, opt_.addr) < 0) return -1;
        uint8_t buf[2] = {reg, val};
        return (::write(fd_, buf, 2) == 2) ? 0 : -1;
    }

    int i2c_read_(uint8_t reg, uint8_t* out) {
        if (ioctl(fd_, I2C_SLAVE, opt_.addr) < 0) return -1;
        if (::write(fd_, &reg, 1) != 1) return -1;
        if (::read(fd_, out, 1) != 1) return -1;
        return 0;
    }

    int i2c_read_rdwr_(uint8_t reg, uint8_t* data, uint16_t len) {
        struct i2c_rdwr_ioctl_data ioctl_data;
        struct i2c_msg msgs[2];

        msgs[0].addr  = opt_.addr;
        msgs[0].flags = 0;
        msgs[0].len   = 1;
        msgs[0].buf   = &reg;

        msgs[1].addr  = opt_.addr;
        msgs[1].flags = I2C_M_RD;
        msgs[1].len   = len;
        msgs[1].buf   = data;

        ioctl_data.msgs = msgs;
        ioctl_data.nmsgs = 2;
        if (ioctl(fd_, I2C_RDWR, &ioctl_data) < 0) return -1;
        return 0;
    }

    int select_bank_(uint8_t bank) {
        return i2c_write_(REG_ADD_REG_BANK_SEL, bank);
    }

    void whoami_check_() {
        select_bank_(REG_VAL_REG_BANK_0);
        uint8_t v = 0;
        if (i2c_read_(REG_ADD_WIA, &v) != 0 || v != REG_VAL_WIA)
            throw std::runtime_error("ICM20948 WHOAMI mismatch (addr/bus wrong?)");
    }

    void init_basic_() {
        select_bank_(REG_VAL_REG_BANK_0);
        i2c_write_(REG_ADD_PWR_MIGMT_1, REG_VAL_ALL_RGE_RESET);
        usleep(10 * 1000);
        i2c_write_(REG_ADD_PWR_MIGMT_1, REG_VAL_RUN_MODE);
        usleep(10 * 1000);

        select_bank_(REG_VAL_REG_BANK_2);
        i2c_write_(REG_ADD_GYRO_SMPLRT_DIV, 0x07);
        i2c_write_(REG_ADD_GYRO_CONFIG_1, (uint8_t)(REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_1000DPS | REG_VAL_BIT_GYRO_DLPF));
        i2c_write_(REG_ADD_ACCEL_SMPLRT_DIV_2, 0x07);
        i2c_write_(REG_ADD_ACCEL_CONFIG, (uint8_t)(REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF));
        select_bank_(REG_VAL_REG_BANK_0);
        usleep(10 * 1000);
    }

    bool read_accel_gyro_raw_(int16_t acc[3], int16_t gyr[3]) {
        uint8_t a[6], g[6];
        if (select_bank_(REG_VAL_REG_BANK_0) != 0) return false;
        if (i2c_read_rdwr_(REG_ADD_ACCEL_XOUT_H, a, 6) != 0) return false;
        if (i2c_read_rdwr_(REG_ADD_GYRO_XOUT_H,  g, 6) != 0) return false;

        acc[0] = (int16_t)((a[0] << 8) | a[1]);
        acc[1] = (int16_t)((a[2] << 8) | a[3]);
        acc[2] = (int16_t)((a[4] << 8) | a[5]);

        gyr[0] = (int16_t)((g[0] << 8) | g[1]);
        gyr[1] = (int16_t)((g[2] << 8) | g[3]);
        gyr[2] = (int16_t)((g[4] << 8) | g[5]);
        return true;
    }

    void push_(double t, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyr) {
        std::lock_guard<std::mutex> lk(m_);

        IMU::Point p(
            acc[0], acc[1], acc[2],
            gyr[0], gyr[1], gyr[2],
            t
        );

        ring_[head_] = RingItem{t, p};
        head_ = (head_ + 1) % ring_.size();
        if (size_ < ring_.size()) size_++;
    }

private:
    Options opt_;
    int fd_ = -1;

    std::atomic<bool> running_{false};
    std::atomic<bool> stop_{false};
    pthread_t th_{};

    std::mutex m_;
    size_t ring_cap_{0};
    std::vector<RingItem> ring_;
    size_t head_{0};
    size_t size_{0};

    float gyro_bias_[3] = {0,0,0};
};

// -------------------- Convert Y plane to cv::Mat (copy row-by-row) --------------------
static inline cv::Mat yplane_to_gray_mat(const FrameItem& it)
{
    // libcamera YUV420: plane0 is Y, size roughly stride*height
    cv::Mat gray((int)it.height, (int)it.width, CV_8UC1);
    const uint8_t* y = it.data->data();
    for (unsigned int r = 0; r < it.height; r++) {
        std::memcpy(gray.ptr(r), y + r * it.stride, it.width);
    }
    return gray;
}

// -------------------- main --------------------
int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage:\n  sudo " << argv[0] << " <vocab_path> <settings_yaml>\n";
        return 1;
    }
    const std::string vocab_path = argv[1];
    const std::string settings_path = argv[2];

    std::signal(SIGINT, on_sigint);

    // 1) ORB-SLAM3 system (IMU_STEREO)
    System SLAM(vocab_path, settings_path, System::IMU_STEREO, false);

    // 2) Start IMU poller (keep still for calib_seconds)
    ICM20948Poller::Options imu_opt;
    imu_opt.rate_hz = 200.0;
    imu_opt.calib_seconds = 2.0;
    imu_opt.realtime_fifo = true;   // needs sudo; if fails, still runs
    imu_opt.fifo_prio = 80;
    ICM20948Poller imu(imu_opt);
    imu.Start();

    std::cerr << "[IMU] calibrating... keep still for " << imu_opt.calib_seconds << "s\n";

    // 3) Cameras
    CameraManager cm;
    if (cm.start())
    {
        std::cerr << "CameraManager start failed\n";
        return 1;
    }

    auto cams = cm.cameras();
    if (cams.size() < 2)
    {
        std::cerr << "Need 2 cameras; found " << cams.size() << "\n";
        cm.stop();
        return 1;
    }

    CamCtx left;  left.index = 0;
    CamCtx right; right.index = 1;

    const unsigned int W = 640, H = 480;

    if (setup_camera(left, cams[0], W, H) < 0)
    {
        stop_camera(left);
        cm.stop();
        return 1;
    }

    if (setup_camera(right, cams[1], W, H) < 0)
    {
        stop_camera(left);
        stop_camera(right);
        cm.stop();
        return 1;
    }

    const uint64_t max_dt_ns = 30ULL * 1000ULL * 1000ULL; // 30ms

    // Learn cam1-cam0 offset (your logic)
    bool have_offset = false;
    int n_off = 0;
    double offset_ns = 0.0; // cam1 - cam0

    // ORB-SLAM3 timestamp (seconds) based on left cam timestamp
    double last_t = -1.0;

    // Main loop
    while (g_run.load())
    {
        FrameItem L, R;
        if (!pop_synced_pair(L, R, max_dt_ns)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (!have_offset) {
            double cur = double(R.ts_ns) - double(L.ts_ns);
            offset_ns = (offset_ns * n_off + cur) / double(n_off + 1);
            n_off++;
            if (n_off % 10 == 0)
                std::cerr << "[CAM] learning offset: " << offset_ns * 1e-6 << " ms\n";
            if (n_off >= 60) {
                have_offset = true;
                std::cerr << "[CAM] final offset: " << offset_ns * 1e-6 << " ms\n";
                // initialize last_t on first usable pair
                last_t = ns_to_sec(L.ts_ns);
            }
            continue;
        }

        // Convert to grayscale cv::Mat
        cv::Mat imL = yplane_to_gray_mat(L);
        cv::Mat imR = yplane_to_gray_mat(R);

        double meanL = cv::mean(imL)[0];
        double meanR = cv::mean(imR)[0];
        std::cout << "[IMG] meanL=" << meanL << " meanR=" << meanR
                << " w=" << imL.cols << " h=" << imL.rows
                << " stride=" << L.stride
                << "\n";

        // Use left timestamp as frame timestamp
        const double t = ns_to_sec(L.ts_ns);

        // Get IMU between last frame and current frame
        std::vector<IMU::Point> vImu;
        if (last_t > 0.0) vImu = imu.GetBetween(last_t, t);

        // Minimal guard: for 200Hz, at 30fps you expect ~6-8 imu samples
        if (last_t > 0.0 && vImu.size() < 2) {
            // 如果经常触发，说明时间基准不一致或 IMU consumption 太快/太慢
            // 先跳过这帧，避免喂空导致内部断言或不稳定
            last_t = t;
            continue;
        }

        // Feed ORB-SLAM3 + print pose
        Sophus::SE3f Tcw = SLAM.TrackStereo(imL, imR, t, vImu);
        int st = SLAM.GetTrackingState();
        auto mps = SLAM.GetTrackedMapPoints();
        auto kps = SLAM.GetTrackedKeyPointsUn();

        int n_mps = 0;
        for (auto* p : mps) if (p) n_mps++;

        std::cout
        << "[TRACK] t=" << t
        << " state=" << st              // 重点看是不是一直 NOT_INITIALIZED/LOST
        << " kp=" << kps.size()
        << " mp=" << n_mps
        << " imu=" << vImu.size()
        << "\n";

        // 如果 tracking fail，ORB-SLAM3 通常会返回 identity 或者内部标志，你可以简单用矩阵判断一下
        Eigen::Matrix4f Tcw_m = Tcw.matrix();
        bool valid = Tcw_m.allFinite();

        // ORB-SLAM3 返回的是 Tcw: world -> camera
        // 我们通常打印 Twc: camera -> world (相机在世界坐标系下的位姿)
        if (valid) {
            Sophus::SE3f Twc = Tcw.inverse();

            Eigen::Vector3f p_w = Twc.translation();           // position in world
            Eigen::Quaternionf q_w(Twc.so3().unit_quaternion()); // orientation in world

            // 你可以按需改格式：时间戳 + xyz + qxyzw
            std::cout.setf(std::ios::fixed);
            std::cout.precision(6);

            std::cout
                << "[POSE] t=" << t
                << " p=(" << p_w.x() << "," << p_w.y() << "," << p_w.z() << ")"
                << " q=(" << q_w.x() << "," << q_w.y() << "," << q_w.z() << "," << q_w.w() << ")"
                << "\n";
        } else {
            std::cout << "[POSE] t=" << t << " INVALID\n";
        }
        last_t = t;
    }

    // Cleanup
    imu.Stop();
    stop_camera(left);
    stop_camera(right);
    cm.stop();

    SLAM.Shutdown();
    return 0;
}
