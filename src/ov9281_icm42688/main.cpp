#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include "ImuTypes.h"
#include "System.h"
#include "app_args.hpp"
#include "drdy_gpio.hpp"
#include "icm42688_imu.hpp"
#include "imu_buffer.hpp"
#include "logger.hpp"
#include "mavlink_pose_sender.hpp"
#include "stereo_ov9281.hpp"
#include "udp_image_sender.hpp"

#include "../udp_server/mavlink_hooks.hpp"
#include "../udp_server/tlv_cmd_router.hpp"
#include "../udp_server/tlv_pack.hpp"
#include "../udp_server/tlv_parser.hpp"
#include "../udp_server/tlv_protocol.hpp"
#include "../udp_server/udp_server.hpp"

#include "../udp_server/tlv_cmd_router.cpp"
#include <sophus/se3.hpp>

namespace fs = std::filesystem;

namespace {

enum class UnifiedMode : uint8_t { Idle = RUNTIME_MODE_IDLE, Slam = RUNTIME_MODE_SLAM, Calib = RUNTIME_MODE_CALIB };

struct CalibConfig {
    std::string root{"./calib_runs"};
    int maxFrames{0};
};

struct UnifiedConfig {
    AppConfig app;
    CalibConfig calib;
};

struct RemoteRuntimeConfig {
    int exposureUs{6000};
    float gain{4.0f};
    std::string udpIp;
};

struct MainRuntimeAliases {
    SensorMode sensorMode{SensorMode::Stereo};
    int width{}, height{}, fps{}, exposureUs{}, pairMs{}, keepMs{};
    bool aeDisable{}, requestY8{}, r16Norm{}, udpEnable{}, allowEmptyImu{}, rtImu{};
    float gain{};
    std::string udpIp, spiDev, gpiochip;
    int udpPort{}, cmdPort{}, udpJpegQ{}, udpPayload{}, udpQueue{}, imuHz{}, accelFsG{}, gyroFsDps{},
        rtPrio{};
    uint32_t spiSpeed{};
    uint8_t spiMode{}, spiBits{}, imuStartReg{};
    unsigned drdyLine{};
};

struct ImuThreadState {
    ImuBuffer imuBuffer;
    std::atomic<bool> imuOk{false};
    std::atomic<uint64_t> imuCnt{0};
    std::atomic<uint64_t> imuDrop{0};
    std::atomic<float> accelLsbPerG{0.0f};
    std::atomic<float> gyroLsbPerDps{0.0f};
};

struct LivePoseState {
    struct Snapshot {
        bool hasPeer{false};
        UdpPeer peer{};
        bool poseValid{false};
        uint8_t runtimeMode{RUNTIME_MODE_IDLE};
        uint8_t trackingState{0xFF};
        float x{0.0f}, y{0.0f}, z{0.0f};
        float qw{1.0f}, qx{0.0f}, qy{0.0f}, qz{0.0f};
        uint32_t seq{0};
    };

    void UpdatePeer(const UdpPeer& peer)
    {
        if (!peer.valid) return;
        std::lock_guard<std::mutex> lock(mu);
        latestPeer = peer;
        hasPeer = true;
    }

    void SetRuntimeMode(uint8_t mode)
    {
        std::lock_guard<std::mutex> lock(mu);
        runtimeMode = mode;
        if (mode != RUNTIME_MODE_SLAM) {
            poseValid = false;
            trackingState = 0xFF;
        }
        dirty = true;
    }

    void UpdatePose(uint8_t mode, uint8_t tracking, const MavlinkSerial::Pose& p)
    {
        std::lock_guard<std::mutex> lock(mu);
        runtimeMode = mode;
        trackingState = tracking;
        x = p.x; y = p.y; z = p.z;
        qw = p.qw; qx = p.qx; qy = p.qy; qz = p.qz;
        poseValid = true;
        dirty = true;
    }

    bool ConsumeSnapshot(Snapshot& out)
    {
        std::lock_guard<std::mutex> lock(mu);
        if (!hasPeer || !dirty) return false;
        out.hasPeer = hasPeer;
        out.peer = latestPeer;
        out.poseValid = poseValid;
        out.runtimeMode = runtimeMode;
        out.trackingState = trackingState;
        out.x = x; out.y = y; out.z = z;
        out.qw = qw; out.qx = qx; out.qy = qy; out.qz = qz;
        out.seq = ++txSeq;
        dirty = false;
        return true;
    }

    std::mutex mu;
    UdpPeer latestPeer{};
    bool hasPeer{false};
    bool poseValid{false};
    uint8_t runtimeMode{RUNTIME_MODE_IDLE};
    uint8_t trackingState{0xFF};
    float x{0.0f}, y{0.0f}, z{0.0f};
    float qw{1.0f}, qx{0.0f}, qy{0.0f}, qz{0.0f};
    uint32_t txSeq{1};
    bool dirty{false};
};

class Px4UdpHooks final : public MavlinkHooks {
public:
    explicit Px4UdpHooks(MavlinkSerial& mavlink) : m_mavlink(mavlink) {}
    VehicleGate GetGate() const override { return VehicleGate{}; }
    bool Arm(std::string* err) override
    {
        if (!EnsureSetpointStream()) {
            if (err) *err = "setpoint stream start failed";
            return false;
        }
        if (!m_mavlink.Arm(true)) {
            if (err) *err = "px4 arm rejected";
            return false;
        }
        return true;
    }
    bool Disarm(std::string* err) override
    {
        if (!m_mavlink.Arm(false)) {
            if (err) *err = "px4 disarm rejected";
            return false;
        }
        return true;
    }
    bool SetOffboard(std::string* err) override
    {
        if (!EnsureSetpointStream()) {
            if (err) *err = "setpoint stream start failed";
            return false;
        }
        if (!m_mavlink.SetModeOffboard()) {
            if (err) *err = "px4 offboard rejected";
            return false;
        }
        return true;
    }
    bool Hold(std::string*) override
    {
        if (!EnsureSetpointStream()) return false;
        MavlinkSerial::SetpointLocalNED setpoint{};
        setpoint.x = NAN; setpoint.y = NAN; setpoint.z = NAN;
        setpoint.vx = 0.0f; setpoint.vy = 0.0f; setpoint.vz = 0.0f;
        setpoint.yaw = NAN; setpoint.yawspeed = 0.0f;
        m_mavlink.UpdateStreamSetpoint(setpoint);
        return true;
    }
    bool Land(std::string* err) override
    {
        m_mavlink.StopSetpointStream();
        m_streamStarted.store(false, std::memory_order_relaxed);
        if (!m_mavlink.SendLand()) {
            if (err) *err = "px4 land rejected";
            return false;
        }
        return true;
    }
    bool SetMoveGoal(const MoveGoal& goal, std::string*) override
    {
        if (!EnsureSetpointStream()) return false;
        MavlinkSerial::SetpointLocalNED setpoint{};
        if (goal.isVelocity) {
            setpoint.x = NAN; setpoint.y = NAN; setpoint.z = NAN;
            setpoint.vx = goal.vx; setpoint.vy = goal.vy; setpoint.vz = goal.vz;
            setpoint.yaw = NAN; setpoint.yawspeed = goal.yawRate;
        } else {
            setpoint.x = goal.x; setpoint.y = goal.y; setpoint.z = goal.z;
            setpoint.vx = NAN; setpoint.vy = NAN; setpoint.vz = NAN;
            setpoint.yaw = goal.yaw; setpoint.yawspeed = NAN;
        }
        m_mavlink.UpdateStreamSetpoint(setpoint);
        return true;
    }

private:
    bool EnsureSetpointStream()
    {
        bool expected = false;
        if (m_streamStarted.compare_exchange_strong(expected, true, std::memory_order_relaxed)) {
            m_mavlink.StartSetpointStreamHz(20.0);
        }
        return true;
    }
    MavlinkSerial& m_mavlink;
    std::atomic<bool> m_streamStarted{false};
};

MainRuntimeAliases BuildRuntimeAliases(const AppConfig& c)
{
    MainRuntimeAliases a{};
    a.sensorMode = c.sensorMode;
    a.width = c.camera.width; a.height = c.camera.height; a.fps = c.camera.fps;
    a.aeDisable = c.camera.aeDisable; a.exposureUs = c.camera.exposureUs; a.gain = c.camera.gain;
    a.requestY8 = c.camera.requestY8; a.r16Norm = c.camera.r16Norm; a.pairMs = c.camera.pairMs;
    a.keepMs = c.camera.keepMs; a.udpEnable = c.udp.enable; a.udpIp = c.udp.ip; a.udpPort = c.udp.port;
    a.cmdPort = c.udp.cmdPort; a.udpJpegQ = c.udp.jpegQ; a.udpPayload = c.udp.payload;
    a.udpQueue = c.udp.queue; a.spiDev = c.imu.spiDev; a.spiSpeed = c.imu.spiSpeed;
    a.spiMode = c.imu.spiMode; a.spiBits = c.imu.spiBits; a.gpiochip = c.imu.gpiochip;
    a.drdyLine = c.imu.drdyLine; a.imuHz = c.imu.imuHz; a.accelFsG = c.imu.accelFsG;
    a.gyroFsDps = c.imu.gyroFsDps; a.imuStartReg = c.imu.imuStartReg; a.allowEmptyImu = c.runtime.allowEmptyImu;
    a.rtImu = c.imu.rtImu; a.rtPrio = c.imu.rtPrio;
    return a;
}

void InstallSignalHandlers()
{
    signal(SIGINT, SigIntHandler);
    signal(SIGTERM, SigIntHandler);
}

uint16_t ReadU16Le(const uint8_t* p)
{
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t ReadU32Le(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

float ReadF32Le(const uint8_t* p)
{
    const uint32_t raw = ReadU32Le(p);
    float out = 0.0f;
    std::memcpy(&out, &raw, sizeof(out));
    return out;
}

uint32_t MonoTimeMs32()
{
    return static_cast<uint32_t>((MonoTimeUs() / 1000ULL) & 0xFFFFFFFFu);
}

std::vector<cv::Point2f> ExtractTrackedLeftFeatures(const std::vector<ORB_SLAM3::MapPoint*>& mapPoints,
                                                    const std::vector<cv::KeyPoint>& keyPoints,
                                                    int imageWidth,
                                                    int imageHeight)
{
    std::vector<cv::Point2f> out;
    const size_t count = std::min(mapPoints.size(), keyPoints.size());
    out.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        if (!mapPoints[i]) {
            continue;
        }
        const cv::Point2f& pt = keyPoints[i].pt;
        if (pt.x < 0.0f || pt.y < 0.0f || pt.x >= static_cast<float>(imageWidth) || pt.y >= static_cast<float>(imageHeight)) {
            continue;
        }
        out.push_back(pt);
    }
    return out;
}

std::string PeerToIpString(const UdpPeer& peer)
{
    if (!peer.valid) {
        return {};
    }
    char ipText[INET_ADDRSTRLEN] = {};
    const void* src = &(peer.addr.sin_addr);
    if (::inet_ntop(AF_INET, src, ipText, sizeof(ipText)) == nullptr) {
        return {};
    }
    return std::string(ipText);
}

void PrintStartupConfig(const AppConfig& app, const MainRuntimeAliases& a, UnifiedMode mode)
{
    std::cerr << "mode="
              << (mode == UnifiedMode::Slam ? "slam" : mode == UnifiedMode::Calib ? "calib" : "idle")
              << "\n";
    std::cerr << "cam " << a.width << "x" << a.height << " @" << a.fps
              << " aeDisable=" << (a.aeDisable ? "true" : "false")
              << " exp_us=" << a.exposureUs << " gain=" << a.gain << " pixelFormat=R16\n";
    std::cerr << "pair_thresh=" << a.pairMs << "ms keep_window=" << a.keepMs << "ms\n";
    std::cerr << "imuHz=" << a.imuHz << " udp=" << (a.udpEnable ? "Y" : "N")
              << " udpPort=" << a.udpPort << " cmdPort=" << a.cmdPort << "\n";
    std::cerr << "vocab=" << app.vocab << "\nsettings=" << app.settings << "\n";
}

void EnsureDir(const fs::path& p)
{
    std::error_code ec;
    fs::create_directories(p, ec);
}

std::string TsToName(int64_t tNs) { return std::to_string(tNs) + ".png"; }

void SetupFileBuffer(FILE* f, size_t bytes)
{
    if (f) setvbuf(f, nullptr, _IOFBF, bytes);
}

bool TryParseCalibIndex(const std::string& name, int& indexOut)
{
    static const std::string prefix = "calib_data_";
    if (name.rfind(prefix, 0) != 0) {
        return false;
    }
    const std::string suffix = name.substr(prefix.size());
    if (suffix.empty()) {
        return false;
    }
    for (char c : suffix) {
        if (c < '0' || c > '9') {
            return false;
        }
    }
    try {
        indexOut = std::stoi(suffix);
        return indexOut >= 0;
    } catch (...) {
        return false;
    }
}

std::string MakeCalibSessionDir(const std::string& root)
{
    EnsureDir(fs::path(root));
    int maxIndex = -1;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(root, ec)) {
        if (ec || !entry.is_directory()) {
            continue;
        }
        int index = -1;
        if (TryParseCalibIndex(entry.path().filename().string(), index)) {
            maxIndex = std::max(maxIndex, index);
        }
    }
    return (fs::path(root) / ("calib_data_" + std::to_string(maxIndex + 1))).string();
}

int CleanupCalibDataDirs(const std::string& root)
{
    int removed = 0;
    std::error_code ec;
    if (!fs::exists(root, ec)) {
        return 0;
    }
    for (const auto& entry : fs::directory_iterator(root, ec)) {
        if (ec || !entry.is_directory()) {
            continue;
        }
        int index = -1;
        if (!TryParseCalibIndex(entry.path().filename().string(), index)) {
            continue;
        }
        std::error_code rmEc;
        const auto n = fs::remove_all(entry.path(), rmEc);
        if (!rmEc && n > 0) {
            removed++;
        }
    }
    return removed;
}

std::thread StartImuThread(const MainRuntimeAliases& a, ImuThreadState& s, std::atomic<bool>& stop)
{
    ImuScale init{};
    s.accelLsbPerG.store(init.accelLsbPerG);
    s.gyroLsbPerDps.store(init.gyroLsbPerDps);
    return std::thread([&a, &s, &stop]() {
        if (a.rtImu) SetThreadRealtime(a.rtPrio);
        SpiDev spi(a.spiDev);
        if (!spi.Open(a.spiSpeed, a.spiMode, a.spiBits)) return;
        ImuScale scale{};
        if (!IcmResetAndConfig(spi, a.imuHz, a.accelFsG, a.gyroFsDps, scale)) return;
        s.accelLsbPerG.store(scale.accelLsbPerG);
        s.gyroLsbPerDps.store(scale.gyroLsbPerDps);
        DrdyGpio drdy;
        if (!drdy.Open(a.gpiochip, a.drdyLine)) return;
        s.imuOk.store(true);
        uint8_t raw12[12]{}; uint8_t st = 0; spi.ReadReg(REG_INT_STATUS, st);
        while (g_runningFlag.load() && !stop.load()) {
            int64_t tNs = 0;
            if (!drdy.WaitTs(1000, tNs)) continue;
            ImuSample sample{}; sample.tNs = tNs;
            spi.ReadReg(REG_INT_STATUS, st);
            if (!spi.ReadRegs(a.imuStartReg, raw12, sizeof(raw12))) {
                s.imuDrop.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            ImuScale cur{}; cur.accelLsbPerG = s.accelLsbPerG.load(); cur.gyroLsbPerDps = s.gyroLsbPerDps.load();
            ConvertRaw12AccelGyroToSi(raw12, cur, sample);
            s.imuBuffer.Push(sample);
            s.imuCnt.fetch_add(1, std::memory_order_relaxed);
        }
    });
}

std::thread StartCalibImuWriterThread(const MainRuntimeAliases& a, FILE* fImu, std::atomic<bool>& imuOk, std::atomic<bool>& stop)
{
    return std::thread([&a, fImu, &imuOk, &stop]() {
        SpiDev spi(a.spiDev);
        if (!spi.Open(a.spiSpeed, a.spiMode, a.spiBits)) return;
        ImuScale scale{};
        if (!IcmResetAndConfig(spi, a.imuHz, a.accelFsG, a.gyroFsDps, scale)) return;
        DrdyGpio drdy;
        if (!drdy.Open(a.gpiochip, a.drdyLine)) return;
        imuOk.store(true);
        uint8_t raw12[12]{}; uint8_t st = 0; spi.ReadReg(REG_INT_STATUS, st);
        int lines = 0;
        while (g_runningFlag.load() && !stop.load()) {
            int64_t tNs = 0;
            if (!drdy.WaitTs(1000, tNs)) continue;
            ImuSample sample{}; sample.tNs = tNs;
            spi.ReadReg(REG_INT_STATUS, st);
            if (!spi.ReadRegs(a.imuStartReg, raw12, sizeof(raw12))) continue;
            ConvertRaw12AccelGyroToSi(raw12, scale, sample);
            std::fprintf(fImu, "%lld,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                (long long)sample.tNs, (double)sample.gx, (double)sample.gy, (double)sample.gz,
                (double)sample.ax, (double)sample.ay, (double)sample.az);
            if ((++lines % 800) == 0) std::fflush(fImu);
        }
        std::fflush(fImu);
    });
}

bool OpenCamera(LibcameraStereoOV9281_TsPair& cam, const MainRuntimeAliases& a)
{
    return cam.Open(a.width, a.height, a.fps, a.aeDisable, a.exposureUs, a.gain, a.requestY8,
                    (int64_t)a.pairMs * 1000000LL, (int64_t)a.keepMs * 1000000LL, 8, a.r16Norm);
}

bool RunSlamSession(const UnifiedConfig& cfg, MavlinkSerial& mav, std::atomic<bool>& stop, LivePoseState& livePose)
{
    const MainRuntimeAliases a = BuildRuntimeAliases(cfg.app);
    PrintStartupConfig(cfg.app, a, UnifiedMode::Slam);
    livePose.SetRuntimeMode(RUNTIME_MODE_SLAM);
    Logger::Init("./stereo_vslam.log", 32 * 1024 * 1024, Logger::INFO, true);
    ORB_SLAM3::System SLAM(cfg.app.vocab, cfg.app.settings,
                           a.sensorMode == SensorMode::StereoImu ? ORB_SLAM3::System::IMU_STEREO
                                                                 : ORB_SLAM3::System::STEREO,
                           false);
    UdpImageSender udp;
    if (a.udpEnable) udp.Open(a.udpIp, a.udpPort, a.udpJpegQ, a.udpPayload, a.udpQueue);
    ImuThreadState imuState;
    std::thread imuThread;
    const bool useImu = (a.sensorMode == SensorMode::StereoImu);
    if (useImu) imuThread = StartImuThread(a, imuState, stop);
    LibcameraStereoOV9281_TsPair cam;
    if (!OpenCamera(cam, a)) {
        stop.store(true);
        if (imuThread.joinable()) imuThread.join();
        SLAM.Shutdown();
        return false;
    }
    int64_t lastFrameNs = 0;
    const int64_t frameStepNs = 1000000000LL / std::max(1, a.fps);
    const int64_t imuDtNs = 1000000000LL / std::max(1, a.imuHz);
    const int64_t slackBeforeNs = std::max<int64_t>(2 * imuDtNs, 5000000);
    const int64_t slackAfterNs = std::max<int64_t>(2 * imuDtNs, 5000000);
    while (g_runningFlag.load() && !stop.load()) {
        FrameItem L, R;
        if (!cam.GrabPair(L, R, 1000)) continue;
        int64_t frameNs = (int64_t)((L.tsNs + R.tsNs) / 2);
        if (lastFrameNs != 0 && frameNs <= lastFrameNs) frameNs = lastFrameNs + frameStepNs;
        std::vector<ORB_SLAM3::IMU::Point> vImu;
        if (useImu && lastFrameNs != 0) {
            vImu = imuState.imuBuffer.PopBetweenNs(lastFrameNs, frameNs, slackBeforeNs, slackAfterNs);
            if (vImu.empty() && !a.allowEmptyImu) continue;
        }
        lastFrameNs = frameNs;
        const double frameTime = (double)frameNs * 1e-9;
        Sophus::SE3f Tcw = useImu ? SLAM.TrackStereo(L.gray, R.gray, frameTime, vImu)
                                  : SLAM.TrackStereo(L.gray, R.gray, frameTime);
        std::vector<cv::Point2f> trackedLeftFeatures;
        if (a.udpEnable) {
            trackedLeftFeatures = ExtractTrackedLeftFeatures(
                SLAM.GetTrackedMapPoints(),
                SLAM.GetTrackedKeyPointsUn(),
                L.gray.cols,
                L.gray.rows);
            udp.Enqueue(0, L.seq, frameTime, L.gray, trackedLeftFeatures);
            udp.Enqueue(1, R.seq, frameTime, R.gray);
        }
        const Sophus::SE3f Twc = Tcw.inverse();
        const Eigen::Vector3f t = Twc.translation();
        const Eigen::Quaternionf q(Twc.so3().unit_quaternion());
        MavlinkSerial::Pose p{};
        p.x = t.x(); p.y = t.y(); p.z = t.z();
        p.qw = q.w(); p.qx = q.x(); p.qy = q.y(); p.qz = q.z();
        MavlinkSerial::NormalizeQuat(p.qw, p.qx, p.qy, p.qz);
        const int state = SLAM.GetTrackingState();
        livePose.UpdatePose(RUNTIME_MODE_SLAM, static_cast<uint8_t>(state), p);
        mav.SendOdometry(MonoTimeUs(), p, MAV_FRAME_LOCAL_NED, MAV_FRAME_BODY_FRD,
                         state == ORB_SLAM3::Tracking::OK ? OdomQualityMode::GOOD : OdomQualityMode::LOST);
    }
    cam.Close();
    if (imuThread.joinable()) imuThread.join();
    mav.StopSetpointStream();
    SLAM.Shutdown();
    livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
    return true;
}

bool RunCalibSession(const UnifiedConfig& cfg, std::atomic<bool>& stop, LivePoseState& livePose)
{
    const MainRuntimeAliases a = BuildRuntimeAliases(cfg.app);
    PrintStartupConfig(cfg.app, a, UnifiedMode::Calib);
    livePose.SetRuntimeMode(RUNTIME_MODE_CALIB);
    const std::string outRoot = MakeCalibSessionDir(cfg.calib.root);
    const fs::path root(outRoot), cam0Dir = root / "cam0", cam1Dir = root / "cam1";
    EnsureDir(cam0Dir); EnsureDir(cam1Dir);
    FILE* fCam0 = std::fopen((cam0Dir / "data.csv").string().c_str(), "w");
    FILE* fCam1 = std::fopen((cam1Dir / "data.csv").string().c_str(), "w");
    FILE* fImu = std::fopen((root / "imu.csv").string().c_str(), "w");
    if (!fCam0 || !fCam1 || !fImu) return false;
    std::cerr << "[calib] out=" << outRoot << "\n";
    SetupFileBuffer(fCam0, 1 << 20); SetupFileBuffer(fCam1, 1 << 20); SetupFileBuffer(fImu, 4 << 20);
    std::fprintf(fCam0, "#timestamp [ns],filename\n");
    std::fprintf(fCam1, "#timestamp [ns],filename\n");
    std::fprintf(fImu, "#timestamp [ns],wX [rad/s],wY [rad/s],wZ [rad/s],aX [m/s^2],aY [m/s^2],aZ [m/s^2]\n");
    UdpImageSender udp;
    if (a.udpEnable) udp.Open(a.udpIp, a.udpPort, a.udpJpegQ, a.udpPayload, a.udpQueue);
    std::atomic<bool> imuOk{false};
    std::thread imuThread = StartCalibImuWriterThread(a, fImu, imuOk, stop);
    LibcameraStereoOV9281_TsPair cam;
    if (!OpenCamera(cam, a)) {
        stop.store(true);
        if (imuThread.joinable()) imuThread.join();
        std::fclose(fCam0); std::fclose(fCam1); std::fclose(fImu);
        return false;
    }
    int saved = 0; int64_t lastPairNs = 0;
    const int64_t maxSaveDtNs = (int64_t)std::max(a.pairMs, 1) * 1000000LL;
    int64_t lastDiagNs = NowNs();
    while (g_runningFlag.load() && !stop.load()) {
        if (cfg.calib.maxFrames > 0 && saved >= cfg.calib.maxFrames) break;
        FrameItem L, R;
        if (!cam.GrabPair(L, R, 1000)) {
            const int64_t nowNs = NowNs();
            if (nowNs - lastDiagNs >= 1000000000LL) {
                lastDiagNs = nowNs;
                std::cerr << "[calib-wd] no pair 1s"
                          << " last_seq=" << cam.LastSeq()
                          << " dtMs=" << cam.LastDtMs()
                          << " pendL=" << cam.PendL()
                          << " pendR=" << cam.PendR()
                          << " pairTolMs=" << a.pairMs
                          << " pairTolNs=" << cam.PairTolNs()
                          << "\n";
            }
            continue;
        }
        const int64_t absDtLr = Abs64((int64_t)L.tsNs - (int64_t)R.tsNs);
        if ((saved % 30) == 0) {
            std::cerr << "[calib-pair] dt_lr_us=" << (absDtLr / 1000.0)
                      << " max_save_dt_us=" << (maxSaveDtNs / 1000.0)
                      << " seqL=" << L.seq
                      << " seqR=" << R.seq
                      << "\n";
        }
        if (absDtLr > maxSaveDtNs) {
            static int droppedWide = 0;
            ++droppedWide;
            if ((droppedWide % 10) == 1) {
                std::cerr << "[calib-drop] dt_lr_us=" << (absDtLr / 1000.0)
                          << " exceeds max_save_dt_us=" << (maxSaveDtNs / 1000.0)
                          << " dropped=" << droppedWide
                          << "\n";
            }
            continue;
        }
        int64_t pairNs = (int64_t)((L.tsNs + R.tsNs) / 2);
        if (lastPairNs != 0 && pairNs <= lastPairNs) pairNs = lastPairNs + 1;
        lastPairNs = pairNs;
        const std::string name = TsToName(pairNs);
        const fs::path fnL = cam0Dir / name, fnR = cam1Dir / name;
        if (L.gray.empty() || R.gray.empty()) {
            std::cerr << "[calib-write] empty image"
                      << " seqL=" << L.seq
                      << " seqR=" << R.seq
                      << " rowsL=" << L.gray.rows
                      << " colsL=" << L.gray.cols
                      << " rowsR=" << R.gray.rows
                      << " colsR=" << R.gray.cols
                      << "\n";
            continue;
        }
        const bool okL = cv::imwrite(fnL.string(), L.gray);
        const bool okR = cv::imwrite(fnR.string(), R.gray);
        if (!okL || !okR) {
            std::cerr << "[calib-write] imwrite failed"
                      << " okL=" << (okL ? "true" : "false")
                      << " okR=" << (okR ? "true" : "false")
                      << " pathL=" << fnL.string()
                      << " pathR=" << fnR.string()
                      << " typeL=" << L.gray.type()
                      << " typeR=" << R.gray.type()
                      << " rowsL=" << L.gray.rows
                      << " colsL=" << L.gray.cols
                      << " rowsR=" << R.gray.rows
                      << " colsR=" << R.gray.cols
                      << "\n";
            continue;
        }
        std::fprintf(fCam0, "%lld,%s\n", (long long)pairNs, name.c_str());
        std::fprintf(fCam1, "%lld,%s\n", (long long)pairNs, name.c_str());
        if (a.udpEnable) {
            udp.Enqueue(0, L.seq, pairNs * 1e-9, L.gray);
            udp.Enqueue(1, R.seq, pairNs * 1e-9, R.gray);
        }
        if ((saved % 30) == 0) {
            std::cerr << "[calib-save] saved=" << (saved + 1)
                      << " pathL=" << fnL.string()
                      << " pathR=" << fnR.string()
                      << "\n";
        }
        if ((++saved % 50) == 0) {
            std::fflush(fCam0); std::fflush(fCam1);
        }
    }
    cam.Close();
    stop.store(true);
    if (imuThread.joinable()) imuThread.join();
    std::fflush(fCam0); std::fflush(fCam1); std::fflush(fImu);
    std::fclose(fCam0); std::fclose(fCam1); std::fclose(fImu);
    std::cerr << "[calib] out=" << outRoot << " saved=" << saved << " imuOk=" << (imuOk.load() ? "true" : "false") << "\n";
    livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
    return true;
}

class UnifiedRuntimeController {
public:
    UnifiedRuntimeController(UnifiedConfig initialConfig, MavlinkSerial& mav, LivePoseState& livePose)
        : m_config(std::move(initialConfig)), m_mav(mav), m_livePose(livePose) {}
    void Start() { m_worker = std::thread([this]() { Loop(); }); }
    void Stop()
    {
        {
            std::lock_guard<std::mutex> lock(m_mu);
            m_stopping = true;
            m_desiredMode = UnifiedMode::Idle;
            m_sessionStop.store(true);
        }
        m_cv.notify_all();
        if (m_worker.joinable()) m_worker.join();
        JoinSession();
    }
    bool SetMode(UnifiedMode mode, std::string* err)
    {
        std::lock_guard<std::mutex> lock(m_mu);
        if (m_stopping) {
            if (err) *err = "runtime stopping";
            return false;
        }
        m_desiredMode = mode;
        m_livePose.SetRuntimeMode(static_cast<uint8_t>(mode));
        m_restartRequested = true;
        m_cv.notify_all();
        return true;
    }
    bool UpdateRemoteConfig(const RemoteRuntimeConfig& r, std::string* err)
    {
        if (r.exposureUs <= 0 || !(r.gain > 0.0f)) {
            if (err) *err = "bad runtime config";
            return false;
        }
        std::lock_guard<std::mutex> lock(m_mu);
        CameraConfig& cam = m_config.app.camera;
        cam.exposureUs = r.exposureUs;
        cam.gain = r.gain;
        m_config.app.udp.ip = r.udpIp;
        m_config.app.udp.enable = !r.udpIp.empty();
        if (m_desiredMode != UnifiedMode::Idle) {
            m_restartRequested = true;
            m_sessionStop.store(true);
        }
        m_cv.notify_all();
        return true;
    }
    bool CleanupCalibData(std::string* msg)
    {
        std::lock_guard<std::mutex> lock(m_mu);
        if (m_activeMode != UnifiedMode::Idle || m_desiredMode != UnifiedMode::Idle || m_session.joinable()) {
            if (msg) *msg = "runtime busy";
            return false;
        }
        const int removed = CleanupCalibDataDirs(m_config.calib.root);
        if (msg) *msg = "calib clean removed=" + std::to_string(removed);
        return true;
    }

private:
    void JoinSession() { if (m_session.joinable()) m_session.join(); }
    void Loop()
    {
        while (g_runningFlag.load()) {
            UnifiedMode startMode = UnifiedMode::Idle;
            UnifiedConfig cfg{};
            bool startSession = false, needJoin = false;
            {
                std::unique_lock<std::mutex> lock(m_mu);
                m_cv.wait_for(lock, std::chrono::milliseconds(100), [this]() {
                    return m_stopping || m_restartRequested || m_sessionDone;
                });
                if (m_sessionDone) {
                    needJoin = true;
                }
                if (m_stopping) m_sessionStop.store(true);
                if ((m_restartRequested || m_desiredMode != m_activeMode) && m_activeMode != UnifiedMode::Idle) {
                    m_sessionStop.store(true);
                    needJoin = true;
                }
            }
            if (needJoin) JoinSession();
            {
                std::lock_guard<std::mutex> lock(m_mu);
                if (needJoin) {
                    m_sessionDone = false;
                    m_activeMode = UnifiedMode::Idle;
                }
                if (m_stopping) break;
                if (m_activeMode != UnifiedMode::Idle && m_desiredMode == m_activeMode && !m_restartRequested) continue;
                m_activeMode = UnifiedMode::Idle;
                if (m_desiredMode != UnifiedMode::Idle) {
                    cfg = m_config;
                    startMode = m_desiredMode;
                    m_activeMode = startMode;
                    m_sessionStop.store(false);
                    startSession = true;
                }
                m_restartRequested = false;
            }
            if (startSession) {
                m_session = std::thread([this, cfg, startMode]() mutable {
                    if (startMode == UnifiedMode::Slam) RunSlamSession(cfg, m_mav, m_sessionStop, m_livePose);
                    else if (startMode == UnifiedMode::Calib) RunCalibSession(cfg, m_sessionStop, m_livePose);
                    std::lock_guard<std::mutex> lock(m_mu);
                    m_sessionDone = true;
                    m_cv.notify_all();
                });
            }
        }
    }

    std::mutex m_mu;
    std::condition_variable m_cv;
    UnifiedConfig m_config;
    MavlinkSerial& m_mav;
    LivePoseState& m_livePose;
    UnifiedMode m_desiredMode{UnifiedMode::Idle}, m_activeMode{UnifiedMode::Idle};
    bool m_restartRequested{false}, m_sessionDone{false}, m_stopping{false};
    std::atomic<bool> m_sessionStop{false};
    std::thread m_worker, m_session;
};

RouteResult HandleRuntimeModeFrame(const TlvFrame& frame, UnifiedRuntimeController& c)
{
    if (frame.len != RUNTIME_MODE_PAYLOAD_LEN) return {ACK_E_BAD_LEN, "bad runtime mode len"};
    UnifiedMode mode = UnifiedMode::Idle;
    if (frame.payload[0] == RUNTIME_MODE_SLAM) mode = UnifiedMode::Slam;
    else if (frame.payload[0] == RUNTIME_MODE_CALIB) mode = UnifiedMode::Calib;
    else if (frame.payload[0] != RUNTIME_MODE_IDLE) return {ACK_E_BAD_ARGS, "bad runtime mode"};
    std::string err;
    if (!c.SetMode(mode, &err)) return {ACK_E_BAD_STATE, err.empty() ? "set mode failed" : err};
    return {ACK_OK, mode == UnifiedMode::Idle ? "runtime -> idle" : mode == UnifiedMode::Slam ? "runtime -> slam" : "runtime -> calib"};
}

RouteResult HandleRuntimeConfigFrame(const TlvFrame& frame, const UdpPeer& peer, UnifiedRuntimeController& c)
{
    if (frame.len != RUNTIME_CONFIG_PAYLOAD_LEN) return {ACK_E_BAD_LEN, "bad runtime cfg len"};
    const uint8_t* p = frame.payload.data();
    RemoteRuntimeConfig r{};
    r.exposureUs = (int)ReadU32Le(&p[0]);
    r.gain = ReadF32Le(&p[4]);
    const char* ipChars = reinterpret_cast<const char*>(&p[8]);
    size_t ipLen = 0;
    while (ipLen < 32 && ipChars[ipLen] != '\0') {
        ++ipLen;
    }
    r.udpIp.assign(ipChars, ipLen);
    const std::string peerIp = PeerToIpString(peer);
    if (!peerIp.empty()) {
        r.udpIp = peerIp;
    }
    std::string err;
    if (!c.UpdateRemoteConfig(r, &err)) return {ACK_E_BAD_ARGS, err.empty() ? "runtime cfg failed" : err};
    return {ACK_OK, "runtime cfg updated udp=" + r.udpIp};
}

RouteResult HandleCalibCleanFrame(const TlvFrame& frame, UnifiedRuntimeController& c)
{
    if (frame.len != 0) {
        return {ACK_E_BAD_LEN, "bad calib clean len"};
    }
    std::string msg;
    if (!c.CleanupCalibData(&msg)) {
        return {ACK_E_BAD_STATE, msg.empty() ? "calib clean failed" : msg};
    }
    return {ACK_OK, msg};
}

std::thread StartUdpCommandThread(int port, Px4UdpHooks& hooks, UnifiedRuntimeController& controller, LivePoseState& livePose)
{
    return std::thread([port, &hooks, &controller, &livePose]() {
        UdpServer server;
        if (!server.Open((uint16_t)port)) {
            std::cerr << "[udp_cmd] open failed on 0.0.0.0:" << port << "\n";
            return;
        }
        TlvCmdRouter router(hooks);
        router.RegisterDefaults();
        TlvParser parser;
        uint8_t rx[2048]{};
        auto lastStateTx = std::chrono::steady_clock::now();
        while (g_runningFlag.load()) {
            UdpPeer peer{};
            const int n = server.Recv(rx, sizeof(rx), &peer);
            if (n <= 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            } else {
                livePose.UpdatePeer(peer);
                parser.Push(rx, (size_t)n);
                while (auto frame = parser.TryPop()) {
                    RouteResult rr{};
                    if (frame->cmd == CMD_RUNTIME_MODE) rr = HandleRuntimeModeFrame(*frame, controller);
                else if (frame->cmd == CMD_RUNTIME_CONFIG) rr = HandleRuntimeConfigFrame(*frame, peer, controller);
                    else if (frame->cmd == CMD_CALIB_CLEAN) rr = HandleCalibCleanFrame(*frame, controller);
                    else rr = router.Handle(*frame);
                    std::vector<uint8_t> ack = MakeAckFrame(frame->seq, frame->tMs, frame->cmd, frame->seq, rr.status);
                    server.SendTo(peer, ack.data(), ack.size());
                    if (!rr.msg.empty()) std::cerr << "[udp_cmd] " << rr.msg << "\n";
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - lastStateTx >= std::chrono::milliseconds(100)) {
                lastStateTx = now;
                LivePoseState::Snapshot snap{};
                if (livePose.ConsumeSnapshot(snap) && snap.hasPeer) {
                    std::vector<uint8_t> payload;
                    payload.reserve(STATE_POSE_PAYLOAD_LEN);
                    payload.push_back(snap.runtimeMode);
                    payload.push_back(snap.trackingState);
                    WriteU16Le(payload, 0);
                    WriteF32Le(payload, snap.x);
                    WriteF32Le(payload, snap.y);
                    WriteF32Le(payload, snap.z);
                    WriteF32Le(payload, snap.qw);
                    WriteF32Le(payload, snap.qx);
                    WriteF32Le(payload, snap.qy);
                    WriteF32Le(payload, snap.qz);
                    std::vector<uint8_t> stateFrame =
                        MakeFrame(TLV_VER, CMD_STATE, 0, snap.seq, MonoTimeMs32(),
                                  payload.data(), static_cast<uint16_t>(payload.size()));
                    server.SendTo(snap.peer, stateFrame.data(), stateFrame.size());
                }
            }
        }
    });
}

}  // namespace

int main(int argc, char** argv)
{
    InstallSignalHandlers();
    UnifiedConfig cfg{};
    cfg.app = ParseAppConfig(argc, argv);
    ArgReader args(argc, argv);
    cfg.calib.root = args.GetString("--calib-root", "./calib_runs");
    cfg.calib.maxFrames = args.GetInt("--calib-max-frames", 0);
    std::string autoModeText = args.GetString("--auto-mode", "idle");
    std::transform(autoModeText.begin(), autoModeText.end(), autoModeText.begin(), [](unsigned char c) {
        return (char)std::tolower(c);
    });
    UnifiedMode autoMode = UnifiedMode::Idle;
    if (autoModeText == "slam") autoMode = UnifiedMode::Slam;
    else if (autoModeText == "calib") autoMode = UnifiedMode::Calib;

    const MainRuntimeAliases aliases = BuildRuntimeAliases(cfg.app);
    PrintStartupConfig(cfg.app, aliases, UnifiedMode::Idle);

    MavlinkSerial mav("/dev/ttyAMA0", 921600);
    mav.StartRx();
    Px4UdpHooks hooks(mav);
    LivePoseState livePose;
    UnifiedRuntimeController controller(cfg, mav, livePose);
    controller.Start();
    if (autoMode != UnifiedMode::Idle) controller.SetMode(autoMode, nullptr);
    std::thread udpCmdThread = StartUdpCommandThread(aliases.cmdPort, hooks, controller, livePose);
    while (g_runningFlag.load()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    controller.Stop();
    mav.StopSetpointStream();
    mav.StopRx();
    if (udpCmdThread.joinable()) udpCmdThread.join();
    return 0;
}
