#include "core/application/session/calib_session_graph_service.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "adapters/imu/icm42688/icm42688_imu.h"
#include "adapters/stream/udp_image_sender.h"
#include "common/epg/epg.h"
#include "common/tlv/tlv_protocol.h"
#include "core/application/session/calib_session_service.h"
#include "core/application/session/calib_storage_helpers.h"
#include "core/application/session/epg_messages.h"
#include "core/application/session/epg_registry.h"
#include "core/application/session/runtime_session_common.h"
#include "core/application/session/sensor_runtime_helpers.h"
#include "core/ports/camera_provider.h"
#include "platform/linux/gpio/drdy_gpio.h"

namespace smartdrone::core::application {
namespace {

constexpr int kRecommendedMaxCalibSaveFps = 30;
constexpr const char *kCalibEpgDfxSnapshotPath = "/tmp/smartdrone_epg_calib.json";

std::uint64_t EpgDfxNowMs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

void WriteEpgDfxSnapshotFile(const std::string &path, const std::string &json)
{
    const std::string tmpPath = path + ".tmp";
    {
        std::ofstream output(tmpPath, std::ios::out | std::ios::trunc);
        if (!output) {
            return;
        }
        output << json;
    }
    (void)std::rename(tmpPath.c_str(), path.c_str());
}

int ClampCalibSaveFps(int requestedFps, int cameraFps)
{
    const int baseFps = ClampSlamInputFps(requestedFps, cameraFps);
    return std::clamp(baseFps, 1, std::max(1, std::min(cameraFps, kRecommendedMaxCalibSaveFps)));
}

cv::Mat EnsureGray8(const cv::Mat &src, bool &convertedOut)
{
    convertedOut = false;
    if (src.empty()) {
        return src;
    }
    if (src.type() == CV_8UC1) {
        return src;
    }
    cv::Mat out;
    if (src.type() == CV_16UC1) {
        src.convertTo(out, CV_8U, 1.0 / 256.0);
    } else {
        src.convertTo(out, CV_8U);
    }
    convertedOut = true;
    return out;
}

class CalibRuntimeState final {
  public:
    CalibRuntimeState(const UnifiedConfig &cfg, std::atomic<bool> &stop, LivePoseState &livePose,
                            std::atomic<bool> &runningFlag)
        : m_cfg(cfg), m_stop(stop), m_livePose(livePose), m_runningFlag(runningFlag)
    {
    }

    ~CalibRuntimeState() { Finalize(false); }

    bool EnsureStarted()
    {
        std::lock_guard<std::mutex> lock(m_mu);
        if (m_started) {
            return true;
        }
        if (m_startFailed) {
            return false;
        }

        m_aliases = BuildRuntimeAliases(m_cfg.app);
        PrintStartupConfig(m_cfg.app, m_aliases, ControllerMode::Calib);
        m_livePose.SetRuntimeMode(RUNTIME_MODE_CALIB);

        m_outRoot = MakeCalibSessionDir(m_cfg.calib.root);
        m_root = fs::path(m_outRoot);
        m_cam0Dir = m_root / "cam0";
        m_cam1Dir = m_root / "cam1";
        EnsureDir(m_cam0Dir);
        EnsureDir(m_cam1Dir);

        m_fCam0 = std::fopen((m_cam0Dir / "data.csv").string().c_str(), "w");
        m_fCam1 = std::fopen((m_cam1Dir / "data.csv").string().c_str(), "w");
        m_fImu = std::fopen((m_root / "imu.csv").string().c_str(), "w");
        if (!m_fCam0 || !m_fCam1 || !m_fImu) {
            m_startFailed = true;
            CloseFilesLocked();
            return false;
        }

        std::cerr << "[calib] out=" << m_outRoot << "\n";
        SetupFileBuffer(m_fCam0, 1 << 20);
        SetupFileBuffer(m_fCam1, 1 << 20);
        SetupFileBuffer(m_fImu, 4 << 20);
        std::fprintf(m_fCam0, "#timestamp [ns],filename\n");
        std::fprintf(m_fCam1, "#timestamp [ns],filename\n");
        std::fprintf(m_fImu, "#timestamp [ns],wX [rad/s],wY [rad/s],wZ [rad/s],aX [m/s^2],aY [m/s^2],aZ [m/s^2]\n");

        if (m_aliases.udpEnable && m_aliases.sendImage) {
            m_udp.Open(m_aliases.udpIp, m_aliases.udpPort, m_aliases.udpJpegQ, m_aliases.udpPayload,
                       m_aliases.udpQueue);
            m_udpOpen = true;
        }

        m_cameraProvider = CreateCameraProvider();
        m_cameraOpen = m_cameraProvider && m_cameraProvider->Open(m_aliases);
        if (!m_cameraOpen) {
            m_startFailed = true;
            m_stop.store(true);
            FinalizeLocked(false);
            return false;
        }

        m_maxSaveDtNs = static_cast<std::int64_t>(std::max(m_aliases.pairMs, 1)) * 1000000LL;
        m_calibSaveFps = ClampCalibSaveFps(m_aliases.slamInputFps, m_aliases.fps);
        m_calibSaveStepNs = 1000000000LL / std::max(1, m_calibSaveFps);
        std::cerr << "[calib] target_save_fps=" << m_calibSaveFps
                  << " configured_camera_fps=" << m_aliases.fps
                  << " requested_slam_fps=" << m_aliases.slamInputFps << "\n";

        m_started = true;
        return true;
    }

    smartdrone::core::ports::ICameraProvider *CameraProvider()
    {
        std::lock_guard<std::mutex> lock(m_mu);
        return m_cameraProvider.get();
    }

    const MainRuntimeAliases &Aliases() const { return m_aliases; }

    bool ShouldFinishCapture() const
    {
        return m_cfg.calib.maxFrames > 0 && m_saved.load(std::memory_order_relaxed) >= m_cfg.calib.maxFrames;
    }

    std::int64_t MaxSaveDtNs() const { return m_maxSaveDtNs; }
    int CalibSaveFps() const { return m_calibSaveFps; }
    std::int64_t CalibSaveStepNs() const { return m_calibSaveStepNs; }

    bool NextEligibleSave(std::int64_t pairNs, std::int64_t &adjustedPairNs, std::string &dropReason)
    {
        std::lock_guard<std::mutex> lock(m_paceMu);
        if (m_nextEligibleSaveNs != 0 && pairNs < m_nextEligibleSaveNs) {
            ++m_droppedByPacing;
            if ((m_droppedByPacing % 30) == 1) {
                std::cerr << "[calib-pace] dropped=" << m_droppedByPacing << " pair_ts_ns=" << pairNs
                          << " next_save_ts_ns=" << m_nextEligibleSaveNs
                          << " target_save_fps=" << m_calibSaveFps << "\n";
            }
            dropReason = "pacing";
            return false;
        }
        if (m_lastPairNs != 0 && pairNs <= m_lastPairNs) {
            pairNs = m_lastPairNs + 1;
        }
        m_lastPairNs = pairNs;
        m_nextEligibleSaveNs = pairNs + m_calibSaveStepNs;
        adjustedPairNs = pairNs;
        return true;
    }

    fs::path Cam0Path(const std::string &name) const { return m_cam0Dir / name; }
    fs::path Cam1Path(const std::string &name) const { return m_cam1Dir / name; }

    bool WriteSavePair(const CalibSavePair &pair)
    {
        if (!pair.frame) {
            return false;
        }
        const auto &L = pair.frame->stereo.left;
        const auto &R = pair.frame->stereo.right;
        if (L.gray.empty() || R.gray.empty()) {
            std::cerr << "[calib-write] empty image"
                      << " seqL=" << L.sequence << " seqR=" << R.sequence << " rowsL=" << L.gray.rows
                      << " colsL=" << L.gray.cols << " rowsR=" << R.gray.rows << " colsR=" << R.gray.cols << "\n";
            return false;
        }

        bool convertedL = false;
        bool convertedR = false;
        const cv::Mat calibGrayL = EnsureGray8(L.gray, convertedL);
        const cv::Mat calibGrayR = EnsureGray8(R.gray, convertedR);
        if (convertedL || convertedR) {
            ++m_conversionLogCount;
            if ((m_conversionLogCount % 30) == 1) {
                std::cerr << "[calib-gray] converted to 8-bit"
                          << " typeL=" << L.gray.type() << " typeR=" << R.gray.type()
                          << " count=" << m_conversionLogCount << "\n";
            }
        }

        const bool okL = cv::imwrite(pair.fnL.string(), calibGrayL);
        const bool okR = cv::imwrite(pair.fnR.string(), calibGrayR);
        if (!okL || !okR) {
            std::cerr << "[calib-write] imwrite failed"
                      << " okL=" << (okL ? "true" : "false") << " okR=" << (okR ? "true" : "false")
                      << " pathL=" << pair.fnL.string() << " pathR=" << pair.fnR.string()
                      << " typeL=" << L.gray.type() << " typeR=" << R.gray.type()
                      << " rowsL=" << L.gray.rows << " colsL=" << L.gray.cols
                      << " rowsR=" << R.gray.rows << " colsR=" << R.gray.cols << "\n";
            return false;
        }

        std::lock_guard<std::mutex> lock(m_fileMu);
        std::fprintf(m_fCam0, "%lld,%s\n", static_cast<long long>(pair.pairNs), pair.name.c_str());
        std::fprintf(m_fCam1, "%lld,%s\n", static_cast<long long>(pair.pairNs), pair.name.c_str());
        m_savedImagePaths.push_back(pair.fnL);
        m_savedImagePaths.push_back(pair.fnR);
        const int saved = m_saved.fetch_add(1, std::memory_order_relaxed) + 1;
        if (((saved - 1) % 30) == 0) {
            std::cerr << "[calib-save] saved=" << saved << " pathL=" << pair.fnL.string()
                      << " pathR=" << pair.fnR.string() << "\n";
        }
        if ((saved % 50) == 0) {
            std::fflush(m_fCam0);
            std::fflush(m_fCam1);
        }
        return true;
    }

    void EnqueuePreview(const CalibStereoFrame &frame)
    {
        if (!m_udpOpen || !m_aliases.sendImage) {
            return;
        }
        const auto &L = frame.stereo.left;
        const auto &R = frame.stereo.right;
        const std::int64_t pairNs = static_cast<std::int64_t>((L.timestampNs + R.timestampNs) / 2);
        bool convertedL = false;
        bool convertedR = false;
        const cv::Mat calibGrayL = EnsureGray8(L.gray, convertedL);
        const cv::Mat calibGrayR = EnsureGray8(R.gray, convertedR);
        m_udp.Enqueue(0, static_cast<std::uint64_t>(L.sequence), L.sequence, pairNs * 1e-9, calibGrayL, {}, true,
                      false);
        m_udp.Enqueue(1, static_cast<std::uint64_t>(R.sequence), R.sequence, pairNs * 1e-9, calibGrayR, {}, true,
                      false);
    }

    bool WriteImuSample(const ImuSample &sample)
    {
        std::lock_guard<std::mutex> lock(m_fileMu);
        if (!m_fImu) {
            return false;
        }
        std::fprintf(m_fImu, "%lld,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n", static_cast<long long>(sample.tNs),
                     static_cast<double>(sample.gx), static_cast<double>(sample.gy), static_cast<double>(sample.gz),
                     static_cast<double>(sample.ax), static_cast<double>(sample.ay), static_cast<double>(sample.az));
        if ((++m_imuLines % 800) == 0) {
            std::fflush(m_fImu);
        }
        m_imuOk.store(true, std::memory_order_relaxed);
        return true;
    }

    bool ImuOk() const { return m_imuOk.load(std::memory_order_relaxed); }
    int Saved() const { return m_saved.load(std::memory_order_relaxed); }

    void Finalize(bool sessionOk)
    {
        std::lock_guard<std::mutex> lock(m_mu);
        FinalizeLocked(sessionOk);
    }

  private:
    void FinalizeLocked(bool sessionOk)
    {
        if (m_finalized) {
            return;
        }
        m_finalized = true;
        m_stop.store(true);

        if (m_cameraProvider) {
            m_cameraProvider->Stop();
            if (m_cameraOpen) {
                std::cerr << "[session] calib camera closed\n";
            }
            m_cameraOpen = false;
        }
        if (m_udpOpen) {
            m_udp.Close();
            m_udpOpen = false;
            std::cerr << "[session] calib udp closed\n";
        }

        {
            std::lock_guard<std::mutex> fileLock(m_fileMu);
            std::cerr << "[calib-sync] flushing outputs on calib stop saved=" << m_saved.load() << "\n";
            FlushCalibOutputs(m_fCam0, m_fCam1, m_fImu, m_savedImagePaths, m_root, m_cam0Dir, m_cam1Dir);
            CloseFilesLocked();
        }

        std::cerr << "[calib] out=" << m_outRoot << " saved=" << m_saved.load()
                  << " imuOk=" << (m_imuOk.load() ? "true" : "false") << "\n";
        m_livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
        std::cerr << "[session] calib exit\n";
        m_sessionOk.store(sessionOk, std::memory_order_relaxed);
    }

    void CloseFilesLocked()
    {
        if (m_fCam0) {
            std::fclose(m_fCam0);
            m_fCam0 = nullptr;
        }
        if (m_fCam1) {
            std::fclose(m_fCam1);
            m_fCam1 = nullptr;
        }
        if (m_fImu) {
            std::fclose(m_fImu);
            m_fImu = nullptr;
        }
    }

    const UnifiedConfig &m_cfg;
    std::atomic<bool> &m_stop;
    LivePoseState &m_livePose;
    std::atomic<bool> &m_runningFlag;
    mutable std::mutex m_mu;
    std::mutex m_fileMu;
    std::mutex m_paceMu;
    MainRuntimeAliases m_aliases{};
    std::string m_outRoot;
    fs::path m_root;
    fs::path m_cam0Dir;
    fs::path m_cam1Dir;
    FILE *m_fCam0{nullptr};
    FILE *m_fCam1{nullptr};
    FILE *m_fImu{nullptr};
    UdpImageSender m_udp;
    std::unique_ptr<smartdrone::core::ports::ICameraProvider> m_cameraProvider;
    std::vector<fs::path> m_savedImagePaths;
    std::atomic<bool> m_sessionOk{true};
    std::atomic<bool> m_imuOk{false};
    std::atomic<int> m_saved{0};
    std::int64_t m_maxSaveDtNs{0};
    int m_calibSaveFps{1};
    std::int64_t m_calibSaveStepNs{1000000000LL};
    std::int64_t m_lastPairNs{0};
    std::int64_t m_nextEligibleSaveNs{0};
    int m_droppedByPacing{0};
    int m_conversionLogCount{0};
    int m_imuLines{0};
    bool m_started{false};
    bool m_startFailed{false};
    bool m_cameraOpen{false};
    bool m_udpOpen{false};
    bool m_finalized{false};
};

class CalibResourceTask final : public epg::ITask {
  public:
    CalibResourceTask(std::shared_ptr<CalibRuntimeState> state, std::atomic<bool> &stop,
                      std::atomic<bool> &runningFlag)
        : m_state(std::move(state)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (m_emitted || !m_runningFlag.load() || m_stop.load()) {
            return;
        }
        if (!m_state->EnsureStarted()) {
            m_stop.store(true);
            EmitDone(context, false);
            return;
        }
        auto ready = context.Make<CalibResourceReady>();
        ready->ready = true;
        const bool cameraOk = context.Push(0, ready);
        const bool imuOk = context.Push(1, std::move(ready));
        m_emitted = cameraOk || imuOk;
    }

  private:
    void EmitDone(epg::TaskContext &context, bool sessionOk)
    {
        auto done = context.Make<CalibCaptureDone>();
        done->sessionOk = sessionOk;
        context.Push(2, std::move(done));
    }

    std::shared_ptr<CalibRuntimeState> m_state;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    bool m_emitted{false};
};
EPG_REGISTER_TASK_TYPE(CalibResourceTask, "CalibResourceTask")

class CalibClockTask final : public epg::ITask {
  public:
    CalibClockTask(std::atomic<bool> &stop, std::atomic<bool> &runningFlag)
        : m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (!m_runningFlag.load() || m_stop.load()) {
            return;
        }
        auto tick = context.Make<CalibTick>();
        tick->sequence = ++m_sequence;
        context.Push(0, std::move(tick));
    }

  private:
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    std::uint64_t m_sequence{0};
};
EPG_REGISTER_TASK_TYPE(CalibClockTask, "CalibClockTask")

class CalibCameraAcquireTask final : public epg::ITask {
  public:
    CalibCameraAcquireTask(std::shared_ptr<CalibRuntimeState> state, std::atomic<bool> &stop,
                           std::atomic<bool> &runningFlag)
        : m_state(std::move(state)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (auto ready = context.TryPopLatest<CalibResourceReady>(0)) {
            m_ready = ready->ready;
        }
        const auto tick = context.TryPopLatest<CalibTick>(1);
        if (!m_ready || !tick || !m_runningFlag.load() || m_stop.load()) {
            return;
        }
        if (m_state->ShouldFinishCapture()) {
            EmitDone(context, true);
            return;
        }

        auto *camera = m_state->CameraProvider();
        if (!camera) {
            EmitDone(context, false);
            return;
        }

        auto frame = context.Make<CalibStereoFrame>();
        if (!camera->GrabStereo(frame->stereo, 1000, true)) {
            if (!camera->GetHealth().healthy) {
                std::cerr << "[calib] camera pipeline unhealthy, aborting session\n";
                EmitDone(context, false);
            }
            return;
        }

        const auto &L = frame->stereo.left;
        const auto &R = frame->stereo.right;
        if (camera->Semantics() == smartdrone::core::ports::CameraProviderSemantics::DualStreamPaired) {
            const std::int64_t absDtLr =
                std::llabs(static_cast<std::int64_t>(L.timestampNs) - static_cast<std::int64_t>(R.timestampNs));
            if (absDtLr > m_state->MaxSaveDtNs()) {
                ++m_droppedWide;
                if ((m_droppedWide % 10) == 1) {
                    std::cerr << "[calib-drop] dt_lr_us=" << (absDtLr / 1000.0)
                              << " exceeds max_save_dt_us=" << (m_state->MaxSaveDtNs() / 1000.0)
                              << " dropped=" << m_droppedWide << "\n";
                }
                return;
            }
        }

        context.Push(0, frame);
        context.Push(1, frame);
    }

  private:
    void EmitDone(epg::TaskContext &context, bool sessionOk)
    {
        if (m_doneEmitted) {
            return;
        }
        m_doneEmitted = true;
        auto done = context.Make<CalibCaptureDone>();
        done->sessionOk = sessionOk;
        context.Push(2, std::move(done));
    }

    std::shared_ptr<CalibRuntimeState> m_state;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    bool m_ready{false};
    bool m_doneEmitted{false};
    int m_droppedWide{0};
};
EPG_REGISTER_TASK_TYPE(CalibCameraAcquireTask, "CalibCameraAcquireTask")

class CalibPacingFilterTask final : public epg::ITask {
  public:
    explicit CalibPacingFilterTask(std::shared_ptr<CalibRuntimeState> state) : m_state(std::move(state)) {}

    void OnTick(epg::TaskContext &context) override
    {
        while (auto frame = context.TryPop<CalibStereoFrame>(0)) {
            const auto &L = frame->stereo.left;
            const auto &R = frame->stereo.right;
            std::int64_t pairNs = static_cast<std::int64_t>((L.timestampNs + R.timestampNs) / 2);
            std::int64_t adjustedPairNs = pairNs;
            std::string dropReason;
            if (!m_state->NextEligibleSave(pairNs, adjustedPairNs, dropReason)) {
                continue;
            }
            auto save = context.Make<CalibSavePair>();
            save->frame = std::move(frame);
            save->pairNs = adjustedPairNs;
            save->name = TsToName(adjustedPairNs);
            save->fnL = m_state->Cam0Path(save->name);
            save->fnR = m_state->Cam1Path(save->name);
            context.Push(0, std::move(save));
        }
    }

  private:
    std::shared_ptr<CalibRuntimeState> m_state;
};
EPG_REGISTER_TASK_TYPE(CalibPacingFilterTask, "CalibPacingFilterTask")

class CalibStorageWriteTask final : public epg::ITask {
  public:
    explicit CalibStorageWriteTask(std::shared_ptr<CalibRuntimeState> state) : m_state(std::move(state)) {}

    void OnTick(epg::TaskContext &context) override
    {
        while (auto save = context.TryPop<CalibSavePair>(0)) {
            auto status = context.Make<CalibStorageStatus>();
            status->ok = m_state->WriteSavePair(*save);
            context.Push(0, std::move(status));
        }
    }

  private:
    std::shared_ptr<CalibRuntimeState> m_state;
};
EPG_REGISTER_TASK_TYPE(CalibStorageWriteTask, "CalibStorageWriteTask")

class CalibUdpPreviewTask final : public epg::ITask {
  public:
    explicit CalibUdpPreviewTask(std::shared_ptr<CalibRuntimeState> state) : m_state(std::move(state)) {}

    void OnTick(epg::TaskContext &context) override
    {
        while (auto frame = context.TryPopLatest<CalibStereoFrame>(0)) {
            m_state->EnqueuePreview(*frame);
            auto status = context.Make<CalibPreviewStatus>();
            status->ok = true;
            context.Push(0, std::move(status));
        }
    }

  private:
    std::shared_ptr<CalibRuntimeState> m_state;
};
EPG_REGISTER_TASK_TYPE(CalibUdpPreviewTask, "CalibUdpPreviewTask")

class CalibImuWriterTask final : public epg::ITask {
  public:
    CalibImuWriterTask(std::shared_ptr<CalibRuntimeState> state, std::atomic<bool> &stop,
                       std::atomic<bool> &runningFlag)
        : m_state(std::move(state)), m_stop(stop), m_runningFlag(runningFlag)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (auto ready = context.TryPopLatest<CalibResourceReady>(0)) {
            m_ready = ready->ready;
        }
        if (!m_ready || !m_runningFlag.load() || m_stop.load()) {
            return;
        }
        if (!EnsureOpened()) {
            return;
        }

        std::int64_t tNs = 0;
        if (!m_drdy.WaitTs(1, tNs)) {
            return;
        }
        ImuSample sample{};
        sample.tNs = tNs;
        std::uint8_t status = 0;
        m_spi->ReadReg(REG_INT_STATUS, status);
        if (!m_spi->ReadRegs(m_state->Aliases().imuStartReg, m_raw12, sizeof(m_raw12))) {
            return;
        }
        ConvertRaw12AccelGyroToSi(m_raw12, m_scale, sample);
        const bool ok = m_state->WriteImuSample(sample);
        auto imuStatus = context.Make<CalibImuStatus>();
        imuStatus->ok = ok;
        context.Push(0, std::move(imuStatus));
    }

  private:
    bool EnsureOpened()
    {
        if (m_opened || m_openFailed) {
            return m_opened;
        }
        const auto &a = m_state->Aliases();
        m_spi.reset(new SpiDev(a.spiDev));
        if (!m_spi->Open(a.spiSpeed, a.spiMode, a.spiBits)) {
            m_openFailed = true;
            return false;
        }
        if (!IcmResetAndConfig(*m_spi, a.imuHz, a.accelFsG, a.gyroFsDps, m_scale)) {
            m_openFailed = true;
            return false;
        }
        if (!m_drdy.Open(a.gpiochip, a.drdyLine)) {
            m_openFailed = true;
            return false;
        }
        std::uint8_t status = 0;
        m_spi->ReadReg(REG_INT_STATUS, status);
        m_opened = true;
        return true;
    }

    std::shared_ptr<CalibRuntimeState> m_state;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    std::unique_ptr<SpiDev> m_spi;
    DrdyGpio m_drdy;
    ImuScale m_scale{};
    std::uint8_t m_raw12[12]{};
    bool m_ready{false};
    bool m_opened{false};
    bool m_openFailed{false};
};
EPG_REGISTER_TASK_TYPE(CalibImuWriterTask, "CalibImuWriterTask")

class CalibCompletionTask final : public epg::ITask {
  public:
    explicit CalibCompletionTask(std::shared_ptr<CalibRuntimeState> state) : m_state(std::move(state)) {}

    void OnTick(epg::TaskContext &context) override
    {
        while (auto status = context.TryPop<CalibStorageStatus>(1)) {
            if (!status->ok) {
                m_sessionOk = false;
            }
        }
        while (auto status = context.TryPop<CalibImuStatus>(2)) {
            m_seenImu = m_seenImu || status->ok;
        }
        while (context.TryPop<CalibPreviewStatus>(3)) {
        }
        while (auto done = context.TryPop<CalibCaptureDone>(0)) {
            m_sessionOk = m_sessionOk && done->sessionOk;
            EmitFlush(context);
        }
        if (!m_flushEmitted && m_state->ShouldFinishCapture()) {
            EmitFlush(context);
        }
    }

  private:
    void EmitFlush(epg::TaskContext &context)
    {
        if (m_flushEmitted) {
            return;
        }
        m_flushEmitted = true;
        auto flush = context.Make<CalibFlushRequest>();
        flush->sessionOk = m_sessionOk;
        context.Push(0, std::move(flush));
    }

    std::shared_ptr<CalibRuntimeState> m_state;
    bool m_sessionOk{true};
    bool m_seenImu{false};
    bool m_flushEmitted{false};
};
EPG_REGISTER_TASK_TYPE(CalibCompletionTask, "CalibCompletionTask")

class CalibFlushSyncTask final : public epg::ITask {
  public:
    CalibFlushSyncTask(std::shared_ptr<CalibRuntimeState> state, std::atomic<bool> &completed,
                       std::atomic<bool> &sessionOk)
        : m_state(std::move(state)), m_completed(completed), m_sessionOk(sessionOk)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        if (m_completed.load(std::memory_order_relaxed)) {
            return;
        }
        if (auto flush = context.TryPopLatest<CalibFlushRequest>(0)) {
            m_state->Finalize(flush->sessionOk);
            m_sessionOk.store(flush->sessionOk, std::memory_order_relaxed);
            auto status = context.Make<CalibStatus>();
            status->sessionOk = flush->sessionOk;
            status->completed = true;
            context.Push(0, std::move(status));
        }
    }

  private:
    std::shared_ptr<CalibRuntimeState> m_state;
    std::atomic<bool> &m_completed;
    std::atomic<bool> &m_sessionOk;
};
EPG_REGISTER_TASK_TYPE(CalibFlushSyncTask, "CalibFlushSyncTask")

class CalibMonitorTask final : public epg::ITask {
  public:
    CalibMonitorTask(std::atomic<bool> &sessionOk, std::atomic<bool> &completed)
        : m_sessionOk(sessionOk), m_completed(completed)
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        while (auto status = context.TryPop<CalibStatus>(0)) {
            m_sessionOk.store(status->sessionOk, std::memory_order_relaxed);
            if (status->completed) {
                m_completed.store(true, std::memory_order_relaxed);
            }
        }
    }

  private:
    std::atomic<bool> &m_sessionOk;
    std::atomic<bool> &m_completed;
};
EPG_REGISTER_TASK_TYPE(CalibMonitorTask, "CalibMonitorTask")

EpgTaskFactoryResolver MakeCalibGraphTaskFactoryResolver(
    const std::shared_ptr<CalibRuntimeState> &state,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag,
    std::atomic<bool> &sessionOk,
    std::atomic<bool> &completed)
{
    auto &catalog = epg::TypeCatalog::Global();
    return epg::TypeCatalog::MakeTaskFactoryResolver({
        catalog.MakeTaskFactoryEntry<CalibResourceTask>([state, &stop, &runningFlag]() {
                return std::unique_ptr<epg::ITask>(
                    new CalibResourceTask(state, stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<CalibClockTask>([&stop, &runningFlag]() {
                return std::unique_ptr<epg::ITask>(
                    new CalibClockTask(stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<CalibCameraAcquireTask>([state, &stop, &runningFlag]() {
                return std::unique_ptr<epg::ITask>(
                    new CalibCameraAcquireTask(state, stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<CalibPacingFilterTask>([state]() {
                return std::unique_ptr<epg::ITask>(
                    new CalibPacingFilterTask(state));
            }),
        catalog.MakeTaskFactoryEntry<CalibStorageWriteTask>([state]() {
                return std::unique_ptr<epg::ITask>(
                    new CalibStorageWriteTask(state));
            }),
        catalog.MakeTaskFactoryEntry<CalibImuWriterTask>([state, &stop, &runningFlag]() {
                return std::unique_ptr<epg::ITask>(
                    new CalibImuWriterTask(state, stop, runningFlag));
            }),
        catalog.MakeTaskFactoryEntry<CalibUdpPreviewTask>([state]() {
                return std::unique_ptr<epg::ITask>(
                    new CalibUdpPreviewTask(state));
            }),
        catalog.MakeTaskFactoryEntry<CalibCompletionTask>([state]() {
                return std::unique_ptr<epg::ITask>(
                    new CalibCompletionTask(state));
            }),
        catalog.MakeTaskFactoryEntry<CalibFlushSyncTask>([state, &completed, &sessionOk]() {
                return std::unique_ptr<epg::ITask>(
                    new CalibFlushSyncTask(state, completed, sessionOk));
            }),
        catalog.MakeTaskFactoryEntry<CalibMonitorTask>([&sessionOk, &completed]() {
                return std::unique_ptr<epg::ITask>(
                    new CalibMonitorTask(sessionOk, completed));
            }),
    });
}

} // namespace

bool RunCalibSessionGraph(const UnifiedConfig &cfg, std::atomic<bool> &stop, LivePoseState &livePose,
                          std::atomic<bool> &runningFlag)
{
    std::atomic<bool> sessionOk{true};
    std::atomic<bool> completed{false};

    {
        epg::Registry registry;
        auto state = std::make_shared<CalibRuntimeState>(cfg, stop, livePose, runningFlag);
        RegisterEpgTypes(
            registry, EpgDomain::CalibSession,
            MakeCalibGraphTaskFactoryResolver(state, stop, runningFlag, sessionOk, completed));

        epg::EventPipelineGraph graph(registry);
        graph.Configure(CompileEpgConfig(EpgDomain::CalibSession, registry));
        graph.Start();

        auto nextDfxSnapshot = std::chrono::steady_clock::now();
        while (runningFlag.load() && !stop.load() && !completed.load(std::memory_order_relaxed)) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= nextDfxSnapshot) {
                WriteEpgDfxSnapshotFile(
                    kCalibEpgDfxSnapshotPath,
                    graph.DfxSnapshotJson("cluster_calib_session_graph", EpgDfxNowMs()));
                nextDfxSnapshot = now + std::chrono::milliseconds(500);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        WriteEpgDfxSnapshotFile(
            kCalibEpgDfxSnapshotPath,
            graph.DfxSnapshotJson("cluster_calib_session_graph", EpgDfxNowMs()));
        if (!completed.load(std::memory_order_relaxed)) {
            state->Finalize(sessionOk.load(std::memory_order_relaxed));
        }
        graph.Stop();
    }

    return sessionOk.load(std::memory_order_relaxed);
}

} // namespace smartdrone::core::application
