#include "core/application/session/calib/calib_output_store.h"

#include <atomic>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "core/application/session/calib/calib_image_utils.h"
#include "core/application/session/calib/calib_output_sync.h"
#include "core/application/session/calib/calib_storage_helpers.h"
#include "core/ports/camera_provider.h"

namespace SmartDrone::core::application {

class CalibOutputStore::Impl final {
  public:
    explicit Impl(CalibOutputStoreConfig config)
        : m_config(std::move(config))
    {
    }

    ~Impl()
    {
        Close();
    }

    bool Open()
    {
        PrepareOutputDirs();
        if (!OpenOutputCsv()) {
            Close();
            return false;
        }
        InitOutputCsv();
        m_opened = true;
        return true;
    }

    bool WriteSavePair(const CalibSavePair &pair)
    {
        if (!pair.frame) {
            return false;
        }
        const auto &left = pair.frame->stereo.left;
        const auto &right = pair.frame->stereo.right;
        if (!ValidateImagesForSave(left, right)) {
            return false;
        }

        bool convertedLeft = false;
        bool convertedRight = false;
        const cv::Mat leftGray =
            EnsureCalibGray8(left.gray, convertedLeft);
        const cv::Mat rightGray =
            EnsureCalibGray8(right.gray, convertedRight);
        LogGrayConversion(left, right, convertedLeft || convertedRight);
        if (!WriteImages(pair, left, right, leftGray, rightGray)) {
            return false;
        }
        RecordSavedPair(pair);
        return true;
    }

    bool WriteImuSample(const ImuSample &sample)
    {
        std::lock_guard<std::mutex> lock(m_fileMu);
        if (!m_fImu) {
            return false;
        }
        std::fprintf(m_fImu,
                     "%lld,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                     static_cast<long long>(sample.tNs),
                     static_cast<double>(sample.gx),
                     static_cast<double>(sample.gy),
                     static_cast<double>(sample.gz),
                     static_cast<double>(sample.ax),
                     static_cast<double>(sample.ay),
                     static_cast<double>(sample.az));
        if ((++m_imuLines % 800) == 0) {
            std::fflush(m_fImu);
        }
        return true;
    }

    void FlushAndClose()
    {
        std::lock_guard<std::mutex> fileLock(m_fileMu);
        std::cerr << "[calib-sync] flushing outputs on calib stop saved="
                  << m_saved.load() << "\n";
        FlushCalibOutputs(
            {m_fCam0, m_fCam1, m_fImu, m_savedImagePaths, m_root, m_cam0Dir,
             m_cam1Dir});
        CloseLocked();
        m_opened = false;
    }

    void Close()
    {
        std::lock_guard<std::mutex> fileLock(m_fileMu);
        CloseLocked();
        m_opened = false;
    }

    bool Opened() const
    {
        return m_opened;
    }

    int SavedCount() const
    {
        return m_saved.load(std::memory_order_relaxed);
    }

    std::string OutputRoot() const
    {
        return m_outRoot;
    }

    std::filesystem::path Cam0Dir() const
    {
        return m_cam0Dir;
    }

    std::filesystem::path Cam1Dir() const
    {
        return m_cam1Dir;
    }

  private:
    void PrepareOutputDirs()
    {
        m_outRoot = MakeCalibSessionDir(m_config.root);
        m_root = fs::path(m_outRoot);
        m_cam0Dir = m_root / "cam0";
        m_cam1Dir = m_root / "cam1";
        EnsureDir(m_cam0Dir);
        EnsureDir(m_cam1Dir);
    }

    bool OpenOutputCsv()
    {
        m_fCam0 = std::fopen((m_cam0Dir / "data.csv").string().c_str(), "w");
        m_fCam1 = std::fopen((m_cam1Dir / "data.csv").string().c_str(), "w");
        m_fImu = std::fopen((m_root / "imu.csv").string().c_str(), "w");
        return m_fCam0 && m_fCam1 && m_fImu;
    }

    void InitOutputCsv()
    {
        SetupFileBuffer(m_fCam0, 1 << 20);
        SetupFileBuffer(m_fCam1, 1 << 20);
        SetupFileBuffer(m_fImu, 4 << 20);
        std::fprintf(m_fCam0, "#timestamp [ns],filename\n");
        std::fprintf(m_fCam1, "#timestamp [ns],filename\n");
        std::fprintf(m_fImu,
                     "#timestamp [ns],wX [rad/s],wY [rad/s],wZ [rad/s],"
                     "aX [m/s^2],aY [m/s^2],aZ [m/s^2]\n");
    }

    bool ValidateImagesForSave(
        const SmartDrone::core::ports::ImageFrame &left,
        const SmartDrone::core::ports::ImageFrame &right) const
    {
        if (!left.gray.empty() && !right.gray.empty()) {
            return true;
        }
        std::cerr << "[calib-write] empty image"
                  << " seqL=" << left.sequence << " seqR=" << right.sequence
                  << " rowsL=" << left.gray.rows
                  << " colsL=" << left.gray.cols
                  << " rowsR=" << right.gray.rows
                  << " colsR=" << right.gray.cols << "\n";
        return false;
    }

    void LogGrayConversion(
        const SmartDrone::core::ports::ImageFrame &left,
        const SmartDrone::core::ports::ImageFrame &right,
        bool converted)
    {
        if (!converted) {
            return;
        }
        ++m_conversionLogCount;
        if ((m_conversionLogCount % 30) != 1) {
            return;
        }
        std::cerr << "[calib-gray] converted to 8-bit"
                  << " typeL=" << left.gray.type()
                  << " typeR=" << right.gray.type()
                  << " count=" << m_conversionLogCount << "\n";
    }

    bool WriteImages(const CalibSavePair &pair,
                     const SmartDrone::core::ports::ImageFrame &left,
                     const SmartDrone::core::ports::ImageFrame &right,
                     const cv::Mat &leftGray,
                     const cv::Mat &rightGray) const
    {
        const bool okLeft = cv::imwrite(pair.fnL.string(), leftGray);
        const bool okRight = cv::imwrite(pair.fnR.string(), rightGray);
        if (okLeft && okRight) {
            return true;
        }
        std::cerr << "[calib-write] imwrite failed"
                  << " okL=" << (okLeft ? "true" : "false")
                  << " okR=" << (okRight ? "true" : "false")
                  << " pathL=" << pair.fnL.string()
                  << " pathR=" << pair.fnR.string()
                  << " typeL=" << left.gray.type()
                  << " typeR=" << right.gray.type()
                  << " rowsL=" << left.gray.rows
                  << " colsL=" << left.gray.cols
                  << " rowsR=" << right.gray.rows
                  << " colsR=" << right.gray.cols << "\n";
        return false;
    }

    void RecordSavedPair(const CalibSavePair &pair)
    {
        std::lock_guard<std::mutex> lock(m_fileMu);
        std::fprintf(m_fCam0, "%lld,%s\n",
                     static_cast<long long>(pair.pairNs), pair.name.c_str());
        std::fprintf(m_fCam1, "%lld,%s\n",
                     static_cast<long long>(pair.pairNs), pair.name.c_str());
        m_savedImagePaths.push_back(pair.fnL);
        m_savedImagePaths.push_back(pair.fnR);
        const int saved = m_saved.fetch_add(1, std::memory_order_relaxed) + 1;
        LogSavedPair(pair, saved);
        FlushImageCsvIfNeeded(saved);
    }

    void LogSavedPair(const CalibSavePair &pair, int saved) const
    {
        if (((saved - 1) % 30) != 0) {
            return;
        }
        std::cerr << "[calib-save] saved=" << saved
                  << " pathL=" << pair.fnL.string()
                  << " pathR=" << pair.fnR.string() << "\n";
    }

    void FlushImageCsvIfNeeded(int saved)
    {
        if ((saved % 50) != 0) {
            return;
        }
        std::fflush(m_fCam0);
        std::fflush(m_fCam1);
    }

    void CloseLocked()
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

    CalibOutputStoreConfig m_config;
    mutable std::mutex m_fileMu;
    std::string m_outRoot;
    fs::path m_root;
    fs::path m_cam0Dir;
    fs::path m_cam1Dir;
    FILE *m_fCam0{nullptr};
    FILE *m_fCam1{nullptr};
    FILE *m_fImu{nullptr};
    std::vector<fs::path> m_savedImagePaths;
    std::atomic<int> m_saved{0};
    int m_conversionLogCount{0};
    int m_imuLines{0};
    bool m_opened{false};
};

CalibOutputStore::CalibOutputStore(CalibOutputStoreConfig config)
    : m_impl(new Impl(std::move(config)))
{
}

CalibOutputStore::~CalibOutputStore() = default;

bool CalibOutputStore::Open()
{
    return m_impl->Open();
}

bool CalibOutputStore::WriteSavePair(const CalibSavePair &pair)
{
    return m_impl->WriteSavePair(pair);
}

bool CalibOutputStore::WriteImuSample(const ImuSample &sample)
{
    return m_impl->WriteImuSample(sample);
}

void CalibOutputStore::FlushAndClose()
{
    m_impl->FlushAndClose();
}

void CalibOutputStore::Close()
{
    m_impl->Close();
}

bool CalibOutputStore::Opened() const
{
    return m_impl->Opened();
}

int CalibOutputStore::SavedCount() const
{
    return m_impl->SavedCount();
}

std::string CalibOutputStore::OutputRoot() const
{
    return m_impl->OutputRoot();
}

std::filesystem::path CalibOutputStore::Cam0Dir() const
{
    return m_impl->Cam0Dir();
}

std::filesystem::path CalibOutputStore::Cam1Dir() const
{
    return m_impl->Cam1Dir();
}

} // namespace SmartDrone::core::application
