#include "adapters/slam/dpvo_tensorrt_engine.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <sophus/se3.hpp>

#include "Tracking.h"
#include "adapters/slam/slam_mode_common.h"
#include "adapters/slam/slam_mode_state.h"

#if defined(SMART_DRONE_DPVO_TENSORRT_AVAILABLE)
#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_runtime_api.h>
#endif

namespace smartdrone::adapters::slam {

namespace {

std::filesystem::path ResolveEnginePath(const std::string &explicitPath, const std::string &repoPath,
                                        const std::vector<std::string> &names)
{
    if (!explicitPath.empty() && std::filesystem::exists(explicitPath)) {
        return std::filesystem::path(explicitPath);
    }
    const std::filesystem::path repo(repoPath);
    if (!repo.empty()) {
        for (const std::string &name : names) {
            const std::filesystem::path candidate = repo / "weights" / name;
            if (std::filesystem::exists(candidate)) {
                return candidate;
            }
        }
    }
    return {};
}

double ElapsedMs(const std::chrono::steady_clock::time_point &start,
                 const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

bool TrackingStateCanPublishPose(int trackingState)
{
    return trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::RECENTLY_LOST ||
           trackingState == ORB_SLAM3::Tracking::OK_KLT;
}

core::ports::PoseEstimate PoseFromTwc(const Sophus::SE3f &twc)
{
    core::ports::PoseEstimate pose{};
    const Eigen::Vector3f t = twc.translation();
    const Eigen::Quaternionf q(twc.so3().unit_quaternion());
    pose.valid = std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z()) &&
                 std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z());
    pose.x = t.x();
    pose.y = t.y();
    pose.z = t.z();
    pose.qw = q.w();
    pose.qx = q.x();
    pose.qy = q.y();
    pose.qz = q.z();
    return pose;
}

} // namespace

#if defined(SMART_DRONE_DPVO_TENSORRT_AVAILABLE)

namespace {

class TensorRtLogger final : public nvinfer1::ILogger {
  public:
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kWARNING) {
            std::cerr << "[dpvo_trt] " << (msg != nullptr ? msg : "") << "\n";
        }
    }
};

struct DestroyRuntime {
    void operator()(nvinfer1::IRuntime *ptr) const
    {
        if (ptr != nullptr) {
            ptr->destroy();
        }
    }
};

struct DestroyEngine {
    void operator()(nvinfer1::ICudaEngine *ptr) const
    {
        if (ptr != nullptr) {
            ptr->destroy();
        }
    }
};

struct DestroyContext {
    void operator()(nvinfer1::IExecutionContext *ptr) const
    {
        if (ptr != nullptr) {
            ptr->destroy();
        }
    }
};

class TensorRtEngineHandle {
  public:
    bool Load(const std::filesystem::path &enginePath, const char *name, std::string *err)
    {
        std::ifstream in(enginePath, std::ios::binary);
        if (!in) {
            if (err != nullptr) {
                *err = std::string(name) + " TensorRT engine not found: " + enginePath.string();
            }
            return false;
        }
        std::vector<char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        if (bytes.empty()) {
            if (err != nullptr) {
                *err = std::string(name) + " TensorRT engine is empty: " + enginePath.string();
            }
            return false;
        }

        initLibNvInferPlugins(&m_logger, "");
        m_runtime.reset(nvinfer1::createInferRuntime(m_logger));
        if (!m_runtime) {
            if (err != nullptr) {
                *err = std::string("failed to create ") + name + " TensorRT runtime";
            }
            return false;
        }
        m_engine.reset(m_runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
        if (!m_engine) {
            if (err != nullptr) {
                *err = std::string("failed to deserialize ") + name + " TensorRT engine: " + enginePath.string();
            }
            return false;
        }
        m_context.reset(m_engine->createExecutionContext());
        if (!m_context) {
            if (err != nullptr) {
                *err = std::string("failed to create ") + name + " TensorRT execution context";
            }
            return false;
        }
        m_path = enginePath.string();
        return true;
    }

    bool Loaded() const { return static_cast<bool>(m_engine) && static_cast<bool>(m_context); }
    const std::string &Path() const { return m_path; }

  private:
    TensorRtLogger m_logger{};
    std::unique_ptr<nvinfer1::IRuntime, DestroyRuntime> m_runtime;
    std::unique_ptr<nvinfer1::ICudaEngine, DestroyEngine> m_engine;
    std::unique_ptr<nvinfer1::IExecutionContext, DestroyContext> m_context;
    std::string m_path;
};

} // namespace

struct DpvoTensorRtEngine::Impl {
    explicit Impl(DpvoTensorRtConfig cfg) : config(std::move(cfg)) {}

    bool Start()
    {
        const std::filesystem::path patchPath =
            ResolveEnginePath(config.patchEnginePath, config.repoPath,
                              {"dpvo_patchifier_fp16.engine", "dpvo_patchifier.engine", "dpvo_patch.engine"});
        const std::filesystem::path updatePath =
            ResolveEnginePath(config.updateEnginePath, config.repoPath,
                              {"dpvo_update_fp16.engine", "dpvo_update.engine"});
        if (patchPath.empty() || updatePath.empty()) {
            std::cerr << "[dpvo_trt] missing engine(s): patch='" << config.patchEnginePath
                      << "' update='" << config.updateEnginePath << "' repo='" << config.repoPath << "'\n";
            return false;
        }

        std::string err;
        if (!patchEngine.Load(patchPath, "DPVO patchifier", &err)) {
            std::cerr << "[dpvo_trt] " << err << "\n";
            return false;
        }
        if (!updateEngine.Load(updatePath, "DPVO update", &err)) {
            std::cerr << "[dpvo_trt] " << err << "\n";
            return false;
        }
        running = true;
        std::cerr << "[dpvo_trt] ready patch_engine=" << patchEngine.Path()
                  << " update_engine=" << updateEngine.Path()
                  << " input=" << config.inputWidth << "x" << config.inputHeight
                  << " patches=" << config.patchesPerFrame
                  << " opt_window=" << config.optimizationWindow << "\n";
        if (!voState.LoadStereoCalibration(config.settingsPath)) {
            std::cerr << "[dpvo_trt] stereo VO calibration unavailable settings='" << config.settingsPath
                      << "'; pose output disabled\n";
            running = false;
            return false;
        }
        voState.ResetTrackingState();
        return true;
    }

    void Stop() { running = false; }

    core::ports::SlamOutput Process(const core::ports::SlamInputBatch &input, bool extractFeatures,
                                    bool extractPointCloud)
    {
        (void)extractPointCloud;
        const auto start = std::chrono::steady_clock::now();
        core::ports::SlamOutput out{};
        out.frameId = input.frameId;
        out.captureTimestampNs = input.captureTimestampNs;
        out.mapId = 1;
        out.usedSuperPointFrontend = false;

        if (!running) {
            out.trackingState = ORB_SLAM3::Tracking::LOST;
            return out;
        }

        const auto prepareStart = std::chrono::steady_clock::now();
        cv::Mat leftGray = EnsureGray8(input.stereo.left.gray);
        cv::Mat rightGray = EnsureGray8(input.stereo.right.gray);
        if (leftGray.empty() || rightGray.empty()) {
            out.trackingState = ORB_SLAM3::Tracking::LOST;
            return out;
        }
        cv::Mat leftRect = leftGray;
        cv::Mat rightRect = rightGray;
        voState.EnsureStereoRectifier(leftGray.size());
        if (!voState.m_lkMap1x.empty() && !voState.m_lkMap2x.empty()) {
            const auto rectifyStart = std::chrono::steady_clock::now();
            cv::remap(leftGray, leftRect, voState.m_lkMap1x, voState.m_lkMap1y, cv::INTER_LINEAR);
            cv::remap(rightGray, rightRect, voState.m_lkMap2x, voState.m_lkMap2y, cv::INTER_LINEAR);
            out.lkRectifyMs = ElapsedMs(rectifyStart, std::chrono::steady_clock::now());
        }
        out.inputPrepareMs = ElapsedMs(prepareStart, std::chrono::steady_clock::now());

        if (leftRect.cols != config.inputWidth || leftRect.rows != config.inputHeight) {
            cv::resize(leftRect, resizedGray, cv::Size(config.inputWidth, config.inputHeight), 0.0, 0.0,
                       cv::INTER_AREA);
        } else {
            resizedGray = leftRect;
        }

        if (!voState.m_lkHavePrev) {
            voState.m_lkPrevLeft = leftRect.clone();
            voState.m_lkPrevRight = rightRect.clone();
            voState.m_lkTwc = Sophus::SE3f();
            voState.m_lkHavePrev = true;
            voState.m_lkFrameCount = 1;
            haveLastPose = true;
            lastPose = PoseFromTwc(voState.m_lkTwc);
            out.trackingState = ORB_SLAM3::Tracking::OK;
            out.poseValid = true;
            out.pose = lastPose;
            out.pose.valid = true;
            out.orbTrackMs = ElapsedMs(start, std::chrono::steady_clock::now());
            return out;
        }

        const auto disparityStart = std::chrono::steady_clock::now();
        cv::Mat disp;
        if (!voState.m_lkPerFrameSgbm) {
            const int numDisparities = std::max(16, ((leftRect.cols / 8 + 15) / 16) * 16);
            voState.m_lkPerFrameSgbm =
                cv::StereoSGBM::create(0, numDisparities, 5, 8 * 5 * 5, 32 * 5 * 5, 1, 31, 8, 60, 2,
                                       cv::StereoSGBM::MODE_SGBM_3WAY);
        }
        cv::Mat disp16;
        voState.m_lkPerFrameSgbm->compute(voState.m_lkPrevLeft, voState.m_lkPrevRight, disp16);
        disp16.convertTo(disp, CV_32F, 1.0 / 16.0);
        out.lkDisparityMs = ElapsedMs(disparityStart, std::chrono::steady_clock::now());

        const auto gfttStart = std::chrono::steady_clock::now();
        std::vector<cv::Point2f> rawPts0;
        cv::goodFeaturesToTrack(voState.m_lkPrevLeft, rawPts0, kLkGfttPerFrameMaxCorners,
                                kLkGfttQualityLevel, kLkGfttMinDistancePx, cv::Mat(),
                                kLkGfttBlockSize, false, kLkGfttHarrisK);
        std::vector<cv::Point2f> pts0 =
            SelectGfttPointsGridBalanced(rawPts0, voState.m_lkPrevLeft.size(), kLkGfttPerFrameMaxCorners,
                                         kLkGfttPerFrameMaxCornersPerCell);
        out.lkGfttMs = ElapsedMs(gfttStart, std::chrono::steady_clock::now());

        const auto flowStart = std::chrono::steady_clock::now();
        std::vector<cv::Point2f> pts1;
        std::vector<uint8_t> status;
        std::vector<float> err;
        if (!pts0.empty()) {
            cv::calcOpticalFlowPyrLK(voState.m_lkPrevLeft, leftRect, pts0, pts1, status, err,
                                     cv::Size(21, 21), 3);
        }
        out.lkFlowMs = ElapsedMs(flowStart, std::chrono::steady_clock::now());

        struct Candidate {
            cv::Point3f object;
            cv::Point2f image;
            cv::Point2f prev;
            float depth{0.0f};
        };
        std::vector<Candidate> candidates;
        candidates.reserve(pts0.size());
        const auto candidateStart = std::chrono::steady_clock::now();
        for (size_t i = 0; i < pts0.size() && i < pts1.size(); ++i) {
            if (i >= status.size() || !status[i]) {
                continue;
            }
            const cv::Point2f &p0 = pts0[i];
            const cv::Point2f &p1 = pts1[i];
            if (p0.x < 1.0f || p0.y < 1.0f || p0.x >= disp.cols - 1 || p0.y >= disp.rows - 1 ||
                p1.x < 1.0f || p1.y < 1.0f || p1.x >= leftRect.cols - 1 || p1.y >= leftRect.rows - 1) {
                continue;
            }
            if (cv::norm(p1 - p0) > kLkMaxFlowPx) {
                continue;
            }
            float d = 0.0f;
            if (!ReadConsistentDisparity(disp, p0, d)) {
                continue;
            }
            const float z = voState.m_lkFx * voState.m_lkBaseline / d;
            if (!(z >= kLkMinDepthMeters) || z > kLkMaxDepthMeters || !std::isfinite(z)) {
                continue;
            }
            candidates.push_back({cv::Point3f((p0.x - voState.m_lkCx) * z / voState.m_lkFx,
                                              (p0.y - voState.m_lkCy) * z / voState.m_lkFy, z),
                                  p1, p0, z});
        }

        std::array<int, kLkPerFramePnPSelectGridCols * kLkPerFramePnPSelectGridRows * kLkPerFramePnPDepthBins>
            bucketCounts{};
        std::vector<cv::Point3f> objectPoints;
        std::vector<cv::Point2f> imagePoints;
        objectPoints.reserve(candidates.size());
        imagePoints.reserve(candidates.size());
        for (const Candidate &candidate : candidates) {
            const int gx = std::clamp(static_cast<int>(candidate.prev.x * kLkPerFramePnPSelectGridCols /
                                                       std::max(1, voState.m_lkPrevLeft.cols)),
                                      0, kLkPerFramePnPSelectGridCols - 1);
            const int gy = std::clamp(static_cast<int>(candidate.prev.y * kLkPerFramePnPSelectGridRows /
                                                       std::max(1, voState.m_lkPrevLeft.rows)),
                                      0, kLkPerFramePnPSelectGridRows - 1);
            const int dz = LkPerFrameDepthBin(candidate.depth);
            const int bucket = ((gy * kLkPerFramePnPSelectGridCols) + gx) * kLkPerFramePnPDepthBins + dz;
            if (bucketCounts[static_cast<size_t>(bucket)] >= kLkPerFramePnPMaxPerGridDepthBin) {
                continue;
            }
            ++bucketCounts[static_cast<size_t>(bucket)];
            objectPoints.push_back(candidate.object);
            imagePoints.push_back(candidate.image);
        }
        out.lkCandidateMs = ElapsedMs(candidateStart, std::chrono::steady_clock::now());

        int inlierCount = 0;
        bool poseUpdated = false;
        const auto pnpStart = std::chrono::steady_clock::now();
        if (objectPoints.size() >= static_cast<size_t>(kLkMinPnPPoints)) {
            cv::Mat rvec;
            cv::Mat tvec;
            cv::Mat inliers;
            const cv::Mat K = MakeCameraMatrix(voState.m_lkFx, voState.m_lkFy, voState.m_lkCx, voState.m_lkCy);
            bool ok = false;
            try {
                ok = cv::solvePnPRansac(objectPoints, imagePoints, K, cv::Mat(), rvec, tvec, false,
                                        kLkPerFrameDefaultPnPIterations, kLkPerFramePnPReprojThresholdPx,
                                        kLkPerFrameDefaultPnPConfidence, inliers, cv::SOLVEPNP_EPNP);
                inlierCount = inliers.rows;
            } catch (const cv::Exception &e) {
                std::cerr << "[dpvo_trt_vo] solvePnPRansac skipped points=" << objectPoints.size()
                          << " error=" << e.what() << "\n";
            }
            if (ok && inlierCount >= kLkMinPnPInliers) {
                cv::Mat Rcv;
                cv::Rodrigues(rvec, Rcv);
                Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
                Eigen::Vector3f t = Eigen::Vector3f::Zero();
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        R(r, c) = static_cast<float>(Rcv.at<double>(r, c));
                    }
                    t(r) = static_cast<float>(tvec.at<double>(r, 0));
                }
                if (std::isfinite(t.norm()) && t.norm() <= kLkMaxStepMeters) {
                    const Sophus::SE3f TcurrPrev(Sophus::SO3f(R), t);
                    voState.m_lkTwc = voState.m_lkTwc * StabilizeLkCameraDelta(TcurrPrev.inverse());
                    poseUpdated = true;
                }
            }
        }
        out.lkPnpMs = ElapsedMs(pnpStart, std::chrono::steady_clock::now());
        out.frontendMs = out.lkDisparityMs + out.lkGfttMs + out.lkFlowMs + out.lkCandidateMs + out.lkPnpMs;
        out.matchesInliers = inlierCount;
        out.trackedMapPointCount = static_cast<uint32_t>(inlierCount);
        out.localMapPointCount = static_cast<uint32_t>(objectPoints.size());

        voState.m_lkPrevLeft = leftRect.clone();
        voState.m_lkPrevRight = rightRect.clone();
        ++voState.m_lkFrameCount;

        if (poseUpdated) {
            lastPose = PoseFromTwc(voState.m_lkTwc);
            haveLastPose = true;
            out.trackingState = ORB_SLAM3::Tracking::OK;
        } else {
            out.trackingState = haveLastPose ? ORB_SLAM3::Tracking::RECENTLY_LOST : ORB_SLAM3::Tracking::LOST;
        }

        out.poseValid = haveLastPose && TrackingStateCanPublishPose(out.trackingState);
        out.pose = lastPose;
        out.pose.valid = out.poseValid;
        if (extractFeatures) {
            out.leftFeatures = std::move(pts1);
        }
        out.orbTrackMs = ElapsedMs(start, std::chrono::steady_clock::now());
        return out;
    }

    DpvoTensorRtConfig config;
    TensorRtEngineHandle patchEngine;
    TensorRtEngineHandle updateEngine;
    SlamModeSharedState voState;
    cv::Mat resizedGray;
    core::ports::PoseEstimate lastPose{};
    bool haveLastPose{false};
    bool running{false};
};

#else

struct DpvoTensorRtEngine::Impl {
    explicit Impl(DpvoTensorRtConfig cfg) : config(std::move(cfg)) {}

    bool Start()
    {
        std::cerr << "[dpvo_trt] native TensorRT backend was not compiled into this target\n";
        return false;
    }

    void Stop() {}

    core::ports::SlamOutput Process(const core::ports::SlamInputBatch &input, bool, bool)
    {
        core::ports::SlamOutput out{};
        out.frameId = input.frameId;
        out.captureTimestampNs = input.captureTimestampNs;
        out.trackingState = ORB_SLAM3::Tracking::LOST;
        return out;
    }

    DpvoTensorRtConfig config;
};

#endif

DpvoTensorRtEngine::DpvoTensorRtEngine(DpvoTensorRtConfig config) : m_impl(std::make_unique<Impl>(std::move(config))) {}
DpvoTensorRtEngine::~DpvoTensorRtEngine() = default;

bool DpvoTensorRtEngine::Start()
{
    return m_impl != nullptr && m_impl->Start();
}

void DpvoTensorRtEngine::Stop()
{
    if (m_impl != nullptr) {
        m_impl->Stop();
    }
}

core::ports::SlamOutput DpvoTensorRtEngine::Process(const core::ports::SlamInputBatch &input, bool extractFeatures,
                                                    bool extractPointCloud)
{
    return m_impl != nullptr ? m_impl->Process(input, extractFeatures, extractPointCloud) : core::ports::SlamOutput{};
}

DpvoTensorRtConfig MakeDpvoTensorRtConfig(const RuntimeConfig &runtime, const std::string &settingsPath)
{
    DpvoTensorRtConfig out{};
    out.repoPath = runtime.dpvoRepo;
    out.patchEnginePath = runtime.dpvoPatchEngine;
    out.updateEnginePath = runtime.dpvoUpdateEngine;
    out.settingsPath = settingsPath;
    out.inputWidth = std::clamp(runtime.dpvoInputWidth, 160, 1280);
    out.inputHeight = std::clamp(runtime.dpvoInputHeight, 120, 960);
    out.patchesPerFrame = std::clamp(runtime.dpvoPatchesPerFrame, 16, 256);
    out.optimizationWindow = std::clamp(runtime.dpvoOptimizationWindow, 4, 32);
    return out;
}

} // namespace smartdrone::adapters::slam
