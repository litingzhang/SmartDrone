#include "adapters/slam/openvins/openvins_runtime_driver.h"

#include <iostream>
#include <memory>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "adapters/slam/openvins/openvins_dynamic_init_policy.h"
#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/sensor_data.h"

namespace SmartDrone::Adapters::Slam {

namespace {

struct OpenVinsAdapterOptions {
    uint64_t initMinFrameId{0};
    double initMinTimestampSec{0.0};
};

Sophus::SE3f IdentityPose()
{
    return Sophus::SE3f();
}

bool IsValidStereoImuRequest(const Core::Ports::SlamTrackRequest &request)
{
    return request.input != nullptr &&
           request.inputMode == Core::Ports::SlamInputMode::Stereo &&
           request.useImu;
}

ov_core::CameraData MakeCameraData(
    const Core::Ports::SlamInputBatch &input)
{
    ov_core::CameraData cameraData;
    cameraData.timestamp = input.frameTimeSec;
    cameraData.sensor_ids = {0, 1};
    cameraData.images = {input.stereo.left.gray, input.stereo.right.gray};
    cameraData.masks = {
        cv::Mat::zeros(input.stereo.left.gray.rows,
                       input.stereo.left.gray.cols, CV_8UC1),
        cv::Mat::zeros(input.stereo.right.gray.rows,
                       input.stereo.right.gray.cols, CV_8UC1)};
    return cameraData;
}

void LogEmptyStereoImages(const ov_core::CameraData &cameraData,
                          uint64_t frameId)
{
    if (!cameraData.images[0].empty() && !cameraData.images[1].empty()) {
        return;
    }
    std::cerr << "[openvins] empty stereo image left="
              << cameraData.images[0].cols << "x"
              << cameraData.images[0].rows << " right="
              << cameraData.images[1].cols << "x"
              << cameraData.images[1].rows << " frame_id=" << frameId
              << "\n";
}

OpenVinsAdapterOptions LoadAdapterOptions(ov_core::YamlParser &parser)
{
    OpenVinsAdapterOptions options;
    int initMinFrameId = 0;
    parser.parse_config("smart_drone_init_min_frame_id", initMinFrameId,
                        false);
    parser.parse_config("smart_drone_init_min_timestamp_sec",
                        options.initMinTimestampSec, false);
    if (initMinFrameId > 0) {
        options.initMinFrameId = static_cast<uint64_t>(initMinFrameId);
    }
    return options;
}

void ApplyAdapterRuntimePolicy(ov_msckf::VioManagerOptions &options)
{
    options.num_opencv_threads = 1;
    options.use_multi_threading_pubs = false;
    options.use_multi_threading_subs = false;
    if (options.init_options.init_dyn_use &&
        !OpenVinsDynamicInitializationAvailable()) {
        std::cerr << "[openvins] dynamic initialization unavailable; "
                  << "using static initialization\n";
        options.init_options.init_dyn_use = false;
    }
}

Sophus::SE3f PoseFromState(const ov_msckf::State &state)
{
    const Eigen::Vector4d qGtoI = state._imu->quat();
    const Eigen::Vector3d pIinG = state._imu->pos();
    Eigen::Quaterniond qGtoIQuat(qGtoI(3), qGtoI(0), qGtoI(1), qGtoI(2));
    qGtoIQuat.normalize();
    const Eigen::Quaternionf qTwc = qGtoIQuat.conjugate().cast<float>();
    const Eigen::Vector3f tTwc = pIinG.cast<float>();
    return Sophus::SE3f(Sophus::SO3f(qTwc), tTwc);
}

class OpenVinsRuntimeDriverImpl final : public OpenVinsRuntimeDriver {
  public:
    explicit OpenVinsRuntimeDriverImpl(const std::string &settingsPath);

    bool Available() const override;
    void Shutdown() override;
    Sophus::SE3f TrackRaw(
        const Core::Ports::SlamTrackRequest &request) override;
    bool Initialized() const override;

  private:
    void FeedImu(const Core::Ports::SlamInputBatch &input);
    bool ShouldFeedCamera(const Core::Ports::SlamInputBatch &input) const;

    std::unique_ptr<ov_msckf::VioManager> m_manager;
    OpenVinsAdapterOptions m_options;
    bool m_initialized{false};
};

OpenVinsRuntimeDriverImpl::OpenVinsRuntimeDriverImpl(
    const std::string &settingsPath)
{
    if (settingsPath.empty()) {
        return;
    }
    auto parser = std::make_shared<ov_core::YamlParser>(settingsPath, false);
    if (parser == nullptr || !parser->successful()) {
        return;
    }
    ov_msckf::VioManagerOptions options;
    options.print_and_load(parser);
    m_options = LoadAdapterOptions(*parser);
    ApplyAdapterRuntimePolicy(options);
    m_manager = std::make_unique<ov_msckf::VioManager>(options);
}

bool OpenVinsRuntimeDriverImpl::Available() const
{
    return m_manager != nullptr;
}

void OpenVinsRuntimeDriverImpl::Shutdown()
{
    m_manager.reset();
    m_initialized = false;
}

Sophus::SE3f OpenVinsRuntimeDriverImpl::TrackRaw(
    const Core::Ports::SlamTrackRequest &request)
{
    if (!IsValidStereoImuRequest(request) || !Available()) {
        return IdentityPose();
    }

    ov_core::CameraData cameraData = MakeCameraData(*request.input);
    LogEmptyStereoImages(cameraData, request.input->frameId);
    FeedImu(*request.input);
    if (!ShouldFeedCamera(*request.input)) {
        m_initialized = false;
        return IdentityPose();
    }
    m_manager->feed_measurement_camera(cameraData);

    m_initialized = m_manager->initialized();
    if (!m_initialized) {
        return IdentityPose();
    }
    const std::shared_ptr<ov_msckf::State> state = m_manager->get_state();
    if (state == nullptr || state->_imu == nullptr) {
        m_initialized = false;
        return IdentityPose();
    }
    return PoseFromState(*state).inverse();
}

bool OpenVinsRuntimeDriverImpl::Initialized() const
{
    return m_initialized;
}

bool OpenVinsRuntimeDriverImpl::ShouldFeedCamera(
    const Core::Ports::SlamInputBatch &input) const
{
    return input.frameId >= m_options.initMinFrameId &&
           input.frameTimeSec >= m_options.initMinTimestampSec;
}

void OpenVinsRuntimeDriverImpl::FeedImu(
    const Core::Ports::SlamInputBatch &input)
{
    for (const Core::Ports::ImuReading &reading : input.imu) {
        ov_core::ImuData imuData;
        imuData.timestamp = static_cast<double>(reading.timestampNs) * 1e-9;
        imuData.wm << static_cast<double>(reading.gx),
            static_cast<double>(reading.gy),
            static_cast<double>(reading.gz);
        imuData.am << static_cast<double>(reading.ax),
            static_cast<double>(reading.ay),
            static_cast<double>(reading.az);
        m_manager->feed_measurement_imu(imuData);
    }
}

} // namespace

std::unique_ptr<OpenVinsRuntimeDriver>
CreateOpenVinsRuntimeDriver(const std::string &settingsPath)
{
    return std::make_unique<OpenVinsRuntimeDriverImpl>(settingsPath);
}

} // namespace SmartDrone::Adapters::Slam
