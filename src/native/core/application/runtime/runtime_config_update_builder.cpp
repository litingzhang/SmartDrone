#include "core/application/runtime/runtime_config_update_builder.h"

#include <cstdint>
#include <string>

#include "core/application/config/app_args.h"
#include "core/application/config/config_registry.h"

namespace SmartDrone::Core::Application {
namespace {

void AddCameraAndModeConfig(ConfigUpdate &update,
                            const RemoteRuntimeConfig &remote)
{
    update.values[std::string(ConfigRegistry::CAMERA_EXPOSURE_US)] =
        static_cast<std::int64_t>(remote.exposureUs);
    update.values[std::string(ConfigRegistry::CAMERA_GAIN)] =
        static_cast<double>(remote.gain);
    update.values[std::string(ConfigRegistry::CAMERA_AUTO_EXPOSURE)] =
        remote.autoExposureEnabled;
    update.values[std::string(ConfigRegistry::CAMERA_PAIR_WINDOW_MS)] =
        static_cast<std::int64_t>(remote.pairMs);
    update.values[std::string(ConfigRegistry::SLAM_INPUT_FPS)] =
        static_cast<std::int64_t>(remote.slamInputFps);
    update.values[std::string(ConfigRegistry::SLAM_BACKEND)] =
        std::string(ToSlamBackendText(remote.slamBackend));
    update.values[std::string(ConfigRegistry::SLAM_FEATURE_FRONTEND)] =
        std::string(ToFeatureFrontendText(remote.featureFrontend));
    update.values[std::string(ConfigRegistry::SLAM_OPERATION_MODE)] =
        std::string(SmartDrone::Core::Domain::ToString(
            remote.slamOperationMode));
    update.values[std::string(ConfigRegistry::SLAM_PERCEPTION_MODE)] =
        std::string(ToSensorModeText(remote.sensorMode));
}

void AddStreamConfig(ConfigUpdate &update,
                     const RemoteRuntimeConfig &remote)
{
    update.values[std::string(ConfigRegistry::STREAM_UDP_ENABLED)] =
        remote.udpEnabled;
    update.values[std::string(ConfigRegistry::STREAM_UDP_IP)] = remote.udpIp;
    update.values[std::string(ConfigRegistry::STREAM_SEND_IMAGE)] =
        remote.sendImage;
    update.values[std::string(ConfigRegistry::STREAM_SEND_FEATURE)] =
        remote.sendFeature;
    update.values[std::string(ConfigRegistry::STREAM_SEND_MAP)] =
        remote.sendMap;
}

void AddPx4Config(ConfigUpdate &update, const RemoteRuntimeConfig &remote)
{
    update.values[std::string(ConfigRegistry::PX4_POSE_OUTPUT_MODE)] =
        std::string(ToPx4PoseOutputModeText(remote.px4PoseOutputMode));
}

void AddTbcConfig(ConfigUpdate &update, const RemoteRuntimeConfig &remote)
{
    update.values[std::string(ConfigRegistry::SLAM_USE_CUSTOM_TBC)] =
        remote.useCustomTbc;
    update.values[std::string(ConfigRegistry::SLAM_TBC_TX)] =
        static_cast<double>(remote.tbcTx);
    update.values[std::string(ConfigRegistry::SLAM_TBC_TY)] =
        static_cast<double>(remote.tbcTy);
    update.values[std::string(ConfigRegistry::SLAM_TBC_TZ)] =
        static_cast<double>(remote.tbcTz);
    update.values[std::string(ConfigRegistry::SLAM_TBC_ROLL_DEG)] =
        static_cast<double>(remote.tbcRollDeg);
    update.values[std::string(ConfigRegistry::SLAM_TBC_PITCH_DEG)] =
        static_cast<double>(remote.tbcPitchDeg);
    update.values[std::string(ConfigRegistry::SLAM_TBC_YAW_DEG)] =
        static_cast<double>(remote.tbcYawDeg);
}

void AddOrbConfig(ConfigUpdate &update, const RemoteRuntimeConfig &remote)
{
    update.values[std::string(ConfigRegistry::SLAM_ORB_N_FEATURES)] =
        static_cast<std::int64_t>(remote.orbNFeatures);
    update.values[std::string(ConfigRegistry::SLAM_ORB_SCALE_FACTOR)] =
        static_cast<double>(remote.orbScaleFactor);
    update.values[std::string(ConfigRegistry::SLAM_ORB_N_LEVELS)] =
        static_cast<std::int64_t>(remote.orbNLevels);
    update.values[std::string(ConfigRegistry::SLAM_ORB_INI_TH_FAST)] =
        static_cast<std::int64_t>(remote.orbIniThFAST);
    update.values[std::string(ConfigRegistry::SLAM_ORB_MIN_TH_FAST)] =
        static_cast<std::int64_t>(remote.orbMinThFAST);
    update.values[std::string(ConfigRegistry::SLAM_ORB_ACCELERATION)] =
        remote.orbAcceleration;
}

void AddVisualFeatureConfig(ConfigUpdate &update,
                            const RemoteRuntimeConfig &remote)
{
    update.values[std::string(ConfigRegistry::SLAM_VISUAL_FEATURE_TOP_K)] =
        static_cast<std::int64_t>(remote.visualFeatureTopK);
    update.values[std::string(ConfigRegistry::SLAM_VISUAL_FEATURE_MAX_POINTS)] =
        static_cast<std::int64_t>(remote.visualFeatureMaxPoints);
    update.values[std::string(ConfigRegistry::SLAM_VISUAL_FEATURE_INPUT_MAX_WIDTH)] =
        static_cast<std::int64_t>(remote.visualFeatureInputMaxWidth);
    update.values[std::string(ConfigRegistry::SLAM_VISUAL_FEATURE_INPUT_MAX_HEIGHT)] =
        static_cast<std::int64_t>(remote.visualFeatureInputMaxHeight);
    update.values[std::string(ConfigRegistry::SLAM_LK_SUPER_POINT_SEEDING)] =
        false;
    update.values[std::string(ConfigRegistry::SLAM_LK_PER_FRAME_ACCELERATION)] =
        remote.lkPerFrameAcceleration;
}

void AddAvoidanceConfig(ConfigUpdate &update,
                        const RemoteRuntimeConfig &remote)
{
    update.values[std::string(ConfigRegistry::AVOIDANCE_ENABLED)] =
        remote.avoidanceEnabled;
    update.values[std::string(ConfigRegistry::AVOIDANCE_HOLD_ON_STALE_CLOUD)] =
        remote.avoidanceHoldOnStaleCloud;
    update.values[std::string(ConfigRegistry::AVOIDANCE_RADIUS_M)] =
        static_cast<double>(remote.avoidanceRadiusM);
    update.values[std::string(ConfigRegistry::AVOIDANCE_LOOKAHEAD_M)] =
        static_cast<double>(remote.avoidanceLookaheadM);
    update.values[std::string(ConfigRegistry::AVOIDANCE_SPEED_LOOKAHEAD_S)] =
        static_cast<double>(remote.avoidanceSpeedLookaheadS);
    update.values[std::string(ConfigRegistry::AVOIDANCE_NEAR_FIELD_RADIUS_M)] =
        static_cast<double>(remote.avoidanceNearFieldRadiusM);
    update.values[std::string(ConfigRegistry::AVOIDANCE_MAX_POINT_AGE_MS)] =
        static_cast<std::int64_t>(remote.avoidanceMaxPointCloudAgeMs);
    update.values[std::string(ConfigRegistry::AVOIDANCE_MIN_CLOUD_POINTS)] =
        static_cast<std::int64_t>(remote.avoidanceMinCloudPoints);
    update.values[std::string(ConfigRegistry::AVOIDANCE_MIN_BLOCKING_POINTS)] =
        static_cast<std::int64_t>(remote.avoidanceMinBlockingPoints);
}

} // namespace

ConfigUpdate BuildRuntimeConfigUpdate(const RemoteRuntimeConfig &remote)
{
    ConfigUpdate update{};
    AddCameraAndModeConfig(update, remote);
    AddStreamConfig(update, remote);
    AddPx4Config(update, remote);
    AddTbcConfig(update, remote);
    AddOrbConfig(update, remote);
    AddVisualFeatureConfig(update, remote);
    AddAvoidanceConfig(update, remote);
    return update;
}

} // namespace SmartDrone::Core::Application
