#include "core/application/runtime/runtime_config_message.h"

#include <algorithm>
#include <string>

#include "core/application/config/app_args.h"

namespace SmartDrone::Core::Application {

std::string BuildRuntimeConfigMessage(const RemoteRuntimeConfig &remote,
                                      const UnifiedConfig &currentConfig)
{
    const int cameraFps =
        currentConfig.app.camera.fps > 0 ? currentConfig.app.camera.fps : 1;
    const int clampedSlamFps = remote.slamInputFps <= 0
                                   ? cameraFps
                                   : std::min(cameraFps, remote.slamInputFps);
    return "runtime cfg updated sensor=" +
           std::string(ToSensorModeText(remote.sensorMode)) +
           " backend=" + std::string(ToSlamBackendText(remote.slamBackend)) +
           " frontend=" +
           std::string(ToFeatureFrontendText(remote.featureFrontend)) +
           " slam_mode=" +
           std::string(Domain::ToString(remote.slamOperationMode)) +
           " slam_fps=" + std::to_string(clampedSlamFps) +
           " pair_ms=" + std::to_string(remote.pairMs) +
           " provider_specific visual_feature_top_k=" +
           std::to_string(remote.visualFeatureTopK) +
           " visual_feature_max_points=" +
           std::to_string(remote.visualFeatureMaxPoints) +
           " visual_feature_input_max=" +
           std::to_string(remote.visualFeatureInputMaxWidth) + "x" +
           std::to_string(remote.visualFeatureInputMaxHeight) +
           " lk_seed=gftt lk_accel=" + remote.lkPerFrameAcceleration +
           " orb_accel=" + remote.orbAcceleration +
           " px4_pose=" +
           std::string(ToPx4PoseOutputModeText(remote.px4PoseOutputMode)) +
           " avoid=" + (remote.avoidanceEnabled ? "on" : "off") +
           " avoid_radius_m=" + std::to_string(remote.avoidanceRadiusM) +
           " avoid_min_cloud_points=" +
           std::to_string(remote.avoidanceMinCloudPoints);
}

} // namespace SmartDrone::Core::Application
