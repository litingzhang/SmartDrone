#pragma once

namespace SmartDrone::Core::Ports {

enum SlamTrackingState : int {
    SLAM_TRACKING_SYSTEM_NOT_READY = -1,
    SLAM_TRACKING_NO_IMAGES_YET = 0,
    SLAM_TRACKING_NOT_INITIALIZED = 1,
    SLAM_TRACKING_OK = 2,
    SLAM_TRACKING_RECENTLY_LOST = 3,
    SLAM_TRACKING_LOST = 4,
    SLAM_TRACKING_OK_KLT = 5,
};

bool IsSlamTrackingPoseUsable(int trackingState);

} // namespace SmartDrone::Core::Ports
