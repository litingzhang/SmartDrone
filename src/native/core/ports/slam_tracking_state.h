#pragma once

namespace SmartDrone::core::ports {

enum SlamTrackingState : int {
    kSlamTrackingSystemNotReady = -1,
    kSlamTrackingNoImagesYet = 0,
    kSlamTrackingNotInitialized = 1,
    kSlamTrackingOk = 2,
    kSlamTrackingRecentlyLost = 3,
    kSlamTrackingLost = 4,
    kSlamTrackingOkKlt = 5,
};

inline bool IsSlamTrackingPoseUsable(int trackingState)
{
    return trackingState == kSlamTrackingOk || trackingState == kSlamTrackingOkKlt;
}

} // namespace SmartDrone::core::ports
