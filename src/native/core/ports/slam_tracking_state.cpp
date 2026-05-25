#include "core/ports/slam_tracking_state.h"

namespace SmartDrone::Core::Ports {

bool IsSlamTrackingPoseUsable(int trackingState)
{
    return trackingState == SLAM_TRACKING_OK ||
           trackingState == SLAM_TRACKING_OK_KLT;
}

} // namespace SmartDrone::Core::Ports
