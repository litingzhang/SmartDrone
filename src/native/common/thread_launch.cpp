#include "common/thread_launch.h"

#include <iostream>
#include <mutex>

namespace smartdrone::common {
namespace {

std::mutex& ThreadLaunchLogMutex()
{
    static std::mutex mu;
    return mu;
}

}  // namespace

const char* ThreadRoleName(ThreadRole role)
{
    switch (role) {
        case ThreadRole::RuntimeWorker: return "runtime_worker";
        case ThreadRole::RuntimeSession: return "runtime_session";
        case ThreadRole::RuntimeForceRestart: return "runtime_force_restart";
        case ThreadRole::MavlinkRx: return "mavlink_rx";
        case ThreadRole::MavlinkTimesync: return "mavlink_timesync";
        case ThreadRole::MavlinkSetpointStream: return "mavlink_setpoint_stream";
        case ThreadRole::UdpImageCam0: return "udp_image_cam0";
        case ThreadRole::UdpImageCam1: return "udp_image_cam1";
        case ThreadRole::UdpCommand: return "udp_command";
        case ThreadRole::Imu: return "imu";
        case ThreadRole::CalibImuWriter: return "calib_imu_writer";
        case ThreadRole::ManualControl: return "manual_control";
    }
    return "unknown";
}

void LogThreadLaunch(const ThreadLaunchInfo& info, bool detached)
{
    std::lock_guard<std::mutex> lock(ThreadLaunchLogMutex());
    std::cerr << "[thread] launch"
              << " role=" << ThreadRoleName(info.role)
              << " owner=" << (info.owner ? info.owner : "unknown")
              << " file=" << (info.file ? info.file : "unknown")
              << ":" << info.line
              << " mode=" << (detached ? "detached" : "joinable")
              << "\n";
}

}  // namespace smartdrone::common
