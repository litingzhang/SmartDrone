#pragma once

#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

namespace smartdrone::common {

enum class ThreadRole {
    RuntimeWorker,
    RuntimeSession,
    RuntimeForceRestart,
    MavlinkRx,
    MavlinkTimesync,
    MavlinkSetpointStream,
    UdpImageCam0,
    UdpImageCam1,
    UdpCommand,
    Imu,
    CalibImuWriter,
    ManualControl
};

inline const char* ThreadRoleName(ThreadRole role)
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

struct ThreadLaunchInfo {
    ThreadRole role;
    const char* owner;
    const char* file;
    int line;
};

inline std::mutex& ThreadLaunchLogMutex()
{
    static std::mutex mu;
    return mu;
}

inline void LogThreadLaunch(const ThreadLaunchInfo& info, bool detached)
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

template <typename Fn>
std::thread StartThread(ThreadLaunchInfo info, Fn&& fn)
{
    using Task = std::decay_t<Fn>;
    LogThreadLaunch(info, false);
    return std::thread(Task(std::forward<Fn>(fn)));
}

template <typename Fn>
void StartDetachedThread(ThreadLaunchInfo info, Fn&& fn)
{
    using Task = std::decay_t<Fn>;
    LogThreadLaunch(info, true);
    std::thread(Task(std::forward<Fn>(fn))).detach();
}

}  // namespace smartdrone::common

#define SMARTDRONE_THREAD_INFO(role, owner) \
    ::smartdrone::common::ThreadLaunchInfo{role, owner, __FILE__, __LINE__}

#define SMARTDRONE_START_THREAD(role, owner, ...) \
    ::smartdrone::common::StartThread(SMARTDRONE_THREAD_INFO(role, owner), __VA_ARGS__)

#define SMARTDRONE_START_DETACHED_THREAD(role, owner, ...) \
    ::smartdrone::common::StartDetachedThread(SMARTDRONE_THREAD_INFO(role, owner), __VA_ARGS__)
