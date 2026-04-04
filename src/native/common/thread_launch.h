#pragma once

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

const char *ThreadRoleName(ThreadRole role);

struct ThreadLaunchInfo {
    ThreadRole role;
    const char *owner;
    const char *file;
    int line;
};

void LogThreadLaunch(const ThreadLaunchInfo &info, bool detached);

template <typename Fn> std::thread StartThread(ThreadLaunchInfo info, Fn &&fn)
{
    using Task = std::decay_t<Fn>;
    LogThreadLaunch(info, false);
    return std::thread(Task(std::forward<Fn>(fn)));
}

template <typename Fn> void StartDetachedThread(ThreadLaunchInfo info, Fn &&fn)
{
    using Task = std::decay_t<Fn>;
    LogThreadLaunch(info, true);
    std::thread(Task(std::forward<Fn>(fn))).detach();
}

} // namespace smartdrone::common

#define SMARTDRONE_THREAD_INFO(role, owner)                                                                            \
    ::smartdrone::common::ThreadLaunchInfo { role, owner, __FILE__, __LINE__ }

#define SMARTDRONE_START_THREAD(role, owner, ...)                                                                      \
    ::smartdrone::common::StartThread(SMARTDRONE_THREAD_INFO(role, owner), __VA_ARGS__)

#define SMARTDRONE_START_DETACHED_THREAD(role, owner, ...)                                                             \
    ::smartdrone::common::StartDetachedThread(SMARTDRONE_THREAD_INFO(role, owner), __VA_ARGS__)
