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
    DiscoveryBeacon,
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

inline ThreadLaunchInfo MakeThreadLaunchInfo(ThreadRole role, const char *owner, const char *file = nullptr, int line = 0)
{
    return ThreadLaunchInfo{role, owner, file, line};
}

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
