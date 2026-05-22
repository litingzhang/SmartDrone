#include "common/runtime_state.h"

#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <csignal>
#include <unistd.h>

#include <array>
#include <atomic>
#include <mutex>

namespace SmartDrone::common {

namespace {

std::once_flag g_stopPipeInitOnce;
int g_stopPipeReadFd{-1};
std::atomic<int> g_stopPipeWriteFd{-1};

void SetNonBlocking(int fd)
{
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        (void)::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
}

void InitRuntimeStopPipe()
{
    std::array<int, 2> fds{-1, -1};
    if (::pipe(fds.data()) != 0) {
        return;
    }
    SetNonBlocking(fds[0]);
    SetNonBlocking(fds[1]);
    g_stopPipeReadFd = fds[0];
    g_stopPipeWriteFd.store(fds[1], std::memory_order_release);
}

void EnsureRuntimeStopPipe()
{
    std::call_once(g_stopPipeInitOnce, InitRuntimeStopPipe);
}

void NotifyRuntimeStopWaiter()
{
    const int fd = g_stopPipeWriteFd.load(std::memory_order_acquire);
    if (fd < 0) {
        return;
    }
    const char byte = 1;
    (void)::write(fd, &byte, sizeof(byte));
}

void DrainRuntimeStopPipe()
{
    char buffer[32];
    while (::read(g_stopPipeReadFd, buffer, sizeof(buffer)) > 0) {
    }
}

} // namespace

std::atomic<bool> g_runningFlag{true};

void SigIntHandler(int)
{
    g_runningFlag.store(false);
    NotifyRuntimeStopWaiter();
}

void RequestRuntimeStop()
{
    g_runningFlag.store(false);
    NotifyRuntimeStopWaiter();
}

bool RuntimeStopRequested()
{
    return !g_runningFlag.load(std::memory_order_acquire);
}

void WaitUntilRuntimeStopRequested()
{
    EnsureRuntimeStopPipe();
    while (g_runningFlag.load()) {
        if (g_stopPipeReadFd < 0) {
            (void)::pause();
            continue;
        }
        pollfd fd{};
        fd.fd = g_stopPipeReadFd;
        fd.events = POLLIN;
        const int result = ::poll(&fd, 1, -1);
        if (result < 0 && errno == EINTR) {
            continue;
        }
        if (result > 0) {
            DrainRuntimeStopPipe();
        }
    }
}

} // namespace SmartDrone::common
