#include "common/runtime_state.h"

#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <csignal>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>

namespace SmartDrone::Common {

namespace {

struct RuntimeStopPipe {
    RuntimeStopPipe();

    int readFd{-1};
    std::atomic<int> writeFd{-1};
};

void SetNonBlocking(int fd)
{
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
        (void)::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
}

RuntimeStopPipe::RuntimeStopPipe()
{
    std::array<int, 2> fds{-1, -1};
    if (::pipe(fds.data()) != 0) {
        return;
    }
    SetNonBlocking(fds[0]);
    SetNonBlocking(fds[1]);
    readFd = fds[0];
    writeFd.store(fds[1], std::memory_order_release);
}

RuntimeStopPipe g_stopPipe;

void NotifyRuntimeStopWaiter()
{
    const int fd = g_stopPipe.writeFd.load(std::memory_order_acquire);
    if (fd < 0) {
        return;
    }
    const char byte = 1;
    (void)::write(fd, &byte, sizeof(byte));
}

void DrainRuntimeStopPipe()
{
    char buffer[32];
    while (::read(g_stopPipe.readFd, buffer, sizeof(buffer)) > 0) {
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

bool WaitForRuntimeStop(std::chrono::milliseconds timeout)
{
    if (!g_runningFlag.load(std::memory_order_acquire)) {
        return true;
    }
    if (g_stopPipe.readFd < 0) {
        return RuntimeStopRequested();
    }
    pollfd fd{};
    fd.fd = g_stopPipe.readFd;
    fd.events = POLLIN;
    const int timeoutMs = static_cast<int>(timeout.count());
    const int result = ::poll(&fd, 1, timeoutMs);
    if (result > 0) {
        DrainRuntimeStopPipe();
        return RuntimeStopRequested();
    }
    if (result < 0 && errno != EINTR) {
        return RuntimeStopRequested();
    }
    return RuntimeStopRequested();
}

void WaitUntilRuntimeStopRequested()
{
    while (g_runningFlag.load()) {
        if (g_stopPipe.readFd < 0) {
            (void)::pause();
            continue;
        }
        pollfd fd{};
        fd.fd = g_stopPipe.readFd;
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

} // namespace SmartDrone::Common
