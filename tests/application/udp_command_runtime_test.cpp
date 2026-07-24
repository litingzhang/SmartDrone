#include "core/application/runtime/udp_command_runtime.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "common/tlv/tlv_pack.h"
#include "common/tlv/tlv_protocol.h"

namespace {

using SmartDrone::Core::Application::CommandResult;
using SmartDrone::Core::Application::ConfigUpdate;
using SmartDrone::Core::Application::IRuntimeCommandTarget;
using SmartDrone::Core::Application::RuntimeAction;
using SmartDrone::Core::Application::UdpCommandRuntime;
using SmartDrone::Core::Application::UdpCommandRuntimeConfig;
using SmartDrone::Core::Application::UdpCommandRuntimeConfigValid;
using SmartDrone::Core::Application::UdpRuntimeStateSnapshot;

uint16_t FindAvailableUdpPort()
{
    const int socketFd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socketFd < 0) {
        return 0;
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    socklen_t addressLength = sizeof(address);
    const bool bound = ::bind(socketFd, reinterpret_cast<sockaddr *>(&address),
                              addressLength) == 0;
    const bool resolved = bound &&
        ::getsockname(socketFd, reinterpret_cast<sockaddr *>(&address),
                      &addressLength) == 0;
    ::close(socketFd);
    return resolved ? ntohs(address.sin_port) : 0;
}

class HeartbeatPeer final {
  public:
    HeartbeatPeer()
        : m_socketFd(::socket(AF_INET, SOCK_DGRAM, 0))
    {
    }

    ~HeartbeatPeer()
    {
        if (m_socketFd >= 0) {
            ::close(m_socketFd);
        }
    }

    bool Valid() const
    {
        return m_socketFd >= 0;
    }

    bool Send(uint16_t port) const
    {
        const TlvFrameBuildRequest request{
            TLV_VER, CMD_HEARTBEAT, 0, 1, 0, nullptr, 0};
        const std::vector<uint8_t> frame = MakeFrame(request);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(port);
        const ssize_t sent = ::sendto(
            m_socketFd, frame.data(), frame.size(), 0,
            reinterpret_cast<const sockaddr *>(&address), sizeof(address));
        return sent == static_cast<ssize_t>(frame.size());
    }

  private:
    int m_socketFd{-1};
};

class CapturingCommandHook final : public RuntimeCommandHook {
  public:
    RuntimeCommandGate ReadCommandGate() const override
    {
        return {};
    }

    bool ArmVehicle(std::string *) override
    {
        return true;
    }

    bool DisarmVehicle(std::string *) override
    {
        return true;
    }

    bool StopVehicleImmediately(std::string *) override
    {
        return true;
    }

    bool EnterGuidedControl(std::string *) override
    {
        return true;
    }

    bool HoldVehicle(std::string *) override
    {
        return true;
    }

    bool EnterPositionControl(std::string *) override
    {
        return true;
    }

    bool LandVehicle(std::string *) override
    {
        m_landCount.fetch_add(1, std::memory_order_relaxed);
        m_landEntered.store(true, std::memory_order_release);
        while (m_blockLand.load(std::memory_order_acquire) &&
               !m_releaseLand.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
        return m_landSucceeds.load(std::memory_order_acquire);
    }

    bool IsLandingConfirmed() const override
    {
        return m_landingConfirmed.load(std::memory_order_acquire);
    }

    bool ApplyMoveGoal(const MoveGoal &, std::string *) override
    {
        return true;
    }

    void SetLandSucceeds(bool succeeds)
    {
        m_landSucceeds.store(succeeds, std::memory_order_release);
    }

    void SetLandingConfirmed(bool confirmed)
    {
        m_landingConfirmed.store(confirmed, std::memory_order_release);
    }

    void BlockLand()
    {
        m_blockLand.store(true, std::memory_order_release);
        m_releaseLand.store(false, std::memory_order_release);
        m_landEntered.store(false, std::memory_order_release);
    }

    bool LandEntered() const
    {
        return m_landEntered.load(std::memory_order_acquire);
    }

    void ReleaseLand()
    {
        m_releaseLand.store(true, std::memory_order_release);
    }

    int LandCount() const
    {
        return m_landCount.load(std::memory_order_acquire);
    }

  private:
    std::atomic<bool> m_landSucceeds{true};
    std::atomic<bool> m_landingConfirmed{false};
    std::atomic<bool> m_blockLand{false};
    std::atomic<bool> m_releaseLand{false};
    std::atomic<bool> m_landEntered{false};
    std::atomic<int> m_landCount{0};
};

class NoopCommandTarget final : public IRuntimeCommandTarget {
  public:
    CommandResult ExecuteAction(const RuntimeAction &) override
    {
        return {true, {}};
    }

    CommandResult ApplyConfig(const ConfigUpdate &) override
    {
        return {true, {}};
    }
};

UdpCommandRuntimeConfig MakeRuntimeConfig(
    uint16_t port, CapturingCommandHook &hook, NoopCommandTarget &target,
    UdpRuntimeStateSnapshot &state)
{
    UdpCommandRuntimeConfig config{};
    config.port = port;
    config.heartbeatTimeoutMs = 10;
    config.heartbeatLandRetryMs = 10;
    config.commandHook = &hook;
    config.commandTarget = &target;
    config.currentConfig = []() {
        return SmartDrone::Core::Application::UnifiedConfig{};
    };
    config.currentRuntimeMode = []() {
        return SmartDrone::Core::Domain::RuntimeMode::Idle;
    };
    config.updateCommandPeer = [](const UdpPeer &) {};
    config.readRuntimeState = [&state](UdpRuntimeStateSnapshot &snapshot) {
        snapshot = state;
        return true;
    };
    return config;
}

TEST(UdpCommandRuntimeTest, RejectsLandRetryLongerThanHeartbeatTimeout)
{
    CapturingCommandHook hook;
    NoopCommandTarget target;
    UdpRuntimeStateSnapshot state{};
    UdpCommandRuntimeConfig config = MakeRuntimeConfig(14550, hook, target, state);
    config.heartbeatLandRetryMs = config.heartbeatTimeoutMs + 1;

    EXPECT_FALSE(UdpCommandRuntimeConfigValid(config));
}

TEST(UdpCommandRuntimeTest, TimeoutUsesCachedArmedStateAndRetriesFailedLand)
{
    const uint16_t port = FindAvailableUdpPort();
    ASSERT_NE(port, 0);
    CapturingCommandHook hook;
    NoopCommandTarget target;
    UdpRuntimeStateSnapshot state{};
    state.armed = true;
    state.px4FlightStateValid = false;
    HeartbeatPeer heartbeatPeer;
    ASSERT_TRUE(heartbeatPeer.Valid());
    UdpCommandRuntime runtime(MakeRuntimeConfig(port, hook, target, state));
    ASSERT_TRUE(runtime.Start());
    ASSERT_TRUE(heartbeatPeer.Send(port));
    runtime.StepReceive();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    hook.SetLandSucceeds(false);
    runtime.StepHeartbeatTimeout();
    EXPECT_EQ(hook.LandCount(), 1);

    hook.SetLandSucceeds(true);
    runtime.StepHeartbeatTimeout();
    runtime.StepHeartbeatTimeout();
    EXPECT_EQ(hook.LandCount(), 2);
}

TEST(UdpCommandRuntimeTest, SuccessfulLandRetriesUntilLandingIsConfirmed)
{
    const uint16_t port = FindAvailableUdpPort();
    ASSERT_NE(port, 0);
    CapturingCommandHook hook;
    NoopCommandTarget target;
    UdpRuntimeStateSnapshot state{};
    state.armed = true;
    HeartbeatPeer heartbeatPeer;
    ASSERT_TRUE(heartbeatPeer.Valid());
    UdpCommandRuntime runtime(MakeRuntimeConfig(port, hook, target, state));
    ASSERT_TRUE(runtime.Start());
    ASSERT_TRUE(heartbeatPeer.Send(port));
    runtime.StepReceive();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    runtime.StepHeartbeatTimeout();
    EXPECT_EQ(hook.LandCount(), 1);
    runtime.StepHeartbeatTimeout();
    EXPECT_EQ(hook.LandCount(), 1);

    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    runtime.StepHeartbeatTimeout();
    EXPECT_EQ(hook.LandCount(), 2);

    hook.SetLandingConfirmed(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    runtime.StepHeartbeatTimeout();
    EXPECT_EQ(hook.LandCount(), 2);
}

TEST(UdpCommandRuntimeTest, ConcurrentHeartbeatDoesNotSuppressFutureLand)
{
    const uint16_t port = FindAvailableUdpPort();
    ASSERT_NE(port, 0);
    CapturingCommandHook hook;
    NoopCommandTarget target;
    UdpRuntimeStateSnapshot state{};
    state.armed = true;
    HeartbeatPeer heartbeatPeer;
    ASSERT_TRUE(heartbeatPeer.Valid());
    UdpCommandRuntime runtime(MakeRuntimeConfig(port, hook, target, state));
    ASSERT_TRUE(runtime.Start());
    ASSERT_TRUE(heartbeatPeer.Send(port));
    runtime.StepReceive();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));

    hook.BlockLand();
    std::thread timeoutThread([&runtime]() {
        runtime.StepHeartbeatTimeout();
    });
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(1);
    while (!hook.LandEntered() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::yield();
    }
    const bool landEntered = hook.LandEntered();
    const bool heartbeatSent = heartbeatPeer.Send(port);
    if (heartbeatSent) {
        runtime.StepReceive();
    }
    hook.ReleaseLand();
    timeoutThread.join();
    ASSERT_TRUE(landEntered);
    ASSERT_TRUE(heartbeatSent);

    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    runtime.StepHeartbeatTimeout();
    EXPECT_EQ(hook.LandCount(), 2);
}

} // namespace
