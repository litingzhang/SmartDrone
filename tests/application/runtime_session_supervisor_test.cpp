#include "core/application/runtime/runtime_session_supervisor.h"
#include "core/application/session/epg/session_graph_runtime.h"

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

using SmartDrone::Core::Application::ISessionGraphRuntime;
using SmartDrone::Core::Application::RuntimeSessionSupervisor;
using SmartDrone::Core::Application::UnifiedConfig;
using RuntimeMode = SmartDrone::Core::Domain::RuntimeMode;
using SessionStartRequest = RuntimeSessionSupervisor::SessionStartRequest;

struct SessionProbe {
    RuntimeMode mode{RuntimeMode::Idle};
    bool stopFlagAtCreate{false};
    bool startOk{true};
    bool ok{true};
    bool done{false};
    int startCount{0};
    int stepCount{0};
    int requestStopCount{0};
    int stopCount{0};
};

class FakeSessionGraphRuntime final : public ISessionGraphRuntime {
  public:
    explicit FakeSessionGraphRuntime(std::shared_ptr<SessionProbe> probe)
        : m_probe(std::move(probe))
    {
    }

    bool Start() override
    {
        ++m_probe->startCount;
        return m_probe->startOk;
    }

    void Step() override
    {
        ++m_probe->stepCount;
    }

    void RequestStop() override
    {
        ++m_probe->requestStopCount;
        m_probe->done = true;
    }

    void Stop() override
    {
        ++m_probe->stopCount;
        m_probe->done = true;
    }

    bool Done() override
    {
        return m_probe->done;
    }

    bool Ok() const override
    {
        return m_probe->ok;
    }

  private:
    std::shared_ptr<SessionProbe> m_probe;
};

class RuntimeSessionSupervisorTest : public ::testing::Test {
  protected:
    RuntimeSessionSupervisor MakeSupervisor()
    {
        return RuntimeSessionSupervisor(RuntimeSessionSupervisor::Config{
            runningFlag,
            [this]() {
                ++configReads;
                return config;
            },
            [this](const SessionStartRequest &request) {
                return CreateSession(request);
            }});
    }

    std::unique_ptr<ISessionGraphRuntime> CreateSession(
        const SessionStartRequest &request)
    {
        auto probe = std::make_shared<SessionProbe>();
        probe->mode = request.mode;
        probe->stopFlagAtCreate = request.stop.load(std::memory_order_acquire);
        probes.push_back(probe);
        return std::make_unique<FakeSessionGraphRuntime>(std::move(probe));
    }

    std::atomic<bool> runningFlag{true};
    UnifiedConfig config{};
    std::vector<std::shared_ptr<SessionProbe>> probes;
    int configReads{0};
};

TEST_F(RuntimeSessionSupervisorTest, LaunchesRequestedSessionAndStepsIt)
{
    RuntimeSessionSupervisor supervisor = MakeSupervisor();

    std::string err;
    ASSERT_TRUE(supervisor.RequestMode(RuntimeMode::Slam, &err));
    supervisor.Step();

    ASSERT_EQ(probes.size(), 1U);
    EXPECT_EQ(probes[0]->mode, RuntimeMode::Slam);
    EXPECT_FALSE(probes[0]->stopFlagAtCreate);
    EXPECT_EQ(probes[0]->startCount, 1);
    EXPECT_EQ(configReads, 1);
    EXPECT_EQ(supervisor.DesiredMode(), RuntimeMode::Slam);
    EXPECT_EQ(supervisor.ActiveMode(), RuntimeMode::Slam);

    supervisor.Step();

    EXPECT_EQ(probes[0]->stepCount, 1);
}

TEST_F(RuntimeSessionSupervisorTest, RestartStopsActiveSessionAndLaunchesReplacement)
{
    RuntimeSessionSupervisor supervisor = MakeSupervisor();

    std::string err;
    ASSERT_TRUE(supervisor.RequestMode(RuntimeMode::Slam, &err));
    supervisor.Step();
    ASSERT_EQ(probes.size(), 1U);

    supervisor.RequestRestart();
    supervisor.Step();

    ASSERT_EQ(probes.size(), 2U);
    EXPECT_EQ(probes[0]->requestStopCount, 1);
    EXPECT_EQ(probes[1]->mode, RuntimeMode::Slam);
    EXPECT_EQ(probes[1]->startCount, 1);
    EXPECT_EQ(supervisor.DesiredMode(), RuntimeMode::Slam);
    EXPECT_EQ(supervisor.ActiveMode(), RuntimeMode::Slam);
}

TEST_F(RuntimeSessionSupervisorTest, StopSynchronouslyStopsSessionAndRejectsModes)
{
    RuntimeSessionSupervisor supervisor = MakeSupervisor();

    std::string err;
    ASSERT_TRUE(supervisor.RequestMode(RuntimeMode::Calib, &err));
    supervisor.Step();
    ASSERT_EQ(probes.size(), 1U);

    supervisor.Stop();

    RuntimeSessionSupervisor::IdleStatus status = supervisor.GetIdleStatus();
    EXPECT_TRUE(status.idle);
    EXPECT_TRUE(status.stopping);
    EXPECT_EQ(probes[0]->stopCount, 1);
    EXPECT_EQ(supervisor.DesiredMode(), RuntimeMode::Idle);
    EXPECT_EQ(supervisor.ActiveMode(), RuntimeMode::Idle);

    EXPECT_FALSE(supervisor.RequestMode(RuntimeMode::Slam, &err));
    EXPECT_EQ(err, "runtime stopping");
}

} // namespace
