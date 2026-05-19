#include "core/application/runtime/system_runtime_graph_service.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cerrno>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "common/epg/epg.h"
#include "core/application/runtime/epg_dfx_snapshot.h"
#include "core/application/runtime/epg_graph_lifecycle.h"
#include "core/application/session/epg_registry.h"

namespace smartdrone::core::application {
namespace {

constexpr const char *kDiscoveryMagic = "smartdrone_discovery";
constexpr const char *kSystemEpgDfxSnapshotPath = "/tmp/smartdrone_epg_system.json";
constexpr auto kDiscoveryPeriod = std::chrono::seconds(1);
constexpr auto kDiscoveryOpenRetryPeriod = std::chrono::seconds(1);

bool SetSocketNonBlocking(int fd)
{
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        return false;
    }
    return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

class DiscoveryBeaconRuntime;

struct SystemTaskFactoryDeps {
    RuntimeGraphStepFn stepVehicleTelemetryRx;
    RuntimeGraphStepFn stepSetpointStream;
    RuntimeGraphStepFn stepManualControl;
    RuntimeGraphStepFn stepForceRestart;
    RuntimeGraphStepFn stepSessionSupervisor;
    std::shared_ptr<UdpCommandRuntime> commandRuntime;
    std::shared_ptr<DiscoveryBeaconRuntime> discoveryRuntime;
    std::shared_ptr<EpgGraphRef> graphRef;
};

template <class TaskTag>
class VehicleTelemetryRxStepTask final : public epg::ITask {
  public:
    explicit VehicleTelemetryRxStepTask(RuntimeGraphStepFn step) : m_step(std::move(step)) {}

    void OnTick(epg::TaskContext &context) override
    {
        (void)context;
        if (m_step) {
            m_step();
        }
    }

  private:
    RuntimeGraphStepFn m_step;
};

struct VehicleTelemetryRxTaskTag {};
using VehicleTelemetryRxTask = VehicleTelemetryRxStepTask<VehicleTelemetryRxTaskTag>;
EPG_REGISTER_TASK_TYPE(VehicleTelemetryRxTask, "VehicleTelemetryRxTask")

class SetpointStreamTask final : public epg::ITask {
  public:
    explicit SetpointStreamTask(RuntimeGraphStepFn step) : m_step(std::move(step)) {}

    void OnTick(epg::TaskContext &context) override
    {
        (void)context;
        if (m_step) {
            m_step();
        }
    }

  private:
    RuntimeGraphStepFn m_step;
};
EPG_REGISTER_TASK_TYPE(SetpointStreamTask, "SetpointStreamTask")

class UdpCommandTask final : public epg::ITask {
  public:
    explicit UdpCommandTask(std::shared_ptr<UdpCommandRuntime> runtime) : m_runtime(std::move(runtime)) {}

    ~UdpCommandTask() override
    {
        if (m_runtime) {
            m_runtime->Stop();
        }
    }

    void OnTick(epg::TaskContext &context) override
    {
        (void)context;
        if (m_runtime) {
            m_runtime->Step();
        }
    }

  private:
    std::shared_ptr<UdpCommandRuntime> m_runtime;
};
EPG_REGISTER_TASK_TYPE(UdpCommandTask, "UdpCommandTask")

class ManualControlTask final : public epg::ITask {
  public:
    explicit ManualControlTask(RuntimeGraphStepFn step) : m_step(std::move(step)) {}

    void OnTick(epg::TaskContext &context) override
    {
        (void)context;
        if (m_step) {
            m_step();
        }
    }

  private:
    RuntimeGraphStepFn m_step;
};
EPG_REGISTER_TASK_TYPE(ManualControlTask, "ManualControlTask")

class ForceRestartTask final : public epg::ITask {
  public:
    explicit ForceRestartTask(RuntimeGraphStepFn step) : m_step(std::move(step)) {}

    void OnTick(epg::TaskContext &context) override
    {
        (void)context;
        if (m_step) {
            m_step();
        }
    }

  private:
    RuntimeGraphStepFn m_step;
};
EPG_REGISTER_TASK_TYPE(ForceRestartTask, "ForceRestartTask")

class RuntimeSupervisorTask final : public epg::ITask {
  public:
    explicit RuntimeSupervisorTask(RuntimeGraphStepFn step) : m_step(std::move(step)) {}

    void OnTick(epg::TaskContext &context) override
    {
        (void)context;
        if (m_step) {
            m_step();
        }
    }

  private:
    RuntimeGraphStepFn m_step;
};
EPG_REGISTER_TASK_TYPE(RuntimeSupervisorTask, "RuntimeSupervisorTask")

class DiscoveryBeaconRuntime final {
  public:
    DiscoveryBeaconRuntime(int discoveryPort, int cmdPort, int videoPort)
        : m_discoveryPort(discoveryPort), m_cmdPort(cmdPort), m_videoPort(videoPort)
    {
    }

    ~DiscoveryBeaconRuntime() { Stop(); }

    bool Start()
    {
        if (m_fd >= 0) {
            return true;
        }
        const auto now = std::chrono::steady_clock::now();
        if (!CanRetryOpen(now)) {
            return false;
        }
        m_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (m_fd < 0) {
            m_nextOpenAttempt = now + kDiscoveryOpenRetryPeriod;
            std::cerr << "[discovery] socket open failed\n";
            return false;
        }
        if (!SetSocketNonBlocking(m_fd)) {
            Stop();
            m_nextOpenAttempt = now + kDiscoveryOpenRetryPeriod;
            std::cerr << "[discovery] socket nonblock failed\n";
            return false;
        }
        int one = 1;
        ::setsockopt(m_fd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));
        ::setsockopt(m_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        m_dst.sin_family = AF_INET;
        m_dst.sin_port = htons(static_cast<uint16_t>(m_discoveryPort));
        m_dst.sin_addr.s_addr = htonl(INADDR_BROADCAST);
        m_payload = std::string(kDiscoveryMagic) + ";device=cm5;cmd=" + std::to_string(m_cmdPort) +
                    ";video=" + std::to_string(m_videoPort);
        m_nextOpenAttempt = {};
        return true;
    }

    void Stop()
    {
        if (m_fd < 0) {
            return;
        }
        ::close(m_fd);
        m_fd = -1;
    }

    void Step()
    {
        if (!Start()) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (m_lastSent.time_since_epoch().count() != 0 && now - m_lastSent < kDiscoveryPeriod) {
            return;
        }
        m_lastSent = now;
        const ssize_t sent = ::sendto(m_fd, m_payload.data(), m_payload.size(), 0,
                                      reinterpret_cast<const sockaddr *>(&m_dst), sizeof(m_dst));
        LogFirstSendResult(sent);
    }

  private:
    bool CanRetryOpen(std::chrono::steady_clock::time_point now) const
    {
        return m_nextOpenAttempt.time_since_epoch().count() == 0 || now >= m_nextOpenAttempt;
    }

    void LogFirstSendResult(ssize_t sent)
    {
        if (!m_firstLog) {
            return;
        }
        m_firstLog = false;
        if (sent < 0) {
            std::cerr << "[discovery] broadcast failed errno=" << errno << "\n";
            return;
        }
        std::cerr << "[discovery] broadcasting on udp/" << m_discoveryPort << " cmd=" << m_cmdPort
                  << " video=" << m_videoPort << "\n";
    }

    int m_discoveryPort{0};
    int m_cmdPort{0};
    int m_videoPort{0};
    int m_fd{-1};
    sockaddr_in m_dst{};
    std::string m_payload;
    std::chrono::steady_clock::time_point m_lastSent{};
    std::chrono::steady_clock::time_point m_nextOpenAttempt{};
    bool m_firstLog{true};
};

class DiscoveryBeaconTask final : public epg::ITask {
  public:
    explicit DiscoveryBeaconTask(std::shared_ptr<DiscoveryBeaconRuntime> runtime) : m_runtime(std::move(runtime)) {}

    ~DiscoveryBeaconTask() override
    {
        if (m_runtime) {
            m_runtime->Stop();
        }
    }

    void OnTick(epg::TaskContext &context) override
    {
        (void)context;
        if (m_runtime) {
            m_runtime->Step();
        }
    }

  private:
    std::shared_ptr<DiscoveryBeaconRuntime> m_runtime;
};
EPG_REGISTER_TASK_TYPE(DiscoveryBeaconTask, "DiscoveryBeaconTask")

epg::TypeCatalog::TaskFactoryResolver MakeSystemTaskFactoryResolver(SystemTaskFactoryDeps deps)
{
    auto &catalog = epg::TypeCatalog::Global();
    return epg::TypeCatalog::MakeTaskFactoryResolver({
        catalog.MakeTaskFactoryEntry<VehicleTelemetryRxTask>(
            [deps]() { return std::unique_ptr<epg::ITask>(new VehicleTelemetryRxTask(deps.stepVehicleTelemetryRx)); }),
        catalog.MakeTaskFactoryEntry<SetpointStreamTask>(
            [deps]() { return std::unique_ptr<epg::ITask>(new SetpointStreamTask(deps.stepSetpointStream)); }),
        catalog.MakeTaskFactoryEntry<UdpCommandTask>(
            [deps]() { return std::unique_ptr<epg::ITask>(new UdpCommandTask(deps.commandRuntime)); }),
        catalog.MakeTaskFactoryEntry<ManualControlTask>(
            [deps]() { return std::unique_ptr<epg::ITask>(new ManualControlTask(deps.stepManualControl)); }),
        catalog.MakeTaskFactoryEntry<ForceRestartTask>(
            [deps]() { return std::unique_ptr<epg::ITask>(new ForceRestartTask(deps.stepForceRestart)); }),
        catalog.MakeTaskFactoryEntry<RuntimeSupervisorTask>(
            [deps]() { return std::unique_ptr<epg::ITask>(new RuntimeSupervisorTask(deps.stepSessionSupervisor)); }),
        catalog.MakeTaskFactoryEntry<DiscoveryBeaconTask>(
            [deps]() { return std::unique_ptr<epg::ITask>(new DiscoveryBeaconTask(deps.discoveryRuntime)); }),
        catalog.MakeTaskFactoryEntry<EpgDfxSnapshotTask>(
            [deps]() {
                return std::unique_ptr<epg::ITask>(new EpgDfxSnapshotTask({
                    deps.graphRef,
                    "cluster_system_runtime_graph",
                    kSystemEpgDfxSnapshotPath,
                }));
            }),
    });
}

bool ConfigValid(const SystemRuntimeGraphConfig &config)
{
    return config.stepVehicleTelemetryRx && config.stepSetpointStream && config.stepManualControl &&
           config.stepForceRestart && config.stepSessionSupervisor &&
           UdpCommandRuntimeConfigValid(config.commandRuntime) && config.aliases.cmdPort > 0 &&
           config.aliases.udpPort > 0 && config.discoveryPort > 0;
}

} // namespace

class SystemRuntimeGraph::Impl final {
  public:
    explicit Impl(SystemRuntimeGraphConfig config)
        : m_config(std::move(config)),
          m_lifecycle(EpgGraphLifecycleConfig{
              m_stop,
              []() { return true; },
              {},
              []() {},
          })
    {
    }

    ~Impl() { Stop(); }

    bool Start()
    {
        if (m_lifecycle.HasGraph()) {
            return true;
        }
        if (!ConfigValid(m_config)) {
            std::cerr << "[runtime] system EPG config invalid\n";
            return false;
        }
        m_lifecycle.ResetForStart();
        auto commandRuntime = MakeCommandRuntime();
        auto discoveryRuntime = std::make_shared<DiscoveryBeaconRuntime>(
            m_config.discoveryPort, m_config.aliases.cmdPort, m_config.aliases.udpPort);
        auto graphRef = std::make_shared<EpgGraphRef>();
        epg::Registry registry;
        RegisterEpgTypes(registry, EpgDomain::SystemRuntime,
                         MakeSystemTaskFactoryResolver({
                             m_config.stepVehicleTelemetryRx,
                             m_config.stepSetpointStream,
                             m_config.stepManualControl,
                             m_config.stepForceRestart,
                             m_config.stepSessionSupervisor,
                             commandRuntime,
                             discoveryRuntime,
                             graphRef,
                         }));
        auto graph = std::make_unique<epg::EventPipelineGraph>(registry);
        graphRef->graph = graph.get();
        graph->Configure(CompileEpgConfig(EpgDomain::SystemRuntime, registry));
        graph->Start();
        m_lifecycle.AttachGraph(std::move(graph));
        return true;
    }

    void Stop()
    {
        if (m_lifecycle.Done()) {
            return;
        }
        if (m_lifecycle.HasGraph()) {
            m_lifecycle.StopSynchronously();
        }
    }

  private:
    std::shared_ptr<UdpCommandRuntime> MakeCommandRuntime()
    {
        UdpCommandRuntimeConfig commandConfig = m_config.commandRuntime;
        commandConfig.port = m_config.aliases.cmdPort;
        commandConfig.buildCapabilitiesPayload = m_config.buildCapabilitiesPayload;
        commandConfig.buildConfigPayload = m_config.buildConfigPayload;
        commandConfig.peerToIpString = m_config.peerToIpString;
        return std::make_shared<UdpCommandRuntime>(std::move(commandConfig));
    }

    SystemRuntimeGraphConfig m_config;
    std::atomic<bool> m_stop{false};
    EpgGraphLifecycle m_lifecycle;
};

SystemRuntimeGraph::SystemRuntimeGraph(SystemRuntimeGraphConfig config) : m_impl(new Impl(std::move(config))) {}

SystemRuntimeGraph::~SystemRuntimeGraph() = default;

bool SystemRuntimeGraph::Start() { return m_impl->Start(); }

void SystemRuntimeGraph::Stop() { m_impl->Stop(); }

} // namespace smartdrone::core::application
