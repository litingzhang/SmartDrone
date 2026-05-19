#include "core/application/runtime/system_runtime_graph_service.h"

#include <atomic>
#include <iostream>
#include <memory>
#include <utility>

#include "common/epg/epg.h"
#include "core/application/runtime/discovery_beacon_runtime.h"
#include "core/application/runtime/epg_dfx_snapshot.h"
#include "core/application/runtime/epg_graph_lifecycle.h"
#include "core/application/session/epg_registry.h"

namespace smartdrone::core::application {
namespace {

constexpr const char *kSystemEpgDfxSnapshotPath = "/tmp/smartdrone_epg_system.json";

struct SystemTaskFactoryDeps {
    std::shared_ptr<SystemRuntimeStepServices> stepServices;
    std::shared_ptr<UdpCommandRuntime> commandRuntime;
    std::shared_ptr<DiscoveryBeaconRuntime> discoveryRuntime;
    std::shared_ptr<EpgGraphRef> graphRef;
};

class VehicleTelemetryRxTask final : public epg::ITask {
  public:
    explicit VehicleTelemetryRxTask(std::shared_ptr<SystemRuntimeStepServices> services)
        : m_services(std::move(services))
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        (void)context;
        if (m_services) {
            m_services->OnVehicleTelemetryRxGraphTick();
        }
    }

  private:
    std::shared_ptr<SystemRuntimeStepServices> m_services;
};
EPG_REGISTER_TASK_TYPE(VehicleTelemetryRxTask, "VehicleTelemetryRxTask")

class SetpointStreamTask final : public epg::ITask {
  public:
    explicit SetpointStreamTask(std::shared_ptr<SystemRuntimeStepServices> services)
        : m_services(std::move(services))
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        (void)context;
        if (m_services) {
            m_services->OnSetpointStreamGraphTick();
        }
    }

  private:
    std::shared_ptr<SystemRuntimeStepServices> m_services;
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
            m_runtime->OnGraphTick();
        }
    }

  private:
    std::shared_ptr<UdpCommandRuntime> m_runtime;
};
EPG_REGISTER_TASK_TYPE(UdpCommandTask, "UdpCommandTask")

class ManualControlTask final : public epg::ITask {
  public:
    explicit ManualControlTask(std::shared_ptr<SystemRuntimeStepServices> services)
        : m_services(std::move(services))
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        (void)context;
        if (m_services) {
            m_services->OnManualControlGraphTick();
        }
    }

  private:
    std::shared_ptr<SystemRuntimeStepServices> m_services;
};
EPG_REGISTER_TASK_TYPE(ManualControlTask, "ManualControlTask")

class ForceRestartTask final : public epg::ITask {
  public:
    explicit ForceRestartTask(std::shared_ptr<SystemRuntimeStepServices> services)
        : m_services(std::move(services))
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        (void)context;
        if (m_services) {
            m_services->OnForceRestartGraphTick();
        }
    }

  private:
    std::shared_ptr<SystemRuntimeStepServices> m_services;
};
EPG_REGISTER_TASK_TYPE(ForceRestartTask, "ForceRestartTask")

class RuntimeSupervisorTask final : public epg::ITask {
  public:
    explicit RuntimeSupervisorTask(std::shared_ptr<SystemRuntimeStepServices> services)
        : m_services(std::move(services))
    {
    }

    void OnTick(epg::TaskContext &context) override
    {
        (void)context;
        if (m_services) {
            m_services->OnSessionSupervisorGraphTick();
        }
    }

  private:
    std::shared_ptr<SystemRuntimeStepServices> m_services;
};
EPG_REGISTER_TASK_TYPE(RuntimeSupervisorTask, "RuntimeSupervisorTask")

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
            m_runtime->OnGraphTick();
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
            [deps]() { return std::unique_ptr<epg::ITask>(new VehicleTelemetryRxTask(deps.stepServices)); }),
        catalog.MakeTaskFactoryEntry<SetpointStreamTask>(
            [deps]() { return std::unique_ptr<epg::ITask>(new SetpointStreamTask(deps.stepServices)); }),
        catalog.MakeTaskFactoryEntry<UdpCommandTask>(
            [deps]() { return std::unique_ptr<epg::ITask>(new UdpCommandTask(deps.commandRuntime)); }),
        catalog.MakeTaskFactoryEntry<ManualControlTask>(
            [deps]() { return std::unique_ptr<epg::ITask>(new ManualControlTask(deps.stepServices)); }),
        catalog.MakeTaskFactoryEntry<ForceRestartTask>(
            [deps]() { return std::unique_ptr<epg::ITask>(new ForceRestartTask(deps.stepServices)); }),
        catalog.MakeTaskFactoryEntry<RuntimeSupervisorTask>(
            [deps]() { return std::unique_ptr<epg::ITask>(new RuntimeSupervisorTask(deps.stepServices)); }),
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
    const SystemRuntimeStepServices services({
        config.stepVehicleTelemetryRx,
        config.stepSetpointStream,
        config.stepManualControl,
        config.stepForceRestart,
        config.stepSessionSupervisor,
    });
    return services.Valid() && UdpCommandRuntimeConfigValid(config.commandRuntime) && config.aliases.cmdPort > 0 &&
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
        auto stepServices = std::make_shared<SystemRuntimeStepServices>(
            SystemRuntimeStepServicesConfig{
                m_config.stepVehicleTelemetryRx,
                m_config.stepSetpointStream,
                m_config.stepManualControl,
                m_config.stepForceRestart,
                m_config.stepSessionSupervisor,
            });
        auto graphRef = std::make_shared<EpgGraphRef>();
        epg::Registry registry;
        RegisterEpgTypes(registry, EpgDomain::SystemRuntime,
                         MakeSystemTaskFactoryResolver({
                             stepServices,
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
