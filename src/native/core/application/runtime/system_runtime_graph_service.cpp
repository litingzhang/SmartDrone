#include "core/application/runtime/system_runtime_graph_service.h"

#include <atomic>
#include <iostream>
#include <memory>
#include <utility>

#include "common/epg/epg.h"
#include "common/tlv/udp_server.h"
#include "core/application/epg/epg_graph_runtime.h"
#include "core/application/runtime/discovery_beacon_runtime.h"
#include "core/application/runtime/epg_graph_lifecycle.h"
#include "core/application/runtime/epg_redeploy_coordinator.h"
#include "core/application/runtime/payload_builders.h"
#include "core/application/runtime/system_runtime_task_factory.h"
#include "core/application/runtime/udp_command_runtime.h"

namespace SmartDrone::Core::Application {
namespace {

bool ConfigValid(const SystemRuntimeGraphConfig &config)
{
    const SystemRuntimeStepServices services({
        config.stepVehicleTelemetryRx,
        config.stepSetpointStream,
        config.stepManualControl,
        config.stepForceRestart,
        config.stepSessionSupervisor,
        config.stepEpgRedeploy,
    });
    return services.Valid() &&
           config.redeployCoordinator &&
           UdpCommandRuntimeConfigValid(config.commandRuntime) &&
           config.aliases.cmdPort > 0 &&
           config.aliases.udpPort > 0 &&
           config.discoveryPort > 0 &&
           !config.cameraProvider.providerName.empty();
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

    ~Impl()
    {
        Stop();
    }

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
        auto discoveryRuntime =
            std::make_shared<DiscoveryBeaconRuntime>(
                m_config.discoveryPort, m_config.aliases.cmdPort,
                m_config.aliases.udpPort);
        auto stepServices = std::make_shared<SystemRuntimeStepServices>(
            SystemRuntimeStepServicesConfig{
                m_config.stepVehicleTelemetryRx,
                m_config.stepSetpointStream,
                m_config.stepManualControl,
                m_config.stepForceRestart,
                m_config.stepSessionSupervisor,
                m_config.stepEpgRedeploy,
            });
        auto graphRef = std::make_shared<EpgGraphRef>();
        auto graph = StartEpgGraph({
            EpgDomain::SystemRuntime,
            MakeSystemRuntimeTaskFactoryResolver({
                stepServices,
                commandRuntime,
                discoveryRuntime,
                graphRef,
                m_config.redeployCoordinator,
            }),
            graphRef,
            {},
        });
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
        commandConfig.buildCapabilitiesPayload =
            [cameraProvider = m_config.cameraProvider]() {
                return BuildCapabilitiesPayload(cameraProvider);
            };
        commandConfig.buildConfigPayload =
            [cameraProvider = m_config.cameraProvider](
                const UnifiedConfig &config, Domain::RuntimeMode mode) {
                return BuildConfigPayload(config, mode, cameraProvider);
            };
        commandConfig.peerToIpString = [](const UdpPeer &peer) {
            return UdpPeerToIpString(peer);
        };
        return std::make_shared<UdpCommandRuntime>(std::move(commandConfig));
    }

    SystemRuntimeGraphConfig m_config;
    std::atomic<bool> m_stop{false};
    EpgGraphLifecycle m_lifecycle;
};

SystemRuntimeGraph::SystemRuntimeGraph(SystemRuntimeGraphConfig config)
    : m_impl(new Impl(std::move(config)))
{
}

SystemRuntimeGraph::~SystemRuntimeGraph() = default;

bool SystemRuntimeGraph::Start()
{
    return m_impl->Start();
}

void SystemRuntimeGraph::Stop()
{
    m_impl->Stop();
}

} // namespace SmartDrone::Core::Application
