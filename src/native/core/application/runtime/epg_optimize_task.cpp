#include "core/application/runtime/epg_optimize_task.h"

#include <iostream>
#include <utility>

#include "core/application/epg/epg_runtime_optimizer.h"
#include "core/application/runtime/epg_dfx_snapshot.h"
#include "core/application/runtime/system_runtime_messages.h"

namespace SmartDrone::Core::Application {
namespace {

EpgRedeployRequest MakeRedeployRequest(const EpgTaskManifest &manifest,
                                       const EpgRuntimeOptimizerResult &result)
{
    EpgRedeployRequest request;
    request.graphName = result.targetGraph.empty()
                            ? manifest.subgraphName
                            : result.targetGraph;
    request.reason = result.message;
    request.topologyVersion = result.topologyVersion;
    request.sourceProfile = result.sourceProfile;
    request.sourceProfilePath = result.sourceProfilePath;
    request.sourceTimestampMs = result.sourceTimestampMs;
    request.generatedAtMs = result.generatedAtMs;
    request.solverVersion = result.solverVersion;
    request.optimizedConfigPath = result.optimizedConfigPath;
    request.solverReportPath = result.solverReportPath;
    return request;
}

} // namespace

EpgOptimizeTask::EpgOptimizeTask(std::vector<EpgDomain> domains)
    : m_domains(std::move(domains))
{
}

EpgOptimizeTask::EpgOptimizeTask(
    std::vector<EpgDomain> domains,
    std::shared_ptr<EpgRedeployCoordinator> redeploy)
    : m_domains(std::move(domains)), m_redeploy(std::move(redeploy))
{
}

void EpgOptimizeTask::OnTick(Epg::TaskContext &context)
{
    DrainSystemRuntimePulse(context);
    const std::uint64_t nowMs = EpgDfxNowMs();
    for (const auto domain : m_domains) {
        const auto &manifest = EpgManifestForDomain(domain);
        const auto result = OptimizeEpgProfileForManifest(manifest, nowMs);
        if (result.optimized) {
            std::cerr << "[epg] " << manifest.subgraphName
                      << " optimizer: " << result.message << "\n";
        } else if (result.message != "profile missing") {
            std::cerr << "[epg] " << manifest.subgraphName
                      << " optimizer skipped: " << result.message << "\n";
        }
        if (!result.configChanged || !m_redeploy) {
            continue;
        }
        const auto request = MakeRedeployRequest(manifest, result);
        if (domain == EpgDomain::SystemRuntime) {
            m_redeploy->RequestSystemRedeploy(request);
        } else {
            m_redeploy->RequestSessionRedeploy(request);
        }
    }
}

const bool EPG_OPTIMIZE_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<EpgOptimizeTask>(
        "EpgOptimizeTask");

} // namespace SmartDrone::Core::Application
