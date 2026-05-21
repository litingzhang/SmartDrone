#include "core/application/runtime/epg_optimize_task.h"

#include <iostream>
#include <utility>

#include "core/application/epg/epg_runtime_optimizer.h"
#include "core/application/runtime/epg_dfx_snapshot.h"

namespace smartdrone::core::application {
namespace {

EpgRedeployRequest MakeRedeployRequest(const EpgTaskManifest &manifest,
                                       const EpgRuntimeOptimizerResult &result)
{
    return {
        manifest.subgraphName,
        result.message,
    };
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

void EpgOptimizeTask::OnTick(epg::TaskContext &context)
{
    (void)context;
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

EPG_REGISTER_TASK_TYPE(EpgOptimizeTask, "EpgOptimizeTask")

} // namespace smartdrone::core::application
