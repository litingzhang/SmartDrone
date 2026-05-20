#include "core/application/runtime/epg_optimize_task.h"

#include <iostream>
#include <utility>

#include "core/application/epg/epg_runtime_optimizer.h"
#include "core/application/runtime/epg_dfx_snapshot.h"

namespace smartdrone::core::application {

EpgOptimizeTask::EpgOptimizeTask(std::vector<EpgDomain> domains)
    : m_domains(std::move(domains))
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
        }
    }
}

EPG_REGISTER_TASK_TYPE(EpgOptimizeTask, "EpgOptimizeTask")

} // namespace smartdrone::core::application
