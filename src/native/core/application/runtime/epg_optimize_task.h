#pragma once

#include <memory>
#include <vector>

#include "common/epg/epg.h"
#include "core/application/epg/epg_task_manifest.h"
#include "core/application/runtime/epg_redeploy_coordinator.h"

namespace smartdrone::core::application {

class EpgOptimizeTask final : public epg::ITask {
  public:
    explicit EpgOptimizeTask(std::vector<EpgDomain> domains);
    EpgOptimizeTask(std::vector<EpgDomain> domains,
                    std::shared_ptr<EpgRedeployCoordinator> redeploy);
    void OnTick(epg::TaskContext &context) override;

  private:
    std::vector<EpgDomain> m_domains;
    std::shared_ptr<EpgRedeployCoordinator> m_redeploy;
};

} // namespace smartdrone::core::application
