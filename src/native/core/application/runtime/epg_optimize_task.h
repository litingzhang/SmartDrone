#pragma once

#include <memory>
#include <vector>

#include "common/epg/epg.h"
#include "core/application/epg/epg_task_manifest.h"
#include "core/application/runtime/epg_redeploy_coordinator.h"

namespace SmartDrone::Core::Application {

class EpgOptimizeTask final : public Epg::ITask {
  public:
    explicit EpgOptimizeTask(std::vector<EpgDomain> domains);
    EpgOptimizeTask(std::vector<EpgDomain> domains,
                    std::shared_ptr<EpgRedeployCoordinator> redeploy);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::vector<EpgDomain> m_domains;
    std::shared_ptr<EpgRedeployCoordinator> m_redeploy;
};

} // namespace SmartDrone::Core::Application
