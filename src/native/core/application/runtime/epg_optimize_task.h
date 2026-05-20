#pragma once

#include <memory>
#include <vector>

#include "common/epg/epg.h"
#include "core/application/epg/epg_task_manifest.h"

namespace smartdrone::core::application {

class EpgOptimizeTask final : public epg::ITask {
  public:
    explicit EpgOptimizeTask(std::vector<EpgDomain> domains);
    void OnTick(epg::TaskContext &context) override;

  private:
    std::vector<EpgDomain> m_domains;
};

} // namespace smartdrone::core::application
