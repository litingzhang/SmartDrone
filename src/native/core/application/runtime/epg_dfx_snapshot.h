#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "common/epg/epg.h"

namespace smartdrone::core::application {

struct EpgGraphRef {
    epg::EventPipelineGraph *graph{nullptr};
};

struct EpgDfxSnapshotTarget {
    std::shared_ptr<EpgGraphRef> graphRef;
    std::string graphName;
    std::string topologyVersion;
    std::string taskCatalogJson;
    std::string snapshotPath;
    std::string profilePath;
};

class EpgDfxSnapshotTask final : public epg::ITask {
  public:
    explicit EpgDfxSnapshotTask(EpgDfxSnapshotTarget target);
    void OnTick(epg::TaskContext &context) override;

  private:
    EpgDfxSnapshotTarget m_target;
};

std::uint64_t EpgDfxNowMs();
void WriteEpgDfxSnapshotFile(const std::string &path, const std::string &json);

} // namespace smartdrone::core::application
