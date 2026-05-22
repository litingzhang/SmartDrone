#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "common/epg/epg.h"

namespace SmartDrone::Core::Application {

struct EpgGraphRef {
    Epg::EventPipelineGraph *graph{nullptr};
};

struct EpgDfxSnapshotTarget {
    std::shared_ptr<EpgGraphRef> graphRef;
    std::string graphName;
    std::string topologyVersion;
    std::string taskCatalogJson;
    std::string snapshotPath;
    std::string profilePath;
};

class EpgDfxSnapshotTask final : public Epg::ITask {
  public:
    explicit EpgDfxSnapshotTask(EpgDfxSnapshotTarget target);
    void OnTick(Epg::TaskContext &context) override;

  private:
    EpgDfxSnapshotTarget m_target;
    std::uint64_t m_pulseSequence{0};
};

std::uint64_t EpgDfxNowMs();
void WriteEpgDfxSnapshotFile(const std::string &path, const std::string &json);

} // namespace SmartDrone::Core::Application
