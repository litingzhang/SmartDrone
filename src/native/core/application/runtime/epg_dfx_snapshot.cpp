#include "core/application/runtime/epg_dfx_snapshot.h"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <utility>

namespace smartdrone::core::application {

EpgDfxSnapshotTask::EpgDfxSnapshotTask(EpgDfxSnapshotTarget target) : m_target(std::move(target)) {}

void EpgDfxSnapshotTask::OnTick(epg::TaskContext &context)
{
    (void)context;
    if (!m_target.graphRef || !m_target.graphRef->graph) {
        return;
    }
    WriteEpgDfxSnapshotFile(
        m_target.path,
        m_target.graphRef->graph->DfxSnapshotJson(m_target.graphName, EpgDfxNowMs()));
}

EPG_REGISTER_TASK_TYPE(EpgDfxSnapshotTask, "EpgDfxSnapshotTask")

std::uint64_t EpgDfxNowMs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

void WriteEpgDfxSnapshotFile(const std::string &path, const std::string &json)
{
    const std::string tempPath = path + ".tmp";
    {
        std::ofstream output(tempPath, std::ios::out | std::ios::trunc);
        if (!output) {
            return;
        }
        output << json;
    }
    (void)std::rename(tempPath.c_str(), path.c_str());
}

} // namespace smartdrone::core::application
