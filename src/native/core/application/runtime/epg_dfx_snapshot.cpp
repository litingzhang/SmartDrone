#include "core/application/runtime/epg_dfx_snapshot.h"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <utility>

#include "core/application/runtime/system_runtime_messages.h"

namespace SmartDrone::core::application {

EpgDfxSnapshotTask::EpgDfxSnapshotTask(EpgDfxSnapshotTarget target)
    : m_target(std::move(target))
{
}

void EpgDfxSnapshotTask::OnTick(Epg::TaskContext &context)
{
    DrainSystemRuntimePulse(context);
    if (!m_target.graphRef || !m_target.graphRef->graph) {
        return;
    }
    const std::uint64_t nowMs = EpgDfxNowMs();
    WriteEpgDfxSnapshotFile(m_target.snapshotPath,
                            m_target.graphRef->graph->DfxSnapshotJson(
                                m_target.graphName, nowMs));
    WriteEpgDfxSnapshotFile(m_target.profilePath,
                            m_target.graphRef->graph->ProfileJson(
                                m_target.graphName, nowMs,
                                m_target.topologyVersion,
                                m_target.taskCatalogJson));
    PushSystemRuntimePulse(context, m_pulseSequence);
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

} // namespace SmartDrone::core::application
