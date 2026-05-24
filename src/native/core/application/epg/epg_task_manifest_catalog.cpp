#include "core/application/epg/epg_task_manifest.h"

#include "core/application/epg/epg_solver_primitives.h"

#include <sstream>
#include <string>
#include <vector>

namespace SmartDrone::Core::Application {
namespace {

namespace Solver = EpgSolverPrimitives;

void WriteJsonStringArray(std::ostringstream &out,
                          const std::vector<std::string> &values)
{
    out << "[";
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index != 0) {
            out << ",";
        }
        out << "\"" << Solver::JsonEscape(values[index]) << "\"";
    }
    out << "]";
}

void WriteCatalogEntryJson(std::ostringstream &out,
                           const EpgTaskCatalogEntry &entry)
{
    out << "{\"taskType\":\"" << Solver::JsonEscape(entry.taskType) << "\",";
    out << "\"role\":\"" << Solver::JsonEscape(entry.role) << "\",";
    out << "\"resource\":\"" << Solver::JsonEscape(entry.resource) << "\",";
    if (!entry.resourceAlternates.empty()) {
        out << "\"resourceAlternates\":";
        WriteJsonStringArray(out, entry.resourceAlternates);
        out << ",";
    }
    out << "\"budgetUs\":" << entry.budgetUs << ",";
    out << "\"deadlineUs\":" << entry.deadlineUs << ",";
    out << "\"replaceable\":" << (entry.replaceable ? "true" : "false");
    if (entry.preserveAccuracy) {
        out << ",\"preserveAccuracy\":true";
    }
    out << "}";
}

} // namespace

std::vector<std::string> EpgTaskCatalogTypes(
    const EpgTaskManifest &manifest)
{
    std::vector<std::string> taskTypes;
    taskTypes.reserve(manifest.catalog.size());
    for (const auto &entry : manifest.catalog) {
        taskTypes.push_back(entry.taskType);
    }
    return taskTypes;
}

std::string EpgTaskCatalogJson(const EpgTaskManifest &manifest)
{
    std::ostringstream out;
    out << "[";
    for (std::size_t index = 0; index < manifest.catalog.size(); ++index) {
        if (index != 0) {
            out << ",";
        }
        WriteCatalogEntryJson(out, manifest.catalog[index]);
    }
    out << "]";
    return out.str();
}

} // namespace SmartDrone::Core::Application
