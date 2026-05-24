#include "core/application/epg/epg_task_manifest_runtime_tuning.h"

#include "core/application/epg/epg_task_manifest_internal.h"

#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace SmartDrone::Core::Application {
namespace {

using EpgTaskManifestInternal::TaskGraphLabel;

bool RuntimeTuningEnabled(const EpgTaskRuntimeTuningEntry &entry)
{
    return entry.interval || entry.realtime || entry.priority;
}

const EpgTaskRuntimeTuningEntry *FindRuntimeTuning(
    const std::vector<EpgTaskRuntimeTuningEntry> &entries,
    const std::string &taskName)
{
    for (const auto &entry : entries) {
        if (entry.taskName == taskName) {
            return &entry;
        }
    }
    return nullptr;
}

bool GraphUsesTaskName(const Epg::GraphConfig &graphConfig,
                       const std::string &taskName)
{
    for (const auto &task : graphConfig.tasks) {
        if (task.name == taskName) {
            return true;
        }
    }
    return false;
}

void ValidateRuntimeTuningEntry(const EpgTaskManifest &manifest,
                                const EpgTaskRuntimeTuningEntry &entry)
{
    if (entry.taskName.empty() || !RuntimeTuningEnabled(entry)) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " runtime tuning metadata is incomplete");
    }
}

void ValidateRuntimeTuningRequestAllowed(
    const EpgTaskManifest &manifest,
    const EpgTaskRuntimeTuningEntry &allowed,
    const EpgTaskRuntimeTuningEntry &requested)
{
    if ((requested.interval && !allowed.interval) ||
        (requested.realtime && !allowed.realtime) ||
        (requested.priority && !allowed.priority)) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " runtime tuning is not allowed: " +
                                 requested.taskName);
    }
}

void ValidateRuntimeTuningTaskUnique(const EpgTaskManifest &manifest,
                                     std::set<std::string> &taskNames,
                                     const std::string &taskName)
{
    if (taskNames.insert(taskName).second) {
        return;
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " duplicates runtime tuning task: " + taskName);
}

const EpgTaskRuntimeTuningEntry &RequireAllowedRuntimeTuning(
    const EpgTaskManifest &manifest,
    const EpgTaskRuntimeTuningEntry &requested)
{
    const auto *allowed =
        FindRuntimeTuning(manifest.runtimeTuning, requested.taskName);
    if (allowed) {
        return *allowed;
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " runtime tuning task is not declared: " +
                             requested.taskName);
}

void ValidateRuntimeTuningGraphTask(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig &graphConfig,
    const std::string &taskName)
{
    if (GraphUsesTaskName(graphConfig, taskName)) {
        return;
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " runtime tuning task is missing: " + taskName);
}

} // namespace

void ValidateEpgManifestRuntimeTuning(const EpgTaskManifest &manifest)
{
    std::set<std::string> taskNames;
    for (const auto &entry : manifest.runtimeTuning) {
        ValidateRuntimeTuningEntry(manifest, entry);
        ValidateRuntimeTuningTaskUnique(manifest, taskNames, entry.taskName);
    }
}

void ValidateEpgGraphRuntimeTuningDeclared(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig &graphConfig)
{
    for (const auto &entry : manifest.runtimeTuning) {
        ValidateRuntimeTuningGraphTask(manifest, graphConfig, entry.taskName);
    }
}

void ValidateEpgTaskRuntimeTuning(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig &graphConfig,
    const std::vector<EpgTaskRuntimeTuningEntry> &requestedTuning)
{
    ValidateEpgManifestRuntimeTuning(manifest);
    for (const auto &requested : requestedTuning) {
        ValidateRuntimeTuningEntry(manifest, requested);
        const auto &allowed =
            RequireAllowedRuntimeTuning(manifest, requested);
        ValidateRuntimeTuningRequestAllowed(manifest, allowed, requested);
        ValidateRuntimeTuningGraphTask(
            manifest, graphConfig, requested.taskName);
    }
}

} // namespace SmartDrone::Core::Application
