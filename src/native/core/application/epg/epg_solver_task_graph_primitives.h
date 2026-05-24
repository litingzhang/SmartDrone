#pragma once

#include "common/epg/epg_types.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace SmartDrone::Core::Application::EpgSolverPrimitives {

struct TaskDependencyNode {
    std::string name;
    std::size_t sourceIndex{0};
    std::uint64_t durationMs{0};
    std::vector<std::size_t> predecessors;
    std::vector<std::size_t> successors;
};

struct TaskDependencyGraph {
    std::vector<TaskDependencyNode> nodes;
};

struct TaskTopologyScheduleEntry {
    std::string name;
    std::size_t sourceIndex{0};
    std::uint64_t level{0};
    std::uint64_t phaseOffsetMs{0};
    std::uint64_t durationMs{0};
};

struct TaskTopologySchedule {
    std::vector<TaskTopologyScheduleEntry> entries;
};

struct CpuBindingScheduleEntry {
    std::string name;
    std::size_t sourceIndex{0};
    int cpuAffinity{-1};
    std::uint64_t startMs{0};
    std::uint64_t finishMs{0};
};

struct CpuBindingSchedule {
    std::vector<CpuBindingScheduleEntry> entries;
    std::uint64_t makespanMs{0};
    bool exact{false};
};

TaskDependencyGraph BuildTaskDependencyGraph(
    const Epg::GraphConfig &graphConfig,
    const std::map<std::string, std::uint64_t> &taskDurationsMs);

TaskTopologySchedule BuildTaskTopologySchedule(
    const Epg::GraphConfig &graphConfig,
    const std::map<std::string, std::uint64_t> &taskDurationsMs);

CpuBindingSchedule BuildCpuBindingSchedule(
    const Epg::GraphConfig &graphConfig,
    const std::map<std::string, std::uint64_t> &taskDurationsMs,
    std::size_t cpuCount,
    std::uint64_t maxExactStates);

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
