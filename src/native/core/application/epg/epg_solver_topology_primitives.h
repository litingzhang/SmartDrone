#pragma once

#include "common/epg/epg_types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace SmartDrone::Core::Application::EpgSolverPrimitives {

struct BackpressureTopologyPenaltyInput {
    const Epg::GraphConfig *sourceGraphConfig{nullptr};
    const Epg::GraphProfileDiagnostics *diagnostics{nullptr};
    const Epg::TaskConfig *task{nullptr};
    const std::vector<Epg::PortId> *before{nullptr};
    const std::vector<Epg::PortId> *after{nullptr};
    std::uint64_t totalResourceWaitUs{0};
    bool replaceable{false};
};

std::vector<Epg::PortId> SortedUniquePorts(
    std::vector<Epg::PortId> ports);

bool ContainsPort(const std::vector<Epg::PortId> &ports, Epg::PortId port);

const Epg::QueueConfig *FindQueueConfig(const Epg::GraphConfig &graphConfig,
                                        const std::string &name);

const Epg::TaskConfig *FindTaskConfig(const Epg::GraphConfig &graphConfig,
                                      const std::string &name);

std::vector<Epg::PortId> CandidateBackpressurePorts(
    const Epg::GraphConfig &config,
    const Epg::GraphProfileDiagnostics *diagnostics,
    const Epg::TaskConfig &task,
    const std::vector<Epg::PortId> &before,
    bool replaceable);

std::uint64_t BackpressureTopologyPenalty(
    const BackpressureTopologyPenaltyInput &input);

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
