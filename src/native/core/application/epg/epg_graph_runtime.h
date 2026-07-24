#pragma once

#include <functional>
#include <memory>

#include "core/application/epg/epg_registry.h"

namespace SmartDrone::Core::Application {

struct EpgGraphRef {
    Epg::EventPipelineGraph *graph{nullptr};
};

using EpgGraphConfigCustomizer = std::function<void(Epg::GraphConfig &)>;
using EpgGraphBeforeStart =
    std::function<void(Epg::EventPipelineGraph &)>;

struct EpgGraphStartRequest {
    EpgDomain domain;
    EpgTaskFactoryResolver factoryResolver;
    std::shared_ptr<EpgGraphRef> graphRef;
    EpgGraphConfigCustomizer customizeConfig;
    EpgGraphBeforeStart beforeStart;
};

std::unique_ptr<Epg::EventPipelineGraph> StartEpgGraph(
    EpgGraphStartRequest request);

} // namespace SmartDrone::Core::Application
