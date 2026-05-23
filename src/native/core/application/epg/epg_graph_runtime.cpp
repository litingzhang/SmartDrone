#include "core/application/epg/epg_graph_runtime.h"

#include <stdexcept>
#include <utility>

namespace SmartDrone::Core::Application {

std::unique_ptr<Epg::EventPipelineGraph> StartEpgGraph(
    EpgGraphStartRequest request)
{
    if (!request.graphRef) {
        throw std::runtime_error("EPG graph reference is required");
    }

    Epg::Registry registry;
    RegisterEpgTypes(registry, request.domain, request.factoryResolver);
    Epg::GraphConfig graphConfig =
        CompileEpgConfig(request.domain, registry);
    if (request.customizeConfig) {
        request.customizeConfig(graphConfig);
    }

    auto graph = std::make_unique<Epg::EventPipelineGraph>(std::move(registry));
    graph->Configure(graphConfig);
    request.graphRef->graph = graph.get();
    graph->Start();
    return graph;
}

} // namespace SmartDrone::Core::Application
