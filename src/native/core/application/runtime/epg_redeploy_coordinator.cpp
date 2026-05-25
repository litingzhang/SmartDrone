#include "core/application/runtime/epg_redeploy_coordinator.h"

#include <sstream>
#include <utility>

namespace SmartDrone::Core::Application {
namespace {

void AppendTextField(std::ostringstream &out,
                     const char *name,
                     const std::string &value,
                     bool &first)
{
    if (value.empty()) {
        return;
    }
    if (!first) {
        out << " ";
    }
    out << name << "=" << value;
    first = false;
}

void AppendNumberField(std::ostringstream &out,
                       const char *name,
                       std::uint64_t value,
                       bool &first)
{
    if (value == 0) {
        return;
    }
    if (!first) {
        out << " ";
    }
    out << name << "=" << value;
    first = false;
}

} // namespace

std::string DescribeEpgRedeployRequest(
    const EpgRedeployRequest &request)
{
    std::ostringstream out;
    bool first = true;
    AppendTextField(out, "graph", request.graphName, first);
    AppendTextField(out, "reason", request.reason, first);
    AppendTextField(out, "topology", request.topologyVersion, first);
    AppendTextField(out, "source", request.sourceProfile, first);
    AppendTextField(out, "profile", request.sourceProfilePath, first);
    AppendNumberField(out, "source_ts_ms", request.sourceTimestampMs, first);
    AppendNumberField(out, "generated_ms", request.generatedAtMs, first);
    AppendTextField(out, "solver", request.solverVersion, first);
    AppendTextField(out, "optimized", request.optimizedConfigPath, first);
    AppendTextField(out, "report", request.solverReportPath, first);
    return out.str();
}

void EpgRedeployCoordinator::RequestSystemRedeploy()
{
    RequestSystemRedeploy({});
}

void EpgRedeployCoordinator::RequestSystemRedeploy(
    EpgRedeployRequest request)
{
    auto snapshot =
        std::make_shared<const EpgRedeployRequest>(std::move(request));
    std::atomic_store_explicit(&m_systemRequest, std::move(snapshot),
                               std::memory_order_release);
}

bool EpgRedeployCoordinator::TakeSystemRedeployRequest()
{
    EpgRedeployRequest request;
    return TakeSystemRedeployRequest(request);
}

bool EpgRedeployCoordinator::TakeSystemRedeployRequest(
    EpgRedeployRequest &request)
{
    auto snapshot = SystemRequest();
    if (!snapshot) {
        return false;
    }
    request = *snapshot;
    std::shared_ptr<const EpgRedeployRequest> expected = snapshot;
    (void)std::atomic_compare_exchange_strong_explicit(
        &m_systemRequest,
        &expected,
        std::shared_ptr<const EpgRedeployRequest>{},
        std::memory_order_acq_rel,
        std::memory_order_acquire);
    return true;
}

bool EpgRedeployCoordinator::SystemRedeployRequested() const
{
    return static_cast<bool>(SystemRequest());
}

void EpgRedeployCoordinator::RequestSessionRedeploy()
{
    RequestSessionRedeploy({});
}

void EpgRedeployCoordinator::RequestSessionRedeploy(
    EpgRedeployRequest request)
{
    auto snapshot =
        std::make_shared<const EpgRedeployRequest>(std::move(request));
    std::atomic_store_explicit(&m_sessionRequest, std::move(snapshot),
                               std::memory_order_release);
}

bool EpgRedeployCoordinator::TakeSessionRedeployRequest()
{
    EpgRedeployRequest request;
    return TakeSessionRedeployRequest(request);
}

bool EpgRedeployCoordinator::TakeSessionRedeployRequest(
    EpgRedeployRequest &request)
{
    auto snapshot = SessionRequest();
    if (!snapshot) {
        return false;
    }
    request = *snapshot;
    std::shared_ptr<const EpgRedeployRequest> expected = snapshot;
    (void)std::atomic_compare_exchange_strong_explicit(
        &m_sessionRequest,
        &expected,
        std::shared_ptr<const EpgRedeployRequest>{},
        std::memory_order_acq_rel,
        std::memory_order_acquire);
    return true;
}

bool EpgRedeployCoordinator::SessionRedeployRequested() const
{
    return static_cast<bool>(SessionRequest());
}

std::shared_ptr<const EpgRedeployRequest>
EpgRedeployCoordinator::SystemRequest() const
{
    return std::atomic_load_explicit(&m_systemRequest,
                                     std::memory_order_acquire);
}

std::shared_ptr<const EpgRedeployRequest>
EpgRedeployCoordinator::SessionRequest() const
{
    return std::atomic_load_explicit(&m_sessionRequest,
                                     std::memory_order_acquire);
}

} // namespace SmartDrone::Core::Application
