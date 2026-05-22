#include "core/application/runtime/epg_redeploy_coordinator.h"

#include <sstream>
#include <utility>

namespace SmartDrone::core::application {
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
    {
        std::lock_guard<std::mutex> lock(m_systemRedeployMutex);
        m_systemRequest = std::move(request);
        m_systemRedeployRequested.store(true, std::memory_order_release);
    }
    m_systemRedeployCv.notify_all();
}

bool EpgRedeployCoordinator::TakeSystemRedeployRequest()
{
    EpgRedeployRequest request;
    return TakeSystemRedeployRequest(request);
}

bool EpgRedeployCoordinator::TakeSystemRedeployRequest(
    EpgRedeployRequest &request)
{
    std::lock_guard<std::mutex> lock(m_systemRedeployMutex);
    if (!m_systemRedeployRequested.load(std::memory_order_acquire)) {
        return false;
    }
    m_systemRedeployRequested.store(false, std::memory_order_release);
    request = m_systemRequest;
    m_systemRequest = {};
    return true;
}

bool EpgRedeployCoordinator::SystemRedeployRequested() const
{
    return m_systemRedeployRequested.load(std::memory_order_acquire);
}

bool EpgRedeployCoordinator::WaitForSystemRedeploy(
    std::chrono::milliseconds timeout) const
{
    std::unique_lock<std::mutex> lock(m_systemRedeployMutex);
    return m_systemRedeployCv.wait_for(lock, timeout, [this]() {
        return SystemRedeployRequested();
    });
}

void EpgRedeployCoordinator::RequestSessionRedeploy()
{
    RequestSessionRedeploy({});
}

void EpgRedeployCoordinator::RequestSessionRedeploy(
    EpgRedeployRequest request)
{
    {
        std::lock_guard<std::mutex> lock(m_sessionRedeployMutex);
        m_sessionRequest = std::move(request);
        m_sessionRedeployRequested.store(true, std::memory_order_release);
    }
}

bool EpgRedeployCoordinator::TakeSessionRedeployRequest()
{
    EpgRedeployRequest request;
    return TakeSessionRedeployRequest(request);
}

bool EpgRedeployCoordinator::TakeSessionRedeployRequest(
    EpgRedeployRequest &request)
{
    std::lock_guard<std::mutex> lock(m_sessionRedeployMutex);
    if (!m_sessionRedeployRequested.load(std::memory_order_acquire)) {
        return false;
    }
    m_sessionRedeployRequested.store(false, std::memory_order_release);
    request = m_sessionRequest;
    m_sessionRequest = {};
    return true;
}

bool EpgRedeployCoordinator::SessionRedeployRequested() const
{
    return m_sessionRedeployRequested.load(std::memory_order_acquire);
}

} // namespace SmartDrone::core::application
