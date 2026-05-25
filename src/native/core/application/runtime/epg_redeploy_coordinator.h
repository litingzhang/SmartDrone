#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace SmartDrone::Core::Application {

struct EpgRedeployRequest {
    std::string graphName;
    std::string reason;
    std::string topologyVersion;
    std::string sourceProfile;
    std::string sourceProfilePath;
    std::uint64_t sourceTimestampMs{0};
    std::uint64_t generatedAtMs{0};
    std::string solverVersion;
    std::string optimizedConfigPath;
    std::string solverReportPath;
};

std::string DescribeEpgRedeployRequest(
    const EpgRedeployRequest &request);

class EpgRedeployCoordinator final {
  public:
    void RequestSystemRedeploy();
    void RequestSystemRedeploy(EpgRedeployRequest request);
    bool TakeSystemRedeployRequest();
    bool TakeSystemRedeployRequest(EpgRedeployRequest &request);
    bool SystemRedeployRequested() const;

    void RequestSessionRedeploy();
    void RequestSessionRedeploy(EpgRedeployRequest request);
    bool TakeSessionRedeployRequest();
    bool TakeSessionRedeployRequest(EpgRedeployRequest &request);
    bool SessionRedeployRequested() const;

  private:
    std::shared_ptr<const EpgRedeployRequest> SystemRequest() const;
    std::shared_ptr<const EpgRedeployRequest> SessionRequest() const;

    std::shared_ptr<const EpgRedeployRequest> m_systemRequest;
    std::shared_ptr<const EpgRedeployRequest> m_sessionRequest;
};

} // namespace SmartDrone::Core::Application
