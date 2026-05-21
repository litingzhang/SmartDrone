#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>

namespace smartdrone::core::application {

struct EpgRedeployRequest {
    std::string graphName;
    std::string reason;
};

class EpgRedeployCoordinator final {
  public:
    void RequestSystemRedeploy();
    void RequestSystemRedeploy(EpgRedeployRequest request);
    bool TakeSystemRedeployRequest();
    bool TakeSystemRedeployRequest(EpgRedeployRequest &request);
    bool SystemRedeployRequested() const;
    bool WaitForSystemRedeploy(std::chrono::milliseconds timeout) const;

    void RequestSessionRedeploy();
    void RequestSessionRedeploy(EpgRedeployRequest request);
    bool TakeSessionRedeployRequest();
    bool TakeSessionRedeployRequest(EpgRedeployRequest &request);
    bool SessionRedeployRequested() const;

  private:
    std::atomic<bool> m_systemRedeployRequested{false};
    std::atomic<bool> m_sessionRedeployRequested{false};
    mutable std::mutex m_systemRedeployMutex;
    mutable std::condition_variable m_systemRedeployCv;
    mutable std::mutex m_sessionRedeployMutex;
    EpgRedeployRequest m_systemRequest;
    EpgRedeployRequest m_sessionRequest;
};

} // namespace smartdrone::core::application
