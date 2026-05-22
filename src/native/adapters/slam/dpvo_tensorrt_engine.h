#pragma once

#include <memory>
#include <string>

#include "core/application/config/app_args.h"
#include "core/ports/slam_engine.h"

namespace SmartDrone::Adapters::Slam {

struct DpvoTensorRtConfig {
    std::string repoPath;
    std::string patchEnginePath;
    std::string updateEnginePath;
    std::string settingsPath;
    int inputWidth{640};
    int inputHeight{400};
    int patchesPerFrame{48};
    int optimizationWindow{7};
};

class DpvoTensorRtEngine final : public Core::Ports::ISlamEngine {
  public:
    explicit DpvoTensorRtEngine(DpvoTensorRtConfig config);
    ~DpvoTensorRtEngine() override;

    bool Start() override;
    void Stop() override;
    Core::Ports::SlamOutput Process(const Core::Ports::SlamInputBatch &input, bool extractFeatures,
                                    bool extractPointCloud) override;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

DpvoTensorRtConfig MakeDpvoTensorRtConfig(const RuntimeConfig &runtime,
                                          const std::string &settingsPath = std::string{});

} // namespace SmartDrone::Adapters::Slam
