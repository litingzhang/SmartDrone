#pragma once

#include <memory>
#include <string>

#include "core/domain/feature_frontend.h"
#include "core/ports/visual_feature_frontend.h"

namespace SmartDrone::Adapters::Slam {

struct VisualFeatureFrontendRuntimeConfig {
    std::string repoPath;
    std::string device{"auto"};
    int topK{1024};
    int maxPoints{512};
    int inputMaxWidth{640};
    int inputMaxHeight{409};
};

class IManagedVisualFeatureFrontend
    : public Core::Ports::IVisualFeatureFrontend {
  public:
    ~IManagedVisualFeatureFrontend() override = default;

    virtual bool Start(const VisualFeatureFrontendRuntimeConfig &config,
                       std::string *err) = 0;
    virtual void Stop() = 0;
};

using VisualFeatureFrontendClientFactory =
    std::unique_ptr<IManagedVisualFeatureFrontend> (*)();

void RegisterVisualFeatureFrontendClient(
    FeatureFrontend frontend, VisualFeatureFrontendClientFactory factory);

class VisualFeatureFrontendClientRegistrar {
  public:
    VisualFeatureFrontendClientRegistrar(
        FeatureFrontend frontend, VisualFeatureFrontendClientFactory factory);
};

std::unique_ptr<IManagedVisualFeatureFrontend>
CreateVisualFeatureFrontendClient(FeatureFrontend frontend);
bool VisualFeatureFrontendClientEnabled(FeatureFrontend frontend);
std::string ResolveVisualFeatureFrontendRepo(FeatureFrontend frontend,
                                             const std::string &configuredRepo);
void ConfigureVisualFeatureFrontendDefaults(
    FeatureFrontend frontend, const VisualFeatureFrontendRuntimeConfig &config);

} // namespace SmartDrone::Adapters::Slam
