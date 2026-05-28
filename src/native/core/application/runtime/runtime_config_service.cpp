#include "core/application/runtime/runtime_config_service.h"

#include <atomic>
#include <memory>
#include <string>
#include <utility>

#include "core/application/config/orb_acceleration_config.h"
#include "core/application/runtime/runtime_config_application.h"
#include "core/application/runtime/runtime_config_message.h"
#include "core/application/runtime/runtime_config_projection.h"
#include "core/application/runtime/runtime_config_validation.h"
#include "core/application/runtime/runtime_config_value_applier.h"

namespace SmartDrone::Core::Application {

RuntimeConfigService::RuntimeConfigService(std::shared_ptr<const UnifiedConfig> &config,
                                           LiveRuntimeTuning &tuning,
                                           RestartFn requestRestart)
    : m_config(config), m_tuning(tuning),
      m_requestRestart(std::move(requestRestart))
{
}

bool RuntimeConfigService::UpdateRemoteConfig(RemoteRuntimeConfig remote,
                                              std::string *err)
{
    NormalizeRemoteRuntimeConfig(remote);
    if (!ValidateRemoteRuntimeConfig(remote, err)) {
        return false;
    }

    AppliedRuntimeConfig applied{};
    while (true) {
        std::shared_ptr<const UnifiedConfig> current = LoadConfig();
        UnifiedConfig nextConfig = current ? *current : UnifiedConfig{};
        applied = ApplyRemoteRuntimeConfig(nextConfig, remote);
        auto next = std::make_shared<const UnifiedConfig>(std::move(nextConfig));
        if (ReplaceConfig(current, std::move(next))) {
            break;
        }
    }

    if (remote.slamBackend == SlamBackend::OrbSlam3) {
        ApplyOrbAccelerationEnvironment(remote.orbAcceleration);
    }

    SyncRuntimeTuning(m_tuning, remote, applied.tbc);

    if (applied.restartNeeded && m_requestRestart) {
        m_requestRestart();
    }
    return true;
}

CommandResult
RuntimeConfigService::ApplyConfig(const ConfigUpdate &update,
                                  const UnifiedConfig &currentConfig)
{
    RemoteRuntimeConfig remote = BuildRemoteConfig(currentConfig);

    for (const auto &[key, value] : update.values) {
        const CommandResult result = ApplyConfigValue(key, value, remote);
        if (!result.ok) {
            return result;
        }
    }

    std::string err;
    if (!UpdateRemoteConfig(remote, &err)) {
        return {false, err.empty() ? "runtime cfg failed" : err};
    }

    return {true, BuildRuntimeConfigMessage(remote, currentConfig)};
}

std::shared_ptr<const UnifiedConfig> RuntimeConfigService::LoadConfig() const
{
    return std::atomic_load_explicit(&m_config, std::memory_order_acquire);
}

bool RuntimeConfigService::ReplaceConfig(
    std::shared_ptr<const UnifiedConfig> &expected,
    std::shared_ptr<const UnifiedConfig> next)
{
    return std::atomic_compare_exchange_weak_explicit(&m_config, &expected, std::move(next),
                                                      std::memory_order_acq_rel,
                                                      std::memory_order_acquire);
}

} // namespace SmartDrone::Core::Application
