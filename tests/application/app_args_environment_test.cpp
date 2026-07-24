#include "core/application/config/app_args.h"

#include <cstdlib>
#include <optional>
#include <string>

#include <gtest/gtest.h>

namespace {

class ScopedEnvironment final {
  public:
    ScopedEnvironment(const char *name, const char *value)
        : m_name(name)
    {
        const char *previous = std::getenv(name);
        if (previous != nullptr) {
            m_previous = previous;
        }
        setenv(name, value, 1);
    }

    ~ScopedEnvironment()
    {
        if (m_previous) {
            setenv(m_name.c_str(), m_previous->c_str(), 1);
        } else {
            unsetenv(m_name.c_str());
        }
    }

  private:
    std::string m_name;
    std::optional<std::string> m_previous;
};

TEST(AppArgsEnvironmentTest, LoadsSimulationDiagnosticsAndPoseDefaults)
{
    ScopedEnvironment diagnostics("SMART_DRONE_JSON_DIAGNOSTICS", "1");
    ScopedEnvironment poseOutput("SMART_DRONE_PX4_POSE_OUTPUT_MODE", "none");
    char program[] = "smart_drone";
    char *arguments[] = {program};

    const AppConfig config = ParseAppConfig(1, arguments);

    EXPECT_TRUE(config.runtime.jsonDiagnostics);
    EXPECT_EQ(config.runtime.px4PoseOutputMode,
              SmartDrone::Core::Domain::Px4PoseOutputMode::None);
}

} // namespace
