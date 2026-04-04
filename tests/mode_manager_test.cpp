#include <gtest/gtest.h>

#include "core/application/runtime/mode_manager.h"

namespace smartdrone::core::application {
namespace {

TEST(ModeManagerTest, RequestModeMarksRestartAndRequiresStopForActiveSession)
{
    ModeManager manager;

    manager.MarkSessionLaunching(ModeManager::RuntimeMode::Slam);
    manager.RequestMode(ModeManager::RuntimeMode::Calib);

    EXPECT_EQ(manager.DesiredMode(), ModeManager::RuntimeMode::Calib);
    EXPECT_EQ(manager.ActiveMode(), ModeManager::RuntimeMode::Slam);
    EXPECT_TRUE(manager.RestartRequested());
    EXPECT_TRUE(manager.ShouldStopActiveSession());
}

TEST(ModeManagerTest, MarkSessionJoinedResetsToIdleAndClearsRestart)
{
    ModeManager manager;

    manager.MarkSessionLaunching(ModeManager::RuntimeMode::Slam);
    manager.RequestRestart();
    manager.MarkSessionJoined();

    EXPECT_EQ(manager.ActiveMode(), ModeManager::RuntimeMode::Idle);
    EXPECT_FALSE(manager.RestartRequested());
    EXPECT_FALSE(manager.ShouldStopActiveSession());
}

} // namespace
} // namespace smartdrone::core::application
