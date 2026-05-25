#include "core/application/session/calib/calib_save_pacing_port.h"

#include <gtest/gtest.h>

namespace {

using SmartDrone::Core::Application::CalibSavePacingPort;
using SmartDrone::Core::Application::CalibSavePacingPortConfig;
using SmartDrone::Core::Application::CalibSavePair;
using SmartDrone::Core::Application::CalibStereoFrame;

std::shared_ptr<CalibStereoFrame> MakeFrame(std::uint64_t leftNs,
                                            std::uint64_t rightNs)
{
    auto frame = std::make_shared<CalibStereoFrame>();
    frame->stereo.left.timestampNs = leftNs;
    frame->stereo.right.timestampNs = rightNs;
    return frame;
}

CalibSavePacingPortConfig MakePacingConfig()
{
    return {10, 30, "/tmp/smartdrone_calib/cam0",
            "/tmp/smartdrone_calib/cam1"};
}

TEST(CalibSavePacingPortTest, BuildsSavePairForEligibleFrame)
{
    CalibSavePacingPort port(MakePacingConfig());

    CalibSavePair pair{};
    ASSERT_TRUE(port.TryBuildSavePair(MakeFrame(1000, 3000), pair));

    EXPECT_EQ(pair.pairNs, 2000);
    EXPECT_EQ(pair.name, "2000.png");
    EXPECT_EQ(pair.fnL.filename().string(), "2000.png");
    EXPECT_EQ(pair.fnR.filename().string(), "2000.png");
    ASSERT_TRUE(pair.frame);
}

TEST(CalibSavePacingPortTest, DropsFrameBeforeNextSaveSlot)
{
    CalibSavePacingPort port(MakePacingConfig());

    CalibSavePair first{};
    ASSERT_TRUE(port.TryBuildSavePair(MakeFrame(1000, 3000), first));

    CalibSavePair dropped{};
    EXPECT_FALSE(port.TryBuildSavePair(MakeFrame(5000, 7000), dropped));
}

TEST(CalibSavePacingPortTest, AcceptsFrameAtNextSaveSlot)
{
    CalibSavePacingPort port(MakePacingConfig());

    CalibSavePair first{};
    ASSERT_TRUE(port.TryBuildSavePair(MakeFrame(1000, 3000), first));

    CalibSavePair second{};
    ASSERT_TRUE(port.TryBuildSavePair(MakeFrame(100002000, 100002000),
                                      second));
    EXPECT_EQ(second.pairNs, 100002000);
    EXPECT_EQ(second.name, "100002000.png");
}

} // namespace
