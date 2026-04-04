#include <gtest/gtest.h>

#include <filesystem>

#include "core/application/state/frame_timing_tracker.h"
#include "core/application/state/perception_pipeline.h"
#include "test_support/replay_dataset.h"

namespace smartdrone::tests {
namespace {

std::filesystem::path DataRoot()
{
    return std::filesystem::path(TESTS_SOURCE_DIR) / "data";
}

TEST(ReplayDatasetTest, LoadsDatasetAndKeepsStereoCountsAligned)
{
    const ReplayDataset dataset = ReplayDataset::Load(DataRoot(), 16);

    ASSERT_FALSE(dataset.Empty());
    EXPECT_EQ(dataset.LeftFrames().size(), dataset.RightFrames().size());
    EXPECT_EQ(dataset.LeftFrames().size(), 16u);
    EXPECT_FALSE(dataset.ImuSamples().empty());
    EXPECT_LT(dataset.LeftFrames().front().timestampNs, dataset.LeftFrames().back().timestampNs);
}

TEST(ReplayDatasetTest, ReplaysStereoFramesThroughPerceptionPipeline)
{
    const ReplayDataset dataset = ReplayDataset::Load(DataRoot(), 8);
    ReplayCameraProvider camera(dataset);
    smartdrone::core::application::PerceptionPipeline pipeline({60, true});
    smartdrone::core::application::FrameTimingTracker tracker(32);
    smartdrone::core::application::StereoBatch batch{};
    smartdrone::core::application::FrameTimingRecord timing{};

    ASSERT_TRUE(camera.Start());
    ASSERT_EQ(pipeline.AcquireNextStereoBatch(camera, 20, 1000, batch, &tracker),
              smartdrone::core::application::StereoAcquireStatus::Ok);
    EXPECT_GT(batch.frameId, 0u);
    EXPECT_GT(batch.captureTimestampNs, 0);
    EXPECT_FALSE(batch.stereo.left.gray.empty());
    EXPECT_FALSE(batch.stereo.right.gray.empty());
    ASSERT_TRUE(tracker.Lookup(batch.frameId, timing));
    EXPECT_EQ(timing.tCamNs, static_cast<uint64_t>(batch.captureTimestampNs));
}

TEST(ReplayDatasetTest, ReturnsImuWindowForFrameRange)
{
    const ReplayDataset dataset = ReplayDataset::Load(DataRoot(), 8);
    ReplayImuProvider imu(dataset);

    ASSERT_TRUE(imu.Start());
    const int64_t fromNs = static_cast<int64_t>(dataset.LeftFrames()[0].timestampNs);
    const int64_t toNs = static_cast<int64_t>(dataset.LeftFrames()[3].timestampNs);
    const auto samples = imu.PopWindow(fromNs, toNs);

    ASSERT_FALSE(samples.empty());
    EXPECT_GE(samples.front().timestampNs, fromNs);
    EXPECT_LE(samples.back().timestampNs, toNs);
}

} // namespace
} // namespace smartdrone::tests
