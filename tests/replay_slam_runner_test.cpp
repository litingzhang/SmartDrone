#include <gtest/gtest.h>

#include <filesystem>

#include "core/application/state/frame_timing_tracker.h"
#include "test_support/replay_dataset.h"
#include "test_support/replay_slam_runner.h"

namespace smartdrone::tests {
namespace {

class FakeSlamEngine final : public smartdrone::core::ports::ISlamEngine {
  public:
    bool Start() override
    {
        started = true;
        stopped = false;
        return true;
    }

    void Stop() override { stopped = true; }

    smartdrone::core::ports::SlamOutput Process(const smartdrone::core::ports::SlamInputBatch &input, bool, bool) override
    {
        smartdrone::core::ports::SlamOutput output{};
        output.frameId = input.frameId;
        output.captureTimestampNs = input.captureTimestampNs;
        output.poseValid = true;
        output.pose.valid = true;
        output.pose.x = static_cast<float>(input.captureTimestampNs * 1e-9);
        output.pose.y = static_cast<float>(input.imu.size());
        output.pose.z = static_cast<float>(input.stereo.left.gray.cols);
        output.pose.qw = 1.0f;
        output.trackingState = 1;
        output.mapId = 42;
        processedFrameIds.push_back(input.frameId);
        imuWindowSizes.push_back(input.imu.size());
        return output;
    }

    bool started{false};
    bool stopped{false};
    std::vector<uint64_t> processedFrameIds;
    std::vector<size_t> imuWindowSizes;
};

std::filesystem::path DataRoot()
{
    return std::filesystem::path(TESTS_SOURCE_DIR) / "data";
}

TEST(ReplaySlamRunnerTest, ProducesPoseSamplesFromReplayDataset)
{
    const ReplayDataset dataset = ReplayDataset::Load(DataRoot(), 18);
    ReplayCameraProvider camera(dataset);
    ReplayImuProvider imu(dataset);
    FakeSlamEngine slamEngine;
    ReplaySlamRunner runner(camera, imu, slamEngine, {.cameraFps = 60, .slamInputFps = 20, .useImu = true});
    smartdrone::core::application::FrameTimingTracker tracker(32);

    const auto outputs = runner.Run(5, &tracker);

    ASSERT_EQ(outputs.size(), 5u);
    EXPECT_TRUE(slamEngine.started);
    EXPECT_TRUE(slamEngine.stopped);
    EXPECT_EQ(slamEngine.processedFrameIds.size(), 5u);
    EXPECT_TRUE(outputs.front().poseValid);
    EXPECT_EQ(outputs.front().frameId, 1u);
    EXPECT_EQ(outputs.front().mapId, 42ul);
    EXPECT_GT(outputs.front().captureTimestampNs, 0);
    EXPECT_EQ(outputs.front().imuSampleCount, 0u);
    EXPECT_GT(outputs.back().imuSampleCount, 0u);

    smartdrone::core::application::FrameTimingRecord timing{};
    ASSERT_TRUE(tracker.Lookup(outputs.back().frameId, timing));
    EXPECT_EQ(timing.tCamNs, static_cast<uint64_t>(outputs.back().captureTimestampNs));
}

} // namespace
} // namespace smartdrone::tests
