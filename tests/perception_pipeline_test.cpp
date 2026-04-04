#include <gtest/gtest.h>

#include <deque>

#include <opencv2/core/mat.hpp>

#include "core/application/state/perception_pipeline.h"

namespace smartdrone::core::application {
namespace {

class FakeCameraProvider final : public ports::ICameraProvider {
  public:
    bool Start() override { return true; }
    void Stop() override {}

    bool GrabStereo(ports::StereoFrame &out, int timeoutMs, bool preferLatest, uint64_t minTimestampNs) override
    {
        lastTimeoutMs = timeoutMs;
        lastPreferLatest = preferLatest;
        lastMinTimestampNs = minTimestampNs;
        if (frames.empty()) {
            return false;
        }
        out = frames.front();
        frames.pop_front();
        return true;
    }

    ports::CameraHealth GetHealth() const override { return health; }

    mutable int lastTimeoutMs{0};
    mutable bool lastPreferLatest{false};
    mutable uint64_t lastMinTimestampNs{0};
    ports::CameraHealth health{};
    std::deque<ports::StereoFrame> frames;
};

ports::StereoFrame MakeStereoFrame(uint64_t leftTs, uint64_t rightTs, int64_t leftArrive, int64_t rightArrive)
{
    ports::StereoFrame frame{};
    frame.left.timestampNs = leftTs;
    frame.right.timestampNs = rightTs;
    frame.left.arriveNs = leftArrive;
    frame.right.arriveNs = rightArrive;
    frame.left.gray = cv::Mat(2, 2, CV_8UC1, cv::Scalar(0));
    frame.right.gray = cv::Mat(2, 2, CV_8UC1, cv::Scalar(0));
    return frame;
}

TEST(PerceptionPipelineTest, SeparatesCaptureTimeFromLogicalMonotonicTime)
{
    FakeCameraProvider camera;
    PerceptionPipeline pipeline({60, true});
    FrameTimingTracker tracker(8);
    StereoBatch first{};
    StereoBatch second{};
    FrameTimingRecord timing{};

    const ports::StereoFrame firstFrame =
        MakeStereoFrame(1'000'000'000ULL, 1'000'000'400ULL, 1'000'100'000LL, 1'000'100'400LL);
    const ports::StereoFrame secondFrame =
        MakeStereoFrame(999'000'000ULL, 999'000'400ULL, 999'100'000LL, 999'100'400LL);
    camera.frames.push_back(firstFrame);
    camera.frames.push_back(secondFrame);

    ASSERT_EQ(pipeline.AcquireNextStereoBatch(camera, 60, 1000, first, &tracker), StereoAcquireStatus::Ok);
    ASSERT_EQ(pipeline.AcquireNextStereoBatch(camera, 60, 1000, second, &tracker), StereoAcquireStatus::Ok);

    EXPECT_EQ(first.captureTimestampNs, 1'000'000'200LL);
    EXPECT_EQ(first.logicalFrameTimestampNs, 1'000'000'200LL);
    EXPECT_EQ(second.captureTimestampNs, 999'000'200LL);
    EXPECT_GT(second.logicalFrameTimestampNs, first.logicalFrameTimestampNs);
    ASSERT_TRUE(tracker.Lookup(second.frameId, timing));
    EXPECT_EQ(timing.tCamNs, static_cast<uint64_t>(second.captureTimestampNs));
    EXPECT_EQ(timing.tCbNs, static_cast<uint64_t>((secondFrame.left.arriveNs + secondFrame.right.arriveNs) / 2LL));
}

TEST(PerceptionPipelineTest, ReportsCameraUnhealthyWhenGrabFailsAndHealthIsBad)
{
    FakeCameraProvider camera;
    PerceptionPipeline pipeline({60, true});
    StereoBatch batch{};
    camera.health = ports::CameraHealth{false, 3};

    EXPECT_EQ(pipeline.AcquireNextStereoBatch(camera, 20, 1000, batch, nullptr),
              StereoAcquireStatus::CameraUnhealthy);
}

} // namespace
} // namespace smartdrone::core::application
