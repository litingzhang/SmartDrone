#include "core/application/state/frame_timing_tracker.h"

#include <gtest/gtest.h>

namespace {

using SmartDrone::Core::Application::FrameTimingRecord;
using SmartDrone::Core::Application::FrameTimingTracker;
using SmartDrone::Core::Application::FrameCaptureTiming;

TEST(FrameTimingTrackerTest, RecordsFrameStageTimestamps)
{
    FrameTimingTracker tracker(4);

    tracker.UpsertCapture(10, FrameCaptureTiming{1000, 1050, 1080, 1100});
    tracker.MarkSlamIn(10, 1200);
    tracker.MarkSlamOut(10, 1800);
    tracker.MarkMavTx(10, 2200);

    FrameTimingRecord record{};
    ASSERT_TRUE(tracker.Lookup(10, record));
    EXPECT_EQ(record.frameId, 10U);
    EXPECT_EQ(record.tCamNs, 1000U);
    EXPECT_EQ(record.tCaptureMonotonicNs, 1050U);
    EXPECT_EQ(record.tLeftArrivalNs, 1080U);
    EXPECT_EQ(record.tRightArrivalNs, 1100U);
    EXPECT_EQ(record.tPairReadyNs, 1100U);
    EXPECT_EQ(record.tCbNs, 1100U);
    EXPECT_EQ(record.tSlamInNs, 1200U);
    EXPECT_EQ(record.tSlamOutNs, 1800U);
    EXPECT_EQ(record.tMavTxNs, 2200U);
}

TEST(FrameTimingTrackerTest, IgnoresStageTimestampBeforeCapture)
{
    FrameTimingTracker tracker(4);

    tracker.MarkSlamIn(1, 1200);
    tracker.MarkSlamOut(1, 1800);
    tracker.MarkMavTx(1, 2200);

    FrameTimingRecord record{};
    EXPECT_FALSE(tracker.Lookup(1, record));
}

TEST(FrameTimingTrackerTest, ReusesFixedSlotsByFrameId)
{
    FrameTimingTracker tracker(2);

    tracker.UpsertCapture(1, 1000, 1050, 1100);
    tracker.UpsertCapture(3, 3000, 3050, 3100);

    FrameTimingRecord record{};
    EXPECT_FALSE(tracker.Lookup(1, record));
    ASSERT_TRUE(tracker.Lookup(3, record));
    EXPECT_EQ(record.frameId, 3U);
    EXPECT_EQ(record.tCamNs, 3000U);
    EXPECT_EQ(record.tCbNs, 3100U);
}

TEST(FrameTimingTrackerTest, IgnoresStaleStageWriteAfterSlotReuse)
{
    FrameTimingTracker tracker(2);

    tracker.UpsertCapture(1, 1000, 1050, 1100);
    tracker.UpsertCapture(3, 3000, 3050, 3100);
    tracker.MarkSlamOut(1, 1800);

    FrameTimingRecord record{};
    ASSERT_TRUE(tracker.Lookup(3, record));
    EXPECT_EQ(record.frameId, 3U);
    EXPECT_EQ(record.tSlamOutNs, 0U);
}

TEST(FrameTimingTrackerTest, SupportsFrameIdZero)
{
    FrameTimingTracker tracker(2);

    FrameTimingRecord record{};
    EXPECT_FALSE(tracker.Lookup(0, record));

    tracker.UpsertCapture(0, 1000, 1050, 1100);

    ASSERT_TRUE(tracker.Lookup(0, record));
    EXPECT_EQ(record.frameId, 0U);
    EXPECT_EQ(record.tCamNs, 1000U);
    EXPECT_EQ(record.tCbNs, 1100U);
}

TEST(FrameTimingTrackerTest, LegacyCaptureUsesCallbackForBothEyes)
{
    FrameTimingTracker tracker(2);

    tracker.UpsertCapture(1, 1000, 1050, 1100);

    FrameTimingRecord record{};
    ASSERT_TRUE(tracker.Lookup(1, record));
    EXPECT_EQ(record.tLeftArrivalNs, 1100U);
    EXPECT_EQ(record.tRightArrivalNs, 1100U);
    EXPECT_EQ(record.tPairReadyNs, 1100U);
}

} // namespace
