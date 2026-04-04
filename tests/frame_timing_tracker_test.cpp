#include <gtest/gtest.h>

#include "core/application/state/frame_timing_tracker.h"

namespace smartdrone::core::application {
namespace {

TEST(FrameTimingTrackerTest, StoresAndLooksUpTimingMarkers)
{
    FrameTimingTracker tracker(8);
    FrameTimingRecord record{};

    tracker.UpsertCapture(7, 100, 120);
    tracker.MarkSlamIn(7, 140);
    tracker.MarkSlamOut(7, 180);
    tracker.MarkMavTx(7, 200);

    ASSERT_TRUE(tracker.Lookup(7, record));
    EXPECT_EQ(record.frameId, 7u);
    EXPECT_EQ(record.tCamNs, 100u);
    EXPECT_EQ(record.tCbNs, 120u);
    EXPECT_EQ(record.tSlamInNs, 140u);
    EXPECT_EQ(record.tSlamOutNs, 180u);
    EXPECT_EQ(record.tMavTxNs, 200u);
}

TEST(FrameTimingTrackerTest, TrimsOldestRecordsWhenCapacityExceeded)
{
    FrameTimingTracker tracker(2);
    FrameTimingRecord record{};

    tracker.UpsertCapture(1, 10, 11);
    tracker.UpsertCapture(2, 20, 21);
    tracker.UpsertCapture(3, 30, 31);

    EXPECT_FALSE(tracker.Lookup(1, record));
    ASSERT_TRUE(tracker.Lookup(2, record));
    EXPECT_EQ(record.tCamNs, 20u);
    ASSERT_TRUE(tracker.Lookup(3, record));
    EXPECT_EQ(record.tCamNs, 30u);
}

} // namespace
} // namespace smartdrone::core::application
