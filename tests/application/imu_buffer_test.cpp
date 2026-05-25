#include "core/application/state/imu_buffer.h"

#include <gtest/gtest.h>

namespace {

ImuSample MakeSample(std::int64_t timestampNs, float base)
{
    ImuSample sample{};
    sample.tNs = timestampNs;
    sample.ax = base + 1.0f;
    sample.ay = base + 2.0f;
    sample.az = base + 3.0f;
    sample.gx = base + 4.0f;
    sample.gy = base + 5.0f;
    sample.gz = base + 6.0f;
    return sample;
}

TEST(ImuBufferTest, TracksSizeAndFirstLast)
{
    ImuBuffer buffer;

    EXPECT_EQ(buffer.Size(), 0U);

    buffer.Push(MakeSample(100, 1.0f));
    buffer.Push(MakeSample(200, 2.0f));

    int64_t firstNs = 0;
    int64_t lastNs = 0;
    ASSERT_TRUE(buffer.PeekFirstLast(firstNs, lastNs));
    EXPECT_EQ(firstNs, 100);
    EXPECT_EQ(lastNs, 200);
    EXPECT_EQ(buffer.Size(), 2U);
}

TEST(ImuBufferTest, BuildsWindowWithInterpolatedBoundaries)
{
    ImuBuffer buffer;
    buffer.Push(MakeSample(0, 0.0f));
    buffer.Push(MakeSample(100, 10.0f));
    buffer.Push(MakeSample(200, 20.0f));
    buffer.Push(MakeSample(300, 30.0f));

    const auto readings = buffer.PopBetweenNs(50, 250, 0, 100);

    ASSERT_EQ(readings.size(), 4U);
    EXPECT_EQ(readings[0].timestampNs, 50);
    EXPECT_FLOAT_EQ(readings[0].ax, 6.0f);
    EXPECT_EQ(readings[1].timestampNs, 100);
    EXPECT_EQ(readings[2].timestampNs, 200);
    EXPECT_EQ(readings[3].timestampNs, 250);
    EXPECT_FLOAT_EQ(readings[3].ax, 26.0f);
}

TEST(ImuBufferTest, UsesSlackSampleWhenInterpolationUnavailable)
{
    ImuBuffer buffer;
    buffer.Push(MakeSample(80, 8.0f));

    const auto readings = buffer.PopBetweenNs(100, 160, 30, 0);

    ASSERT_EQ(readings.size(), 1U);
    EXPECT_EQ(readings[0].timestampNs, 80);
}

TEST(ImuBufferTest, MarksConsumedHistoryAsPurged)
{
    ImuBuffer buffer;
    buffer.Push(MakeSample(0, 0.0f));
    buffer.Push(MakeSample(100000000, 10.0f));
    buffer.Push(MakeSample(200000000, 20.0f));

    const auto current = buffer.PopBetweenNs(100000000, 200000000, 0, 0);
    ASSERT_FALSE(current.empty());

    const auto purged = buffer.PopBetweenNs(0, 0, 0, 0);
    EXPECT_TRUE(purged.empty());
}

TEST(ImuBufferTest, RetainsFixedCapacityNewestSamples)
{
    ImuBuffer buffer;
    constexpr int sampleCount = 16385;

    for (int i = 0; i < sampleCount; ++i) {
        buffer.Push(MakeSample(i, static_cast<float>(i)));
    }

    int64_t firstNs = 0;
    int64_t lastNs = 0;
    ASSERT_TRUE(buffer.PeekFirstLast(firstNs, lastNs));
    EXPECT_EQ(firstNs, 1);
    EXPECT_EQ(lastNs, sampleCount - 1);
    EXPECT_EQ(buffer.Size(), 16384U);
}

} // namespace
