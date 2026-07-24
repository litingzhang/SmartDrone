#include "core/application/state/perception_pipeline.h"

#include <utility>

#include <gtest/gtest.h>

namespace {

using SmartDrone::Core::Application::FrameTimingRecord;
using SmartDrone::Core::Application::FrameTimingTracker;
using SmartDrone::Core::Application::PerceptionPipeline;
using SmartDrone::Core::Application::PerceptionPipelineConfig;
using SmartDrone::Core::Application::StereoAcquireStatus;
using SmartDrone::Core::Application::StereoBatch;
using SmartDrone::Core::Ports::CameraDiagnostics;
using SmartDrone::Core::Ports::CameraHealth;
using SmartDrone::Core::Ports::CameraOpenConfig;
using SmartDrone::Core::Ports::CameraProviderSemantics;
using SmartDrone::Core::Ports::ICameraProvider;
using SmartDrone::Core::Ports::StereoFrame;

class OneFrameCamera final : public ICameraProvider {
  public:
    explicit OneFrameCamera(StereoFrame frame)
        : m_frame(std::move(frame))
    {
    }

    bool Open(const CameraOpenConfig &) override
    {
        return true;
    }

    void Close() override
    {
    }

    bool Start() override
    {
        return true;
    }

    void Stop() override
    {
    }

    bool GrabStereo(StereoFrame &out, bool, std::uint64_t minTimestampNs) override
    {
        m_lastMinTimestampNs = minTimestampNs;
        if (m_delivered) {
            return false;
        }
        const std::uint64_t timestampNs =
            m_frame.left.timestampNs +
            (m_frame.right.timestampNs - m_frame.left.timestampNs) / 2ULL;
        if (timestampNs < minTimestampNs) {
            return false;
        }
        out = m_frame;
        m_delivered = true;
        return true;
    }

    CameraHealth GetHealth() const override
    {
        return {true, 0};
    }

    CameraDiagnostics GetDiagnostics() const override
    {
        CameraDiagnostics diagnostics;
        diagnostics.clockResetCounter = m_clockResetCounter;
        return diagnostics;
    }

    CameraProviderSemantics Semantics() const override
    {
        return CameraProviderSemantics::DualStreamPaired;
    }

    void SetClockResetCounter(std::uint32_t resetCounter)
    {
        m_clockResetCounter = resetCounter;
    }

    void SetFrame(StereoFrame frame)
    {
        m_frame = std::move(frame);
        m_delivered = false;
    }

    std::uint64_t LastMinTimestampNs() const
    {
        return m_lastMinTimestampNs;
    }

  private:
    StereoFrame m_frame;
    bool m_delivered{false};
    std::uint32_t m_clockResetCounter{0};
    std::uint64_t m_lastMinTimestampNs{0};
};

StereoFrame FrameAt(std::uint64_t timestampNs)
{
    StereoFrame frame;
    frame.left.timestampNs = timestampNs;
    frame.right.timestampNs = timestampNs;
    return frame;
}

TEST(PerceptionPipelineTimingTest, KeepsMeasurementAndMonotonicDomainsSeparate)
{
    StereoFrame stereo;
    stereo.left.timestampNs = 10;
    stereo.right.timestampNs = 14;
    stereo.left.captureMonotonicNs = 200;
    stereo.right.captureMonotonicNs = 400;
    stereo.left.arriveNs = 500;
    stereo.right.arriveNs = 700;
    OneFrameCamera camera(stereo);
    PerceptionPipeline pipeline(PerceptionPipelineConfig{30, true});
    FrameTimingTracker tracker;
    StereoBatch batch;

    ASSERT_EQ(pipeline.AcquireNextStereoBatch(camera, 30, 0, batch,
                                               &tracker),
              StereoAcquireStatus::Ok);
    FrameTimingRecord timing;
    ASSERT_TRUE(tracker.Lookup(batch.frameId, timing));
    EXPECT_EQ(timing.tCamNs, 12U);
    EXPECT_EQ(timing.tCaptureMonotonicNs, 300U);
    EXPECT_EQ(timing.tLeftArrivalNs, 500U);
    EXPECT_EQ(timing.tRightArrivalNs, 700U);
    EXPECT_EQ(timing.tPairReadyNs, 700U);
    EXPECT_EQ(timing.tCbNs, 700U);
}

TEST(PerceptionPipelineTimingTest, RequestsSessionRestartOnClockReset)
{
    StereoFrame stereo;
    stereo.left.timestampNs = 1000000000;
    stereo.right.timestampNs = 1000000000;
    OneFrameCamera camera(stereo);
    PerceptionPipeline pipeline(PerceptionPipelineConfig{30, true});
    StereoBatch batch;
    ASSERT_EQ(pipeline.AcquireNextStereoBatch(camera, 30, 0, batch),
              StereoAcquireStatus::Ok);

    camera.SetClockResetCounter(1);
    EXPECT_EQ(pipeline.AcquireNextStereoBatch(camera, 30, 0, batch),
              StereoAcquireStatus::CameraClockReset);
}

TEST(PerceptionPipelineTimingTest, AcceptsExistingResetGenerationOnNewSession)
{
    StereoFrame stereo;
    stereo.left.timestampNs = 1000000000;
    stereo.right.timestampNs = 1000000000;
    OneFrameCamera camera(stereo);
    camera.SetClockResetCounter(3);
    PerceptionPipeline pipeline(PerceptionPipelineConfig{30, true});
    StereoBatch batch;

    EXPECT_EQ(pipeline.AcquireNextStereoBatch(camera, 30, 0, batch),
              StereoAcquireStatus::Ok);
}

TEST(PerceptionPipelineTimingTest, LimitsInputByMeasurementTimeWithoutDrift)
{
    constexpr std::uint64_t startNs = 1000000000ULL;
    OneFrameCamera camera(FrameAt(startNs));
    PerceptionPipeline pipeline(PerceptionPipelineConfig{30, true});
    StereoBatch batch;
    ASSERT_EQ(pipeline.AcquireNextStereoBatch(camera, 15, 0, batch),
              StereoAcquireStatus::Ok);

    camera.SetFrame(FrameAt(startNs + 32000000ULL));
    EXPECT_EQ(pipeline.AcquireNextStereoBatch(camera, 15, 0, batch),
              StereoAcquireStatus::Timeout);
    EXPECT_EQ(camera.LastMinTimestampNs(), startNs + 62500000ULL);

    camera.SetFrame(FrameAt(startNs + 68000000ULL));
    ASSERT_EQ(pipeline.AcquireNextStereoBatch(camera, 15, 0, batch),
              StereoAcquireStatus::Ok);
    camera.SetFrame(FrameAt(startNs + 100000000ULL));
    EXPECT_EQ(pipeline.AcquireNextStereoBatch(camera, 15, 0, batch),
              StereoAcquireStatus::Timeout);
    camera.SetFrame(FrameAt(startNs + 132000000ULL));
    EXPECT_EQ(pipeline.AcquireNextStereoBatch(camera, 15, 0, batch),
              StereoAcquireStatus::Ok);
}

} // namespace
