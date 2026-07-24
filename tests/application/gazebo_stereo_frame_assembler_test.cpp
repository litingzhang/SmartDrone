#include "adapters/camera/bounded_gazebo_image_queue.h"
#include "adapters/camera/gazebo_stereo_config.h"
#include "adapters/camera/gazebo_stereo_frame_assembler.h"
#include "adapters/camera/gazebo_image_processor.h"
#include "adapters/simulation/gazebo_measurement_clock.h"
#include "adapters/simulation/gazebo_pose_math.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

using SmartDrone::Adapters::Camera::GazeboImageFaultConfig;
using SmartDrone::Adapters::Camera::GazeboImagePixelFormat;
using SmartDrone::Adapters::Camera::GazeboImageProcessor;
using SmartDrone::Adapters::Camera::GazeboRawImage;
using SmartDrone::Adapters::Camera::BoundedGazeboImageQueue;
using SmartDrone::Adapters::Camera::GazeboStereoAssemblerConfig;
using SmartDrone::Adapters::Camera::GazeboStereoConfig;
using SmartDrone::Adapters::Camera::GazeboStereoEye;
using SmartDrone::Adapters::Camera::GazeboStereoFrameAssembler;
using SmartDrone::Adapters::Camera::LoadGazeboImageFaultConfigFromEnvironment;
using SmartDrone::Adapters::Camera::LoadGazeboStereoConfig;
using SmartDrone::Adapters::Simulation::GazeboMeasurementClock;
using SmartDrone::Adapters::Simulation::GazeboNedPoseSample;
using SmartDrone::Adapters::Simulation::GazeboPoseValue;
using SmartDrone::Adapters::Simulation::ConvertGazeboPoseToNedFrd;
using SmartDrone::Adapters::Simulation::EstimateGazeboNedVelocity;
using SmartDrone::Core::Ports::StereoFrame;

class ScopedEnvironment final {
public:
    ScopedEnvironment(const char *name, const char *value) : m_name(name)
    {
        const char *previous = std::getenv(name);
        if (previous != nullptr) {
            m_previous = previous;
        }
        setenv(name, value, 1);
    }

    ~ScopedEnvironment()
    {
        if (m_previous.has_value()) {
            setenv(m_name.c_str(), m_previous->c_str(), 1);
        } else {
            unsetenv(m_name.c_str());
        }
    }

    ScopedEnvironment(const ScopedEnvironment &) = delete;
    ScopedEnvironment &operator=(const ScopedEnvironment &) = delete;

private:
    std::string m_name;
    std::optional<std::string> m_previous;
};

class ScopedNeutralFaultEnvironment final {
private:
    ScopedEnvironment m_action{"SMART_DRONE_FAULT_ACTION", ""};
    ScopedEnvironment m_blur{"SMART_DRONE_FAULT_BLUR", ""};
    ScopedEnvironment m_blurSigma{"SMART_DRONE_FAULT_BLUR_SIGMA", ""};
    ScopedEnvironment m_brightness{"SMART_DRONE_FAULT_BRIGHTNESS", ""};
    ScopedEnvironment m_noise{"SMART_DRONE_FAULT_NOISE", ""};
    ScopedEnvironment m_drop{"SMART_DRONE_FAULT_DROP", ""};
    ScopedEnvironment m_delay{"SMART_DRONE_FAULT_DELAY", ""};
    ScopedEnvironment m_blackout{"SMART_DRONE_FAULT_BLACKOUT_MS", ""};
};

GazeboStereoAssemblerConfig MakeAssemblerConfig(std::size_t queueDepth = 8)
{
    GazeboStereoConfig simulation;
    simulation.queueDepth = queueDepth;
    simulation.pairToleranceNs = 5000000;
    return {2, 2, simulation};
}

GazeboRawImage MakeImage(GazeboImagePixelFormat format,
                         std::uint64_t timestampNs,
                         std::uint32_t sequence,
                         std::uint8_t value,
                         std::size_t stepPadding = 0)
{
    const std::size_t channels =
        format == GazeboImagePixelFormat::Mono8 ? 1 : 3;
    GazeboRawImage image;
    image.width = 2;
    image.height = 2;
    image.step = 2 * channels + stepPadding;
    image.pixelFormat = format;
    image.measurementTimestampNs = timestampNs;
    image.captureMonotonicNs = static_cast<std::int64_t>(timestampNs + 10);
    image.arrivalMonotonicNs = static_cast<std::int64_t>(timestampNs + 20);
    image.sequence = sequence;
    image.payload = std::make_shared<std::vector<std::uint8_t>>(
        image.step * 2, value);
    return image;
}

void PushPair(GazeboStereoFrameAssembler &assembler,
              std::uint64_t timestampNs, std::uint32_t sequence,
              std::uint8_t value)
{
    ASSERT_TRUE(assembler.PushImage(
        GazeboStereoEye::Left,
        MakeImage(GazeboImagePixelFormat::Mono8, timestampNs, sequence,
                  value)));
    ASSERT_TRUE(assembler.PushImage(
        GazeboStereoEye::Right,
        MakeImage(GazeboImagePixelFormat::Mono8, timestampNs, sequence,
                  value)));
}

TEST(GazeboStereoFrameAssemblerTest,
     RecoversWhenNextImageMatchesPendingEyeAfterTimestampMismatch)
{
    GazeboStereoFrameAssembler assembler(MakeAssemblerConfig());
    assembler.SetAcceptFrames(true);

    ASSERT_TRUE(assembler.PushImage(
        GazeboStereoEye::Left,
        MakeImage(GazeboImagePixelFormat::Mono8, 1000000000, 1, 10)));
    ASSERT_TRUE(assembler.PushImage(
        GazeboStereoEye::Right,
        MakeImage(GazeboImagePixelFormat::Mono8, 2000000000, 1, 20)));

    StereoFrame frame;
    EXPECT_FALSE(assembler.GrabStereo(frame, false));

    ASSERT_TRUE(assembler.PushImage(
        GazeboStereoEye::Left,
        MakeImage(GazeboImagePixelFormat::Mono8, 2000000000, 2, 20)));
    ASSERT_TRUE(assembler.GrabStereo(frame, false));
    EXPECT_EQ(frame.left.timestampNs, 2000000000U);
    EXPECT_EQ(frame.right.timestampNs, 2000000000U);
}

TEST(GazeboStereoFrameAssemblerTest, PairsOutOfOrderFramesByMeasurementTime)
{
    GazeboStereoFrameAssembler assembler(MakeAssemblerConfig());
    assembler.SetAcceptFrames(true);
    ASSERT_TRUE(assembler.PushImage(
        GazeboStereoEye::Left,
        MakeImage(GazeboImagePixelFormat::Mono8, 2000000000, 2, 20)));
    ASSERT_TRUE(assembler.PushImage(
        GazeboStereoEye::Left,
        MakeImage(GazeboImagePixelFormat::Mono8, 1000000000, 1, 10)));
    ASSERT_TRUE(assembler.PushImage(
        GazeboStereoEye::Right,
        MakeImage(GazeboImagePixelFormat::Mono8, 1000000000, 1, 10)));
    ASSERT_TRUE(assembler.PushImage(
        GazeboStereoEye::Right,
        MakeImage(GazeboImagePixelFormat::Mono8, 2000000000, 2, 20)));

    StereoFrame frame;
    ASSERT_TRUE(assembler.GrabStereo(frame, false));
    EXPECT_EQ(frame.left.timestampNs, 1000000000U);
    EXPECT_EQ(frame.right.timestampNs, 1000000000U);
    EXPECT_EQ(frame.left.gray.at<std::uint8_t>(0, 0), 10);
}

TEST(GazeboStereoFrameAssemblerTest, PreferLatestHonorsMinimumTimestamp)
{
    GazeboStereoFrameAssembler assembler(MakeAssemblerConfig());
    assembler.SetAcceptFrames(true);
    PushPair(assembler, 1000000000, 1, 10);
    PushPair(assembler, 2000000000, 2, 20);

    StereoFrame frame;
    ASSERT_TRUE(assembler.GrabStereo(frame, true, 1500000000));
    EXPECT_EQ(frame.left.timestampNs, 2000000000U);
    EXPECT_EQ(frame.left.gray.at<std::uint8_t>(0, 0), 20);
}

TEST(GazeboStereoFrameAssemblerTest, ConvertsRgbAndBgrWithPaddedStride)
{
    GazeboStereoFrameAssembler assembler(MakeAssemblerConfig());
    assembler.SetAcceptFrames(true);
    auto left = MakeImage(GazeboImagePixelFormat::Rgb8, 1000000000, 1, 0, 2);
    auto right = MakeImage(GazeboImagePixelFormat::Bgr8, 1000000000, 1, 0, 2);
    for (int row = 0; row < 2; ++row) {
        const std::size_t offset = static_cast<std::size_t>(row) * left.step;
        (*left.payload)[offset] = 255;
        (*right.payload)[offset] = 255;
    }
    ASSERT_TRUE(assembler.PushImage(GazeboStereoEye::Left, std::move(left)));
    ASSERT_TRUE(assembler.PushImage(GazeboStereoEye::Right, std::move(right)));

    StereoFrame frame;
    ASSERT_TRUE(assembler.GrabStereo(frame, false));
    EXPECT_EQ(frame.left.gray.at<std::uint8_t>(0, 0), 76);
    EXPECT_EQ(frame.right.gray.at<std::uint8_t>(0, 0), 29);
}

TEST(GazeboStereoFrameAssemblerTest, RejectsInvalidDimensionsAndStride)
{
    GazeboStereoFrameAssembler assembler(MakeAssemblerConfig());
    assembler.SetAcceptFrames(true);
    auto invalidSize = MakeImage(GazeboImagePixelFormat::Mono8,
                                 1000000000, 1, 10);
    invalidSize.width = 3;
    EXPECT_FALSE(assembler.PushImage(GazeboStereoEye::Left,
                                     std::move(invalidSize)));
    auto invalidStride = MakeImage(GazeboImagePixelFormat::Rgb8,
                                   1000000000, 1, 10);
    invalidStride.step = 2;
    EXPECT_FALSE(assembler.PushImage(GazeboStereoEye::Right,
                                     std::move(invalidStride)));
    EXPECT_FALSE(assembler.GetHealth().healthy);
}

TEST(GazeboStereoFrameAssemblerTest, BoundedQueuesKeepNewestFrames)
{
    GazeboStereoFrameAssembler assembler(MakeAssemblerConfig(2));
    assembler.SetAcceptFrames(true);
    for (std::uint32_t sequence = 1; sequence <= 4; ++sequence) {
        const std::uint64_t timestamp = sequence * 1000000000ULL;
        ASSERT_TRUE(assembler.PushImage(
            GazeboStereoEye::Left,
            MakeImage(GazeboImagePixelFormat::Mono8, timestamp, sequence,
                      static_cast<std::uint8_t>(sequence))));
    }
    for (std::uint32_t sequence = 1; sequence <= 4; ++sequence) {
        const std::uint64_t timestamp = sequence * 1000000000ULL;
        ASSERT_TRUE(assembler.PushImage(
            GazeboStereoEye::Right,
            MakeImage(GazeboImagePixelFormat::Mono8, timestamp, sequence,
                      static_cast<std::uint8_t>(sequence))));
    }

    StereoFrame frame;
    ASSERT_TRUE(assembler.GrabStereo(frame, true));
    EXPECT_EQ(frame.left.sequence, 4U);
    const auto diagnostics = assembler.GetDiagnostics();
    EXPECT_EQ(diagnostics.queueOverflowL, 2U);
    EXPECT_EQ(diagnostics.queueOverflowR, 2U);
}

TEST(BoundedGazeboImageQueueTest, EnforcesMinimumValidCapacity)
{
    BoundedGazeboImageQueue queue(1);
    EXPECT_EQ(queue.Capacity(), 2U);
    EXPECT_TRUE(queue.TryEnqueue({}));
    EXPECT_TRUE(queue.TryEnqueue({}));
    EXPECT_FALSE(queue.TryEnqueue({}));
}

TEST(GazeboStereoFrameAssemblerTest, RewindDropsOldGenerationAndRecovers)
{
    GazeboStereoFrameAssembler assembler(MakeAssemblerConfig());
    assembler.SetAcceptFrames(true);
    PushPair(assembler, 3000000000, 1, 30);
    StereoFrame frame;
    ASSERT_TRUE(assembler.GrabStereo(frame, false));

    PushPair(assembler, 100000000, 2, 10);
    ASSERT_TRUE(assembler.GrabStereo(frame, false));
    EXPECT_EQ(frame.left.timestampNs, 100000000U);
    const auto diagnostics = assembler.GetDiagnostics();
    EXPECT_EQ(diagnostics.timestampRewinds, 1U);
    EXPECT_EQ(diagnostics.clockResetCounter, 1U);
}

TEST(GazeboStereoFrameAssemblerTest, FrameOwnerSurvivesAssemblerDestruction)
{
    StereoFrame frame;
    {
        GazeboStereoFrameAssembler assembler(MakeAssemblerConfig());
        assembler.SetAcceptFrames(true);
        PushPair(assembler, 1000000000, 1, 77);
        ASSERT_TRUE(assembler.GrabStereo(frame, false));
    }
    ASSERT_TRUE(frame.left.owner);
    EXPECT_EQ(frame.left.gray.at<std::uint8_t>(1, 1), 77);
}

TEST(GazeboStereoFrameAssemblerTest, LoadsAtomicFaultFileByGeneration)
{
    const auto path = std::filesystem::temp_directory_path() /
                      "smart_drone_fault_state_test.json";
    {
        std::ofstream stream(path);
        stream << R"({"schema":"smartdrone.sitl.image_fault.v1",)"
               << R"("generation":4,"updated_monotonic_ns":1,)"
               << R"("kind":"quality","action":"apply",)"
               << R"("blur_sigma":1.5,"brightness":0.4,)"
               << R"("noise_stddev":3.0,"drop_rate":0.1,)"
               << R"("delay_ms":40,"blackout_ms":0})";
    }
    GazeboImageFaultConfig config;
    ASSERT_TRUE(SmartDrone::Adapters::Camera::TryLoadGazeboImageFaultState(
        path.string(), 3, config));
    EXPECT_EQ(config.generation, 4U);
    EXPECT_DOUBLE_EQ(config.blurSigma, 1.5);
    EXPECT_DOUBLE_EQ(config.dropRate, 0.1);
    EXPECT_EQ(config.delayMs, 40);
    EXPECT_FALSE(SmartDrone::Adapters::Camera::TryLoadGazeboImageFaultState(
        path.string(), 4, config));
    std::filesystem::remove(path);
}

TEST(GazeboStereoFrameAssemblerTest, FaultStateOverridesStartupFaultDefaults)
{
    const auto path = std::filesystem::temp_directory_path() /
                      "smart_drone_fault_precedence_test.json";
    {
        std::ofstream stream(path);
        stream << R"({"schema":"smartdrone.sitl.image_fault.v1",)"
               << R"("generation":1,"action":"apply",)"
               << R"("blur_sigma":0.0,"brightness":2.0,)"
               << R"("noise_stddev":0.0,"drop_rate":0.0,)"
               << R"("delay_ms":0,"blackout_ms":0})";
    }
    auto config = MakeAssemblerConfig();
    config.simulation.fault.brightness = 0.5;
    config.simulation.faultStatePath = path.string();
    GazeboStereoFrameAssembler assembler(std::move(config));
    assembler.SetAcceptFrames(true);
    PushPair(assembler, 1000000000, 1, 100);

    StereoFrame frame;
    ASSERT_TRUE(assembler.GrabStereo(frame, false));
    EXPECT_EQ(frame.left.gray.at<std::uint8_t>(0, 0), 200);
    std::filesystem::remove(path);
}

TEST(GazeboImageProcessorTest, AppliesDropRateOncePerStereoPair)
{
    GazeboImageFaultConfig fault;
    fault.dropRate = 0.10;
    GazeboImageProcessor processor(fault);
    int dropped = 0;

    for (std::uint64_t millisecond = 0; millisecond < 100; ++millisecond) {
        dropped += processor.ShouldDrop(millisecond * 1000000ULL) ? 1 : 0;
    }

    EXPECT_EQ(dropped, 10);
}

TEST(GazeboMeasurementClockTest, MapsMeasurementTimeAndCountsRewinds)
{
    GazeboMeasurementClock clock;
    EXPECT_FALSE(clock.Valid());
    clock.Observe(1000000000, 5000000000);
    EXPECT_TRUE(clock.Valid());
    EXPECT_EQ(clock.NowNs(), 1000000000U);
    EXPECT_EQ(clock.EstimateCaptureMonotonicNs(900000000, 0), 4900000000);
    const auto firstStamp = clock.ResolveMeasurementStamp(0, 42);
    EXPECT_TRUE(firstStamp.clockValid);
    EXPECT_EQ(firstStamp.measurementNs, 1000000000U);
    EXPECT_EQ(firstStamp.captureMonotonicNs, 5000000000);
    EXPECT_EQ(firstStamp.resetCounter, 0U);
    clock.Observe(100000000, 6000000000);
    EXPECT_EQ(clock.ResetCounter(), 1U);
    const auto resetStamp = clock.ResolveMeasurementStamp(90000000, 42);
    EXPECT_EQ(resetStamp.measurementNs, 90000000U);
    EXPECT_EQ(resetStamp.captureMonotonicNs, 5990000000);
    EXPECT_EQ(resetStamp.resetCounter, 1U);
}

TEST(GazeboMeasurementClockTest, MapsFasterThanRealTimeSimulation)
{
    GazeboMeasurementClock clock;
    clock.Observe(1000000000, 5000000000);
    clock.Observe(1100000000, 5050000000);

    EXPECT_EQ(clock.EstimateCaptureMonotonicNs(1080000000, 0), 5040000000);
    EXPECT_EQ(clock.EstimateCaptureMonotonicNs(1120000000, 0), 5060000000);
}

TEST(GazeboMeasurementClockTest, MapsSlowerThanRealTimeSimulation)
{
    GazeboMeasurementClock clock;
    clock.Observe(2000000000, 8000000000);
    clock.Observe(2100000000, 8200000000);

    EXPECT_EQ(clock.EstimateCaptureMonotonicNs(2080000000, 0), 8160000000);
    EXPECT_EQ(clock.EstimateCaptureMonotonicNs(2120000000, 0), 8240000000);
}

TEST(GazeboMeasurementClockTest, UsesSimulationRealTimeForRateEstimate)
{
    GazeboMeasurementClock clock;
    clock.Observe(1000000000, 10000000000ULL, 5000000000);
    clock.Observe(1100000000, 10050000000ULL, 5200000000);

    EXPECT_EQ(clock.EstimateCaptureMonotonicNs(1080000000, 0), 5190000000);
    EXPECT_EQ(clock.EstimateCaptureMonotonicNs(1120000000, 0), 5210000000);
}

TEST(GazeboMeasurementClockTest, PauseStallsOnceAndRestartsRateEstimate)
{
    GazeboMeasurementClock clock;
    clock.Observe(1000000000, 5000000000);
    clock.Observe(1100000000, 5050000000);

    clock.Observe(1100000000, 5500000000);
    EXPECT_EQ(clock.LastArrivalMonotonicNs(), 5050000000);
    EXPECT_EQ(clock.EstimateCaptureMonotonicNs(1080000000, 0), 5040000000);
    EXPECT_TRUE(clock.DetectStall(5600000000, 500000000));
    EXPECT_FALSE(clock.DetectStall(5700000000, 500000000));

    clock.Observe(1200000000, 5800000000);
    clock.Observe(1300000000, 5850000000);
    EXPECT_EQ(clock.EstimateCaptureMonotonicNs(1280000000, 0), 5840000000);
    EXPECT_EQ(clock.ResetCounter(), 1U);
}

TEST(GazeboMeasurementClockTest, RewindRestartsRateEstimate)
{
    GazeboMeasurementClock clock;
    clock.Observe(1000000000, 5000000000);
    clock.Observe(1100000000, 5050000000);
    clock.Observe(200000000, 6000000000);

    EXPECT_EQ(clock.ResetCounter(), 1U);
    EXPECT_EQ(clock.EstimateCaptureMonotonicNs(180000000, 0), 5980000000);

    clock.Observe(300000000, 6200000000);
    EXPECT_EQ(clock.EstimateCaptureMonotonicNs(280000000, 0), 6160000000);
    EXPECT_EQ(clock.ResetCounter(), 1U);
}

TEST(GazeboMeasurementClockTest, MedianRateRejectsOneTimingOutlier)
{
    GazeboMeasurementClock clock;
    clock.Observe(1000000000, 5000000000);
    clock.Observe(1100000000, 5050000000);
    clock.Observe(1200000000, 5100000000);
    clock.Observe(1300000000, 5150000000);
    clock.Observe(1400000000, 5300000000);

    EXPECT_EQ(clock.EstimateCaptureMonotonicNs(1380000000, 0), 5290000000);
}

TEST(GazeboMeasurementClockTest, CountsEachSteadyClockStallOnce)
{
    GazeboMeasurementClock clock;
    clock.Observe(1000000000, 5000000000);

    EXPECT_FALSE(clock.DetectStall(5400000000, 500000000));
    EXPECT_TRUE(clock.DetectStall(5600000000, 500000000));
    EXPECT_FALSE(clock.DetectStall(7000000000, 500000000));
    EXPECT_EQ(clock.ResetCounter(), 1U);

    clock.Observe(1100000000, 7100000000);
    EXPECT_TRUE(clock.DetectStall(7700000000, 500000000));
    EXPECT_EQ(clock.ResetCounter(), 2U);
}

TEST(GazeboMeasurementClockTest, IgnoresSmallOutOfOrderClockCallbacks)
{
    GazeboMeasurementClock clock;
    clock.Observe(1000000000, 5000000000);

    clock.Observe(980000000, 5100000000);

    EXPECT_EQ(clock.NowNs(), 1000000000U);
    EXPECT_EQ(clock.LastArrivalMonotonicNs(), 5000000000);
    EXPECT_EQ(clock.ResetCounter(), 0U);
}

TEST(GazeboStereoConfigTest, LoadsImageGeometryFromReferencedCalibration)
{
    const auto directory = std::filesystem::temp_directory_path();
    const auto calibrationPath = directory / "smartdrone_gz_calibration.yaml";
    const auto configPath = directory / "smartdrone_gz_config.yaml";
    {
        std::ofstream stream(calibrationPath);
        stream << "%YAML:1.0\nCamera.width: 640\nCamera.height: 480\n"
               << "Camera.fps: 30\n";
    }
    {
        std::ofstream stream(configPath);
        stream << "%YAML:1.0\nleft_image_topic: /left\n"
               << "right_image_topic: /right\nclock_topic: /clock\n"
               << "calibration_path: smartdrone_gz_calibration.yaml\n";
    }

    const auto loaded = LoadGazeboStereoConfig(configPath.string());

    ASSERT_TRUE(loaded.ok) << loaded.error;
    EXPECT_EQ(loaded.config.cameraWidth, 640);
    EXPECT_EQ(loaded.config.cameraHeight, 480);
    EXPECT_EQ(loaded.config.cameraFps, 30);
    EXPECT_EQ(loaded.config.calibrationPath,
              calibrationPath.lexically_normal().string());
    std::filesystem::remove(configPath);
    std::filesystem::remove(calibrationPath);
}

TEST(GazeboStereoConfigTest, LoadsFaultDefaultsFromSimulationYaml)
{
    ScopedNeutralFaultEnvironment environment;
    const auto directory = std::filesystem::temp_directory_path();
    const auto calibrationPath = directory / "smartdrone_gz_fault_calibration.yaml";
    const auto configPath = directory / "smartdrone_gz_fault_config.yaml";
    {
        std::ofstream stream(calibrationPath);
        stream << "%YAML:1.0\nCamera.width: 640\nCamera.height: 480\n"
               << "Camera.fps: 30\n";
    }
    {
        std::ofstream stream(configPath);
        stream << "%YAML:1.0\nleft_image_topic: /left\n"
               << "right_image_topic: /right\nclock_topic: /clock\n"
               << "calibration_path: smartdrone_gz_fault_calibration.yaml\n"
               << "faults:\n  blur_sigma: 1.5\n  brightness: 0.7\n"
               << "  noise_stddev: 3.0\n  drop_rate: 0.1\n"
               << "  delay_ms: 40\n  blackout_ms: 300\n";
    }

    const auto loaded = LoadGazeboStereoConfig(configPath.string());

    ASSERT_TRUE(loaded.ok) << loaded.error;
    EXPECT_DOUBLE_EQ(loaded.config.fault.blurSigma, 1.5);
    EXPECT_DOUBLE_EQ(loaded.config.fault.brightness, 0.7);
    EXPECT_DOUBLE_EQ(loaded.config.fault.noiseStddev, 3.0);
    EXPECT_DOUBLE_EQ(loaded.config.fault.dropRate, 0.1);
    EXPECT_EQ(loaded.config.fault.delayMs, 40);
    EXPECT_EQ(loaded.config.fault.blackoutMs, 300);
    std::filesystem::remove(configPath);
    std::filesystem::remove(calibrationPath);
}

TEST(GazeboStereoConfigTest, ExplicitEnvironmentOverridesYamlByField)
{
    ScopedNeutralFaultEnvironment environment;
    ScopedEnvironment blur("SMART_DRONE_FAULT_BLUR_SIGMA", "2.5");
    ScopedEnvironment brightness("SMART_DRONE_FAULT_BRIGHTNESS", "0.5");
    ScopedEnvironment drop("SMART_DRONE_FAULT_DROP", "25");
    ScopedEnvironment delay("SMART_DRONE_FAULT_DELAY", "80");
    GazeboImageFaultConfig defaults;
    defaults.blurSigma = 1.5;
    defaults.brightness = 0.7;
    defaults.noiseStddev = 3.0;
    defaults.dropRate = 0.1;
    defaults.delayMs = 40;
    defaults.blackoutMs = 300;

    const auto loaded =
        LoadGazeboImageFaultConfigFromEnvironment(std::move(defaults));

    EXPECT_DOUBLE_EQ(loaded.blurSigma, 2.5);
    EXPECT_DOUBLE_EQ(loaded.brightness, 0.5);
    EXPECT_DOUBLE_EQ(loaded.noiseStddev, 3.0);
    EXPECT_DOUBLE_EQ(loaded.dropRate, 0.25);
    EXPECT_EQ(loaded.delayMs, 80);
    EXPECT_EQ(loaded.blackoutMs, 300);
}

TEST(GazeboStereoConfigTest, LegacyBlurKernelOverridesYamlSigma)
{
    ScopedNeutralFaultEnvironment environment;
    ScopedEnvironment blur("SMART_DRONE_FAULT_BLUR", "7");
    GazeboImageFaultConfig defaults;
    defaults.blurSigma = 1.5;

    const auto loaded =
        LoadGazeboImageFaultConfigFromEnvironment(std::move(defaults));

    EXPECT_EQ(loaded.blurKernel, 7);
    EXPECT_DOUBLE_EQ(loaded.blurSigma, 0.0);
}

TEST(GazeboStereoConfigTest, AppliesLauncherWorldAndModelOverrides)
{
    ScopedEnvironment world("SMART_DRONE_SIM_WORLD", "alternate_world");
    ScopedEnvironment model("SMART_DRONE_SIM_MODEL", "alternate_vehicle");
    const auto directory = std::filesystem::temp_directory_path();
    const auto calibrationPath = directory / "smartdrone_gz_override_calibration.yaml";
    const auto configPath = directory / "smartdrone_gz_override_config.yaml";
    {
        std::ofstream stream(calibrationPath);
        stream << "%YAML:1.0\nCamera.width: 640\nCamera.height: 480\n"
               << "Camera.fps: 30\n";
    }
    {
        std::ofstream stream(configPath);
        stream << "%YAML:1.0\nleft_image_topic: /left\n"
               << "right_image_topic: /right\nclock_topic: /world/default/clock\n"
               << "truth_pose_topic: /world/default/dynamic_pose/info\n"
               << "truth_model_name: default_vehicle\n"
               << "calibration_path: smartdrone_gz_override_calibration.yaml\n";
    }

    const auto loaded = LoadGazeboStereoConfig(configPath.string());

    ASSERT_TRUE(loaded.ok) << loaded.error;
    EXPECT_EQ(loaded.config.clockTopic, "/world/alternate_world/clock");
    EXPECT_EQ(loaded.config.truthPoseTopic,
              "/world/alternate_world/dynamic_pose/info");
    EXPECT_EQ(loaded.config.truthModelName, "alternate_vehicle");
    std::filesystem::remove(configPath);
    std::filesystem::remove(calibrationPath);
}

TEST(GazeboPoseMathTest, ConvertsIdentityFluPoseToNedYawPositiveNinety)
{
    const auto pose = ConvertGazeboPoseToNedFrd(
        GazeboPoseValue{2.0, 3.0, 4.0, 1.0, 0.0, 0.0, 0.0});

    EXPECT_TRUE(pose.valid);
    EXPECT_FLOAT_EQ(pose.x, 3.0f);
    EXPECT_FLOAT_EQ(pose.y, 2.0f);
    EXPECT_FLOAT_EQ(pose.z, -4.0f);
    EXPECT_NEAR(pose.qw, 0.70710678f, 1.0e-6f);
    EXPECT_NEAR(pose.qx, 0.0f, 1.0e-6f);
    EXPECT_NEAR(pose.qy, 0.0f, 1.0e-6f);
    EXPECT_NEAR(pose.qz, 0.70710678f, 1.0e-6f);
}

TEST(GazeboPoseMathTest, EstimatesNedVelocityAndRejectsClockReset)
{
    GazeboNedPoseSample previous{};
    previous.measurementTimestampNs = 1000000000ULL;
    previous.pose = {true, 1.0f, 2.0f, -3.0f, 1.0f, 0.0f, 0.0f, 0.0f};
    GazeboNedPoseSample current = previous;
    current.measurementTimestampNs = 1100000000ULL;
    current.pose.x = 1.2f;
    current.pose.y = 1.9f;
    current.pose.z = -2.7f;

    const auto velocity = EstimateGazeboNedVelocity(previous, current);

    ASSERT_TRUE(velocity.valid);
    EXPECT_NEAR(velocity.vx, 2.0f, 1.0e-5f);
    EXPECT_NEAR(velocity.vy, -1.0f, 1.0e-5f);
    EXPECT_NEAR(velocity.vz, 3.0f, 1.0e-5f);
    current.resetCounter = 1;
    EXPECT_FALSE(EstimateGazeboNedVelocity(previous, current).valid);
}

} // namespace
