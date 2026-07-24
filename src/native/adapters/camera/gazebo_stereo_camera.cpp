#include "adapters/camera/gazebo_stereo_camera.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/msgs/clock.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

#include "adapters/camera/gazebo_stereo_config.h"
#include "adapters/camera/gazebo_stereo_frame_assembler.h"
#include "adapters/simulation/gazebo_measurement_clock.h"
#include "adapters/simulation/gazebo_message_time.h"
#include "common/environment.h"
#include "common/time_utils.h"

namespace SmartDrone::Adapters::Camera {
namespace {

constexpr std::int64_t CLOCK_STALL_TIMEOUT_NS = 500000000LL;

std::int64_t NowMonotonicNs()
{
    return static_cast<std::int64_t>(MonoTimeUs() * 1000ULL);
}

GazeboImagePixelFormat PixelFormat(const gz::msgs::Image &message)
{
    switch (message.pixel_format_type()) {
    case gz::msgs::PixelFormatType::L_INT8:
        return GazeboImagePixelFormat::Mono8;
    case gz::msgs::PixelFormatType::RGB_INT8:
        return GazeboImagePixelFormat::Rgb8;
    case gz::msgs::PixelFormatType::BGR_INT8:
        return GazeboImagePixelFormat::Bgr8;
    default:
        return GazeboImagePixelFormat::Unsupported;
    }
}

std::uint64_t ImageHeaderTimestamp(const gz::msgs::Image &message)
{
    if (!message.has_header() || !message.header().has_stamp()) {
        return 0;
    }
    return SmartDrone::Adapters::Simulation::GazeboMessageTimeToNs(
        message.header().stamp());
}

} // namespace

struct GazeboStereoCamera::Impl {
    explicit Impl(std::shared_ptr<
                  SmartDrone::Adapters::Simulation::GazeboMeasurementClock>
                      clockValue)
        : clock(std::move(clockValue))
    {
    }

    bool Open(const SmartDrone::Core::Ports::CameraOpenConfig &openConfig);
    void Close();
    bool Subscribe();
    GazeboRawImage BuildRawImage(const gz::msgs::Image &message,
                                 std::uint32_t sequence) const;
    void OnLeftImage(const gz::msgs::Image &message);
    void OnRightImage(const gz::msgs::Image &message);
    void OnClock(const gz::msgs::Clock &message);
    void NotifyFrameReady() const;

    std::shared_ptr<SmartDrone::Adapters::Simulation::GazeboMeasurementClock>
        clock;
    std::unique_ptr<gz::transport::Node> node;
    std::shared_ptr<GazeboStereoFrameAssembler> assembler;
    GazeboStereoConfig config;
    std::atomic<std::uint32_t> leftSequence{0};
    std::atomic<std::uint32_t> rightSequence{0};
    std::atomic<bool> open{false};
    std::shared_ptr<SmartDrone::Core::Ports::CameraFrameReadyCallback>
        frameReadyCallback;
};

bool GazeboStereoCamera::Impl::Subscribe()
{
    node = std::make_unique<gz::transport::Node>();
    const bool clockSubscribed = node->Subscribe(
        config.clockTopic, &GazeboStereoCamera::Impl::OnClock, this);
    const bool leftSubscribed = node->Subscribe(
        config.leftImageTopic, &GazeboStereoCamera::Impl::OnLeftImage, this);
    const bool rightSubscribed = node->Subscribe(
        config.rightImageTopic, &GazeboStereoCamera::Impl::OnRightImage, this);
    return clockSubscribed && leftSubscribed && rightSubscribed;
}

bool GazeboStereoCamera::Impl::Open(
    const SmartDrone::Core::Ports::CameraOpenConfig &openConfig)
{
    Close();
    const std::string configPath = SmartDrone::Common::EnvStringValue(
        "SMART_DRONE_SIM_CONFIG", "");
    const GazeboStereoConfigLoadResult loaded =
        LoadGazeboStereoConfig(configPath);
    if (!loaded.ok || openConfig.width <= 0 || openConfig.height <= 0) {
        std::cerr << "[gz_stereo] configuration failed: " << loaded.error
                  << "\n";
        return false;
    }
    config = loaded.config;
    auto newAssembler = std::make_shared<GazeboStereoFrameAssembler>(
        GazeboStereoAssemblerConfig{openConfig.width, openConfig.height,
                                    config});
    std::atomic_store_explicit(&assembler, newAssembler,
                               std::memory_order_release);
    if (!Subscribe()) {
        std::cerr << "[gz_stereo] failed to subscribe to Gazebo topics\n";
        Close();
        return false;
    }
    newAssembler->SetAcceptFrames(true);
    open.store(true, std::memory_order_release);
    return true;
}

void GazeboStereoCamera::Impl::Close()
{
    open.store(false, std::memory_order_release);
    auto currentAssembler = std::atomic_load_explicit(
        &assembler, std::memory_order_acquire);
    if (currentAssembler) {
        currentAssembler->SetAcceptFrames(false);
    }
    node.reset();
    std::atomic_store_explicit(
        &assembler, std::shared_ptr<GazeboStereoFrameAssembler>{},
        std::memory_order_release);
    leftSequence.store(0, std::memory_order_release);
    rightSequence.store(0, std::memory_order_release);
}

GazeboRawImage GazeboStereoCamera::Impl::BuildRawImage(
    const gz::msgs::Image &message, std::uint32_t sequence) const
{
    GazeboRawImage image;
    if (message.width() > static_cast<std::uint32_t>(
                              std::numeric_limits<int>::max()) ||
        message.height() > static_cast<std::uint32_t>(
                               std::numeric_limits<int>::max())) {
        return image;
    }
    const std::int64_t arrivalNs = NowMonotonicNs();
    const auto timing = clock->ResolveMeasurementStamp(
        ImageHeaderTimestamp(message), arrivalNs);
    image.width = static_cast<int>(message.width());
    image.height = static_cast<int>(message.height());
    image.step = static_cast<std::size_t>(message.step());
    image.pixelFormat = PixelFormat(message);
    image.measurementTimestampNs = timing.measurementNs;
    image.captureMonotonicNs = timing.captureMonotonicNs;
    image.arrivalMonotonicNs = arrivalNs;
    image.sequence = sequence;
    image.clockResetCounter = timing.resetCounter;
    image.payload = std::make_shared<std::vector<std::uint8_t>>(
        message.data().begin(), message.data().end());
    return image;
}

void GazeboStereoCamera::Impl::OnLeftImage(
    const gz::msgs::Image &message)
{
    auto currentAssembler = std::atomic_load_explicit(
        &assembler, std::memory_order_acquire);
    if (!open.load(std::memory_order_acquire) || !currentAssembler) {
        return;
    }
    const std::uint32_t sequence =
        leftSequence.fetch_add(1, std::memory_order_relaxed) + 1U;
    if (currentAssembler->PushImage(GazeboStereoEye::Left,
                                    BuildRawImage(message, sequence))) {
        NotifyFrameReady();
    }
}

void GazeboStereoCamera::Impl::OnRightImage(
    const gz::msgs::Image &message)
{
    auto currentAssembler = std::atomic_load_explicit(
        &assembler, std::memory_order_acquire);
    if (!open.load(std::memory_order_acquire) || !currentAssembler) {
        return;
    }
    const std::uint32_t sequence =
        rightSequence.fetch_add(1, std::memory_order_relaxed) + 1U;
    if (currentAssembler->PushImage(GazeboStereoEye::Right,
                                    BuildRawImage(message, sequence))) {
        NotifyFrameReady();
    }
}

void GazeboStereoCamera::Impl::NotifyFrameReady() const
{
    const auto callback = std::atomic_load_explicit(
        &frameReadyCallback, std::memory_order_acquire);
    if (callback && *callback) {
        (*callback)();
    }
}

void GazeboStereoCamera::Impl::OnClock(const gz::msgs::Clock &message)
{
    if (!open.load(std::memory_order_acquire)) {
        return;
    }
    const std::uint64_t measurementNs =
        SmartDrone::Adapters::Simulation::GazeboMessageTimeToNs(message.sim());
    if (measurementNs > 0) {
        const std::uint64_t simulationRealNs =
            SmartDrone::Adapters::Simulation::GazeboMessageTimeToNs(
                message.real());
        clock->Observe(measurementNs, simulationRealNs, NowMonotonicNs());
    }
}

GazeboStereoCamera::GazeboStereoCamera(
    std::shared_ptr<SmartDrone::Adapters::Simulation::GazeboMeasurementClock>
        measurementClock)
    : m_impl(std::make_unique<Impl>(std::move(measurementClock)))
{
}

GazeboStereoCamera::~GazeboStereoCamera()
{
    Close();
}

bool GazeboStereoCamera::Open(
    const SmartDrone::Core::Ports::CameraOpenConfig &config)
{
    return m_impl->clock && m_impl->Open(config);
}

void GazeboStereoCamera::Close()
{
    m_impl->Close();
}

bool GazeboStereoCamera::Start()
{
    return m_impl->open.load(std::memory_order_acquire);
}

void GazeboStereoCamera::Stop()
{
    Close();
}

bool GazeboStereoCamera::GrabStereo(
    SmartDrone::Core::Ports::StereoFrame &out, bool preferLatest,
    std::uint64_t minTimestampNs)
{
    auto assembler = std::atomic_load_explicit(
        &m_impl->assembler, std::memory_order_acquire);
    return m_impl->open.load(std::memory_order_acquire) && assembler &&
           assembler->GrabStereo(out, preferLatest, minTimestampNs);
}

SmartDrone::Core::Ports::CameraHealth GazeboStereoCamera::GetHealth() const
{
    auto assembler = std::atomic_load_explicit(
        &m_impl->assembler, std::memory_order_acquire);
    return assembler ? assembler->GetHealth()
                     : SmartDrone::Core::Ports::CameraHealth{false, 0};
}

SmartDrone::Core::Ports::CameraDiagnostics
GazeboStereoCamera::GetDiagnostics() const
{
    if (m_impl->clock) {
        m_impl->clock->DetectStall(NowMonotonicNs(), CLOCK_STALL_TIMEOUT_NS);
    }
    auto assembler = std::atomic_load_explicit(
        &m_impl->assembler, std::memory_order_acquire);
    auto diagnostics = assembler
                           ? assembler->GetDiagnostics()
                           : SmartDrone::Core::Ports::CameraDiagnostics{};
    if (m_impl->clock) {
        diagnostics.clockResetCounter = std::max(
            diagnostics.clockResetCounter, m_impl->clock->ResetCounter());
    }
    return diagnostics;
}

SmartDrone::Core::Ports::CameraProviderSemantics
GazeboStereoCamera::Semantics() const
{
    return SmartDrone::Core::Ports::CameraProviderSemantics::DualStreamPaired;
}

bool GazeboStereoCamera::SetFrameReadyCallback(
    SmartDrone::Core::Ports::CameraFrameReadyCallback callback)
{
    if (m_impl->open.load(std::memory_order_acquire)) {
        return false;
    }
    std::atomic_store_explicit(
        &m_impl->frameReadyCallback,
        std::make_shared<SmartDrone::Core::Ports::CameraFrameReadyCallback>(
            std::move(callback)),
        std::memory_order_release);
    return true;
}

} // namespace SmartDrone::Adapters::Camera
