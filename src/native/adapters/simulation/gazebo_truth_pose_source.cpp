#include "adapters/simulation/gazebo_truth_pose_source.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include <gz/msgs/clock.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include "adapters/camera/gazebo_stereo_config.h"
#include "adapters/simulation/gazebo_measurement_clock.h"
#include "adapters/simulation/gazebo_message_time.h"
#include "adapters/simulation/gazebo_pose_math.h"
#include "common/environment.h"
#include "common/time_utils.h"
#include "core/ports/slam_tracking_state.h"

namespace SmartDrone::Adapters::Simulation {
namespace {

constexpr std::int64_t CLOCK_STALL_TIMEOUT_NS = 500000000LL;

std::int64_t NowMonotonicNs()
{
    return static_cast<std::int64_t>(MonoTimeUs() * 1000ULL);
}

} // namespace

struct GazeboTruthPoseSource::Impl {
    struct Snapshot {
        std::uint64_t sequence{0};
        SmartDrone::Core::Ports::PosePublishRequest request;
    };

    explicit Impl(std::shared_ptr<GazeboMeasurementClock> clockValue)
        : clock(std::move(clockValue))
    {
    }

    bool Open();
    void Close();
    void OnPoseVector(const gz::msgs::Pose_V &message);
    void OnClock(const gz::msgs::Clock &message);
    bool BuildRequest(const gz::msgs::Pose &pose,
                      const GazeboMeasurementStamp &timing,
                      SmartDrone::Core::Ports::PosePublishRequest &out);
    bool PublishRequest(
        SmartDrone::Core::Ports::PosePublishRequest request);
    void PopulateVelocity(
        const std::shared_ptr<const Snapshot> &previous,
        SmartDrone::Core::Ports::PosePublishRequest &request) const;

    std::shared_ptr<GazeboMeasurementClock> clock;
    std::unique_ptr<gz::transport::Node> node;
    std::string topic;
    std::string clockTopic;
    std::string modelName;
    std::shared_ptr<const Snapshot> latest;
    std::atomic<std::uint64_t> sequence{0};
    std::uint64_t lastReadSequence{0};
    std::atomic<bool> open{false};
    std::atomic<bool> healthy{false};
};

bool GazeboTruthPoseSource::Impl::Open()
{
    Close();
    const std::string configPath = SmartDrone::Common::EnvStringValue(
        "SMART_DRONE_SIM_CONFIG", "");
    const SmartDrone::Adapters::Camera::GazeboStereoConfigLoadResult loaded =
        SmartDrone::Adapters::Camera::LoadGazeboStereoConfig(configPath);
    if (!loaded.ok || loaded.config.truthPoseTopic.empty() ||
        loaded.config.truthModelName.empty()) {
        return false;
    }
    topic = loaded.config.truthPoseTopic;
    clockTopic = loaded.config.clockTopic;
    modelName = loaded.config.truthModelName;
    node = std::make_unique<gz::transport::Node>();
    open.store(true, std::memory_order_release);
    const bool clockSubscribed = node->Subscribe(
        clockTopic, &Impl::OnClock, this);
    const bool poseSubscribed = node->Subscribe(
        topic, &Impl::OnPoseVector, this);
    if (!clockSubscribed || !poseSubscribed) {
        Close();
        return false;
    }
    open.store(true, std::memory_order_release);
    healthy.store(true, std::memory_order_release);
    return true;
}

void GazeboTruthPoseSource::Impl::Close()
{
    open.store(false, std::memory_order_release);
    healthy.store(false, std::memory_order_release);
    node.reset();
    std::atomic_store_explicit(&latest, std::shared_ptr<const Snapshot>{},
                               std::memory_order_release);
    lastReadSequence = 0;
}

bool GazeboTruthPoseSource::Impl::BuildRequest(
    const gz::msgs::Pose &pose,
    const GazeboMeasurementStamp &timing,
    SmartDrone::Core::Ports::PosePublishRequest &out)
{
    if (!timing.clockValid || timing.measurementNs == 0) {
        return false;
    }
    out.frameId = sequence.fetch_add(1, std::memory_order_relaxed) + 1U;
    out.measurementTimestampNs = timing.measurementNs;
    out.captureMonotonicNs = static_cast<std::uint64_t>(
        std::max<std::int64_t>(0, timing.captureMonotonicNs));
    out.pose = ConvertGazeboPoseToNedFrd(
        {pose.position().x(), pose.position().y(), pose.position().z(),
         pose.orientation().w(), pose.orientation().x(),
         pose.orientation().y(), pose.orientation().z()});
    out.referenceFrame = SmartDrone::Core::Ports::PoseReferenceFrame::LocalNed;
    out.resetCounter = static_cast<std::uint8_t>(timing.resetCounter & 0xFFU);
    out.trackingState = SmartDrone::Core::Ports::SLAM_TRACKING_OK;
    out.quality = SmartDrone::Core::Ports::PoseQuality::Good;
    return true;
}

bool GazeboTruthPoseSource::Impl::PublishRequest(
    SmartDrone::Core::Ports::PosePublishRequest request)
{
    auto current = std::atomic_load_explicit(
        &latest, std::memory_order_acquire);
    while (true) {
        if (current && current->request.resetCounter == request.resetCounter &&
            current->request.measurementTimestampNs >=
                request.measurementTimestampNs) {
            return false;
        }
        auto candidate = std::make_shared<Snapshot>();
        candidate->request = request;
        PopulateVelocity(current, candidate->request);
        candidate->sequence = candidate->request.frameId;
        std::shared_ptr<const Snapshot> desired = std::move(candidate);
        if (std::atomic_compare_exchange_weak_explicit(
                &latest, &current, desired, std::memory_order_release,
                std::memory_order_acquire)) {
            healthy.store(true, std::memory_order_release);
            return true;
        }
    }
}

void GazeboTruthPoseSource::Impl::PopulateVelocity(
    const std::shared_ptr<const Snapshot> &previous,
    SmartDrone::Core::Ports::PosePublishRequest &request) const
{
    if (!previous) {
        return;
    }
    const GazeboNedPoseSample previousSample{
        previous->request.measurementTimestampNs,
        previous->request.resetCounter,
        previous->request.pose};
    const GazeboNedPoseSample currentSample{
        request.measurementTimestampNs, request.resetCounter, request.pose};
    request.velocity = EstimateGazeboNedVelocity(previousSample, currentSample);
}

void GazeboTruthPoseSource::Impl::OnClock(const gz::msgs::Clock &message)
{
    if (!open.load(std::memory_order_acquire)) {
        return;
    }
    const std::uint64_t measurementNs = GazeboMessageTimeToNs(message.sim());
    if (measurementNs > 0) {
        const std::uint64_t simulationRealNs =
            GazeboMessageTimeToNs(message.real());
        clock->Observe(measurementNs, simulationRealNs, NowMonotonicNs());
    }
}

void GazeboTruthPoseSource::Impl::OnPoseVector(
    const gz::msgs::Pose_V &message)
{
    if (!open.load(std::memory_order_acquire)) {
        return;
    }
    std::uint64_t measurementNs = 0;
    if (message.has_header() && message.header().has_stamp()) {
        const std::uint64_t headerNs =
            GazeboMessageTimeToNs(message.header().stamp());
        measurementNs = headerNs;
    }
    const GazeboMeasurementStamp timing = clock->ResolveMeasurementStamp(
        measurementNs, NowMonotonicNs());
    for (const auto &pose : message.pose()) {
        if (pose.name() != modelName) {
            continue;
        }
        SmartDrone::Core::Ports::PosePublishRequest request{};
        if (!BuildRequest(pose, timing, request)) {
            return;
        }
        (void)PublishRequest(std::move(request));
        return;
    }
}

GazeboTruthPoseSource::GazeboTruthPoseSource(
    std::shared_ptr<GazeboMeasurementClock> measurementClock)
    : m_impl(std::make_unique<Impl>(std::move(measurementClock)))
{
}

GazeboTruthPoseSource::~GazeboTruthPoseSource()
{
    Close();
}

bool GazeboTruthPoseSource::Open()
{
    return m_impl->clock && m_impl->Open();
}

void GazeboTruthPoseSource::Close()
{
    m_impl->Close();
}

bool GazeboTruthPoseSource::TryRead(
    SmartDrone::Core::Ports::PosePublishRequest &out)
{
    m_impl->clock->DetectStall(NowMonotonicNs(), CLOCK_STALL_TIMEOUT_NS);
    const auto snapshot = std::atomic_load_explicit(
        &m_impl->latest, std::memory_order_acquire);
    if (!snapshot || snapshot->sequence == m_impl->lastReadSequence) {
        return false;
    }
    m_impl->lastReadSequence = snapshot->sequence;
    out = snapshot->request;
    return true;
}

bool GazeboTruthPoseSource::Healthy() const
{
    return m_impl->healthy.load(std::memory_order_acquire);
}

} // namespace SmartDrone::Adapters::Simulation
