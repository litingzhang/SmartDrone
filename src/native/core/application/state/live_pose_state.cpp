#include "core/application/state/live_pose_state.h"

#include <atomic>
#include <memory>
#include <utility>

#include "common/time_utils.h"

namespace SmartDrone::Core::Application {
namespace {

constexpr auto LIVE_POSE_WRITE_ORDER = std::memory_order_release;
constexpr auto LIVE_POSE_READ_ORDER = std::memory_order_acquire;

struct LivePoseData {
    UdpPeer latestPeer{};
    bool hasPeer{false};
    bool poseValid{false};
    uint8_t runtimeMode{RUNTIME_MODE_IDLE};
    uint8_t slamMode{RUNTIME_SLAM_MODE_MAPPING};
    uint8_t trackingState{0xFF};
    bool px4FlightStateValid{false};
    bool armed{false};
    uint8_t px4MainMode{0};
    uint8_t px4SubMode{0};
    LivePoseQuality poseQuality{LivePoseQuality::Lost};
    SmartDrone::Core::Ports::PoseReferenceFrame referenceFrame{
        SmartDrone::Core::Ports::PoseReferenceFrame::LocalNed};
    uint64_t poseUpdateUs{0};
    uint16_t resetCounter{0};
    uint16_t resetMapCount{0};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float qw{1.0f};
    float qx{0.0f};
    float qy{0.0f};
    float qz{0.0f};
    std::shared_ptr<const std::vector<float>> pointCloudXyz;
    uint32_t pointCloudSeq{0};
    uint64_t pointCloudUpdateUs{0};
    AvoidanceTelemetry avoidance{};
    uint32_t txSeq{1};
    bool dirty{false};
};

void CopyStateToSnapshot(const LivePoseData &state,
                         LivePoseState::Snapshot &out)
{
    out.hasPeer = state.hasPeer;
    out.peer = state.latestPeer;
    out.poseValid = state.poseValid;
    out.runtimeMode = state.runtimeMode;
    out.slamMode = state.slamMode;
    out.trackingState = state.trackingState;
    out.px4FlightStateValid = state.px4FlightStateValid;
    out.armed = state.armed;
    out.px4MainMode = state.px4MainMode;
    out.px4SubMode = state.px4SubMode;
    out.poseQuality = state.poseQuality;
    out.referenceFrame = state.referenceFrame;
    out.poseUpdateUs = state.poseUpdateUs;
    out.resetCounter = state.resetCounter;
    out.resetMapCount = state.resetMapCount;
    out.x = state.x;
    out.y = state.y;
    out.z = state.z;
    out.qw = state.qw;
    out.qx = state.qx;
    out.qy = state.qy;
    out.qz = state.qz;
    out.seq = state.txSeq;
    out.pointCloudXyz = state.pointCloudXyz;
    out.pointCloudSeq = state.pointCloudSeq;
    out.pointCloudUpdateUs = state.pointCloudUpdateUs;
    out.avoidance = state.avoidance;
}

} // namespace

class LivePoseState::Impl {
  public:
    Impl()
        : m_state(std::make_shared<LivePoseData>())
    {
    }

    std::shared_ptr<const LivePoseData> Load() const
    {
        return std::atomic_load_explicit(&m_state, LIVE_POSE_READ_ORDER);
    }

    bool TryPublish(std::shared_ptr<const LivePoseData> &current,
                    std::shared_ptr<const LivePoseData> published)
    {
        return std::atomic_compare_exchange_weak_explicit(
            &m_state, &current, std::move(published), LIVE_POSE_WRITE_ORDER,
            LIVE_POSE_READ_ORDER);
    }

    template <typename ApplyFn>
    void PublishUpdate(ApplyFn apply)
    {
        auto current = Load();
        while (current) {
            LivePoseData next = *current;
            apply(next);
            std::shared_ptr<const LivePoseData> published =
                std::make_shared<LivePoseData>(std::move(next));
            if (TryPublish(current, std::move(published))) {
                return;
            }
        }
    }

    std::shared_ptr<const LivePoseData> m_state;
};

LivePoseState::LivePoseState()
    : m_impl(std::make_unique<Impl>())
{
}

LivePoseState::~LivePoseState() = default;

void LivePoseState::UpdatePeer(const UdpPeer &peer)
{
    if (!peer.valid) {
        return;
    }

    m_impl->PublishUpdate([&peer](LivePoseData &state) {
        state.latestPeer = peer;
        state.hasPeer = true;
    });
}

void LivePoseState::SetRuntimeMode(uint8_t mode)
{
    m_impl->PublishUpdate([mode](LivePoseData &state) {
        state.runtimeMode = mode;
        if (mode != RUNTIME_MODE_SLAM) {
            state.poseValid = false;
            state.slamMode = RUNTIME_SLAM_MODE_MAPPING;
            state.trackingState = 0xFF;
            state.poseQuality = LivePoseQuality::Lost;
        }
        state.dirty = true;
    });
}

void LivePoseState::SetSlamMode(uint8_t mode)
{
    m_impl->PublishUpdate([mode](LivePoseData &state) {
        state.slamMode = mode;
        state.dirty = true;
    });
}

void LivePoseState::SetVehicleFlightState(bool validIn, bool armedIn,
                                          uint8_t px4MainModeIn,
                                          uint8_t px4SubModeIn)
{
    m_impl->PublishUpdate([validIn, armedIn, px4MainModeIn,
                           px4SubModeIn](LivePoseData &state) {
        state.px4FlightStateValid = validIn;
        if (!validIn) {
            state.dirty = true;
            return;
        }
        state.armed = armedIn;
        state.px4MainMode = px4MainModeIn;
        state.px4SubMode = px4SubModeIn;
        state.dirty = true;
    });
}

void LivePoseState::SetAvoidanceTelemetry(
    const AvoidanceTelemetry &telemetry)
{
    m_impl->PublishUpdate([&telemetry](LivePoseData &state) {
        state.avoidance = telemetry;
        state.dirty = true;
    });
}

void LivePoseState::UpdatePose(const LivePoseUpdate &update)
{
    m_impl->PublishUpdate([&update](LivePoseData &state) {
        state.runtimeMode = update.runtimeMode;
        state.trackingState = update.trackingState;
        state.poseQuality = update.quality;
        state.referenceFrame = update.referenceFrame;
        state.poseUpdateUs = MonoTimeUs();
        state.resetCounter = update.resetCounter;
        state.resetMapCount = update.resetMapCount;
        state.x = update.pose.x;
        state.y = update.pose.y;
        state.z = update.pose.z;
        state.qw = update.pose.qw;
        state.qx = update.pose.qx;
        state.qy = update.pose.qy;
        state.qz = update.pose.qz;
        state.poseValid = update.poseValid;
        state.dirty = true;
    });
}

void LivePoseState::UpdatePointCloud(std::vector<float> xyz)
{
    auto pointCloud = std::make_shared<const std::vector<float>>(std::move(xyz));
    const uint64_t updateUs = MonoTimeUs();
    m_impl->PublishUpdate([pointCloud, updateUs](LivePoseData &state) {
        state.pointCloudXyz = pointCloud;
        ++state.pointCloudSeq;
        state.pointCloudUpdateUs = updateUs;
        state.dirty = true;
    });
}

bool LivePoseState::ConsumeSnapshot(Snapshot &out)
{
    while (true) {
        auto current = m_impl->Load();
        if (!current || !current->hasPeer || !current->dirty) {
            return false;
        }

        LivePoseData next = *current;
        ++next.txSeq;
        next.dirty = false;
        std::shared_ptr<const LivePoseData> published =
            std::make_shared<LivePoseData>(std::move(next));
        if (m_impl->TryPublish(current, published)) {
            CopyStateToSnapshot(*published, out);
            return true;
        }
    }
}

bool LivePoseState::ReadSnapshot(Snapshot &out) const
{
    const auto current = m_impl->Load();
    if (!current || !current->hasPeer) {
        return false;
    }

    CopyStateToSnapshot(*current, out);
    return true;
}

} // namespace SmartDrone::Core::Application
