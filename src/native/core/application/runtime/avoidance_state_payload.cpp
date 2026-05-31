#include "core/application/runtime/avoidance_state_payload.h"

#include <algorithm>

#include "common/time_utils.h"
#include "common/tlv/tlv_pack.h"

namespace SmartDrone::Core::Application {
namespace {

void WriteAvoidanceFlags(std::vector<uint8_t> &payload,
                         const AvoidanceTelemetry &telemetry)
{
    payload.push_back(telemetry.enabled ? 1u : 0u);
    payload.push_back(telemetry.activeGoal ? 1u : 0u);
    payload.push_back(telemetry.holding ? 1u : 0u);
    payload.push_back(telemetry.holding
                          ? static_cast<uint8_t>(telemetry.holdReason)
                          : 0u);
}

uint32_t PointCloudAgeMs(const UdpRuntimeStateSnapshot &snapshot)
{
    if (snapshot.pointCloudUpdateUs == 0) {
        return snapshot.avoidance.pointCloudAgeMs;
    }
    const uint64_t nowUs = MonoTimeUs();
    if (snapshot.pointCloudUpdateUs >= nowUs) {
        return 0;
    }
    return static_cast<uint32_t>(
        std::min<uint64_t>(0xFFFFFFFFULL,
                           (nowUs - snapshot.pointCloudUpdateUs) / 1000ULL));
}

uint32_t PointCloudPointCount(const UdpRuntimeStateSnapshot &snapshot)
{
    if (!snapshot.pointCloudXyz) {
        return 0;
    }
    const size_t pointCount = snapshot.pointCloudXyz->size() / 3;
    return static_cast<uint32_t>(
        std::min<size_t>(0xFFFFFFFFULL, pointCount));
}

} // namespace

std::vector<uint8_t> BuildAvoidanceStatePayload(
    const UdpRuntimeStateSnapshot &snapshot)
{
    std::vector<uint8_t> payload;
    payload.reserve(AVOIDANCE_STATE_PAYLOAD_LEN);
    WriteAvoidanceFlags(payload, snapshot.avoidance);
    WriteF32Le(payload, snapshot.avoidance.nearestObstacleM);
    WriteU32Le(payload, snapshot.avoidance.holdCount);
    WriteU32Le(payload, PointCloudAgeMs(snapshot));
    WriteU32Le(payload, snapshot.pointCloudSeq);
    WriteU32Le(payload, PointCloudPointCount(snapshot));
    return payload;
}

} // namespace SmartDrone::Core::Application
