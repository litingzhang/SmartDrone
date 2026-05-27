#include "core/application/session/epg/messages/slam_epg_messages.h"

#include "common/epg/epg.h"

namespace SmartDrone::Core::Application {

const bool SLAM_RESOURCE_READY_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<SlamResourceReady>(
        "SlamResourceReady");
const bool SLAM_TICK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<SlamTick>("SlamTick");
const bool SLAM_IMU_READY_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<SlamImuReady>("SlamImuReady");
const bool SLAM_FRAME_READY_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<SlamFrameReady>(
        "SlamFrameReady");
const bool SLAM_PREPARED_FRAME_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<SlamPreparedFrame>(
        "SlamPreparedFrame");
const bool SLAM_KLT_PREPARED_FRAME_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<SlamKltPreparedFrame>(
        "SlamKltPreparedFrame");
const bool SLAM_DPVO_PREPARED_FRAME_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<SlamDpvoPreparedFrame>(
        "SlamDpvoPreparedFrame");
const bool SLAM_ORB_PREPARED_FRAME_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<SlamOrbPreparedFrame>(
        "SlamOrbPreparedFrame");
const bool SLAM_OPENVINS_PREPARED_FRAME_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<SlamOpenVinsPreparedFrame>(
        "SlamOpenVinsPreparedFrame");
const bool SLAM_VISUAL_FEATURE_PREPARED_FRAME_REGISTERED =
    Epg::TypeCatalog::Global()
        .RegisterMessage<SlamVisualFeaturePreparedFrame>(
            "SlamVisualFeaturePreparedFrame");
const bool SLAM_TRACKED_FRAME_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<SlamTrackedFrame>(
        "SlamTrackedFrame");
const bool SLAM_PUBLISHED_FRAME_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<SlamPublishedFrame>(
        "SlamPublishedFrame");
const bool SLAM_PREVIEW_READY_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<SlamPreviewReady>(
        "SlamPreviewReady");
const bool SLAM_STATUS_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<SlamStatus>("SlamStatus");

} // namespace SmartDrone::Core::Application
