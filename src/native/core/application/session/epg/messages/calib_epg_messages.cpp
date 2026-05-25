#include "core/application/session/epg/messages/calib_epg_messages.h"

#include "common/epg/epg.h"

namespace SmartDrone::Core::Application {

const bool CALIB_RESOURCE_READY_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<CalibResourceReady>(
        "CalibResourceReady");
const bool CALIB_TICK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<CalibTick>("CalibTick");
const bool CALIB_STEREO_FRAME_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<CalibStereoFrame>(
        "CalibStereoFrame");
const bool CALIB_SAVE_PAIR_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<CalibSavePair>(
        "CalibSavePair");
const bool CALIB_CAPTURE_DONE_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<CalibCaptureDone>(
        "CalibCaptureDone");
const bool CALIB_STOP_REQUEST_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<CalibStopRequest>(
        "CalibStopRequest");
const bool CALIB_IMU_SAMPLE_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<CalibImuSample>(
        "CalibImuSample");
const bool CALIB_PREVIEW_STATUS_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<CalibPreviewStatus>(
        "CalibPreviewStatus");
const bool CALIB_FLUSH_REQUEST_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<CalibFlushRequest>(
        "CalibFlushRequest");
const bool CALIB_STORAGE_FLUSHED_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<CalibStorageFlushed>(
        "CalibStorageFlushed");
const bool CALIB_STATUS_REGISTERED =
    Epg::TypeCatalog::Global().RegisterMessage<CalibStatus>("CalibStatus");

} // namespace SmartDrone::Core::Application
