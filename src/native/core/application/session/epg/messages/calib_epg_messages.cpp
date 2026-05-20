#include "core/application/session/epg/messages/calib_epg_messages.h"

#include "common/epg/epg.h"

namespace smartdrone::core::application {

EPG_REGISTER_MESSAGE(CalibResourceReady, "CalibResourceReady")
EPG_REGISTER_MESSAGE(CalibTick, "CalibTick")
EPG_REGISTER_MESSAGE(CalibStereoFrame, "CalibStereoFrame")
EPG_REGISTER_MESSAGE(CalibSavePair, "CalibSavePair")
EPG_REGISTER_MESSAGE(CalibCaptureDone, "CalibCaptureDone")
EPG_REGISTER_MESSAGE(CalibStopRequest, "CalibStopRequest")
EPG_REGISTER_MESSAGE(CalibStorageStatus, "CalibStorageStatus")
EPG_REGISTER_MESSAGE(CalibImuStatus, "CalibImuStatus")
EPG_REGISTER_MESSAGE(CalibPreviewStatus, "CalibPreviewStatus")
EPG_REGISTER_MESSAGE(CalibFlushRequest, "CalibFlushRequest")
EPG_REGISTER_MESSAGE(CalibStatus, "CalibStatus")

} // namespace smartdrone::core::application
