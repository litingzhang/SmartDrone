#include "core/application/session/epg_messages.h"

namespace smartdrone::core::application {

EPG_REGISTER_MESSAGE(SlamResourceReady, "SlamResourceReady")
EPG_REGISTER_MESSAGE(SlamTick, "SlamTick")
EPG_REGISTER_MESSAGE(SlamFrameReady, "SlamFrameReady")
EPG_REGISTER_MESSAGE(SlamPreparedFrame, "SlamPreparedFrame")
EPG_REGISTER_MESSAGE(SlamTrackedFrame, "SlamTrackedFrame")
EPG_REGISTER_MESSAGE(SlamPublishedFrame, "SlamPublishedFrame")
EPG_REGISTER_MESSAGE(SlamStatus, "SlamStatus")

EPG_REGISTER_MESSAGE(CalibResourceReady, "CalibResourceReady")
EPG_REGISTER_MESSAGE(CalibTick, "CalibTick")
EPG_REGISTER_MESSAGE(CalibStereoFrame, "CalibStereoFrame")
EPG_REGISTER_MESSAGE(CalibSavePair, "CalibSavePair")
EPG_REGISTER_MESSAGE(CalibCaptureDone, "CalibCaptureDone")
EPG_REGISTER_MESSAGE(CalibStorageStatus, "CalibStorageStatus")
EPG_REGISTER_MESSAGE(CalibImuStatus, "CalibImuStatus")
EPG_REGISTER_MESSAGE(CalibPreviewStatus, "CalibPreviewStatus")
EPG_REGISTER_MESSAGE(CalibFlushRequest, "CalibFlushRequest")
EPG_REGISTER_MESSAGE(CalibStatus, "CalibStatus")

} // namespace smartdrone::core::application
