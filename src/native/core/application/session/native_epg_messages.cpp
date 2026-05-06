#include "core/application/session/native_epg_messages.h"

namespace smartdrone::core::application {

EPG_REGISTER_MESSAGE(NativeSlamResourceReady, "NativeSlamResourceReady")
EPG_REGISTER_MESSAGE(NativeSlamTick, "NativeSlamTick")
EPG_REGISTER_MESSAGE(NativeSlamFrameReady, "NativeSlamFrameReady")
EPG_REGISTER_MESSAGE(NativeSlamPreparedFrame, "NativeSlamPreparedFrame")
EPG_REGISTER_MESSAGE(NativeSlamTrackedFrame, "NativeSlamTrackedFrame")
EPG_REGISTER_MESSAGE(NativeSlamPublishedFrame, "NativeSlamPublishedFrame")
EPG_REGISTER_MESSAGE(NativeSlamStatus, "NativeSlamStatus")

EPG_REGISTER_MESSAGE(CalibResourceReady, "CalibResourceReady")
EPG_REGISTER_MESSAGE(NativeCalibTick, "NativeCalibTick")
EPG_REGISTER_MESSAGE(CalibStereoFrame, "CalibStereoFrame")
EPG_REGISTER_MESSAGE(CalibSavePair, "CalibSavePair")
EPG_REGISTER_MESSAGE(CalibCaptureDone, "CalibCaptureDone")
EPG_REGISTER_MESSAGE(CalibStorageStatus, "CalibStorageStatus")
EPG_REGISTER_MESSAGE(CalibImuStatus, "CalibImuStatus")
EPG_REGISTER_MESSAGE(CalibPreviewStatus, "CalibPreviewStatus")
EPG_REGISTER_MESSAGE(CalibFlushRequest, "CalibFlushRequest")
EPG_REGISTER_MESSAGE(NativeCalibStatus, "NativeCalibStatus")

} // namespace smartdrone::core::application
