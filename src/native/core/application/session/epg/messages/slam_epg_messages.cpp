#include "core/application/session/epg/messages/slam_epg_messages.h"

#include "common/epg/epg.h"

namespace smartdrone::core::application {

EPG_REGISTER_MESSAGE(SlamResourceReady, "SlamResourceReady")
EPG_REGISTER_MESSAGE(SlamTick, "SlamTick")
EPG_REGISTER_MESSAGE(SlamImuReady, "SlamImuReady")
EPG_REGISTER_MESSAGE(SlamFrameReady, "SlamFrameReady")
EPG_REGISTER_MESSAGE(SlamPreparedFrame, "SlamPreparedFrame")
EPG_REGISTER_MESSAGE(SlamTrackedFrame, "SlamTrackedFrame")
EPG_REGISTER_MESSAGE(SlamPublishedFrame, "SlamPublishedFrame")
EPG_REGISTER_MESSAGE(SlamStatus, "SlamStatus")

} // namespace smartdrone::core::application
