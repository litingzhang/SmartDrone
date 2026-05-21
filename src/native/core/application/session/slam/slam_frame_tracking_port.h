#pragma once

#include <memory>

#include "core/application/session/slam/slam_frame_stage_context.h"
#include "core/application/session/slam/slam_frame_stage_data.h"
#include "core/application/session/slam/slam_frame_step_result.h"

namespace smartdrone::core::application {

class SlamFrameTrackingPort final {
  public:
    explicit SlamFrameTrackingPort(SlamFrameTrackingContext &context);

    SlamFrameStepResult TrackPreparedFrame(
        std::shared_ptr<SlamPreparedFrameData> frame,
        SlamTrackedFrameData &tracked);

  private:
    SlamFrameTrackingContext &m_ctx;
};

} // namespace smartdrone::core::application
