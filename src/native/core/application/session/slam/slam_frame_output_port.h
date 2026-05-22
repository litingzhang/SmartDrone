#pragma once

#include <iosfwd>
#include <string>

#include "core/application/session/slam/slam_frame_stage_context.h"
#include "core/application/session/slam/slam_frame_stage_data.h"
#include "core/application/session/slam/slam_frame_stage_state.h"
#include "core/application/session/slam/slam_frame_step_result.h"

namespace SmartDrone::Core::Application {

class SlamFrameOutputPort final {
  public:
    SlamFrameOutputPort(SlamFrameOutputContext &context,
                        SlamFrameOutputState &state);

    SlamFrameStepResult EmitPointCloud(SlamPublishedFrameData &published);
    SlamFrameStepResult EmitLivePose(SlamPublishedFrameData &published);
    SlamFrameStepResult EmitMavlink(SlamPublishedFrameData &published);
    SlamFrameStepResult EmitUdp(SlamPublishedFrameData &published);
    SlamFrameStepResult EmitDfx(SlamPublishedFrameData &published);

  private:
    struct DfxTiming {
        double acquireMs{0.0};
        double imuMs{0.0};
        double slamMs{0.0};
        double cloudMs{0.0};
        double udpMs{0.0};
        double postMs{0.0};
        double livePoseMs{0.0};
        double publishMs{0.0};
        double totalMs{0.0};
    };

    struct DfxSample {
        SlamPreparedFrameData &frame;
        SlamTrackedFrameData &tracked;
        SlamPublishedFrameData &published;
        SmartDrone::Core::Ports::SlamInputBatch &slamInput;
        SmartDrone::Core::Ports::SlamOutput &slamOutput;
        const PosePostprocessor::Result &poseResult;
        DfxTiming timing;
        bool visualFeatureStereoWeak{false};
    };

    DfxSample MakeDfxSample(SlamPublishedFrameData &published) const;
    DfxTiming MakeDfxTiming(const SlamPreparedFrameData &frame,
                            const SlamTrackedFrameData &tracked,
                            const SlamPublishedFrameData &published) const;
    bool ShouldLogDfx(const DfxSample &sample) const;
    void LogDfxLine(const DfxSample &sample) const;
    void UpdateDfxAverages(const DfxSample &sample);
    std::string BuildJsonDfxLine(const DfxSample &sample) const;
    std::string BuildTextDfxLine(const DfxSample &sample) const;
    void AppendDfxJsonCore(std::ostream &out, const DfxSample &sample) const;
    void AppendDfxJsonBackend(std::ostream &out,
                              const DfxSample &sample) const;
    void AppendDfxJsonVisual(std::ostream &out, const DfxSample &sample) const;
    void AppendDfxJsonFrameStats(std::ostream &out,
                                 const DfxSample &sample) const;
    void AppendDfxJsonTiming(std::ostream &out,
                             const DfxSample &sample) const;
    void AppendDfxTextCore(std::ostream &out, const DfxSample &sample) const;
    void AppendDfxTextBackend(std::ostream &out,
                              const DfxSample &sample) const;
    void AppendDfxTextVisual(std::ostream &out, const DfxSample &sample) const;
    void AppendDfxTextFrameStats(std::ostream &out,
                                 const DfxSample &sample) const;
    void AppendDfxTextTiming(std::ostream &out,
                             const DfxSample &sample) const;

    SlamFrameOutputContext &m_ctx;
    SlamFrameOutputState &m_state;
};

} // namespace SmartDrone::Core::Application
