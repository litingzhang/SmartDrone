#pragma once

#include <cstdint>
#include <memory>

#include "adapters/camera/gazebo_stereo_types.h"
#include "core/ports/camera_provider.h"

namespace SmartDrone::Adapters::Camera {

struct GazeboStereoAssemblerConfig {
    int width{0};
    int height{0};
    GazeboStereoConfig simulation;
};

class GazeboStereoFrameAssembler final {
  public:
    explicit GazeboStereoFrameAssembler(
        GazeboStereoAssemblerConfig config);
    ~GazeboStereoFrameAssembler();

    void SetAcceptFrames(bool acceptFrames);
    bool PushImage(GazeboStereoEye eye, GazeboRawImage image);
    bool GrabStereo(SmartDrone::Core::Ports::StereoFrame &out,
                    bool preferLatest, std::uint64_t minTimestampNs = 0);
    SmartDrone::Core::Ports::CameraHealth GetHealth() const;
    SmartDrone::Core::Ports::CameraDiagnostics GetDiagnostics() const;
    void Clear();

  private:
    struct Impl;

    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Adapters::Camera
