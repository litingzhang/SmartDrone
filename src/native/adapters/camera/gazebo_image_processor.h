#pragma once

#include <cstdint>

#include "adapters/camera/gazebo_stereo_types.h"
#include "core/ports/camera_provider.h"

namespace SmartDrone::Adapters::Camera {

class GazeboImageProcessor final {
  public:
    explicit GazeboImageProcessor(GazeboImageFaultConfig config);

    void UpdateConfig(GazeboImageFaultConfig config);
    std::uint64_t Generation() const;
    bool ShouldDrop(std::uint64_t pairMeasurementTimestampNs) const;
    bool Convert(const GazeboRawImage &image, GazeboStereoEye eye,
                 SmartDrone::Core::Ports::ImageFrame &out);

  private:
    bool BuildGray(const GazeboRawImage &image, cv::Mat &gray) const;
    bool EffectsEnabled() const;
    void ApplyEffects(cv::Mat &gray, const GazeboRawImage &image,
                      GazeboStereoEye eye);
    void ApplyNoise(cv::Mat &gray, const GazeboRawImage &image,
                    GazeboStereoEye eye) const;
    bool BlackoutActive(std::uint64_t measurementTimestampNs);

    GazeboImageFaultConfig m_config;
    std::uint64_t m_firstMeasurementTimestampNs{0};
};

} // namespace SmartDrone::Adapters::Camera
