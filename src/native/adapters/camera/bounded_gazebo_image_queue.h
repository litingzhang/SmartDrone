#pragma once

#include <cstddef>
#include <memory>

#include "adapters/camera/gazebo_stereo_types.h"

namespace SmartDrone::Adapters::Camera {

class BoundedGazeboImageQueue final {
  public:
    explicit BoundedGazeboImageQueue(std::size_t capacity);
    ~BoundedGazeboImageQueue();

    bool TryEnqueue(const GazeboRawImage &image);
    bool TryDequeue(GazeboRawImage &image);
    std::size_t Size() const;
    std::size_t Capacity() const;

  private:
    struct Impl;

    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Adapters::Camera
