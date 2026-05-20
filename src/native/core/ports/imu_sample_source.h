#pragma once

#include <cstdint>

namespace smartdrone::core::ports {

enum class ImuSampleReadStatus : std::uint8_t {
    Ready,
    Pending,
    Failed,
};

} // namespace smartdrone::core::ports

struct ImuSample;
struct ImuScale;

namespace smartdrone::core::ports {

class IImuSampleSource {
  public:
    virtual ~IImuSampleSource() = default;

    virtual bool Start() = 0;
    virtual bool EnsureOpen() = 0;
    virtual void Stop() = 0;
    virtual ImuSampleReadStatus ReadSample(ImuSample &sample) = 0;
    virtual ImuScale Scale() const = 0;
    virtual bool Failed() const = 0;
};

} // namespace smartdrone::core::ports
