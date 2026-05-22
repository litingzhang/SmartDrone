#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <sys/types.h>

namespace SmartDrone::Adapters::Telemetry {

class MavlinkSerialTransport {
  public:
    MavlinkSerialTransport() = default;
    MavlinkSerialTransport(const std::string &dev, int baud);
    ~MavlinkSerialTransport();

    void Open(const std::string &dev, int baud);
    void Close();
    ssize_t WriteSome(const uint8_t *data, size_t len) const;
    int PollReadable(int timeoutMs) const;
    ssize_t Read(uint8_t *buffer, size_t len) const;

  private:
    static unsigned int BaudToTermios(int baud);

    int m_fd{-1};
};

} // namespace SmartDrone::Adapters::Telemetry
