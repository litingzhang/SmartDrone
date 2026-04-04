#include "adapters/telemetry/mavlink_serial_transport.h"

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace smartdrone::adapters::telemetry {

MavlinkSerialTransport::MavlinkSerialTransport(const std::string &dev, int baud) { Open(dev, baud); }

MavlinkSerialTransport::~MavlinkSerialTransport() { Close(); }

void MavlinkSerialTransport::Open(const std::string &dev, int baud)
{
    Close();
    m_fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_fd < 0) {
        throw std::runtime_error("open(" + dev + ") failed: " + std::string(std::strerror(errno)));
    }

    termios tio{};
    if (tcgetattr(m_fd, &tio) != 0) {
        throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
    }

    cfmakeraw(&tio);
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);

    const speed_t spd = static_cast<speed_t>(BaudToTermios(baud));
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(m_fd, TCSANOW, &tio) != 0) {
        throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
    }

    const int flags = fcntl(m_fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(m_fd, F_SETFL, flags | O_NONBLOCK);
    }
}

void MavlinkSerialTransport::Close()
{
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
}

bool MavlinkSerialTransport::WriteAll(const uint8_t *data, size_t len, int timeoutMs)
{
    if (m_fd < 0 || data == nullptr) {
        return false;
    }

    size_t written = 0;
    while (written < len) {
        pollfd pfd{};
        pfd.fd = m_fd;
        pfd.events = POLLOUT;

        const int pr = ::poll(&pfd, 1, timeoutMs);
        if (pr == 0) {
            return false;
        }
        if (pr < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            return false;
        }
        if ((pfd.revents & POLLOUT) == 0) {
            continue;
        }

        const ssize_t n = ::write(m_fd, data + written, len - written);
        if (n > 0) {
            written += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            continue;
        }
        return false;
    }
    return true;
}

int MavlinkSerialTransport::PollReadable(int timeoutMs) const
{
    if (m_fd < 0) {
        return -1;
    }

    pollfd pfd{};
    pfd.fd = m_fd;
    pfd.events = POLLIN;
    return ::poll(&pfd, 1, timeoutMs);
}

ssize_t MavlinkSerialTransport::Read(uint8_t *buffer, size_t len) const
{
    if (m_fd < 0 || buffer == nullptr || len == 0) {
        return -1;
    }
    return ::read(m_fd, buffer, len);
}

unsigned int MavlinkSerialTransport::BaudToTermios(int baud)
{
    switch (baud) {
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 921600:
        return B921600;
    default:
        throw std::runtime_error("Unsupported baud for termios: " + std::to_string(baud));
    }
}

} // namespace smartdrone::adapters::telemetry
