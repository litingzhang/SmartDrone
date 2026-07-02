#include "adapters/telemetry/mavlink_serial_transport.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace SmartDrone::Adapters::Telemetry {
namespace {

constexpr int INVALID_FD = -1;

std::string QueryPart(const std::string &text)
{
    const std::size_t pos = text.find('?');
    if (pos == std::string::npos) {
        return {};
    }
    return text.substr(pos + 1);
}

std::string PathPart(const std::string &text)
{
    const std::size_t pos = text.find('?');
    if (pos == std::string::npos) {
        return text;
    }
    return text.substr(0, pos);
}

} // namespace

MavlinkSerialTransport::MavlinkSerialTransport(const std::string &dev,
                                               int baud)
{
    Open(dev, baud);
}

MavlinkSerialTransport::~MavlinkSerialTransport()
{
    Close();
}

void MavlinkSerialTransport::Open(const std::string &dev, int baud)
{
    Close();
    if (IsUdpEndpoint(dev)) {
        OpenUdp(dev);
        return;
    }
    OpenSerial(dev, baud);
}

void MavlinkSerialTransport::Close()
{
    if (m_fd >= 0) {
        ::close(m_fd);
    }
    m_fd = INVALID_FD;
    m_mode = TransportMode::Closed;
    ResetUdpPeer();
}

ssize_t MavlinkSerialTransport::WriteSome(const uint8_t *data,
                                          size_t len) const
{
    if (m_mode == TransportMode::Udp) {
        return WriteUdp(data, len);
    }
    if (m_fd < 0 || data == nullptr) {
        return -1;
    }
    return ::write(m_fd, data, len);
}

int MavlinkSerialTransport::PollReadable() const
{
    if (m_fd < 0) {
        return -1;
    }
    pollfd pfd{};
    pfd.fd = m_fd;
    pfd.events = POLLIN;
    return ::poll(&pfd, 1, 0);
}

ssize_t MavlinkSerialTransport::Read(uint8_t *buffer, size_t len) const
{
    if (m_mode == TransportMode::Udp) {
        return ReadUdp(buffer, len);
    }
    if (m_fd < 0 || buffer == nullptr || len == 0) {
        return -1;
    }
    return ::read(m_fd, buffer, len);
}

void MavlinkSerialTransport::OpenSerial(const std::string &dev, int baud)
{
    m_fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_fd < 0) {
        throw std::runtime_error("open(" + dev + ") failed: " +
                                 std::string(std::strerror(errno)));
    }
    termios tio{};
    if (tcgetattr(m_fd, &tio) != 0) {
        throw std::runtime_error("tcgetattr failed: " +
                                 std::string(std::strerror(errno)));
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
        throw std::runtime_error("tcsetattr failed: " +
                                 std::string(std::strerror(errno)));
    }
    const int flags = fcntl(m_fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(m_fd, F_SETFL, flags | O_NONBLOCK);
    }
    m_mode = TransportMode::Serial;
}

void MavlinkSerialTransport::OpenUdp(const std::string &dev)
{
    const UdpOpenConfig config = ParseUdpEndpoint(dev);
    sockaddr_storage bindAddr{};
    socklen_t bindLen = 0;
    if (!ResolveSockaddr(config.bindHost, config.bindPort, true, bindAddr,
                         bindLen)) {
        throw std::runtime_error("resolve udp bind failed: " + dev);
    }
    m_fd = ::socket(bindAddr.ss_family, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (m_fd < 0) {
        throw std::runtime_error("udp socket failed: " +
                                 std::string(std::strerror(errno)));
    }
    if (::bind(m_fd, reinterpret_cast<sockaddr *>(&bindAddr), bindLen) < 0) {
        throw std::runtime_error("udp bind failed: " +
                                 std::string(std::strerror(errno)));
    }
    if (config.hasRemote &&
        !ResolveSockaddr(config.remoteHost, config.remotePort, false,
                         m_udpRemote, m_udpRemoteLen)) {
        throw std::runtime_error("resolve udp remote failed: " + dev);
    }
    m_udpHaveRemote.store(config.hasRemote, std::memory_order_release);
    m_mode = TransportMode::Udp;
}

void MavlinkSerialTransport::ResetUdpPeer()
{
    m_udpRemote = sockaddr_storage{};
    m_udpRemoteLen = 0;
    m_udpHaveRemote.store(false, std::memory_order_release);
}

ssize_t MavlinkSerialTransport::WriteUdp(const uint8_t *data,
                                         size_t len) const
{
    if (m_fd < 0 || data == nullptr ||
        !m_udpHaveRemote.load(std::memory_order_acquire)) {
        errno = EAGAIN;
        return -1;
    }
    return ::sendto(m_fd, data, len, MSG_DONTWAIT,
                    reinterpret_cast<const sockaddr *>(&m_udpRemote),
                    m_udpRemoteLen);
}

ssize_t MavlinkSerialTransport::ReadUdp(uint8_t *buffer, size_t len) const
{
    if (m_fd < 0 || buffer == nullptr || len == 0) {
        return -1;
    }
    sockaddr_storage peer{};
    socklen_t peerLen = sizeof(peer);
    const ssize_t readLength =
        ::recvfrom(m_fd, buffer, len, MSG_DONTWAIT,
                   reinterpret_cast<sockaddr *>(&peer), &peerLen);
    if (readLength > 0 &&
        !m_udpHaveRemote.load(std::memory_order_acquire)) {
        m_udpRemote = peer;
        m_udpRemoteLen = peerLen;
        m_udpHaveRemote.store(true, std::memory_order_release);
    }
    return readLength;
}

bool MavlinkSerialTransport::IsUdpEndpoint(const std::string &dev)
{
    return dev.rfind("udp://", 0) == 0 ||
           dev.rfind("udp-listen://", 0) == 0;
}

MavlinkSerialTransport::UdpOpenConfig
MavlinkSerialTransport::ParseUdpEndpoint(const std::string &dev)
{
    UdpOpenConfig config{};
    const std::string path = StripUdpPrefix(PathPart(dev));
    const std::string query = QueryPart(dev);
    const std::string bind = ExtractBindValue(query);
    if (!bind.empty() &&
        !ParseHostPort(bind, config.bindHost, config.bindPort)) {
        throw std::runtime_error("invalid udp bind endpoint: " + bind);
    }
    if (dev.rfind("udp-listen://", 0) == 0) {
        config.bindHost = "0.0.0.0";
        if (!ParsePort(path, config.bindPort)) {
            throw std::runtime_error("invalid udp listen port: " + path);
        }
        config.hasRemote = false;
        return config;
    }
    if (!ParseHostPort(path, config.remoteHost, config.remotePort)) {
        throw std::runtime_error("invalid udp endpoint: " + dev);
    }
    config.hasRemote = true;
    return config;
}

bool MavlinkSerialTransport::ParseHostPort(const std::string &text,
                                           std::string &host, uint16_t &port)
{
    const std::size_t pos = text.rfind(':');
    if (pos == std::string::npos || pos == 0 || pos + 1 >= text.size()) {
        return false;
    }
    host = text.substr(0, pos);
    return ParsePort(text.substr(pos + 1), port);
}

bool MavlinkSerialTransport::ParsePort(const std::string &text,
                                       uint16_t &port)
{
    char *end = nullptr;
    errno = 0;
    const long parsed = std::strtol(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0') {
        return false;
    }
    if (parsed < 0 || parsed > 65535) {
        return false;
    }
    port = static_cast<uint16_t>(parsed);
    return true;
}

std::string MavlinkSerialTransport::StripUdpPrefix(const std::string &dev)
{
    if (dev.rfind("udp-listen://", 0) == 0) {
        return dev.substr(std::string("udp-listen://").size());
    }
    if (dev.rfind("udp://", 0) == 0) {
        return dev.substr(std::string("udp://").size());
    }
    return dev;
}

std::string MavlinkSerialTransport::ExtractBindValue(const std::string &query)
{
    const std::string key = "bind=";
    const std::size_t pos = query.find(key);
    if (pos == std::string::npos) {
        return {};
    }
    const std::size_t valueStart = pos + key.size();
    const std::size_t valueEnd = query.find('&', valueStart);
    if (valueEnd == std::string::npos) {
        return query.substr(valueStart);
    }
    return query.substr(valueStart, valueEnd - valueStart);
}

bool MavlinkSerialTransport::ResolveSockaddr(const std::string &host,
                                             uint16_t port, bool passive,
                                             sockaddr_storage &out,
                                             socklen_t &outLen)
{
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = passive ? AI_PASSIVE : 0;
    addrinfo *result = nullptr;
    const std::string portText = std::to_string(port);
    const char *node = host.empty() ? nullptr : host.c_str();
    const int rc = getaddrinfo(node, portText.c_str(), &hints, &result);
    if (rc != 0 || result == nullptr) {
        return false;
    }
    std::memcpy(&out, result->ai_addr, result->ai_addrlen);
    outLen = static_cast<socklen_t>(result->ai_addrlen);
    freeaddrinfo(result);
    return true;
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
        throw std::runtime_error("Unsupported baud for termios: " +
                                 std::to_string(baud));
    }
}

} // namespace SmartDrone::Adapters::Telemetry
