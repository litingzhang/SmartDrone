#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

class UdpClient {
public:
    ~UdpClient() { Close(); }

    bool Open(const std::string& ip, uint16_t port)
    {
        Close();

        m_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (m_fd < 0) {
            return false;
        }

        std::memset(&m_dstAddr, 0, sizeof(m_dstAddr));
        m_dstAddr.sin_family = AF_INET;
        m_dstAddr.sin_port = htons(port);
        if (::inet_pton(AF_INET, ip.c_str(), &m_dstAddr.sin_addr) != 1) {
            Close();
            return false;
        }

        sockaddr_in localAddr{};
        localAddr.sin_family = AF_INET;
        localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        // Prefer binding to the same port so server-side video sender can target a fixed port.
        localAddr.sin_port = htons(port);
        if (::bind(m_fd, reinterpret_cast<sockaddr*>(&localAddr), sizeof(localAddr)) != 0) {
            localAddr.sin_port = htons(0);
            ::bind(m_fd, reinterpret_cast<sockaddr*>(&localAddr), sizeof(localAddr));
        }

        const int flags = ::fcntl(m_fd, F_GETFL, 0);
        ::fcntl(m_fd, F_SETFL, flags | O_NONBLOCK);
        return true;
    }

    void Close()
    {
        if (m_fd >= 0) {
            ::close(m_fd);
            m_fd = -1;
        }
        std::memset(&m_dstAddr, 0, sizeof(m_dstAddr));
    }

    bool Send(const uint8_t* data, size_t len)
    {
        if (m_fd < 0) {
            return false;
        }
        const ssize_t sentLen = ::sendto(
            m_fd,
            data,
            len,
            0,
            reinterpret_cast<sockaddr*>(&m_dstAddr),
            sizeof(m_dstAddr));
        return sentLen == static_cast<ssize_t>(len);
    }

    // Returns bytes received (0 if none)
    int Recv(uint8_t* out, size_t cap)
    {
        if (m_fd < 0) {
            return -1;
        }
        sockaddr_in srcAddr{};
        socklen_t srcLen = sizeof(srcAddr);
        const ssize_t recvLen =
            ::recvfrom(m_fd, out, cap, 0, reinterpret_cast<sockaddr*>(&srcAddr), &srcLen);
        if (recvLen < 0) {
            return 0;
        }
        return static_cast<int>(recvLen);
    }

private:
    int m_fd{-1};
    sockaddr_in m_dstAddr{};
};
