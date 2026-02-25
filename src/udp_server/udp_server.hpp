#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>

struct UdpPeer {
    sockaddr_in addr{};
    socklen_t len{sizeof(sockaddr_in)};
    bool valid{false};
};

class UdpServer {
public:
    ~UdpServer() { Close(); }

    bool Open(uint16_t port)
    {
        Close();

        m_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (m_fd < 0) {
            return false;
        }

        int one = 1;
        ::setsockopt(m_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_addr.s_addr = htonl(INADDR_ANY);
        local.sin_port = htons(port);

        if (::bind(m_fd, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
            Close();
            return false;
        }

        int flags = ::fcntl(m_fd, F_GETFL, 0);
        ::fcntl(m_fd, F_SETFL, flags | O_NONBLOCK);
        return true;
    }

    void Close()
    {
        if (m_fd >= 0) {
            ::close(m_fd);
            m_fd = -1;
        }
    }

    // Non-blocking receive: returns bytes, 0 if none, <0 if server not open.
    int Recv(uint8_t* buf, size_t cap, UdpPeer* peerOut)
    {
        if (m_fd < 0) {
            return -1;
        }

        sockaddr_in src{};
        socklen_t srcLen = sizeof(src);
        const ssize_t recvLen =
            ::recvfrom(m_fd, buf, cap, 0, reinterpret_cast<sockaddr*>(&src), &srcLen);
        if (recvLen < 0) {
            return 0;
        }

        if (peerOut != nullptr) {
            peerOut->addr = src;
            peerOut->len = srcLen;
            peerOut->valid = true;
        }
        return static_cast<int>(recvLen);
    }

    bool SendTo(const UdpPeer& peer, const uint8_t* data, size_t len)
    {
        if (m_fd < 0 || !peer.valid) {
            return false;
        }
        const ssize_t sentLen = ::sendto(
            m_fd,
            data,
            len,
            0,
            reinterpret_cast<const sockaddr*>(&peer.addr),
            peer.len);
        return sentLen == static_cast<ssize_t>(len);
    }

private:
    int m_fd{-1};
};
