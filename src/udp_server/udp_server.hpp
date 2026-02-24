#pragma once
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <cstdint>
#include <cstring>
#include <string>

struct UdpPeer
{
    sockaddr_in addr{};
    socklen_t len{sizeof(sockaddr_in)};
    bool valid{false};
};

class UdpServer
{
public:
    bool open(uint16_t port)
    {
        close();

        m_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (m_fd < 0) return false;

        int one = 1;
        ::setsockopt(m_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_addr.s_addr = htonl(INADDR_ANY);
        local.sin_port = htons(port);

        if (::bind(m_fd, (sockaddr*)&local, sizeof(local)) < 0)
        {
            close();
            return false;
        }

        int flags = ::fcntl(m_fd, F_GETFL, 0);
        ::fcntl(m_fd, F_SETFL, flags | O_NONBLOCK);
        return true;
    }

    void close()
    {
        if (m_fd >= 0) ::close(m_fd);
        m_fd = -1;
    }

    // non-blocking receive
    // returns bytes, 0 if none, <0 if error
    int recv(uint8_t* buf, size_t cap, UdpPeer* peerOut)
    {
        if (m_fd < 0) return -1;
        sockaddr_in src{};
        socklen_t slen = sizeof(src);
        const ssize_t n = ::recvfrom(m_fd, buf, cap, 0, (sockaddr*)&src, &slen);
        if (n < 0) return 0;

        if (peerOut)
        {
            peerOut->addr = src;
            peerOut->len = slen;
            peerOut->valid = true;
        }
        return (int)n;
    }

    bool sendTo(const UdpPeer& peer, const uint8_t* data, size_t len)
    {
        if (m_fd < 0 || !peer.valid) return false;
        const ssize_t n = ::sendto(m_fd, data, len, 0, (sockaddr*)&peer.addr, peer.len);
        return (n == (ssize_t)len);
    }

private:
    int m_fd{-1};
};