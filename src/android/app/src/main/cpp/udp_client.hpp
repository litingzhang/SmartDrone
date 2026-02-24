#pragma once
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

class UdpClient
{
public:
    bool open(const std::string& ip, uint16_t port)
    {
        close();

        m_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (m_fd < 0) return false;

        std::memset(&m_dst, 0, sizeof(m_dst));
        m_dst.sin_family = AF_INET;
        m_dst.sin_port = htons(port);
        if (::inet_pton(AF_INET, ip.c_str(), &m_dst.sin_addr) != 1)
        {
            close();
            return false;
        }

        // optional: bind local port for receiving
        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_addr.s_addr = htonl(INADDR_ANY);
        local.sin_port = htons(0); // auto
        ::bind(m_fd, (sockaddr*)&local, sizeof(local));

        // non-blocking recv
        int flags = ::fcntl(m_fd, F_GETFL, 0);
        ::fcntl(m_fd, F_SETFL, flags | O_NONBLOCK);

        return true;
    }

    void close()
    {
        if (m_fd >= 0) ::close(m_fd);
        m_fd = -1;
        std::memset(&m_dst, 0, sizeof(m_dst));
    }

    bool send(const uint8_t* data, size_t len)
    {
        if (m_fd < 0) return false;
        const ssize_t n = ::sendto(m_fd, data, len, 0, (sockaddr*)&m_dst, sizeof(m_dst));
        return (n == (ssize_t)len);
    }

    // returns bytes received (0 if none)
    int recv(uint8_t* out, size_t cap)
    {
        if (m_fd < 0) return -1;
        sockaddr_in src{};
        socklen_t slen = sizeof(src);
        const ssize_t n = ::recvfrom(m_fd, out, cap, 0, (sockaddr*)&src, &slen);
        if (n < 0) return 0; // non-blocking: no data
        return (int)n;
    }

private:
    int m_fd{-1};
    sockaddr_in m_dst{};
};