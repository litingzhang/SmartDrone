#pragma once

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <string>

struct UdpPeer {
    sockaddr_in addr{};
    socklen_t len{sizeof(sockaddr_in)};
    bool valid{false};
};

std::string UdpPeerToIpString(const UdpPeer &peer);

class UdpServer {
  public:
    ~UdpServer();

    bool Open(uint16_t port);
    void Close();
    int Recv(uint8_t *buf, size_t cap, UdpPeer *peerOut);
    bool SendTo(const UdpPeer &peer, const uint8_t *data, size_t len);

  private:
    int m_fd{-1};
};
