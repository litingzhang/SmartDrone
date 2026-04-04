#pragma once

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <string>

class UdpClient {
  public:
    ~UdpClient();

    bool Open(const std::string &ip, uint16_t sendPort, uint16_t bindPort);
    void Close();
    bool Send(const uint8_t *data, size_t len);
    int Recv(uint8_t *out, size_t cap);

  private:
    int m_fd{-1};
    sockaddr_in m_dstAddr{};
};
