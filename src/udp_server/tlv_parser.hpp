#pragma once
#include "crc16_ccitt_false.hpp"
#include "tlv_protocol.hpp"
#include <cstdint>
#include <vector>
#include <optional>
#include <cstring>

struct TlvFrame
{
    uint8_t ver{0};
    uint8_t cmd{0};
    uint8_t flags{0};
    uint16_t len{0};
    uint32_t seq{0};
    uint32_t t_ms{0};
    std::vector<uint8_t> payload;
};

class TlvParser
{
public:
    void push(const uint8_t* data, size_t n)
    {
        m_buf.insert(m_buf.end(), data, data + n);
    }

    std::optional<TlvFrame> tryPop()
    {
        while (true)
        {
            const size_t minSize = 2 + (1+1+1 + 2 + 4 + 4) + 2;
            if (m_buf.size() < minSize) return std::nullopt;

            // find sync
            size_t i = 0;
            for (; i + 1 < m_buf.size(); ++i)
            {
                if (m_buf[i] == TLV_SYNC0 && m_buf[i+1] == TLV_SYNC1) break;
            }
            if (i > 0) m_buf.erase(m_buf.begin(), m_buf.begin() + i);
            if (m_buf.size() < minSize) return std::nullopt;
            if (!(m_buf[0] == TLV_SYNC0 && m_buf[1] == TLV_SYNC1)) return std::nullopt;

            const uint8_t ver   = m_buf[2];
            const uint8_t cmd   = m_buf[3];
            const uint8_t flags = m_buf[4];
            const uint16_t len  = rdU16LE(&m_buf[5]);

            const size_t total = 2 + (1+1+1 + 2 + 4 + 4) + (size_t)len + 2;
            if (m_buf.size() < total) return std::nullopt;

            const uint32_t seq  = rdU32LE(&m_buf[7]);
            const uint32_t t_ms = rdU32LE(&m_buf[11]);

            // CRC check over VER..PAYLOAD
            const uint8_t* crcBase = &m_buf[2];
            const size_t crcLen = (total - 2) - 2; // exclude sync, exclude crc
            const uint16_t crcCalc = crc16_ccitt_false(crcBase, crcLen);
            const uint16_t crcRecv = rdU16LE(&m_buf[total - 2]);

            if (crcCalc != crcRecv)
            {
                // resync: drop 1 byte and scan again
                m_buf.erase(m_buf.begin());
                continue;
            }

            TlvFrame f;
            f.ver = ver; f.cmd = cmd; f.flags = flags; f.len = len; f.seq = seq; f.t_ms = t_ms;
            f.payload.resize(len);
            if (len) std::memcpy(f.payload.data(), &m_buf[15], len);

            m_buf.erase(m_buf.begin(), m_buf.begin() + total);
            return f;
        }
    }

private:
    std::vector<uint8_t> m_buf;

    static uint16_t rdU16LE(const uint8_t* p)
    {
        return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    }
    static uint32_t rdU32LE(const uint8_t* p)
    {
        return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    }
};