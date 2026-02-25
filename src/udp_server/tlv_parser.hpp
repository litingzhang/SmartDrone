#pragma once

#include "crc16_ccitt_false.hpp"
#include "tlv_protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

struct TlvFrame {
    uint8_t ver{0};
    uint8_t cmd{0};
    uint8_t flags{0};
    uint16_t len{0};
    uint32_t seq{0};
    uint32_t tMs{0};
    std::vector<uint8_t> payload;
};

class TlvParser {
public:
    void Push(const uint8_t* data, size_t size)
    {
        m_buffer.insert(m_buffer.end(), data, data + size);
    }

    std::optional<TlvFrame> TryPop()
    {
        while (true) {
            const size_t kMinSize = 2 + (1 + 1 + 1 + 2 + 4 + 4) + 2;
            if (m_buffer.size() < kMinSize) {
                return std::nullopt;
            }

            size_t syncIndex = 0;
            for (; syncIndex + 1 < m_buffer.size(); ++syncIndex) {
                if (m_buffer[syncIndex] == TLV_SYNC0 && m_buffer[syncIndex + 1] == TLV_SYNC1) {
                    break;
                }
            }
            if (syncIndex > 0) {
                m_buffer.erase(m_buffer.begin(), m_buffer.begin() + syncIndex);
            }
            if (m_buffer.size() < kMinSize) {
                return std::nullopt;
            }
            if (!(m_buffer[0] == TLV_SYNC0 && m_buffer[1] == TLV_SYNC1)) {
                return std::nullopt;
            }

            const uint8_t ver = m_buffer[2];
            const uint8_t cmd = m_buffer[3];
            const uint8_t flags = m_buffer[4];
            const uint16_t len = ReadU16Le(&m_buffer[5]);

            const size_t totalSize = 2 + (1 + 1 + 1 + 2 + 4 + 4) + static_cast<size_t>(len) + 2;
            if (m_buffer.size() < totalSize) {
                return std::nullopt;
            }

            const uint32_t seq = ReadU32Le(&m_buffer[7]);
            const uint32_t tMs = ReadU32Le(&m_buffer[11]);

            const uint8_t* crcBase = &m_buffer[2];
            const size_t crcLen = (totalSize - 2) - 2;
            const uint16_t crcCalc = Crc16CcittFalse(crcBase, crcLen);
            const uint16_t crcRecv = ReadU16Le(&m_buffer[totalSize - 2]);
            if (crcCalc != crcRecv) {
                m_buffer.erase(m_buffer.begin());
                continue;
            }

            TlvFrame frame;
            frame.ver = ver;
            frame.cmd = cmd;
            frame.flags = flags;
            frame.len = len;
            frame.seq = seq;
            frame.tMs = tMs;
            frame.payload.resize(len);
            if (len > 0) {
                std::memcpy(frame.payload.data(), &m_buffer[15], len);
            }

            m_buffer.erase(m_buffer.begin(), m_buffer.begin() + totalSize);
            return frame;
        }
    }

private:
    static uint16_t ReadU16Le(const uint8_t* p)
    {
        return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
    }

    static uint32_t ReadU32Le(const uint8_t* p)
    {
        return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
               (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
    }

    std::vector<uint8_t> m_buffer;
};
