#pragma once

#include "crc16_ccitt_false.h"
#include "tlv_protocol.h"

#include <algorithm>
#include <array>
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
    static constexpr size_t kMaxPayloadSize = 512;
    static constexpr size_t kMaxBufferSize = 4096;
    static constexpr size_t kFrameHeaderSize = 2 + (1 + 1 + 1 + 2 + 4 + 4);
    static constexpr size_t kFrameCrcSize = 2;
    static constexpr size_t kMinFrameSize = kFrameHeaderSize + kFrameCrcSize;

    void Push(const uint8_t *data, size_t size)
    {
        if (data == nullptr || size == 0) {
            return;
        }
        if (size >= kMaxBufferSize) {
            data += (size - kMaxBufferSize);
            size = kMaxBufferSize;
            m_head = 0;
            m_size = 0;
        }
        const size_t overflow = (m_size + size > kMaxBufferSize) ? (m_size + size - kMaxBufferSize) : 0;
        if (overflow > 0) {
            Advance(overflow);
        }
        for (size_t i = 0; i < size; ++i) {
            m_buffer[(m_head + m_size + i) % kMaxBufferSize] = data[i];
        }
        m_size += size;
    }

    std::optional<TlvFrame> TryPop()
    {
        while (true) {
            if (m_size < kMinFrameSize) {
                return std::nullopt;
            }

            if (!AlignToSync()) {
                return std::nullopt;
            }

            const TlvFrameHeader header = ReadFrameHeader();
            if (header.len > kMaxPayloadSize) {
                Advance(1);
                continue;
            }

            const size_t totalSize = FrameTotalSize(header.len);
            if (!FrameAvailable(totalSize)) {
                return std::nullopt;
            }

            if (!FrameCrcValid(totalSize)) {
                Advance(1);
                continue;
            }

            TlvFrame frame = BuildFrame(header);
            Advance(totalSize);
            return frame;
        }
    }

  private:
    struct TlvFrameHeader {
        uint8_t ver{0};
        uint8_t cmd{0};
        uint8_t flags{0};
        uint16_t len{0};
        uint32_t seq{0};
        uint32_t tMs{0};
    };

    bool AlignToSync()
    {
        const size_t syncIndex = FindSyncIndex();
        if (syncIndex > 0) {
            Advance(syncIndex);
        }
        return m_size >= kMinFrameSize && ByteAt(0) == TLV_SYNC0 && ByteAt(1) == TLV_SYNC1;
    }

    size_t FindSyncIndex() const
    {
        size_t syncIndex = 0;
        for (; syncIndex + 1 < m_size; ++syncIndex) {
            if (ByteAt(syncIndex) == TLV_SYNC0 && ByteAt(syncIndex + 1) == TLV_SYNC1) {
                break;
            }
        }
        return syncIndex;
    }

    TlvFrameHeader ReadFrameHeader() const
    {
        TlvFrameHeader header;
        header.ver = ByteAt(2);
        header.cmd = ByteAt(3);
        header.flags = ByteAt(4);
        header.len = ReadU16LeAt(5);
        header.seq = ReadU32LeAt(7);
        header.tMs = ReadU32LeAt(11);
        return header;
    }

    static size_t FrameTotalSize(uint16_t payloadLen)
    {
        return kFrameHeaderSize + static_cast<size_t>(payloadLen) + kFrameCrcSize;
    }

    bool FrameAvailable(size_t totalSize) const
    {
        return m_size >= totalSize;
    }

    bool FrameCrcValid(size_t totalSize) const
    {
        const size_t crcLen = (totalSize - 2) - kFrameCrcSize;
        const uint16_t crcCalc = CalcCrc(2, crcLen);
        const uint16_t crcRecv = ReadU16LeAt(totalSize - kFrameCrcSize);
        return crcCalc == crcRecv;
    }

    TlvFrame BuildFrame(const TlvFrameHeader &header) const
    {
        TlvFrame frame;
        frame.ver = header.ver;
        frame.cmd = header.cmd;
        frame.flags = header.flags;
        frame.len = header.len;
        frame.seq = header.seq;
        frame.tMs = header.tMs;
        frame.payload.resize(header.len);
        if (header.len > 0) {
            CopyOut(kFrameHeaderSize, frame.payload.data(), header.len);
        }
        return frame;
    }

    uint8_t ByteAt(size_t offset) const
    {
        return m_buffer[(m_head + offset) % kMaxBufferSize];
    }

    uint16_t CalcCrc(size_t offset, size_t len) const
    {
        if (len == 0) {
            return 0xFFFF;
        }

        const size_t start = (m_head + offset) % kMaxBufferSize;
        const size_t firstLen = std::min(len, kMaxBufferSize - start);
        uint16_t crc = Crc16CcittFalseUpdate(0xFFFF, m_buffer.data() + start, firstLen);
        if (firstLen < len) {
            crc = Crc16CcittFalseUpdate(crc, m_buffer.data(), len - firstLen);
        }
        return crc;
    }

    void CopyOut(size_t offset, uint8_t *dst, size_t len) const
    {
        for (size_t i = 0; i < len; ++i) {
            dst[i] = ByteAt(offset + i);
        }
    }

    void Advance(size_t count)
    {
        const size_t delta = (count > m_size) ? m_size : count;
        m_head = (m_head + delta) % kMaxBufferSize;
        m_size -= delta;
    }

    uint16_t ReadU16LeAt(size_t offset) const
    {
        return static_cast<uint16_t>(ByteAt(offset)) | (static_cast<uint16_t>(ByteAt(offset + 1)) << 8);
    }

    uint32_t ReadU32LeAt(size_t offset) const
    {
        return static_cast<uint32_t>(ByteAt(offset)) | (static_cast<uint32_t>(ByteAt(offset + 1)) << 8) |
               (static_cast<uint32_t>(ByteAt(offset + 2)) << 16) | (static_cast<uint32_t>(ByteAt(offset + 3)) << 24);
    }

    std::array<uint8_t, kMaxBufferSize> m_buffer{};
    size_t m_head{0};
    size_t m_size{0};
};
