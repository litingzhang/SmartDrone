#include "common/tlv/tlv_parser.h"

#include "common/tlv/crc16_ccitt_false.h"
#include "common/tlv/tlv_protocol.h"

#include <algorithm>

void TlvParser::Push(const uint8_t *data, size_t size)
{
    if (data == nullptr || size == 0) {
        return;
    }
    if (size >= MAX_BUFFER_SIZE) {
        data += size - MAX_BUFFER_SIZE;
        size = MAX_BUFFER_SIZE;
        m_head = 0;
        m_size = 0;
    }
    const size_t overflow =
        m_size + size > MAX_BUFFER_SIZE ? m_size + size - MAX_BUFFER_SIZE : 0;
    if (overflow > 0) {
        Advance(overflow);
    }
    for (size_t i = 0; i < size; ++i) {
        m_buffer[(m_head + m_size + i) % MAX_BUFFER_SIZE] = data[i];
    }
    m_size += size;
}

std::optional<TlvFrame> TlvParser::TryPop()
{
    while (true) {
        if (m_size < MIN_FRAME_SIZE) {
            return std::nullopt;
        }
        if (!AlignToSync()) {
            return std::nullopt;
        }
        const TlvFrameHeader header = ReadFrameHeader();
        if (header.len > MAX_PAYLOAD_SIZE) {
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

bool TlvParser::AlignToSync()
{
    const size_t syncIndex = FindSyncIndex();
    if (syncIndex > 0) {
        Advance(syncIndex);
    }
    return m_size >= MIN_FRAME_SIZE && ByteAt(0) == TLV_SYNC0 &&
           ByteAt(1) == TLV_SYNC1;
}

size_t TlvParser::FindSyncIndex() const
{
    size_t syncIndex = 0;
    for (; syncIndex + 1 < m_size; ++syncIndex) {
        if (ByteAt(syncIndex) == TLV_SYNC0 &&
            ByteAt(syncIndex + 1) == TLV_SYNC1) {
            break;
        }
    }
    return syncIndex;
}

TlvParser::TlvFrameHeader TlvParser::ReadFrameHeader() const
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

size_t TlvParser::FrameTotalSize(uint16_t payloadLen)
{
    return FRAME_HEADER_SIZE + static_cast<size_t>(payloadLen) + FRAME_CRC_SIZE;
}

bool TlvParser::FrameAvailable(size_t totalSize) const
{
    return m_size >= totalSize;
}

bool TlvParser::FrameCrcValid(size_t totalSize) const
{
    const size_t crcLen = totalSize - 2 - FRAME_CRC_SIZE;
    const uint16_t crcCalc = CalcCrc(2, crcLen);
    const uint16_t crcRecv = ReadU16LeAt(totalSize - FRAME_CRC_SIZE);
    return crcCalc == crcRecv;
}

TlvFrame TlvParser::BuildFrame(const TlvFrameHeader &header) const
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
        CopyOut(FRAME_HEADER_SIZE, frame.payload.data(), header.len);
    }
    return frame;
}

uint8_t TlvParser::ByteAt(size_t offset) const
{
    return m_buffer[(m_head + offset) % MAX_BUFFER_SIZE];
}

uint16_t TlvParser::CalcCrc(size_t offset, size_t len) const
{
    if (len == 0) {
        return 0xFFFF;
    }

    const size_t start = (m_head + offset) % MAX_BUFFER_SIZE;
    const size_t firstLen = std::min(len, MAX_BUFFER_SIZE - start);
    uint16_t crc =
        Crc16CcittFalseUpdate(0xFFFF, m_buffer.data() + start, firstLen);
    if (firstLen < len) {
        crc = Crc16CcittFalseUpdate(crc, m_buffer.data(), len - firstLen);
    }
    return crc;
}

void TlvParser::CopyOut(size_t offset, uint8_t *dst, size_t len) const
{
    for (size_t i = 0; i < len; ++i) {
        dst[i] = ByteAt(offset + i);
    }
}

void TlvParser::Advance(size_t count)
{
    const size_t delta = count > m_size ? m_size : count;
    m_head = (m_head + delta) % MAX_BUFFER_SIZE;
    m_size -= delta;
}

uint16_t TlvParser::ReadU16LeAt(size_t offset) const
{
    return static_cast<uint16_t>(ByteAt(offset)) |
           (static_cast<uint16_t>(ByteAt(offset + 1)) << 8);
}

uint32_t TlvParser::ReadU32LeAt(size_t offset) const
{
    return static_cast<uint32_t>(ByteAt(offset)) |
           (static_cast<uint32_t>(ByteAt(offset + 1)) << 8) |
           (static_cast<uint32_t>(ByteAt(offset + 2)) << 16) |
           (static_cast<uint32_t>(ByteAt(offset + 3)) << 24);
}
