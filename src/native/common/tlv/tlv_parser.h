#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
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
    static constexpr size_t MAX_PAYLOAD_SIZE = 512;
    static constexpr size_t MAX_BUFFER_SIZE = 4096;
    static constexpr size_t FRAME_HEADER_SIZE = 2 + (1 + 1 + 1 + 2 + 4 + 4);
    static constexpr size_t FRAME_CRC_SIZE = 2;
    static constexpr size_t MIN_FRAME_SIZE = FRAME_HEADER_SIZE + FRAME_CRC_SIZE;

    void Push(const uint8_t *data, size_t size);
    std::optional<TlvFrame> TryPop();

  private:
    struct TlvFrameHeader {
        uint8_t ver{0};
        uint8_t cmd{0};
        uint8_t flags{0};
        uint16_t len{0};
        uint32_t seq{0};
        uint32_t tMs{0};
    };

    bool AlignToSync();
    size_t FindSyncIndex() const;
    TlvFrameHeader ReadFrameHeader() const;
    static size_t FrameTotalSize(uint16_t payloadLen);
    bool FrameAvailable(size_t totalSize) const;
    bool FrameCrcValid(size_t totalSize) const;
    TlvFrame BuildFrame(const TlvFrameHeader &header) const;
    uint8_t ByteAt(size_t offset) const;
    uint16_t CalcCrc(size_t offset, size_t len) const;
    void CopyOut(size_t offset, uint8_t *dst, size_t len) const;
    void Advance(size_t count);
    uint16_t ReadU16LeAt(size_t offset) const;
    uint32_t ReadU32LeAt(size_t offset) const;

    std::array<uint8_t, MAX_BUFFER_SIZE> m_buffer{};
    size_t m_head{0};
    size_t m_size{0};
};
