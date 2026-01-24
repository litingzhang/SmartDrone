#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <libcamera/pixel_format.h>

struct FrameItem {
    int m_camIndex;
    uint64_t m_tsNs;

    unsigned int m_width;
    unsigned int m_height;
    unsigned int m_stride;
    libcamera::PixelFormat m_fmt;

    std::shared_ptr<std::vector<uint8_t>> m_data;

    FrameItem()
        : m_camIndex(-1), m_tsNs(0), m_width(0), m_height(0), m_stride(0), m_fmt(), m_data(nullptr)
    {
    }
};
