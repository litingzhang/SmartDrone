#pragma once

#include <chrono>
#include <cstdint>

inline uint64_t MonoTimeUs()
{
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

inline uint32_t MonoTimeMs32()
{
    return static_cast<uint32_t>((MonoTimeUs() / 1000ULL) & 0xFFFFFFFFu);
}
