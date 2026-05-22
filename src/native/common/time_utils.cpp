#include "common/time_utils.h"

#include <chrono>

uint64_t MonoTimeUs()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

uint32_t MonoTimeMs32()
{
    return static_cast<uint32_t>((MonoTimeUs() / 1000ULL) & 0xFFFFFFFFu);
}
