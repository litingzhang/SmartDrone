#pragma once

#include <cstdarg>
#include <cstddef>

class Logger {
  public:
    enum Level : int {
        DEBUG = 0,
        INFO = 1,
        WARN = 2,
        ERROR = 3,
        OFF = 99,
    };

    static bool Init(const char *path,
                     std::size_t maxBytes,
                     Level level = INFO,
                     bool flushEach = true);
    static void Shutdown();
    static void SetLevel(Level level);
    static std::size_t Tell();
    static void Logf(Level level, const char *fmt, ...);
    static void VLogf(Level level, const char *fmt, va_list ap);
};
