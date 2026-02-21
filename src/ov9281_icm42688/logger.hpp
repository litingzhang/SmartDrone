// Usage:
//   #include "logger.hpp"
//   Logger::init("/tmp/app.log", 8*1024*1024); // 8MB ring
//   LOGI("hello %d", 123);
//   LOGE("err: %s", msg);
// Optional: #define LOGGER_OVERRIDE_PRINTF before including to redirect printf -> LOGF.
//
// Notes:
// - Fixed-size ring file. Writes wrap-around and overwrite old content.
// - File content is not a normal chronological text file once wrapped.
// - Thread-safe by a single mutex.
// - Minimal overhead; uses fwrite+fflush (you can disable flush if you want).

#pragma once

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <atomic>
#include <chrono>

class Logger
{
public:
    enum Level : int { DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3, OFF = 99 };

    // Initialize ring log file.
    // max_bytes: fixed file size (depth). Must be > 4KB recommended.
    // flush_each: flush every log line (safer) vs buffered (faster).
    static bool init(const char* path, size_t max_bytes,
                     Level level = INFO, bool flush_each = true)
    {
        std::lock_guard<std::mutex> lk(s_mtx());
        close_locked();

        if (!path || !*path || max_bytes < 4096) return false;

        s_path() = path;
        s_maxBytes() = max_bytes;
        s_level().store(level, std::memory_order_relaxed);
        s_flushEach() = flush_each;

        // open file r+b else create w+b
        FILE* fp = std::fopen(path, "r+b");
        if (!fp) fp = std::fopen(path, "w+b");
        if (!fp) return false;

        s_fp() = fp;

        // ensure file size == max_bytes (pre-allocate by seeking and writing one byte)
        if (!ensure_size_locked(max_bytes)) {
            close_locked();
            return false;
        }

        // restore write position from sidecar (best-effort)
        s_writePos() = load_pos_locked();
        if (s_writePos() >= s_maxBytes()) s_writePos() = 0;

        // seek to current write position
        std::fseek(s_fp(), (long)s_writePos(), SEEK_SET);

        return true;
    }

    static void shutdown()
    {
        std::lock_guard<std::mutex> lk(s_mtx());
        close_locked();
    }

    static void setLevel(Level level) { s_level().store(level, std::memory_order_relaxed); }

    // Get current write position in ring file (0..max_bytes-1)
    static size_t tell()
    {
        return s_writePosAtomic().load(std::memory_order_relaxed);
    }

    static void logf(Level lvl, const char* fmt, ...)
    {
        if (!fmt) return;
        if (!enabled(lvl)) return;

        va_list ap;
        va_start(ap, fmt);
        vlogf(lvl, fmt, ap);
        va_end(ap);
    }

    static void vlogf(Level lvl, const char* fmt, va_list ap)
    {
        if (!fmt) return;
        if (!enabled(lvl)) return;

        // Build one line into a stack buffer (truncates if too long)
        char line[1024];
        size_t off = 0;

        // prefix: time + level
        off += format_prefix(line + off, sizeof(line) - off, lvl);
        if (off >= sizeof(line)) off = sizeof(line) - 1;

        // message body
        if (off < sizeof(line)) {
            int n = std::vsnprintf(line + off, sizeof(line) - off, fmt, ap);
            if (n < 0) n = 0;
            size_t nn = (size_t)n;
            if (nn >= sizeof(line) - off) nn = sizeof(line) - off - 1;
            off += nn;
        }

        // ensure newline
        if (off + 1 < sizeof(line)) {
            line[off++] = '\n';
            line[off] = '\0';
        } else {
            // last char newline
            line[sizeof(line) - 2] = '\n';
            line[sizeof(line) - 1] = '\0';
            off = sizeof(line) - 1;
        }

        write_locked(line, off);
    }

private:
    static bool enabled(Level lvl)
    {
        Level cur = (Level)s_level().load(std::memory_order_relaxed);
        return (cur != OFF) && (lvl >= cur);
    }

    static const char* lvl_str(Level lvl)
    {
        switch (lvl) {
        case DEBUG: return "D";
        case INFO:  return "I";
        case WARN:  return "W";
        case ERROR: return "E";
        default:    return "?";
        }
    }

    static size_t format_prefix(char* out, size_t cap, Level lvl)
    {
        if (!out || cap == 0) return 0;

        // monotonic time ms (good enough for ordering within a run)
        using namespace std::chrono;
        uint64_t ms = (uint64_t)duration_cast<milliseconds>(
                          steady_clock::now().time_since_epoch())
                          .count();

        // "[12345678][I] "
        int n = std::snprintf(out, cap, "[%llu][%s] ",
                              (unsigned long long)ms, lvl_str(lvl));
        if (n < 0) return 0;
        size_t nn = (size_t)n;
        if (nn >= cap) nn = cap - 1;
        return nn;
    }

    static void write_locked(const char* data, size_t len)
    {
        std::lock_guard<std::mutex> lk(s_mtx());

        if (!s_fp() || s_maxBytes() == 0 || !data || len == 0)
            return;

        // If single line larger than ring, truncate to ring-1
        if (len >= s_maxBytes()) {
            data += (len - (s_maxBytes() - 1));
            len = s_maxBytes() - 1;
        }

        size_t pos = s_writePos();
        size_t maxb = s_maxBytes();

        // Write in up to two chunks (wrap)
        size_t tail = maxb - pos;
        if (len <= tail) {
            std::fseek(s_fp(), (long)pos, SEEK_SET);
            std::fwrite(data, 1, len, s_fp());
            pos += len;
            if (pos == maxb) pos = 0;
        } else {
            // chunk1 to end
            std::fseek(s_fp(), (long)pos, SEEK_SET);
            std::fwrite(data, 1, tail, s_fp());
            // chunk2 from beginning
            std::fseek(s_fp(), 0L, SEEK_SET);
            std::fwrite(data + tail, 1, len - tail, s_fp());
            pos = len - tail;
        }

        if (s_flushEach()) std::fflush(s_fp());

        s_writePos() = pos;
        s_writePosAtomic().store(pos, std::memory_order_relaxed);

        // persist write position occasionally (every ~64 writes)
        if ((++s_posSaveCounter() & 63u) == 0u) {
            save_pos_locked(pos);
        }
    }

    static bool ensure_size_locked(size_t max_bytes)
    {
        // Ensure file length = max_bytes
        std::fseek(s_fp(), 0L, SEEK_END);
        long cur = std::ftell(s_fp());
        if (cur < 0) return false;

        if ((size_t)cur == max_bytes) return true;

        if ((size_t)cur > max_bytes) {
            // shrink: reopen by truncation
            // portable approach: rewrite file to exact size
            // (simple: create new file and swap)
            // Here we do a minimal approach: overwrite by extending logic:
            // If larger, we keep it; but ring logic assumes max_bytes.
            // Safer: return false.
            return false;
        }

        // extend
        std::fseek(s_fp(), (long)(max_bytes - 1), SEEK_SET);
        unsigned char zero = 0;
        if (std::fwrite(&zero, 1, 1, s_fp()) != 1) return false;
        std::fflush(s_fp());
        return true;
    }

    static std::string pos_path_locked()
    {
        return s_path() + ".pos";
    }

    static size_t load_pos_locked()
    {
        const std::string p = pos_path_locked();
        FILE* f = std::fopen(p.c_str(), "rb");
        if (!f) return 0;
        unsigned long long v = 0;
        size_t n = std::fread(&v, 1, sizeof(v), f);
        std::fclose(f);
        if (n != sizeof(v)) return 0;
        return (size_t)v;
    }

    static void save_pos_locked(size_t pos)
    {
        const std::string p = pos_path_locked();
        FILE* f = std::fopen(p.c_str(), "wb");
        if (!f) return;
        unsigned long long v = (unsigned long long)pos;
        std::fwrite(&v, 1, sizeof(v), f);
        std::fclose(f);
    }

    static void close_locked()
    {
        if (s_fp()) {
            save_pos_locked(s_writePos());
            std::fclose(s_fp());
            s_fp() = nullptr;
        }
        s_writePos() = 0;
        s_writePosAtomic().store(0, std::memory_order_relaxed);
        s_path().clear();
        s_maxBytes() = 0;
        s_posSaveCounter() = 0;
    }

    // --- statics as function-locals to keep header-only ODR-safe ---
    static std::mutex& s_mtx() { static std::mutex m; return m; }
    static FILE*& s_fp() { static FILE* fp = nullptr; return fp; }
    static std::string& s_path() { static std::string p; return p; }
    static size_t& s_maxBytes() { static size_t b = 0; return b; }
    static bool& s_flushEach() { static bool f = true; return f; }
    static std::atomic<int>& s_level() { static std::atomic<int> lv((int)INFO); return lv; }
    static size_t& s_writePos() { static size_t pos = 0; return pos; }
    static std::atomic<size_t>& s_writePosAtomic() { static std::atomic<size_t> pos(0); return pos; }
    static unsigned& s_posSaveCounter() { static unsigned c = 0; return c; }
};

// Convenience macros
#define LOGD(...) Logger::logf(Logger::DEBUG, __VA_ARGS__)
#define LOGI(...) Logger::logf(Logger::INFO,  __VA_ARGS__)
#define LOGW(...) Logger::logf(Logger::WARN,  __VA_ARGS__)
#define LOGE(...) Logger::logf(Logger::ERROR, __VA_ARGS__)

// If you want to replace printf usage in your codebase (only in files including this header)
// define LOGGER_OVERRIDE_PRINTF before including logger.hpp
#ifdef LOGGER_OVERRIDE_PRINTF
static inline int log_printf_compat(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    Logger::vlogf(Logger::INFO, fmt, ap);
    va_end(ap);
    return 0; // printf returns chars; we ignore for simplicity
}
#define printf(...) log_printf_compat(__VA_ARGS__)
#endif