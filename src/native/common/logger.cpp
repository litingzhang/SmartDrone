#include "common/logger.h"

#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

namespace {

struct LoggerState {
    ~LoggerState();

    int fd{-1};
    std::string path;
    std::size_t maxBytes{0};
    bool flushEach{true};
    mutable std::atomic<std::size_t> writePos{0};
    mutable std::atomic<unsigned> posSaveCounter{0};
};

std::shared_ptr<const LoggerState> g_loggerState;
std::atomic<int> g_logLevel{Logger::INFO};

const char *LevelText(Logger::Level level)
{
    switch (level) {
    case Logger::DEBUG:
        return "D";
    case Logger::INFO:
        return "I";
    case Logger::WARN:
        return "W";
    case Logger::ERROR:
        return "E";
    default:
        return "?";
    }
}

bool Enabled(Logger::Level level)
{
    const auto current =
        static_cast<Logger::Level>(g_logLevel.load(std::memory_order_relaxed));
    return current != Logger::OFF && level >= current;
}

std::string PosPath(const std::string &path)
{
    return path + ".pos";
}

std::size_t LoadPosition(const std::string &path)
{
    const int fd = ::open(PosPath(path).c_str(), O_RDONLY);
    if (fd < 0) {
        return 0;
    }
    unsigned long long value = 0;
    const ssize_t read = ::read(fd, &value, sizeof(value));
    (void)::close(fd);
    if (read != static_cast<ssize_t>(sizeof(value))) {
        return 0;
    }
    return static_cast<std::size_t>(value);
}

void SavePosition(const std::string &path, std::size_t position)
{
    const int fd = ::open(PosPath(path).c_str(),
                          O_WRONLY | O_CREAT | O_TRUNC,
                          0644);
    if (fd < 0) {
        return;
    }
    const auto value = static_cast<unsigned long long>(position);
    const char *data = reinterpret_cast<const char *>(&value);
    std::size_t written = 0;
    while (written < sizeof(value)) {
        const ssize_t result = ::write(fd, data + written, sizeof(value) - written);
        if (result < 0 && errno == EINTR) {
            continue;
        }
        if (result <= 0) {
            break;
        }
        written += static_cast<std::size_t>(result);
    }
    (void)::close(fd);
}

LoggerState::~LoggerState()
{
    if (fd < 0 || maxBytes == 0) {
        return;
    }
    SavePosition(path, writePos.load(std::memory_order_acquire) % maxBytes);
    (void)::close(fd);
}

bool EnsureSize(int fd, std::size_t maxBytes)
{
    const off_t current = ::lseek(fd, 0, SEEK_END);
    if (current < 0) {
        return false;
    }
    if (static_cast<std::size_t>(current) == maxBytes) {
        return true;
    }
    if (static_cast<std::size_t>(current) > maxBytes) {
        return false;
    }
    const unsigned char zero = 0;
    return ::pwrite(fd, &zero, 1, static_cast<off_t>(maxBytes - 1)) == 1;
}

std::size_t FormatPrefix(char *out, std::size_t cap, Logger::Level level)
{
    if (!out || cap == 0) {
        return 0;
    }
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
    const int written =
        std::snprintf(out, cap, "[%llu][%s] ",
                      static_cast<unsigned long long>(ms), LevelText(level));
    if (written < 0) {
        return 0;
    }
    const auto used = static_cast<std::size_t>(written);
    return used >= cap ? cap - 1 : used;
}

std::size_t FormatLine(char *line,
                       std::size_t cap,
                       Logger::Level level,
                       const char *fmt,
                       va_list ap)
{
    std::size_t offset = FormatPrefix(line, cap, level);
    if (offset >= cap) {
        offset = cap - 1;
    }
    if (offset < cap) {
        int written = std::vsnprintf(line + offset, cap - offset, fmt, ap);
        if (written < 0) {
            written = 0;
        }
        std::size_t body = static_cast<std::size_t>(written);
        if (body >= cap - offset) {
            body = cap - offset - 1;
        }
        offset += body;
    }
    if (offset + 1 < cap) {
        line[offset++] = '\n';
        line[offset] = '\0';
        return offset;
    }
    line[cap - 2] = '\n';
    line[cap - 1] = '\0';
    return cap - 1;
}

bool WriteAllAt(int fd, const char *data, std::size_t len, std::size_t pos)
{
    std::size_t written = 0;
    while (written < len) {
        const ssize_t result =
            ::pwrite(fd, data + written, len - written,
                     static_cast<off_t>(pos + written));
        if (result <= 0) {
            return false;
        }
        written += static_cast<std::size_t>(result);
    }
    return true;
}

void WriteRing(const LoggerState &state, const char *data, std::size_t len)
{
    if (state.fd < 0 || state.maxBytes == 0 || !data || len == 0) {
        return;
    }
    if (len >= state.maxBytes) {
        data += len - (state.maxBytes - 1);
        len = state.maxBytes - 1;
    }
    const std::size_t start =
        state.writePos.fetch_add(len, std::memory_order_acq_rel) %
        state.maxBytes;
    const std::size_t tail = state.maxBytes - start;
    if (len <= tail) {
        (void)WriteAllAt(state.fd, data, len, start);
    } else {
        (void)WriteAllAt(state.fd, data, tail, start);
        (void)WriteAllAt(state.fd, data + tail, len - tail, 0);
    }
    const std::size_t next = (start + len) % state.maxBytes;
    if (state.flushEach) {
        (void)::fsync(state.fd);
    }
    if ((state.posSaveCounter.fetch_add(1, std::memory_order_acq_rel) & 63u) == 0u) {
        SavePosition(state.path, next);
    }
}

} // namespace

bool Logger::Init(const char *path,
                  std::size_t maxBytes,
                  Level level,
                  bool flushEach)
{
    if (!path || !*path || maxBytes < 4096) {
        return false;
    }
    int fd = ::open(path, O_RDWR | O_CREAT, 0644);
    if (fd < 0) {
        return false;
    }
    if (!EnsureSize(fd, maxBytes)) {
        (void)::close(fd);
        return false;
    }
    auto state = std::make_shared<LoggerState>();
    state->fd = fd;
    state->path = path;
    state->maxBytes = maxBytes;
    state->flushEach = flushEach;
    const std::size_t position = LoadPosition(state->path) % maxBytes;
    state->writePos.store(position, std::memory_order_release);
    g_logLevel.store(level, std::memory_order_relaxed);
    (void)std::atomic_exchange_explicit(&g_loggerState,
                                        std::shared_ptr<const LoggerState>(state),
                                        std::memory_order_acq_rel);
    return true;
}

void Logger::Shutdown()
{
    (void)std::atomic_exchange_explicit(&g_loggerState,
                                        std::shared_ptr<const LoggerState>{},
                                        std::memory_order_acq_rel);
}

void Logger::SetLevel(Level level)
{
    g_logLevel.store(level, std::memory_order_relaxed);
}

std::size_t Logger::Tell()
{
    std::shared_ptr<const LoggerState> state =
        std::atomic_load_explicit(&g_loggerState, std::memory_order_acquire);
    if (!state || state->maxBytes == 0) {
        return 0;
    }
    return state->writePos.load(std::memory_order_acquire) % state->maxBytes;
}

void Logger::Logf(Level level, const char *fmt, ...)
{
    if (!fmt || !Enabled(level)) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    VLogf(level, fmt, ap);
    va_end(ap);
}

void Logger::VLogf(Level level, const char *fmt, va_list ap)
{
    if (!fmt || !Enabled(level)) {
        return;
    }
    std::shared_ptr<const LoggerState> state =
        std::atomic_load_explicit(&g_loggerState, std::memory_order_acquire);
    if (!state) {
        return;
    }
    char line[1024];
    const std::size_t len = FormatLine(line, sizeof(line), level, fmt, ap);
    WriteRing(*state, line, len);
}
