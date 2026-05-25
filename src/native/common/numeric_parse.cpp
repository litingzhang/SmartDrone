#include "common/numeric_parse.h"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <limits>

namespace SmartDrone::Common {

bool TryParseIntPrefix(const char *value, int base, int &out)
{
    if (value == nullptr || value[0] == '\0') {
        return false;
    }
    char *end = nullptr;
    errno = 0;
    const long parsed = std::strtol(value, &end, base);
    if (end == value || parsed < std::numeric_limits<int>::min() ||
        parsed > std::numeric_limits<int>::max() || errno == ERANGE) {
        return false;
    }
    out = static_cast<int>(parsed);
    return true;
}

bool TryParseIntFull(const char *value, int base, int &out)
{
    if (value == nullptr || value[0] == '\0') {
        return false;
    }
    char *end = nullptr;
    errno = 0;
    const long parsed = std::strtol(value, &end, base);
    if (end == value || *end != '\0' ||
        parsed < std::numeric_limits<int>::min() ||
        parsed > std::numeric_limits<int>::max() || errno == ERANGE) {
        return false;
    }
    out = static_cast<int>(parsed);
    return true;
}

bool TryParseInt64Prefix(const char *value, int base, std::int64_t &out)
{
    if (value == nullptr || value[0] == '\0') {
        return false;
    }
    char *end = nullptr;
    errno = 0;
    const long long parsed = std::strtoll(value, &end, base);
    if (end == value || errno == ERANGE) {
        return false;
    }
    out = static_cast<std::int64_t>(parsed);
    return true;
}

bool TryParseSizePrefix(const char *value, std::size_t &out)
{
    if (value == nullptr || value[0] == '\0' || value[0] == '-') {
        return false;
    }
    char *end = nullptr;
    errno = 0;
    const unsigned long long parsed = std::strtoull(value, &end, 10);
    if (end == value || errno == ERANGE ||
        parsed > std::numeric_limits<std::size_t>::max()) {
        return false;
    }
    out = static_cast<std::size_t>(parsed);
    return true;
}

bool TryParseSizeFull(const char *value, std::size_t &out)
{
    if (!TryParseSizePrefix(value, out)) {
        return false;
    }
    char *end = nullptr;
    errno = 0;
    (void)std::strtoull(value, &end, 10);
    return end != nullptr && *end == '\0' && errno != ERANGE;
}

bool TryParseUInt64Full(const char *value, std::uint64_t &out)
{
    if (value == nullptr || value[0] == '\0' || value[0] == '-') {
        return false;
    }
    char *end = nullptr;
    errno = 0;
    const unsigned long long parsed = std::strtoull(value, &end, 10);
    if (end == value || *end != '\0' || errno == ERANGE) {
        return false;
    }
    out = static_cast<std::uint64_t>(parsed);
    return true;
}

bool TryParseFloatPrefix(const char *value, float &out)
{
    if (value == nullptr || value[0] == '\0') {
        return false;
    }
    char *end = nullptr;
    errno = 0;
    const float parsed = std::strtof(value, &end);
    if (end == value || errno == ERANGE) {
        return false;
    }
    out = parsed;
    return true;
}

bool TryParseDoublePrefix(const char *value, double &out)
{
    if (value == nullptr || value[0] == '\0') {
        return false;
    }
    char *end = nullptr;
    errno = 0;
    const double parsed = std::strtod(value, &end);
    if (end == value || errno == ERANGE) {
        return false;
    }
    out = parsed;
    return true;
}

int ParseIntValue(const char *value, int fallback)
{
    return ParseIntValueBase(value, fallback, 10);
}

int ParseIntValueBase(const char *value, int fallback, int base)
{
    int parsed = fallback;
    return TryParseIntPrefix(value, base, parsed) ? parsed : fallback;
}

std::int64_t ParseInt64Value(const char *value, std::int64_t fallback)
{
    std::int64_t parsed = fallback;
    return TryParseInt64Prefix(value, 10, parsed) ? parsed : fallback;
}

std::size_t ParseSizeValueClamped(const char *value, std::size_t fallback,
                                  std::size_t minValue,
                                  std::size_t maxValue)
{
    std::size_t parsed = fallback;
    if (!TryParseSizeFull(value, parsed)) {
        return fallback;
    }
    return std::clamp(parsed, minValue, maxValue);
}

float ParseFloatValue(const char *value, float fallback)
{
    float parsed = fallback;
    return TryParseFloatPrefix(value, parsed) ? parsed : fallback;
}

} // namespace SmartDrone::Common
