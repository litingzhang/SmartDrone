#pragma once

#include <cstddef>
#include <cstdint>

namespace SmartDrone::Common {

bool TryParseIntPrefix(const char *value, int base, int &out);
bool TryParseIntFull(const char *value, int base, int &out);
bool TryParseInt64Prefix(const char *value, int base, std::int64_t &out);
bool TryParseSizePrefix(const char *value, std::size_t &out);
bool TryParseSizeFull(const char *value, std::size_t &out);
bool TryParseUInt64Full(const char *value, std::uint64_t &out);
bool TryParseFloatPrefix(const char *value, float &out);
bool TryParseDoublePrefix(const char *value, double &out);
int ParseIntValue(const char *value, int fallback);
int ParseIntValueBase(const char *value, int fallback, int base);
std::int64_t ParseInt64Value(const char *value, std::int64_t fallback);
std::size_t ParseSizeValueClamped(const char *value, std::size_t fallback,
                                  std::size_t minValue,
                                  std::size_t maxValue);
float ParseFloatValue(const char *value, float fallback);

} // namespace SmartDrone::Common
