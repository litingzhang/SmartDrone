#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <variant>

namespace SmartDrone::Core::Application {

using ConfigValue = std::variant<int64_t, double, bool, std::string>;

struct ConfigUpdate {
    std::unordered_map<std::string, ConfigValue> values;
};

} // namespace SmartDrone::Core::Application
