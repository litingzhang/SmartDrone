#pragma once

#include <string>

namespace SmartDrone::Core::Application {

struct CommandResult {
    bool ok{false};
    std::string message;
};

} // namespace SmartDrone::Core::Application
