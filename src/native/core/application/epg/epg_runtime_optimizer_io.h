#pragma once

#include <string>

namespace SmartDrone::Core::Application {

std::string ReadEpgOptimizerFile(const std::string &path);
bool EpgOptimizedConfigChanged(const std::string &oldJson,
                               const std::string &newJson);
void WriteRequiredEpgOptimizerArtifactFile(const std::string &path,
                                           const std::string &text);

} // namespace SmartDrone::Core::Application
