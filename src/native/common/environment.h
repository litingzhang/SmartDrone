#pragma once

namespace SmartDrone::Common {

bool EnvVarIsUnsetOrEmpty(const char *name);
void SetEnvVar(const char *name, const char *value);
void UnsetEnvVar(const char *name);
void SetEnvVarIfUnset(const char *name, const char *value);

} // namespace SmartDrone::Common
