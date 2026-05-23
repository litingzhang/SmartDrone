#include "common/environment.h"

#include <cstdlib>

namespace SmartDrone::Common {

bool EnvVarIsUnsetOrEmpty(const char *name)
{
    const char *value = std::getenv(name);
    return value == nullptr || value[0] == '\0';
}

void SetEnvVar(const char *name, const char *value)
{
#if defined(_WIN32)
    _putenv_s(name, value);
#else
    setenv(name, value, 1);
#endif
}

void UnsetEnvVar(const char *name)
{
#if defined(_WIN32)
    _putenv_s(name, "");
#else
    unsetenv(name);
#endif
}

void SetEnvVarIfUnset(const char *name, const char *value)
{
    if (!EnvVarIsUnsetOrEmpty(name)) {
        return;
    }
    SetEnvVar(name, value);
}

} // namespace SmartDrone::Common
