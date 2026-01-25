set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

set(CMAKE_SYSROOT "${SYSROOT}")
set(CMAKE_FIND_ROOT_PATH "${SYSROOT}")
set(ENV{PKG_CONFIG_SYSROOT_DIR} "${SYSROOT}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# pkg-config 只走 sysroot
set(ENV{PKG_CONFIG_SYSROOT_DIR} "${SYSROOT}")

set(_pkgconfig_libdir
    "${SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig"
    "${SYSROOT}/usr/lib/pkgconfig"
    "${SYSROOT}/usr/share/pkgconfig"
)
string(JOIN ":" _pkgconfig_libdir_str ${_pkgconfig_libdir})
set(ENV{PKG_CONFIG_LIBDIR} "${_pkgconfig_libdir_str}")
set(ENV{PKG_CONFIG_PATH} "")
