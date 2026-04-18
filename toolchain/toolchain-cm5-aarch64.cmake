set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(_CM5_SYSROOT "")
if(DEFINED SYSROOT AND NOT SYSROOT STREQUAL "")
  set(_CM5_SYSROOT "${SYSROOT}")
elseif(DEFINED CMAKE_SYSROOT AND NOT CMAKE_SYSROOT STREQUAL "")
  set(_CM5_SYSROOT "${CMAKE_SYSROOT}")
elseif(DEFINED ENV{SYSROOT} AND NOT "$ENV{SYSROOT}" STREQUAL "")
  set(_CM5_SYSROOT "$ENV{SYSROOT}")
endif()

if(_CM5_SYSROOT MATCHES "^~")
  string(REGEX REPLACE "^~" "$ENV{HOME}" _CM5_SYSROOT "${_CM5_SYSROOT}")
endif()

if(_CM5_SYSROOT STREQUAL "")
  message(FATAL_ERROR "SYSROOT must be provided when using toolchain-cm5-aarch64.cmake")
endif()

set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

set(CMAKE_SYSROOT "${_CM5_SYSROOT}" CACHE PATH "Cross-compilation sysroot" FORCE)
set(CMAKE_FIND_ROOT_PATH "${_CM5_SYSROOT}" CACHE PATH "Cross-compilation search root" FORCE)
set(ENV{PKG_CONFIG_SYSROOT_DIR} "${_CM5_SYSROOT}")

set(_CM5_SYSROOT_FLAG "--sysroot=${_CM5_SYSROOT}")
set(CMAKE_C_FLAGS_INIT "${_CM5_SYSROOT_FLAG}")
set(CMAKE_CXX_FLAGS_INIT "${_CM5_SYSROOT_FLAG}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${_CM5_SYSROOT_FLAG}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${_CM5_SYSROOT_FLAG}")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "${_CM5_SYSROOT_FLAG}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# pkg-config 只走 sysroot
set(ENV{PKG_CONFIG_SYSROOT_DIR} "${SYSROOT}")

set(_pkgconfig_libdir
    "${_CM5_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig"
    "${_CM5_SYSROOT}/usr/lib/pkgconfig"
    "${_CM5_SYSROOT}/usr/share/pkgconfig"
)
string(JOIN ":" _pkgconfig_libdir_str ${_pkgconfig_libdir})
set(ENV{PKG_CONFIG_LIBDIR} "${_pkgconfig_libdir_str}")
set(ENV{PKG_CONFIG_PATH} "")
