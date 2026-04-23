cmake_minimum_required(VERSION 3.16)

include(FetchContent)

option(CROSS_AARCH64 "Enable aarch64 cross compile" ON)
option(BUILD_SMART_DRONE "Build unified stereo runtime application" ON)
option(ENABLE_UNIT_TESTS "Build unit tests" OFF)
option(ENABLE_OFFLINE_REPLAY "Build host-side offline replay tool" OFF)
option(SMART_DRONE_ENABLE_XFEAT_NATIVE "Build native libtorch-based XFeat backend" OFF)
set(SMART_DRONE_CAMERA_PROVIDER "libcamera_stereo_ov9281" CACHE STRING
    "Camera provider implementation compiled into the native runtime")
set(_SMART_DRONE_CAMERA_PROVIDERS "libcamera_stereo_ov9281;uvc_stereo_opencv")
if(NOT SMART_DRONE_CAMERA_PROVIDER IN_LIST _SMART_DRONE_CAMERA_PROVIDERS)
  message(FATAL_ERROR
    "Unsupported SMART_DRONE_CAMERA_PROVIDER='${SMART_DRONE_CAMERA_PROVIDER}'. "
    "Supported values: ${_SMART_DRONE_CAMERA_PROVIDERS}")
endif()
set(SMART_DRONE_USE_LIBCAMERA OFF)
if(SMART_DRONE_CAMERA_PROVIDER STREQUAL "libcamera_stereo_ov9281")
  set(SMART_DRONE_USE_LIBCAMERA ON)
endif()
set(SMART_DRONE_LIBCAMERA_INCLUDE_DIRS "")
set(SMART_DRONE_GPIOD_V2 OFF)
set(SMART_DRONE_GPIOD_INCLUDE_DIRS "")
set(SMART_DRONE_GPIOD_LIBRARY_DIRS "")
set(SMART_DRONE_GPIOD_LIBRARIES gpiod)
set(SMART_DRONE_GPIOD_CFLAGS_OTHER "")

if(NOT BUILD_SMART_DRONE AND NOT ENABLE_UNIT_TESTS AND NOT ENABLE_OFFLINE_REPLAY)
  message(FATAL_ERROR "Enable BUILD_SMART_DRONE, ENABLE_UNIT_TESTS, or ENABLE_OFFLINE_REPLAY")
endif()

if((ENABLE_UNIT_TESTS OR ENABLE_OFFLINE_REPLAY) AND CROSS_AARCH64)
  message(STATUS "Host-side tooling enabled, forcing CROSS_AARCH64=OFF")
  set(CROSS_AARCH64 OFF CACHE BOOL "Enable aarch64 cross compile" FORCE)
endif()

if(CROSS_AARCH64)
  set(CMAKE_SYSTEM_NAME Linux)
  set(CMAKE_SYSTEM_PROCESSOR aarch64)

  set(TOOLCHAIN_PREFIX "aarch64-linux-gnu" CACHE STRING "Cross toolchain prefix")
  set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}-gcc)
  set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)

  set(CMAKE_SYSROOT "${SYSROOT}")
  set(CMAKE_FIND_ROOT_PATH "${SYSROOT}")

  set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
  set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

  set(ENV{PKG_CONFIG_SYSROOT_DIR} "${SYSROOT}")
  set(ENV{PKG_CONFIG_LIBDIR} "${SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig:${SYSROOT}/usr/lib/pkgconfig:${SYSROOT}/usr/share/pkgconfig")
  set(ENV{PKG_CONFIG_PATH} "")
endif()

project(SmartDrone CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ENABLE_ARM_CPU_TUNING "Enable tuned ARM CPU flags for the deployment target" ON)
set(ARM_CPU_TUNE "cortex-a76" CACHE STRING "ARM CPU passed to -mcpu/-mtune when tuning is enabled")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

list(PREPEND CMAKE_PREFIX_PATH "${SYSROOT}/usr")

function(configure_common_target target_name)
  target_include_directories(${target_name} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/c_library_v2"
    ${OpenCV_INCLUDE_DIRS}
    ${SMART_DRONE_GPIOD_INCLUDE_DIRS}
  )
  if(SMART_DRONE_USE_LIBCAMERA)
    target_include_directories(${target_name} PRIVATE ${SMART_DRONE_LIBCAMERA_INCLUDE_DIRS})
    if(LIBCAMERA_CFLAGS_OTHER)
      target_compile_options(${target_name} PRIVATE ${LIBCAMERA_CFLAGS_OTHER})
    endif()
  endif()

  target_compile_options(${target_name} PRIVATE -O3 -w)
  if(SMART_DRONE_GPIOD_CFLAGS_OTHER)
    target_compile_options(${target_name} PRIVATE ${SMART_DRONE_GPIOD_CFLAGS_OTHER})
  endif()
  if(SMART_DRONE_GPIOD_LIBRARY_DIRS)
    target_link_directories(${target_name} PRIVATE ${SMART_DRONE_GPIOD_LIBRARY_DIRS})
  endif()
  if(CROSS_AARCH64 AND NOT SYSROOT STREQUAL "")
    target_link_options(${target_name} PRIVATE
      "-Wl,-rpath-link,${SYSROOT}/lib"
      "-Wl,-rpath-link,${SYSROOT}/usr/lib"
      "-Wl,-rpath-link,${SYSROOT}/lib/aarch64-linux-gnu"
      "-Wl,-rpath-link,${SYSROOT}/usr/lib/aarch64-linux-gnu")
  endif()
  target_link_libraries(${target_name} PRIVATE ${OpenCV_LIBS} Threads::Threads ${SMART_DRONE_GPIOD_LIBRARIES})
  target_compile_definitions(${target_name} PRIVATE SMART_DRONE_GPIOD_V2=$<BOOL:${SMART_DRONE_GPIOD_V2}>)
  if(SMART_DRONE_USE_LIBCAMERA)
    target_link_directories(${target_name} PRIVATE ${LIBCAMERA_LIBRARY_DIRS})
    target_link_libraries(${target_name} PRIVATE ${LIBCAMERA_LIBRARIES})
  endif()

  if(ENABLE_ARM_CPU_TUNING AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$" AND NOT ARM_CPU_TUNE STREQUAL "")
    target_compile_options(${target_name} PRIVATE "-mcpu=${ARM_CPU_TUNE}" "-mtune=${ARM_CPU_TUNE}")
  endif()
endfunction()

function(configure_orb_target target_name)
  find_package(Eigen3 QUIET)
  if(TARGET Eigen3::Eigen)
    target_link_libraries(${target_name} PRIVATE Eigen3::Eigen)
  else()
    target_include_directories(${target_name} PRIVATE "${SYSROOT}/usr/include/eigen3")
  endif()

  target_include_directories(${target_name} PRIVATE
    "${ORB_SLAM3_ROOT}"
    "${ORB_SLAM3_ROOT}/include"
    "${ORB_SLAM3_ROOT}/include/CameraModels"
    "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
    "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2"
    "${ORB_SLAM3_ROOT}/Thirdparty/g2o"
  )

  target_link_directories(${target_name} PRIVATE
    "${ORB_LIB_DIR}"
    "${G2O_LIB_DIR}"
    "${DBOW2_LIB_DIR}"
  )

  target_link_libraries(${target_name} PRIVATE
    ORB_SLAM3
    DBoW2
    g2o
  )
endfunction()

function(configure_xfeat_native_target target_name)
  if(NOT SMART_DRONE_ENABLE_XFEAT_NATIVE)
    return()
  endif()
  find_package(Torch REQUIRED)
  target_link_libraries(${target_name} PRIVATE ${TORCH_LIBRARIES})
  target_compile_definitions(${target_name} PRIVATE SMART_DRONE_XFEAT_NATIVE_AVAILABLE=1)
  target_include_directories(${target_name} PRIVATE ${TORCH_INCLUDE_DIRS})
  if(TORCH_CXX_FLAGS)
    separate_arguments(_SMART_DRONE_TORCH_CXX_FLAGS UNIX_COMMAND "${TORCH_CXX_FLAGS}")
    target_compile_options(${target_name} PRIVATE ${_SMART_DRONE_TORCH_CXX_FLAGS})
  endif()
endfunction()

if(BUILD_SMART_DRONE)
  set(ORB_SLAM3_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3" CACHE PATH "ORB_SLAM3 root")
  set(ORB_LIB_DIR "${ORB_LIB_DIR}" CACHE PATH "Directory containing libORB_SLAM3")
  if(ORB_LIB_DIR STREQUAL "")
    set(ORB_LIB_DIR "${ORB_SLAM3_ROOT}/lib" CACHE PATH "Directory containing libORB_SLAM3" FORCE)
  endif()

  set(G2O_LIB_DIR "${G2O_LIB_DIR}" CACHE PATH "Directory containing libg2o")
  if(G2O_LIB_DIR STREQUAL "")
    set(G2O_LIB_DIR "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib" CACHE PATH "Directory containing libg2o" FORCE)
  endif()

  set(DBOW2_LIB_DIR "${DBOW2_LIB_DIR}" CACHE PATH "Directory containing libDBoW2")
  if(DBOW2_LIB_DIR STREQUAL "")
    set(DBOW2_LIB_DIR "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib" CACHE PATH "Directory containing libDBoW2" FORCE)
  endif()
endif()

if(ENABLE_OFFLINE_REPLAY)
  set(ORB_SLAM3_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3")
  set(ORB_SLAM3_OUTPUT_DIR "${CMAKE_BINARY_DIR}/orb_host/lib" CACHE PATH "Host ORB_SLAM3 output directory" FORCE)
  set(DBOW2_OUTPUT_DIR "${CMAKE_BINARY_DIR}/orb_host/lib" CACHE PATH "Host DBoW2 output directory" FORCE)
  set(G2O_OUTPUT_DIR "${CMAKE_BINARY_DIR}/orb_host/lib" CACHE PATH "Host g2o output directory" FORCE)
  add_subdirectory(ORB_SLAM3 EXCLUDE_FROM_ALL)
endif()

if(BUILD_SMART_DRONE)
  find_package(Threads REQUIRED)
  find_package(OpenCV REQUIRED COMPONENTS core calib3d features2d imgproc videoio)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GPIOD REQUIRED libgpiod)
  set(SMART_DRONE_GPIOD_INCLUDE_DIRS ${GPIOD_INCLUDE_DIRS})
  set(SMART_DRONE_GPIOD_LIBRARY_DIRS ${GPIOD_LIBRARY_DIRS})
  set(SMART_DRONE_GPIOD_LIBRARIES ${GPIOD_LIBRARIES})
  set(SMART_DRONE_GPIOD_CFLAGS_OTHER ${GPIOD_CFLAGS_OTHER})
  if(GPIOD_VERSION VERSION_GREATER_EQUAL "2.0")
    set(SMART_DRONE_GPIOD_V2 ON)
  endif()
  if(SMART_DRONE_USE_LIBCAMERA)
    pkg_check_modules(LIBCAMERA REQUIRED libcamera)
    set(_SMART_DRONE_LIBCAMERA_CANDIDATES
      ${LIBCAMERA_INCLUDE_DIRS}
      ${LIBCAMERA_INCLUDEDIR}
      "${SYSROOT}/usr/include"
      "${SYSROOT}/usr/include/libcamera"
    )
    foreach(_candidate IN LISTS _SMART_DRONE_LIBCAMERA_CANDIDATES)
      if(NOT _candidate)
        continue()
      endif()
      if(EXISTS "${_candidate}/libcamera/framebuffer_allocator.h")
        list(APPEND SMART_DRONE_LIBCAMERA_INCLUDE_DIRS "${_candidate}")
      elseif(EXISTS "${_candidate}/framebuffer_allocator.h")
        get_filename_component(_candidate_parent "${_candidate}" DIRECTORY)
        if(EXISTS "${_candidate_parent}/libcamera/framebuffer_allocator.h")
          list(APPEND SMART_DRONE_LIBCAMERA_INCLUDE_DIRS "${_candidate_parent}")
        endif()
      endif()
    endforeach()
    list(REMOVE_DUPLICATES SMART_DRONE_LIBCAMERA_INCLUDE_DIRS)
    if(SMART_DRONE_LIBCAMERA_INCLUDE_DIRS STREQUAL "")
      message(FATAL_ERROR
        "libcamera was found via pkg-config, but framebuffer_allocator.h was not found. "
        "Check the sysroot headers under ${SYSROOT}.")
    endif()
    message(STATUS "Resolved libcamera include dirs: ${SMART_DRONE_LIBCAMERA_INCLUDE_DIRS}")
  endif()

  add_subdirectory(src/native)
  target_compile_definitions(smart_drone PRIVATE
    SMART_DRONE_CAMERA_PROVIDER_NAME="${SMART_DRONE_CAMERA_PROVIDER}"
  )
  if(SMART_DRONE_CAMERA_PROVIDER STREQUAL "libcamera_stereo_ov9281")
    target_compile_definitions(smart_drone PRIVATE SMART_DRONE_CAMERA_PROVIDER_LIBCAMERA_OV9281=1)
  elseif(SMART_DRONE_CAMERA_PROVIDER STREQUAL "uvc_stereo_opencv")
    target_compile_definitions(smart_drone PRIVATE SMART_DRONE_CAMERA_PROVIDER_UVC_OPENCV=1)
  endif()
  configure_common_target(smart_drone)
  configure_xfeat_native_target(smart_drone)
  configure_orb_target(smart_drone)
endif()

if(ENABLE_UNIT_TESTS)
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    message(STATUS "System GTest not found, fetching googletest...")
    set(FETCHCONTENT_QUIET OFF)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(googletest)
  else()
    message(STATUS "Using system GTest")
  endif()

endif()

if(ENABLE_UNIT_TESTS OR ENABLE_OFFLINE_REPLAY)
  enable_testing()
  add_subdirectory(tests)
endif()
