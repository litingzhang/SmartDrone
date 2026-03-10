cmake_minimum_required(VERSION 3.16)

option(CROSS_AARCH64 "Enable aarch64 cross compile" ON)
option(BUILD_CALIB_RECORDER "Build stereo calibration recorder" ON)
option(BUILD_SMART_DRONE "Build stereo VIO application" ON)

if(NOT BUILD_CALIB_RECORDER AND NOT BUILD_SMART_DRONE)
  message(FATAL_ERROR "At least one of BUILD_CALIB_RECORDER or BUILD_SMART_DRONE must be ON")
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

project(ov9281_apps CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

list(PREPEND CMAKE_PREFIX_PATH "${SYSROOT}/usr")

set(ORB_SLAM3_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3-master" CACHE PATH "ORB_SLAM3 root")
set(ORB_LIB_DIR    "${ORB_SLAM3_ROOT}/lib")
set(G2O_LIB_DIR    "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib")
set(DBOW2_LIB_DIR  "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib")

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

find_package(OpenCV REQUIRED)

function(configure_common_target target_name)
  target_include_directories(${target_name} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/c_library_v2"
    ${LIBCAMERA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )

  target_compile_options(${target_name} PRIVATE -O3 -w)
  target_link_directories(${target_name} PRIVATE ${LIBCAMERA_LIBRARY_DIRS})
  target_link_libraries(${target_name} PRIVATE
    ${LIBCAMERA_LIBRARIES}
    ${OpenCV_LIBS}
    pthread
    gpiod
  )
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

if(BUILD_CALIB_RECORDER)
  add_executable(calib_recorder
    src/ov9281_icm42688/stereo_imu_calib_recorder.cpp
  )
  configure_common_target(calib_recorder)
endif()

if(BUILD_SMART_DRONE)
  add_executable(smart_drone
    src/ov9281_icm42688/main.cpp
  )
  configure_common_target(smart_drone)
  configure_orb_target(smart_drone)
endif()
