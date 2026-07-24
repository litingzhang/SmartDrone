cmake_minimum_required(VERSION 3.16)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

include(FetchContent)

option(CROSS_AARCH64 "Enable aarch64 cross compile" ON)
option(BUILD_SMART_DRONE "Build unified stereo runtime application" ON)
option(ENABLE_UNIT_TESTS "Build unit tests" OFF)
option(ENABLE_OFFLINE_REPLAY "Build host-side offline replay tool" OFF)
set(SMART_DRONE_CAMERA_PROVIDER "libcamera_stereo_ov9281" CACHE STRING
    "Camera provider implementation compiled into the native runtime")
set(_SMART_DRONE_CAMERA_PROVIDERS
    "libcamera_stereo_ov9281;uvc_stereo_opencv;gz_stereo")
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
set(SMART_DRONE_VPI_AVAILABLE OFF)
set(SMART_DRONE_RESOLVED_VPI_ROOT "")
set(SMART_DRONE_TENSORRT_AVAILABLE OFF)
set(SMART_DRONE_OPENVINS_AVAILABLE OFF)
set(SMART_DRONE_OPENVINS_CERES_AVAILABLE OFF)
set(SMART_DRONE_GPIOD_INCLUDE_DIRS "")
set(SMART_DRONE_GPIOD_LIBRARY_DIRS "")
set(SMART_DRONE_GPIOD_LIBRARIES gpiod)
set(SMART_DRONE_GPIOD_CFLAGS_OTHER "")
option(SMART_DRONE_ENABLE_VPI "Enable NVIDIA VPI acceleration when headers and libraries are available" OFF)
option(SMART_DRONE_ENABLE_OPENCV_CUDA_ORB "Enable experimental OpenCV CUDA ORB extraction when supported" OFF)
option(SMART_DRONE_ENABLE_ORB_SLAM3 "Build the optional ORB-SLAM3 backend from src/native/adapters/slam/orb/orb_slam3" OFF)
option(SMART_DRONE_ENABLE_OPENVINS "Build the optional OpenVINS backend from the in-tree src/native OpenVINS sources" OFF)
option(SMART_DRONE_ENABLE_BOARD_IO "Build board SPI/GPIO sensor IO adapters" ON)
set(SMART_DRONE_VPI_ROOT "" CACHE PATH "NVIDIA VPI root, e.g. /opt/nvidia/vpi2")
set(SMART_DRONE_OPENVINS_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/src/native/adapters/slam/openvins/open_vins")

if(NOT BUILD_SMART_DRONE AND NOT ENABLE_UNIT_TESTS AND NOT ENABLE_OFFLINE_REPLAY)
  message(FATAL_ERROR "Enable BUILD_SMART_DRONE, ENABLE_UNIT_TESTS, or ENABLE_OFFLINE_REPLAY")
endif()

if(ENABLE_UNIT_TESTS AND CROSS_AARCH64)
  message(STATUS "Unit tests are host-side only, forcing CROSS_AARCH64=OFF")
  set(CROSS_AARCH64 OFF CACHE BOOL "Enable aarch64 cross compile" FORCE)
endif()

if(BUILD_SMART_DRONE AND SMART_DRONE_CAMERA_PROVIDER STREQUAL "gz_stereo"
   AND CROSS_AARCH64)
  message(FATAL_ERROR "gz_stereo is a host-only simulation camera provider")
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

set(SMART_DRONE_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(SMART_DRONE_ORB_MODULE_ROOT "${SMART_DRONE_REPO_ROOT}/src/native/adapters/slam/orb/orb_slam3")
set(SMART_DRONE_SOPHUS_ROOT "${SMART_DRONE_REPO_ROOT}/third_party/Sophus")
set(SMART_DRONE_DBOW2_ROOT "${SMART_DRONE_REPO_ROOT}/third_party/DBoW2")
set(SMART_DRONE_G2O_ROOT "${SMART_DRONE_REPO_ROOT}/third_party/g2o")
set(SMART_DRONE_OPENVINS_CORE_SOURCES "")
set(SMART_DRONE_OPENVINS_INIT_SOURCES "")
set(SMART_DRONE_OPENVINS_INIT_CERES_SOURCES "")
set(SMART_DRONE_OPENVINS_INIT_DYNAMIC_SOURCE "")
set(SMART_DRONE_OPENVINS_MSCKF_SOURCES "")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ENABLE_ARM_CPU_TUNING "Enable tuned ARM CPU flags for the deployment target" ON)
set(ARM_CPU_TUNE "cortex-a76" CACHE STRING "ARM CPU passed to -mcpu/-mtune when tuning is enabled")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

list(PREPEND CMAKE_PREFIX_PATH "${SYSROOT}/usr")

function(seed_smart_drone_openvins_jetson_prefix)
  if(NOT CROSS_AARCH64 OR NOT SMART_DRONE_ENABLE_OPENVINS)
    return()
  endif()

  set(_SMART_DRONE_OPENVINS_JETSON_PREFIX
    "${SYSROOT}/home/nvidia/openvins_deps/prefix/usr")
  if(NOT EXISTS "${_SMART_DRONE_OPENVINS_JETSON_PREFIX}")
    return()
  endif()

  list(PREPEND CMAKE_PREFIX_PATH "${_SMART_DRONE_OPENVINS_JETSON_PREFIX}")
  set(Boost_NO_SYSTEM_PATHS ON CACHE BOOL
      "Prefer Jetson OpenVINS Boost prefix when cross-compiling" FORCE)
  set(Boost_ROOT "${SYSROOT}/usr" CACHE PATH
      "Jetson sysroot Boost include prefix" FORCE)
  set(BOOST_ROOT "${Boost_ROOT}" CACHE PATH
      "Jetson sysroot Boost include prefix" FORCE)
  set(BOOST_INCLUDEDIR "${SYSROOT}/usr/include" CACHE PATH
      "Jetson sysroot Boost include dir" FORCE)
  set(BOOST_LIBRARYDIR
      "${_SMART_DRONE_OPENVINS_JETSON_PREFIX}/lib/aarch64-linux-gnu" CACHE PATH
      "Jetson OpenVINS Boost library dir" FORCE)
  set(Boost_INCLUDE_DIR "${BOOST_INCLUDEDIR}" CACHE PATH
      "Jetson OpenVINS Boost include dir" FORCE)
  set(Boost_LIBRARY_DIR_RELEASE "${BOOST_LIBRARYDIR}" CACHE PATH
      "Jetson OpenVINS Boost library dir" FORCE)
  set(Boost_LIBRARY_DIR_DEBUG "${BOOST_LIBRARYDIR}" CACHE PATH
      "Jetson OpenVINS Boost library dir" FORCE)
  set(Boost_SYSTEM_LIBRARY_RELEASE
      "${BOOST_LIBRARYDIR}/libboost_system.so" CACHE FILEPATH
      "Jetson OpenVINS boost_system" FORCE)
  set(Boost_SYSTEM_LIBRARY_DEBUG "${Boost_SYSTEM_LIBRARY_RELEASE}" CACHE FILEPATH
      "Jetson OpenVINS boost_system" FORCE)
  set(Boost_FILESYSTEM_LIBRARY_RELEASE
      "${BOOST_LIBRARYDIR}/libboost_filesystem.so" CACHE FILEPATH
      "Jetson OpenVINS boost_filesystem" FORCE)
  set(Boost_FILESYSTEM_LIBRARY_DEBUG
      "${Boost_FILESYSTEM_LIBRARY_RELEASE}" CACHE FILEPATH
      "Jetson OpenVINS boost_filesystem" FORCE)

  if(EXISTS "${_SMART_DRONE_OPENVINS_JETSON_PREFIX}/lib/cmake/Ceres/CeresConfig.cmake")
    set(Ceres_DIR
        "${_SMART_DRONE_OPENVINS_JETSON_PREFIX}/lib/cmake/Ceres" CACHE PATH
        "Jetson OpenVINS Ceres config" FORCE)
  endif()
  set(GLOG_INCLUDE_DIR_HINTS
      "${_SMART_DRONE_OPENVINS_JETSON_PREFIX}/include" CACHE PATH
      "Jetson OpenVINS glog include hint" FORCE)
  set(GLOG_LIBRARY_DIR_HINTS
      "${_SMART_DRONE_OPENVINS_JETSON_PREFIX}/lib/aarch64-linux-gnu" CACHE PATH
      "Jetson OpenVINS glog library hint" FORCE)
  set(GLOG_INCLUDE_DIR
      "${_SMART_DRONE_OPENVINS_JETSON_PREFIX}/include" CACHE PATH
      "Jetson OpenVINS glog include dir" FORCE)
  set(GLOG_LIBRARY
      "${_SMART_DRONE_OPENVINS_JETSON_PREFIX}/lib/aarch64-linux-gnu/libglog.so"
      CACHE FILEPATH "Jetson OpenVINS glog library" FORCE)
  set(GFLAGS_INCLUDE_DIR_HINTS
      "${_SMART_DRONE_OPENVINS_JETSON_PREFIX}/include" CACHE PATH
      "Jetson OpenVINS gflags include hint" FORCE)
  set(GFLAGS_LIBRARY_DIR_HINTS
      "${_SMART_DRONE_OPENVINS_JETSON_PREFIX}/lib/aarch64-linux-gnu" CACHE PATH
      "Jetson OpenVINS gflags library hint" FORCE)
  set(GFLAGS_INCLUDE_DIR
      "${_SMART_DRONE_OPENVINS_JETSON_PREFIX}/include" CACHE PATH
      "Jetson OpenVINS gflags include dir" FORCE)
  set(GFLAGS_LIBRARY
      "${_SMART_DRONE_OPENVINS_JETSON_PREFIX}/lib/aarch64-linux-gnu/libgflags.so"
      CACHE FILEPATH "Jetson OpenVINS gflags library" FORCE)
endfunction()

seed_smart_drone_openvins_jetson_prefix()

function(discard_host_ceres_for_cross_build)
  if(NOT CROSS_AARCH64 OR NOT DEFINED Ceres_DIR OR Ceres_DIR STREQUAL "")
    return()
  endif()
  if(Ceres_DIR MATCHES "-NOTFOUND$")
    unset(Ceres_DIR CACHE)
    return()
  endif()

  get_filename_component(_SMART_DRONE_CERES_DIR_ABS "${Ceres_DIR}" ABSOLUTE)
  get_filename_component(_SMART_DRONE_SYSROOT_ABS "${SYSROOT}" ABSOLUTE)
  string(FIND "${_SMART_DRONE_CERES_DIR_ABS}"
              "${_SMART_DRONE_SYSROOT_ABS}/"
              _SMART_DRONE_CERES_SYSROOT_POS)
  if(_SMART_DRONE_CERES_SYSROOT_POS EQUAL 0)
    return()
  endif()

  message(WARNING
    "Ignoring Ceres_DIR='${Ceres_DIR}' for aarch64 cross build because it is "
    "outside SYSROOT='${SYSROOT}'. OpenVINS dynamic initialization will be "
    "disabled unless an aarch64 Ceres package is provided.")
  unset(Ceres_DIR CACHE)
endfunction()

function(link_smart_drone_openvins_jetson_deps target_name)
  if(NOT CROSS_AARCH64 OR NOT SMART_DRONE_ENABLE_OPENVINS)
    return()
  endif()
  set(_SMART_DRONE_OPENVINS_JETSON_LIBDIR
    "${SYSROOT}/home/nvidia/openvins_deps/prefix/usr/lib/aarch64-linux-gnu")
  set(_SMART_DRONE_OPENVINS_JETSON_CERESDIR
    "${SYSROOT}/home/nvidia/openvins_deps/prefix/usr/lib")
  if(NOT EXISTS "${_SMART_DRONE_OPENVINS_JETSON_LIBDIR}")
    return()
  endif()

  target_link_directories(${target_name} PRIVATE
    "${_SMART_DRONE_OPENVINS_JETSON_LIBDIR}"
    "${_SMART_DRONE_OPENVINS_JETSON_CERESDIR}")
  target_link_options(${target_name} PRIVATE
    "-Wl,-rpath-link,${_SMART_DRONE_OPENVINS_JETSON_LIBDIR}"
    "-Wl,-rpath-link,${_SMART_DRONE_OPENVINS_JETSON_CERESDIR}")
  target_link_libraries(${target_name} PRIVATE
    "${_SMART_DRONE_OPENVINS_JETSON_CERESDIR}/libceres.so.1.14.0"
    "${_SMART_DRONE_OPENVINS_JETSON_LIBDIR}/libglog.so"
    "${_SMART_DRONE_OPENVINS_JETSON_LIBDIR}/libgflags.so"
    "${_SMART_DRONE_OPENVINS_JETSON_LIBDIR}/libspqr.so.2"
    "${_SMART_DRONE_OPENVINS_JETSON_LIBDIR}/libcholmod.so.3"
    "${_SMART_DRONE_OPENVINS_JETSON_LIBDIR}/libcxsparse.so.3"
    "${_SMART_DRONE_OPENVINS_JETSON_LIBDIR}/libamd.so.2"
    "${_SMART_DRONE_OPENVINS_JETSON_LIBDIR}/libcamd.so.2"
    "${_SMART_DRONE_OPENVINS_JETSON_LIBDIR}/libccolamd.so.2"
    "${_SMART_DRONE_OPENVINS_JETSON_LIBDIR}/libmetis.so.5"
    "${_SMART_DRONE_OPENVINS_JETSON_LIBDIR}/libatlas.so.3"
    "${_SMART_DRONE_OPENVINS_JETSON_LIBDIR}/libf77blas.so.3"
    "${SYSROOT}/usr/lib/aarch64-linux-gnu/libsuitesparseconfig.so.5")
endfunction()

function(resolve_smart_drone_vpi)
  set(SMART_DRONE_VPI_AVAILABLE OFF PARENT_SCOPE)
  set(SMART_DRONE_RESOLVED_VPI_ROOT "" PARENT_SCOPE)
  if(NOT SMART_DRONE_ENABLE_VPI)
    return()
  endif()
  set(_SMART_DRONE_VPI_CANDIDATE_ROOTS
    "${SMART_DRONE_VPI_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/output/vpi2_jetson"
    "${SYSROOT}/opt/nvidia/vpi2"
    "/opt/nvidia/vpi2"
  )
  foreach(_vpi_root IN LISTS _SMART_DRONE_VPI_CANDIDATE_ROOTS)
    if(NOT _vpi_root)
      continue()
    endif()
    if(EXISTS "${_vpi_root}/include/vpi/VPI.h" AND EXISTS "${_vpi_root}/lib/aarch64-linux-gnu/libnvvpi.so")
      set(SMART_DRONE_VPI_AVAILABLE ON PARENT_SCOPE)
      set(SMART_DRONE_RESOLVED_VPI_ROOT "${_vpi_root}" PARENT_SCOPE)
      return()
    endif()
  endforeach()
endfunction()

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
  if(SMART_DRONE_CAMERA_PROVIDER STREQUAL "gz_stereo")
    target_link_libraries(${target_name} PRIVATE gz-transport13::core)
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
  target_link_libraries(${target_name} PRIVATE ${OpenCV_LIBS} Threads::Threads)
  if(SMART_DRONE_ENABLE_BOARD_IO)
    target_link_libraries(${target_name} PRIVATE ${SMART_DRONE_GPIOD_LIBRARIES})
  endif()
  if(SMART_DRONE_ENABLE_VPI)
    if(SMART_DRONE_VPI_AVAILABLE)
      target_include_directories(${target_name} PRIVATE "${SMART_DRONE_RESOLVED_VPI_ROOT}/include")
      target_link_directories(${target_name} PRIVATE "${SMART_DRONE_RESOLVED_VPI_ROOT}/lib/aarch64-linux-gnu")
      target_link_libraries(${target_name} PRIVATE nvvpi)
      target_link_options(${target_name} PRIVATE
        "-Wl,-rpath-link,${SMART_DRONE_RESOLVED_VPI_ROOT}/lib/aarch64-linux-gnu"
        "-Wl,--allow-shlib-undefined")
      message(STATUS "VPI enabled for ${target_name}: ${SMART_DRONE_RESOLVED_VPI_ROOT}")
    else()
      message(WARNING "SMART_DRONE_ENABLE_VPI=ON but VPI root was not found; ${target_name} will use CPU fallback")
    endif()
  endif()
  if(SMART_DRONE_USE_LIBCAMERA)
    target_link_directories(${target_name} PRIVATE ${LIBCAMERA_LIBRARY_DIRS})
    target_link_libraries(${target_name} PRIVATE ${LIBCAMERA_LIBRARIES})
  endif()

  if(ENABLE_ARM_CPU_TUNING AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$" AND NOT ARM_CPU_TUNE STREQUAL "")
    target_compile_options(${target_name} PRIVATE "-mcpu=${ARM_CPU_TUNE}" "-mtune=${ARM_CPU_TUNE}")
  endif()
endfunction()

function(configure_sophus_target target_name)
  find_package(Eigen3 QUIET)
  if(TARGET Eigen3::Eigen)
    target_link_libraries(${target_name} PRIVATE Eigen3::Eigen)
  else()
    target_include_directories(${target_name} SYSTEM PRIVATE "${SYSROOT}/usr/include/eigen3")
  endif()

  target_include_directories(${target_name} SYSTEM PRIVATE
    "${SMART_DRONE_SOPHUS_ROOT}"
  )
endfunction()

function(configure_orb_target target_name)
  configure_sophus_target(${target_name})

  target_include_directories(${target_name} PRIVATE
    "${SMART_DRONE_ORB_MODULE_ROOT}"
    "${SMART_DRONE_ORB_MODULE_ROOT}/camera"
    "${SMART_DRONE_ORB_MODULE_ROOT}/camera/CameraModels"
    "${SMART_DRONE_ORB_MODULE_ROOT}/common"
    "${SMART_DRONE_ORB_MODULE_ROOT}/config"
    "${SMART_DRONE_ORB_MODULE_ROOT}/features"
    "${SMART_DRONE_ORB_MODULE_ROOT}/frame"
    "${SMART_DRONE_ORB_MODULE_ROOT}/geometry"
    "${SMART_DRONE_ORB_MODULE_ROOT}/graph_optimization"
    "${SMART_DRONE_ORB_MODULE_ROOT}/loop_closing"
    "${SMART_DRONE_ORB_MODULE_ROOT}/map"
    "${SMART_DRONE_ORB_MODULE_ROOT}/mapping"
    "${SMART_DRONE_ORB_MODULE_ROOT}/place_recognition"
    "${SMART_DRONE_ORB_MODULE_ROOT}/system"
    "${SMART_DRONE_ORB_MODULE_ROOT}/tracking"
  )
  target_include_directories(${target_name} SYSTEM PRIVATE
    "${SMART_DRONE_DBOW2_ROOT}"
    "${SMART_DRONE_G2O_ROOT}"
  )

  target_link_libraries(${target_name} PRIVATE
    ORB_SLAM3
    DBoW2
    g2o
  )
endfunction()

function(add_native_orb_slam3)
  if(NOT EXISTS "${SMART_DRONE_ORB_MODULE_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR
      "SMART_DRONE_ENABLE_ORB_SLAM3=ON requires ORB-SLAM3 sources at "
      "${SMART_DRONE_ORB_MODULE_ROOT}.")
  endif()
  if(TARGET ORB_SLAM3)
    return()
  endif()

  add_subdirectory("${SMART_DRONE_ORB_MODULE_ROOT}"
                   "${CMAKE_BINARY_DIR}/orb_slam3/src"
                   EXCLUDE_FROM_ALL)
endfunction()

function(resolve_smart_drone_openvins)
  set(SMART_DRONE_OPENVINS_AVAILABLE OFF PARENT_SCOPE)
  set(SMART_DRONE_OPENVINS_CERES_AVAILABLE OFF PARENT_SCOPE)
  if(NOT SMART_DRONE_ENABLE_OPENVINS)
    return()
  endif()
  if(NOT EXISTS "${SMART_DRONE_OPENVINS_ROOT}/ov_core/src/utils/sensor_data.h"
     OR NOT EXISTS "${SMART_DRONE_OPENVINS_ROOT}/ov_init/src/init/InertialInitializer.cpp"
     OR NOT EXISTS "${SMART_DRONE_OPENVINS_ROOT}/ov_msckf/src/core/VioManager.cpp")
    message(FATAL_ERROR
      "In-tree OpenVINS sources are missing from '${SMART_DRONE_OPENVINS_ROOT}'.")
  endif()
  set(SMART_DRONE_OPENVINS_CORE_SOURCES
    "${SMART_DRONE_OPENVINS_ROOT}/ov_core/src/utils/print.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_core/src/track/TrackKLT.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_core/src/track/TrackBase.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_core/src/track/TrackAruco.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_core/src/track/TrackDescriptor.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_core/src/types/Landmark.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_core/src/cpi/CpiV1.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_core/src/cpi/CpiV2.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_core/src/feat/FeatureDatabase.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_core/src/feat/Feature.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_core/src/feat/FeatureInitializer.cpp"
    PARENT_SCOPE)
  set(SMART_DRONE_OPENVINS_INIT_SOURCES
    "${SMART_DRONE_OPENVINS_ROOT}/ov_init/src/init/InertialInitializer.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_init/src/static/StaticInitializer.cpp"
    PARENT_SCOPE)
  set(SMART_DRONE_OPENVINS_INIT_CERES_SOURCES
    "${SMART_DRONE_OPENVINS_ROOT}/ov_init/src/dynamic/DynamicInitializer.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_init/src/ceres/Factor_ImuCPIv1.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_init/src/ceres/Factor_GenericPrior.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_init/src/ceres/Factor_ImageReprojCalib.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_init/src/ceres/State_JPLQuatLocal.cpp"
    PARENT_SCOPE)
  set(SMART_DRONE_OPENVINS_INIT_DYNAMIC_SOURCE
    "${SMART_DRONE_OPENVINS_ROOT}/ov_init/src/dynamic/DynamicInitializerStub.cpp"
    PARENT_SCOPE)
  set(SMART_DRONE_OPENVINS_MSCKF_SOURCES
    "${SMART_DRONE_OPENVINS_ROOT}/ov_msckf/src/core/VioManager.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_msckf/src/core/VioManagerHelper.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_msckf/src/update/UpdaterHelper.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_msckf/src/update/UpdaterMSCKF.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_msckf/src/update/UpdaterSLAM.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_msckf/src/update/UpdaterZeroVelocity.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_msckf/src/state/Propagator.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_msckf/src/state/StateHelper.cpp"
    "${SMART_DRONE_OPENVINS_ROOT}/ov_msckf/src/state/State.cpp"
    PARENT_SCOPE)
  set(SMART_DRONE_OPENVINS_AVAILABLE ON PARENT_SCOPE)
endfunction()

function(configure_superpoint_native_target target_name)
  set(_SMART_DRONE_JETSON_DEP_DIRS
    "${SYSROOT}/usr/local/cuda/targets/aarch64-linux/lib"
    "${SYSROOT}/usr/local/cuda/lib64"
    "${SYSROOT}/usr/local/cuda-11.4/targets/aarch64-linux/lib"
    "${SYSROOT}/usr/local/cuda-11.4/lib64"
    "${SYSROOT}/usr/lib/aarch64-linux-gnu"
    "${SYSROOT}/usr/lib/aarch64-linux-gnu/tegra"
    "${SYSROOT}/lib/aarch64-linux-gnu"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/jetson_runtime_libs/lib"
  )
  if(DEFINED ENV{SMART_DRONE_JETSON_LIB_DIR})
    list(PREPEND _SMART_DRONE_JETSON_DEP_DIRS "$ENV{SMART_DRONE_JETSON_LIB_DIR}")
  endif()
  set(_SMART_DRONE_CUDA_DEP_LIBS)
  set(_SMART_DRONE_CUDA_DEP_LINK_DIRS)
  set(_SMART_DRONE_MISSING_CUDA_DEPS)
  foreach(_dep_name IN ITEMS
    libcudart.so.11.0
    libcudnn.so.8)
    set(_dep_path "")
    foreach(_dep_dir IN LISTS _SMART_DRONE_JETSON_DEP_DIRS)
      if(NOT _dep_path AND EXISTS "${_dep_dir}/${_dep_name}")
        set(_dep_path "${_dep_dir}/${_dep_name}")
      endif()
    endforeach()
    if(_dep_path)
      list(APPEND _SMART_DRONE_CUDA_DEP_LIBS "${_dep_path}")
      get_filename_component(_dep_link_dir "${_dep_path}" DIRECTORY)
      list(APPEND _SMART_DRONE_CUDA_DEP_LINK_DIRS "${_dep_link_dir}")
    else()
      list(APPEND _SMART_DRONE_MISSING_CUDA_DEPS "${_dep_name}")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES _SMART_DRONE_CUDA_DEP_LINK_DIRS)
  foreach(_dep_link_dir IN LISTS _SMART_DRONE_CUDA_DEP_LINK_DIRS)
    target_link_options(${target_name} PRIVATE "-Wl,-rpath-link,${_dep_link_dir}")
  endforeach()
  message(STATUS "Resolved Jetson CUDA dependency dirs: ${_SMART_DRONE_CUDA_DEP_LINK_DIRS}")

  set(_SMART_DRONE_TENSORRT_INCLUDE_DIRS
    "${SYSROOT}/usr/include"
    "${SYSROOT}/usr/include/aarch64-linux-gnu"
  )
  set(_SMART_DRONE_TENSORRT_LIB_DIRS
    "${SYSROOT}/usr/lib/aarch64-linux-gnu"
    "${SYSROOT}/usr/lib/aarch64-linux-gnu/tegra"
    "${SYSROOT}/usr/local/cuda/targets/aarch64-linux/lib"
    "${SYSROOT}/usr/local/cuda-11.4/targets/aarch64-linux/lib"
    "${SYSROOT}/usr/local/tensorrt/lib"
  )
  set(_SMART_DRONE_TENSORRT_INCLUDE "")
  foreach(_trt_include_dir IN LISTS _SMART_DRONE_TENSORRT_INCLUDE_DIRS)
    if(NOT _SMART_DRONE_TENSORRT_INCLUDE AND EXISTS "${_trt_include_dir}/NvInfer.h")
      set(_SMART_DRONE_TENSORRT_INCLUDE "${_trt_include_dir}")
    endif()
  endforeach()
  set(_SMART_DRONE_TENSORRT_LIBS)
  foreach(_trt_lib_name IN ITEMS nvinfer nvinfer_plugin nvdla_compiler cudla)
    set(_trt_lib_path "")
    foreach(_trt_lib_dir IN LISTS _SMART_DRONE_TENSORRT_LIB_DIRS)
      if(NOT _trt_lib_path)
        file(GLOB _trt_lib_candidates "${_trt_lib_dir}/lib${_trt_lib_name}.so*")
        if(_trt_lib_candidates)
          list(SORT _trt_lib_candidates)
          list(REVERSE _trt_lib_candidates)
          list(GET _trt_lib_candidates 0 _trt_first_candidate)
          set(_trt_lib_path "${_trt_first_candidate}")
        endif()
      endif()
    endforeach()
    if(_trt_lib_path)
      list(APPEND _SMART_DRONE_TENSORRT_LIBS "${_trt_lib_path}")
    endif()
  endforeach()
  if(_SMART_DRONE_TENSORRT_INCLUDE AND _SMART_DRONE_TENSORRT_LIBS)
    if(_SMART_DRONE_MISSING_CUDA_DEPS)
      message(FATAL_ERROR
        "Missing Jetson runtime libraries required by TensorRT SuperPoint: ${_SMART_DRONE_MISSING_CUDA_DEPS}. "
        "Sync CUDA/cuDNN into the sysroot or set SMART_DRONE_JETSON_LIB_DIR.")
    endif()
    message(STATUS "Resolved TensorRT include: ${_SMART_DRONE_TENSORRT_INCLUDE}")
    message(STATUS "Resolved TensorRT libs: ${_SMART_DRONE_TENSORRT_LIBS}")
    set(SMART_DRONE_TENSORRT_AVAILABLE ON PARENT_SCOPE)
    target_include_directories(${target_name} PRIVATE
      "${_SMART_DRONE_TENSORRT_INCLUDE}"
      "${SYSROOT}/usr/local/cuda/targets/aarch64-linux/include"
      "${SYSROOT}/usr/local/cuda-11.4/targets/aarch64-linux/include"
    )
    target_link_libraries(${target_name} PRIVATE ${_SMART_DRONE_TENSORRT_LIBS})
    target_link_libraries(${target_name} PRIVATE ${_SMART_DRONE_CUDA_DEP_LIBS})
    target_link_libraries(${target_name} PRIVATE ${CMAKE_DL_LIBS})
  else()
    message(STATUS "TensorRT headers/libs not found in sysroot; SuperPoint TensorRT backend disabled")
  endif()
endfunction()

if((BUILD_SMART_DRONE OR ENABLE_OFFLINE_REPLAY) AND SMART_DRONE_ENABLE_ORB_SLAM3)
  add_native_orb_slam3()
endif()

if(BUILD_SMART_DRONE)
  find_package(Threads REQUIRED)
  find_package(OpenCV REQUIRED COMPONENTS core calib3d features2d imgproc video videoio)
  if(SMART_DRONE_CAMERA_PROVIDER STREQUAL "gz_stereo")
    find_package(gz-transport13 REQUIRED)
  endif()
  resolve_smart_drone_openvins()
  if(SMART_DRONE_ENABLE_OPENVINS)
    set(Boost_NO_BOOST_CMAKE ON)
    find_package(Boost REQUIRED COMPONENTS system filesystem)
    discard_host_ceres_for_cross_build()
    find_package(Ceres QUIET)
    if(Ceres_FOUND)
      set(SMART_DRONE_OPENVINS_CERES_AVAILABLE ON)
    else()
      message(WARNING "Ceres not found; OpenVINS will build without dynamic initialization support")
    endif()
  endif()
  if(SMART_DRONE_ENABLE_BOARD_IO)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GPIOD REQUIRED libgpiod)
    set(SMART_DRONE_GPIOD_INCLUDE_DIRS ${GPIOD_INCLUDE_DIRS})
    set(SMART_DRONE_GPIOD_LIBRARY_DIRS ${GPIOD_LIBRARY_DIRS})
    set(SMART_DRONE_GPIOD_LIBRARIES ${GPIOD_LIBRARIES})
    set(SMART_DRONE_GPIOD_CFLAGS_OTHER ${GPIOD_CFLAGS_OTHER})
    if(GPIOD_VERSION VERSION_GREATER_EQUAL "2.0")
      set(SMART_DRONE_GPIOD_V2 ON)
    endif()
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

  resolve_smart_drone_vpi()
  add_executable(smart_drone)
  configure_superpoint_native_target(smart_drone)
  add_subdirectory(src/native)
  configure_common_target(smart_drone)
  configure_sophus_target(smart_drone)
  if(SMART_DRONE_ENABLE_OPENVINS)
    target_sources(smart_drone PRIVATE
      ${SMART_DRONE_OPENVINS_CORE_SOURCES}
      ${SMART_DRONE_OPENVINS_INIT_SOURCES}
      ${SMART_DRONE_OPENVINS_MSCKF_SOURCES}
    )
    if(SMART_DRONE_OPENVINS_CERES_AVAILABLE)
      target_sources(smart_drone PRIVATE
        ${SMART_DRONE_OPENVINS_INIT_CERES_SOURCES}
        "${SMART_DRONE_REPO_ROOT}/src/native/adapters/slam/openvins/openvins_dynamic_init_enabled.cpp"
      )
    else()
      target_sources(smart_drone PRIVATE
        ${SMART_DRONE_OPENVINS_INIT_DYNAMIC_SOURCE}
        "${SMART_DRONE_REPO_ROOT}/src/native/adapters/slam/openvins/openvins_dynamic_init_disabled.cpp"
      )
    endif()
    target_include_directories(smart_drone PRIVATE
      "${SMART_DRONE_OPENVINS_ROOT}/ov_core/src"
      "${SMART_DRONE_OPENVINS_ROOT}/ov_init/src"
      "${SMART_DRONE_OPENVINS_ROOT}/ov_msckf/src"
    )
    if(SMART_DRONE_OPENVINS_CERES_AVAILABLE AND CERES_INCLUDE_DIRS)
      target_include_directories(smart_drone PRIVATE ${CERES_INCLUDE_DIRS})
    endif()
    target_link_libraries(smart_drone PRIVATE ${Boost_LIBRARIES})
    if(SMART_DRONE_OPENVINS_CERES_AVAILABLE AND NOT CROSS_AARCH64)
      if(TARGET Ceres::ceres)
        target_link_libraries(smart_drone PRIVATE Ceres::ceres)
      else()
        target_link_libraries(smart_drone PRIVATE ${CERES_LIBRARIES})
      endif()
    endif()
    link_smart_drone_openvins_jetson_deps(smart_drone)
  endif()
  if(SMART_DRONE_ENABLE_ORB_SLAM3)
    configure_orb_target(smart_drone)
  endif()
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
  resolve_smart_drone_openvins()
  if(SMART_DRONE_ENABLE_OPENVINS AND NOT BUILD_SMART_DRONE)
    set(Boost_NO_BOOST_CMAKE ON)
    find_package(Boost REQUIRED COMPONENTS system filesystem)
    if(DEFINED GLOG_INCLUDE_DIR_HINTS AND NOT GLOG_INCLUDE_DIR)
      set(GLOG_INCLUDE_DIR "${GLOG_INCLUDE_DIR_HINTS}" CACHE PATH
          "Preseeded glog include directory for Ceres" FORCE)
    endif()
    if(DEFINED GLOG_LIBRARY_DIR_HINTS AND NOT GLOG_LIBRARY)
      find_library(_SMART_DRONE_PRESEEDED_GLOG NAMES glog
                   HINTS "${GLOG_LIBRARY_DIR_HINTS}")
      if(_SMART_DRONE_PRESEEDED_GLOG)
        set(GLOG_LIBRARY "${_SMART_DRONE_PRESEEDED_GLOG}" CACHE FILEPATH
            "Preseeded glog library for Ceres" FORCE)
      endif()
    endif()
    if(DEFINED GFLAGS_INCLUDE_DIR_HINTS AND NOT GFLAGS_INCLUDE_DIR)
      set(GFLAGS_INCLUDE_DIR "${GFLAGS_INCLUDE_DIR_HINTS}" CACHE PATH
          "Preseeded gflags include directory for Ceres" FORCE)
    endif()
    if(DEFINED GFLAGS_LIBRARY_DIR_HINTS AND NOT GFLAGS_LIBRARY)
      find_library(_SMART_DRONE_PRESEEDED_GFLAGS NAMES gflags
                   HINTS "${GFLAGS_LIBRARY_DIR_HINTS}")
      if(_SMART_DRONE_PRESEEDED_GFLAGS)
        set(GFLAGS_LIBRARY "${_SMART_DRONE_PRESEEDED_GFLAGS}" CACHE FILEPATH
            "Preseeded gflags library for Ceres" FORCE)
      endif()
    endif()
    discard_host_ceres_for_cross_build()
    find_package(Ceres QUIET)
    if(Ceres_FOUND)
      set(SMART_DRONE_OPENVINS_CERES_AVAILABLE ON)
    else()
      message(WARNING "Ceres not found; OpenVINS replay build will disable dynamic initialization support")
    endif()
  endif()
  enable_testing()
  add_subdirectory(tests)
endif()
