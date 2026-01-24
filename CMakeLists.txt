cmake_minimum_required(VERSION 3.16)

option(CROSS_AARCH64 "Enable aarch64 cross compile" ON)

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

project(stereo_vslam CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Release)

list(PREPEND CMAKE_PREFIX_PATH "${SYSROOT}/usr")

set(ORB_SLAM3_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3-master" CACHE PATH "ORB_SLAM3 root")
set(ORB_LIB_DIR    "${ORB_SLAM3_ROOT}/lib")
set(G2O_LIB_DIR    "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib")
set(DBOW2_LIB_DIR  "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib")

file(GLOB_RECURSE SRC CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
add_executable(stereo_vslam ${SRC})

target_include_directories(stereo_vslam PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_compile_options(stereo_vslam PRIVATE -O3 -w)

find_package(Eigen3 QUIET)
if(TARGET Eigen3::Eigen)
  target_link_libraries(stereo_vslam PRIVATE Eigen3::Eigen)
else()
  target_include_directories(stereo_vslam PRIVATE "${SYSROOT}/usr/include/eigen3")
endif()

target_include_directories(stereo_vslam PRIVATE
  "${ORB_SLAM3_ROOT}"
  "${ORB_SLAM3_ROOT}/include"
  "${ORB_SLAM3_ROOT}/include/CameraModels"
  "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
  "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2"
  "${ORB_SLAM3_ROOT}/Thirdparty/g2o"
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)
target_include_directories(stereo_vslam PRIVATE ${LIBCAMERA_INCLUDE_DIRS})
target_link_directories(stereo_vslam PRIVATE ${LIBCAMERA_LIBRARY_DIRS})
target_link_libraries(stereo_vslam PRIVATE ${LIBCAMERA_LIBRARIES})

find_package(OpenCV REQUIRED)
target_include_directories(stereo_vslam PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stereo_vslam PRIVATE ${OpenCV_LIBS})

target_link_directories(stereo_vslam PRIVATE
  "${ORB_LIB_DIR}"
  "${G2O_LIB_DIR}"
  "${DBOW2_LIB_DIR}"
)

target_link_libraries(stereo_vslam PRIVATE
  ORB_SLAM3
  DBoW2
  g2o
  pthread
)
