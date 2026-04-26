find_package(OpenCV REQUIRED COMPONENTS core imgcodecs features2d)
find_package(Threads REQUIRED)
find_package(Eigen3 QUIET)

if(ENABLE_UNIT_TESTS)
  message(STATUS "ENABLE_UNIT_TESTS is on, but only EuRoC offline replay tests are kept in this repo.")
endif()

if(ENABLE_OFFLINE_REPLAY)
  find_package(Python3 COMPONENTS Interpreter QUIET)

  set(EUROC_TEST_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/euroc")

  add_executable(smart_drone_offline_replay
    euroc/offline_replay_main.cpp
    euroc/support/replay_dataset.cpp
    euroc/support/replay_slam_runner.cpp
    ../src/native/adapters/slam/orbslam3_engine.cpp
    ../src/native/adapters/slam/xfeat_frontend_client.cpp
    ../src/native/adapters/slam/xfeat_native_extractor.cpp
    ../src/native/core/application/config/app_args.cpp
    ../src/native/core/application/state/frame_timing_tracker.cpp
    ../src/native/core/application/state/perception_pipeline.cpp
    ../src/native/core/domain/runtime_mode.cpp
  )

  target_include_directories(smart_drone_offline_replay PRIVATE
    "${EUROC_TEST_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../src/native"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3/include/CameraModels"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/Sophus"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/DBoW2"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/g2o"
  )

  if(TARGET Eigen3::Eigen)
    target_link_libraries(smart_drone_offline_replay PRIVATE Eigen3::Eigen)
  else()
    target_include_directories(smart_drone_offline_replay PRIVATE "/usr/include/eigen3")
  endif()

  target_compile_definitions(smart_drone_offline_replay PRIVATE
    TESTS_SOURCE_DIR="${EUROC_TEST_ROOT}"
  )

  target_link_libraries(smart_drone_offline_replay PRIVATE
    ORB_SLAM3
    ${OpenCV_LIBS}
    Threads::Threads
  )

  set(EUROC_REGRESSION_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/euroc_regression")
  if(Python3_EXECUTABLE)
    set(EUROC_REGRESSION_PYTHON "${Python3_EXECUTABLE}")
  else()
    set(EUROC_REGRESSION_PYTHON "")
  endif()

  add_test(
    NAME OfflineReplayEurocRegression
    COMMAND "${CMAKE_COMMAND}"
            -DREPLAY_EXE=$<TARGET_FILE:smart_drone_offline_replay>
            -DPYTHON_EXECUTABLE=${EUROC_REGRESSION_PYTHON}
            -DEVALUATOR=${EUROC_TEST_ROOT}/evaluate_euroc_regression.py
            -DOUTPUT_DIR=${EUROC_REGRESSION_OUTPUT_DIR}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P "${EUROC_TEST_ROOT}/run_euroc_regression.cmake"
  )
endif()
