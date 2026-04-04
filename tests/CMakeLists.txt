find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
find_package(Threads REQUIRED)
find_package(Eigen3 QUIET)

if(ENABLE_UNIT_TESTS)
  add_executable(smart_drone_unit_tests
    frame_timing_tracker_test.cpp
    mode_manager_test.cpp
    perception_pipeline_test.cpp
    replay_dataset_test.cpp
    replay_slam_runner_test.cpp
    runtime_config_service_test.cpp
    test_main.cpp
    test_support/replay_dataset.cpp
    test_support/replay_slam_runner.cpp
    ../src/native/core/application/config/app_args.cpp
    ../src/native/core/application/config/config_registry.cpp
    ../src/native/core/application/runtime/mode_manager.cpp
    ../src/native/core/application/runtime/runtime_config_service.cpp
    ../src/native/core/application/state/frame_timing_tracker.cpp
    ../src/native/core/application/state/perception_pipeline.cpp
    ../src/native/core/domain/runtime_mode.cpp
  )

  target_include_directories(smart_drone_unit_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../src/native"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3/include/CameraModels"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/Sophus"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/DBoW2"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/g2o"
  )

  if(TARGET Eigen3::Eigen)
    target_link_libraries(smart_drone_unit_tests PRIVATE Eigen3::Eigen)
  else()
    target_include_directories(smart_drone_unit_tests PRIVATE "/usr/include/eigen3")
  endif()

  target_compile_definitions(smart_drone_unit_tests PRIVATE
    TESTS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

  target_link_libraries(smart_drone_unit_tests PRIVATE
    GTest::gtest
    ${OpenCV_LIBS}
    Threads::Threads
  )

  include(GoogleTest)
  gtest_discover_tests(smart_drone_unit_tests)
endif()

if(ENABLE_OFFLINE_REPLAY)
  add_executable(smart_drone_offline_replay
    offline_replay_main.cpp
    test_support/replay_dataset.cpp
    test_support/replay_slam_runner.cpp
    ../src/native/adapters/slam/orbslam3_engine.cpp
    ../src/native/core/application/config/app_args.cpp
    ../src/native/core/application/state/frame_timing_tracker.cpp
    ../src/native/core/application/state/perception_pipeline.cpp
    ../src/native/core/domain/runtime_mode.cpp
  )

  target_include_directories(smart_drone_offline_replay PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
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
    TESTS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

  target_link_libraries(smart_drone_offline_replay PRIVATE
    ORB_SLAM3
    ${OpenCV_LIBS}
    Threads::Threads
  )
endif()
