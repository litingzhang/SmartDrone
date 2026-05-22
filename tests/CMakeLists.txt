find_package(OpenCV REQUIRED COMPONENTS core calib3d features2d imgcodecs imgproc video)
find_package(Threads REQUIRED)
find_package(Eigen3 QUIET)

if(ENABLE_UNIT_TESTS)
  add_executable(epg_tests
    epg/epg_test.cpp
    ../src/native/common/epg/epg_context.cpp
    ../src/native/common/epg/epg.cpp
    ../src/native/common/epg/epg_dot.cpp
    ../src/native/common/epg/epg_json.cpp
    ../src/native/common/epg/epg_mermaid.cpp
    ../src/native/common/epg/epg_task_runner.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer.cpp
    ../src/native/core/application/epg/epg_task_manifest.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report.cpp
    ../src/native/core/application/runtime/epg_dfx_snapshot.cpp
    ../src/native/core/application/runtime/epg_graph_lifecycle.cpp
    ../src/native/core/application/runtime/epg_redeploy_coordinator.cpp
    ../src/native/core/application/runtime/system_runtime_messages.cpp
  )

  target_include_directories(epg_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../src/native"
  )

  target_compile_definitions(epg_tests PRIVATE
    TEST_EPG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/epg"
  )

  target_link_libraries(epg_tests PRIVATE
    GTest::gtest_main
    Threads::Threads
  )

  add_test(NAME EventPipelineGraphTests COMMAND epg_tests)

  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_EXECUTABLE)
    add_test(
      NAME EventPipelineGraphSolver
      COMMAND "${Python3_EXECUTABLE}"
              "${CMAKE_CURRENT_SOURCE_DIR}/epg/run_epg_solver_test.py"
              "${CMAKE_SOURCE_DIR}"
              "${CMAKE_CURRENT_BINARY_DIR}/epg_solver"
    )
    add_test(
      NAME NativeArchitectureBoundaries
      COMMAND "${Python3_EXECUTABLE}"
              "${CMAKE_CURRENT_SOURCE_DIR}/architecture/test_native_boundaries.py"
              "${CMAKE_SOURCE_DIR}"
    )
    add_test(
      NAME CppCodingStyle
      COMMAND "${Python3_EXECUTABLE}"
              "${CMAKE_CURRENT_SOURCE_DIR}/architecture/test_cpp_coding_style.py"
              "${CMAKE_SOURCE_DIR}"
    )
  else()
    message(STATUS "Python3 interpreter not available; skipping Python tests")
  endif()
endif()

if(ENABLE_OFFLINE_REPLAY)
  find_package(Python3 COMPONENTS Interpreter QUIET)

  set(EUROC_TEST_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/euroc")

  add_executable(smart_drone_offline_replay
    euroc/offline_replay_main.cpp
    euroc/support/replay_dataset.cpp
    euroc/support/replay_slam_runner.cpp
    ../src/native/adapters/slam/dpvo_runtime_options.cpp
    ../src/native/adapters/slam/slam_engine_factory.cpp
    ../src/native/adapters/slam/slam_mode_state.cpp
    ../src/native/adapters/slam/klt_slam_engine.cpp
    ../src/native/adapters/slam/klt_continuous_frontend.cpp
    ../src/native/adapters/slam/klt_loop_closure_backend.cpp
    ../src/native/adapters/slam/klt_mode_utils.cpp
    ../src/native/adapters/slam/klt_per_frame_frontend.cpp
    ../src/native/adapters/slam/klt_pnp_observation_builder.cpp
    ../src/native/adapters/slam/klt_pose_estimator.cpp
    ../src/native/adapters/slam/klt_track_manager.cpp
    ../src/native/adapters/slam/feature_point_tracking.cpp
    ../src/native/adapters/slam/stereo_feature_frontend_runner.cpp
    ../src/native/adapters/slam/descriptor_geometry.cpp
    ../src/native/adapters/slam/stereo_matching.cpp
    ../src/native/adapters/slam/stereo_pair_builder.cpp
    ../src/native/adapters/slam/stereo_feature_packet.cpp
    ../src/native/adapters/slam/temporal_stereo.cpp
    ../src/native/adapters/slam/slam_image_utils.cpp
    ../src/native/adapters/slam/slam_output_utils.cpp
    ../src/native/adapters/slam/slam_output_optional.cpp
    ../src/native/adapters/slam/slam_pose_utils.cpp
    ../src/native/adapters/slam/stereo_calibration.cpp
    ../src/native/adapters/slam/stereo_frame_preprocessor.cpp
    ../src/native/adapters/slam/stereo_geometry.cpp
    ../src/native/adapters/slam/visual_pnp_pose_backend.cpp
    ../src/native/adapters/slam/visual_feature_frontend_client.cpp
    ../src/native/adapters/slam/superpoint_runtime_options.cpp
    ../src/native/adapters/slam/superpoint_lightglue_frontend_client.cpp
    ../src/native/core/application/config/app_args.cpp
    ../src/native/core/application/config/slam_backend_availability.cpp
    ../src/native/core/application/state/frame_timing_tracker.cpp
    ../src/native/core/application/state/perception_pipeline.cpp
    ../src/native/core/domain/runtime_mode.cpp
  )

  target_include_directories(smart_drone_offline_replay PRIVATE
    "${EUROC_TEST_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../src/native"
    "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/Sophus"
  )

  if(SMART_DRONE_ENABLE_ORB_SLAM3)
    target_sources(smart_drone_offline_replay PRIVATE
      ../src/native/adapters/slam/slam_engine_adapter.cpp
      ../src/native/adapters/slam/slam_mode_strategy.cpp
      ../src/native/adapters/slam/slam_tracking_backend.cpp
      ../src/native/adapters/slam/orb_mode_strategy.cpp
      ../src/native/adapters/slam/visual_feature_lightglue_mode_strategy.cpp
      ../src/native/adapters/slam/slam_output_optional_orb.cpp
      ../src/native/adapters/slam/orb_feature_utils.cpp
      ../src/native/adapters/slam/orb_visual_components.cpp
      ../src/native/adapters/slam/orb_slam3_runtime.cpp
      ../src/native/adapters/slam/orb_slam3_backend.cpp
      ../src/native/core/application/config/slam_backend_availability_orb.cpp
    )
    set_source_files_properties(
      ../src/native/adapters/slam/slam_output_optional.cpp
      ../src/native/core/application/config/slam_backend_availability.cpp
      PROPERTIES HEADER_FILE_ONLY ON)
    configure_orb_target(smart_drone_offline_replay)
  endif()

  if(TARGET Eigen3::Eigen)
    target_link_libraries(smart_drone_offline_replay PRIVATE Eigen3::Eigen)
  else()
    target_include_directories(smart_drone_offline_replay PRIVATE "/usr/include/eigen3")
  endif()

  target_compile_definitions(smart_drone_offline_replay PRIVATE
    TESTS_SOURCE_DIR="${EUROC_TEST_ROOT}"
  )

  set(SMART_DRONE_REPLAY_VPI_SOURCE
    ../src/native/adapters/slam/klt_vpi_acceleration_stub.cpp)
  set(SMART_DRONE_REPLAY_DPVO_SOURCE
    ../src/native/adapters/slam/dpvo_tensorrt_engine.cpp)
  set(SMART_DRONE_REPLAY_SUPERPOINT_SOURCE
    ../src/native/adapters/slam/superpoint_native_extractor.cpp)

  if(SMART_DRONE_ENABLE_VPI)
    resolve_smart_drone_vpi()
    if(SMART_DRONE_VPI_AVAILABLE)
      set(SMART_DRONE_REPLAY_VPI_SOURCE
        ../src/native/adapters/slam/klt_vpi_acceleration.cpp)
      target_include_directories(smart_drone_offline_replay PRIVATE "${SMART_DRONE_RESOLVED_VPI_ROOT}/include")
      target_link_directories(smart_drone_offline_replay PRIVATE "${SMART_DRONE_RESOLVED_VPI_ROOT}/lib/aarch64-linux-gnu")
      target_link_libraries(smart_drone_offline_replay PRIVATE nvvpi)
      target_link_options(smart_drone_offline_replay PRIVATE
        "-Wl,-rpath-link,${SMART_DRONE_RESOLVED_VPI_ROOT}/lib/aarch64-linux-gnu"
        "-Wl,--allow-shlib-undefined")
      message(STATUS "VPI enabled for smart_drone_offline_replay: ${SMART_DRONE_RESOLVED_VPI_ROOT}")
    else()
      message(WARNING "SMART_DRONE_ENABLE_VPI=ON but VPI root was not found; replay will use CPU fallback")
    endif()
  endif()

  if(CROSS_AARCH64)
    configure_superpoint_native_target(smart_drone_offline_replay)
    if(SMART_DRONE_TENSORRT_AVAILABLE)
      set(SMART_DRONE_REPLAY_DPVO_SOURCE
        ../src/native/adapters/slam/dpvo_tensorrt_engine_trt.cpp)
      set(SMART_DRONE_REPLAY_SUPERPOINT_SOURCE
        ../src/native/adapters/slam/superpoint_native_extractor_trt.cpp)
    endif()
  endif()

  target_sources(smart_drone_offline_replay PRIVATE
    ${SMART_DRONE_REPLAY_VPI_SOURCE}
    ${SMART_DRONE_REPLAY_DPVO_SOURCE}
    ${SMART_DRONE_REPLAY_SUPERPOINT_SOURCE}
  )

  target_link_libraries(smart_drone_offline_replay PRIVATE
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
