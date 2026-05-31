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
    ../src/native/common/epg/epg_trigger_modes.cpp
    ../src/native/common/environment.cpp
    ../src/native/common/numeric_parse.cpp
    ../src/native/core/application/config/orb_acceleration_config.cpp
    ../src/native/core/application/epg/epg_discrete_global_solver.cpp
    ../src/native/core/application/epg/epg_graph_runtime.cpp
    ../src/native/core/application/epg/epg_cpu_binding_solver_exact.cpp
    ../src/native/core/application/epg/epg_cpu_binding_solver_heuristic.cpp
    ../src/native/core/application/epg/epg_cpu_binding_solver_support.cpp
    ../src/native/core/application/epg/epg_cpu_binding_solver.cpp
    ../src/native/core/application/epg/epg_registry.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_apply.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_candidates.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_cpu_binding.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_entry.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_io.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_plan.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_queue_candidates.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_report.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_report_decisions.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_task_apply.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_topology.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_topology_solver.cpp
    ../src/native/core/application/epg/epg_solver_primitives.cpp
    ../src/native/core/application/epg/epg_solver_report_score.cpp
    ../src/native/core/application/epg/epg_solver_topology_primitives.cpp
    ../src/native/core/application/epg/epg_task_decision_primitives.cpp
    ../src/native/core/application/epg/epg_task_evaluation_primitives.cpp
    ../src/native/core/application/epg/epg_task_dependency_graph.cpp
    ../src/native/core/application/epg/epg_task_manifest.cpp
    ../src/native/core/application/epg/epg_task_manifest_catalog.cpp
    ../src/native/core/application/epg/epg_task_manifest_internal.cpp
    ../src/native/core/application/epg/epg_task_manifest_optimized_graph_validation.cpp
    ../src/native/core/application/epg/epg_task_manifest_registry.cpp
    ../src/native/core/application/epg/epg_task_manifest_runtime_tuning.cpp
    ../src/native/core/application/epg/epg_task_manifest_validation.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_decisions.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_global_optimum.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_global_optimum_support.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_metadata.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_queue_optimum.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_schedule.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_task_decisions.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_task_optimum.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_task_penalty.cpp
    ../src/native/core/application/runtime/epg_dfx_snapshot.cpp
    ../src/native/core/application/runtime/epg_graph_lifecycle.cpp
    ../src/native/core/application/runtime/epg_redeploy_coordinator.cpp
    ../src/native/core/application/runtime/system_runtime_messages.cpp
  )

  target_include_directories(epg_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../src/native"
  )

  target_link_libraries(epg_tests PRIVATE
    GTest::gtest_main
    Threads::Threads
  )

  add_test(NAME EventPipelineGraphTests COMMAND epg_tests)

  add_executable(application_state_tests
    application/avoidance_state_payload_test.cpp
    application/calib_save_pacing_port_test.cpp
    application/frame_timing_tracker_test.cpp
    application/goal_cache_test.cpp
    application/imu_buffer_test.cpp
    application/local_occupancy_map_test.cpp
    application/live_pose_runtime_snapshots_test.cpp
    application/live_pose_state_test.cpp
    application/obstacle_avoidance_policy_test.cpp
    application/payload_builders_test.cpp
    application/px4_udp_hooks_test.cpp
    application/runtime_config_modules_test.cpp
    application/runtime_session_supervisor_test.cpp
    ../src/native/common/environment.cpp
    ../src/native/common/numeric_parse.cpp
    ../src/native/common/time_utils.cpp
    ../src/native/common/tlv/crc16_ccitt_false.cpp
    ../src/native/common/tlv/goal_cache.cpp
    ../src/native/common/tlv/tlv_pack.cpp
    ../src/native/core/application/config/app_args.cpp
    ../src/native/core/application/config/capability_catalog.cpp
    ../src/native/core/application/config/config_registry.cpp
    ../src/native/core/application/config/orb_acceleration_config.cpp
    ../src/native/core/application/config/slam_backend_availability.cpp
    ../src/native/core/application/runtime/avoidance_state_payload.cpp
    ../src/native/core/application/session/calib/calib_save_pacing_port.cpp
    ../src/native/core/application/session/calib/calib_storage_helpers.cpp
    ../src/native/core/application/session/slam/slam_settings_loader.cpp
    ../src/native/core/application/runtime/hover_algorithm_plugin.cpp
    ../src/native/core/application/runtime/local_occupancy_map.cpp
    ../src/native/core/application/runtime/live_pose_runtime_snapshots.cpp
    ../src/native/core/application/runtime/mode_manager.cpp
    ../src/native/core/application/runtime/obstacle_avoidance_config.cpp
    ../src/native/core/application/runtime/obstacle_avoidance_policy.cpp
    ../src/native/core/application/runtime/payload_builders.cpp
    ../src/native/core/application/runtime/px4_udp_hooks.cpp
    ../src/native/core/application/runtime/runtime_config_application.cpp
    ../src/native/core/application/runtime/runtime_config_message.cpp
    ../src/native/core/application/runtime/runtime_config_projection.cpp
    ../src/native/core/application/runtime/runtime_config_validation.cpp
    ../src/native/core/application/runtime/runtime_config_update_builder.cpp
    ../src/native/core/application/runtime/runtime_config_value_applier.cpp
    ../src/native/core/application/runtime/runtime_config_value_assignments.cpp
    ../src/native/core/application/runtime/runtime_session_supervisor.cpp
    ../src/native/core/domain/runtime_mode.cpp
    ../src/native/core/application/state/frame_timing_tracker.cpp
    ../src/native/core/application/state/imu_buffer.cpp
    ../src/native/core/application/state/live_pose_state.cpp
    ../src/native/core/ports/slam_tracking_state.cpp
  )

  target_include_directories(application_state_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../src/native"
    "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/Sophus"
  )

  target_link_libraries(application_state_tests PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
  )

  if(TARGET Eigen3::Eigen)
    target_link_libraries(application_state_tests PRIVATE Eigen3::Eigen)
  else()
    target_include_directories(application_state_tests SYSTEM PRIVATE
      "/usr/include/eigen3")
  endif()

  add_test(NAME ApplicationStateTests COMMAND application_state_tests)

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
  set(SMART_DRONE_REPLAY_BACKEND_AVAILABILITY_SOURCE
    ../src/native/core/application/config/slam_backend_availability.cpp)
  if(SMART_DRONE_ENABLE_ORB_SLAM3 AND SMART_DRONE_ENABLE_OPENVINS)
    set(SMART_DRONE_REPLAY_BACKEND_AVAILABILITY_SOURCE
      ../src/native/core/application/config/slam_backend_availability_orb_openvins.cpp)
  elseif(SMART_DRONE_ENABLE_ORB_SLAM3)
    set(SMART_DRONE_REPLAY_BACKEND_AVAILABILITY_SOURCE
      ../src/native/core/application/config/slam_backend_availability_orb.cpp)
  elseif(SMART_DRONE_ENABLE_OPENVINS)
    set(SMART_DRONE_REPLAY_BACKEND_AVAILABILITY_SOURCE
      ../src/native/core/application/config/slam_backend_availability_openvins.cpp)
  endif()

  add_executable(smart_drone_offline_replay
    euroc/offline_replay_main.cpp
    euroc/support/replay_dataset.cpp
    euroc/support/replay_imu_sample_source.cpp
    euroc/support/replay_slam_runner.cpp
    ../src/native/adapters/imu/icm42688_imu_provider.cpp
    ../src/native/common/epg/epg_context.cpp
    ../src/native/common/epg/epg.cpp
    ../src/native/common/epg/epg_dot.cpp
    ../src/native/common/epg/epg_json.cpp
    ../src/native/common/epg/epg_task_runner.cpp
    ../src/native/common/epg/epg_trigger_modes.cpp
    ../src/native/common/time_utils.cpp
    ../src/native/adapters/slam/dpvo/dpvo_tensorrt_engine_config.cpp
    ../src/native/adapters/slam/dpvo/dpvo_runtime_options.cpp
    ../src/native/adapters/slam/engine/slam_engine_adapter.cpp
    ../src/native/adapters/slam/engine/slam_engine_factory.cpp
    ../src/native/adapters/slam/engine/slam_env.cpp
    ../src/native/adapters/slam/engine/slam_mode_state.cpp
    ../src/native/adapters/slam/engine/slam_mode_strategy.cpp
    ../src/native/adapters/slam/engine/slam_tracking_backend.cpp
    ../src/native/adapters/slam/openvins/openvins_runtime.cpp
    ../src/native/adapters/slam/openvins/openvins_runtime_stub.cpp
    ../src/native/adapters/slam/klt/klt_slam_engine.cpp
    ../src/native/adapters/slam/klt/klt_continuous_frontend.cpp
    ../src/native/adapters/slam/klt/klt_loop_closure_backend.cpp
    ../src/native/adapters/slam/klt/klt_mode_utils.cpp
    ../src/native/adapters/slam/klt/klt_per_frame_frontend.cpp
    ../src/native/adapters/slam/klt/klt_pnp_observation_builder.cpp
    ../src/native/adapters/slam/klt/klt_pose_estimator.cpp
    ../src/native/adapters/slam/klt/klt_track_manager.cpp
    ../src/native/adapters/slam/stereo/feature_point_tracking.cpp
    ../src/native/adapters/slam/stereo/stereo_feature_frontend_runner.cpp
    ../src/native/adapters/slam/stereo/descriptor_geometry.cpp
    ../src/native/adapters/slam/stereo/stereo_matching.cpp
    ../src/native/adapters/slam/stereo/stereo_pair_builder.cpp
    ../src/native/adapters/slam/stereo/stereo_feature_packet.cpp
    ../src/native/adapters/slam/stereo/stereo_feature_packet_hash.cpp
    ../src/native/adapters/slam/stereo/temporal_stereo.cpp
    ../src/native/adapters/slam/engine/slam_image_utils.cpp
    ../src/native/adapters/slam/engine/slam_output_utils.cpp
    ../src/native/adapters/slam/engine/slam_output_optional.cpp
    ../src/native/adapters/slam/engine/slam_pose_utils.cpp
    ../src/native/adapters/slam/stereo/stereo_calibration.cpp
    ../src/native/adapters/slam/stereo/stereo_frame_preprocessor.cpp
    ../src/native/adapters/slam/stereo/stereo_geometry.cpp
    ../src/native/adapters/slam/stereo/visual_pnp_pose_backend.cpp
    ../src/native/adapters/slam/superpoint/visual_feature_frontend_client.cpp
    ../src/native/adapters/slam/superpoint/superpoint_runtime_options.cpp
    ../src/native/adapters/slam/superpoint/superpoint_lightglue_frontend_client.cpp
    ../src/native/adapters/slam/superpoint/visual_feature_lightglue_mode_strategy.cpp
    ../src/native/adapters/slam/orb/orb_mode_strategy.cpp
    ../src/native/common/environment.cpp
    ../src/native/common/logger.cpp
    ../src/native/common/numeric_parse.cpp
    ../src/native/core/ports/slam_tracking_state.cpp
    ../src/native/core/ports/visual_tracking.cpp
    ../src/native/core/application/config/app_args.cpp
    ../src/native/core/application/config/orb_acceleration_config.cpp
    ${SMART_DRONE_REPLAY_BACKEND_AVAILABILITY_SOURCE}
    ../src/native/core/application/epg/epg_discrete_global_solver.cpp
    ../src/native/core/application/epg/epg_graph_runtime.cpp
    ../src/native/core/application/epg/epg_cpu_binding_solver_exact.cpp
    ../src/native/core/application/epg/epg_cpu_binding_solver_heuristic.cpp
    ../src/native/core/application/epg/epg_cpu_binding_solver_support.cpp
    ../src/native/core/application/epg/epg_cpu_binding_solver.cpp
    ../src/native/core/application/epg/epg_registry.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_apply.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_candidates.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_cpu_binding.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_entry.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_io.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_plan.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_queue_candidates.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_report.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_report_decisions.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_task_apply.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_topology.cpp
    ../src/native/core/application/epg/epg_runtime_optimizer_topology_solver.cpp
    ../src/native/core/application/epg/epg_solver_primitives.cpp
    ../src/native/core/application/epg/epg_solver_report_score.cpp
    ../src/native/core/application/epg/epg_solver_topology_primitives.cpp
    ../src/native/core/application/epg/epg_task_decision_primitives.cpp
    ../src/native/core/application/epg/epg_task_evaluation_primitives.cpp
    ../src/native/core/application/epg/epg_task_dependency_graph.cpp
    ../src/native/core/application/epg/epg_task_manifest.cpp
    ../src/native/core/application/epg/epg_task_manifest_catalog.cpp
    ../src/native/core/application/epg/epg_task_manifest_internal.cpp
    ../src/native/core/application/epg/epg_task_manifest_optimized_graph_validation.cpp
    ../src/native/core/application/epg/epg_task_manifest_registry.cpp
    ../src/native/core/application/epg/epg_task_manifest_runtime_tuning.cpp
    ../src/native/core/application/epg/epg_task_manifest_validation.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_decisions.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_global_optimum.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_global_optimum_support.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_metadata.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_queue_optimum.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_schedule.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_task_decisions.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_task_optimum.cpp
    ../src/native/core/application/epg/epg_task_manifest_solver_report_task_penalty.cpp
    ../src/native/core/application/runtime/epg_dfx_snapshot.cpp
    ../src/native/core/application/runtime/epg_graph_lifecycle.cpp
    ../src/native/core/application/runtime/runtime_aliases.cpp
    ../src/native/core/application/runtime/system_runtime_messages.cpp
    ../src/native/core/application/sensors/imu_sensor_poller.cpp
    ../src/native/core/application/session/epg/messages/slam_epg_messages.cpp
    ../src/native/core/application/session/epg/slam_session_graph_service.cpp
    ../src/native/core/application/session/epg/slam_session_task_factory.cpp
    ../src/native/core/application/session/epg/slam_session_task_utils.cpp
    ../src/native/core/application/session/epg/slam_session_tasks.cpp
    ../src/native/core/application/session/slam/imu_window_filter.cpp
    ../src/native/core/application/session/slam/slam_backend_maintenance_port.cpp
    ../src/native/core/application/session/slam/slam_frame_input_port.cpp
    ../src/native/core/application/session/slam/slam_frame_output_port.cpp
    ../src/native/core/application/session/slam/slam_frame_port_set.cpp
    ../src/native/core/application/session/slam/slam_frame_pose_postprocess_port.cpp
    ../src/native/core/application/session/slam/slam_frame_tracking_port.cpp
    ../src/native/core/application/session/slam/slam_processing_support.cpp
    ../src/native/core/application/session/slam/slam_runtime_control_port.cpp
    ../src/native/core/application/session/slam/slam_runtime_environment.cpp
    ../src/native/core/application/session/slam/slam_session_processing_port.cpp
    ../src/native/core/application/session/slam/slam_session_resource_lifecycle.cpp
    ../src/native/core/application/session/slam/slam_session_runtime.cpp
    ../src/native/core/application/session/slam/slam_session_runtime_service.cpp
    ../src/native/core/application/session/slam/slam_settings_loader.cpp
    ../src/native/core/application/session/stream/preview_output_port.cpp
    ../src/native/core/application/state/frame_timing_tracker.cpp
    ../src/native/core/application/state/imu_buffer.cpp
    ../src/native/core/application/state/live_pose_state.cpp
    ../src/native/core/application/state/perception_pipeline.cpp
    ../src/native/core/application/state/pose_postprocessor.cpp
    ../src/native/core/domain/runtime_mode.cpp
  )

  target_include_directories(smart_drone_offline_replay PRIVATE
    "${EUROC_TEST_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../src/native"
    "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/Sophus"
  )

  if(SMART_DRONE_ENABLE_ORB_SLAM3)
    target_sources(smart_drone_offline_replay PRIVATE
      ../src/native/adapters/slam/engine/slam_engine_adapter.cpp
      ../src/native/adapters/slam/engine/slam_mode_strategy.cpp
      ../src/native/adapters/slam/engine/slam_tracking_backend.cpp
      ../src/native/adapters/slam/orb/orb_mode_strategy.cpp
      ../src/native/adapters/slam/superpoint/visual_feature_lightglue_mode_strategy.cpp
      ../src/native/adapters/slam/engine/slam_output_optional_orb.cpp
      ../src/native/adapters/slam/orb/orb_feature_utils.cpp
      ../src/native/adapters/slam/orb/orb_visual_components.cpp
      ../src/native/adapters/slam/orb/orb_slam3_runtime.cpp
      ../src/native/adapters/slam/orb/orb_slam3_backend.cpp
    )
    set_source_files_properties(
      ../src/native/adapters/slam/engine/slam_output_optional.cpp
      PROPERTIES HEADER_FILE_ONLY ON)
    configure_orb_target(smart_drone_offline_replay)
  endif()

  if(SMART_DRONE_ENABLE_OPENVINS)
    set_source_files_properties(
      ../src/native/adapters/slam/openvins/openvins_runtime_stub.cpp
      PROPERTIES HEADER_FILE_ONLY ON)
    target_sources(smart_drone_offline_replay PRIVATE
      ../src/native/adapters/slam/openvins/openvins_runtime_openvins.cpp
      ../src/native/adapters/slam/openvins/openvins_backend.cpp
      ../src/native/adapters/slam/openvins/openvins_slam_engine.cpp
      ${SMART_DRONE_OPENVINS_CORE_SOURCES}
      ${SMART_DRONE_OPENVINS_INIT_SOURCES}
      ${SMART_DRONE_OPENVINS_MSCKF_SOURCES}
    )
    if(SMART_DRONE_OPENVINS_CERES_AVAILABLE)
      target_sources(smart_drone_offline_replay PRIVATE
        ${SMART_DRONE_OPENVINS_INIT_CERES_SOURCES}
        ../src/native/adapters/slam/openvins/openvins_dynamic_init_enabled.cpp
      )
    else()
      target_sources(smart_drone_offline_replay PRIVATE
        ${SMART_DRONE_OPENVINS_INIT_DYNAMIC_SOURCE}
        ../src/native/adapters/slam/openvins/openvins_dynamic_init_disabled.cpp
      )
    endif()
    target_include_directories(smart_drone_offline_replay PRIVATE
      "${SMART_DRONE_OPENVINS_ROOT}/ov_core/src"
      "${SMART_DRONE_OPENVINS_ROOT}/ov_init/src"
      "${SMART_DRONE_OPENVINS_ROOT}/ov_msckf/src"
    )
    if(SMART_DRONE_OPENVINS_CERES_AVAILABLE)
      target_include_directories(smart_drone_offline_replay PRIVATE
        ${CERES_INCLUDE_DIRS})
    endif()
    target_link_libraries(smart_drone_offline_replay PRIVATE ${Boost_LIBRARIES})
    if(SMART_DRONE_OPENVINS_CERES_AVAILABLE AND NOT CROSS_AARCH64)
      target_link_libraries(smart_drone_offline_replay PRIVATE ${CERES_LIBRARIES})
    endif()
    link_smart_drone_openvins_jetson_deps(smart_drone_offline_replay)
  endif()

  if(TARGET Eigen3::Eigen)
    target_link_libraries(smart_drone_offline_replay PRIVATE Eigen3::Eigen)
  else()
    target_include_directories(smart_drone_offline_replay SYSTEM PRIVATE "/usr/include/eigen3")
  endif()

  set(SMART_DRONE_REPLAY_VPI_SOURCE
    ../src/native/adapters/slam/klt/klt_vpi_acceleration_stub.cpp)
  set(SMART_DRONE_REPLAY_DPVO_SOURCE
    ../src/native/adapters/slam/dpvo/dpvo_tensorrt_engine.cpp)
  set(SMART_DRONE_REPLAY_SUPERPOINT_SOURCE
    ../src/native/adapters/slam/superpoint/superpoint_native_extractor.cpp)

  if(SMART_DRONE_ENABLE_VPI AND NOT SMART_DRONE_ENABLE_OPENVINS)
    resolve_smart_drone_vpi()
    if(SMART_DRONE_VPI_AVAILABLE)
      set(SMART_DRONE_REPLAY_VPI_SOURCE
        ../src/native/adapters/slam/klt/klt_vpi_acceleration.cpp)
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

  if(CROSS_AARCH64 AND NOT SMART_DRONE_ENABLE_OPENVINS)
    configure_superpoint_native_target(smart_drone_offline_replay)
    if(SMART_DRONE_TENSORRT_AVAILABLE)
      set(SMART_DRONE_REPLAY_DPVO_SOURCE
        ../src/native/adapters/slam/dpvo/dpvo_tensorrt_engine_trt.cpp
        ../src/native/adapters/slam/dpvo/dpvo_tensorrt_engine_bindings.cpp
        ../src/native/adapters/slam/dpvo/dpvo_tensorrt_engine_patchifier_runtime.cpp
        ../src/native/adapters/slam/dpvo/dpvo_tensorrt_engine_update_preagg_runtime.cpp
        ../src/native/adapters/slam/dpvo/dpvo_tensorrt_engine_update_postagg_runtime.cpp
        ../src/native/adapters/slam/dpvo/dpvo_tensorrt_engine_cuda_buffers.cpp
        ../src/native/adapters/slam/dpvo/dpvo_tensorrt_engine_cuda_runtime.cpp
        ../src/native/adapters/slam/dpvo/dpvo_tensorrt_engine_cuda_compute.cpp
        ../src/native/adapters/slam/dpvo/dpvo_tensorrt_engine_graph_state.cpp
        ../src/native/adapters/slam/dpvo/dpvo_tensorrt_engine_graph_state_internal.cpp
        ../src/native/adapters/slam/dpvo/dpvo_tensorrt_engine_native_solver_math.cpp
        ../src/native/adapters/slam/dpvo/dpvo_tensorrt_engine_native_solver_bundle_adjustment.cpp
        ../src/native/adapters/slam/dpvo/dpvo_tensorrt_engine_native_solver_step.cpp)
      set(SMART_DRONE_REPLAY_SUPERPOINT_SOURCE
        ../src/native/adapters/slam/superpoint/superpoint_native_extractor_tensorrt_common.cpp
        ../src/native/adapters/slam/superpoint/superpoint_native_extractor_postprocess_heatmap.cpp
        ../src/native/adapters/slam/superpoint/superpoint_native_extractor_postprocess_descriptors.cpp
        ../src/native/adapters/slam/superpoint/superpoint_native_extractor_tensorrt_output.cpp
        ../src/native/adapters/slam/superpoint/superpoint_native_extractor_tensorrt_utils.cpp
        ../src/native/adapters/slam/superpoint/superpoint_native_extractor_tensorrt_superpoint.cpp
        ../src/native/adapters/slam/superpoint/superpoint_native_extractor_tensorrt_lightglue.cpp
        ../src/native/adapters/slam/superpoint/superpoint_native_extractor_impl_runtime.cpp
        ../src/native/adapters/slam/superpoint/superpoint_native_extractor_impl_batch.cpp
        ../src/native/adapters/slam/superpoint/superpoint_native_extractor_trt.cpp)
    endif()
  endif()

  if(SMART_DRONE_ENABLE_OPENVINS)
    set(SMART_DRONE_REPLAY_VPI_SOURCE
      ../src/native/adapters/slam/klt/klt_vpi_acceleration_stub.cpp)
    set(SMART_DRONE_REPLAY_DPVO_SOURCE
      ../src/native/adapters/slam/dpvo/dpvo_tensorrt_engine.cpp)
    set(SMART_DRONE_REPLAY_SUPERPOINT_SOURCE
      ../src/native/adapters/slam/superpoint/superpoint_native_extractor.cpp)
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
