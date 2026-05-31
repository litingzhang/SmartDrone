set(SMART_DRONE_NATIVE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")

set(SMART_DRONE_MAIN_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/app/bootstrap/runtime_host.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/app/bootstrap/signal_handlers.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/main.cpp"
)

add_library(epg STATIC
  "${SMART_DRONE_NATIVE_ROOT}/common/epg/epg_context.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/epg/epg.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/epg/epg_dot.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/epg/epg_json.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/epg/epg_mermaid.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/epg/epg_task_runner.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/epg/epg_trigger_modes.cpp"
)

target_include_directories(epg PUBLIC
  "${SMART_DRONE_NATIVE_ROOT}"
)

target_link_libraries(epg PUBLIC
  Threads::Threads
)

set(SMART_DRONE_CAMERA_SOURCES)
if(SMART_DRONE_CAMERA_PROVIDER STREQUAL "libcamera_stereo_ov9281")
  list(APPEND SMART_DRONE_CAMERA_SOURCES
    "${SMART_DRONE_NATIVE_ROOT}/adapters/camera/libcamera_ov9281/stereo_ov9281.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/camera/libcamera_ov9281/stereo_ov9281_pairing.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/camera/libcamera_stereo_camera.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/camera/camera_provider_factory_libcamera.cpp"
  )
elseif(SMART_DRONE_CAMERA_PROVIDER STREQUAL "uvc_stereo_opencv")
  list(APPEND SMART_DRONE_CAMERA_SOURCES
    "${SMART_DRONE_NATIVE_ROOT}/adapters/camera/uvc_stereo_camera.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/camera/camera_provider_factory_uvc.cpp"
  )
else()
  message(FATAL_ERROR
    "Unsupported SMART_DRONE_CAMERA_PROVIDER='${SMART_DRONE_CAMERA_PROVIDER}'. "
    "Currently supported: libcamera_stereo_ov9281, uvc_stereo_opencv")
endif()

set(SMART_DRONE_ADAPTER_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/adapters/command/udp_tlv_command_channel.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/app/composition/default_application_runtime_factories.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/app/composition/imu_sample_source_provider.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/imu/icm42688/icm42688_imu.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/imu/icm42688_imu_provider.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/imu/icm42688_sample_source.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/engine/slam_engine_factory.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/engine/slam_env.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_tensorrt_engine_config.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_tensorrt_engine.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_runtime_options.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/openvins/openvins_runtime_stub.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/openvins/openvins_backend.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/openvins/openvins_slam_engine.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo/stereo_feature_frontend_runner.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo/descriptor_geometry.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo/stereo_pair_builder.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo/stereo_matching.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo/stereo_feature_packet.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo/stereo_feature_packet_hash.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo/temporal_stereo.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo/visual_frame_observation_loader.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo/feature_point_tracking.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo/stereo_calibration.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo/stereo_frame_preprocessor.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo/stereo_geometry.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt/klt_slam_engine.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt/klt_continuous_frontend.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt/klt_loop_closure_backend.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt/klt_mode_utils.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt/klt_per_frame_frontend.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt/klt_pnp_observation_builder.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt/klt_pose_estimator.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt/klt_track_manager.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt/klt_vpi_acceleration_stub.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/engine/slam_image_utils.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo/visual_pnp_pose_backend.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint/visual_feature_frontend_client.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/engine/slam_mode_state.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/engine/slam_output_utils.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/engine/slam_output_optional.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/engine/slam_pose_utils.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint/superpoint_native_extractor.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint/superpoint_runtime_options.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint/superpoint_lightglue_frontend_client.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/stream/udp_image_sender.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/telemetry/mavlink_pose_publisher.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/telemetry/mavlink_serial_transport.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/telemetry/px4_mavlink_gateway.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/telemetry/px4_vehicle_control_port.cpp"
  ${SMART_DRONE_CAMERA_SOURCES}
)

if(SMART_DRONE_TENSORRT_AVAILABLE)
  list(REMOVE_ITEM SMART_DRONE_ADAPTER_SOURCES
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_tensorrt_engine.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint/superpoint_native_extractor.cpp"
  )
  list(APPEND SMART_DRONE_ADAPTER_SOURCES
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_tensorrt_engine_trt.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_tensorrt_engine_bindings.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_tensorrt_engine_patchifier_runtime.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_tensorrt_engine_update_preagg_runtime.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_tensorrt_engine_update_postagg_runtime.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_tensorrt_engine_cuda_buffers.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_tensorrt_engine_cuda_runtime.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_tensorrt_engine_cuda_compute.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_tensorrt_engine_graph_state.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_tensorrt_engine_graph_state_internal.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_tensorrt_engine_native_solver_math.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_tensorrt_engine_native_solver_bundle_adjustment.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo/dpvo_tensorrt_engine_native_solver_step.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint/superpoint_native_extractor_tensorrt_common.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint/superpoint_native_extractor_postprocess_heatmap.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint/superpoint_native_extractor_postprocess_descriptors.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint/superpoint_native_extractor_tensorrt_output.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint/superpoint_native_extractor_tensorrt_utils.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint/superpoint_native_extractor_tensorrt_superpoint.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint/superpoint_native_extractor_tensorrt_lightglue.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint/superpoint_native_extractor_impl_runtime.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint/superpoint_native_extractor_impl_batch.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint/superpoint_native_extractor_trt.cpp"
  )
endif()

if(SMART_DRONE_VPI_AVAILABLE)
  list(REMOVE_ITEM SMART_DRONE_ADAPTER_SOURCES
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt/klt_vpi_acceleration_stub.cpp"
  )
  list(APPEND SMART_DRONE_ADAPTER_SOURCES
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt/klt_vpi_acceleration.cpp"
  )
endif()

set(SMART_DRONE_ORB_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/engine/slam_engine_adapter.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/engine/slam_mode_strategy.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/orb/orb_feature_utils.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/orb/orb_visual_components.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/orb/orb_vocabulary.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/orb/orb_slam3_runtime.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/orb/orb_slam3_backend.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/engine/slam_tracking_backend.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/orb/orb_mode_strategy.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint/visual_feature_lightglue_mode_strategy.cpp"
)

if(SMART_DRONE_ENABLE_ORB_SLAM3)
  list(REMOVE_ITEM SMART_DRONE_ADAPTER_SOURCES
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/engine/slam_output_optional.cpp"
  )
  list(APPEND SMART_DRONE_ADAPTER_SOURCES
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/engine/slam_output_optional_orb.cpp"
  )
  list(APPEND SMART_DRONE_ADAPTER_SOURCES ${SMART_DRONE_ORB_SOURCES})
endif()

if(SMART_DRONE_ENABLE_OPENVINS)
  list(REMOVE_ITEM SMART_DRONE_ADAPTER_SOURCES
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/openvins/openvins_runtime_stub.cpp"
  )
  list(APPEND SMART_DRONE_ADAPTER_SOURCES
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/openvins/openvins_runtime.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/openvins/openvins_runtime_openvins.cpp"
  )
else()
  list(REMOVE_ITEM SMART_DRONE_ADAPTER_SOURCES
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/openvins/openvins_runtime_stub.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/openvins/openvins_backend.cpp"
    "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/openvins/openvins_slam_engine.cpp"
  )
endif()

set(SMART_DRONE_SLAM_BACKEND_AVAILABILITY_SOURCE
  "${SMART_DRONE_NATIVE_ROOT}/core/application/config/slam_backend_availability.cpp"
)
if(SMART_DRONE_ENABLE_ORB_SLAM3 AND SMART_DRONE_ENABLE_OPENVINS)
  set(SMART_DRONE_SLAM_BACKEND_AVAILABILITY_SOURCE
    "${SMART_DRONE_NATIVE_ROOT}/core/application/config/slam_backend_availability_orb_openvins.cpp"
  )
elseif(SMART_DRONE_ENABLE_ORB_SLAM3)
  set(SMART_DRONE_SLAM_BACKEND_AVAILABILITY_SOURCE
    "${SMART_DRONE_NATIVE_ROOT}/core/application/config/slam_backend_availability_orb.cpp"
  )
elseif(SMART_DRONE_ENABLE_OPENVINS)
  set(SMART_DRONE_SLAM_BACKEND_AVAILABILITY_SOURCE
    "${SMART_DRONE_NATIVE_ROOT}/core/application/config/slam_backend_availability_openvins.cpp"
  )
endif()

set(SMART_DRONE_COMMON_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/common/environment.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/logger.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/numeric_parse.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/runtime_state.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/time_utils.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/tlv/crc16_ccitt_false.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/tlv/goal_cache.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/tlv/tlv_cmd_router.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/tlv/tlv_pack.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/tlv/tlv_parser.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/tlv/udp_server.cpp"
)

set(SMART_DRONE_CORE_DOMAIN_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/core/domain/runtime_mode.cpp"
)

set(SMART_DRONE_CORE_PORT_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/core/ports/slam_tracking_state.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/ports/visual_tracking.cpp"
)

set(SMART_DRONE_CORE_APPLICATION_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/core/application/config/app_args.cpp"
  "${SMART_DRONE_SLAM_BACKEND_AVAILABILITY_SOURCE}"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/config/capability_catalog.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/config/config_registry.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/config/orb_acceleration_config.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/avoidance_state_payload.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/discovery_beacon_runtime.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/mode_manager.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/epg_dfx_snapshot.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/epg_optimize_task.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/epg_redeploy_coordinator.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/hover_algorithm_plugin.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/local_occupancy_map.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/live_pose_runtime_snapshots.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/obstacle_avoidance_config.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/obstacle_avoidance_policy.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/payload_builders.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_aliases.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_config_application.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/system_runtime_messages.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_config_frame_codec.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_config_message.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_config_projection.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_config_service.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_config_validation.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_config_value_applier.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_config_value_assignments.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_config_update_builder.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/px4_udp_hooks.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_command_service.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_controller.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_session_supervisor.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/system_runtime_graph_service.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/system_runtime_task_factory.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/system_runtime_step_services.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/system_runtime_tasks.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/udp_command_runtime.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/epg/calib_session_graph_service.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/calib/calib_camera_input_port.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/calib/calib_image_utils.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/calib/calib_imu_sample_port.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/calib/calib_output_store.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/calib/calib_output_sync.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/calib/calib_preview_port.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/calib/calib_runtime_state.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/calib/calib_save_pacing_port.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/calib/calib_session_port_set.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/epg/calib_session_task_factory.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/epg/calib_session_tasks.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/calib/calib_storage_port.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/calib/calib_storage_helpers.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/epg/messages/calib_epg_messages.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/epg/messages/slam_epg_messages.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_discrete_global_solver.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_registry.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_graph_runtime.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_cpu_binding_solver_exact.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_cpu_binding_solver_heuristic.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_cpu_binding_solver_support.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_cpu_binding_solver.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_runtime_optimizer_apply.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_runtime_optimizer_candidates.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_runtime_optimizer.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_runtime_optimizer_cpu_binding.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_runtime_optimizer_entry.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_runtime_optimizer_io.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_runtime_optimizer_plan.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_runtime_optimizer_queue_candidates.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_runtime_optimizer_report.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_runtime_optimizer_report_decisions.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_runtime_optimizer_task_apply.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_runtime_optimizer_topology.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_runtime_optimizer_topology_solver.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_solver_primitives.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_solver_report_score.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_solver_topology_primitives.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_decision_primitives.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_evaluation_primitives.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_dependency_graph.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest_catalog.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest_internal.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest_optimized_graph_validation.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest_registry.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest_runtime_tuning.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest_validation.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest_solver_report.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest_solver_report_decisions.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest_solver_report_global_optimum.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest_solver_report_global_optimum_support.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest_solver_report_metadata.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest_solver_report_queue_optimum.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest_solver_report_schedule.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest_solver_report_task_decisions.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest_solver_report_task_optimum.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest_solver_report_task_penalty.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam/imu_window_filter.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/epg_graph_lifecycle.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/epg/session_graph_runtime_factory.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam/slam_frame_input_port.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam/slam_backend_maintenance_port.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam/slam_frame_output_port.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam/slam_frame_port_set.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam/slam_frame_pose_postprocess_port.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam/slam_frame_tracking_port.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/sensors/imu_sensor_poller.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/stream/preview_output_port.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam/slam_runtime_control_port.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam/slam_runtime_environment.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam/slam_settings_loader.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam/slam_processing_support.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam/slam_session_resource_lifecycle.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam/slam_session_processing_port.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/epg/slam_session_graph_service.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/epg/slam_session_task_factory.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/epg/slam_session_task_utils.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/epg/slam_session_tasks.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam/slam_session_runtime.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/session/slam/slam_session_runtime_service.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/state/frame_timing_tracker.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/state/imu_buffer.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/state/live_pose_state.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/state/perception_pipeline.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/state/pose_postprocessor.cpp"
)

set(SMART_DRONE_PLATFORM_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/platform/linux/gpio/drdy_gpio.cpp"
)

if(SMART_DRONE_GPIOD_V2)
  list(APPEND SMART_DRONE_PLATFORM_SOURCES
    "${SMART_DRONE_NATIVE_ROOT}/platform/linux/gpio/drdy_gpio_v2.cpp"
  )
else()
  list(APPEND SMART_DRONE_PLATFORM_SOURCES
    "${SMART_DRONE_NATIVE_ROOT}/platform/linux/gpio/drdy_gpio_v1.cpp"
  )
endif()

list(APPEND SMART_DRONE_PLATFORM_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/platform/linux/spi/spi_dev.cpp"
)

set(SMART_DRONE_ALL_SOURCES
  ${SMART_DRONE_MAIN_SOURCES}
  ${SMART_DRONE_ADAPTER_SOURCES}
  ${SMART_DRONE_COMMON_SOURCES}
  ${SMART_DRONE_CORE_DOMAIN_SOURCES}
  ${SMART_DRONE_CORE_PORT_SOURCES}
  ${SMART_DRONE_CORE_APPLICATION_SOURCES}
  ${SMART_DRONE_PLATFORM_SOURCES}
)

target_sources(smart_drone PRIVATE
  ${SMART_DRONE_ALL_SOURCES}
)

target_include_directories(smart_drone PRIVATE
  "${SMART_DRONE_NATIVE_ROOT}"
  "${SMART_DRONE_NATIVE_ROOT}/common/tlv"
)

target_link_libraries(smart_drone PRIVATE
  epg
)

source_group(TREE "${SMART_DRONE_NATIVE_ROOT}" FILES
  ${SMART_DRONE_ALL_SOURCES}
)
