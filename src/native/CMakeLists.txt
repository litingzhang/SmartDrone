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
  "${SMART_DRONE_NATIVE_ROOT}/adapters/runtime/default_application_runtime_factories.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/imu/icm42688/icm42688_imu.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/imu/icm42688_imu_provider.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/imu/icm42688_sample_source.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/imu/imu_sample_source_provider.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/slam_engine_factory.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo_tensorrt_engine.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/dpvo_runtime_options.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo_feature_frontend_runner.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/descriptor_geometry.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo_pair_builder.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo_matching.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo_feature_packet.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/temporal_stereo.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/visual_frame_observation_loader.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/feature_point_tracking.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo_calibration.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo_frame_preprocessor.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/stereo_geometry.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt_slam_engine.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt_continuous_frontend.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt_loop_closure_backend.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt_mode_utils.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt_per_frame_frontend.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt_pnp_observation_builder.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt_pose_estimator.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt_track_manager.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/klt_vpi_acceleration.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/slam_image_utils.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/visual_pnp_pose_backend.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/visual_feature_frontend_client.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/slam_mode_state.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/slam_output_utils.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/slam_pose_utils.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint_native_extractor.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint_runtime_options.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/superpoint_lightglue_frontend_client.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/stream/udp_image_sender.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/telemetry/mavlink_pose_publisher.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/telemetry/mavlink_serial_transport.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/telemetry/px4_mavlink_gateway.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/telemetry/px4_vehicle_control_port.cpp"
  ${SMART_DRONE_CAMERA_SOURCES}
)

set(SMART_DRONE_ORB_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/slam_engine_adapter.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/slam_mode_strategy.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/orb_feature_utils.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/orb_visual_components.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/orb_vocabulary.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/orb_slam3_runtime.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/orb_slam3_backend.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/slam_tracking_backend.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/orb_mode_strategy.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/adapters/slam/visual_feature_lightglue_mode_strategy.cpp"
)

if(SMART_DRONE_ENABLE_ORB_SLAM3)
  list(APPEND SMART_DRONE_ADAPTER_SOURCES ${SMART_DRONE_ORB_SOURCES})
endif()

set(SMART_DRONE_COMMON_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/common/runtime_state.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/time_utils.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/tlv/crc16_ccitt_false.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/tlv/tlv_cmd_router.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/tlv/tlv_pack.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/common/tlv/udp_server.cpp"
)

set(SMART_DRONE_CORE_DOMAIN_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/core/domain/runtime_mode.cpp"
)

set(SMART_DRONE_CORE_APPLICATION_SOURCES
  "${SMART_DRONE_NATIVE_ROOT}/core/application/config/app_args.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/config/capability_catalog.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/config/config_registry.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/discovery_beacon_runtime.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/mode_manager.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/epg_dfx_snapshot.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/epg_optimize_task.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/epg_redeploy_coordinator.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/payload_builders.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_aliases.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/system_runtime_messages.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_config_frame_codec.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/runtime/runtime_config_service.cpp"
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
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_registry.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_runtime_optimizer.cpp"
  "${SMART_DRONE_NATIVE_ROOT}/core/application/epg/epg_task_manifest.cpp"
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
  "${SMART_DRONE_NATIVE_ROOT}/platform/linux/spi/spi_dev.cpp"
)

set(SMART_DRONE_ALL_SOURCES
  ${SMART_DRONE_MAIN_SOURCES}
  ${SMART_DRONE_ADAPTER_SOURCES}
  ${SMART_DRONE_COMMON_SOURCES}
  ${SMART_DRONE_CORE_DOMAIN_SOURCES}
  ${SMART_DRONE_CORE_APPLICATION_SOURCES}
  ${SMART_DRONE_PLATFORM_SOURCES}
)

add_executable(smart_drone)

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
